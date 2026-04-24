#!/usr/bin/env python3
"""
8051 simulator for RTD 8051 MCU firmware tracing.

Implements the full MCS-51 instruction set with proper state tracking:
  - 256-byte IRAM (register banks, bit-addressable area, scratch, stack)
  - Dual DPTR (DPS selects DPTR0 at 0x82/0x83 vs DPTR1 at 0x84/0x85)
  - XFR space (0xFD00-0xFFFF) via handler-based policy table
  - XDATA 0x0000-0xFEFF = scaler regs when FFFC bit4 (scalar_addr_remapping) set

In local mode, all XFR registers are faked. In bridge (proxy) mode,
XFR reads and writes go to real hardware with per-register safety masks
to keep watchdog, DDC, ISP, and reset activity from disrupting the debug
session.
"""

import sys
import time

_XFR_BASE = 0xFD00
_XRAM_BASE = 0xD700
_XRAM_END = 0xF6FF

# =============================================================================
# XFR policy — each register maps to a (read_fn, write_fn) pair.
#
#   read_fn(sim, addr) -> byte
#   write_fn(sim, addr, val) -> None
#
# Built with fake()/proxy()/handler() helpers inside _build_xfr_policy().
# =============================================================================

def _xfr_fake_read(r_set, r_clr):
    if r_set == 0 and r_clr == 0:
        return lambda sim, addr: sim.xfr_get(addr)
    return lambda sim, addr: (sim.xfr_get(addr) | r_set) & ~r_clr

def _xfr_fake_write(w_set, w_clr):
    if w_set == 0 and w_clr == 0:
        return lambda sim, addr, val: sim.xfr_set(addr, val & 0xFF)
    return lambda sim, addr, val: sim.xfr_set(addr, (val | w_set) & ~w_clr & 0xFF)

def _xfr_proxy_read(r_set, r_clr):
    if r_set == 0 and r_clr == 0:
        return lambda sim, addr: sim.bridge.read((addr >> 8) & 0xFF, addr & 0xFF)
    return lambda sim, addr: (sim.bridge.read((addr >> 8) & 0xFF, addr & 0xFF) | r_set) & ~r_clr

def _xfr_proxy_write(w_set, w_clr):
    if w_set == 0 and w_clr == 0:
        return lambda sim, addr, val: sim.bridge.write((addr >> 8) & 0xFF, addr & 0xFF, val & 0xFF)
    return lambda sim, addr, val: sim.bridge.write((addr >> 8) & 0xFF, addr & 0xFF, (val | w_set) & ~w_clr & 0xFF)


def _build_xfr_policy(sim):
    """Build XFR policy table {0xFD00-0xFFFF -> (read_fn, write_fn)}.

    Every address gets an explicit entry. Local mode uses fake handlers
    for the entire range.
    """
    p = {}
    has_bridge = sim.bridge is not None

    def fake(*addrs, w_set=0, w_clr=0, r_set=0, r_clr=0):
        rf, wf = _xfr_fake_read(r_set, r_clr), _xfr_fake_write(w_set, w_clr)
        for a in addrs:
            p[a] = (rf, wf)

    def proxy(*addrs, w_set=0, w_clr=0, r_set=0, r_clr=0):
        rf, wf = _xfr_proxy_read(r_set, r_clr), _xfr_proxy_write(w_set, w_clr)
        for a in addrs:
            p[a] = (rf, wf)

    def handler(addr, read_fn=None, write_fn=None):
        rf = read_fn or _xfr_fake_read(0, 0)
        wf = write_fn or _xfr_fake_write(0, 0)
        p[addr] = (rf, wf)

    # --- Default: everything fake or proxy ---
    for a in range(_XFR_BASE, 0x10000):
        if not has_bridge:
            fake(a)
        else:
            proxy(a)

    # --- SCA_INF (0xFFF3-0xFFF5) ---
    def _sca_ctrl_write(sim, a, v):
        v &= 0xFF
        sim._sca_inf_ctrl = v
        # Burst triggers: bit 4 = reg_burcmd_wr, bit 3 = reg_burdat_wr.
        # Source select: FFFC bit 5 (0=flash, 1=XRAM). Address in FFF6-FFF8,
        # count in FFF9-FFFA. HW halts MCU and auto-clears the trigger bit.
        if v & 0x18:
            sim._sca_inf_burst(v)
            sim._sca_inf_ctrl &= ~0x18 & 0xFF
    handler(0xFFF3, lambda sim, a: sim._sca_inf_ctrl, _sca_ctrl_write)
    handler(0xFFF4,
            lambda sim, a: sim._sca_inf_addr,
            lambda sim, a, v: setattr(sim, '_sca_inf_addr', v & 0xFF))

    def _sca_data_read(sim, a):
        v = sim._scaler_read(sim._sca_page, sim._sca_inf_addr)
        if not (sim._sca_inf_ctrl & 0x20):
            sim._sca_inf_addr = (sim._sca_inf_addr + 1) & 0xFF
        return v
    def _sca_data_write(sim, a, v):
        v &= 0xFF
        sim._scaler_write(sim._sca_page, sim._sca_inf_addr, v)
        if not (sim._sca_inf_ctrl & 0x20):
            sim._sca_inf_addr = (sim._sca_inf_addr + 1) & 0xFF
    handler(0xFFF5, _sca_data_read, _sca_data_write)

    # Burst write address/count/period: plain fake registers, read at trigger.
    # FFFC has its own handler below; FFFD-FFFE are bank-switch registers.
    for a in range(0xFFF6, 0xFFFC):
        fake(a)
    for a in (0xFFFD, 0xFFFE):
        fake(a)

    # --- Scaler remap enable (0xFFFC bit 4): cache for _xdata fast path ---
    def _scaler_remap_write(sim, a, v):
        sim.xfr_set(a, v & 0xFF)
        sim._scaler_remap_en = bool(v & 0x10)
    handler(0xFFFC, write_fn=_scaler_remap_write)

    # --- PBANK_SEL (0xFFFF) ---
    handler(0xFFFF,
            lambda sim, a: sim.bank & 0xFF,
            lambda sim, a, v: setattr(sim, 'bank', v & 0x0F))

    # FFEA: clr_wdt (bit 6) resets the simulated WDT counter (always, both modes)
    def _wdt_ctrl_write(sim, addr, val):
        val &= 0xFF
        if val & 0x40:
            sim._wdt_counter_base = sim.steps
        sim.xfr_set(addr, val & ~0xC0 & 0xFF)  # wdt_en and clr_wdt not stored
    handler(0xFFEA, write_fn=_wdt_ctrl_write)

    # WDT latched counter chain: cnt1 (3b) -> cnt2 (11b) -> cnt3 (12b)
    # cnt3 is the visible "elapsed time"; cnt1 is the source-clock prescaler.
    # All derived from instruction count.
    def _wdt_counter(sim):
        # cnt3 value: clamped to 12 bits
        return min((sim.steps - sim._wdt_counter_base) * sim._WDT_TICKS_PER_INSN, 0xFFF)
    def _wdt_cnt1(sim):
        # 3-bit prescaler — rolls every WDT clock tick (faster than cnt3).
        # XOR steps in so the low bits actually vary regardless of multiplier.
        return (sim.steps * sim._WDT_TICKS_PER_INSN ^ sim.steps) & 0x07
    def _wdt_cnt2_low(sim):
        # cnt2 low byte — runs faster than cnt3 by the cnt3-prescaler factor
        return ((sim.steps - sim._wdt_counter_base) * sim._WDT_TICKS_PER_INSN * 32) & 0xFF
    handler(0xFFAA,
            lambda sim, a: _wdt_cnt2_low(sim))               # WATCHDOG_CNT2_LOW
    handler(0xFFAB,
            lambda sim, a: ((_wdt_counter(sim) << 3) & 0xF8) # cnt3[4:0] in bits [7:3]
                         | (((_wdt_counter(sim) * 32) >> 8) & 0x07))  # cnt2[10:8] in bits [2:0]
    handler(0xFFAC,
            lambda sim, a: (_wdt_counter(sim) >> 5) & 0x7F)  # cnt3[11:5]
    handler(0xFFAD,
            lambda sim, a: _wdt_cnt1(sim))                   # WATCHDOG_CNT1_VALUE (cnt1[2:0])

    if not has_bridge:
        return p

    # Watchdog: proxy reads, force-clear enable bits on write (bridge mode only)
    proxy(0xFFEB, w_clr=0x04)            # enable_wdt_irq
    proxy(0xFFE9, w_clr=0x80)            # wdt_en_3
    proxy(0xFF3A, w_clr=0x40)            # wdt_en_2

    # MCU reset: force-clear reset triggers
    proxy(0xFFEE, w_clr=0x03)            # sof_rst, sca_hrst

    # Cache controller
    for a in (0xFDA0, 0xFDA1):
        fake(a)

    # MCU interrupts: write-through, read stale. _sync_hw_interrupts refreshes
    # the mirror between writes (polled path).
    # FF00: plain write-through; mirror stays stale until next sync poll.
    def _write_through(sim, addr, val):
        pg, reg = (addr >> 8) & 0xFF, addr & 0xFF
        sim.bridge.write(pg, reg, val & 0xFF)
        sim.xfr_set(addr, val & 0xFF)

    # FFB8/FFB9: bit 3 is W1C interrupt flag. Write-through + local W1C.
    def _write_and_w1c_bit3(sim, addr, val):
        pg, reg = (addr >> 8) & 0xFF, addr & 0xFF
        sim.bridge.write(pg, reg, val & 0xFF)
        sim.xfr_set(addr, sim.xfr_get(addr) & ~(val & 0x08) & 0xFF)

    handler(0xFF00, write_fn=_write_through)
    for a in (0xFFB8, 0xFFB9):
        handler(a, write_fn=_write_and_w1c_bit3)
    fake(0xFFC4)  # priority reg — stub owns ddc_irq_pri anyway

    # I2C
    for a in range(0xFF22, 0xFF2C):
        fake(a)

    # DDC
    for a in (0xFE6F, 0xFE86, 0xFE87, 0xFF35):
        fake(a)

    return p


class _LoggingBridge:
    """Wraps a DebugBridge to print read/write to stderr.
    If verbose=True, logs everything; else only logs entries in _WATCH_XXX."""

    _WATCH_SCALER = {}
    _WATCH_SFR = {}

    def __init__(self, inner, verbose=True):
        self._inner = inner
        self._verbose = verbose

    def read(self, page, reg):
        v = self._inner.read(page, reg)
        if self._verbose or (page, reg) in self._WATCH_SCALER:
            print(f"  [bridge] R {page:02X}:{reg:02X} = {v:02X}", file=sys.stderr)
        return v

    def write(self, page, reg, val):
        if self._verbose or (page, reg) in self._WATCH_SCALER:
            print(f"  [bridge] W {page:02X}:{reg:02X} = {val:02X}", file=sys.stderr)
        self._inner.write(page, reg, val)

    def sfr_read(self, addr):
        v = self._inner.sfr_read(addr)
        if self._verbose or addr in self._WATCH_SFR:
            print(f"  [bridge] R SFR {addr:02X} = {v:02X}", file=sys.stderr)
        return v

    def sfr_write(self, addr, val):
        if self._verbose or addr in self._WATCH_SFR:
            print(f"  [bridge] W SFR {addr:02X} = {val:02X}", file=sys.stderr)
        self._inner.sfr_write(addr, val)

    def __getattr__(self, name):
        # Pass through anything else unchanged
        return getattr(self._inner, name)


class Sim8051:
    def __init__(self, firmware: bytes, bank: int = 0, bridge=None, bridge_log=False):
        self.fw = bytearray(firmware)
        self.bank = bank
        self.bridge = _LoggingBridge(bridge, verbose=bridge_log) if bridge is not None else None

        # --- Core registers ---
        self.iram = bytearray(256)      # Internal RAM (registers, stack, scratch)
        self.sfr = bytearray(128)       # SFR space 0x80-0xFF (indexed 0x00-0x7F)
        self.pc = 0
        self.logging = False            # set True to record trace_log
        self.trace_log = []

        # --- Shadowed hot registers (plain attributes, synced in _sfr_get/_sfr_set) ---
        self.a = 0                      # ACC (SFR 0xE0)
        self.b = 0                      # B   (SFR 0xF0)
        self.sp = 0x07                  # SP  (SFR 0x81)
        self.cy = 0                     # PSW.7  carry
        self.ac = 0                     # PSW.6  aux carry
        self.ov = 0                     # PSW.2  overflow
        self._rs = 0                    # PSW[4:3] register bank select
        self._rb = 0                    # register bank base (_rs << 3), cached
        self._dps = 0                   # DPS (SFR 0x86) bit 0 selects DPTR0/1
        self.dptr = 0                   # active DPTR (16-bit)
        self._dptr_alt = 0              # inactive DPTR

        # Bank switching is via XDATA 0xFFFF (PBANK_SEL).

        # SFR reset values (use sfr[] directly for non-shadowed regs)
        self.sfr[0x81 - 0x80] = 0x07   # SP
        self.sfr[0x87 - 0x80] = 0x30   # PCON
        self.sfr[0x90 - 0x80] = 0xFF   # P1
        self.sfr[0xB0 - 0x80] = 0xFF   # P3

        # --- XRAM: MCU variables in scaler-remapped space (always local) ---
        self.xram = bytearray(_XRAM_END - _XRAM_BASE + 1)  # 0xD700-0xF6FF

        self._isr_level = 0             # 0=none, 1=low pri active, 2=high pri active
        self._isr_stack = []            # saved level on each ISR entry (for RETI restore)

        # --- XFR state (handlers in policy table reference these) ---
        self.xfr = bytearray(0x10000 - _XFR_BASE)  # 0xFD00-0xFFFF
        self._sca_inf_ctrl = 0x00       # FFF3
        self._sca_inf_addr = 0x00       # FFF4
        self._sca_page = 0x00           # last page written via reg P00:0x9F
        self._scaler_remap_en = False   # cache of 0xFFFC bit 4
        self.scaler = bytearray(256 * 256)  # local scaler regs [page<<8|reg]
        self._wdt_counter_base = 0      # step count when WDT counter last cleared

        # Build policy after state exists (handlers capture self)
        self._xfr_policy = _build_xfr_policy(self)

        # --- Execution state ---
        self.steps = 0
        self.halted = False
        self.halt_reason = ""

    # =====================================================================
    # XFR byte access (inline offset from _XFR_BASE)
    # =====================================================================
    def xfr_get(self, addr):
        return self.xfr[addr - _XFR_BASE]

    def xfr_set(self, addr, val):
        self.xfr[addr - _XFR_BASE] = val

    # =====================================================================
    # SFR access (0x80-0xFF direct addressing)
    #
    # Hot registers (A, B, SP, PSW, DPS, DPTR) are shadowed as plain
    # attributes.  _sfr_get/_sfr_set intercept those addresses so
    # firmware direct-addressing still works.
    # =====================================================================

    # Addresses that are shadowed — _sfr_get/_sfr_set handle them specially.
    # Everything else goes straight to self.sfr[].
    # Port SFRs (0x90 P1, 0xB0 P3) are proxied to bridge if present.
    _PORT_SFRS = frozenset((0x90, 0xB0))

    def _build_psw(self):
        """Reconstruct PSW byte from individual flag attributes."""
        return (self.cy << 7) | (self.ac << 6) | (self._rs << 3) | (self.ov << 2)
        # bit 0 (P, parity) intentionally omitted — firmware rarely reads PSW
        # as a whole byte, and parity is expensive to compute every instruction.

    def _set_psw(self, val):
        """Decompose PSW byte into individual flag attributes."""
        self.cy = (val >> 7) & 1
        self.ac = (val >> 6) & 1
        self._rs = (val >> 3) & 3
        self._rb = self._rs << 3
        self.ov = (val >> 2) & 1

    def _sfr_get(self, addr, internal=False):
        # Shadowed registers — return cached attribute
        if addr == 0xE0: return self.a
        if addr == 0xF0: return self.b
        if addr == 0x81: return self.sp
        if addr == 0xD0: return self._build_psw()
        if addr == 0x86: return self._dps
        # DPTR: 0x82/0x83 = DPTR0, 0x84/0x85 = DPTR1
        if addr == 0x82:
            return (self._dptr_alt if self._dps & 1 else self.dptr) & 0xFF
        if addr == 0x83:
            return ((self._dptr_alt if self._dps & 1 else self.dptr) >> 8) & 0xFF
        if addr == 0x84:
            return (self.dptr if self._dps & 1 else self._dptr_alt) & 0xFF
        if addr == 0x85:
            return ((self.dptr if self._dps & 1 else self._dptr_alt) >> 8) & 0xFF
        # Port SFRs — proxy from bridge if present
        if addr in self._PORT_SFRS and self.bridge is not None:
            v = self.bridge.sfr_read(addr)
            if self.logging and not internal:
                self.trace_log.append(('sfr', addr, v, False, self.bank, self.pc))
            return v
        return self.sfr[addr - 0x80]

    def _sfr_set(self, addr, val):
        val &= 0xFF
        # Shadowed registers — update cached attribute AND sfr[]
        if addr == 0xE0:
            self.a = val
            return
        if addr == 0xF0:
            self.b = val
            return
        if addr == 0x81:
            self.sp = val
            return
        if addr == 0xD0:
            self._set_psw(val)
            self.sfr[0xD0 - 0x80] = val
            return
        if addr == 0x86:
            old_dps = self._dps & 1
            self._dps = val
            # If active DPTR selection changed, swap
            if (val & 1) != old_dps:
                self.dptr, self._dptr_alt = self._dptr_alt, self.dptr
            self.sfr[0x86 - 0x80] = val
            return
        # DPTR byte writes
        if addr == 0x82:
            if self._dps & 1:
                self._dptr_alt = (self._dptr_alt & 0xFF00) | val
            else:
                self.dptr = (self.dptr & 0xFF00) | val
            return
        if addr == 0x83:
            if self._dps & 1:
                self._dptr_alt = (self._dptr_alt & 0x00FF) | (val << 8)
            else:
                self.dptr = (self.dptr & 0x00FF) | (val << 8)
            return
        if addr == 0x84:
            if self._dps & 1:
                self.dptr = (self.dptr & 0xFF00) | val
            else:
                self._dptr_alt = (self._dptr_alt & 0xFF00) | val
            return
        if addr == 0x85:
            if self._dps & 1:
                self.dptr = (self.dptr & 0x00FF) | (val << 8)
            else:
                self._dptr_alt = (self._dptr_alt & 0x00FF) | (val << 8)
            return
        # General SFR write
        self.sfr[addr - 0x80] = val
        # Port SFRs — proxy to bridge if present
        if addr in self._PORT_SFRS:
            if self.bridge is not None:
                self.bridge.sfr_write(addr, val)
            if self.logging:
                self.trace_log.append(('sfr', addr, val, True, self.bank, self.pc))

    # =====================================================================
    # Memory access
    # =====================================================================
    def _code_read(self, addr):
        """Read from code space (banked)."""
        offset = self.bank * 0x10000 + (addr & 0xFFFF)
        if offset < len(self.fw):
            return self.fw[offset]
        return 0xFF

    def _fetch(self):
        """Fetch next instruction byte, advance PC."""
        b = self._code_read(self.pc)
        self.pc = (self.pc + 1) & 0xFFFF
        return b

    def _fetch16(self):
        hi = self._fetch()
        lo = self._fetch()
        return (hi << 8) | lo

    def _direct_get(self, addr):
        """Read direct-addressed byte. 0x00-0x7F = IRAM, 0x80-0xFF = SFR."""
        if addr < 0x80:
            return self.iram[addr]
        return self._sfr_get(addr)

    def _direct_set(self, addr, val):
        """Write direct-addressed byte."""
        val &= 0xFF
        if addr < 0x80:
            self.iram[addr] = val
        else:
            self._sfr_set(addr, val)

    def _indirect_get(self, addr):
        """Read indirect-addressed byte (@R0/@R1). Always IRAM 0-255."""
        return self.iram[addr & 0xFF]

    def _indirect_set(self, addr, val):
        """Write indirect-addressed byte."""
        self.iram[addr & 0xFF] = val & 0xFF

    # =====================================================================
    # Scaler register access (backend for both remapped XDATA and SCA_INF)
    # In proxy mode, forwards to the bridge (real scaler hardware).
    # =====================================================================
    def _scaler_read(self, page, reg):
        if page == 0x00 and reg == 0x9F:
            return self._sca_page
        if self.bridge is not None:
            return self.bridge.read(page, reg)
        return self.scaler[page << 8 | reg]

    def _scaler_write(self, page, reg, val):
        val &= 0xFF
        if page == 0x00 and reg == 0x9F:
            self._sca_page = val
            return
        if self.bridge is not None:
            self.bridge.write(page, reg, val)
        else:
            self.scaler[page << 8 | reg] = val

    def _sca_inf_burst(self, ctrl):
        """Execute a scaler-interface burst write.

        Source address in FFF7:FFF8 (16-bit), count in FFF9:FFFA (16-bit).
        Source bank in FFF6, FFFC bit 5 (0=flash, 1=XRAM).
        Destination: current page + SCA_INF_ADDR, ctrl bit 5 addr_non_inc.

        - bit 3 (reg_burdat_wr): stream N bytes to (page, SCA_INF_ADDR).
        - bit 4 (reg_burcmd_wr): command stream.

        In proxy mode we stream bytes individually through the bridge rather
        than letting the real HW handler fire — real HW would fetch from the
        stub's flash, not the firmware image we're simulating.
        """
        offset = self.xfr_get(0xFFF6) * 0x10000
        addr = ((self.xfr_get(0xFFF7) << 8) |
                 self.xfr_get(0xFFF8))
        count = (self.xfr_get(0xFFF9) << 8) | self.xfr_get(0xFFFA)
        from_xram = bool(self.xfr_get(0xFFFC) & 0x20)
        non_inc = bool(ctrl & 0x20)

        def _src(i):
            a = (addr + i) & 0xFFFF
            if from_xram:
                if (_XRAM_BASE <= a <= _XRAM_END):
                    return self.xram[a - _XRAM_BASE]
                else:
                    raise ValueError(f"SCA_INF burst: XRAM source 0x{a:04X} outside valid XRAM ranges (D700-DFFF, EC00-F6FF)")
            return self.fw[offset + a] if (offset + a) < len(self.fw) else 0xFF

        if ctrl & 0x08:                     # burdat: stream to HOST_ADDR
            reg = self._sca_inf_addr
            for i in range(count):
                self._scaler_write(self._sca_page, reg, _src(i))
                if not non_inc:
                    reg = (reg + 1) & 0xFF
            self._sca_inf_addr = reg
        elif ctrl & 0x10:                   # burcmd: [len][ni][reg][data]...
            i = 0
            while i < count:
                pkt_len = _src(i)
                if pkt_len < 3 or i + pkt_len > count:
                    break
                pkt_ni = _src(i + 1)
                reg = _src(i + 2)
                for j in range(pkt_len - 3):
                    v = _src(i + 3 + j)
                    self._scaler_write(self._sca_page, reg, v)
                    if not pkt_ni:
                        reg = (reg + 1) & 0xFF
                i += pkt_len

    # =====================================================================
    # XFR access (XDATA 0xFD00-0xFFFF) — pure policy dispatch
    # =====================================================================
    def _xfr_get(self, addr):
        return self._xfr_policy[addr][0](self, addr)

    def _xfr_set(self, addr, val):
        self._xfr_policy[addr][1](self, addr, val & 0xFF)

    # =====================================================================
    # XDATA access
    #   0xFD00-0xFFFF : XFR — policy dispatch
    #   0x0000-0xFEFF : scaler regs (bridge or local)
    # =====================================================================
    def _xdata_get(self, addr):
        addr &= 0xFFFF
        if addr >= _XFR_BASE:
            v = self._xfr_get(addr)
        elif (_XRAM_BASE <= addr <= _XRAM_END):
            v = self.xram[addr - _XRAM_BASE]
        elif self._scaler_remap_en:
            v = self._scaler_read((addr >> 8) & 0xFF, addr & 0xFF)
        else:
            v = 0
        if self.logging:
            self.trace_log.append(('xdata', addr, v, False, self.bank, self.pc))
        return v

    def _xdata_set(self, addr, val):
        addr &= 0xFFFF
        val &= 0xFF
        if self.logging:
            self.trace_log.append(('xdata', addr, val, True, self.bank, self.pc))
        if addr >= _XFR_BASE:
            self._xfr_set(addr, val)
        elif (_XRAM_BASE <= addr <= _XRAM_END):
            self.xram[addr - _XRAM_BASE] = val
        elif self._scaler_remap_en:
            self._scaler_write((addr >> 8) & 0xFF, addr & 0xFF, val)

    def _bit_get(self, bit_addr):
        """Read bit-addressable location.
        0x00-0x7F: IRAM 0x20-0x2F (byte = 0x20 + bit_addr//8, bit = bit_addr%8)
        0x80-0xFF: SFR (byte = bit_addr & 0xF8, bit = bit_addr & 0x07)
        """
        if bit_addr < 0x80:
            byte_addr = 0x20 + (bit_addr >> 3)
            bit_pos = bit_addr & 7
            return (self.iram[byte_addr] >> bit_pos) & 1
        else:
            byte_addr = bit_addr & 0xF8
            bit_pos = bit_addr & 7
            return (self._sfr_get(byte_addr) >> bit_pos) & 1

    def _bit_set(self, bit_addr, val):
        """Write bit-addressable location."""
        if bit_addr < 0x80:
            byte_addr = 0x20 + (bit_addr >> 3)
            bit_pos = bit_addr & 7
            if val:
                self.iram[byte_addr] |= (1 << bit_pos)
            else:
                self.iram[byte_addr] &= ~(1 << bit_pos)
        else:
            byte_addr = bit_addr & 0xF8
            bit_pos = bit_addr & 7
            v = self._sfr_get(byte_addr, internal=True)
            if val:
                v |= (1 << bit_pos)
            else:
                v &= ~(1 << bit_pos)
            self._sfr_set(byte_addr, v)

    # Register bank R0-R7 (uses cached _rb = rs << 3)
    def _rn(self, n):
        return self.iram[self._rb + n]

    def _rn_set(self, n, val):
        self.iram[self._rb + n] = val & 0xFF

    # Stack push/pop (sp is a plain attribute now — no property overhead)
    def _push(self, val):
        sp = (self.sp + 1) & 0xFF
        self.sp = sp
        self.iram[sp] = val & 0xFF

    def _pop(self):
        sp = self.sp
        val = self.iram[sp]
        self.sp = (sp - 1) & 0xFF
        return val

    # Signed relative offset
    @staticmethod
    def _rel(byte):
        return byte if byte < 0x80 else byte - 256

    # =====================================================================
    # ALU helpers with flag computation
    # =====================================================================
    def _add(self, a, b, carry_in=0):
        """ADD/ADDC: compute result and set C, AC, OV."""
        result = a + b + carry_in
        # Carry
        self.cy = 1 if result > 0xFF else 0
        # Aux carry (half-carry on low nibble)
        self.ac = 1 if ((a & 0xF) + (b & 0xF) + carry_in) > 0xF else 0
        # Overflow (signed)
        r8 = result & 0xFF
        self.ov = 1 if ((a ^ r8) & (b ^ r8) & 0x80) else 0
        return r8

    def _sub(self, a, b, borrow_in=0):
        """SUBB: compute result and set C, AC, OV."""
        result = a - b - borrow_in
        self.cy = 1 if result < 0 else 0
        self.ac = 1 if ((a & 0xF) - (b & 0xF) - borrow_in) < 0 else 0
        r8 = result & 0xFF
        self.ov = 1 if ((a ^ b) & (a ^ r8) & 0x80) else 0
        return r8

    # =====================================================================
    # Instruction execution
    # =====================================================================
    def _get_operand(self, op):
        """Decode operand for opcodes that use the low nibble pattern:
        0-3: A with source, 4: A with #imm, 5: A with direct,
        6-7: A with @R0/@R1, 8-F: A with R0-R7
        Returns the operand VALUE."""
        lo = op & 0x0F
        if lo == 4:
            return self._fetch()
        elif lo == 5:
            return self._direct_get(self._fetch())
        elif lo in (6, 7):
            return self._indirect_get(self._rn(lo - 6))
        elif 8 <= lo <= 0xF:
            return self._rn(lo - 8)
        return 0  # shouldn't reach

    def _set_operand(self, op, val):
        """Write back to the location indicated by low nibble (5,6,7,8-F)."""
        lo = op & 0x0F
        val &= 0xFF
        if lo == 5:
            # direct addr was already fetched - this needs the addr stored
            raise ValueError("_set_operand(5) needs special handling")
        elif lo in (6, 7):
            self._indirect_set(self._rn(lo - 6), val)
        elif 8 <= lo <= 0xF:
            self._rn_set(lo - 8, val)

    def step(self):
        """Execute one instruction. Returns False if halted."""
        if self.halted:
            return False
        op = self._fetch()
        self.steps += 1
        _OP_TABLE[op](self)
        return not self.halted

    # 8051 interrupt table: (IE bit, enable check, flag setter, vector, remote)
    # Priority order matches hardware (low index = high priority).
    # `remote` = True for bridge-sourced flags (need _sync_hw_interrupts first);
    # `remote` = False for pure local state (timers, serial, WDT).
    _IRQ_TABLE = [
        # (ie_bit, flag_check,              flag_set,                          vector,  remote)
        (0, lambda s: s.xfr_get(0xFFB8)&0x08, lambda s: None, 0x0003, True),  # EX0: scaler int (ISR W1Cs FFB8 bit 3)
        (1, lambda s: s._timer_tick(0, 0x88, 0x10, 0x20), lambda s: s._sfr_set(0x88, s._sfr_get(0x88)|0x20), 0x000B, False),  # ET0
        (2, lambda s: s.xfr_get(0xFFB9)&0x08, lambda s: None, 0x0013, True),  # EX1: ext int 1 (ISR W1Cs FFB9 bit 3)
        (3, lambda s: s._timer_tick(1, 0x88, 0x40, 0x80), lambda s: s._sfr_set(0x88, s._sfr_get(0x88)|0x80), 0x001B, False),  # ET1
        (4, lambda s: s._sfr_get(0x98)&0x03,lambda s: None, 0x0023, False),  # ES: serial (SCON.RI|SCON.TI)
        (5, lambda s: s._timer_tick(2, 0xC8, 0x04, 0x80), lambda s: s._sfr_set(0xC8, s._sfr_get(0xC8)|0x80), 0x002B, False),  # ET2
        # WDT interrupt: enable in FFEB bit 2, fires when counter >= threshold (FFAE + FFB0[7:4])
        (-1, lambda s: s.xfr_get(0xFFEB)&0x04 and s._wdt_counter_reached(), lambda s: s._xfr_or(0xFFEB, 0x80), 0x0043, False),
    ]

    _WDT_TICKS_PER_INSN = 30

    def _xfr_or(self, addr, mask):
        """Set bits in the local XFR mirror without going through the policy."""
        self.xfr_set(addr, self.xfr_get(addr) | mask & 0xFF)

    def _wdt_counter_reached(self):
        """Check if WDT counter >= threshold (FFAE + FFB0[7:4])."""
        counter = min((self.steps - self._wdt_counter_base) * self._WDT_TICKS_PER_INSN, 0xFFF)
        threshold = ((self.xfr_get(0xFFB0) & 0xF0) << 4) | self.xfr_get(0xFFAE)
        return counter >= threshold and threshold > 0

    def _timer_tick(self, idx, sfr, run_bit, flag_bit):
        """Timer fires whenever running. Rate is paced by the local IRQ poll cadence."""
        return bool(self._sfr_get(sfr) & run_bit)

    def _sync_hw_interrupts(self):
        """In bridge mode, sync real interrupt flags from hardware into local mirror.
        FF00/FFB8/FFB9 are faked for perf; we refresh them here per poll tick.
        """
        if not self.bridge:
            return
        # int0_status
        hw = self.bridge.read(0xFF, 0xB8)
        if hw & 0x08:
            self.xfr_set(0xFFB8, self.xfr_get(0xFFB8) | 0x08)
        # int1_status
        hw = self.bridge.read(0xFF, 0xB9)
        if hw & 0x08:
            self.xfr_set(0xFFB9, self.xfr_get(0xFFB9) | 0x08)
        self.xfr_set(0xFF00, self.bridge.read(0xFF, 0x00))   # IRQ_STATUS

    # IP (SFR 0xB8) bit per interrupt source (matches IRQ_TABLE order):
    # bit 0=PX0, 1=PT0, 2=PX1, 3=PT1, 4=PS, 5=PT2, (-=WDT no priority bit, treat as low)
    _IRQ_PRIO_BITS = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00]

    def _check_interrupts(self, include_remote=True):
        """Poll and fire pending interrupts. Called periodically from run().
        Supports 2-level nesting: high-priority (level 2) preempts low (level 1).
        Same/lower level is blocked until RETI.

        include_remote=True: also sync bridge-sourced flags and consider remote
        IRQs (EX0/EX1). Slower because of bridge I/O.
        include_remote=False: local-only poll (timers, serial, WDT). Zero bridge I/O."""
        if include_remote:
            self._sync_hw_interrupts()
        ie = self._sfr_get(0xA8)
        if not (ie & 0x80):             # EA (global enable) off
            return
        ip = self._sfr_get(0xB8)
        for idx, (ie_bit, check, set_flag, vector, remote) in enumerate(self._IRQ_TABLE):
            if remote and not include_remote:
                continue
            if (ie_bit >= 0 and not (ie & (1 << ie_bit))):
                continue
            prio_bit = self._IRQ_PRIO_BITS[idx]
            level = 2 if (ip & prio_bit) else 1
            if level <= self._isr_level:
                continue                # can't preempt same/higher level
            if not check(self):
                continue
            set_flag(self)
            self._push(self.pc & 0xFF)
            self._push((self.pc >> 8) & 0xFF)
            self.pc = vector
            self._isr_stack.append(self._isr_level)
            self._isr_level = level
            return                      # one interrupt per poll

    def run(self, max_steps=0, stop_on_ljmp=False, stop_addr=None, stop_bank=0):
        """Run until halt, stop_addr, or max_steps (0 = unlimited)."""
        init_sp = self.sp
        start = self.steps
        HEARTBEAT_INTERVAL = 100000        # heartbeat every N steps
        CLOCK_SAMPLE_INTERVAL = 1000       # sample wall clock every N steps
        hb_next = start + HEARTBEAT_INTERVAL
        now = time.monotonic()
        local_irq_period = 0.001           # 1 ms — local interrupt poll (timers/serial/WDT)
        remote_irq_period = 0.010          # 10 ms — remote interrupt poll (bridge-sourced)
        local_irq_next = now + local_irq_period
        remote_irq_next = now + remote_irq_period
        clock_sample_next = start + CLOCK_SAMPLE_INTERVAL

        # Local refs for hot-loop speed
        op_table = _OP_TABLE
        fw = self.fw
        iram = self.iram

        it = range(max_steps) if max_steps else iter(int, 1)
        for _ in it:
            steps = self.steps
            if steps >= hb_next:
                sp = self.sp
                depth = (sp - init_sp) // 2
                if sp > init_sp:
                    pcl, pch = iram[sp - 1], iram[sp]
                    ret = f"{self.bank}:0x{(pch << 8 | pcl):04X}"
                else:
                    ret = "-"
                print(f"  [{steps}] {self.bank}:0x{self.pc:04X}  sp={sp:02X} depth={depth} ret={ret}", file=sys.stderr, flush=True)
                hb_next += HEARTBEAT_INTERVAL
            if steps >= clock_sample_next:
                now = time.monotonic()
                clock_sample_next = steps + CLOCK_SAMPLE_INTERVAL
                if now >= remote_irq_next:
                    self._check_interrupts(include_remote=True)
                    remote_irq_next = now + remote_irq_period
                    local_irq_next = now + local_irq_period
                elif now >= local_irq_next:
                    self._check_interrupts(include_remote=False)
                    local_irq_next = now + local_irq_period
            if self.halted:
                break
            if stop_addr is not None and self.pc == stop_addr and self.bank == stop_bank:
                self.halt_reason = f"reached stop address {stop_bank}:0x{stop_addr:04X}"
                break

            # Peek opcode for function-trace mode exit detection
            pc = self.pc
            offset = self.bank * 0x10000 + pc
            op = fw[offset] if offset < len(fw) else 0xFF

            if stop_on_ljmp:
                if op == 0x02:  # LJMP
                    self.pc = (pc + 1) & 0xFFFF
                    target = self._fetch16()
                    self.halt_reason = f"LJMP 0x{target:04X} (tail call exit)"
                    break
                if op == 0x22 and self.sp == init_sp:  # RET at top level
                    self.halt_reason = "top-level RET"
                    break

            # Inline step: advance PC past opcode, dispatch
            self.pc = (pc + 1) & 0xFFFF
            self.steps = steps + 1
            op_table[op](self)
            if self.halted:
                break

        if not self.halted and not self.halt_reason and max_steps:
            self.halt_reason = f"max_steps ({max_steps}) reached"


# =========================================================================
# Opcode dispatch table — 256-entry list of handler functions.
# Each handler takes a single argument: the Sim8051 instance.
# Built once at module load; step() and run() index into it.
# =========================================================================

def _build_op_table():
    T = [None] * 256
    _rel = Sim8051._rel  # static method, local ref

    # --- 0x00: NOP ---
    def op_00(s): pass
    T[0x00] = op_00

    # --- AJMP addr11 (aaa00001): 0x01,0x21,0x41,0x61,0x81,0xA1,0xC1,0xE1 ---
    for _op in range(0x01, 0x100, 0x20):
        _pg = ((_op >> 5) & 7) << 8
        def _ajmp(s, pg=_pg):
            lo = s._fetch()
            s.pc = (s.pc & 0xF800) | pg | lo
        T[_op] = _ajmp

    # --- 0x02: LJMP addr16 ---
    def op_02(s):
        s.pc = s._fetch16()
    T[0x02] = op_02

    # --- 0x03: RR A ---
    def op_03(s):
        a = s.a
        s.a = ((a >> 1) | ((a & 1) << 7)) & 0xFF
    T[0x03] = op_03

    # --- 0x04: INC A ---
    def op_04(s):
        s.a = (s.a + 1) & 0xFF
    T[0x04] = op_04

    # --- 0x05: INC direct ---
    def op_05(s):
        d = s._fetch()
        s._direct_set(d, (s._direct_get(d) + 1) & 0xFF)
    T[0x05] = op_05

    # --- 0x06-0x07: INC @Ri ---
    for _i in range(2):
        def _inc_ri(s, i=_i):
            a = s._rn(i)
            s._indirect_set(a, (s._indirect_get(a) + 1) & 0xFF)
        T[0x06 + _i] = _inc_ri

    # --- 0x08-0x0F: INC Rn ---
    for _n in range(8):
        def _inc_rn(s, n=_n):
            s._rn_set(n, (s._rn(n) + 1) & 0xFF)
        T[0x08 + _n] = _inc_rn

    # --- 0x10: JBC bit, rel ---
    def op_10(s):
        bit = s._fetch()
        rel = _rel(s._fetch())
        if s._bit_get(bit):
            s._bit_set(bit, 0)
            s.pc = (s.pc + rel) & 0xFFFF
    T[0x10] = op_10

    # --- ACALL addr11 (aaa10001): 0x11,0x31,0x51,0x71,0x91,0xB1,0xD1,0xF1 ---
    for _op in range(0x11, 0x100, 0x20):
        _pg = ((_op >> 5) & 7) << 8
        def _acall(s, pg=_pg):
            lo = s._fetch()
            target = (s.pc & 0xF800) | pg | lo
            s._push(s.pc & 0xFF)
            s._push((s.pc >> 8) & 0xFF)
            s.pc = target
        T[_op] = _acall

    # --- 0x12: LCALL addr16 ---
    def op_12(s):
        target = s._fetch16()
        s._push(s.pc & 0xFF)
        s._push((s.pc >> 8) & 0xFF)
        s.pc = target
    T[0x12] = op_12

    # --- 0x13: RRC A ---
    def op_13(s):
        a = s.a; c = s.cy
        s.cy = a & 1
        s.a = ((c << 7) | (a >> 1)) & 0xFF
    T[0x13] = op_13

    # --- 0x14: DEC A ---
    def op_14(s):
        s.a = (s.a - 1) & 0xFF
    T[0x14] = op_14

    # --- 0x15: DEC direct ---
    def op_15(s):
        d = s._fetch()
        s._direct_set(d, (s._direct_get(d) - 1) & 0xFF)
    T[0x15] = op_15

    # --- 0x16-0x17: DEC @Ri ---
    for _i in range(2):
        def _dec_ri(s, i=_i):
            a = s._rn(i)
            s._indirect_set(a, (s._indirect_get(a) - 1) & 0xFF)
        T[0x16 + _i] = _dec_ri

    # --- 0x18-0x1F: DEC Rn ---
    for _n in range(8):
        def _dec_rn(s, n=_n):
            s._rn_set(n, (s._rn(n) - 1) & 0xFF)
        T[0x18 + _n] = _dec_rn

    # --- 0x20: JB bit, rel ---
    def op_20(s):
        bit = s._fetch()
        rel = _rel(s._fetch())
        if s._bit_get(bit):
            s.pc = (s.pc + rel) & 0xFFFF
    T[0x20] = op_20

    # --- 0x22: RET ---
    def op_22(s):
        hi = s._pop(); lo = s._pop()
        s.pc = (hi << 8) | lo
    T[0x22] = op_22

    # --- 0x23: RL A ---
    def op_23(s):
        a = s.a
        s.a = ((a << 1) | (a >> 7)) & 0xFF
    T[0x23] = op_23

    # --- 0x24: ADD A, #imm ---
    def op_24(s):
        s.a = s._add(s.a, s._fetch())
    T[0x24] = op_24

    # --- 0x25: ADD A, direct ---
    def op_25(s):
        s.a = s._add(s.a, s._direct_get(s._fetch()))
    T[0x25] = op_25

    # --- 0x26-0x27: ADD A, @Ri ---
    for _i in range(2):
        def _add_ri(s, i=_i):
            s.a = s._add(s.a, s._indirect_get(s._rn(i)))
        T[0x26 + _i] = _add_ri

    # --- 0x28-0x2F: ADD A, Rn ---
    for _n in range(8):
        def _add_rn(s, n=_n):
            s.a = s._add(s.a, s._rn(n))
        T[0x28 + _n] = _add_rn

    # --- 0x30: JNB bit, rel ---
    def op_30(s):
        bit = s._fetch()
        rel = _rel(s._fetch())
        if not s._bit_get(bit):
            s.pc = (s.pc + rel) & 0xFFFF
    T[0x30] = op_30

    # --- 0x32: RETI ---
    def op_32(s):
        hi = s._pop(); lo = s._pop()
        s.pc = (hi << 8) | lo
        s._isr_level = s._isr_stack.pop() if s._isr_stack else 0
    T[0x32] = op_32

    # --- 0x33: RLC A ---
    def op_33(s):
        a = s.a; c = s.cy
        s.cy = (a >> 7) & 1
        s.a = ((a << 1) | c) & 0xFF
    T[0x33] = op_33

    # --- 0x34: ADDC A, #imm ---
    def op_34(s):
        s.a = s._add(s.a, s._fetch(), s.cy)
    T[0x34] = op_34

    # --- 0x35: ADDC A, direct ---
    def op_35(s):
        s.a = s._add(s.a, s._direct_get(s._fetch()), s.cy)
    T[0x35] = op_35

    # --- 0x36-0x37: ADDC A, @Ri ---
    for _i in range(2):
        def _addc_ri(s, i=_i):
            s.a = s._add(s.a, s._indirect_get(s._rn(i)), s.cy)
        T[0x36 + _i] = _addc_ri

    # --- 0x38-0x3F: ADDC A, Rn ---
    for _n in range(8):
        def _addc_rn(s, n=_n):
            s.a = s._add(s.a, s._rn(n), s.cy)
        T[0x38 + _n] = _addc_rn

    # --- 0x40: JC rel ---
    def op_40(s):
        rel = _rel(s._fetch())
        if s.cy:
            s.pc = (s.pc + rel) & 0xFFFF
    T[0x40] = op_40

    # --- 0x42: ORL direct, A ---
    def op_42(s):
        d = s._fetch()
        s._direct_set(d, s._direct_get(d) | s.a)
    T[0x42] = op_42

    # --- 0x43: ORL direct, #imm ---
    def op_43(s):
        d = s._fetch(); imm = s._fetch()
        s._direct_set(d, s._direct_get(d) | imm)
    T[0x43] = op_43

    # --- 0x44: ORL A, #imm ---
    def op_44(s):
        s.a |= s._fetch()
    T[0x44] = op_44

    # --- 0x45: ORL A, direct ---
    def op_45(s):
        s.a |= s._direct_get(s._fetch())
    T[0x45] = op_45

    # --- 0x46-0x47: ORL A, @Ri ---
    for _i in range(2):
        def _orl_ri(s, i=_i):
            s.a |= s._indirect_get(s._rn(i))
        T[0x46 + _i] = _orl_ri

    # --- 0x48-0x4F: ORL A, Rn ---
    for _n in range(8):
        def _orl_rn(s, n=_n):
            s.a |= s._rn(n)
        T[0x48 + _n] = _orl_rn

    # --- 0x50: JNC rel ---
    def op_50(s):
        rel = _rel(s._fetch())
        if not s.cy:
            s.pc = (s.pc + rel) & 0xFFFF
    T[0x50] = op_50

    # --- 0x52: ANL direct, A ---
    def op_52(s):
        d = s._fetch()
        s._direct_set(d, s._direct_get(d) & s.a)
    T[0x52] = op_52

    # --- 0x53: ANL direct, #imm ---
    def op_53(s):
        d = s._fetch(); imm = s._fetch()
        s._direct_set(d, s._direct_get(d) & imm)
    T[0x53] = op_53

    # --- 0x54: ANL A, #imm ---
    def op_54(s):
        s.a &= s._fetch()
    T[0x54] = op_54

    # --- 0x55: ANL A, direct ---
    def op_55(s):
        s.a &= s._direct_get(s._fetch())
    T[0x55] = op_55

    # --- 0x56-0x57: ANL A, @Ri ---
    for _i in range(2):
        def _anl_ri(s, i=_i):
            s.a &= s._indirect_get(s._rn(i))
        T[0x56 + _i] = _anl_ri

    # --- 0x58-0x5F: ANL A, Rn ---
    for _n in range(8):
        def _anl_rn(s, n=_n):
            s.a &= s._rn(n)
        T[0x58 + _n] = _anl_rn

    # --- 0x60: JZ rel ---
    def op_60(s):
        rel = _rel(s._fetch())
        if s.a == 0:
            s.pc = (s.pc + rel) & 0xFFFF
    T[0x60] = op_60

    # --- 0x62: XRL direct, A ---
    def op_62(s):
        d = s._fetch()
        s._direct_set(d, s._direct_get(d) ^ s.a)
    T[0x62] = op_62

    # --- 0x63: XRL direct, #imm ---
    def op_63(s):
        d = s._fetch(); imm = s._fetch()
        s._direct_set(d, s._direct_get(d) ^ imm)
    T[0x63] = op_63

    # --- 0x64: XRL A, #imm ---
    def op_64(s):
        s.a ^= s._fetch()
    T[0x64] = op_64

    # --- 0x65: XRL A, direct ---
    def op_65(s):
        s.a ^= s._direct_get(s._fetch())
    T[0x65] = op_65

    # --- 0x66-0x67: XRL A, @Ri ---
    for _i in range(2):
        def _xrl_ri(s, i=_i):
            s.a ^= s._indirect_get(s._rn(i))
        T[0x66 + _i] = _xrl_ri

    # --- 0x68-0x6F: XRL A, Rn ---
    for _n in range(8):
        def _xrl_rn(s, n=_n):
            s.a ^= s._rn(n)
        T[0x68 + _n] = _xrl_rn

    # --- 0x70: JNZ rel ---
    def op_70(s):
        rel = _rel(s._fetch())
        if s.a != 0:
            s.pc = (s.pc + rel) & 0xFFFF
    T[0x70] = op_70

    # --- 0x72: ORL C, bit ---
    def op_72(s):
        s.cy = s.cy | s._bit_get(s._fetch())
    T[0x72] = op_72

    # --- 0x73: JMP @A+DPTR ---
    def op_73(s):
        s.pc = (s.a + s.dptr) & 0xFFFF
    T[0x73] = op_73

    # --- 0x74: MOV A, #imm ---
    def op_74(s):
        s.a = s._fetch()
    T[0x74] = op_74

    # --- 0x75: MOV direct, #imm ---
    def op_75(s):
        d = s._fetch(); imm = s._fetch()
        s._direct_set(d, imm)
    T[0x75] = op_75

    # --- 0x76-0x77: MOV @Ri, #imm ---
    for _i in range(2):
        def _mov_ri_imm(s, i=_i):
            s._indirect_set(s._rn(i), s._fetch())
        T[0x76 + _i] = _mov_ri_imm

    # --- 0x78-0x7F: MOV Rn, #imm ---
    for _n in range(8):
        def _mov_rn_imm(s, n=_n):
            s._rn_set(n, s._fetch())
        T[0x78 + _n] = _mov_rn_imm

    # --- 0x80: SJMP rel ---
    def op_80(s):
        rel = _rel(s._fetch())
        s.pc = (s.pc + rel) & 0xFFFF
    T[0x80] = op_80

    # --- 0x82: ANL C, bit ---
    def op_82(s):
        s.cy = s.cy & s._bit_get(s._fetch())
    T[0x82] = op_82

    # --- 0x83: MOVC A, @A+PC ---
    def op_83(s):
        s.a = s._code_read((s.a + s.pc) & 0xFFFF)
    T[0x83] = op_83

    # --- 0x84: DIV AB ---
    def op_84(s):
        if s.b == 0:
            s.ov = 1
        else:
            q, r = divmod(s.a, s.b)
            s.a = q & 0xFF
            s.b = r & 0xFF
            s.ov = 0
        s.cy = 0
    T[0x84] = op_84

    # --- 0x85: MOV direct, direct ---
    def op_85(s):
        src = s._fetch(); dst = s._fetch()
        s._direct_set(dst, s._direct_get(src))
    T[0x85] = op_85

    # --- 0x86-0x87: MOV direct, @Ri ---
    for _i in range(2):
        def _mov_d_ri(s, i=_i):
            d = s._fetch()
            s._direct_set(d, s._indirect_get(s._rn(i)))
        T[0x86 + _i] = _mov_d_ri

    # --- 0x88-0x8F: MOV direct, Rn ---
    for _n in range(8):
        def _mov_d_rn(s, n=_n):
            d = s._fetch()
            s._direct_set(d, s._rn(n))
        T[0x88 + _n] = _mov_d_rn

    # --- 0x90: MOV DPTR, #imm16 ---
    def op_90(s):
        s.dptr = s._fetch16()
    T[0x90] = op_90

    # --- 0x92: MOV bit, C ---
    def op_92(s):
        s._bit_set(s._fetch(), s.cy)
    T[0x92] = op_92

    # --- 0x93: MOVC A, @A+DPTR ---
    def op_93(s):
        s.a = s._code_read((s.a + s.dptr) & 0xFFFF)
    T[0x93] = op_93

    # --- 0x94: SUBB A, #imm ---
    def op_94(s):
        s.a = s._sub(s.a, s._fetch(), s.cy)
    T[0x94] = op_94

    # --- 0x95: SUBB A, direct ---
    def op_95(s):
        s.a = s._sub(s.a, s._direct_get(s._fetch()), s.cy)
    T[0x95] = op_95

    # --- 0x96-0x97: SUBB A, @Ri ---
    for _i in range(2):
        def _subb_ri(s, i=_i):
            s.a = s._sub(s.a, s._indirect_get(s._rn(i)), s.cy)
        T[0x96 + _i] = _subb_ri

    # --- 0x98-0x9F: SUBB A, Rn ---
    for _n in range(8):
        def _subb_rn(s, n=_n):
            s.a = s._sub(s.a, s._rn(n), s.cy)
        T[0x98 + _n] = _subb_rn

    # --- 0xA0: ORL C, /bit ---
    def op_A0(s):
        s.cy = s.cy | (1 - s._bit_get(s._fetch()))
    T[0xA0] = op_A0

    # --- 0xA2: MOV C, bit ---
    def op_A2(s):
        s.cy = s._bit_get(s._fetch())
    T[0xA2] = op_A2

    # --- 0xA3: INC DPTR ---
    def op_A3(s):
        s.dptr = (s.dptr + 1) & 0xFFFF
    T[0xA3] = op_A3

    # --- 0xA4: MUL AB ---
    def op_A4(s):
        result = s.a * s.b
        s.a = result & 0xFF
        s.b = (result >> 8) & 0xFF
        s.cy = 0
        s.ov = 1 if result > 0xFF else 0
    T[0xA4] = op_A4

    # --- 0xA6-0xA7: MOV @Ri, direct ---
    for _i in range(2):
        def _mov_ri_d(s, i=_i):
            d = s._fetch()
            s._indirect_set(s._rn(i), s._direct_get(d))
        T[0xA6 + _i] = _mov_ri_d

    # --- 0xA8-0xAF: MOV Rn, direct ---
    for _n in range(8):
        def _mov_rn_d(s, n=_n):
            d = s._fetch()
            s._rn_set(n, s._direct_get(d))
        T[0xA8 + _n] = _mov_rn_d

    # --- 0xB0: ANL C, /bit ---
    def op_B0(s):
        s.cy = s.cy & (1 - s._bit_get(s._fetch()))
    T[0xB0] = op_B0

    # --- 0xB2: CPL bit ---
    def op_B2(s):
        bit = s._fetch()
        s._bit_set(bit, 1 - s._bit_get(bit))
    T[0xB2] = op_B2

    # --- 0xB3: CPL C ---
    def op_B3(s):
        s.cy = 1 - s.cy
    T[0xB3] = op_B3

    # --- 0xB4: CJNE A, #imm, rel ---
    def op_B4(s):
        imm = s._fetch()
        rel = _rel(s._fetch())
        s.cy = 1 if s.a < imm else 0
        if s.a != imm:
            s.pc = (s.pc + rel) & 0xFFFF
    T[0xB4] = op_B4

    # --- 0xB5: CJNE A, direct, rel ---
    def op_B5(s):
        d = s._fetch()
        rel = _rel(s._fetch())
        val = s._direct_get(d)
        s.cy = 1 if s.a < val else 0
        if s.a != val:
            s.pc = (s.pc + rel) & 0xFFFF
    T[0xB5] = op_B5

    # --- 0xB6-0xB7: CJNE @Ri, #imm, rel ---
    for _i in range(2):
        def _cjne_ri(s, i=_i):
            imm = s._fetch()
            rel = _rel(s._fetch())
            val = s._indirect_get(s._rn(i))
            s.cy = 1 if val < imm else 0
            if val != imm:
                s.pc = (s.pc + rel) & 0xFFFF
        T[0xB6 + _i] = _cjne_ri

    # --- 0xB8-0xBF: CJNE Rn, #imm, rel ---
    for _n in range(8):
        def _cjne_rn(s, n=_n):
            imm = s._fetch()
            rel = _rel(s._fetch())
            val = s._rn(n)
            s.cy = 1 if val < imm else 0
            if val != imm:
                s.pc = (s.pc + rel) & 0xFFFF
        T[0xB8 + _n] = _cjne_rn

    # --- 0xC0: PUSH direct ---
    def op_C0(s):
        s._push(s._direct_get(s._fetch()))
    T[0xC0] = op_C0

    # --- 0xC2: CLR bit ---
    def op_C2(s):
        s._bit_set(s._fetch(), 0)
    T[0xC2] = op_C2

    # --- 0xC3: CLR C ---
    def op_C3(s):
        s.cy = 0
    T[0xC3] = op_C3

    # --- 0xC4: SWAP A ---
    def op_C4(s):
        s.a = ((s.a >> 4) | (s.a << 4)) & 0xFF
    T[0xC4] = op_C4

    # --- 0xC5: XCH A, direct ---
    def op_C5(s):
        d = s._fetch()
        tmp = s._direct_get(d)
        s._direct_set(d, s.a)
        s.a = tmp
    T[0xC5] = op_C5

    # --- 0xC6-0xC7: XCH A, @Ri ---
    for _i in range(2):
        def _xch_ri(s, i=_i):
            addr = s._rn(i)
            tmp = s._indirect_get(addr)
            s._indirect_set(addr, s.a)
            s.a = tmp
        T[0xC6 + _i] = _xch_ri

    # --- 0xC8-0xCF: XCH A, Rn ---
    for _n in range(8):
        def _xch_rn(s, n=_n):
            tmp = s._rn(n)
            s._rn_set(n, s.a)
            s.a = tmp
        T[0xC8 + _n] = _xch_rn

    # --- 0xD0: POP direct ---
    def op_D0(s):
        d = s._fetch()
        s._direct_set(d, s._pop())
    T[0xD0] = op_D0

    # --- 0xD2: SETB bit ---
    def op_D2(s):
        s._bit_set(s._fetch(), 1)
    T[0xD2] = op_D2

    # --- 0xD3: SETB C ---
    def op_D3(s):
        s.cy = 1
    T[0xD3] = op_D3

    # --- 0xD4: DA A ---
    def op_D4(s):
        a = s.a
        if (a & 0x0F) > 9 or s.ac:
            a += 6
            if a > 0xFF:
                s.cy = 1
        if ((a >> 4) & 0x0F) > 9 or s.cy:
            a += 0x60
            if a > 0xFF:
                s.cy = 1
        s.a = a & 0xFF
    T[0xD4] = op_D4

    # --- 0xD5: DJNZ direct, rel ---
    def op_D5(s):
        d = s._fetch()
        rel = _rel(s._fetch())
        val = (s._direct_get(d) - 1) & 0xFF
        s._direct_set(d, val)
        if val != 0:
            s.pc = (s.pc + rel) & 0xFFFF
    T[0xD5] = op_D5

    # --- 0xD6-0xD7: XCHD A, @Ri ---
    for _i in range(2):
        def _xchd(s, i=_i):
            addr = s._rn(i)
            val = s._indirect_get(addr)
            s._indirect_set(addr, (val & 0xF0) | (s.a & 0x0F))
            s.a = (s.a & 0xF0) | (val & 0x0F)
        T[0xD6 + _i] = _xchd

    # --- 0xD8-0xDF: DJNZ Rn, rel ---
    for _n in range(8):
        def _djnz_rn(s, n=_n):
            rel = _rel(s._fetch())
            val = (s._rn(n) - 1) & 0xFF
            s._rn_set(n, val)
            if val != 0:
                s.pc = (s.pc + rel) & 0xFFFF
        T[0xD8 + _n] = _djnz_rn

    # --- 0xE0: MOVX A, @DPTR ---
    def op_E0(s):
        s.a = s._xdata_get(s.dptr)
    T[0xE0] = op_E0

    # --- 0xE2-0xE3: MOVX A, @Ri ---
    for _i in range(2):
        def _movx_a_ri(s, i=_i):
            addr = s._rn(i) | (s._sfr_get(0xA0) << 8)
            s.a = s._xdata_get(addr)
        T[0xE2 + _i] = _movx_a_ri

    # --- 0xE4: CLR A ---
    def op_E4(s):
        s.a = 0
    T[0xE4] = op_E4

    # --- 0xE5: MOV A, direct ---
    def op_E5(s):
        s.a = s._direct_get(s._fetch())
    T[0xE5] = op_E5

    # --- 0xE6-0xE7: MOV A, @Ri ---
    for _i in range(2):
        def _mov_a_ri(s, i=_i):
            s.a = s._indirect_get(s._rn(i))
        T[0xE6 + _i] = _mov_a_ri

    # --- 0xE8-0xEF: MOV A, Rn ---
    for _n in range(8):
        def _mov_a_rn(s, n=_n):
            s.a = s._rn(n)
        T[0xE8 + _n] = _mov_a_rn

    # --- 0xF0: MOVX @DPTR, A ---
    def op_F0(s):
        s._xdata_set(s.dptr, s.a)
    T[0xF0] = op_F0

    # --- 0xF2-0xF3: MOVX @Ri, A ---
    for _i in range(2):
        def _movx_ri_a(s, i=_i):
            addr = s._rn(i) | (s._sfr_get(0xA0) << 8)
            s._xdata_set(addr, s.a)
        T[0xF2 + _i] = _movx_ri_a

    # --- 0xF4: CPL A ---
    def op_F4(s):
        s.a = (~s.a) & 0xFF
    T[0xF4] = op_F4

    # --- 0xF5: MOV direct, A ---
    def op_F5(s):
        s._direct_set(s._fetch(), s.a)
    T[0xF5] = op_F5

    # --- 0xF6-0xF7: MOV @Ri, A ---
    for _i in range(2):
        def _mov_ri_a(s, i=_i):
            s._indirect_set(s._rn(i), s.a)
        T[0xF6 + _i] = _mov_ri_a

    # --- 0xF8-0xFF: MOV Rn, A ---
    for _n in range(8):
        def _mov_rn_a(s, n=_n):
            s._rn_set(n, s.a)
        T[0xF8 + _n] = _mov_rn_a

    # --- Fill remaining slots (unhandled opcodes) ---
    for _op in range(256):
        if T[_op] is None:
            def _bad(s, op=_op):
                s.halted = True
                s.halt_reason = f"unhandled opcode 0x{op:02X} at 0x{(s.pc-1)&0xFFFF:04X}"
            T[_op] = _bad

    return T

_OP_TABLE = _build_op_table()


# =========================================================================
# Regmap loader
# =========================================================================
def load_regmap(path):
    """Load rtd2795t_cg_regmap.json → dict mapping (page, reg) → name."""
    import json
    with open(path) as f:
        raw = json.load(f)
    names = {}  # (page, reg) -> name
    for key, info in raw.items():
        parts = key.split(":")
        if len(parts) >= 3:
            try:
                page = int(parts[0][1:], 16)
                reg = int(parts[1], 16)
            except ValueError:
                continue
            names[(page, reg)] = parts[2]
    return names


def addr_name(regmap, addr):
    """Annotate a 16-bit XDATA address with a human-readable name."""
    page = (addr >> 8) & 0xFF
    reg = addr & 0xFF
    name = regmap.get((page, reg))
    if name:
        return f"P{page:02X}:{reg:02X} {name}"
    if addr >= _XFR_BASE:
        return f"XFR_{addr:04X}"
    return f"P{page:02X}:{reg:02X}"


# =========================================================================
# CLI: general-purpose firmware trace tool
# =========================================================================
def main():
    import argparse

    p = argparse.ArgumentParser(
        description="8051 firmware trace — execute locally, optionally proxy to real hardware")
    p.add_argument("fw", default=None, help="firmware binary path")
    p.add_argument("-e", "--entry", type=str, default="0:0x0000",
                   help="entry as bank:addr (e.g. 4:0xEDC7, default: 0:0x0000)")
    p.add_argument("-n", "--max-steps", type=int, default=0,
                   help="max instructions (0 = unlimited)")
    p.add_argument("--stop-addr", type=str, default=None,
                   help="stop at bank:addr (e.g. 4:0xEDC7)")
    p.add_argument("--no-stop-on-ljmp", action="store_true",
                   help="don't treat LJMP as function exit (default: stop on LJMP)")
    p.add_argument("--proxy", action="store_true",
                   help="proxy to real hardware via debug bridge")
    p.add_argument("--regmap", default=None, help="register map JSON path")
    p.add_argument("--log", action="store_true",
                   help="enable access logging")
    p.add_argument("--patch", action="append", default=[], metavar="ADDR=HEX",
                   help="patch firmware bytes: e.g. 0x2B76=00,00,00 (NOP out delay)")
    args = p.parse_args()

    stop_on_ljmp = not args.no_stop_on_ljmp

    def parse_bank_addr(s):
        if ':' in s:
            b, a = s.split(':', 1)
            return int(b, 0), int(a, 0)
        return 0, int(s, 0)

    entry_bank, entry_pc = parse_bank_addr(args.entry)
    stop_bank, stop_addr = parse_bank_addr(args.stop_addr) if args.stop_addr else (0, None)

    # Load firmware
    if args.fw:
        with open(args.fw, "rb") as f:
            fw = f.read()
    else:
        p.print_help()
        return

    # Load register map
    regmap = {}
    if args.regmap:
        import os
        if os.path.exists(args.regmap):
            regmap = load_regmap(args.regmap)

    # Set up bridge for proxy mode
    bridge = None
    if args.proxy:
        from rtd_i2c import RealtekI2C
        from rtd_scaler import DebugBridge
        i2c = RealtekI2C()
        bridge = DebugBridge(i2c)
        bridge.enter()

    try:
        sim = Sim8051(fw, entry_bank, bridge=bridge)
        sim.pc = entry_pc

        # Apply firmware patches
        for spec in args.patch:
            addr_s, hex_s = spec.split("=")
            addr = int(addr_s, 0)
            data = bytes.fromhex(hex_s.replace(",", ""))
            sim.fw[addr:addr + len(data)] = data

        sim.logging = args.log
        sim.run(max_steps=args.max_steps, stop_on_ljmp=stop_on_ljmp,
                stop_addr=stop_addr, stop_bank=stop_bank)

        # Header
        mode = "PROXY" if args.proxy else "local"
        print(f"Trace bank {entry_bank}:0x{entry_pc:04X}  [{mode}]  {args.fw}")
        print(f"{sim.steps} instructions, stopped: {sim.halt_reason}")

        # Print access log
        if sim.trace_log:
            print()
            sfr_names = {0x90: 'P1', 0xB0: 'P3'}
            for entry in sim.trace_log:
                kind, addr, val, is_write, bank, pc = entry
                direction = "<-" if is_write else "->"
                if kind == 'sfr':
                    name = f"SFR:{sfr_names.get(addr, f'0x{addr:02X}')}"
                else:
                    name = addr_name(regmap, addr)
                print(f"  {bank}:{pc:04X}  {name:40s} {direction} 0x{val:02X}")

    finally:
        if bridge is not None:
            bridge.exit()
            bridge.i2c.close()


if __name__ == "__main__":
    main()
