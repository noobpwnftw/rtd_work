#!/usr/bin/env python3
"""RX3081 MIPS16 LE disassembler

Covers base MIPS16 plus the Lexra extensions Realtek's MCU actually wires
in: the CP0 family at op=0x17 (b8xx/b9xx/baxx — mfc0/mtc0/eret/sleep/di/ei/
deret/cache) and the MAC family at op=0x1F (f8xx — madh/madl/mazh/mazl/msbh/
msbl/mszh/mszl). Unknown halfwords emit bare hex as the mnemonic.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Callable, Iterator, Optional


# ---------------------------------------------------------------------------
# Register conventions
# ---------------------------------------------------------------------------

# 3-bit "compact" register field → GPR subset.
CREG = ['s0', 's1', 'v0', 'v1', 'a0', 'a1', 'a2', 'a3']

# 5-bit full GPR field.
GPR = [
    'zero','at','v0','v1','a0','a1','a2','a3',
    't0','t1','t2','t3','t4','t5','t6','t7',
    's0','s1','s2','s3','s4','s5','s6','s7',
    't8','t9','k0','k1','gp','sp','s8','ra',
]

# RX3081-specific CP0 register names.
# Registers not listed render as "$N".
CP0_NAMES = {
     0:'c0_index',    1:'c0_random',   2:'c0_entrylo',
     4:'c0_context',  6:'c0_wired',
     8:'c0_badvaddr', 9:'c0_count',   10:'c0_entryhi',  11:'c0_compare',
    12:'c0_status',  13:'c0_cause',   14:'c0_epc',      15:'c0_prid',
    16:'c0_dreg',    17:'c0_depc',    18:'c0_watchlo',  19:'c0_watchhi',
    20:'c0_cctl',    23:'c0_config',  26:'c0_ck0',      27:'c0_ck1',
    28:'c0_datalo',  29:'c0_datahi',  31:'c0_desave',
}


def cp0_name(n: int) -> str:
    return CP0_NAMES.get(n, f'${n}')


# ---------------------------------------------------------------------------
# Numeric helpers
# ---------------------------------------------------------------------------

def sx(v: int, bits: int) -> int:
    """Sign-extend the low `bits` of `v`."""
    m = 1 << (bits - 1)
    return (v & (m - 1)) - (v & m)


def imm(v: int) -> str:
    """Format a signed immediate as hex. `-0xN` for negative, `0xN` for non-neg.
    Hex is far more readable than decimal for firmware RE (16-bit page:reg
    values, bit masks, and MMIO offsets all jump out)."""
    if v < 0:
        return f'-0x{-v:x}'
    return f'0x{v:x}'


def addr32(v: int) -> str:
    return f'0x{v & 0xFFFFFFFF:x}'


def raw16(data: bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def ext_mnem(ext: Optional[int], base: str) -> str:
    return f'{base}32' if ext is not None else base


# ---------------------------------------------------------------------------
# MIPS16e extended-immediate reassembly
# ---------------------------------------------------------------------------
#
# The EXTEND prefix's 11-bit field is split non-contiguously into the 16-bit
# extended immediate for most instructions:
#
#     imm[15:11] = ext[4:0]    (low 5 of EXTEND)
#     imm[10:5]  = ext[10:5]   (high 6 of EXTEND)
#     imm[4:0]   = base[4:0]
#
# RRI-A (op=0x08) uses a 15-bit signed form with ext[0] hoisted to imm[4].

def ext_imm16_u(ext: int, base_low5: int) -> int:
    """Unsigned 16-bit immediate from EXTEND + base low-5."""
    return (((ext & 0x1F) << 11)
            | (((ext >> 5) & 0x3F) << 5)
            | (base_low5 & 0x1F)) & 0xFFFF


def ext_imm16_s(ext: int, base_low5: int) -> int:
    return sx(ext_imm16_u(ext, base_low5), 16)


def ext_imm15_rri_a(ext: int, base_low4: int) -> int:
    """Signed 15-bit immediate for RRI-A (op=0x08)."""
    raw = (((ext & 0xF) << 11)
           | (((ext >> 5) & 0x3F) << 5)
           | (((ext >> 4) & 1) << 4)
           | (base_low4 & 0xF))
    return sx(raw, 15)


# ---------------------------------------------------------------------------
# Decoded result shape
# ---------------------------------------------------------------------------

Decoded = tuple[str, str]
Decoder = Callable[[int, Optional[int], int], Decoded]


def raw_unknown(hw: int) -> Decoded:
    """Sentinel for an un-decodable halfword. Mnemonic is bare hex, args empty."""
    return f'0x{hw:04x}', ''


def is_unknown(decoded: Decoded) -> bool:
    mn, args = decoded
    return args == '' and mn.startswith('0x')


# ---------------------------------------------------------------------------
# Bit-field accessors (top-level halfword layout)
# ---------------------------------------------------------------------------
#
# Most MIPS16 R-form instructions lay out their two 3-bit register fields
# identically. Name them once.

def _rx(hw: int) -> int: return (hw >> 8) & 7
def _ry(hw: int) -> int: return (hw >> 5) & 7


# ===========================================================================
# Sub-decoders
# ===========================================================================

# -- op=0x00  ADDIUSP  rx, sp, imm --------------------------------------
def _addiusp(hw: int, ext: Optional[int], pc: int) -> Decoded:
    if ext is not None:
        if not _require_ext_zero(hw, 0xE0):
            return raw_unknown(hw)
        v = ext_imm16_s(ext, hw & 0x1F)
        return 'addiu32', f'{CREG[_rx(hw)]},sp,{imm(v)}'
    return 'addiu', f'{CREG[_rx(hw)]},sp,{imm((hw & 0xFF) * 4)}'


# -- op=0x01  ADDIUPC / 'la'  rx, target (pc & ~3 + imm) ----------------
def _la(hw: int, ext: Optional[int], pc: int) -> Decoded:
    if ext is not None:
        if not _require_ext_zero(hw, 0xE0):
            return raw_unknown(hw)
        off = ext_imm16_s(ext, hw & 0x1F)
        return 'la32', f'{CREG[_rx(hw)]},{addr32((pc & ~3) + off)}'
    return 'la', f'{CREG[_rx(hw)]},{addr32((pc & ~3) + (hw & 0xFF) * 4)}'


# -- op=0x02/0x04/0x05  b / beqz / bnez ----------------------------------
def _branch(mnem: str, with_reg: bool, base_width: int) -> Decoder:
    """Build a branch decoder. Extended form: target = pc + 4 + imm16 * 2.
    Unextended: target = pc + 2 + sign_extend(imm_{base_width}) * 2.

    Extended-form base halfword has no rx for `b` (reserved [10:5]) and rx
    in [10:8] for beqz/bnez (reserved [7:5]); either way reserved bits must
    be zero or we fall back to raw hex."""
    reserved_mask = 0x7E0 if not with_reg else 0xE0
    def inner(hw: int, ext: Optional[int], pc: int) -> Decoded:
        if ext is not None:
            if not _require_ext_zero(hw, reserved_mask):
                return raw_unknown(hw)
            off = ext_imm16_s(ext, hw & 0x1F)
            tgt = pc + 4 + off * 2
        else:
            base = hw & ((1 << base_width) - 1)
            tgt = pc + 2 + sx(base, base_width) * 2
        tgt_s = addr32(tgt)
        return ext_mnem(ext, mnem), f'{CREG[_rx(hw)]},{tgt_s}' if with_reg else tgt_s
    return inner


_b    = _branch('b',    with_reg=False, base_width=11)
_beqz = _branch('beqz', with_reg=True,  base_width=8)
_bnez = _branch('bnez', with_reg=True,  base_width=8)


# -- op=0x06  sll / srl / sra --------------------------------------------
_SHIFT_MNEMS = {0: 'sll', 2: 'srl', 3: 'sra'}

def _shift(hw: int, ext: Optional[int], pc: int) -> Decoded:
    mnem = _SHIFT_MNEMS.get(hw & 3)
    if mnem is None:
        return raw_unknown(hw)
    rx, ry = _rx(hw), _ry(hw)
    if ext is not None:
        # Extended shift: sa lives in ext[10:6]; ext[5:0] and base[4:2] are
        # reserved zero. Reject non-canonical encodings.
        if (ext & 0x3F) or (hw & 0x1C):
            return raw_unknown(hw)
        sa = (ext >> 6) & 0x1F
    else:
        sa = ((hw >> 2) & 7) or 8  # encoded 000 means shift-by-8
    regs = f'{CREG[rx]}' if rx == ry else f'{CREG[rx]},{CREG[ry]}'
    return ext_mnem(ext, mnem), f'{regs},{sa}'


# -- op=0x08  RRI-A  addiu ry, rx, imm4 ---------------------------------
def _rri_a(hw: int, ext: Optional[int], pc: int) -> Decoded:
    if (hw >> 4) & 1:        # bit 4 must be 0 in RRI-A
        return raw_unknown(hw)
    v = ext_imm15_rri_a(ext, hw & 0xF) if ext is not None else sx(hw & 0xF, 4)
    return ('addiu32' if ext is not None else 'addiu'), f'{CREG[_ry(hw)]},{CREG[_rx(hw)]},{imm(v)}'


# -- op=0x09  ADDIU8  rx, imm8 ------------------------------------------
def _addiu8(hw: int, ext: Optional[int], pc: int) -> Decoded:
    if ext is not None:
        if not _require_ext_zero(hw, 0xE0):
            return raw_unknown(hw)
        v = ext_imm16_s(ext, hw & 0x1F)
        return 'addiu32', f'{CREG[_rx(hw)]},{imm(v)}'
    return 'addiu', f'{CREG[_rx(hw)]},{imm(sx(hw & 0xFF, 8))}'


# -- op=0x0A/0x0B  slti / sltiu -----------------------------------------
def _slti_like(mnem: str) -> Decoder:
    """slti / sltiu: unextended 8-bit zero-extended; extended 16-bit signed."""
    def inner(hw: int, ext: Optional[int], pc: int) -> Decoded:
        if ext is not None:
            if not _require_ext_zero(hw, 0xE0):
                return raw_unknown(hw)
            v = ext_imm16_s(ext, hw & 0x1F)
        else:
            v = hw & 0xFF
        return ext_mnem(ext, mnem), f'{CREG[_rx(hw)]},{imm(v)}'
    return inner

_slti  = _slti_like('slti')
_sltiu = _slti_like('sltiu')


# -- op=0x0C  i8: bteqz/btnez/sw-ra/adjsp/svrs/movr32/mov32r -----------
def _i8_btxz(mnem: str) -> Decoder:
    """bteqz / btnez: branch on T-flag."""
    def inner(hw: int, ext: Optional[int], pc: int) -> Decoded:
        if ext is not None:
            if not _require_ext_zero(hw, 0xE0):
                return raw_unknown(hw)
            tgt = pc + 4 + ext_imm16_s(ext, hw & 0x1F) * 2
        else:
            tgt = pc + 2 + sx(hw & 0xFF, 8) * 2
        return ext_mnem(ext, mnem), addr32(tgt)
    return inner

def _i8_sw_ra(hw: int, ext: Optional[int], pc: int) -> Decoded:
    if ext is not None:
        if not _require_ext_zero(hw, 0xE0):
            return raw_unknown(hw)
        off = ext_imm16_s(ext, hw & 0x1F)
        return 'swra32', f'ra,{imm(off)}(sp)'
    return 'sw', f'ra,{imm((hw & 0xFF) * 4)}(sp)'

def _i8_adjsp(hw: int, ext: Optional[int], pc: int) -> Decoded:
    if ext is not None:
        if not _require_ext_zero(hw, 0xE0):
            return raw_unknown(hw)
        return 'adjsp32', f'sp,{imm(ext_imm16_s(ext, hw & 0x1F))}'
    return 'addiu', f'sp,{imm(sx(hw & 0xFF, 8) * 8)}'

def _i8_movr32(hw: int, ext: Optional[int], pc: int) -> Decoded:
    """MOVR32 (i8 func=5): move r32, crY. Emitted as `move` unless the GPR is
    in the CREG subset, in which case the assembly would be ambiguous with the
    MOV32R encoding — use `movr32` so roundtrip picks the right func."""
    if hw == 0x6500 and ext is None:
        return 'nop', ''
    r32_idx = (hw & 0x18) | ((hw >> 5) & 7)
    mnem = 'movr32' if GPR[r32_idx] in CREG else 'move'
    return mnem, f'{GPR[r32_idx]},{CREG[hw & 7]}'

def _i8_mov32r(hw: int, ext: Optional[int], pc: int) -> Decoded:
    """MOV32R (i8 func=7): move crX, r32. Dual of _i8_movr32 — use `mov32r`
    when the GPR source is in CREG to disambiguate."""
    r32_idx = hw & 0x1F
    mnem = 'mov32r' if GPR[r32_idx] in CREG else 'move'
    return mnem, f'{CREG[(hw >> 5) & 7]},{GPR[r32_idx]}'

_I8_FUNCS: dict[int, Decoder] = {
    0: _i8_btxz('bteqz'),
    1: _i8_btxz('btnez'),
    2: _i8_sw_ra,
    3: _i8_adjsp,
    4: lambda hw, ext, pc: raw_unknown(hw),  # svrs only defined under EXTEND
    5: _i8_movr32,
    7: _i8_mov32r,
}

def _i8(hw: int, ext: Optional[int], pc: int) -> Decoded:
    func = (hw >> 8) & 7
    return _I8_FUNCS.get(func, lambda h, e, p: raw_unknown(h))(hw, ext, pc)


# -- op=0x0D / 0x0E  li / cmpi ------------------------------------------
def _li(hw: int, ext: Optional[int], pc: int) -> Decoded:
    if ext is not None:
        canonical = ext_imm16_u(ext, hw & 0x1F)
        # base[7:5] is the LUI-vs-LI selector on RX3081: 0 ⇒ zero-extend imm16
        # (plain LI32), 1 ⇒ imm16 << 16 (upper-half load used by la pairs).
        bm = (hw >> 5) & 7
        if bm == 0:
            return 'li32', f'{CREG[_rx(hw)]},0x{canonical:x}'
        if bm == 1:
            return 'li32', f'{CREG[_rx(hw)]},0x{(canonical << 16) & 0xFFFFFFFF:x}'
        return raw_unknown(hw)
    return 'li', f'{CREG[_rx(hw)]},0x{hw & 0xFF:x}'

def _cmpi(hw: int, ext: Optional[int], pc: int) -> Decoded:
    if ext is not None:
        if not _require_ext_zero(hw, 0xE0):
            return raw_unknown(hw)
        v = ext_imm16_s(ext, hw & 0x1F)
    else:
        v = hw & 0xFF
    return ext_mnem(ext, 'cmpi'), f'{CREG[_rx(hw)]},0x{v & 0xFFFF:x}'


# -- op=0x10..0x15, 0x18..0x1B  loads and stores with (base, off) form -
def _mem(mnem: str, scale: int) -> Decoder:
    """Generic MIPS16 load/store: bits 10:8 = base, bits 7:5 = data.
    Extended-form offset is always signed 16-bit regardless of data sign."""
    def inner(hw: int, ext: Optional[int], pc: int) -> Decoded:
        base = _rx(hw)
        data = _ry(hw)
        if ext is not None:
            off = ext_imm16_s(ext, hw & 0x1F)
        else:
            off = (hw & 0x1F) * scale
        return ext_mnem(ext, mnem), f'{CREG[data]},{imm(off)}({CREG[base]})'
    return inner


# -- op=0x12  LWSP;  op=0x1A  SWSP ---------------------------------------
def _sp_rel(mnem: str) -> Decoder:
    def inner(hw: int, ext: Optional[int], pc: int) -> Decoded:
        if ext is not None:
            if not _require_ext_zero(hw, 0xE0):
                return raw_unknown(hw)
            v = ext_imm16_s(ext, hw & 0x1F)
        else:
            v = (hw & 0xFF) * 4
        return ext_mnem(ext, mnem), f'{CREG[_rx(hw)]},{imm(v)}(sp)'
    return inner


# -- op=0x16  LWPC (PC-relative word load) -------------------------------
def _lwpc(hw: int, ext: Optional[int], pc: int) -> Decoded:
    if ext is not None:
        if not _require_ext_zero(hw, 0xE0):
            return raw_unknown(hw)
        v = ext_imm16_s(ext, hw & 0x1F)
    else:
        v = (hw & 0xFF) * 4
    return ext_mnem(ext, 'lwpc'), f'{CREG[_rx(hw)]},{addr32((pc & ~3) + v)}'


# -- op=0x17  CP0 / system: mfc0 / mtc0 / eret / sleep / di / ei / deret / cache
def _cp0(hw: int, ext: Optional[int], pc: int) -> Decoded:
    sub = (hw >> 8) & 7
    if sub == 0:
        return 'mfc0', f'{CREG[_ry(hw)]},{cp0_name(hw & 0x1F)}'
    if sub == 1:
        # cp0 register number is split: bits 4:3 contribute the high 2 bits,
        # bits 7:5 contribute the low 3. GPR source is in bits 2:0 (compact).
        cp0 = (hw & 0x18) | ((hw >> 5) & 7)
        return 'mtc0', f'{CREG[hw & 7]},{cp0_name(cp0)}'
    if sub == 2:
        sel = (hw >> 5) & 7
        low5 = hw & 0x1F
        if ext is not None:
            operand = str(ext_imm16_u(ext, low5))
        else:
            operand = str(low5) if low5 else ''
        if sel == 0: return 'eret',  operand
        if sel == 1: return 'sleep', operand
        if low5 == 0:
            if sel == 2: return 'di', ''
            if sel == 3: return 'ei', ''
            if sel == 7: return 'deret', ''
        return raw_unknown(hw)
    if sub == 3:
        return 'cache', f'{hw & 0x1F},0({CREG[_ry(hw)]})'
    return raw_unknown(hw)


# -- op=0x1C  RRR: addu / subu -------------------------------------------
_RRR_MNEMS = {1: 'addu', 3: 'subu'}

def _rrr(hw: int, ext: Optional[int], pc: int) -> Decoded:
    mnem = _RRR_MNEMS.get(hw & 3)
    if mnem is None:
        return raw_unknown(hw)
    rx, ry, rz = _rx(hw), _ry(hw), (hw >> 2) & 7
    if rz == rx:
        return mnem, f'{CREG[rx]},{CREG[ry]}'
    return mnem, f'{CREG[rz]},{CREG[rx]},{CREG[ry]}'


# -- op=0x1D  RR: reg-reg, compact jumps, save/restore, break, entry/exit
def _rr_func_2op(mnem: str) -> Decoder:
    return lambda hw, ext, pc: (mnem, f'{CREG[_rx(hw)]},{CREG[_ry(hw)]}')

def _rr_func_shift(mnem: str) -> Decoder:
    return lambda hw, ext, pc: (mnem, f'{CREG[_ry(hw)]},{CREG[_rx(hw)]}')

def _rr_func_div(mnem: str) -> Decoder:
    return lambda hw, ext, pc: (mnem, f'zero,{CREG[_rx(hw)]},{CREG[_ry(hw)]}')

def _rr_func_neg_not(mnem: str) -> Decoder:
    def inner(hw, ext, pc):
        rx, ry = _rx(hw), _ry(hw)
        regs = CREG[rx] if rx == ry else f'{CREG[rx]},{CREG[ry]}'
        return mnem, regs
    return inner

def _rr_sdbbp(hw: int, ext: Optional[int], pc: int) -> Decoded:
    return 'sdbbp', f'0x{(hw >> 5) & 0x3F:x}'

def _rr_break(hw: int, ext: Optional[int], pc: int) -> Decoded:
    code = (hw >> 5) & 0x3F
    return 'break', (f'0x{code:x}' if code else '')


_SAVE_LIST_S = {0: '', 1: 's0', 2: 's0-s1', 3: '??'}

def _save_list(bits_7_5: int) -> list[str]:
    """Decode the entry/exit save-list field (bits 7:5) into ordered parts."""
    ra = (bits_7_5 & 1) != 0
    sregs = _SAVE_LIST_S[bits_7_5 >> 1]
    parts: list[str] = []
    if sregs: parts.append(sregs)
    if ra: parts.append('ra')
    return parts


_ENTRY_ARG_LISTS = ['', 'a0', 'a0-a1', 'a0-a2', 'a0-a3']
_EXIT_FP_SUFFIX = {5: '$f0', 6: '$f0-$f1'}

def _rr_entry_exit(hw: int, ext: Optional[int], pc: int) -> Decoded:
    cnt = (hw >> 8) & 7
    sl = (hw >> 5) & 7
    if cnt <= 4:
        parts = []
        if _ENTRY_ARG_LISTS[cnt]:
            parts.append(_ENTRY_ARG_LISTS[cnt])
        parts.extend(_save_list(sl))
        return 'entry', ','.join(parts)
    parts = _save_list(sl)
    if cnt in _EXIT_FP_SUFFIX:
        parts.append(_EXIT_FP_SUFFIX[cnt])
    return 'exit', ','.join(parts)


# Ops whose mnem depends only on bits 10:8 (rx) — emitted by _rr with mask 0xf8ff.
_RR_RX_MASKED = {
    0xe800: lambda hw: ('jr',   CREG[_rx(hw)]),
    0xe840: lambda hw: ('jalr', CREG[_rx(hw)]),
    0xe810: lambda hw: ('mfhi', CREG[_rx(hw)]),
    0xe812: lambda hw: ('mflo', CREG[_rx(hw)]),
    0xe830: lambda hw: ('mthi', CREG[_rx(hw)]),
    0xe832: lambda hw: ('mtlo', CREG[_rx(hw)]),
}

# Ops keyed by bits 4:0 (func).
_RR_FUNCS: dict[int, Decoder] = {
    0x01: _rr_sdbbp,
    0x02: _rr_func_2op('slt'),
    0x03: _rr_func_2op('sltu'),
    0x04: _rr_func_shift('sllv'),
    0x05: _rr_break,
    0x06: _rr_func_shift('srlv'),
    0x07: _rr_func_shift('srav'),
    0x09: _rr_entry_exit,
    0x0A: _rr_func_2op('cmp'),
    0x0B: _rr_func_neg_not('neg'),
    0x0C: _rr_func_2op('and'),
    0x0D: _rr_func_2op('or'),
    0x0E: _rr_func_2op('xor'),
    0x0F: _rr_func_neg_not('not'),
    0x18: _rr_func_2op('mult'),
    0x19: _rr_func_2op('multu'),
    0x1A: _rr_func_div('div'),
    0x1B: _rr_func_div('divu'),
}

def _rr(hw: int, ext: Optional[int], pc: int) -> Decoded:
    # `jr ra` is the one exact-match case (rx field is forced to 0b100).
    if hw == 0xe820:
        return 'jr', 'ra'
    masked = _RR_RX_MASKED.get(hw & 0xf8ff)
    if masked is not None:
        return masked(hw)
    handler = _RR_FUNCS.get(hw & 0x1F)
    if handler is not None:
        return handler(hw, ext, pc)
    return raw_unknown(hw)


# -- op=0x1F  Lexra MAC --------------------------------------------------
_LEXRA_MAC = {
    0x00: 'madh', 0x02: 'madl', 0x04: 'mazh', 0x06: 'mazl',
    0x10: 'msbh', 0x12: 'msbl', 0x14: 'mszh', 0x16: 'mszl',
}

def _lexra(hw: int, ext: Optional[int], pc: int) -> Decoded:
    mnem = _LEXRA_MAC.get(hw & 0x1F)
    if mnem is None:
        return raw_unknown(hw)
    return mnem, f'{CREG[_rx(hw)]},{CREG[_ry(hw)]}'


# -- reserved / unallocated -----------------------------------------------
def _reserved(hw: int, ext: Optional[int], pc: int) -> Decoded:
    return raw_unknown(hw)


# ===========================================================================
# Top-level opcode dispatch
# ===========================================================================

OP_DECODERS: dict[int, Decoder] = {
    0x00: _addiusp,
    0x01: _la,
    0x02: _b,
    0x04: _beqz,
    0x05: _bnez,
    0x06: _shift,
    0x07: _reserved,
    0x08: _rri_a,
    0x09: _addiu8,
    0x0A: _slti,
    0x0B: _sltiu,
    0x0C: _i8,
    0x0D: _li,
    0x0E: _cmpi,
    0x0F: _reserved,
    0x10: _mem('lb',  1),
    0x11: _mem('lh',  2),
    0x12: _sp_rel('lw'),
    0x13: _mem('lw',  4),
    0x14: _mem('lbu', 1),
    0x15: _mem('lhu', 2),
    0x16: _lwpc,
    0x17: _cp0,
    0x18: _mem('sb',  1),
    0x19: _mem('sh',  2),
    0x1A: _sp_rel('sw'),
    0x1B: _mem('sw',  4),
    0x1C: _rrr,
    0x1D: _rr,
    0x1F: _lexra,
    # 0x03 (JAL/JALX) and 0x1E (EXTEND) are 4-byte, handled by `disassemble`.
}

# Ops that accept a leading EXTEND prefix. Register-only ops (1C, 1D, plus
# i8 funcs 5/7, CP0 mfc0/mtc0/cache, Lexra MAC) do not — an EXTEND before them
# renders as a standalone 2-byte "extend 0xN" followed by the base instruction.
_EXTENDABLE_OPS = frozenset({
    0x00, 0x01, 0x02, 0x04, 0x05, 0x06, 0x08, 0x09, 0x0A, 0x0B,
    0x0D, 0x0E, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x18, 0x19, 0x1A, 0x1B,
})


def _ext_allowed(hw: int, op: int) -> bool:
    if op in _EXTENDABLE_OPS:
        return True
    if op == 0x0C:
        # i8 funcs 0..4 (branches / sw ra / addiu sp / svrs) take an imm;
        # funcs 5/7 (MOVR32 / MOV32R) don't.
        return ((hw >> 8) & 7) < 5
    if op == 0x17:
        # Only sub=2 (eret/sleep/di/ei/deret) consumes the extended operand.
        # sub=0 (mfc0), sub=1 (mtc0), sub=3 (cache) ignore it.
        return ((hw >> 8) & 7) == 2
    return False


# Extended-form base halfwords have bits that the MIPS16e spec leaves
# reserved-zero. Every .text instance we've observed honors that; the only
# exception is extended LI, where RX3081 repurposes base[7:5]=1 to flag the
# LUI-style <<16 immediate. Everything else is blanket-rejected (yields the
# raw_unknown sentinel so the outer loop emits "extend 0xN" + raw-hex base
# and the roundtrip stays byte-exact for data regions that happen to decode
# as instructions).
def _require_ext_zero(hw: int, reserved_mask: int) -> bool:
    return (hw & reserved_mask) == 0


def decode(hw: int, ext: Optional[int], pc: int) -> Decoded:
    """Decode one MIPS16 halfword, optionally under an EXTEND prefix."""
    op = (hw >> 11) & 0x1F
    if ext is not None and not _ext_allowed(hw, op):
        return raw_unknown(hw)
    decoder = OP_DECODERS.get(op, _reserved)
    return decoder(hw, ext, pc)


# ===========================================================================
# Outer loop: resolve JAL/JALX and EXTEND prefixes, then dispatch to decode().
# ===========================================================================

def _decode_jal(hw: int, hw2: int, pc: int) -> Decoded:
    x = (hw >> 10) & 1
    imm26 = ((hw & 0x1F) << 21) | (((hw >> 5) & 0x1F) << 16) | hw2
    tgt = ((pc + 4) & 0xF0000000) | (imm26 << 2)
    return ('jalx' if x else 'jal'), addr32(tgt)


def disassemble(
    data: bytes,
    start: int = 0,
    vma: int = 0x85000000,
) -> Iterator[tuple[int, bytes, str, str]]:
    offset = 0
    n = len(data)
    while offset + 2 <= n:
        pc = (vma + start + offset) & 0xFFFFFFFF
        hw = raw16(data, offset)
        op = (hw >> 11) & 0x1F

        if op == 0x03 and offset + 4 <= n:
            mnem, args = _decode_jal(hw, raw16(data, offset + 2), pc)
            yield pc, bytes(data[offset:offset + 4]), mnem, args
            offset += 4
            continue

        if op == 0x1E and offset + 4 <= n:
            ext = hw & 0x7FF
            hw2 = raw16(data, offset + 2)
            # PC in extended-form formulas is the address of the EXTEND halfword.
            decoded = decode(hw2, ext, pc)
            if is_unknown(decoded):
                # Base halfword isn't extend-able — emit the EXTEND alone and
                # continue fresh from the next halfword.
                yield pc, bytes(data[offset:offset + 2]), 'extend', f'0x{ext:x}'
                offset += 2
            else:
                yield pc, bytes(data[offset:offset + 4]), decoded[0], decoded[1]
                offset += 4
            continue

        mnem, args = decode(hw, None, pc)
        yield pc, bytes(data[offset:offset + 2]), mnem, args
        offset += 2


# ===========================================================================
# CLI
# ===========================================================================

def _fmt_raw(raw: bytes) -> str:
    return ' '.join(
        f'{raw[i] | (raw[i + 1] << 8):04x}'
        for i in range(0, len(raw), 2)
    )


def _hex_arg(value: str) -> int:
    try:
        return int(value, 16)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f'invalid hex value: {value!r}') from exc


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Disassemble raw RX3081 MIPS16 little-endian firmware bytes.'
    )
    parser.add_argument('file', type=Path, help='raw binary to decode')
    parser.add_argument('start', nargs='?', type=_hex_arg, default=0,
                        help='start offset (hex)')
    parser.add_argument('stop', nargs='?', type=_hex_arg,
                        help='stop offset (hex, exclusive)')
    parser.add_argument('--vma', type=_hex_arg, default=0x85000000,
                        help='virtual load address base (hex, default: 0x85000000)')
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    data = args.file.read_bytes()
    start = args.start
    stop = len(data) if args.stop is None else args.stop
    if start < 0 or stop < start or stop > len(data):
        print(
            f'invalid range: start=0x{start:x}, stop=0x{stop:x}, size=0x{len(data):x}',
            file=sys.stderr,
        )
        sys.exit(2)
    if start & 1 or stop & 1:
        print(
            f'warning: odd range boundary start=0x{start:x}, stop=0x{stop:x}; '
            f'decoding stays halfword-based',
            file=sys.stderr,
        )
    for pc, raw, mnem, operands in disassemble(data[start:stop], start, args.vma):
        print(f'{pc:8x}:  {_fmt_raw(raw):<10}  {mnem:<8} {operands}')


if __name__ == '__main__':
    main()
