# RTD Scaler Reverse Engineering Notes

## Overview

This project is primarily a **firmware reverse-engineering effort** across two
different Realtek scaler families, with the dongle, ISP tooling, and support
scripts built to make that firmware work practical on real hardware.

Reference notes for the full stack: **2 boards, 2 chips, 1 dongle**.

- **RTD2775QT** — 8051 target
- **RTD2721C** — RX3081 / Lexra target
- **FX2LP USB-I2C** — access / programming / debug dongle

The two scaler chips share a common scaler architecture but run on entirely
different cores, so the main work here was reconstructing both firmware worlds
well enough to analyze, compare, and validate behavior across generations:

- **RTD2775QT**: banked 8051 firmware RE, static analysis, simulation, and
  bridge-assisted validation on hardware
- **RTD2721C**: RX3081 / Lexra firmware RE, custom disassembler/assembler, and
  mapping of its MMIO view back to the older 8051-style scaler model
- **FX2LP / ISP / host tooling**: supporting infrastructure built to extract,
  probe, validate, and operationalize the firmware work above

I also rebuilt the dongle firmware so the entire workflow is independent of the
original vendor tooling.

## Firmware RE Highlights

- Built a custom bank-aware 8051 firmware analyzer (`rtd_dis.py`) for the
  RTD2775QT, then paired it with an 8051 simulator / bridge (`rtd_mcu.py`) to
  validate understanding against real hardware.
- Built `lexra_dis.py` and `lexra_asm.py` from scratch for the RTD2721C's
  RX3081 / Lexra core, including byte-exact round-trip capability against real
  firmware images.
- Mapped the shared scaler register model across two very different CPU views:
  8051 logical `PP:RR` space on one side, Lexra MMIO on the other.
- Used the firmware work to recover bring-up, register flow, watchdog behavior,
  flash access behavior, and cross-generation architectural correspondence.
- Wrote the dongle firmware, ISP helpers, and register/flash tooling.

## Table of Contents

- [Firmware RE Highlights](#firmware-re-highlights)
- [Project Files](#project-files)
- [RTD2775QT (8051)](#rtd2775qt-8051)
- [RTD2721C (RX3081 / Lexra)](#rtd2721c-rx3081--lexra)
- [FX2LP USB-I2C Dongle](#fx2lp-usb-i2c-dongle)
- [Open-Source Dongle Firmware (openrtd)](#open-source-dongle-firmware-openrtd)
- [ISP Flash Protocol](#isp-flash-protocol)

## Project Files

| Path | Description |
|------|-------------|
| `lexra_dis.py` | RX3081 / Lexra disassembler |
| `lexra_asm.py` | RX3081 / Lexra assembler |
| `rtd_dis.py` | RTD 8051 banked-firmware disassembler |
| `rtd_mcu.py` | 8051 simulator with bridge/proxy mode |
| `rtd_scaler.py` | Shared scaler register access helpers across boards |
| `rtd_isp.py` | Shared RTD ISP interface across boards |
| `rtd_prog.py` | Shared flash programmer CLI across boards |
| `rtd_i2c.py` | Shared USB-I2C transport driver |
| `dump_scaler_regs.py` | Shared scaler-register dump / A-B diff tool |
| `edid_read.py` | EDID reader with E-DDC support |
| `i2c_scan.py` | I2C bus scanner |
| `parse_dbbin.py` | Parse DB.bin into machine-readable JSON |
| `fx2_dis.py` | FX2 firmware disassembler |
| `firmware/dongle/` | FX2LP USB-I2C firmware (`openrtd`) |
| `firmware/stub/` | RTD 8051 debug/stub firmware |

## RTD2775QT (8051)

### Core and tooling

- **Core**: 8051
- **Firmware tooling**: `rtd_dis.py` — a custom, mini-IDA-style static
  analyzer for the banked 8051 firmware — paired with `rtd_mcu.py`, a
  simulator with bridge/proxy mode.
- **Reverse-engineering output**: enough recovered structure to use the 8051
  firmware as the anchor reference for init sequencing, register mapping,
  watchdog behavior, and cross-checking newer scaler families.
- **Execution status**: `rtd_mcu.py` runs the actual firmware in Python while
  forwarding every scaler access to real silicon over the debug bridge — well
  enough to drive DPTx link training and render an OSD on a real panel.
- **Best use**: init/config work, register tracing, and debug-bridge workflows.

### Limits

- Wall-clock-sensitive behavior is not modeled faithfully.
- Timing-heavy paths still need real MCU execution.

### Mapping approach

- Use the 8051 firmware and tooling as the direct reference path for 2775QT work.
- For shared scaler architecture layers, distinctive register writes and call
  order can be used to align work against newer firmware targets.
- The 8051 view of scaler space is the logical `PP:RR` register map exposed
  through scaler remapping into XDATA.

### Stub firmware

Minimal SDCC C51 firmware implementing the RTD debug bridge protocol on DDC3.
This is part of the 8051 debug / simulation stack, not a separate target.

#### Debug Bridge Protocol (0x6A on DDC3)

| Command | Sub | Data | Action |
|---------|-----|------|--------|
| Stage page | 0xX1 | page | `g_data[2] = page` |
| Stage value | 0xX3 | val | `g_data[3] = val` |
| Halt/enter | 0x80 | 0x01 | Enter debug loop (buffer retains value for readback) |
| Exit | 0x80 | 0x00 | Exit debug loop |
| Read reg | 0x3A | reg | `DATA_OUT = XDATA[(page<<8)\|reg]` |
| Write reg | 0x3B | reg | `XDATA[(page<<8)\|reg] = staged_val` |
| XFR read | 0x41 | reg | Read via SCA_INF (sets page from staged value) |
| XFR write | 0xX0 | val | Write via SCA_INF (reads page from 0x9F) |
| SFR write | 0x5A | addr | Write staged value to MCU SFR port (P1=0x90, P3=0xB0 only) |
| SFR read | 0x5B | addr | Read MCU SFR port into DATA_OUT (P1=0x90, P3=0xB0 only) |

Host reads result with `read_byte(0x6A, 0x08)` — DATA_OUT buffer retains value.

#### RTD 8051 MCU Triple Watchdog

The RTD 8051 MCU has **three independent watchdog enable bits**. If any one of
them is set, the watchdog is enabled. The vendor firmware feeds it from the
Timer2 ISR. Without a Timer2 handler, all three must be explicitly disabled:

| Register | Bit | Name |
|----------|-----|------|
| 0xFFEA | [7] | `wdt_en` |
| 0xFFE9 | [7] | `wdt_en_3` |
| 0xFF3A | [6] | `wdt_en_2` |

At least one defaults to enabled at power-on. Failing to clear all three causes
periodic MCU resets that show up as random I2C NAKs. This is especially hard to
diagnose because the reset is fast enough that the MCU still appears to be
running normally.

## RTD2721C (RX3081 / Lexra)

### Core and tooling

- **Core**: RX3081, MIPS16-only, non-interlocked load-delay behavior.
- **Firmware tooling**: `lexra_dis.py` and `lexra_asm.py` — written from
  scratch for this effort, and to the best of my knowledge the only
  RX3081/Lexra disassembler and assembler pair with byte-exact round-trip
  capability against real firmware.
- **Reverse-engineering output**: a usable firmware analysis path for a poorly
  documented core, plus a bridge from Lexra MMIO behavior back to the older
  8051-oriented scaler model.

### Scaler register views

The same scaler register space is addressable two ways — the 8051 logical view
(used when mapping against 2775QT work) and the Lexra MMIO view (used when
reading RX3081 disassembly).

8051 logical view via scaler remapping:

```text
PP:RR
```

Lexra MMIO view:

```text
addr = 0xA0000000 + page * 0x400 + reg * 4
```

This is the key bridge from 8051 `Pxx_yy` register names and `PP:RR` register
access to Lexra MMIO addresses.

### Firmware layout

The firmware is loaded in two alias views of the same flash:

```text
0x85000000 + off
0x86000000 + off
```

In practice:

- Vectors are referenced through the `0x85` view.
- Main firmware is referenced through the `0x86` view.

That is the part that matters when reading disassembly.

## FX2LP USB-I2C Dongle

This section covers the support hardware and transport layer built around the
firmware RE work. It is included because it enabled extraction, validation, and
experimentation on real boards, not because the dongle itself was the main
target.

- **USB adapter**: Cypress CY7C68013A (FX2LP)-based — small dongle form factor (56-pin SSOP/QFN)
- **Default USB ID** (bootloader): `04b4:8613` — Cypress EZ-USB FX2
- **After firmware load**: `2007:0808` — Realtek USB ISP
- **FX2 has no flash** — firmware loaded into RAM every power cycle
- **Firmware source**: extractable from the original Windows host package
- **I2C implementation**: bit-banged on PA0 (SDA) and PA1 (SCL), NOT using FX2 hardware I2C engine
- **Target chip**: RTD scaler (ISP address 0x4A / 8-bit 0x94)

### Firmware Loading

FX2 boots into the ROM bootloader (`04b4:8613`), accepts firmware via USB, and
re-enumerates as `2007:0808`. The stock firmware can be extracted from the
original Windows host package as a raw binary, then converted to Intel HEX with
`srec_cat -address-length=2` (FX2 loaders reject type 4 extended address records).

### Stock FX2 firmware architecture

This section is based on disassembly produced with `fx2_dis.py` (custom 8051
disassembler).

### Memory Map

| Address | Description |
|---------|-------------|
| 0x0000 | Reset vector → LJMP 0x1FB6 (startup/init) |
| 0x0003-0x0041 | I2C bit-bang primitives (read byte, start, stop) |
| 0x0043 | INT2 vector → LJMP 0x1900 (USB autovector table) |
| 0x0053 | INT4 vector → LJMP 0x1900 |
| 0x1900-0x19B7 | USB autovector jump table (SUDAV, SOF, URES, etc.) |
| 0x17E5-0x189F | SETUP packet handler (standard USB ch9 requests) |
| 0x1C00-0x1C9F | USB descriptor data (device, config, string) |
| 0x1E91 | bRequest=0x01 handler (CLEAR_FEATURE) |
| 0x1FB6 | Startup code (init RAM, jump to main) |
| 0x2042-0x20C9 | **Main command dispatcher** (reads EP4 OUT buffer) |
| 0x22C8-0x2334 | USB/endpoint initialization |
| 0x2334-0x233F | **Main loop** |
| 0x2519-0x258B | Response writer (builds EP8 IN packet with checksum) |
| 0x258C-0x25F7 | Post-write clock-stretch check |
| 0x2EBB | bRequest=0x00 handler (GET_STATUS) |
| 0x309F | SUDAV ISR (sets flag bit 0x00) |

### Main Loop (0x2334)

```text
loop:
    if SUDAV_flag:
        handle_setup_packet()    ; 0x17E5 — standard USB ch9
        clear flag
    process_command()            ; 0x2042 — EP4 command dispatch
    goto loop
```

### Command Dispatch (0x2042)

Reads command byte from XDATA `0xF400` (EP4 OUT buffer), subtracts `0x11`,
dispatches through jump table. Note: jump table entry order differs from
LCALL address order in memory (entries 2 and 3 cross over).

| Cmd | Handler | Csum N | Role | Function |
|-----|---------|--------|------|----------|
| 0x11 | 0x293B | 5 | ISP read | I2C read, 1-byte sub: [slave, sub, len_hi, len_lo] → data |
| 0x12 | 0x29E7 | 5 | ISP write | I2C write, 1-byte sub: [slave, sub, len_hi, len_lo, data...] |
| 0x13 | 0x2991 | 6 | Register probe | I2C read, 2-byte sub, single-phase: [slave, sub_hi, sub_lo, len_hi, len_lo] → data |
| 0x14 | 0x27DB | len+6 | Inline edit | I2C write, 1-byte sub, inline variant: [slave, sub, len_hi, len_lo, data...] |
| 0x15 | 0x2BC5 | 4 | Pipe streaming | I2C current-address read (no sub): [slave, len_hi, len_lo] → data |
| 0x16 | 0x2DA8 | 3 | Config | Set I2C clock divider: [div_hi, div_lo] |
| 0x17 | 0x11C5 | 6 | Config | Configure timing mode 1 (default): [mode, speed_b3..b0] → computes delay ticks |
| 0x18 | 0x1F28 | 6 | Config | Configure timing mode 2 (override): [mode, ticks_b3..b0] → raw delay tick override |
| 0x19 | 0x31FA | — | Control | USB re-enumerate / firmware reset |
| 0x1A | 0x25F8 | 7 | Wide read | I2C read, 2-byte sub, 3-phase: [slave, sub_hi, sub_lo, pad, len_hi, len_lo] → data |
| 0x1B | 0x265D | len+7 | Wide write | I2C write, 2-byte sub, 3-phase: [slave, sub_hi, sub_lo, pad, len_hi, len_lo, data...] |

### I2C Transaction Sequences

The firmware implements four distinct I2C read sequences and three write sequences.
All use bit-bang I2C via function pointer tables (different routines per speed mode).

**Function pointer table:**

| Pointer | Function |
|---------|----------|
| 0x50:0x51 | i2c_start / repeated_start |
| 0x44:0x45 | i2c_stop |
| 0x4A:0x4B | i2c_write_byte (send byte, check ACK) |
| 0x47:0x48 | i2c_read_byte_ack (read byte, send ACK) |
| 0x4D:0x4E | i2c_read_byte_nak (read byte, send NAK — last byte) |

**Read sequences:**

CMD 0x11 — ISP register read (1-byte sub, standard I2C random read):
```text
START → slave → sub → RE-START → slave|0x01 → read N → STOP
```

CMD 0x13 — Register probe (2-byte sub, single-phase, standard 16-bit register read):
```text
START → slave → sub_hi → sub_lo → RE-START → slave|0x01 → read N → STOP
```
Handler 0x2991 → function 0x158D. Both sub-address bytes sent to the same
slave in one write phase, then repeated-start for read. Used for reading
extended 16-bit register space (scaler config, version info).

CMD 0x1A — Wide read (2-byte sub, 3-phase with device probing):
```text
START → slave → sub_hi → RE-START → sub_lo → pad → RE-START → slave|0x01 → read N → STOP
```
Handler 0x25F8 → function 0x13CB. After the first RE-START, sub_lo is sent
as a bus address byte. Devices that don't support extended addressing NAK at
phase 2 — acts as a capability probe before committing to the read.

CMD 0x15 — Pipe streaming (current-address read, no sub):
```text
START → slave|0x01 → read N → STOP
```
No address overhead — burst-reads from wherever the device pointer is.
Useful for streaming data from SPI buffer register 0x70 after setting
the address once with CMD 0x11.

**Write sequences:**

CMD 0x12 — ISP register write (1-byte sub):
```text
START → slave → sub → data[0..N-1] → STOP
```
Followed by post-write clock-stretch check (0x258C). Toggles SCL repeatedly
after STOP — if slave never stretches (e.g. write-protected EDID), returns
0x02 (WNAK) indicating write was silently discarded.

CMD 0x14 — Inline edit (1-byte sub, inline variant):
```text
START → slave → sub → data[0..N-1] → STOP
```
Same wire sequence as CMD 0x12, different packet format. Data payload is
embedded inline in the command packet for quick register patches.

CMD 0x1B — Wide write (2-byte sub, 3-phase):
```text
START → slave → sub_hi → RE-START → sub_lo → pad → data[0..N-1] → STOP
```
Handler 0x265D → function 0x1662. Same 3-phase addressing as CMD 0x1A.
Phase 2 sends sub_lo as bus address for device probing. Also has post-write
clock-stretch check.

**Design notes:**

- No single-phase 2-byte sub write exists (counterpart to CMD 0x13). The RTD
  scaler ISP protocol uses 1-byte register addressing (CMD 0x12) for all writes.
  Flash writes go through the ISP register interface, not direct 16-bit writes.
- The 3-phase commands (0x1A/0x1B) have a padding byte in the packet that is
  sent as 0x00 after sub_lo in phase 2.

### Endpoint Configuration (set at 0x26C1)

| Register | Value | Endpoint |
|----------|-------|----------|
| EP1OUTCFG (0xE610) | 0xA0 | Valid, OUT, Bulk, single-buf |
| EP1INCFG (0xE611) | 0xA0 | Valid, IN, Bulk, single-buf |
| EP2CFG (0xE612) | 0xA2 | Valid, OUT, Bulk, double-buf |
| EP4CFG (0xE613) | 0xA0 | Valid, OUT, Bulk, single-buf |
| EP6CFG (0xE614) | 0xE2 | Valid, IN, Bulk, double-buf |
| EP8CFG (0xE615) | 0xE0 | Valid, IN, Bulk, single-buf |

### Endpoint Buffer Addresses (FX2 fixed map)

| Buffer | Address | Used for |
|--------|---------|----------|
| EP0 | 0xE740 | Control transfers |
| EP2 | 0xF000-0xF3FF | OUT (1K) |
| EP4 | 0xF400-0xF7FF | **Command input from host** |
| EP6 | 0xF800-0xFBFF | IN (1K) |
| EP8 | 0xFC00-0xFFFF | **Response/data output to host** |

### I2C Bit-Bang Implementation

- **SDA**: PA0 (IOA bit 0, SFR 0x80)
- **SCL**: PA1 (IOA bit 1, SFR 0x80)
- **OEA** (SFR 0xB2): output enable register, NOT bit-addressable
- **Pin control**: open-drain via OEA toggling. To release (high): set IOA latch
  to 1, then clear OEA bit (input, external pull-up). To drive low: set OEA bit
  (output), then clear IOA latch. IOA latch must be 1 before switching to input
  mode — FX2 reads latch value when pin is input, not actual pin state, unless
  latch is pre-set high.
- **EP2468STAT bit 7** (`bmEP8FULL`, SFR `0xAA`): checked at start of every command
  handler in the stock firmware. If EP8 IN is full (host has not yet drained prior
  response data, or no IN buffer is available), the firmware avoids starting a new
  command that would need to queue another response.
- Timing controlled by two independent mechanisms:
  - **CMD 0x17/0x18** (mode select): speed value → computed ticks at 0x3D-0x40.
    Mode 1 (CMD 0x17) computes from speed, Mode 2 (CMD 0x18) uses raw ticks.
    Mode selected by RAM 0x29.
  - **CMD 0x16** (clock divider): stores 16-bit value in RAM 0x41:0x42. A lookup
    function at 0x0A35 range-checks the divider into one of 10 speed buckets and
    loads the corresponding **function pointer set** into RAM 0x43-0x4E, plus sets
    0x41:0x42 to the bucket's canonical value for the conditional NOP checks.

#### FX2 I2C Clock Stretching

The vendor firmware (realsil.hex) does NOT check for clock stretching in the
basic bit-bang write_byte/read_byte — those are pure push-pull. SCL is only
checked in specific higher-level routines (write-verify post-STOP detection).
The OpenRTD firmware adds SCL release + poll on START/STOP/RESTART only, not
per-bit. In practice, the USB round-trip latency between transactions provides
enough implicit delay for the scaler to release SCL before the next START.

#### CMD 0x16 Speed Buckets (0x0A35)

The lookup function is a cascade of 16-bit range comparisons (`SUBB` + `JC`/`JNC`).
Each bucket stores a canonical divider in 0x41:0x42 and loads 4 sets of 3-byte
function pointers into RAM 0x43-0x4E (different I2C primitive variants per speed):

| Divider (0x41:0x42) | Decimal | Speed class |
|---------------------|---------|-------------|
| 0x0226 | 550 | Fastest |
| 0x01F4 | 500 | |
| 0x01C2 | 450 | |
| 0x0190 | 400 | |
| 0x015E | 350 | |
| 0x012C | 300 | |
| 0x00FA | 250 | |
| 0x00C8 | 200 | Default host-side setting |
| 0x0096 | 150 | |
| 0x0064 | 100 | Slowest |

#### Conditional NOP Delay (I2C primitives at 0x0003-0x0180)

The I2C primitives use the canonical 0x41:0x42 value directly — they do **inline
equality checks** (`XRL`+`ORL`+`JZ`/`JNZ`) against known divider constants to
conditionally insert NOPs. Pattern per delay point:

```asm
MOV  A,0x42          ; load divider low byte
XRL  A,#0x5E         ; check == 0x015E?
JNZ  skip_hi
MOV  A,0x41
XRL  A,#0x01
JZ   fast_path       ; divider == 350 → skip all extra NOPs

MOV  A,0x42
ORL  A,0x41
JZ   fast_path       ; divider == 0 → skip (unused/init)

MOV  A,0x42
XRL  A,#0x64
ORL  A,0x41
JNZ  continue        ; divider != 100 → skip NOP
NOP                  ; divider == 100 → 1 extra NOP
continue:
NOP                  ; 4-5 fixed NOPs
NOP
NOP
NOP
SETB P0.1            ; SCL transition
```

The effective delay per half-bit is: ~8-12 cycles of compare overhead + 0-1
conditional NOPs + 4-5 fixed NOPs. The compare logic itself consumes more
cycles than the NOPs it gates, limiting the achievable speed range.

This is why the stock firmware is only marginally faster than OpenRTD's JIT NOP
sled approach, which provides continuous delay tuning with no compare overhead.

### Response Writer (0x2519)

Writes a response to the EP8 IN buffer (0xFC00):

1. Zero-fills buffer up to offset R5:R4
2. Writes status/result byte at position [R5:R4]
3. Computes additive checksum of bytes [0..N]
4. Writes checksum at position [R5:R4 + 1]
5. Arms EP8BCL (0xE69C/0xE69D) with total length

After each command, re-arms EP4 OUT (writes 0x80 to EP4BCL at 0xE695).

### Transport Layer

All commands use **bulk transfers**, not vendor control transfers:
- **Host → Device**: EP4 OUT (0x04)
- **Device → Host**: EP8 IN (0x88)

### Packet Format

**Command (EP4 OUT):**
```text
[cmd_byte] [param1] [param2] ... [paramN] [checksum]
checksum = sum(all_preceding_bytes) & 0xFF
```

**Response (EP8 IN):**
```text
[data × N] [status] [checksum]
status: 0x00 = success, nonzero = error
checksum = sum(all_preceding_bytes) & 0xFF
```

### I2C Commands

**CMD 0x11 — I2C Read (ISP register read):**
```text
OUT: [0x11] [slave_addr] [sub_addr] [len_hi] [len_lo] [csum]
IN:  [data × len] [status] [csum]
```
**Important:** `slave_addr` must be the **write** address (for example, `0xA0`
for EDID or `0x94` for the RTD scaler). The firmware internally ORs `0x01` to
create the read address for the I2C read phase. Sending the read address
(`0xA1`/`0x95`) will target the wrong device.

**CMD 0x12 — I2C Write (ISP register write):**
```text
OUT: [0x12] [slave_addr] [sub_addr] [len_hi] [len_lo] [data × len] [csum]
IN:  [status] [csum]
```

**CMD 0x13 — I2C Read, 2-byte sub (register probe):**
```text
OUT: [0x13] [slave_addr] [sub_hi] [sub_lo] [len_hi] [len_lo] [csum]
IN:  [data × len] [status] [csum]
```
Single-phase 16-bit register read. Used for probing extended register space.

**CMD 0x14 — I2C Write, inline variant (inline edit):**
```text
OUT: [0x14] [slave_addr] [sub_addr] [len_hi] [len_lo] [data × len] [csum]
IN:  [status] [csum]
```
Same wire sequence as CMD 0x12, different packet layout for inline payloads.

**CMD 0x15 — Current-address read (pipe streaming):**
```text
OUT: [0x15] [slave_addr] [len_hi] [len_lo] [csum]
IN:  [data × len] [status] [csum]
```

**CMD 0x16 — Set I2C Clock Divider:**
```text
OUT: [0x16] [divider_hi] [divider_lo] [csum]
IN:  [status] [csum]
Default divider: 0x00C8 (200 → ~100kHz)
```

**CMD 0x17 — Configure Timing (Mode 1, default — computed):**
```text
OUT: [0x17] [mode] [speed_b3] [speed_b2] [speed_b1] [speed_b0] [csum]
IN:  [status] [csum]
Default: [0x17, 0x01, 0x00, 0x01, 0x86, 0xA0, csum]  (mode=1, speed=100000)
```
Stores mode → RAM 0x3C, speed → RAM 0x25-0x28, then computes delay tick
constants via 32-bit divides → RAM 0x21-0x24 (delay set 1), 0x3D-0x40 (delay
set 2, clamped min=5). This is the default path — when RAM 0x29 != 1 (i.e. 0
at boot, or set to 2 explicitly), the bit-bang loops use 0x3D-0x40.

**CMD 0x18 — Configure Timing (Mode 2, override — raw ticks):**
```text
OUT: [0x18] [mode] [ticks_b3] [ticks_b2] [ticks_b1] [ticks_b0] [csum]
IN:  [status] [csum]
```
Stores mode → RAM 0x29 (mode selector), raw ticks → RAM 0x2A-0x2D. No math —
values are used directly as delay loop counters. When RAM 0x29 == 1, the
bit-bang loops use 0x2A-0x2D instead of 0x3D-0x40.

**Mode selector (RAM 0x29):** Checked by all I2C command handlers to choose
which delay constants to load into the active timing registers (0x1A-0x1D):
- Mode 1 (default, 0x29 != 1): copies 0x3D-0x40 (CMD 0x17 computed ticks)
- Mode 2 (override, 0x29 == 1): copies 0x2A-0x2D (CMD 0x18 raw ticks)

**CMD 0x1A — I2C Read, 2-byte sub, 3-phase (wide read):**
```text
OUT: [0x1A] [slave_addr] [sub_hi] [sub_lo] [pad=0x00] [len_hi] [len_lo] [csum]
IN:  [data × len] [status] [csum]
```

**CMD 0x1B — I2C Write, 2-byte sub, 3-phase (wide write):**
```text
OUT: [0x1B] [slave_addr] [sub_hi] [sub_lo] [pad=0x00] [len_hi] [len_lo] [data × len] [csum]
IN:  [status] [csum]
```

### Initialization Sequence

1. Open USB device
2. Send CMD 0x17 with mode=0x01, speed=100000 → computes delay ticks, sets mode
3. Send CMD 0x16 with divider=0x00C8 → sets clock divider
4. **Must be done before any I2C read/write**

Note: the original host software always uses CMD 0x17 (computed ticks). CMD
0x18 (raw tick override) is available as a lower-level alternative with the
same effect, but you supply delay-loop counts directly instead of a speed value.

### Legacy host API surface

| Export | Address | Notes |
|--------|---------|-------|
| InitialDev | 0x10001150 | Opens USB + sends CMD 0x17/0x16 |
| I2CWrite | 0x10001250 | Wrapper → 0x10008150 (builds CMD 0x12) |
| I2CRead | 0x100012A0 | Wrapper → 0x10008330 (builds CMD 0x11) |
| I2CWriteByte | 0x100012F0 | Single-byte write variant |
| I2CReadSegment | 0x10002190 | Multi-segment read |
| SetI2CSpeed | 0x10001750 | Stores speed param |
| GetI2CSpeed | 0x10001770 | Returns speed param |
| ResetFirmware | 0x10001850 | Resets FX2 |

Many exports are stubs (IspEnable, ReadFlashData, WriteFlashData, etc.) — the
real ISP logic lives in a separate host-side plugin layer.

### Host-side error codes

| Code | Meaning |
|------|---------|
| 0xB02 | Invalid parameter (null pointer, zero length) |
| 0xB03 | USB write (EP4 OUT) failed |
| 0xB04 | Incomplete transfer (byte count mismatch) |
| 0xB05 | USB read (EP8 IN) failed |
| 0xB07 | Checksum mismatch |
| 0xB0D+N | Device error (firmware returned status=N) |

### Historical interface notes

| Offset | Function |
|--------|----------|
| +0x30 | I2CRead (multi-byte, paged) |
| +0x34 | I2CRead variant |
| +0x6C | GetDeviceType (returns 8 for USB-I2C) |
| +0x90 | Pre-transfer check |
| +0xE8 | I2CWriteByte(value, register) |
| +0xF0 | I2CReadByte(register, &result) |
| +0xF4 | I2CWriteByte variant |
| +0x188 | NativeWrite |
| +0x18C | NativeRead / direct USB |

## Open-Source Dongle Firmware (openrtd)

Drop-in replacement for the stock FX2LP firmware. It is protocol-compatible
with all original commands (0x11-0x1B) and adds CMD 0x1C/0x1D for E-DDC segment
read/write (full EDID beyond 256 bytes). Built with SDCC + fx2lib.

Performance matches stock firmware for I2C (~9.9ms vs ~9.5ms for 128-byte EDID
read) and is ~3× faster for 1MB flash writes. Uses a JIT NOP-sled trick:
`i2c_set_delay()` writes N NOP opcodes + RET into executable RAM, replacing the
traditional DJNZ delay loop (1 cycle/NOP vs 3 cycles/iteration).

### Pinout

Connect to the RTD scaler's HDMI DDC / I2C debug header:

```text
FX2LP Board          HDMI / Scaler
-----------          -------------
PA0 (Port A bit 0)   SDA (HDMI pin 16, DDC data)
PA1 (Port A bit 1)   SCL (HDMI pin 15, DDC clock)
GND                  GND (HDMI pin 17, DDC ground)
```

The FX2LP's PA0/PA1 are directly connected — no level shifters needed for
3.3V I2C. External pull-up resistors (4.7K to 3.3V) are required on both
SDA and SCL if not already present on the target board.

On common FX2LP dev boards (e.g. the small blue "EZ-USB FX2LP CY7C68013A"
boards from AliExpress):

- **PA0** = pin labeled `SDA` or `D0` on the header
- **PA1** = pin labeled `SCL` or `D1` on the header
- Some boards break out Port A on a separate header row

### Building

```bash
cd firmware/dongle
make          # builds build/openrtd.ihx
make flash    # loads onto FX2LP via cycfx2prog
```

Requires: `sdcc`, `sdas8051`, `fx2lib` (cloned into `./fx2lib/`).

### New Command: CMD 0x1C — E-DDC Segment Read

```text
OUT: [0x1C] [segment] [slave_addr] [offset] [len_hi] [len_lo] [csum]
IN:  [data × len] [status] [csum]
```

Wire sequence (per VESA E-DDC 1.3):

```text
START → 0x60 → segment → RE-START → slave → offset → RE-START → slave|0x01 → read N → STOP
```

Segment pointer address 0x60 is fixed per spec. Segment 0 = bytes 0-255,
segment 1 = bytes 256-511, etc. Required for monitors with >256 bytes of
EDID (CTA-861 extensions, DisplayID blocks).

Cannot be done with stock firmware — E-DDC requires two repeated starts
between three different slave addresses in a single I2C transaction.

### New Command: CMD 0x1D — E-DDC Segment Write

```text
OUT: [0x1D] [segment] [slave_addr] [offset] [len_hi] [len_lo] [data...] [csum]
IN:  [status] [csum]
```

Wire sequence:

```text
START → 0x60 → segment → RE-START → slave → offset → data[0..N-1] → STOP
```

Post-write clock-stretch check detects silently discarded writes (returns 0x02).

### I2C Status Codes

| Code | Meaning |
|------|---------|
| 0x00 | Success |
| 0x01 | I2C NAK (device not responding / wrong addressing mode) |
| 0x02 | Write rejected (post-write clock-stretch check timed out — slave never stretched) |
| 0x03 | Invalid parameter (length exceeds buffer size) |
| 0x04 | Hardware not ready (I2C timing init failed) |
| 0x05 | Checksum verification failed |
| 0x06 | Unknown/invalid command (cmd byte outside supported range) |

### FX2LP Hardware Gotchas

Two quirks that cost real debugging time during FX2LP firmware work — both
silent, both barely documented, and both easy to trip over.

**SUDPTRH:L requires word-aligned (even) addresses.** The FX2 Setup Data
Pointer registers (SUDPTRH:L), used to auto-serve USB descriptors in response
to GET_DESCRIPTOR requests, will silently corrupt the descriptor transfer if
the target address is odd. The host sees error -71 (EPROTO) on the device
descriptor read — the SIE sends garbage. There is NO error indication on the
device side.

This is documented in a single sentence in the EZ-USB FX2 TRM (Section 15,
SUDPTRH/L register description): "This buffer is used as a target or source by
the Setup Data Pointer and it must be WORD (2-byte) aligned."

fx2lib's `dscr.a51` uses `.even` before every descriptor, so the stock
descriptor tables are always safe. Any descriptor defined in C code (e.g.
`static __code BYTE desc[]`) lands in SDCC's CONST segment at whatever address
the linker assigns — which MAY BE ODD depending on total code size. The fix is
to define descriptors in assembly with `.even`, or place them in DSCR_AREA.

Symptoms: USB enumeration works on one build, then fails with `-71` on another
build that differs by a single byte of code. Adding or removing any code shifts the
CONST segment and flips the descriptor between even/odd alignment. Extremely
confusing to debug because the 8051 architecture has no alignment requirements
whatsoever — this is a quirk of the FX2's USB SIE hardware, not the CPU.

**Direct-addressed variables in code space hit SFRs.** The 8051 `MOV A, direct`
instruction uses only the low byte of the address. If a variable is declared
in the CSEG (CODE) area and accessed via direct addressing (e.g.
`MOV A, _myvar`), the assembler uses the low byte of the CODE address as the
direct address. On 8051, direct addresses 0x80-0xFF map to SFRs, not RAM. If
`_myvar` happens to link at e.g. 0x10B3, the instruction reads SFR 0xB3 (OEB
on FX2LP) instead of the variable. This is silent — no assembler or linker
warning. The variable appears to work until a code change shifts it to a
different SFR and everything breaks in unrelated ways. Fix: put variables in
DSEG (`.area DSEG (DATA)`), not CSEG, even in assembly files.

## ISP Flash Protocol

This is supporting board-access infrastructure for firmware dumping, recovery,
and validation. It mattered operationally, but it sits downstream of the core
firmware reverse-engineering work.

**These boards are effectively unbrickable.** ISP lives in the scaler silicon
and is independent of firmware — it is reachable as long as the chip has power,
regardless of what's on the flash. Even in the worst case (firmware that
disables the ISP I2C slave, or a WP# line that refuses to de-assert), recovery
is just: short the flash pins to prevent a valid boot, then race the scaler's
boot loop and enter ISP before anything else runs.

### RTD ISP Register Map

All registers accessed via I2C write/read to slave address 0x94/0x95 with the
register number as sub-address. Two separate interfaces to the same SPI controller:

#### Common Instruction Interface

For standalone SPI commands (JEDEC ID, erase, status register, etc.).

| Register | Name | R/W | Description |
|----------|------|-----|-------------|
| 0x60 | common_inst_en | R/W | Control register (see bit-field below) |
| 0x61 | common_op_code | W | SPI opcode |
| 0x62 | wren_op_code | W | WREN opcode (default 0x06) |
| 0x63 | ewsr_op_code | W | EWSR opcode (default 0x50) |
| 0x67 | common_inst_read_port0 | R | Read-back data byte 0 / high [23:16] |
| 0x68 | common_inst_read_port1 | R | Read-back data byte 1 / middle [15:8] |
| 0x69 | common_inst_read_port2 | R | Read-back data byte 2 / low [7:0] |

**Register 0x60 (common_inst_en) bit-field:**

| Bits | Field | Description |
|------|-------|-------------|
| 7:5 | com_inst | 000=nop, 001=write, 010=read, 011=write_after_WREN, 100=write_after_EWSR, 101=erase |
| 4:3 | write_num | Address/write byte count (0-3) |
| 2:1 | rd_num | Read-back byte count (0-3) |
| 0 | com_inst_en | Trigger — set 1 to execute, auto-clears on completion |

Sequence: write setup (enable=0) → opcode to 0x61 → address to 0x64-66 → trigger
(enable=1) → poll 0x60 bit 0. Address uses the shared 0x64-66 registers.
Read-back data appears in 0x67-69 (read-only output ports).

WREN is hardware-managed: com_inst types write_after_WREN (011) and erase (101)
automatically send the WREN opcode from register 0x62 before the SPI command.

**Known 0x60 value pairs (setup, trigger):**

| Pair | Encoding | Used for |
|------|----------|----------|
| 0x46/0x47 | read, wn=0, rn=3 | JEDEC ID (0x9F) |
| 0x42/0x43 | read, wn=0, rn=1 | Status register read (0x05) |
| 0x5A/0x5B | read, wn=3, rn=1 | Read with 3-byte addr (SFDP, Release PD) |
| 0x5C/0x5D | read, wn=3, rn=2 | Read Manufacturer/Device ID (0x90) |
| 0x38/0x39 | write, wn=3, rn=0 | SST/MX byte/page program |
| 0x68/0x69 | write_after_WREN, wn=1, rn=0 | Write status register (1 byte) |
| 0x70/0x71 | write_after_WREN, wn=2, rn=0 | Write status register (2 bytes) |
| 0xA0/0xA1 | erase, wn=0, rn=0 | Chip erase (no address) |
| 0xB8/0xB9 | erase, wn=3, rn=0 | Sector/block erase (with address) |

#### Program Engine

For bulk flash read, page program, and CRC. Register names from RTD2660 datasheet.

| Register | Name | R/W | Description |
|----------|------|-----|-------------|
| 0x64 | flash_prog_isp0 | R/W | Flash address byte 2 [23:16] (shared with common instruction) |
| 0x65 | flash_prog_isp1 | R/W | Flash address byte 1 [15:8] |
| 0x66 | flash_prog_isp2 | R/W | Flash address byte 0 [7:0] |
| 0x6A | read_op_code | W | SPI Read opcode (default 0x03) |
| 0x6B | fast_read_op_code | W | SPI Fast Read opcode (default 0x0B) |
| 0x6C | read_instruction | W | SPI read mode / timing config |
| 0x6D | program_op_code | W | SPI Page Program opcode (default 0x02) |
| 0x6E | read_status_register_op_code | W | SPI RDSR opcode (default 0x05) |
| 0x6F | program_instruction | R/W | Program instruction register (see bits below) |
| 0x70 | program_data_port | R/W | SPI data FIFO (read/write up to 256 bytes) |
| 0x71 | program_length | W | Page program byte count (value = length - 1) |
| 0x72 | CRC_end_addr0 | W | CRC end address byte 2 [23:16] |
| 0x73 | CRC_end_addr1 | W | CRC end address byte 1 [15:8] |
| 0x74 | CRC_end_addr2 | W | CRC end address byte 0 [7:0] |
| 0x75 | CRC_result | R | CRC-8 result (poly 0x07). Known: 4K erased=0x09, 64K=0xDE, 2M=0xF3 |

**Register 0x6F (program_instruction) bit-field:**

| Bit | Name | R/W | Default | Description |
|-----|------|-----|---------|-------------|
| 7 | isp_en | R/W | 0 | Enable ISP (gates 8051 clock). Required on all chips — must be preserved in all 0x6F writes via read-modify-write. Without isp_en, SPI registers are inaccessible. Clearing it directly hangs the board; use SOF_RST (0xEE bit 1) to exit ISP cleanly. |
| 6 | prog_mode | R/W | 0 | 0=normal, 1=AAI mode |
| 5 | prog_en | R/W | 0 | Program start — auto-clears on completion |
| 4 | prog_buf_wr_en | R | 1 | SRAM buffer ready for write data |
| 3 | prog_dummy | R/W | 0 | — |
| 2 | crc_start | R/W | 0 | Trigger CRC — auto-clears when crc_done set |
| 1 | crc_done | R | 1 | CRC calculation complete |
| 0 | rst_flash_ctrl | R/W | 0 | Software reset flash controller |

### ISP Initialization

ISP entry is always via 0x6F bit 7 (`isp_en`) on all chips tested. The original
host-side 0x6D write (originally assumed to be the ISP enabler) is just setting the
program opcode — isp_en is the actual gate. Without it, JEDEC reads fail.

```python
I2CWriteByte(0x80, 0x6F)   # isp_en — enters ISP (required on all chips)
sleep(100ms)
I2CWriteByte(0x06, 0x62)   # wren_op_code
I2CWriteByte(0x50, 0x63)   # ewsr_op_code
I2CWriteByte(0x0B, 0x6B)   # fast_read_op_code
I2CWriteByte(0x00, 0x6C)   # read_instruction
I2CWriteByte(0x02, 0x6D)   # program_op_code
I2CWriteByte(0x05, 0x6E)   # read_status_register_op_code
I2CWriteByte(0x84, 0xED)   # MCU_control: FLASH_CLK_DIV=1 (faster SPI clock)
I2CWriteByte(0x04, 0xEE)   # MCU_clock_control: MCU_CLK_DIV=1
```

Exit: SOF_RST via read-modify-write on 0xEE (set bit 1). This resets the
MCU which clears isp_en internally. Do NOT clear 0x6F directly — writing
0x00 to 0x6F hangs the board even with valid flash content.

All writes to 0x6F must use read-modify-write to preserve `isp_en` and the R/O
status bits (`prog_buf_wr_en`, `crc_done`). The host-side implementation reads
0x6F before every write.

### ISP Sequences

**Read JEDEC ID** (common instruction):
```python
I2CWriteByte(0x46, 0x60)       # setup: read, rn=3
I2CWriteByte(0x9F, 0x61)       # JEDEC ID opcode
I2CWriteByte(0x47, 0x60)       # trigger
poll 0x60 until bit 0 clear
mfr = I2CReadByte(0x67)
typ = I2CReadByte(0x68)
cap = I2CReadByte(0x69)
```

**Chip Erase** (common instruction):
```python
I2CWriteByte(0xA0, 0x60)       # setup: erase, wn=0
I2CWriteByte(0xC7, 0x61)       # chip erase opcode
I2CWriteByte(0xA1, 0x60)       # trigger
poll 0x60 until bit 0 clear    # timeout ~20s
poll 0x6F until bit 5 clear    # v1 only: wait for prog_en
```

**Sector/Block Erase** (common instruction):
```python
I2CWriteByte(0xB8, 0x60)       # setup: erase, wn=3
I2CWriteByte(opcode, 0x61)     # 0x20=sector 4K, 0xD8=block 64K
I2CWriteByte(addr[23:16], 0x64)
I2CWriteByte(addr[15:8], 0x65)
I2CWriteByte(addr[7:0], 0x66)
I2CWriteByte(0xB9, 0x60)       # trigger
poll 0x60 until bit 0 clear
poll 0x6F until bit 5 clear    # v1 only: wait for prog_en
```

**Read Flash** (v2 — program engine, v1 — common instruction):

V2: writing 0x6A triggers the read directly:
```python
I2CWriteByte(addr[23:16], 0x64)
I2CWriteByte(addr[15:8], 0x65)
I2CWriteByte(addr[7:0], 0x66)
I2CWriteByte(0x03, 0x6A)       # write read_op_code — triggers SPI read on v2
I2CRead(0x70, N)               # stream data from program_data_port
```

V1: 0x6A is just an opcode register, read goes through common instruction:
```python
common_inst(read, opcode=0x03, wn=0, rn=3, addr)  # via 0x60/0x61, addr in 0x64-66
I2CRead(0x70, N)               # stream data from program_data_port
```

**Page Program** (program engine):
```python
I2CWriteByte(addr[23:16], 0x64)
I2CWriteByte(addr[15:8], 0x65)
I2CWriteByte(addr[7:0], 0x66)
I2CWriteByte(len-1, 0x71)      # program_length
poll 0x6F until bit 4 set      # wait for prog_buf_wr_en
I2CWrite(0x70, data[0..len-1]) # load page data
val = I2CReadByte(0x6F)        # read-modify-write
I2CWriteByte(val | 0x20, 0x6F) # set prog_en, preserving isp_en + status
poll 0x6F until bit 5 clear    # wait for program complete
```

**CRC Verify** (program engine):
```python
I2CWriteByte(start[23:16], 0x64)
I2CWriteByte(start[15:8], 0x65)
I2CWriteByte(start[7:0], 0x66)
I2CWriteByte(end[23:16], 0x72)
I2CWriteByte(end[15:8], 0x73)
I2CWriteByte(end[7:0], 0x74)
val = I2CReadByte(0x6F)        # read-modify-write
I2CWriteByte(val | 0x04, 0x6F) # set crc_start, preserving isp_en + status
poll 0x6F until bit 1 set      # wait for crc_done
crc = I2CReadByte(0x75)        # CRC-8 result
```
CRC-8 polynomial 0x07 (CRC-8/SMBUS). The host-side implementation uses CRC
after erase and after page writes for verification.

### Flash Write Protection

The SPI flash ships with SRP0=1 and full BP protection (SR=0xFC). The scaler
board ties WP# to a GPIO, so WRSR is hardware-locked until WP# is deasserted.
Both GPIO deassert AND WRSR are required — GPIO alone doesn't bypass BP bits,
and WRSR alone fails while WP# is low.

**Unprotect sequence** (from USB capture of the original host software):
```python
# 1. Deassert WP# via scaler GPIO (indirect register interface)
I2CWriteByte(0x9F, 0xF4)       # select page 0x10
I2CWriteByte(0x10, 0xF5)
I2CWriteByte(0x36, 0xF4)       # read reg 0x36
val = I2CReadByte(0xF5)
I2CWriteByte(0x36, 0xF4)       # set bit 0 (GPIO direction for WP#)
I2CWriteByte(val | 0x01, 0xF5)
I2CWriteByte(0x9F, 0xF4)       # select page 0xFE
I2CWriteByte(0xFE, 0xF5)
I2CWriteByte(0x26, 0xF4)       # read reg 0x26
val = I2CReadByte(0xF5)
I2CWriteByte(0x26, 0xF4)       # set bit 0 (drive WP# high)
I2CWriteByte(val | 0x01, 0xF5)

# 2. Clear SR protection via WRSR
I2CWriteByte(0x68, 0x60)       # setup: write_after_WREN, wn=1
I2CWriteByte(0x01, 0x61)       # WRSR opcode
I2CWriteByte(0x02, 0x64)       # SR value (clears BP + SRP0)
I2CWriteByte(0x69, 0x60)       # trigger
poll 0x60 until bit 0 clear
```

**Re-protect** (after write/erase complete):
```python
I2CWriteByte(0x68, 0x60)       # write_after_WREN, wn=1
I2CWriteByte(0x01, 0x61)       # WRSR
I2CWriteByte(0xFF, 0x64)       # full protection
I2CWriteByte(0x69, 0x60)       # trigger
poll 0x60 until bit 0 clear
```

GPIO is not explicitly restored — power cycle resets the scaler GPIO state.

The Zbit ZB25VQ80 (JEDEC `5E6014`) is not in the original host software's
`Flash.dat` database. The host-side implementation has a bytecode interpreter
that runs chip-specific scripts from `Flash.dat` for SR manipulation, but for
unknown chips it falls through to the standard write_after_WREN path shown
above.

### MCU Control Registers

| Register | Name | Description |
|----------|------|-------------|
| 0xED | MCU_control | Bit 7: PORT_PIN_REG (default 1). Bits 5:2: FLASH_CLK_DIV (default 2, set to 1 for faster SPI). |
| 0xEE | MCU_clock_control | Bit 6: MCU_PERI_NON_STOP. Bits 5:2: MCU_CLK_DIV. Bit 1: SOF_RST (software reset MCU). Bit 0: SCA_HRST (hardware reset scaler). |

The host-side implementation sets `FLASH_CLK_DIV=1` (`0xED=0x84`) and
`MCU_CLK_DIV=1` (`0xEE=0x04`) during ISP init. On exit, it sets `SOF_RST`
(`0xEE |= 0x02`) to reset the MCU, which clears ISP state and boots from flash.

### Key host-side ISP entry points

| Address | Name | Description |
|---------|------|-------------|
| 0x10025A40 | EnterIspMode | Write 0x80 to 0x6F (isp_en), configure opcodes |
| 0x100252C0 | ReadFlashData | Flash read (addr→0x64-66, 0x03→0x6A, stream 0x70) |
| 0x10026010 | WriteFlashBlock | Write 4KB block (16 pages) with CRC verify |
| 0x10025BF0 | CalculateCRC | CRC over flash range (0x6F bit 2, result in 0x75) |
| 0x10025D10 | EraseBlock | Block erase 64K via common instruction (0xD8) |
| 0x10025E90 | EraseSector | Sector erase 4K via common instruction (0x20) |
| 0x10034ADD | ChipErase | Chip erase via common instruction (0xC7) |
| 0x10024C00 | ReadJedecID | JEDEC ID via common instruction (0x9F) |
| 0x100216E0 | isp_poll_reg | Poll register with mask |
| 0x10030D10 | ExecuteIsp | Main ISP orchestrator |

### Legacy plugin entry points

| Export | Address | Description |
|--------|---------|-------------|
| ExecuteIsp | 0x1003A420 | Main entry |
| GetPIName | 0x10039FB0 | Plugin name |
| InitialComm | 0x1003A0C0 | Init comm interface |
| Initinal [sic] | 0x10039EB0 | Initialize plugin |
| ReleasePI | 0x1003B390 | Release plugin |
| ReadVersionExport | 0x1003ACB0 | Read firmware version |
| SetCommInterface | 0x10039FA0 | Set comm interface |
