"""
RTD scaler register access.

Two transports:
  - DebugBridge (0x6A)
  - ISPIndirect (0x94)

RTDScaler wraps either transport with higher-level ops:
  - dump_page
  - read_port / write_port
  - poll_reg
  - measure

Debug bridge protocol:
  - Read:  0x01=page → 0x3A=reg → read 0x08
  - Write: 0x01=page → 0x03=val → 0x3B=reg

ISP mode check: 0x94:0x23 returns 0x00 in normal mode and non-zero in ISP mode.
"""

from __future__ import annotations

import argparse
import time


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------

class DebugBridge:
    """Scaler register access via the debug bridge at I2C slave 0x6A."""

    SLAVE = 0x6A

    def __init__(self, i2c, slave=SLAVE):
        self.i2c = i2c
        self.slave = slave

    def enter(self):
        val = self.i2c.read(0x94, 0x23, 1)[0]
        if val != 0x00:
            raise RuntimeError(
                f"Scaler is in ISP mode (0x94:0x23=0x{val:02X}), exit ISP first"
            )
        self.i2c.write(self.slave, 0x80, [0x01])
        time.sleep(0.1)

    def exit(self):
        self.i2c.write(self.slave, 0x80, [0x00])

    def read(self, page, reg):
        self.i2c.write(self.slave, 0x01, [page])
        self.i2c.write(self.slave, 0x3A, [reg])
        return self.i2c.read(self.slave, 0x08, 1)[0]

    def write(self, page, reg, val):
        self.i2c.write(self.slave, 0x01, [page])
        self.i2c.write(self.slave, 0x03, [val])
        self.i2c.write(self.slave, 0x3B, [reg])

    def sfr_write(self, addr, val):
        """Write val to MCU SFR at addr (via stub command 0x5A).

        The stub currently handles only P1 (0x90) and P3 (0xB0). Other
        addresses are ignored.
        """
        self.i2c.write(self.slave, 0x03, [val])
        self.i2c.write(self.slave, 0x5A, [addr])

    def sfr_read(self, addr):
        """Read MCU SFR at addr (via stub command 0x5B).

        The stub currently handles only P1 (0x90) and P3 (0xB0). Other
        addresses return 0.
        """
        self.i2c.write(self.slave, 0x5B, [addr])
        return self.i2c.read(self.slave, 0x08, 1)[0]


class ISPIndirect:
    """Scaler register access via ISP indirect interface (0x94:0xF4/0xF5)."""

    SLAVE = 0x94

    def __init__(self, i2c, slave=SLAVE):
        self.i2c = i2c
        self.slave = slave
        self._page = None

    def enter(self):
        from rtd_isp import RTDISP

        RTDISP(self.i2c).enter()
        self._page = None

    def exit(self):
        from rtd_isp import RTDISP

        RTDISP(self.i2c).exit()

    def _select_page(self, page):
        if self._page == page:
            return
        self.i2c.write(self.slave, 0xF4, [0x9F])
        self.i2c.write(self.slave, 0xF5, [page])
        self._page = page

    def read(self, page, reg):
        self._select_page(page)
        self.i2c.write(self.slave, 0xF4, [reg])
        return self.i2c.read(self.slave, 0xF5, 1)[0]

    def write(self, page, reg, val):
        self._select_page(page)
        self.i2c.write(self.slave, 0xF4, [reg])
        self.i2c.write(self.slave, 0xF5, [val])


# ---------------------------------------------------------------------------
# Scaler (transport-agnostic)
# ---------------------------------------------------------------------------

class RTDScaler:
    """Scaler operations on top of any transport (DebugBridge or ISPIndirect)."""

    def __init__(self, transport):
        self.t = transport

    def enter(self):
        self.t.enter()

    def exit(self):
        self.t.exit()

    def read(self, page, reg):
        return self.t.read(page, reg)

    def write(self, page, reg, val):
        self.t.write(page, reg, val)

    def read_addr(self, addr):
        return self.t.read(addr >> 8, addr & 0xFF)

    def write_addr(self, addr, val):
        self.t.write(addr >> 8, addr & 0xFF, val)

    def dump_page(self, page):
        out = bytearray(256)
        for i in range(256):
            out[i] = self.t.read(page, i)
        return bytes(out)

    def read_port(self, page, index_reg, data_reg, index, count=1):
        """Read from an indexed access port on the given page."""
        self.t.write(page, index_reg, index)
        return [self.t.read(page, data_reg) for _ in range(count)]

    def write_port(self, page, index_reg, data_reg, index, values):
        """Write to an indexed access port on the given page."""
        for i, val in enumerate(values):
            self.t.write(page, index_reg, (index + i) & 0xFF)
            self.t.write(page, data_reg, val)

    def poll_reg(self, page, reg, mask, expect, timeout=600):
        """Poll a scaler register until (val & mask) == expect."""
        for _ in range(timeout):
            val = self.t.read(page, reg)
            if (val & mask) == expect:
                return
            time.sleep(0.002)
        raise TimeoutError(
            f"poll timeout: {page:02X}:{reg:02X}=0x{val:02X}, "
            f"mask=0x{mask:02X}, expect=0x{expect:02X}")

    def measure(self):
        """Trigger measurement and return raw register bytes."""
        self.t.write(0x30, 0x02, 0x00)
        self.t.write(0x30, 0x02, 0x01)
        self.poll_reg(0x30, 0x02, 0x01, 0x00)
        self.t.write(0x30, 0x02, 0x40)

        return {
            "online": [self.t.read(0x30, i) for i in range(0x15)],
            "vgip":   [self.t.read(0x00, i) for i in range(0x14, 0x1F)],
            "fifo":   self.read_port(0x00, 0x30, 0x31, 0x00, 4),
            "disp":   self.read_port(0x00, 0x2A, 0x2B, 0x00, 22),
            "pll":    [self.t.read(0x01, i) for i in [0xBF, 0xC0, 0xC4, 0xC5, 0xDA]],
            "last":   [self.t.read(0x00, 0x44), self.t.read(0x00, 0x45)],
        }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="RTD scaler register tool")
    p.add_argument("-r", "--read",  action="append", default=[], metavar="PP:RR")
    p.add_argument("-w", "--write", action="append", default=[], metavar="PP:RR=VV")
    p.add_argument("-R", "--sfr-read",  action="append", default=[], metavar="AA",
                   help="Read MCU SFR at addr AA (hex, debug bridge only)")
    p.add_argument("-W", "--sfr-write", action="append", default=[], metavar="AA=VV",
                   help="Write VV to MCU SFR at AA (hex, debug bridge only)")
    p.add_argument("-d", "--dump",  action="append", default=[], metavar="PP",
                   type=lambda s: int(s, 16))
    p.add_argument("-m", "--measure", action="store_true")
    p.add_argument("--isp", action="store_true", help="use ISP indirect instead of debug bridge")
    p.add_argument("--exit", action="store_true", help="exit after operations (resumes MCU)")
    args = p.parse_args()

    if not any([args.read, args.write, args.sfr_read, args.sfr_write, args.dump, args.measure, args.exit]):
        p.error("specify -r/-w, -R/-W, -d, -m or --exit")
    if (args.sfr_read or args.sfr_write) and args.isp:
        p.error("--sfr-read/--sfr-write require the debug bridge (not --isp)")

    from rtd_i2c import RealtekI2C
    i2c = RealtekI2C()

    transport = ISPIndirect(i2c) if args.isp else DebugBridge(i2c)
    scaler = RTDScaler(transport)

    try:
        scaler.enter()

        for spec in args.write:
            addr_part, val_str = spec.split("=")
            page, reg = (int(x, 16) for x in addr_part.split(":"))
            val = int(val_str, 16)
            old = scaler.read(page, reg)
            scaler.write(page, reg, val)
            rb = scaler.read(page, reg)
            print(f"  {page:02X}:{reg:02X}: 0x{old:02X} -> 0x{val:02X}  rb=0x{rb:02X}")

        for spec in args.dump:
            data = scaler.dump_page(spec)
            print(f"=== Page 0x{spec:02X} ===")
            for row in range(16):
                off = row * 16
                print(f"  {spec:02X}:{off:02X}  {' '.join(f'{b:02X}' for b in data[off:off+16])}")

        for spec in args.read:
            page, reg = (int(x, 16) for x in spec.split(":"))
            print(f"  {page:02X}:{reg:02X} = 0x{scaler.read(page, reg):02X}")

        for spec in args.sfr_write:
            addr_str, val_str = spec.split("=")
            addr = int(addr_str, 16)
            val = int(val_str, 16)
            old = transport.sfr_read(addr)
            transport.sfr_write(addr, val)
            rb = transport.sfr_read(addr)
            print(f"  SFR {addr:02X}: 0x{old:02X} -> 0x{val:02X}  rb=0x{rb:02X}")

        for spec in args.sfr_read:
            addr = int(spec, 16)
            print(f"  SFR {addr:02X} = 0x{transport.sfr_read(addr):02X}")

        if args.measure:
            m = scaler.measure()

            def be(data, i):
                return (data[i] << 8) | data[i + 1]

            v, d, o, p = m["vgip"], m["disp"], m["online"], m["pll"]

            def h(val, w=4):
                return f"0x{val:0{w}X} ({val})"

            print("=== Measure M1 ===")
            print("Input:")
            print(f"  IH Act Start:     {h(((v[0] & 0x0F) << 8) | v[1])}")
            print(f"  IH Act Width:     {h(((v[0] >> 4) << 8) | v[3])}")
            print(f"  IV Act Start:     {h(((v[4] & 0x0F) << 8) | v[5])}")
            print(f"  IV Act Length:    {h(((v[4] >> 4) << 8) | v[7])}")
            f = m["fifo"]
            print(f"  FIFO Win H:       {h(((f[0] >> 4) << 8) | f[1])}")
            print(f"  FIFO Win V:       {h(((f[0] & 0x0F) << 8) | f[2])}")
            print("Display:")
            print(f"  DH Total:         {h(be(d, 0))}")
            print(f"  DH Sync End:      {h(d[2], 2)}")
            print(f"  DH BK Sta/End:    {h(be(d, 3))} / {h(be(d, 7))}")
            print(f"  DH Act Sta/End:   {h(be(d, 5))} / {h(be(d, 9))}")
            print(f"  DV Total:         {h(be(d, 11))}")
            print(f"  DV Sync End:      {h(d[13], 2)}")
            print(f"  DV BK Sta/End:    {h(be(d, 14))} / {h(be(d, 18))}")
            print(f"  DV Act Sta/End:   {h(be(d, 16))} / {h(be(d, 20))}")
            print(f"  Last Line:        {h(be(m['last'], 0))}")
            print("Off-Line (page30 0x04-0x0D):")
            print(f"  Flags:            {h(o[4], 2)}")
            print(f"  HSync Period:     {h(be(o, 5))}")
            print(f"  VSync Period:     {h(be(o, 8))}")
            print(f"  HS Pulse:         {h(be(o, 0x0A))}")
            print(f"  VS Pulse:         {h(be(o, 0x0C))}")
            print("On-Line (page30 0x0E-0x14):")
            print(f"  HSync Period:     {h(be(o, 0x0E))}")
            print(f"  VSync Period:     {h(be(o, 0x11))}")
            print(f"  HS Pulse:         {h(be(o, 0x13))}")
            print("PLL:")
            print(f"  DPLL M/N:         {h(p[0], 2)} / {h(p[1], 2)}")
            print(f"  DCLK Offset:      {h((p[2] << 8) | p[3])}")
            print(f"  Pixel Clk Div:    {h(p[4], 2)}")

    finally:
        if args.exit:
            scaler.exit()
        i2c.close()


if __name__ == "__main__":
    raise SystemExit(main())
