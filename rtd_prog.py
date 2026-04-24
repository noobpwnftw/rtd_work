#!/usr/bin/env python3
"""
RTD SPI flash programmer via Realtek USB-I2C adapter (FX2LP).

Reads, writes, and erases SPI flash through the RTD scaler's ISP interface.
Requires OpenRTD firmware on the FX2LP dongle.

Usage:
  python3 rtd_prog.py                                  # enter ISP, read JEDEC ID + SR
  python3 rtd_prog.py read out.bin --size 100000       # read 1MB flash
  python3 rtd_prog.py write firmware.bin               # chip erase + write + CRC verify
  python3 rtd_prog.py write firmware.bin --addr 10000  # sector erase + write at offset
  python3 rtd_prog.py erase                            # chip erase
  python3 rtd_prog.py erase --size 1000                # sector erase 4KB at addr 0
  python3 rtd_prog.py verify firmware.bin              # CRC verify flash against file
"""

import argparse
import sys

from rtd_i2c import RealtekI2C
from rtd_isp import RTDISP


def crc8(data, size):
    """CRC-8 (poly 0x07, init 0x00) — matches RTD hardware CRC."""
    crc = 0
    for b in data[:size]:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if crc & 0x80 else (crc << 1)
            crc &= 0xFF
    return crc

JEDEC_MANUFACTURERS = {
    0x01: "Spansion", 0x0B: "XTX", 0x1C: "EON", 0x1F: "Atmel/Adesto",
    0x20: "Micron/XMC", 0x37: "AMIC", 0x7F: "ISSI", 0x85: "Puya",
    0x5E: "Zbit", 0x9D: "ISSI/PMC", 0xA1: "Fudan", 0xBF: "SST", 0xC8: "GigaDevice",
    0xEF: "Winbond",
}


def jedec_size(isp):
    """Return flash size in bytes from JEDEC capacity byte, or 0 if unknown."""
    jedec = isp.read_jedec()
    cap = jedec[2]
    return (1 << cap) if cap < 32 else 0


def print_jedec(isp):
    """Print JEDEC ID, manufacturer, and decoded capacity when available."""
    jedec = isp.read_jedec()
    mfr, typ, cap = jedec[0], jedec[1], jedec[2]
    name = JEDEC_MANUFACTURERS.get(mfr, f"Unknown(0x{mfr:02X})")
    size_kb = (1 << cap) // 1024 if cap < 32 else 0
    sz = f"{size_kb}KB" if size_kb < 1024 else f"{size_kb // 1024}MB"
    print(f"  JEDEC: {jedec.hex()} — {name} {sz}" if size_kb else
          f"  JEDEC: {jedec.hex()} — {name}")


def read_flash(isp, size, outfile, start=0):
    """Read flash contents to a file in 256-byte chunks."""
    print(f"Reading 0x{size:X} bytes from 0x{start:06X} to {outfile}...")
    with open(outfile, "wb") as f:
        offset = 0
        while offset < size:
            chunk = min(256, size - offset)
            try:
                data = isp.read_page(start + offset, chunk)
                f.write(data)
            except (IOError, TimeoutError) as e:
                print(f"\nError at 0x{start + offset:06X}: {e}")
                f.write(b'\xFF' * chunk)
            offset += chunk
            if offset % 0x4000 == 0 or offset >= size:
                pct = offset * 100 // size
                print(f"\r  {offset:06X}/{size:06X} ({pct}%)", end="", flush=True)
    print("\nDone.")


def erase_flash(isp, start, size=None):
    """Erase flash, then verify the erased region with the hardware CRC."""
    if size is None:
        if start != 0:
            print("Error: --addr requires --size for erase")
            return False
        size = jedec_size(isp)
        print("Chip erase...")
        isp.chip_erase()
    else:
        print(f"Erasing 0x{size:X} bytes at 0x{start:06X}...")
        isp.erase(start, size)
    hw = isp.crc(start, size)
    expected = crc8(b'\xFF' * size, size)
    if hw != expected:
        print(f"Erase verify FAILED (hw=0x{hw:02X} expected=0x{expected:02X})")
        return False
    print(f"Erase OK (CRC 0x{hw:02X})")
    return True


def write_flash(isp, data, start=0, size=None):
    """Erase, program, and verify a flash region."""
    if not erase_flash(isp, start, size):
        return False
    if size is None:
        size = len(data)
    print(f"Writing 0x{size:X} bytes at 0x{start:06X}...")
    offset = 0
    while offset < size:
        chunk = min(256, size - offset)
        page = data[offset:offset + chunk]
        if page != b'\xFF' * chunk:
            isp.write_page(start + offset, page)
        offset += chunk
        if offset % 0x1000 == 0 or offset >= size:
            pct = offset * 100 // size
            print(f"\r  {offset:06X}/{size:06X} ({pct}%)", end="", flush=True)
    print("\nDone.")
    return verify_flash(isp, data, start, size)


def verify_flash(isp, data, start=0, size=None):
    """Compare flash contents against local data using the hardware CRC."""
    if size is None:
        size = len(data)
    print(f"Verifying 0x{size:X} bytes at 0x{start:06X}...")
    hw = isp.crc(start, size)
    local = crc8(data, size)
    if hw != local:
        print(f"Verify FAILED (hw=0x{hw:02X} local=0x{local:02X})")
        return False
    print(f"Verify OK (CRC 0x{hw:02X})")
    return True


def parse_addr(s):
    """Parse a hexadecimal CLI address or size."""
    return int(s, 16)


def main():
    p = argparse.ArgumentParser(description="RTD SPI flash programmer (Realtek USB-I2C)")
    sub = p.add_subparsers(dest="cmd")

    sp = sub.add_parser("read", help="Read flash to file")
    sp.add_argument("file", help="Output file")
    sp.add_argument("--size", type=parse_addr, default=None, help="Size in hex (default: auto-detect from JEDEC)")
    sp.add_argument("--addr", type=parse_addr, default=0, help="Start address in hex")

    sp = sub.add_parser("write", help="Erase + write + verify flash from file")
    sp.add_argument("file", help="Input file")
    sp.add_argument("--addr", type=parse_addr, default=0, help="Start address in hex")
    sp.add_argument("--size", type=parse_addr, default=None, help="Size in hex")

    sp = sub.add_parser("erase", help="Erase flash (chip erase unless --size given)")
    sp.add_argument("--addr", type=parse_addr, default=0, help="Start address in hex")
    sp.add_argument("--size", type=parse_addr, default=None, help="Size in hex")

    sp = sub.add_parser("verify", help="Compare flash to file")
    sp.add_argument("file", help="File to compare against")
    sp.add_argument("--addr", type=parse_addr, default=0, help="Start address in hex")
    sp.add_argument("--size", type=parse_addr, default=None, help="Size in hex")

    p.add_argument("--version", "-V", type=int, default=2, choices=[1, 2, 3],
                    help="ISP version: 1=RTD2660, 2=RTD2775, 3=RTD2721C (default: 2)")
    args = p.parse_args()

    ok = True
    entered = False
    i2c = RealtekI2C()
    isp = RTDISP(i2c, version=args.version)
    try:
        isp.enter()
        entered = True
        print_jedec(isp)
        sr = isp.read_status()
        print(f"  SR: 0x{sr:02X} (BP={((sr >> 2) & 0x07)}, SRP0={sr >> 7})")

        if args.cmd == "read":
            size = args.size or jedec_size(isp)
            if size:
                read_flash(isp, size, args.file, args.addr)
            else:
                ok = False

        elif args.cmd in ("write", "verify"):
            with open(args.file, "rb") as f:
                data = f.read()
            if args.cmd == "write":
                isp.flash_unprotect()
                ok = write_flash(isp, data, args.addr, args.size)
                isp.flash_protect()
            elif args.cmd == "verify":
                ok = verify_flash(isp, data, args.addr, args.size)

        elif args.cmd == "erase":
            isp.flash_unprotect()
            ok = erase_flash(isp, args.addr, args.size)
            isp.flash_protect()
    finally:
        if entered:
            isp.exit()
        i2c.close()

    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
