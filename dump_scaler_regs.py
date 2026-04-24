#!/usr/bin/env python3
"""Dump scaler register pages for A/B comparison.

Uses `rtd_scaler` (debug bridge or ISP indirect) to read flat registers.

Note: registers 0x2A/2B (display format port), 0x30/31 (FIFO port),
and 0x33/34 (scaling factor port) are indexed access ports. Reading
them flat captures the port state, not the indexed contents. Use
`rtd_scaler.py --measure` to read indexed values properly.

Usage:
  python3 dump_scaler_regs.py regs.txt
  python3 dump_scaler_regs.py regs.txt --isp
  diff regs_working.txt regs_patched.txt
"""
import argparse
from rtd_i2c import RealtekI2C
from rtd_scaler import RTDScaler, DebugBridge, ISPIndirect

# All pages defined in the regmap.
PAGES = [
    0x00, 0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x18, 0x19, 0x1D,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x27, 0x2A, 0x2D, 0x2E,
    0x30, 0x31, 0x33, 0x34, 0x36, 0x38, 0x39, 0x3A, 0x40,
    0x55, 0x63, 0x67, 0x68, 0x6F, 0x71, 0x72, 0x76, 0x77, 0x7A, 0x7B, 0x7C,
    0x80, 0xB0, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBF,
    0xC0, 0xFD, 0xFE,
]


def main():
    p = argparse.ArgumentParser(description="Dump scaler register pages")
    p.add_argument("output", help="Output file")
    p.add_argument("--isp", action="store_true", help="Use ISP indirect instead of debug bridge")
    p.add_argument("--exit", action="store_true", help="Exit after dump (resumes MCU)")
    args = p.parse_args()

    i2c = RealtekI2C()
    transport = ISPIndirect(i2c) if args.isp else DebugBridge(i2c)
    scaler = RTDScaler(transport)
    entered = False
    try:
        scaler.enter()
        entered = True

        with open(args.output, "w") as f:
            for page in PAGES:
                f.write(f"=== Page 0x{page:02X} ===\n")
                for i in range(0, 256, 16):
                    vals = []
                    for reg in range(i, i + 16):
                        vals.append(scaler.read(page, reg))
                    f.write(f"  {page:02X}:{i:02X}  {' '.join(f'{b:02X}' for b in vals)}\n")
                f.write("\n")
    finally:
        if entered and args.exit:
            scaler.exit()
        i2c.close()

    print(f"Dumped {len(PAGES)} pages to {args.output}")
    print("Note: access ports (00:2A/2B, 00:30/31, etc.) show port state, not indexed contents.")


if __name__ == '__main__':
    main()
