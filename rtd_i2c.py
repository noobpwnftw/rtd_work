"""
USB-I2C adapter driver for FX2LP (CY7C68013A).

Works with OpenRTD firmware (firmware/dongle/build/openrtd.ihx) or the
original Realtek realsil.hex. Commands 0x11-0x1D on EP4 OUT / EP8 IN.
"""

import signal

import usb.core
import usb.util

VID = 0x2007
PID = 0x0808
EP_OUT = 0x04  # EP4 OUT (bulk)
EP_IN = 0x88   # EP8 IN  (bulk)


def csum(data):
    return sum(data) & 0xFF


class _DeferSigint:
    """Hold SIGINT until the block exits, then re-raise via the prior handler."""

    def __enter__(self):
        self._received = None
        try:
            self._old = signal.signal(signal.SIGINT, self._catch)
        except ValueError:
            self._old = None  # not in main thread; can't install handler
        return self

    def _catch(self, sig, frame):
        self._received = (sig, frame)

    def __exit__(self, *exc):
        if self._old is None:
            return
        signal.signal(signal.SIGINT, self._old)
        if self._received and callable(self._old):
            self._old(*self._received)


class RealtekI2C:

    def __init__(self):
        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            raise RuntimeError(
                f"Device {VID:04x}:{PID:04x} not found. "
                "Load firmware first: sudo cycfx2prog prg:firmware/dongle/build/openrtd.ihx run"
            )
        self.dev.set_configuration()
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)
        usb.util.claim_interface(self.dev, 0)
        self.dev.clear_halt(EP_IN)
        self.dev.clear_halt(EP_OUT)
        self._init()

    def _init(self):
        """Configure I2C bit-bang timing (~100kHz)."""
        with _DeferSigint():
            self._send([0x17, 0x01, 0x00, 0x01, 0x86, 0xA0])  # slave mode + speed
            self._recv(0)
        with _DeferSigint():
            self._send([0x16, 0x00, 0xC8])  # clock divider
            self._recv(0)

    def _send(self, pkt):
        pkt = list(pkt)
        pkt.append(csum(pkt))
        self.dev.write(EP_OUT, bytes(pkt), timeout=5000)

    def _recv(self, data_len):
        """Read and validate response: [data x N] [status] [checksum]."""
        raw = bytes(self.dev.read(EP_IN, max(data_len + 8, 64), timeout=5000))
        expected = data_len + 2
        if len(raw) < expected:
            raise IOError(f"Short response ({len(raw)} < {expected}): {raw.hex()}")
        payload = raw[:expected - 1]
        if csum(payload) != raw[expected - 1]:
            raise IOError(f"Checksum error: {raw.hex()}")
        status = raw[data_len]
        if status != 0:
            raise IOError(f"I2C error (status 0x{status:02X})")
        return raw[:data_len]

    def write(self, slave, sub, data, ignore_wnak=True):
        """I2C write. slave = 8-bit write address."""
        n = len(data)
        cmd = 0x14 if ignore_wnak else 0x12
        with _DeferSigint():
            self._send([cmd, slave, sub, (n >> 8) & 0xFF, n & 0xFF] + list(data))
            self._recv(0)

    def read(self, slave, sub, length):
        """I2C read. slave = 8-bit write address (firmware ORs 0x01 for read phase)."""
        with _DeferSigint():
            self._send([0x11, slave, sub, (length >> 8) & 0xFF, length & 0xFF])
            return self._recv(length)

    def eddc_read(self, segment, slave, offset, length):
        """E-DDC segment read (CMD 0x1C). For EDID blocks beyond 256 bytes."""
        with _DeferSigint():
            self._send([0x1C, segment, slave, offset,
                        (length >> 8) & 0xFF, length & 0xFF])
            return self._recv(length)

    def eddc_write(self, segment, slave, offset, data):
        """E-DDC segment write (CMD 0x1D)."""
        n = len(data)
        with _DeferSigint():
            self._send([0x1D, segment, slave, offset,
                        (n >> 8) & 0xFF, n & 0xFF] + list(data))
            self._recv(0)

    def reset(self):
        self._send([0x19])

    def close(self):
        usb.util.release_interface(self.dev, 0)


def main():
    import argparse
    p = argparse.ArgumentParser(description="FX2LP USB-I2C adapter control")
    p.add_argument("--reset", action="store_true", help="reset firmware (CMD 0x19)")
    args = p.parse_args()

    if not args.reset:
        p.error("specify --reset")

    i2c = RealtekI2C()
    i2c.reset()


if __name__ == "__main__":
    raise SystemExit(main())
