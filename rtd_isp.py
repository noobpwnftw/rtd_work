"""
RTD scaler ISP interface over I2C.

ISP entry is always via 0x6F bit 7 (isp_en). All writes to 0x6F must
use read-modify-write to preserve isp_en and other R/O status bits.
Exit resets the MCU via SOF_RST (0xEE bit 1).

Version flag (v1=RTD2660/2662, v2=RTD2775, v3=RTD2721C) affects:
  - read_page: v1 uses common instruction, v2/v3 write 0x6A to start read
  - read_page (v3 only): program engine returns a garbage first byte from
    the 0x70 FIFO when a read is triggered via 0x6A. The remaining bytes
    are correct and in order.
    Workaround: set addr to addr-1 (wrapping 0→0xFFFFFF), then discard the
    first byte with a 1-byte read before the real bulk read. Without this,
    byte 0 of the read is wrong — the hardware CRC (which reads SPI flash
    directly, bypassing the FIFO) will disagree with the dumped data.
  - erase: v1 polls prog_en on 0x6F after each erase command

Flash write protection: SRP0 + WP# pin require GPIO deassert via
scaler internal registers (0xF4/0xF5 indirect interface) before WRSR.

Two interfaces to the SPI flash:

1. Common instruction: control via 0x60 (common_inst_en), opcode in 0x61
   (common_op_code), addr in 0x64-66 (flash_prog_isp, shared with
   program engine), readback in 0x67-69 (common_inst_read_port).
   For SPI commands: JEDEC ID, erase, status register, etc.

2. Program engine: addr in 0x64-66, data through 0x70 (program_data_port),
   control via 0x6F (program_instruction), length in 0x71.
   Opcode config: 0x62=WREN, 0x63=EWSR, 0x6A=read, 0x6B=fast_read,
   0x6D=program, 0x6E=RDSR.
   CRC end addr in 0x72-74, result in 0x75.
"""

import time


class RTDISP:
    def __init__(self, i2c, slave=0x94, version=2):
        self.i2c = i2c
        self.slave = slave
        self.version = version

    # -- Scaler internal registers (via 0xF4/0xF5 indirect interface) --

    def _scaler_page(self, page):
        """Select scaler control register page via 0x9F."""
        self.i2c.write(self.slave, 0xF4, [0x9F])
        self.i2c.write(self.slave, 0xF5, [page])

    def _scaler_read(self, reg):
        """Read scaler internal register (current page)."""
        self.i2c.write(self.slave, 0xF4, [reg])
        return self.i2c.read(self.slave, 0xF5, 1)[0]

    def _scaler_write(self, reg, val):
        """Write scaler internal register (current page)."""
        self.i2c.write(self.slave, 0xF4, [reg])
        self.i2c.write(self.slave, 0xF5, [val])

    def flash_unprotect(self):
        """Deassert WP# via scaler GPIO, then clear flash SR protection."""
        if self.version == 3:
            # Page 0x10: set reg 0x0F bit 0 (GPIO output for WP#)
            self._scaler_page(0x10)
            val = self._scaler_read(0x0F)
            self._scaler_write(0x0F, val | 0x01)
            # Page 0xFE: set reg 0x0F bit 0 (drive WP# high)
            self._scaler_page(0xFE)
            val = self._scaler_read(0x0F)
            self._scaler_write(0x0F, val | 0x01)
        else:
            # Page 0x10: set reg 0x36 bit 0 (GPIO output for WP#)
            self._scaler_page(0x10)
            val = self._scaler_read(0x36)
            self._scaler_write(0x36, val | 0x01)
            # Page 0xFE: set reg 0x26 bit 0 (drive WP# high)
            self._scaler_page(0xFE)
            val = self._scaler_read(0x26)
            self._scaler_write(0x26, val | 0x01)
        # Now WRSR can clear protection bits
        self.write_status(0x02)

    def flash_protect(self):
        """Restore flash SR protection. GPIO resets on power cycle."""
        self.write_status(0xFF)

    # -- Helpers --

    def poll(self, reg, mask, expect=0, timeout=5000):
        """Poll register until (val & mask) == expect."""
        for _ in range(timeout):
            val = self.i2c.read(self.slave, reg, 1)[0]
            if (val & mask) == expect:
                return
            time.sleep(0.002)
        raise TimeoutError(
            f"ISP timeout: reg 0x{reg:02X}=0x{val:02X}, "
            f"mask=0x{mask:02X}, expect=0x{expect:02X}")

    def _set_addr(self, base, addr):
        """Write 24-bit address to 3 consecutive registers at base."""
        self.i2c.write(self.slave, base,     [(addr >> 16) & 0xFF])
        self.i2c.write(self.slave, base + 1, [(addr >> 8)  & 0xFF])
        self.i2c.write(self.slave, base + 2, [ addr        & 0xFF])

    # -- ISP control --

    def enter(self):
        """Enter ISP mode, wait for `isp_en` readback, then configure SPI opcodes."""
        self.i2c.write(self.slave, 0x6F, [0x80])  # isp_en (bit 7)
        time.sleep(0.1)
        self.poll(0x6F, 0x80, expect=0x80)
        self.configure_opcodes()

    def configure_opcodes(self):
        """Set SPI engine opcodes."""
        self.i2c.write(self.slave, 0x62, [0x06])      # wren_op_code
        self.i2c.write(self.slave, 0x63, [0x50])      # ewsr_op_code
        self.i2c.write(self.slave, 0x6B, [0x0B])      # fast_read_op_code
        self.i2c.write(self.slave, 0x6C, [0x00])      # read_instruction
        self.i2c.write(self.slave, 0x6D, [0x02])      # program_op_code
        self.i2c.write(self.slave, 0x6E, [0x05])      # read_status_register_op_code
        self.i2c.write(self.slave, 0xED, [0x84])      # MCU_control: FLASH_CLK_DIV=1
        self.i2c.write(self.slave, 0xEE, [0x04])      # MCU_clock_control: MCU_CLK_DIV=1

    def exit(self):
        """Exit ISP mode and reset MCU to boot from flash."""
        val = self.i2c.read(self.slave, 0xEE, 1)[0]
        try:
            self.i2c.write(self.slave, 0xEE, [val | 0x02])  # SOF_RST
        except IOError:
            pass  # NAK expected — scaler resets the MCU

    # -- Common instruction (0x60/0x61) --
    #
    # Register 0x60 bit-field: com_inst[7:5] | write_num[4:3] | rd_num[2:1] | enable[0]
    #   com_inst: 001=write, 010=read, 011=write_after_WREN, 100=write_after_EWSR, 101=erase
    #   enable: set 1 to trigger, auto-clears on completion

    @staticmethod
    def op(com_inst, wn=0, rn=0):
        """Build (setup, trigger) pair for register 0x60."""
        setup = (com_inst << 5) | (wn << 3) | (rn << 1)
        return (setup, setup | 1)

    def spi_cmd(self, op, opcode, addr=None, timeout=600):
        """Execute SPI command. op = (setup, trigger) pair for 0x60.
        Address goes to 0x64-66 (shared with program engine)."""
        self.i2c.write(self.slave, 0x60, [op[0]])
        self.i2c.write(self.slave, 0x61, [opcode])
        if addr is not None:
            self._set_addr(0x64, addr)
        self.i2c.write(self.slave, 0x60, [op[1]])
        self.poll(0x60, 0x01, timeout=timeout)

    def read_jedec(self):
        """Read 3-byte JEDEC ID via common instruction."""
        READ = self.op(0b010, rn=3)
        self.spi_cmd(READ, 0x9F)
        return bytes([
            self.i2c.read(self.slave, 0x67, 1)[0],
            self.i2c.read(self.slave, 0x68, 1)[0],
            self.i2c.read(self.slave, 0x69, 1)[0],
        ])

    def read_status(self):
        """Read SPI flash status register (0x05)."""
        READ_SR = self.op(0b010, rn=1)
        self.spi_cmd(READ_SR, 0x05)
        return self.i2c.read(self.slave, 0x67, 1)[0]

    def write_status(self, val):
        """Write status register (0x01) after WREN."""
        WREN_W = self.op(0b011, wn=1)
        self.i2c.write(self.slave, 0x64, [val])
        self.spi_cmd(WREN_W, 0x01)

    def write_status_ewsr(self, val):
        """Write status register (0x01) after EWSR."""
        EWSR_W = self.op(0b100, wn=1)
        self.i2c.write(self.slave, 0x64, [val])
        self.spi_cmd(EWSR_W, 0x01)

    def chip_erase(self, timeout=20000):
        """Full chip erase (0xC7)."""
        ERASE = self.op(0b101)
        self.spi_cmd(ERASE, 0xC7, timeout=timeout)
        if self.version == 1:
            self.poll(0x6F, 0x20, timeout=timeout)  # wait for prog_en clear

    def erase(self, start, length):
        """Erase region. 4K sectors (0x20) and 64K blocks (0xD8)."""
        ERASE_A = self.op(0b101, wn=3)
        offset = 0
        while offset < length:
            addr = start + offset
            if addr % 0x10000 == 0 and offset + 0x10000 <= length:
                self.spi_cmd(ERASE_A, 0xD8, addr=addr)
                if self.version == 1:
                    self.poll(0x6F, 0x20)
                offset += 0x10000
            else:
                self.spi_cmd(ERASE_A, 0x20, addr=addr)
                if self.version == 1:
                    self.poll(0x6F, 0x20)
                offset += 0x1000

    # -- Program engine (0x6F) --
    #
    # 0x6F program_instruction bits (always read-modify-write):
    #   7: isp_en (must preserve)  6: prog_mode (AAI) 5: prog_en (auto-clear)
    #   4: prog_buf_wr_en (R/O)    3: prog_dummy
    #   2: crc_start (auto-clear)  1: crc_done (R/O)  0: rst_flash_ctrl

    def read_page(self, addr, length=256):
        """Read flash data from 0x70.

        v3 (RTD2721C): the program engine returns a garbage first byte
        from the 0x70 FIFO on read trigger; remaining bytes are correct.
        Work around by starting one byte early and discarding the first
        byte with a 1-byte dummy read.
        """
        if self.version == 1:
            READ = self.op(0b010, rn=3)
            self.spi_cmd(READ, 0x03, addr=addr)
        else:
            if self.version == 3:
                if addr == 0:
                    addr = 0xFFFFFF
                else:
                    addr -= 1
            self._set_addr(0x64, addr)
            self.i2c.write(self.slave, 0x6A, [0x03])
            if self.version == 3:
                self.i2c.read(self.slave, 0x70, 1)
        return self.i2c.read(self.slave, 0x70, length)

    def write_page(self, addr, data):
        """Write flash page. Waits for prog_buf_wr_en (bit 4) before loading data."""
        self._set_addr(0x64, addr)
        self.i2c.write(self.slave, 0x71, [len(data) - 1])
        self.poll(0x6F, 0x10, expect=0x10)
        self.i2c.write(self.slave, 0x70, data)
        val = self.i2c.read(self.slave, 0x6F, 1)[0]
        self.i2c.write(self.slave, 0x6F, [val | 0x20])
        self.poll(0x6F, 0x20)

    def crc(self, start, length):
        """Hardware CRC over flash range. Returns 1-byte CRC-8."""
        self._set_addr(0x64, start)
        self._set_addr(0x72, start + length - 1)
        val = self.i2c.read(self.slave, 0x6F, 1)[0]
        self.i2c.write(self.slave, 0x6F, [val | 0x04])
        self.poll(0x6F, 0x02, expect=0x02)
        return self.i2c.read(self.slave, 0x75, 1)[0]
