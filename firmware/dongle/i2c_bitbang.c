/*
 * I2C bit-bang engine for FX2LP.
 * SDA = PA0 (IOA bit 0), SCL = PA1 (IOA bit 1).
 *
 * Pin control:
 *   SDA high (release): SETB IOA.0, MOV OEA,#0xFE  (latch=1, then input)
 *   SDA low  (drive):   MOV OEA,#0xFF, CLR IOA.0   (output, then drive low)
 *   SCL high:           SETB IOA.1  (push-pull, OEA.1 always 1)
 *   SCL low:            CLR  IOA.1
 *   Read SDA:           MOV C,IOA.0 (works when OEA.0=0 and latch=1)
 *
 * OEA=0xFE: bit 0 input (SDA released), bit 1 output (SCL driven)
 * OEA=0xFF: all output (SDA + SCL driven by IOA latch)
 *
 * Primitives (start, stop, write_byte, read_byte) are in i2c_fast.a51.
 * Delay uses a JIT NOP sled — see i2c_set_delay().
 */

#include <fx2regs.h>
#include <fx2types.h>
#include "i2c_bitbang.h"

/* i2c_set_delay, primitives, and write_verify are all in i2c_fast.a51 */

/* CMD 0x11: I2C read with 1-byte sub-address.
 * START, slave, sub, RE-START, slave|1, read N, STOP */
BYTE i2c_read_1sub(BYTE slave, BYTE sub, __xdata BYTE *buf, WORD len) {
    WORD i;

    i2c_start();
    if (!i2c_write_byte(slave)) { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(sub))   { i2c_stop(); return I2C_NAK; }

    i2c_repeated_start();
    if (!i2c_write_byte(slave | 0x01)) { i2c_stop(); return I2C_NAK; }

    for (i = 0; i < len; i++)
        buf[i] = i2c_read_byte(i < len - 1);

    i2c_stop();
    return I2C_OK;
}

static BYTE i2c_write_1sub_impl(BYTE slave, BYTE sub, __xdata BYTE *data, WORD len,
                                 BOOL verify) {
    WORD i;

    i2c_start();
    if (!i2c_write_byte(slave)) { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(sub))   { i2c_stop(); return I2C_NAK; }

    for (i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) { i2c_stop(); return I2C_NAK; }
    }

    i2c_stop();

    if (verify && !i2c_write_verify())
        return I2C_WNAK;

    return I2C_OK;
}

/* CMD 0x12: I2C write with 1-byte sub-address.
 * START, slave, sub, data[0..N-1], STOP + write verify */
BYTE i2c_write_1sub(BYTE slave, BYTE sub, __xdata BYTE *data, WORD len) {
    return i2c_write_1sub_impl(slave, sub, data, len, TRUE);
}

/* CMD 0x14: I2C write with 1-byte sub-address, no post-write wait.
 * START, slave, sub, data[0..N-1], STOP */
BYTE i2c_write_1sub_nowait(BYTE slave, BYTE sub, __xdata BYTE *data, WORD len) {
    return i2c_write_1sub_impl(slave, sub, data, len, FALSE);
}

/* CMD 0x13: I2C read with 2-byte sub-address (single-phase).
 * START, slave, sub_hi, sub_lo, RE-START, slave|1, read N, STOP */
BYTE i2c_read_2sub(BYTE slave, BYTE sub_hi, BYTE sub_lo, __xdata BYTE *buf,
                    WORD len) {
    WORD i;

    i2c_start();
    if (!i2c_write_byte(slave))   { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(sub_hi))  { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(sub_lo))  { i2c_stop(); return I2C_NAK; }

    i2c_repeated_start();
    if (!i2c_write_byte(slave | 0x01)) { i2c_stop(); return I2C_NAK; }

    for (i = 0; i < len; i++)
        buf[i] = i2c_read_byte(i < len - 1);

    i2c_stop();
    return I2C_OK;
}

/* CMD 0x15: Current-address read (no sub-address phase).
 * START, slave|1, read N, STOP */
BYTE i2c_read_current(BYTE slave, __xdata BYTE *buf, WORD len) {
    WORD i;

    i2c_start();
    if (!i2c_write_byte(slave | 0x01)) { i2c_stop(); return I2C_NAK; }

    for (i = 0; i < len; i++)
        buf[i] = i2c_read_byte(i < len - 1);

    i2c_stop();
    return I2C_OK;
}

/* CMD 0x1A: I2C read with 2-byte sub-address (3-phase).
 * Phase 1: START, slave, sub_hi
 * Phase 2: RE-START, sub_lo, 0x00 (padding)
 * Phase 3: RE-START, slave|1, read N, STOP */
BYTE i2c_read_2sub_wide(BYTE slave, BYTE sub_hi, BYTE sub_lo,
                    __xdata BYTE *buf, WORD len) {
    WORD i;

    i2c_start();
    if (!i2c_write_byte(slave))   { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(sub_hi))  { i2c_stop(); return I2C_NAK; }

    i2c_repeated_start();
    if (!i2c_write_byte(sub_lo))  { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(0x00))    { i2c_stop(); return I2C_NAK; }

    i2c_repeated_start();
    if (!i2c_write_byte(slave | 0x01)) { i2c_stop(); return I2C_NAK; }

    for (i = 0; i < len; i++)
        buf[i] = i2c_read_byte(i < len - 1);

    i2c_stop();
    return I2C_OK;
}

/* CMD 0x1B: I2C write with 2-byte sub-address (3-phase).
 * Phase 1: START, slave, sub_hi
 * Phase 2: RE-START, sub_lo, 0x00 (padding)
 * Phase 3: write data[0..N-1], STOP + write verify */
BYTE i2c_write_2sub_wide(BYTE slave, BYTE sub_hi, BYTE sub_lo,
                     __xdata BYTE *data, WORD len) {
    WORD i;

    i2c_start();
    if (!i2c_write_byte(slave))   { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(sub_hi))  { i2c_stop(); return I2C_NAK; }

    i2c_repeated_start();
    if (!i2c_write_byte(sub_lo))  { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(0x00))    { i2c_stop(); return I2C_NAK; }

    for (i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) { i2c_stop(); return I2C_NAK; }
    }

    i2c_stop();

    if (!i2c_write_verify())
        return I2C_WNAK;

    return I2C_OK;
}

/* CMD 0x1C: E-DDC segment read (VESA E-DDC 1.3).
 * START, 0x60, segment, RE-START, slave, offset,
 * RE-START, slave|1, read N, STOP */
BYTE i2c_eddc_read(BYTE segment, BYTE slave, BYTE offset,
                    __xdata BYTE *buf, WORD len) {
    WORD i;

    i2c_start();
    if (!i2c_write_byte(0x60))    { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(segment)) { i2c_stop(); return I2C_NAK; }

    i2c_repeated_start();
    if (!i2c_write_byte(slave))   { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(offset))  { i2c_stop(); return I2C_NAK; }

    i2c_repeated_start();
    if (!i2c_write_byte(slave | 0x01)) { i2c_stop(); return I2C_NAK; }

    for (i = 0; i < len; i++)
        buf[i] = i2c_read_byte(i < len - 1);

    i2c_stop();
    return I2C_OK;
}

/* CMD 0x1D: E-DDC segment write.
 * START, 0x60, segment, RE-START, slave, offset,
 * data[0..N-1], STOP + write verify */
BYTE i2c_eddc_write(BYTE segment, BYTE slave, BYTE offset,
                     __xdata BYTE *data, WORD len) {
    WORD i;

    i2c_start();
    if (!i2c_write_byte(0x60))    { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(segment)) { i2c_stop(); return I2C_NAK; }

    i2c_repeated_start();
    if (!i2c_write_byte(slave))   { i2c_stop(); return I2C_NAK; }
    if (!i2c_write_byte(offset))  { i2c_stop(); return I2C_NAK; }

    for (i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) { i2c_stop(); return I2C_NAK; }
    }

    i2c_stop();

    if (!i2c_write_verify())
        return I2C_WNAK;

    return I2C_OK;
}
