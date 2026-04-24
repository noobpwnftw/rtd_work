#ifndef I2C_BITBANG_H
#define I2C_BITBANG_H

#include <fx2types.h>

/* Status codes (protocol-compatible with original firmware) */
#define I2C_OK       0x00
#define I2C_NAK      0x01
#define I2C_WNAK     0x02

/* Configure delay ticks (unified timing) */
void i2c_set_delay(WORD ticks);

/* I2C primitives (implemented in i2c_fast.a51) */
void i2c_start(void);
void i2c_stop(void);
void i2c_repeated_start(void);
BOOL i2c_write_byte(BYTE b);   /* returns TRUE if ACK received */
BYTE i2c_read_byte(BOOL ack);  /* ack=TRUE sends ACK, FALSE sends NAK */
BOOL i2c_write_verify(void);   /* post-write clock-stretch check */

/* High-level transactions — all return status code */
BYTE i2c_read_1sub(BYTE slave, BYTE sub, __xdata BYTE *buf, WORD len);
BYTE i2c_write_1sub(BYTE slave, BYTE sub, __xdata BYTE *data, WORD len);
BYTE i2c_write_1sub_nowait(BYTE slave, BYTE sub, __xdata BYTE *data, WORD len);
BYTE i2c_read_2sub(BYTE slave, BYTE sub_hi, BYTE sub_lo, __xdata BYTE *buf, WORD len);
BYTE i2c_read_2sub_wide(BYTE slave, BYTE sub_hi, BYTE sub_lo, __xdata BYTE *buf, WORD len);
BYTE i2c_write_2sub_wide(BYTE slave, BYTE sub_hi, BYTE sub_lo, __xdata BYTE *data, WORD len);
BYTE i2c_read_current(BYTE slave, __xdata BYTE *buf, WORD len);
BYTE i2c_eddc_read(BYTE segment, BYTE slave, BYTE offset, __xdata BYTE *buf, WORD len);
BYTE i2c_eddc_write(BYTE segment, BYTE slave, BYTE offset, __xdata BYTE *data, WORD len);

#endif
