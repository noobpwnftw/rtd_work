/*
 * OpenRTD — open-source FX2LP USB-I2C firmware.
 * Protocol-compatible with Realtek realsil.hex (commands 0x11-0x1B).
 * Adds CMD 0x1C: E-DDC segment read for full EDID block access.
 *
 * Hardware: Cypress CY7C68013A (FX2LP), 48MHz / 4 = 12 MIPS 8051 core.
 * I2C bit-bang on PA0 (SDA, open-drain) / PA1 (SCL, push-pull).
 * USB bulk: EP4 OUT (commands), EP8 IN (responses).
 *
 * Build: SDCC + fx2lib (github.com/djmuhlestein/fx2lib)
 * Flash: cycfx2prog prg:build/openrtd.ihx run
 */

#include <fx2regs.h>
#include <fx2macros.h>
#include <delay.h>
#include <setupdat.h>
#include <eputils.h>
#include <autovector.h>

#include "i2c_bitbang.h"

#define SYNCDELAY SYNCDELAY4

/* EP4 OUT = command input, EP8 IN = response output (1KB each) */
#define EP4BUF_ADDR 0xF400
#define EP8BUF_ADDR 0xFC00
#define EP_BUF_SIZE 1024

#define ep4buf ((__xdata BYTE *)EP4BUF_ADDR)
#define ep8buf ((__xdata BYTE *)EP8BUF_ADDR)

volatile __bit got_sud;
static WORD ep4_pkt_len;

/* --- Packet helpers --- */

static BYTE checksum(__xdata BYTE *buf, WORD len) {
    BYTE sum = 0;
    WORD i;
    for (i = 0; i < len; i++)
        sum += buf[i];
    return sum;
}

static BOOL validate_checksum(WORD pkt_len) {
    /* pkt_len includes the checksum byte itself */
    if (pkt_len < 2) return FALSE;
    return checksum(ep4buf, pkt_len - 1) == ep4buf[pkt_len - 1];
}

static BYTE validate_exact_packet(WORD pkt_len) {
    if (ep4_pkt_len != pkt_len)
        return 0x03;
    return validate_checksum(pkt_len) ? 0x00 : 0x05;
}

static BYTE validate_variable_packet(WORD base_len, WORD data_len) {
    if (data_len > (EP_BUF_SIZE - base_len - 1))
        return 0x03;

    WORD pkt_len = base_len + data_len + 1;  /* base + data + checksum */
    if (ep4_pkt_len != pkt_len)
        return 0x03;
    return validate_checksum(pkt_len) ? 0x00 : 0x05;
}

static void send_response(WORD data_len, BYTE status) {
    WORD total = data_len + 2;  /* data + status + checksum */
    ep8buf[data_len] = status;
    ep8buf[data_len + 1] = checksum(ep8buf, data_len + 1);
    EP8BCH = MSB(total);
    SYNCDELAY;
    EP8BCL = LSB(total);
    SYNCDELAY;
}

static void arm_ep4(void) {
    EP4BCL = 0x80;
    SYNCDELAY;
}

/* Max payload that fits in EP8 response (data + status + checksum) */
#define MAX_READ_LEN  (EP_BUF_SIZE - 2)

/* --- Command handlers --- */

/* CMD 0x11: I2C read, 1-byte sub */
static void cmd_read_1sub(void) {
    BYTE slave = ep4buf[1];
    BYTE sub   = ep4buf[2];
    WORD len   = MAKEWORD(ep4buf[3], ep4buf[4]);
    BYTE v;
    BYTE st;

    v = validate_exact_packet(6);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }
    if (len > MAX_READ_LEN) { send_response(0, 0x03); arm_ep4(); return; }

    st = i2c_read_1sub(slave, sub, ep8buf, len);
    send_response(st == 0x00 ? len : 0, st);
    arm_ep4();
}

/* CMD 0x12: I2C write, 1-byte sub */
static void cmd_write_1sub(void) {
    BYTE slave = ep4buf[1];
    BYTE sub   = ep4buf[2];
    WORD len   = MAKEWORD(ep4buf[3], ep4buf[4]);
    BYTE v;
    BYTE st;

    v = validate_variable_packet(5, len);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }

    st = i2c_write_1sub(slave, sub, &ep4buf[5], len);
    send_response(0, st);
    arm_ep4();
}

/* CMD 0x13: I2C read, 2-byte sub (single-phase addressing)
 * Packet: [0x13, slave, sub_hi, sub_lo, len_hi, len_lo, csum] */
static void cmd_read_2sub(void) {
    BYTE slave  = ep4buf[1];
    BYTE sub_hi = ep4buf[2];
    BYTE sub_lo = ep4buf[3];
    WORD len    = MAKEWORD(ep4buf[4], ep4buf[5]);
    BYTE v;
    BYTE st;

    v = validate_exact_packet(7);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }
    if (len > MAX_READ_LEN) { send_response(0, 0x03); arm_ep4(); return; }

    st = i2c_read_2sub(slave, sub_hi, sub_lo, ep8buf, len);
    send_response(st == 0x00 ? len : 0, st);
    arm_ep4();
}

/* CMD 0x14: I2C write with inline data, no post-write wait */
static void cmd_write_inline(void) {
    BYTE slave = ep4buf[1];
    BYTE sub   = ep4buf[2];
    WORD len   = MAKEWORD(ep4buf[3], ep4buf[4]);
    BYTE v;
    BYTE st;

    /* Data starts at ep4buf[5], checksum at ep4buf[5+len] */
    v = validate_variable_packet(5, len);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }

    st = i2c_write_1sub_nowait(slave, sub, &ep4buf[5], len);
    send_response(0, st);
    arm_ep4();
}

/* CMD 0x15: Current-address read */
static void cmd_read_current(void) {
    BYTE slave = ep4buf[1];
    WORD len   = MAKEWORD(ep4buf[2], ep4buf[3]);
    BYTE v;
    BYTE st;

    v = validate_exact_packet(5);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }
    if (len > MAX_READ_LEN) { send_response(0, 0x03); arm_ep4(); return; }

    st = i2c_read_current(slave, ep8buf, len);
    send_response(st == 0x00 ? len : 0, st);
    arm_ep4();
}

/* CMD 0x16: Set I2C clock divider.
 * Stock firmware stores this in RAM (0x41/0x42) and uses it to select
 * timing function pointers. We translate it to a NOP sled length:
 * divider / 8 gives a reasonable tick count (e.g. 200 → 25 NOPs). */
static void cmd_set_clk_div(void) {
    WORD div;
    BYTE v;

    v = validate_exact_packet(4);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }

    div = MAKEWORD(ep4buf[1], ep4buf[2]);
    i2c_set_delay(div >> 3);
    send_response(0, 0x00);
    arm_ep4();
}

/* CMD 0x17: Configure timing (computed from speed) */
static void cmd_set_timing(void) {
    BYTE speed_hi;
    WORD ticks;
    BYTE v;

    v = validate_exact_packet(7);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }

    /* Speed is 32-bit big-endian at ep4buf[2..5].
     * Classify by high bytes to avoid DWORD math on 8051.
     * Ticks = NOP count in JIT delay sled (1 cycle each at 12 MIPS).
     * 400000 (0x00061A80): [3]=0x06+  →  5 NOPs
     * 100000 (0x000186A0): [3]=0x01+  → 25 NOPs
     *  50000 (0x0000C350): [4]=0xC0+  → 55 NOPs */
    speed_hi = ep4buf[3];
    if (ep4buf[2] > 0 || speed_hi >= 0x06)
        ticks = 5;        /* >= ~400kHz */
    else if (speed_hi >= 0x01)
        ticks = 25;       /* >= ~65kHz (covers 100kHz) */
    else if (ep4buf[4] >= 0xC0)
        ticks = 55;       /* >= ~49kHz */
    else
        ticks = 120;      /* slow */

    i2c_set_delay(ticks);
    send_response(0, 0x00);
    arm_ep4();
}

/* CMD 0x18: Configure timing (raw tick override) */
static void cmd_set_timing_raw(void) {
    WORD ticks;
    BYTE v;

    v = validate_exact_packet(7);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }

    /* Use low 16 bits of the 32-bit value */
    ticks = MAKEWORD(ep4buf[4], ep4buf[5]);
    if (ticks < 5) ticks = 5;

    i2c_set_delay(ticks);
    send_response(0, 0x00);
    arm_ep4();
}

/* Bootloader device descriptor — defined in dscr.a51 with .even alignment.
 * SUDPTRH:L requires word-aligned (even) addresses per FX2 TRM. */
extern __code BYTE bl_dev_dscr[];

static __bit bootloader_mode;

/* CMD 0x19: Drop to bootloader for firmware reload.
 * Swaps device descriptor to 04b4:8613 and re-enumerates.
 * Vendor request 0xA0 (RAM read/write) is not handled by firmware,
 * so the FX2 hardware handles it — host can upload new firmware. */
static void cmd_bootloader(void) {
    BYTE v = validate_exact_packet(2);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }

    bootloader_mode = TRUE;
    USBCS |= (bmDISCON | bmRENUM);
    delay(1500);
    USBIRQ = 0xFF;
    EPIRQ  = 0xFF;
    EXIF  &= ~0x10;
    USBCS &= ~bmDISCON;
}

/* CMD 0x1A: I2C read, 2-byte sub (3-phase)
 * Packet: [0x1A, slave, sub_hi, sub_lo, pad, len_hi, len_lo, csum] */
static void cmd_read_2sub_wide(void) {
    BYTE slave  = ep4buf[1];
    BYTE sub_hi = ep4buf[2];
    BYTE sub_lo = ep4buf[3];
    /* ep4buf[4] is padding/reserved */
    WORD len    = MAKEWORD(ep4buf[5], ep4buf[6]);
    BYTE v;
    BYTE st;

    v = validate_exact_packet(8);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }
    if (len > MAX_READ_LEN) { send_response(0, 0x03); arm_ep4(); return; }

    st = i2c_read_2sub_wide(slave, sub_hi, sub_lo, ep8buf, len);
    send_response(st == 0x00 ? len : 0, st);
    arm_ep4();
}

/* CMD 0x1B: I2C write, 2-byte sub (3-phase)
 * Packet: [0x1B, slave, sub_hi, sub_lo, pad, len_hi, len_lo, data..., csum] */
static void cmd_write_2sub_wide(void) {
    BYTE slave  = ep4buf[1];
    BYTE sub_hi = ep4buf[2];
    BYTE sub_lo = ep4buf[3];
    /* ep4buf[4] is padding/reserved */
    WORD len    = MAKEWORD(ep4buf[5], ep4buf[6]);
    BYTE v;
    BYTE st;

    v = validate_variable_packet(7, len);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }

    st = i2c_write_2sub_wide(slave, sub_hi, sub_lo, &ep4buf[7], len);
    send_response(0, st);
    arm_ep4();
}

/* CMD 0x1C: E-DDC segment read (NEW) */
static void cmd_eddc_read(void) {
    BYTE segment = ep4buf[1];
    BYTE slave   = ep4buf[2];
    BYTE offset  = ep4buf[3];
    WORD len     = MAKEWORD(ep4buf[4], ep4buf[5]);
    BYTE v;
    BYTE st;

    v = validate_exact_packet(7);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }
    if (len > MAX_READ_LEN) { send_response(0, 0x03); arm_ep4(); return; }

    st = i2c_eddc_read(segment, slave, offset, ep8buf, len);
    send_response(st == 0x00 ? len : 0, st);
    arm_ep4();
}

/* CMD 0x1D: E-DDC segment write (NEW)
 * Packet: [0x1D, segment, slave, offset, len_hi, len_lo, data..., csum] */
static void cmd_eddc_write(void) {
    BYTE segment = ep4buf[1];
    BYTE slave   = ep4buf[2];
    BYTE offset  = ep4buf[3];
    WORD len     = MAKEWORD(ep4buf[4], ep4buf[5]);
    BYTE v;
    BYTE st;

    v = validate_variable_packet(6, len);
    if (v != 0x00) { send_response(0, v); arm_ep4(); return; }

    st = i2c_eddc_write(segment, slave, offset, &ep4buf[6], len);
    send_response(0, st);
    arm_ep4();
}

/* --- Command dispatch --- */

static void process_command(void) {
    /* Check if EP4 has data */
    if (EP2468STAT & bmEP4EMPTY)
        return;

    ep4_pkt_len = MAKEWORD(EP4BCH, EP4BCL);
    if (ep4_pkt_len < 1 || ep4_pkt_len > EP_BUF_SIZE) {
        send_response(0, 0x03);
        arm_ep4();
        return;
    }

    if (EP2468STAT & bmEP8FULL) {
        send_response(0, 0x04);
        arm_ep4();
        return;
    }

    switch (ep4buf[0]) {
        case 0x11: cmd_read_1sub();      break;
        case 0x12: cmd_write_1sub();     break;
        case 0x13: cmd_read_2sub();      break;
        case 0x14: cmd_write_inline();   break;
        case 0x15: cmd_read_current();   break;
        case 0x16: cmd_set_clk_div();    break;
        case 0x17: cmd_set_timing();     break;
        case 0x18: cmd_set_timing_raw(); break;
        case 0x19: cmd_bootloader();     break;
        case 0x1A: cmd_read_2sub_wide(); break;
        case 0x1B: cmd_write_2sub_wide();break;
        case 0x1C: cmd_eddc_read();      break;
        case 0x1D: cmd_eddc_write();     break;
        default:
        {
            send_response(0, 0x06);
            arm_ep4();
            break;
        }
    }
}

/* --- fx2lib callbacks (required by setupdat.c) --- */

BOOL handle_vendorcommand(BYTE cmd) {
    (void)cmd;
    return FALSE;
}

BOOL handle_get_descriptor(void) {
    if (bootloader_mode && SETUPDAT[3] == 1) {  /* device descriptor */
        SUDPTRH = MSB((__code WORD)&bl_dev_dscr);
        SUDPTRL = LSB((__code WORD)&bl_dev_dscr);
        return TRUE;
    }
    return FALSE;
}

BOOL handle_get_interface(BYTE ifc, BYTE *alt_ifc) {
    if (ifc == 0) { *alt_ifc = 0; return TRUE; }
    return FALSE;
}

BOOL handle_set_interface(BYTE ifc, BYTE alt_ifc) {
    return ifc == 0 && alt_ifc == 0;
}

BYTE handle_get_configuration(void) { return 1; }

BOOL handle_set_configuration(BYTE cfg) {
    return cfg == 1;
}

void handle_reset_ep(BYTE ep) {
    /* EP4 and EP8 are quad-buffered (EPxCFG[1:0]=00), so skip 4 buffers each
     * per Cypress AN58069. RESETFIFO already brackets with NAKALL. */
    if (ep == 0x04) {
        RESETTOGGLE(0x04); SYNCDELAY;
        RESETFIFO(0x04);   SYNCDELAY;
        EP4BCL = 0x80;     SYNCDELAY;   /* SKIP: discard committed OUT buffer 1 */
        EP4BCL = 0x80;     SYNCDELAY;   /* buffer 2 */
        EP4BCL = 0x80;     SYNCDELAY;   /* buffer 3 */
        EP4BCL = 0x80;     SYNCDELAY;   /* buffer 4 */
    } else if (ep == 0x88) {
        RESETTOGGLE(0x88); SYNCDELAY;
        RESETFIFO(0x88);   SYNCDELAY;
        INPKTEND = 0x88;   SYNCDELAY;   /* SKIP | EP8: drop pending IN buffer 1 */
        INPKTEND = 0x88;   SYNCDELAY;   /* buffer 2 */
        INPKTEND = 0x88;   SYNCDELAY;   /* buffer 3 */
        INPKTEND = 0x88;   SYNCDELAY;   /* buffer 4 */
    }
}

/* --- Main --- */

void main(void) {
    REVCTL = 0;
    got_sud = FALSE;
    bootloader_mode = FALSE;

    RENUMERATE_UNCOND();

    SETCPUFREQ(CLK_48M);
    SETIF48MHZ();

    USE_USB_INTS();
    ENABLE_SUDAV();
    ENABLE_HISPEED();
    ENABLE_USBRESET();

    /* Configure endpoints: only EP4 OUT and EP8 IN are valid */
    EP1OUTCFG &= ~bmVALID; SYNCDELAY;
    EP1INCFG  &= ~bmVALID; SYNCDELAY;
    EP2CFG    &= ~bmVALID; SYNCDELAY;
    EP4CFG     = 0xA0;     SYNCDELAY;  /* Valid, OUT, Bulk, 512B, quad-buf */
    EP6CFG    &= ~bmVALID; SYNCDELAY;
    EP8CFG     = 0xE0;     SYNCDELAY;  /* Valid, IN, Bulk, 512B, quad-buf */

    /* Arm EP4 OUT: skip all 4 buffers so they're ready to receive */
    EP4BCL = 0x80; SYNCDELAY;
    EP4BCL = 0x80; SYNCDELAY;
    EP4BCL = 0x80; SYNCDELAY;
    EP4BCL = 0x80; SYNCDELAY;

    /* I2C bit-bang init: PA0 (SDA), PA1 (SCL)
     * SCL: push-pull output (OEA.1=1 always), idle high via IOA.1=1
     * SDA: open-drain via OEA.0 toggle, latch=1 for correct pin readback */
    IOA |= 0x03;   /* Both latches high (SCL idle high, SDA released) */
    OEA = 0xFE;    /* SDA=input (released), SCL=output (push-pull) */

    /* Let I2C bus settle after re-enumeration */
    delay(50);

    /* Default I2C timing ~100kHz */
    i2c_set_delay(25);

    EA = 1;  /* Global interrupt enable */

    while (TRUE) {
        if (got_sud) {
            handle_setupdata();
            got_sud = FALSE;
        }
        process_command();
    }
}

/* --- ISRs --- */

void sudav_isr(void) __interrupt (SUDAV_ISR) {
    got_sud = TRUE;
    CLEAR_SUDAV();
}

void usbreset_isr(void) __interrupt (USBRESET_ISR) {
    handle_hispeed(FALSE);

    /* Re-init endpoints — USB bus reset clears EP configs */
    EP1OUTCFG &= ~bmVALID; SYNCDELAY;
    EP1INCFG  &= ~bmVALID; SYNCDELAY;
    EP2CFG    &= ~bmVALID; SYNCDELAY;
    EP4CFG     = 0xA0;     SYNCDELAY;  /* Valid, OUT, Bulk, 512B, quad-buf */
    EP6CFG    &= ~bmVALID; SYNCDELAY;
    EP8CFG     = 0xE0;     SYNCDELAY;  /* Valid, IN, Bulk, 512B, quad-buf */

    /* Arm all 4 OUT buffers */
    EP4BCL = 0x80; SYNCDELAY;
    EP4BCL = 0x80; SYNCDELAY;
    EP4BCL = 0x80; SYNCDELAY;
    EP4BCL = 0x80; SYNCDELAY;

    CLEAR_USBRESET();
}

void hispeed_isr(void) __interrupt (HISPEED_ISR) {
    handle_hispeed(TRUE);
    CLEAR_HISPEED();
}
