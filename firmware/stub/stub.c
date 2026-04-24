/*
 * RTD 8051 MCU stub firmware — I2C slave debug bridge on DDC3.
 * SDCC small model.
 *
 * Build: sdcc -mmcs51 --no-xinit-opt stub.c
 */

typedef unsigned char  BYTE;
typedef unsigned short WORD;

/* =====================================================================
 * 8051 SFRs
 * ===================================================================== */

__sfr __at(0x88) TCON;
__sfr __at(0x90) P1;
__sfr __at(0xB0) P3;
__sfr __at(0x89) TMOD;
__sfr __at(0x8E) CKCON;
__sfr __at(0xA8) IE;
__sfr __at(0xC8) T2CON;

__sbit __at(0x8C) TR0;
__sbit __at(0x8D) TF0;
__sbit __at(0x8E) TR1;
__sbit __at(0x8F) TF1;
__sbit __at(0xA8) EX0;
__sbit __at(0xAA) EX1;
__sbit __at(0xAF) EA;
__sbit __at(0xCA) TR2;
__sbit __at(0xCB) RCLK;
__sbit __at(0xCC) TCLK;
__sbit __at(0xCF) TF2;

#define _BIT0  0x01
#define _BIT1  0x02
#define _BIT2  0x04
#define _BIT3  0x08
#define _BIT4  0x10
#define _BIT5  0x20
#define _BIT6  0x40
#define _BIT7  0x80

/* =====================================================================
 * XDATA register access
 * ===================================================================== */

#define XREG(addr) (*(volatile BYTE __xdata *)(addr))
#define ScalerGetByte(addr)      (*((volatile BYTE __xdata *)(addr)))
#define ScalerSetByte(addr, val) (*((volatile BYTE __xdata *)(addr)) = (val))

/* =====================================================================
 * RTD 8051 MCU XFR registers (XDATA 0xFDxx–0xFFxx)
 * ===================================================================== */

/* SCA_INF: scaler indirect register access (XFR) */
#define MCU_FFF3_SCA_INF_CONTROL     XREG(0xFFF3)
#define MCU_FFF4_SCA_INF_ADDR        XREG(0xFFF4)
#define MCU_FFF5_SCA_INF_DATA        XREG(0xFFF5)
#define MCU_FF35_DDCCI_REMAIN_DATA   XREG(0xFF35)

/* Cache controller (page FD) */
#define MCU_FDA0_CACHE_CTRL          XREG(0xFDA0)
#define MCU_FDA1_CACHE_CTRL_1        XREG(0xFDA1)

/* DDC3 (page FE) */
#define MCU_FE6F_DDC3_CONTROL_2      XREG(0xFE6F)

/* DDC-CI slave 1 (page FE) */
#define MCU_FE86_IIC_1_CH_SEL        XREG(0xFE86)
#define MCU_FE87_IIC_1_SET_SLAVE     XREG(0xFE87)

/* DDC-CI slave 0 (page FF) */
#define MCU_FF22_IIC_CH_SEL          XREG(0xFF22)
#define MCU_FF23_IIC_SET_SLAVE       XREG(0xFF23)
#define MCU_FF24_IIC_SUB_IN          XREG(0xFF24)
#define MCU_FF25_IIC_DATA_IN         XREG(0xFF25)
#define MCU_FF26_IIC_DATA_OUT        XREG(0xFF26)
#define MCU_FF27_IIC_STATUS          XREG(0xFF27)
#define MCU_FF28_IIC_IRQ_CONTROL     XREG(0xFF28)
#define MCU_FF29_IIC_STATUS2         XREG(0xFF29)
#define MCU_FF2A_IIC_IRQ_CONTROL2    XREG(0xFF2A)

/* Watchdog */
#define MCU_FF3A_WDT_TEST            XREG(0xFF3A)
#define MCU_FFB8_EXT_INT0_CTRL       XREG(0xFFB8)
#define MCU_FFB9_EXT_INT1_CTRL       XREG(0xFFB9)
#define MCU_FFC4_IRQ_PRIORITY_3      XREG(0xFFC4)
#define MCU_FFE9_MCU_CLK_CTRL_1      XREG(0xFFE9)
#define MCU_FFEA_WATCHDOG_TIMER      XREG(0xFFEA)
#define MCU_FFED_MCU_CONTROL         XREG(0xFFED)
#define MCU_FFEE_MCU_CLOCK_CONTROL   XREG(0xFFEE)
#define MCU_FFFC_BANK_SWITCH_CTRL    XREG(0xFFFC)
#define MCU_FFFD_XDATA_BANK_START    XREG(0xFFFD)
#define MCU_FFFE_XDATA_BANK_SEL      XREG(0xFFFE)

/* =====================================================================
 * Dummy ISR handlers — safe RETI for unused interrupt vectors
 * ===================================================================== */

void int0_isr(void)   __interrupt(0) { }
void timer0_isr(void) __interrupt(1) { }
void timer1_isr(void) __interrupt(3) { }
void serial_isr(void) __interrupt(4) { }
void timer2_isr(void) __interrupt(5) { }

/* =====================================================================
 * Globals (IRAM — small model)
 * ===================================================================== */

static volatile BYTE g_bRunCommand;
static volatile BYTE g_ucCmdCount;
static volatile BYTE g_data[4]; /* [0]=sub [1]=data [2]=page [3]=val */

/* =====================================================================
 * INT1 ISR — DDC-CI debug command receive
 *
 * Receives I2C writes from the host debug tool:
 *   SUB_I  → latch sub-address (command byte)
 *   D_IN_I → accumulate data bytes
 *   STOP_I → hold SCL, signal main loop to process
 *
 * When a previous command is still being processed (g_bRunCommand),
 * just drain and acknowledge incoming traffic.
 * ===================================================================== */

void int1_isr(void) __interrupt(2) __using(2)
{
    BYTE st;

    /* W1C int1_status so simulator doesn't see stale DDC events */
    MCU_FFB9_EXT_INT1_CTRL |= _BIT3;

    /* Clear buffer overflow/underflow flags */
    MCU_FF29_IIC_STATUS2 &= ~(_BIT5 | _BIT4);

    if (g_bRunCommand == 0) {
        st = MCU_FF27_IIC_STATUS;

        /* SUB_I — sub-address received */
        if ((st & _BIT1) != 0) {
            MCU_FF27_IIC_STATUS = 0xBD;          /* clear SUB_I only */
            g_ucCmdCount = 0;
            g_data[0] = MCU_FF24_IIC_SUB_IN;
            MCU_FF2A_IIC_IRQ_CONTROL2 &= ~_BIT5; /* host write enable */
        }

        /* D_IN_I — data byte(s) received */
        if ((st & _BIT2) != 0) {
            while ((MCU_FF29_IIC_STATUS2 & _BIT1) == 0) {
                if ((g_ucCmdCount + 1) < sizeof(g_data)) {
                    g_ucCmdCount++;
                } else {
                    MCU_FF2A_IIC_IRQ_CONTROL2 |= _BIT6; /* reset buffer */
                    g_ucCmdCount = 0;
                    break;
                }
                g_data[g_ucCmdCount] = MCU_FF25_IIC_DATA_IN;
            }
            MCU_FF27_IIC_STATUS = 0xBB;          /* clear D_IN_I */
        }

        /* STOP_I — end of write transaction */
        if ((st & _BIT4) == _BIT4) {
            MCU_FF27_IIC_STATUS = 0xAF;          /* clear STOP_I */
            if (g_ucCmdCount > 0) {
                MCU_FE6F_DDC3_CONTROL_2 |= _BIT0;    /* hold SCL */
                MCU_FF2A_IIC_IRQ_CONTROL2 |= _BIT5;  /* MCU write enable */
                g_bRunCommand = 1;
            }
        }
    } else {
        /* Drain path — previous command still processing */
        if ((MCU_FF27_IIC_STATUS & _BIT1) == _BIT1)
            MCU_FF27_IIC_STATUS &= ~_BIT1;       /* clear SUB_I */
        if ((MCU_FF27_IIC_STATUS & _BIT2) == _BIT2)
            (void)MCU_FF25_IIC_DATA_IN;           /* drain data */
        if ((MCU_FF27_IIC_STATUS & _BIT4) == _BIT4)
            MCU_FF27_IIC_STATUS &= ~_BIT4;       /* clear STOP_I */
    }
}

/* =====================================================================
 * Post-command delay (~30us)
 * Matches vendor firmware timing between clearing host-write-enable
 * and releasing SCL.
 * ===================================================================== */

static void delay_30us(void)
{
    volatile BYTE i;
    for (i = 0; i < 150; i++);
}

/* =====================================================================
 * MCU init — matches vendor ScalerMcuInitial sequence
 * ===================================================================== */

static void mcu_init(void)
{
    /* Disable ALL three watchdog enables + clear counter.
     * RTD 8051 MCU has three independent enable bits — any one keeps it alive.
     * Without Timer2 ISR to feed it, we must kill all three. */
    MCU_FFEA_WATCHDOG_TIMER &= ~_BIT7;                      /* wdt_en */
    MCU_FFEA_WATCHDOG_TIMER |= _BIT6 | _BIT2 | _BIT1 | _BIT0; /* clr + cnt1=max */
    MCU_FFE9_MCU_CLK_CTRL_1 &= ~_BIT7;                      /* wdt_en_3 */
    MCU_FF3A_WDT_TEST &= ~_BIT6;                             /* wdt_en_2 */

    /* Clock: crystal source, all dividers = 1 (max speed) */
    MCU_FFE9_MCU_CLK_CTRL_1 &= ~_BIT4;                      /* M2PLL mux */
    MCU_FFED_MCU_CONTROL &= ~_BIT1;                          /* crystal source */
    MCU_FFED_MCU_CONTROL = (MCU_FFED_MCU_CONTROL & 0xC3) | _BIT2; /* flash_clk_div=1 */
    MCU_FFE9_MCU_CLK_CTRL_1 = (MCU_FFE9_MCU_CLK_CTRL_1 & 0xF0);  /* flash_clk_div2=0 */
    MCU_FFEE_MCU_CLOCK_CONTROL = (MCU_FFEE_MCU_CLOCK_CONTROL & 0xC3) | _BIT2; /* mcu_clk_div=1 */

    /* XDATA address remapping to scaler register space */
    MCU_FFFC_BANK_SWITCH_CTRL |= _BIT4 | _BIT3 | _BIT2 | _BIT1 | _BIT0;
    MCU_FFFD_XDATA_BANK_START = 0x00;
    MCU_FFFE_XDATA_BANK_SEL = 0x00;

    /* Code cache */
    MCU_FDA1_CACHE_CTRL_1 |= _BIT0;     /* reset */
    MCU_FDA1_CACHE_CTRL_1 &= ~_BIT0;    /* release */
    MCU_FDA1_CACHE_CTRL_1 &= ~_BIT1;    /* disable common bank */
    MCU_FDA0_CACHE_CTRL |= _BIT0;       /* enable */

    /* Clean 8051 timer/interrupt state */
    CKCON &= ~0x3F;
    IE = 0x00;
    TR0 = 0; TF0 = 0;
    TR1 = 0; TF1 = 0;
    TR2 = 0; TF2 = 0;
    TCLK = 0; RCLK = 0;
    TMOD = 0x11;
    T2CON &= ~_BIT0;
    EA = 1;

    /* Enable EX0 — scaler interrupt, empty ISR for simulator proxy */
    EX0 = 1;
}

/* =====================================================================
 * DDC-CI init — matches vendor ScalerMcuDdcciInitial
 * ===================================================================== */

static void ddcci_init(void)
{
    /* Debug tool slave address */
    MCU_FF23_IIC_SET_SLAVE = 0x6A;

    /* Disable DDC-CI slave 1 */
    MCU_FE87_IIC_1_SET_SLAVE = 0x3D;
    MCU_FE86_IIC_1_CH_SEL |= (_BIT2 | _BIT1 | _BIT0);

    /* Select DDC3 channel for DDC-CI slave 0 */
    MCU_FF22_IIC_CH_SEL &= ~(_BIT2 | _BIT1 | _BIT0);
    MCU_FF22_IIC_CH_SEL |= _BIT2;

    /* Route DDC-CI interrupt to INT1 */
    MCU_FFC4_IRQ_PRIORITY_3 &= ~(_BIT2 | _BIT1 | _BIT0);
    MCU_FFC4_IRQ_PRIORITY_3 |= _BIT0;

    /* Enable auto buffer reset */
    MCU_FF2A_IIC_IRQ_CONTROL2 |= _BIT7;

    /* Clear DDC-CI status */
    MCU_FF27_IIC_STATUS = 0x00;
    (void)MCU_FF25_IIC_DATA_IN;
    MCU_FF26_IIC_DATA_OUT = 0x00;
    MCU_FF29_IIC_STATUS2 &= ~(_BIT5 | _BIT4);
    MCU_FF2A_IIC_IRQ_CONTROL2 |= _BIT6;

    /* Enable DDC-CI interrupts: STOP_I, D_IN_I, SUB_I */
    EX1 = 1;
    MCU_FF28_IIC_IRQ_CONTROL |= (_BIT4 | _BIT2 | _BIT1);
}

/* =====================================================================
 * Debug command dispatcher
 *
 * Host-side protocol (each step is one I2C write/read):
 *   Read:  sub=0x01 data=page, sub=0x3A data=reg, read sub=0x08
 *   Write: sub=0x01 data=page, sub=0x03 data=val, sub=0x3B data=reg
 *   Enter: sub=0x80 data=0x01
 *   Exit:  sub=0x80 data=0x00
 * ===================================================================== */

static void cmd_complete(void)
{
    g_bRunCommand = 0;
    g_ucCmdCount = 0;
    MCU_FF2A_IIC_IRQ_CONTROL2 &= ~_BIT5;
    delay_30us();
    MCU_FE6F_DDC3_CONTROL_2 &= ~_BIT0;
}

static void dispatch(void)
{
    WORD addr;

    switch (g_data[0]) {
    case 0x80:                                       /* halt/mode control */
        break;

    case 0x3A:                                       /* read scaler reg */
        addr = ((WORD)g_data[2] << 8) + g_data[1];
        MCU_FF26_IIC_DATA_OUT = ScalerGetByte(addr);
        break;

    case 0x3B:                                       /* write scaler reg */
        addr = ((WORD)g_data[2] << 8) + g_data[1];
        ScalerSetByte(addr, g_data[3]);
        break;

    case 0x41:                                       /* SCA_INF read (XFR) */
        MCU_FFF3_SCA_INF_CONTROL |= _BIT5;
        MCU_FFF4_SCA_INF_ADDR = 0x9F;
        MCU_FFF5_SCA_INF_DATA = g_data[2];          /* set page from staged value */
        MCU_FFF4_SCA_INF_ADDR = g_data[1];
        MCU_FF26_IIC_DATA_OUT = MCU_FFF5_SCA_INF_DATA;
        break;

    case 0x42:                                       /* read interrupt regs */
        MCU_FF26_IIC_DATA_OUT = ScalerGetByte(0xFFB8);
        MCU_FF26_IIC_DATA_OUT = ScalerGetByte(0xFFB9);
        MCU_FF26_IIC_DATA_OUT = ScalerGetByte(0xFF00);
        break;

    case 0x5A:                                       /* write MCU SFR port */
        switch (g_data[1]) {
        case 0x90: P1 = g_data[3]; break;            /* classic SFR P1 */
        case 0xB0: P3 = g_data[3]; break;            /* classic SFR P3 */
        }
        break;

    case 0x5B:                                       /* read MCU SFR port */
        switch (g_data[1]) {
        case 0x90: MCU_FF26_IIC_DATA_OUT = P1; break;
        case 0xB0: MCU_FF26_IIC_DATA_OUT = P3; break;
        default:   MCU_FF26_IIC_DATA_OUT = 0; break;
        }
        break;

    default:                                         /* staging commands */
        switch (g_data[0] & 0x0F) {
        case 0x00:                                   /* SCA_INF write (XFR) */
            MCU_FFF3_SCA_INF_CONTROL |= _BIT5;
            MCU_FFF4_SCA_INF_ADDR = 0x9F;
            addr = ((WORD)MCU_FFF5_SCA_INF_DATA << 8) + g_data[2];
            ScalerSetByte(addr, g_data[1]);
            break;
        case 0x01: g_data[2] = g_data[1]; break;    /* set page */
        case 0x03: g_data[3] = g_data[1]; break;    /* set value */
        }
        break;
    }
}

void main(void)
{
    /* IRAM clear must be inline — it wipes the stack */
    __asm
        mov  r0, #0xFF
        clr  a
    00099$:
        mov  @r0, a
        djnz r0, 00099$
        mov  @r0, a
        mov  sp, #0x66
    __endasm;

    mcu_init();
    ddcci_init();

    g_bRunCommand = 0;
    g_ucCmdCount = 0;

    for (;;) {
        if (g_bRunCommand != 1)
            continue;

        dispatch();
        cmd_complete();
    }
}
