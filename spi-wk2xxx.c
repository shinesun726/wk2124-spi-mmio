/*
 *   FILE NAME  : wk2xxx_spi.c
 *
 *   WKIC Ltd.
 *   By  Xu XunWei Tech
 *   DEMO Version :2.4 Data:2022-07-24
 *   DESCRIPTION: Implements an interface for the wk2xxx of spi interface
 *
 *
 *
 */
#include <linux/init.h>                        
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/spi/spi.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/of_clk.h>
#include <linux/pm_runtime.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/platform_data/spi-rockchip.h>
//#include "wk2xxx.h"
#include "linux/version.h"
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <uapi/linux/sched.h>

#include <uapi/linux/sched/types.h>

MODULE_LICENSE("Dual BSD/GPL");

#define DRIVER_AUTHOR "Xuxunwei"
#define DRIVER_DESC "SPI driver for spi to Uart chip WK2XXX, etc."
#define VERSION_DESC "V2.4 On 2022.07.24"
/*************The debug control **********************************/
//#define _DEBUG_WK_FUNCTION
//#define _DEBUG_WK_RX
//#define _DEBUG_WK_TX
//#define _DEBUG_WK_IRQ
//#define _DEBUG_WK_VALUE
//#define _DEBUG_WK_TEST

/*************Functional control interface************************/
#define WK_FIFO_FUNCTION
//#define WK_FlowControl_FUNCTION
#define WK_WORK_KTHREAD
//#define WK_RS485_FUNCTION
//#define WK_RSTGPIO_FUNCTION
//#define WK_CSGPIO_FUNCTION
/*************SPI control interface******************************/
#define SPI_LEN_LIMIT 30 // MAX<=255

/*************Uart Setting interface******************************/
#define WK2XXX_TXFIFO_LEVEL (0x01) /* TX FIFO level */
#define WK2XXX_RXFIFO_LEVEL (0x40) /* RX FIFO level */ 

#define WK2XXX_STATUS_PE 1
#define WK2XXX_STATUS_FE 2
#define WK2XXX_STATUS_BRK 4
#define WK2XXX_STATUS_OE 8

static DEFINE_MUTEX(wk2xxxs_lock);
static DEFINE_MUTEX(wk2xxxs_reg_lock);
static DEFINE_MUTEX(wk2xxxs_global_lock);

#define ROCKCHIP_SPI_VER2_TYPE1 0x05EC0002
#define ROCKCHIP_SPI_VER2_TYPE2 0x00110002

#define SPI_XFER_BEGIN (1 << 0)
#define SPI_XFER_END (1 << 1)
#define SPI_XFER_ONCE (SPI_XFER_BEGIN | SPI_XFER_END)

#define ROCKCHIP_SPI_TIMEOUT_US 1000000

#define WK2XXX_DEBUG_LOG 0

#if WK2XXX_DEBUG_LOG
#define wk2xxx_dbg(dev, fmt, ...) dev_info((dev), "wk2xxx: " fmt, ##__VA_ARGS__)
#define wk2xxx_dbg_rl(dev, fmt, ...) dev_info_ratelimited((dev), "wk2xxx: " fmt, ##__VA_ARGS__)
#else
#define wk2xxx_dbg(dev, fmt, ...) do { } while (0)
#define wk2xxx_dbg_rl(dev, fmt, ...) do { } while (0)
#endif

enum
{
    DFS_SHIFT = 0,
    DFS_8BIT = 1,
    DFS_16BIT = 2,
    SCPH_SHIFT = 6,
    SCPH_TOGSTA = 1,
    SCOL_SHIFT = 7,
    SCOL_HIGH = 1,
    CSM_SHIFT = 8,
    CSM_KEEP = 0,
    SSN_DELAY_SHIFT = 10,
    SSN_DELAY_ONE = 1,
    SEM_SHIFT = 11,
    SEM_LITTLE = 0,
    FBM_SHIFT = 12,
    FBM_MSB = 0,
    HALF_WORD_TX_SHIFT = 13,
    HALF_WORD_ON = 0,
    HALF_WORD_OFF = 1,
    RXDSD_SHIFT = 14,
    FRF_SHIFT = 16,
    FRF_SPI = 0,
    TMOD_SHIFT = 18,
    TMOD_TR = 0,
    TMOD_TO = 1,
    TMOD_RO = 2,
    OMOD_SHIFT = 20,
    OMOD_MASTER = 0,
};

#define SR_BUSY BIT(0)

/******************************************/
#define NR_PORTS 4
//
#define SERIAL_WK2XXX_MAJOR 207
#define CALLOUT_WK2XXX_MAJOR 208
#define MINOR_START 5
// wk2xxx hardware configuration
#define wk2xxx_spi_speed 10000000
//#define 	WK_CRASTAL_CLK		        (24000000)
#define WK_CRASTAL_CLK (11059200)
#define WK2XXX_ISR_PASS_LIMIT 2
#define PORT_WK2XXX 1
/******************************************/

/************** WK2XXX register definitions********************/
/*wk2xxx  Global register address defines*/
#define WK2XXX_GENA_REG 0X00                                      /*Slave UART Clock Set */
#define WK2XXX_GRST_REG 0X01                                      /*Reset Slave UART*/
#define WK2XXX_GMUT_REG 0X02                                      /*Master UART Control*/
#define WK2XXX_GIER_REG 0X10                                      /*Slave UART Interrupt Enable */
#define WK2XXX_GIFR_REG 0X11                                      /*Slave UART Interrupt Flag*/
#define WK2XXX_GPDIR_REG 0X21 /*GPIO Direction*/                  /*WK2168/WK2212*/
#define WK2XXX_GPDAT_REG 0X31 /*GPIO Data Input and Data Output*/ /*WK2168/WK2212*/

/*****************************
****wk2xxx  slave uarts  register address defines****
******************************/
#define WK2XXX_SPAGE_REG 0X03 /*Slave UART Register page selection*/
#define WK2XXX_PAGE1 1
#define WK2XXX_PAGE0 0

/*PAGE0**/
#define WK2XXX_SCR_REG 0X04   /*Slave UART Transmitter and Receiver Enable*/
#define WK2XXX_LCR_REG 0X05   /* Line Control */
#define WK2XXX_FCR_REG 0X06   /* FIFO control */
#define WK2XXX_SIER_REG 0X07  /* Interrupt enable */
#define WK2XXX_SIFR_REG 0X08  /* Interrupt Identification */
#define WK2XXX_TFCNT_REG 0X09 /* TX FIFO counter */
#define WK2XXX_RFCNT_REG 0X0A /* RX FIFO counter */
#define WK2XXX_FSR_REG 0X0B   /* FIFO Status */
#define WK2XXX_LSR_REG 0X0C   /* Line Status */
#define WK2XXX_FDAT_REG 0X0D  /*  Write transmit FIFO data or Read receive FIFO data */
#define WK2XXX_FWCR_REG 0X0E  /* Flow  Control */
#define WK2XXX_RS485_REG 0X0F /* RS485 Control */
/*PAGE1*/
#define WK2XXX_BAUD1_REG 0X04  /* Divisor Latch High */
#define WK2XXX_BAUD0_REG 0X05  /* Divisor Latch Low */
#define WK2XXX_PRES_REG 0X06   /* Divisor Latch Fractional Part */
#define WK2XXX_RFTL_REG 0X07   /* Receive FIFO Trigger Level */
#define WK2XXX_TFTL_REG 0X08   /* Transmit FIFO Trigger Level */
#define WK2XXX_FWTH_REG 0X09   /*Flow control trigger high level*/
#define WK2XXX_FWTL_REG 0X0A   /*Flow control trigger low level*/
#define WK2XXX_XON1_REG 0X0B   /* Xon1 word */
#define WK2XXX_XOFF1_REG 0X0C  /* Xoff1 word */
#define WK2XXX_SADR_REG 0X0D   /*RS485 auto address*/
#define WK2XXX_SAEN_REG 0X0E   /*RS485 auto address mask*/
#define WK2XXX_RRSDLY_REG 0X0F /*RTS delay when transmit in RS485*/

// wkxxx register bit defines
/*GENA register*/
#define WK2XXX_GENA_UT4EN_BIT 0x08
#define WK2XXX_GENA_UT3EN_BIT 0x04
#define WK2XXX_GENA_UT2EN_BIT 0x02
#define WK2XXX_GENA_UT1EN_BIT 0x01
/*GRST register*/
#define WK2XXX_GRST_UT4SLEEP_BIT 0x80
#define WK2XXX_GRST_UT3SLEEP_BIT 0x40
#define WK2XXX_GRST_UT2SLEEP_BIT 0x20
#define WK2XXX_GRST_UT1SLEEP_BIT 0x10
#define WK2XXX_GRST_UT4RST_BIT 0x08
#define WK2XXX_GRST_UT3RST_BIT 0x04
#define WK2XXX_GRST_UT2RST_BIT 0x02
#define WK2XXX_GRST_UT1RST_BIT 0x01
/*GIER register bits*/
#define WK2XXX_GIER_UT4IE_BIT 0x08
#define WK2XXX_GIER_UT3IE_BIT 0x04
#define WK2XXX_GIER_UT2IE_BIT 0x02
#define WK2XXX_GIER_UT1IE_BIT 0x01
/*GIFR register bits*/
#define WK2XXX_GIFR_UT4INT_BIT 0x08
#define WK2XXX_GIFR_UT3INT_BIT 0x04
#define WK2XXX_GIFR_UT2INT_BIT 0x02
#define WK2XXX_GIFR_UT1INT_BIT 0x01
/*SPAGE register bits*/
#define WK2XXX_SPAGE_PAGE_BIT 0x01
/*SCR register bits*/
#define WK2XXX_SCR_SLEEPEN_BIT 0x04
#define WK2XXX_SCR_TXEN_BIT 0x02
#define WK2XXX_SCR_RXEN_BIT 0x01
/*LCR register bits*/
#define WK2XXX_LCR_BREAK_BIT 0x20
#define WK2XXX_LCR_IREN_BIT 0x10

#define WK2XXX_LCR_PAEN_BIT 0x08
#define WK2XXX_LCR_PAM1_BIT 0x04
#define WK2XXX_LCR_PAM0_BIT 0x02
/*ODD Parity*/
#define WK2XXX_LCR_ODD_PARITY 0x0a
/*Even Parity*/
#define WK2XXX_LCR_EVEN_PARITY 0x0c
/*Parity :=0*/
#define WK2XXX_LCR_SPACE_PARITY 0x08
/*Parity :=1*/
#define WK2XXX_LCR_MARK_PARITY 0x0e

#define WK2XXX_LCR_STPL_BIT 0x01

/*FCR register bits*/
#define WK2XXX_FCR_TFEN_BIT 0x08
#define WK2XXX_FCR_RFEN_BIT 0x02
#define WK2XXX_FCR_RFRST_BIT 0x01
/*SIER register bits*/
#define WK2XXX_SIER_FERR_IEN_BIT 0x80
#define WK2XXX_SIER_CTS_IEN_BIT 0x40
#define WK2XXX_SIER_RTS_IEN_BIT 0x20
#define WK2XXX_SIER_XOFF_IEN_BIT 0x10
#define WK2XXX_SIER_TFEMPTY_IEN_BIT 0x08
#define WK2XXX_SIER_TFTRIG_IEN_BIT 0x04
#define WK2XXX_SIER_RXOUT_IEN_BIT 0x02
#define WK2XXX_SIER_RFTRIG_IEN_BIT 0x01
/*SIFR register bits*/
#define WK2XXX_SIFR_FERR_INT_BIT 0x80
#define WK2XXX_SIFR_CTS_INT_BIT 0x40
#define WK2XXX_SIFR_RTS_INT_BIT 0x20
#define WK2XXX_SIFR_XOFF_INT_BIT 0x10
#define WK2XXX_SIFR_TFEMPTY_INT_BIT 0x08
#define WK2XXX_SIFR_TFTRIG_INT_BIT 0x04
#define WK2XXX_SIFR_RXOVT_INT_BIT 0x02
#define WK2XXX_SIFR_RFTRIG_INT_BIT 0x01
/*FSR register bits*/
#define WK2XXX_FSR_RFOE_BIT 0x80
#define WK2XXX_FSR_RFBI_BIT 0x40
#define WK2XXX_FSR_RFFE_BIT 0x20
#define WK2XXX_FSR_RFPE_BIT 0x10

#define WK2XXX_FSR_ERR_MASK 0xF0

#define WK2XXX_FSR_RDAT_BIT 0x08
#define WK2XXX_FSR_TDAT_BIT 0x04
#define WK2XXX_FSR_TFULL_BIT 0x02
#define WK2XXX_FSR_TBUSY_BIT 0x01
/*LSR register bits*/
#define WK2XXX_LSR_BRK_ERROR_MASK 0X0F /* BI, FE, PE, OE bits */
#define WK2XXX_LSR_OE_BIT 0x08
#define WK2XXX_LSR_BI_BIT 0x04
#define WK2XXX_LSR_FE_BIT 0x02
#define WK2XXX_LSR_PE_BIT 0x01
/*FWCR register bits*/
#define WK2XXX_FWCR_RTS_BIT 0x02
#define WK2XXX_FWCR_CTS _BIT 0x01
/*RS485 register bits*/
#define WK2XXX_RS485_RSRS485_BIT 0x40
#define WK2XXX_RS485_ATADD_BIT 0x20
#define WK2XXX_RS485_DATEN_BIT 0x10
#define WK2XXX_RS485_RTSEN_BIT 0x02
#define WK2XXX_RS485_RTSINV_BIT 0x01

#ifdef WK_CSGPIO_FUNCTION
int cs_gpio_num;
#endif

struct wk2xxx_devtype
{
    char name[10];
    int nr_uart;
};

struct rockchip_spi_regs
{
    u32 ctrlr0;
    u32 ctrlr1;
    u32 enr;
    u32 ser;
    u32 baudr;
    u32 txftlr;
    u32 rxftlr;
    u32 txflr;
    u32 rxflr;
    u32 sr;
    u32 ipr;
    u32 imr;
    u32 isr;
    u32 risr;
    u32 icr;
    u32 dmacr;
    u32 dmatdlr;
    u32 dmardlr;
    u32 ver;
    u32 reserved[0xee];
    u32 txdr[0x100];
    u32 rxdr[0x100];
};

struct wk2xxx_mmio_spi
{
    struct device *ctlr_dev;
    struct clk *spiclk;
    struct clk *apb_pclk;
    struct rockchip_spi_regs __iomem *regs;
    u32 fifo_len;
    u32 fifo_rx_th;
    u32 clock_div;
    u32 cr0;
    u32 rsd;
    u8 cs;
};

struct wk2xxx_one
{

    struct uart_port port; //[NR_PORTS];
    struct kthread_work start_tx_work;
    struct kthread_work stop_rx_work;
    uint8_t line;
    uint8_t new_lcr_reg;
    uint8_t new_fwcr_reg;
    uint8_t new_scr_reg;
    /*baud register*/
    uint8_t new_baud1_reg;
    uint8_t new_baud0_reg;
    uint8_t new_pres_reg;
};

struct wk2xxx_port
{
    const struct wk2xxx_devtype *devtype;
    struct uart_driver uart;
    struct spi_device *spi_wk;
    struct workqueue_struct *workqueue;
    struct work_struct work;
    unsigned char buf[256];
    struct kthread_worker kworker;
    struct task_struct *kworker_task;
    struct kthread_work irq_work;
    // int cs_gpio_num;
    int irq_gpio_num;
    int rst_gpio_num;
    int irq_gpio;
    int minor; /* minor number */
    int tx_empty;
    struct wk2xxx_mmio_spi mmio;
    struct wk2xxx_one p[NR_PORTS];
};

static const struct wk2xxx_devtype wk2124_devtype = {
    .name = "WK2124",
    .nr_uart = 4,
};
static const struct wk2xxx_devtype wk2132_devtype = {
    .name = "WK2132",
    .nr_uart = 2,
};
static const struct wk2xxx_devtype wk2204_devtype = {
    .name = "WK2204",
    .nr_uart = 4,
};
static const struct wk2xxx_devtype wk2168_devtype = {
    .name = "WK2168",
    .nr_uart = 4,
};
static const struct wk2xxx_devtype wk2202_devtype = {
    .name = "WK2202",
    .nr_uart = 2,
};

#define to_wk2xxx_port(p, e) ((container_of((p), struct wk2xxx_port, e)))
#define to_wk2xxx_one(p, e) ((container_of((p), struct wk2xxx_one, e)))

// add by linx for rs485
extern void gpio_set_rs485_txen(void);

extern void gpio_clr_rs485_txen(void);

void rs485_do_tasklet(unsigned long);
static u_int wk2xxx_tx_empty(struct uart_port *port); // or query the tx fifo is not empty?

static struct tasklet_struct rs485_tasklet;

void start_485_tasklet(void)
{
    tasklet_hi_schedule(&rs485_tasklet);
}

static int wk2xxx_rs485_config(struct uart_port *port, struct serial_rs485 *rs485)
{
    if (rs485->flags & SER_RS485_ENABLED)
    {
        gpio_clr_rs485_txen();
        // tasklet_init(&rs485_tasklet, rs485_do_tasklet, (unsigned long) port);
    }

    memcpy(&port->rs485, rs485, sizeof(*rs485));
    // printk("linx: %s, %d rs485=%p, flag=%d\n", __func__, __LINE__, &port->rs485, port->rs485.flags & SER_RS485_ENABLED);
    return 0;
}

#define my_container_of(ptr, type, member) ((type *)((char *)(ptr)-offsetof(type, member)))

void rs485_do_tasklet(unsigned long param)
{
    struct uart_port *port = (struct uart_port *)param;
    // struct uart_state *state;
    // struct tty_struct *tty;
    // struct ktermios termios;
    // unsigned int baud;
    // int bit_width;

    while (wk2xxx_tx_empty(port) != TIOCSER_TEMT)
        ;
    // state = my_container_of(port, struct uart_state, uart_port);
    // tty = my_container_of(state, struct tty_struct, driver_data);
    // termios = tty->termios;
    // baud = uart_get_baud_rate(port, &termios, NULL, 1200, 115200);
    // bit_width = (baud > 0) ? 1000000 / baud : 0;
    // bit_width = (bit_width > 50) ? (bit_width - 50) : 0; // Measured delay value is 50 us
    // //printk("linx: %s, %d,baud=%d,delay=%d us\n", __func__, __LINE__, baud, bit_width);
    // udelay(bit_width); // a stop bit
    gpio_clr_rs485_txen();
}

// end of rs485

static int wk2xxx_mmio_runtime_get(struct wk2xxx_port *s)
{
    int ret;

    if (!s->mmio.ctlr_dev)
        return 0;

    ret = pm_runtime_get_sync(s->mmio.ctlr_dev);
    if (ret < 0)
    {
        pm_runtime_put_noidle(s->mmio.ctlr_dev);
        dev_warn(&s->spi_wk->dev,
                 "wk2xxx: parent runtime PM unavailable (%d), continue without it\n",
                 ret);
        s->mmio.ctlr_dev = NULL;
        return 0;
    }

    return 0;
}

static void wk2xxx_mmio_runtime_put(struct wk2xxx_port *s)
{
    if (s->mmio.ctlr_dev)
        pm_runtime_put(s->mmio.ctlr_dev);
}

static void wk2xxx_mmio_enable_chip(struct rockchip_spi_regs __iomem *regs, bool enable)
{
    writel(enable ? 1 : 0, &regs->enr);
}

static void wk2xxx_mmio_set_baudr(struct wk2xxx_port *s)
{
    writel(s->mmio.clock_div, &s->mmio.regs->baudr);
}

static void wk2xxx_mmio_cs_activate(struct wk2xxx_port *s)
{
    writel(BIT(s->mmio.cs), &s->mmio.regs->ser);
}

static void wk2xxx_mmio_cs_deactivate(struct wk2xxx_port *s)
{
    writel(0, &s->mmio.regs->ser);
}

static int wk2xxx_mmio_wait_till_not_busy(struct wk2xxx_port *s)
{
    unsigned long timeout = jiffies + usecs_to_jiffies(ROCKCHIP_SPI_TIMEOUT_US);

    while (readl(&s->mmio.regs->sr) & SR_BUSY)
    {
        if (time_after(jiffies, timeout))
            return -ETIMEDOUT;
        cpu_relax();
    }

    return 0;
}

static u32 wk2xxx_mmio_get_fifo_len(struct wk2xxx_port *s)
{
    u32 ver = readl(&s->mmio.regs->ver);

    switch (ver)
    {
    case ROCKCHIP_SPI_VER2_TYPE1:
    case ROCKCHIP_SPI_VER2_TYPE2:
        return 64;
    default:
        return 32;
    }
}

static int wk2xxx_mmio_claim_bus(struct wk2xxx_port *s)
{
    u8 spi_dfs;
    u8 spi_tf;
    u32 ctrlr0;

    wk2xxx_mmio_enable_chip(s->mmio.regs, false);

    spi_dfs = DFS_8BIT;
    spi_tf = HALF_WORD_OFF;

    wk2xxx_mmio_set_baudr(s);

    ctrlr0 = OMOD_MASTER << OMOD_SHIFT;
    ctrlr0 |= spi_dfs << DFS_SHIFT;
    if (s->spi_wk->mode & SPI_CPOL)
        ctrlr0 |= SCOL_HIGH << SCOL_SHIFT;
    if (s->spi_wk->mode & SPI_CPHA)
        ctrlr0 |= SCPH_TOGSTA << SCPH_SHIFT;
    ctrlr0 |= CSM_KEEP << CSM_SHIFT;
    ctrlr0 |= SSN_DELAY_ONE << SSN_DELAY_SHIFT;
    ctrlr0 |= SEM_LITTLE << SEM_SHIFT;
    ctrlr0 |= FBM_MSB << FBM_SHIFT;
    ctrlr0 |= spi_tf << HALF_WORD_TX_SHIFT;
    ctrlr0 |= s->mmio.rsd << RXDSD_SHIFT;
    ctrlr0 |= FRF_SPI << FRF_SHIFT;
    s->mmio.cr0 = ctrlr0;

    writel(ctrlr0, &s->mmio.regs->ctrlr0);
    writel(0, &s->mmio.regs->ctrlr1);
    writel(0, &s->mmio.regs->dmacr);
    writel(0, &s->mmio.regs->txftlr);
    writel(0, &s->mmio.regs->rxftlr);
    writel(0, &s->mmio.regs->ser);

    wk2xxx_dbg(&s->spi_wk->dev,
               "claim_bus cs=%u clock_div=%u fifo_len=%u cr0=%#x\n",
               s->mmio.cs, s->mmio.clock_div, s->mmio.fifo_len, s->mmio.cr0);

    return 0;
}

static void wk2xxx_mmio_config(struct wk2xxx_port *s, const void *dout, void *din, int len)
{
    u32 ctrlr0 = s->mmio.cr0;
    u32 tmod;

    wk2xxx_mmio_enable_chip(s->mmio.regs, false);

    if (dout && din)
        tmod = TMOD_TR;
    else if (dout)
        tmod = TMOD_TO;
    else
        tmod = TMOD_RO;

    ctrlr0 |= tmod << TMOD_SHIFT;
    writel(ctrlr0, &s->mmio.regs->ctrlr0);
    if (tmod == TMOD_RO)
        writel(len ? len - 1 : 0, &s->mmio.regs->ctrlr1);
    else
        writel(0, &s->mmio.regs->ctrlr1);
}

static int wk2xxx_mmio_xfer(struct wk2xxx_port *s, int len, const void *dout, void *din, unsigned long flags)
{
    const u8 *out = dout;
    u8 *in = din;
    int toread = din ? len : 0;
    int towrite = dout ? len : 0;
    int ret;
    unsigned long deadline;
    int last_toread = toread;
    int last_towrite = towrite;

    if (!s || !s->mmio.regs)
        return -ENODEV;

    ret = wk2xxx_mmio_runtime_get(s);
    if (ret)
        return ret;

    wk2xxx_mmio_config(s, dout, din, len);
    if (len <= 4)
        wk2xxx_dbg_rl(&s->spi_wk->dev,
                      "xfer start len=%d flags=%#lx tx=%d rx=%d ctrlr0=%#x ctrlr1=%#x\n",
                      len, flags, towrite, toread,
                      readl(&s->mmio.regs->ctrlr0),
                      readl(&s->mmio.regs->ctrlr1));

    if (flags & SPI_XFER_BEGIN)
        wk2xxx_mmio_cs_activate(s);

    wk2xxx_mmio_enable_chip(s->mmio.regs, true);
    deadline = jiffies + usecs_to_jiffies(ROCKCHIP_SPI_TIMEOUT_US);
    while (toread || towrite)
    {
        if (towrite)
        {
            int room = min_t(int, s->mmio.fifo_len - readl(&s->mmio.regs->txflr), towrite);
            int i;

            for (i = 0; i < room; i++)
                writel(out ? *out++ : 0, &s->mmio.regs->txdr[0]);
            towrite -= room;
        }

        if (toread)
        {
            int room = min_t(int, readl(&s->mmio.regs->rxflr), toread);
            int i;

            if (room < s->mmio.fifo_rx_th && room < toread)
            {
                cpu_relax();
                continue;
            }

            for (i = 0; i < room; i++)
                *in++ = readl(&s->mmio.regs->rxdr[0]);
            toread -= room;
        }

        if (toread != last_toread || towrite != last_towrite)
        {
            last_toread = toread;
            last_towrite = towrite;
            deadline = jiffies + usecs_to_jiffies(ROCKCHIP_SPI_TIMEOUT_US);
            continue;
        }

        if (time_after(jiffies, deadline))
        {
            dev_err(&s->spi_wk->dev,
                    "wk2xxx mmio xfer timeout len=%d towrite=%d toread=%d sr=%#x txflr=%#x rxflr=%#x\n",
                    len, towrite, toread,
                    readl(&s->mmio.regs->sr),
                    readl(&s->mmio.regs->txflr),
                    readl(&s->mmio.regs->rxflr));
            ret = -ETIMEDOUT;
            goto out_disable;
        }

        cpu_relax();
    }

    ret = wk2xxx_mmio_wait_till_not_busy(s);
    if (!ret && len <= 4)
        wk2xxx_dbg_rl(&s->spi_wk->dev,
                      "xfer done len=%d sr=%#x txflr=%#x rxflr=%#x\n",
                      len,
                      readl(&s->mmio.regs->sr),
                      readl(&s->mmio.regs->txflr),
                      readl(&s->mmio.regs->rxflr));
out_disable:
    if (flags & SPI_XFER_END)
        wk2xxx_mmio_cs_deactivate(s);
    wk2xxx_mmio_enable_chip(s->mmio.regs, false);
    wk2xxx_mmio_runtime_put(s);

    return ret;
}

static int wk2xxx_mmio_init(struct spi_device *spi, struct wk2xxx_port *s)
{
    struct device *ctlr_dev = spi->dev.parent;
    struct device_node *ctlr_np;
    struct resource res;
    unsigned long clk_rate = 0;
    int ret;
    u32 baudr;

    if (!ctlr_dev || !ctlr_dev->of_node)
        return -ENODEV;

    ctlr_np = ctlr_dev->of_node;
    if (of_address_to_resource(ctlr_np, 0, &res))
        return -ENODEV;

    s->mmio.regs = devm_ioremap(&spi->dev, res.start, resource_size(&res));
    if (!s->mmio.regs)
        return -ENOMEM;

    s->mmio.ctlr_dev = ctlr_dev;
    s->mmio.cs = spi->chip_select;
    s->mmio.rsd = 0;
    s->mmio.spiclk = of_clk_get_by_name(ctlr_np, "spiclk");
    if (IS_ERR(s->mmio.spiclk))
    {
        ret = PTR_ERR(s->mmio.spiclk);
        if (ret == -EPROBE_DEFER)
        {
            s->mmio.spiclk = NULL;
            return ret;
        }
        dev_warn(&spi->dev,
                 "wk2xxx: failed to get parent spiclk (%d), fallback to current controller baud setting\n",
                 ret);
        s->mmio.spiclk = NULL;
    }

    s->mmio.apb_pclk = of_clk_get_by_name(ctlr_np, "apb_pclk");
    if (IS_ERR(s->mmio.apb_pclk))
    {
        ret = PTR_ERR(s->mmio.apb_pclk);
        if (ret == -EPROBE_DEFER)
        {
            s->mmio.apb_pclk = NULL;
            if (s->mmio.spiclk)
            {
                clk_put(s->mmio.spiclk);
                s->mmio.spiclk = NULL;
            }
            return ret;
        }
        dev_warn(&spi->dev,
                 "wk2xxx: failed to get parent apb_pclk (%d), continuing anyway\n",
                 ret);
        s->mmio.apb_pclk = NULL;
    }

    if (s->mmio.apb_pclk)
    {
        ret = clk_prepare_enable(s->mmio.apb_pclk);
        if (ret)
        {
            dev_warn(&spi->dev,
                     "wk2xxx: failed to enable parent apb_pclk (%d), continuing anyway\n",
                     ret);
            clk_put(s->mmio.apb_pclk);
            s->mmio.apb_pclk = NULL;
        }
    }

    if (s->mmio.spiclk)
    {
        ret = clk_prepare_enable(s->mmio.spiclk);
        if (ret)
        {
            dev_warn(&spi->dev,
                     "wk2xxx: failed to enable parent spiclk (%d), fallback to current controller baud setting\n",
                     ret);
            clk_put(s->mmio.spiclk);
            s->mmio.spiclk = NULL;
        }
        else
        {
            clk_rate = clk_get_rate(s->mmio.spiclk);
        }
    }

    s->mmio.clock_div = 8;
    if (clk_rate)
    {
        s->mmio.clock_div = DIV_ROUND_UP(clk_rate, wk2xxx_spi_speed);
        if (s->mmio.clock_div < 2)
            s->mmio.clock_div = 2;
        if (s->mmio.clock_div & 0x1)
            s->mmio.clock_div++;
    }

    wk2xxx_mmio_runtime_get(s);

    baudr = readl(&s->mmio.regs->baudr);
    if (!clk_rate && baudr >= 2)
        s->mmio.clock_div = baudr;

    s->mmio.fifo_len = wk2xxx_mmio_get_fifo_len(s);
    s->mmio.fifo_rx_th = s->mmio.fifo_len * 3 / 4;
    ret = wk2xxx_mmio_claim_bus(s);
    wk2xxx_mmio_runtime_put(s);
    if (ret)
        goto err_clk;

    wk2xxx_dbg(&spi->dev,
               "mmio_init regs=%p cs=%u clk_rate=%lu clock_div=%u baudr=%u fifo_len=%u fifo_rx_th=%u\n",
               s->mmio.regs, s->mmio.cs, clk_rate,
               s->mmio.clock_div, baudr,
               s->mmio.fifo_len, s->mmio.fifo_rx_th);

    return 0;

err_clk:
    if (s->mmio.spiclk)
    {
        clk_disable_unprepare(s->mmio.spiclk);
        clk_put(s->mmio.spiclk);
        s->mmio.spiclk = NULL;
    }
    if (s->mmio.apb_pclk)
    {
        clk_disable_unprepare(s->mmio.apb_pclk);
        clk_put(s->mmio.apb_pclk);
        s->mmio.apb_pclk = NULL;
    }

    return ret;
}

static void wk2xxx_mmio_exit(struct wk2xxx_port *s)
{
    if (s->mmio.spiclk)
    {
        clk_disable_unprepare(s->mmio.spiclk);
        clk_put(s->mmio.spiclk);
        s->mmio.spiclk = NULL;
    }
    if (s->mmio.apb_pclk)
    {
        clk_disable_unprepare(s->mmio.apb_pclk);
        clk_put(s->mmio.apb_pclk);
        s->mmio.apb_pclk = NULL;
    }

    s->mmio.regs = NULL;
    s->mmio.ctlr_dev = NULL;
}

/*
 * This function read wk2xxx of Global register:
 */
static int wk2xxx_read_global_reg(struct spi_device *spi, uint8_t reg, uint8_t *dat)
{
    struct wk2xxx_port *s = dev_get_drvdata(&spi->dev);
    uint8_t buf_wdat[2];
    uint8_t buf_rdat[2];
    int status;

    mutex_lock(&wk2xxxs_reg_lock);
    buf_wdat[0] = 0x40 | reg;
    buf_wdat[1] = 0x00;
    buf_rdat[0] = 0x00;
    buf_rdat[1] = 0x00;
    status = wk2xxx_mmio_xfer(s, 2, buf_wdat, buf_rdat, SPI_XFER_ONCE);
    mutex_unlock(&wk2xxxs_reg_lock);
    if (status)
    {
        wk2xxx_dbg_rl(&spi->dev, "read_global_reg failed reg=%#x status=%d\n", reg, status);
        return status;
    }
    *dat = buf_rdat[1];
    return 0;
}
/*
 * This function write wk2xxx of Global register:
 */
static int wk2xxx_write_global_reg(struct spi_device *spi, uint8_t reg, uint8_t dat)
{
    struct wk2xxx_port *s = dev_get_drvdata(&spi->dev);
    uint8_t buf_reg[2];
    int status;

    mutex_lock(&wk2xxxs_reg_lock);
    /* register index */
    buf_reg[0] = 0x00 | reg;
    buf_reg[1] = dat;
    status = wk2xxx_mmio_xfer(s, 2, buf_reg, NULL, SPI_XFER_ONCE);
    mutex_unlock(&wk2xxxs_reg_lock);
    if (status)
        wk2xxx_dbg_rl(&spi->dev, "write_global_reg failed reg=%#x val=%#x status=%d\n", reg, dat, status);
    return status;
}
/*
 * This function read wk2xxx of slave register:
 */
static int wk2xxx_read_slave_reg(struct spi_device *spi, uint8_t port, uint8_t reg, uint8_t *dat)
{
    struct wk2xxx_port *s = dev_get_drvdata(&spi->dev);
    uint8_t buf_wdat[2];
    uint8_t buf_rdat[2];
    int status;

    mutex_lock(&wk2xxxs_reg_lock);
    buf_wdat[0] = 0x40 | (((port - 1) << 4) | reg);
    buf_wdat[1] = 0x00;
    buf_rdat[0] = 0x00;
    buf_rdat[1] = 0x00;
    status = wk2xxx_mmio_xfer(s, 2, buf_wdat, buf_rdat, SPI_XFER_ONCE);
    mutex_unlock(&wk2xxxs_reg_lock);
    if (status)
    {
        wk2xxx_dbg_rl(&spi->dev, "read_slave_reg failed port=%u reg=%#x status=%d\n", port, reg, status);
        return status;
    }
    *dat = buf_rdat[1];
    return 0;
}
/*
 * This function write wk2xxx of Slave register:
 */
static int wk2xxx_write_slave_reg(struct spi_device *spi, uint8_t port, uint8_t reg, uint8_t dat)
{
    struct wk2xxx_port *s = dev_get_drvdata(&spi->dev);
    uint8_t buf_reg[2];
    int status;

    mutex_lock(&wk2xxxs_reg_lock);
    /* register index */
    buf_reg[0] = ((port - 1) << 4) | reg;
    buf_reg[1] = dat;
    status = wk2xxx_mmio_xfer(s, 2, buf_reg, NULL, SPI_XFER_ONCE);
    mutex_unlock(&wk2xxxs_reg_lock);
    if (status)
        wk2xxx_dbg_rl(&spi->dev, "write_slave_reg failed port=%u reg=%#x val=%#x status=%d\n", port, reg, dat, status);
    return status;
}

#define MAX_RFCOUNT_SIZE 256

/*
 * This function read wk2xxx of fifo:
 */
static int wk2xxx_read_fifo(struct spi_device *spi, uint8_t port, uint8_t fifolen, uint8_t *dat)
{
    struct wk2xxx_port *s = dev_get_drvdata(&spi->dev);
    int status, i;
    uint8_t recive_fifo_data[MAX_RFCOUNT_SIZE + 1] = {0};
    uint8_t transmit_fifo_data[MAX_RFCOUNT_SIZE + 1] = {0};

    if (!(fifolen > 0))
    {
        printk(KERN_ERR "%s,fifolen error!!\n", __func__);
        return 1;
    }
    mutex_lock(&wk2xxxs_reg_lock);
    /* register index */
    transmit_fifo_data[0] = ((port - 1) << 4) | 0xc0;
    status = wk2xxx_mmio_xfer(s, fifolen + 1, transmit_fifo_data, recive_fifo_data, SPI_XFER_ONCE);
    for (i = 0; i < fifolen; i++)
        *(dat + i) = recive_fifo_data[i + 1];
    mutex_unlock(&wk2xxxs_reg_lock);
    if (status)
        wk2xxx_dbg_rl(&spi->dev, "read_fifo failed port=%u len=%u status=%d\n", port, fifolen, status);
    return status;
}
/*
 * This function write wk2xxx of fifo:
 */
static int wk2xxx_write_fifo(struct spi_device *spi, uint8_t port, uint8_t fifolen, uint8_t *dat)
{
    struct wk2xxx_port *s = dev_get_drvdata(&spi->dev);
    int status, i;
    uint8_t transmit_fifo_data[MAX_RFCOUNT_SIZE + 1] = {0};

    if (!(fifolen > 0))
    {
        printk(KERN_ERR "%s,fifolen error,fifolen:%d!!\n", __func__, fifolen);
        return 1;
    }
    mutex_lock(&wk2xxxs_reg_lock);
    /* register index */
    transmit_fifo_data[0] = ((port - 1) << 4) | 0x80;
    for (i = 0; i < fifolen; i++)
    {
        transmit_fifo_data[i + 1] = *(dat + i);
    }
    status = wk2xxx_mmio_xfer(s, fifolen + 1, transmit_fifo_data, NULL, SPI_XFER_ONCE);
    mutex_unlock(&wk2xxxs_reg_lock);
    if (status)
        wk2xxx_dbg_rl(&spi->dev, "write_fifo failed port=%u len=%u status=%d first=%#x\n",
                      port, fifolen, status, fifolen ? dat[0] : 0);
    return status;
}

static void conf_wk2xxx_subport(struct uart_port *port);
static void wk2xxx_stop_tx(struct uart_port *port);
static u_int wk2xxx_tx_empty(struct uart_port *port);

static void wk2xxx_rx_chars(struct uart_port *port)
{
    // struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
    uint8_t fsr, rx_dat[256] = {0};
    uint8_t rfcnt = 0, rfcnt2 = 0;
    unsigned int flg, status = 0, rx_count = 0;
    int rx_num = 0, rxlen = 0;
    int len_rfcnt, len_limit, len_p = 0;
    len_limit = SPI_LEN_LIMIT;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FSR_REG, &fsr);
    if (fsr & WK2XXX_FSR_RDAT_BIT)
    {
        wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_RFCNT_REG, &rfcnt);
        if (rfcnt == 0)
        {
            wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_RFCNT_REG, &rfcnt);
        }
        wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_RFCNT_REG, &rfcnt2);
        if (rfcnt2 == 0)
        {
            wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_RFCNT_REG, &rfcnt2);
        }
        rfcnt = (rfcnt2 >= rfcnt) ? rfcnt : rfcnt2;
        rxlen = (rfcnt == 0) ? 256 : rfcnt;
        wk2xxx_dbg_rl(port->dev,
                      "rx_chars port=%ld fsr=%#x rfcnt=%u rfcnt2=%u rxlen=%d\n",
                      one->port.iobase, fsr, rfcnt, rfcnt2, rxlen);
    }
#ifdef _DEBUG_WK_RX
    printk(KERN_ALERT "rx_chars()-port:%lx--fsr:0x%x--rxlen:%d--\n", one->port.iobase, fsr, rxlen);
#endif
    flg = TTY_NORMAL;
#ifdef WK_FIFO_FUNCTION
    len_rfcnt = rxlen;
    while (len_rfcnt)
    {
        if (len_rfcnt > len_limit)
        {
            wk2xxx_read_fifo(s->spi_wk, one->port.iobase, len_limit, rx_dat + len_p);
            len_rfcnt = len_rfcnt - len_limit;
            len_p = len_p + len_limit;
        }
        else
        {
            wk2xxx_read_fifo(s->spi_wk, one->port.iobase, len_rfcnt, rx_dat + len_p); //
            len_rfcnt = 0;
        }
    }
#else
    for (rx_num = 0; rx_num < rxlen; rx_num++)
    {
        wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FDAT_REG, &rx_dat[rx_num]);
    }
#endif

    one->port.icount.rx += rxlen;
    for (rx_num = 0; rx_num < rxlen; rx_num++)
    {

        if (fsr & WK2XXX_FSR_ERR_MASK)
        {
            fsr &= WK2XXX_FSR_ERR_MASK;
            if (fsr & (WK2XXX_FSR_RFOE_BIT | WK2XXX_FSR_RFBI_BIT | WK2XXX_FSR_RFFE_BIT | WK2XXX_FSR_RFPE_BIT))
            {
                if (fsr & WK2XXX_FSR_RFPE_BIT)
                {
                    one->port.icount.parity++;
                    status |= WK2XXX_STATUS_PE;
                    flg = TTY_PARITY;
                }

                if (fsr & WK2XXX_FSR_RFFE_BIT)
                {
                    one->port.icount.frame++;
                    status |= WK2XXX_STATUS_FE;
                    flg = TTY_FRAME;
                }

                if (fsr & WK2XXX_FSR_RFOE_BIT)
                {
                    one->port.icount.overrun++;
                    status |= WK2XXX_STATUS_OE;
                    flg = TTY_OVERRUN;
                }
                if (fsr & WK2XXX_FSR_RFBI_BIT)
                {
                    one->port.icount.brk++;
                    status |= WK2XXX_STATUS_BRK;
                    flg = TTY_BREAK;
                }
            }
        }
        if (uart_handle_sysrq_char(port, rx_dat[rx_num]))
            continue; //
#ifdef _DEBUG_WK_RX
        printk(KERN_ALERT "rx_chars:0x%x----\n", rx_dat[rx_num]);
#endif
        uart_insert_char(port, status, WK2XXX_STATUS_OE, rx_dat[rx_num], flg);
        rx_count++;
    }
    if (rx_count > 0)
    {
        wk2xxx_dbg_rl(port->dev,
                  "rx_push port=%ld count=%u first=%#x status=%#x\n",
                  one->port.iobase, rx_count, rx_dat[0], status);
#ifdef _DEBUG_WK_RX
        printk(KERN_ALERT "push buffer tty flip port = :%lx count =:%d\n", one->port.iobase, rx_count);
#endif
        tty_flip_buffer_push(&port->state->port);
        rx_count = 0;
    }
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--exit--\n", __func__, one->port.iobase);
#endif
}

static void wk2xxx_tx_chars(struct uart_port *port)
{
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    // struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
    uint8_t fsr, tfcnt, dat[1], txbuf[256] = {0};
    int count, tx_count, i;
    int len_tfcnt, len_limit, len_p = 0;
    len_limit = SPI_LEN_LIMIT;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif
    if (one->port.x_char)
    {
        wk2xxx_dbg_rl(port->dev,
                  "tx_chars x_char port=%ld val=%#x\n",
                  one->port.iobase, one->port.x_char);
#ifdef _DEBUG_WK_TX
        printk(KERN_ALERT "wk2xxx_tx_chars   one->port.x_char:%x,port = %ld\n", one->port.x_char, one->port.iobase);
#endif
        wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FDAT_REG, one->port.x_char);
        one->port.icount.tx++;
        one->port.x_char = 0;
        goto out;
    }

    if (uart_circ_empty(&one->port.state->xmit) || uart_tx_stopped(&one->port))
    {
        goto out;
    }

    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FSR_REG, &fsr);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_TFCNT_REG, &tfcnt);
#ifdef _DEBUG_WK_TX
    printk(KERN_ALERT "wk2xxx_tx_chars   fsr:0x%x,tfcnt:0x%x,port = %ld\n", fsr, tfcnt, one->port.iobase);
#endif
    if (tfcnt == 0)
    {
        tx_count = (fsr & WK2XXX_FSR_TFULL_BIT) ? 0 : 256;
#ifdef _DEBUG_WK_TX
        printk(KERN_ALERT "wk2xxx_tx_chars2   tx_count:%x,port = %ld\n", tx_count, one->port.iobase);
#endif
    }
    else
    {
        tx_count = 256 - tfcnt;
#ifdef _DEBUG_WK_TX
        printk(KERN_ALERT "wk2xxx_tx_chars2   tx_count:%x,port = %ld\n", tx_count, one->port.iobase);
#endif
    }
    if (tx_count > 200)
    {
        tx_count = 200;
    }
    wk2xxx_dbg_rl(port->dev,
                  "tx_chars port=%ld fsr=%#x tfcnt=%u tx_count=%d pending=%d\n",
                  one->port.iobase, fsr, tfcnt, tx_count,
                  uart_circ_chars_pending(&one->port.state->xmit));
    count = tx_count;
    i = 0;
    while (count)
    {
        if (uart_circ_empty(&one->port.state->xmit))
            break;
        txbuf[i] = one->port.state->xmit.buf[one->port.state->xmit.tail];
        one->port.state->xmit.tail = (one->port.state->xmit.tail + 1) & (UART_XMIT_SIZE - 1);
        one->port.icount.tx++;
        i++;
        count = count - 1;
#ifdef _DEBUG_WK_TX
        printk(KERN_ALERT "tx_chars:0x%x--\n", txbuf[i - 1]);
#endif
    };

#ifdef WK_FIFO_FUNCTION
    len_tfcnt = i;
    while (len_tfcnt)
    {
        if (len_tfcnt > len_limit)
        {
            wk2xxx_write_fifo(s->spi_wk, one->port.iobase, len_limit, txbuf + len_p);
            len_p = len_p + len_limit;
            len_tfcnt = len_tfcnt - len_limit;
        }
        else
        {
            wk2xxx_write_fifo(s->spi_wk, one->port.iobase, len_tfcnt, txbuf + len_p);
            len_p = len_p + len_tfcnt;
            len_tfcnt = 0;
        }
    }
#else
    for (count = 0; count < i; count++)
    {
        wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FDAT_REG, txbuf[count]);
    }
#endif
out:
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FSR_REG, dat);
    fsr = dat[0];
    wk2xxx_dbg_rl(port->dev,
                  "tx_chars done port=%ld fsr=%#x remaining=%d\n",
                  one->port.iobase, fsr,
                  uart_circ_chars_pending(&one->port.state->xmit));
#ifdef _DEBUG_WK_VALUE
    printk(KERN_ALERT "%s!!-port:%ld;--FSR:0X%X--\n", __func__, one->port.iobase, fsr);
#endif
    if (((fsr & WK2XXX_FSR_TDAT_BIT) == 0) && ((fsr & WK2XXX_FSR_TBUSY_BIT) == 0))
    {
        if (uart_circ_chars_pending(&one->port.state->xmit) < WAKEUP_CHARS)
        {
            uart_write_wakeup(&one->port);
        }
        if (uart_circ_empty(&one->port.state->xmit))
        {
		
			// add by linx
            // printk("linx: %s, %d\n", __func__, __LINE__);
            if (port->rs485.flags & SER_RS485_ENABLED)
            {
                gpio_clr_rs485_txen(); // rs485_do_tasklet(port);//start_485_tasklet();//启动tasklet死机
            }
		
            wk2xxx_stop_tx(&one->port);
        }
    }
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--exit--\n", __func__, one->port.iobase);
#endif
}

static void wk2xxx_port_irq(struct wk2xxx_port *s, int portno) //
{
    // struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    struct wk2xxx_one *one = &s->p[portno];
    unsigned int pass_counter = 0;
    uint8_t sifr, sier;

#ifdef _DEBUG_WK_IRQ
    uint8_t gier, sifr0, sifr1, sifr2, sifr3, sier1, sier0, sier2, sier3, gifr;

#endif

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif

#ifdef _DEBUG_WK_IRQ
    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GIFR_REG, &gifr);
    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GIER_REG, &gier);
    wk2xxx_read_slave_reg(s->spi_wk, 1, WK2XXX_SIFR_REG, &sifr0);
    wk2xxx_read_slave_reg(s->spi_wk, 2, WK2XXX_SIFR_REG, &sifr1);
    wk2xxx_read_slave_reg(s->spi_wk, 3, WK2XXX_SIFR_REG, &sifr2);
    wk2xxx_read_slave_reg(s->spi_wk, 4, WK2XXX_SIFR_REG, &sifr3);
    wk2xxx_read_slave_reg(s->spi_wk, 1, WK2XXX_SIER_REG, &sier0);
    wk2xxx_read_slave_reg(s->spi_wk, 2, WK2XXX_SIER_REG, &sier1);
    wk2xxx_read_slave_reg(s->spi_wk, 3, WK2XXX_SIER_REG, &sier2);
    wk2xxx_read_slave_reg(s->spi_wk, 4, WK2XXX_SIER_REG, &sier3);
    printk(KERN_ALERT "irq_app....gifr:%x  gier:%x  sier1:%x  sier2:%x sier3:%x sier4:%x   sifr1:%x sifr2:%x sifr3:%x sifr4:%x \n", gifr, gier, sier0, sier1, sier2, sier3, sifr0, sifr1, sifr2, sifr3);
#endif
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIFR_REG, &sifr);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, &sier);
    wk2xxx_dbg_rl(&s->spi_wk->dev,
                  "port_irq port=%ld sifr=%#x sier=%#x\n",
                  one->port.iobase, sifr, sier);
#ifdef _DEBUG_WK_IRQ
    printk(KERN_ALERT "irq_app....port:%ld......sifr:%x sier:%x \n", one->port.iobase, sifr, sier);
#endif

    do
    {
        if ((sifr & WK2XXX_SIFR_RFTRIG_INT_BIT) || (sifr & WK2XXX_SIFR_RXOVT_INT_BIT))
        {
            wk2xxx_rx_chars(&one->port);
        }

        if ((sifr & WK2XXX_SIFR_TFTRIG_INT_BIT) && (sier & WK2XXX_SIER_TFTRIG_IEN_BIT))
        {
            wk2xxx_tx_chars(&one->port);
            return;
        }
        if (pass_counter++ > WK2XXX_ISR_PASS_LIMIT)
            break;
        wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIFR_REG, &sifr);
        wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, &sier);
#ifdef _DEBUG_WK_VALUE
        printk(KERN_ALERT "irq_app...........rx............tx  sifr:%x sier:%x port:%ld\n", sifr, sier, one->port.iobase);
#endif
    } while ((sifr & (WK2XXX_SIFR_RXOVT_INT_BIT | WK2XXX_SIFR_RFTRIG_INT_BIT)) || ((sifr & WK2XXX_SIFR_TFTRIG_INT_BIT) && (sier & WK2XXX_SIER_TFTRIG_IEN_BIT)));
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--exit--\n", __func__, one->port.iobase);
#endif
}

static void wk2xxx_ist(struct kthread_work *ws)
{
    struct wk2xxx_port *s = container_of(ws, struct wk2xxx_port, irq_work);

    uint8_t gifr, i;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif

    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GIFR_REG, &gifr);
    wk2xxx_dbg_rl(&s->spi_wk->dev, "ist enter gifr=%#x\n", gifr);
    while (1)
    {

        for (i = 0; i < s->devtype->nr_uart; ++i)
        {
            if (gifr & (0x01 << i))
            {
                wk2xxx_port_irq(s, i);
            }
        }

        wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GIFR_REG, &gifr);
        wk2xxx_dbg_rl(&s->spi_wk->dev, "ist loop gifr=%#x\n", gifr);
        if (!(gifr & 0x0f))
        {
            break;
        }
    }

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---exit--\n", __func__);
#endif
}

static irqreturn_t wk2xxx_irq(int irq, void *dev_id) //
{
    struct wk2xxx_port *s = (struct wk2xxx_port *)dev_id;
    bool ret;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif
#ifdef WK_WORK_KTHREAD
    ret = kthread_queue_work(&s->kworker, &s->irq_work);
#else
    ret = queue_kthread_work(&s->kworker, &s->irq_work);
#endif
    wk2xxx_dbg_rl(&s->spi_wk->dev, "irq fired irq=%d queued=%d\n", irq, ret);

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!ret:%d---exit--\n", __func__, ret);
#endif
    return IRQ_HANDLED;
}

/*
 *   Return TIOCSER_TEMT when transmitter is not busy.
 */

static u_int wk2xxx_tx_empty(struct uart_port *port) // or query the tx fifo is not empty?
{
    uint8_t fsr = 0;
    // struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif
    mutex_lock(&wk2xxxs_lock);

    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FSR_REG, &fsr);
    while ((fsr & WK2XXX_FSR_TDAT_BIT) | (fsr & WK2XXX_FSR_TBUSY_BIT))
    {
        wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FSR_REG, &fsr);
    }
    s->tx_empty = ((fsr & (WK2XXX_FSR_TBUSY_BIT | WK2XXX_FSR_TDAT_BIT)) == 0) ? TIOCSER_TEMT : 0;
    mutex_unlock(&wk2xxxs_lock);

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;tx_empty:0x%x,fsr:0x%x--exit--\n", __func__, one->port.iobase, s->tx_empty, fsr);
#endif
    return s->tx_empty;
}

static void wk2xxx_set_mctrl(struct uart_port *port, u_int mctrl)
{
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif
}
static u_int wk2xxx_get_mctrl(struct uart_port *port)
{
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif
    return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void wk2xxx_stop_tx(struct uart_port *port) //
{

    uint8_t sier;
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
    // struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif

    mutex_lock(&wk2xxxs_lock);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, &sier);
    sier &= ~WK2XXX_SIER_TFTRIG_IEN_BIT;
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, sier);
    mutex_unlock(&wk2xxxs_lock);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--exit--\n", __func__, one->port.iobase);
#endif
}

static void wk2xxx_start_tx_proc(struct kthread_work *ws)
{
    struct wk2xxx_one *one = to_wk2xxx_one(ws, start_tx_work);
    struct uart_port *port = &(to_wk2xxx_one(ws, start_tx_work)->port);
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);

    uint8_t rx;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif
    mutex_lock(&wk2xxxs_lock);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, &rx);
    rx |= WK2XXX_SIER_TFTRIG_IEN_BIT | WK2XXX_SIER_RFTRIG_IEN_BIT | WK2XXX_SIER_RXOUT_IEN_BIT;
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, rx);
    mutex_unlock(&wk2xxxs_lock);
}

/*
 *  *
 */
static void wk2xxx_start_tx(struct uart_port *port)
{
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
    bool ret;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif
	
	// add by linx
    // printk("linx: %s, %d\n", __func__, __LINE__);
    if (port->rs485.flags & SER_RS485_ENABLED)
        gpio_set_rs485_txen();

#ifdef WK_WORK_KTHREAD
    ret = kthread_queue_work(&s->kworker, &one->start_tx_work);
#else
    ret = queue_kthread_work(&s->kworker, &one->start_tx_work);
#endif	

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;ret=%d--exit--\n", __func__, one->port.iobase, ret);
#endif
}

static void wk2xxx_stop_rx_proc(struct kthread_work *ws)
{
    struct wk2xxx_one *one = to_wk2xxx_one(ws, stop_rx_work);
    struct uart_port *port = &(to_wk2xxx_one(ws, stop_rx_work)->port);
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    uint8_t rx;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif
    mutex_lock(&wk2xxxs_lock);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, &rx);
    rx &= ~WK2XXX_SIER_RFTRIG_IEN_BIT;
    rx &= ~WK2XXX_SIER_RXOUT_IEN_BIT;
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, rx);

    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SCR_REG, &rx);
    rx &= ~WK2XXX_SCR_RXEN_BIT;
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SCR_REG, rx);
    mutex_unlock(&wk2xxxs_lock);
}

static void wk2xxx_stop_rx(struct uart_port *port)
{
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
    bool ret;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);

#endif
#ifdef WK_WORK_KTHREAD
    ret = kthread_queue_work(&s->kworker, &one->stop_rx_work);
#else
    ret = queue_kthread_work(&s->kworker, &one->stop_rx_work);
#endif

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;ret:%d--exit--\n", __func__, one->port.iobase, ret);
#endif
}

/*
 *  * No modem control lines
 *   */
static void wk2xxx_enable_ms(struct uart_port *port) // nothing
{
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif
}
/*
 *  * Interrupts always disabled.
 */
static void wk2xxx_break_ctl(struct uart_port *port, int break_state)
{
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif
}

static int wk2xxx_startup(struct uart_port *port) // i
{
    uint8_t gena, grst, gier, sier, scr, dat[1];
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
    printk(KERN_ALERT "wk2xxx_start(iobase) port1:%ld,port2:%ld,port3:%ld,port4:%ld\n", s->p[0].port.iobase, s->p[1].port.iobase, s->p[2].port.iobase, s->p[3].port.iobase);
    printk(KERN_ALERT "wk2xxx_start(iobase) line1:%d,line2:%d,line3:%d,line4:%d\n", s->p[0].line, s->p[1].line, s->p[2].line, s->p[3].line);
#endif

    mutex_lock(&wk2xxxs_global_lock);
    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GENA_REG, dat);
    gena = dat[0];
    switch (one->port.iobase)
    {
    case 1:
        gena |= WK2XXX_GENA_UT1EN_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GENA_REG, gena);
        break;
    case 2:
        gena |= WK2XXX_GENA_UT2EN_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GENA_REG, gena);
        break;
    case 3:
        gena |= WK2XXX_GENA_UT3EN_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GENA_REG, gena);
        break;
    case 4:
        gena |= WK2XXX_GENA_UT4EN_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GENA_REG, gena);
        break;
    default:
        printk(KERN_ALERT ":%s！！ bad iobase1: %d.\n", __func__, (uint8_t)one->port.iobase);
        break;
    }

    // wk2xxx_read_global_reg(s->spi_wk,WK2XXX_GRST_REG,dat);
    grst = 0;
    switch (one->port.iobase)
    {
    case 1:
        grst |= WK2XXX_GRST_UT1RST_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GRST_REG, grst);
        break;
    case 2:
        grst |= WK2XXX_GRST_UT2RST_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GRST_REG, grst);
        break;
    case 3:
        grst |= WK2XXX_GRST_UT3RST_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GRST_REG, grst);
        break;
    case 4:
        grst |= WK2XXX_GRST_UT4RST_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GRST_REG, grst);
        break;
    default:
        printk(KERN_ALERT ":%s！！ bad iobase2: %d.\n", __func__, (uint8_t)one->port.iobase);
        break;
    }

    // enable the sub port interrupt
    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GIER_REG, dat);
    gier = dat[0];
    switch (one->port.iobase)
    {
    case 1:
        gier |= WK2XXX_GIER_UT1IE_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GIER_REG, gier);
        break;
    case 2:
        gier |= WK2XXX_GIER_UT2IE_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GIER_REG, gier);
        break;
    case 3:
        gier |= WK2XXX_GIER_UT3IE_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GIER_REG, gier);
        break;
    case 4:
        gier |= WK2XXX_GIER_UT4IE_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GIER_REG, gier);
        break;
    default:
        printk(KERN_ALERT ":%s！！bad iobase3: %d.\n", __func__, (uint8_t)one->port.iobase);
        break;
    }
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, dat);
    sier = dat[0];
    sier &= ~WK2XXX_SIER_TFTRIG_IEN_BIT;
    sier |= WK2XXX_SIER_RFTRIG_IEN_BIT;
    sier |= WK2XXX_SIER_RXOUT_IEN_BIT;
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, sier);

    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SCR_REG, dat);
    scr = dat[0] | WK2XXX_SCR_TXEN_BIT | WK2XXX_SCR_RXEN_BIT;
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SCR_REG, scr);

    // initiate the fifos
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FCR_REG, 0xff); // initiate the fifos
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FCR_REG, 0xfc);
    // set rx/tx interrupt
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SPAGE_REG, 1);
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_RFTL_REG, WK2XXX_RXFIFO_LEVEL);
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_TFTL_REG, WK2XXX_TXFIFO_LEVEL);
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SPAGE_REG, 0);

/*enable rs485*/
#ifdef WK_RS485_FUNCTION
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_RS485_REG, 0X02); // default  high
    // wk2xxx_write_slave_reg(s->spi_wk,one->port.iobase,WK2XXX_RS485,0X03);//default low
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SPAGE_REG, 0X01);
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_RRSDLY_REG, 0X10);
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SPAGE_REG, 0X00);
#endif
    /*****************************test**************************************/
#ifdef _DEBUG_WK_TEST
    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GENA_REG, &gena);
    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GIER_REG, &gier);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, &sier);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SCR_REG, &scr);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FCR_REG, dat);
    printk(KERN_ALERT "%s!!-port:%ld;gena:0x%x;gier:0x%x;sier:0x%x;scr:0x%x;fcr:0x%x----\n", __func__, one->port.iobase, gena, gier, sier, scr, dat[0]);
#endif
    /**********************************************************************/

    mutex_unlock(&wk2xxxs_global_lock);
    uart_circ_clear(&one->port.state->xmit);
    wk2xxx_enable_ms(&one->port);

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--exit--\n", __func__, one->port.iobase);
#endif
    return 0;
}

static void wk2xxx_shutdown(struct uart_port *port)
{

    uint8_t gena, grst, gier, dat[1];
    // struct wk2xxx_port *s = container_of(port,struct wk2xxx_port,port);
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif

    mutex_lock(&wk2xxxs_global_lock);
    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GIER_REG, &gier);
    switch (one->port.iobase)
    {
    case 1:
        gier &= ~WK2XXX_GIER_UT1IE_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GIER_REG, gier);
        break;
    case 2:
        gier &= ~WK2XXX_GIER_UT2IE_BIT;
        ;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GIER_REG, gier);
        break;
    case 3:
        gier &= ~WK2XXX_GIER_UT3IE_BIT;
        ;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GIER_REG, gier);
        break;
    case 4:
        gier &= ~WK2XXX_GIER_UT4IE_BIT;
        ;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GIER_REG, gier);
        break;
    default:
        printk(KERN_ALERT "%s!! (GIER)bad iobase %d\n", __func__, (uint8_t)one->port.iobase);
        ;
        break;
    }

    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, 0x0);
    mutex_unlock(&wk2xxxs_global_lock);

#ifdef WK_WORK_KTHREAD
    kthread_flush_work(&one->start_tx_work);
    kthread_flush_work(&one->stop_rx_work);
    kthread_flush_work(&s->irq_work);
    // kthread_flush_worker(&s->kworker);
#else
    flush_kthread_work(&one->start_tx_work);
    flush_kthread_work(&one->stop_rx_work);
    flush_kthread_work(&s->irq_work);
    // flush_kthread_worker(&s->kworker);
#endif

    mutex_lock(&wk2xxxs_global_lock);
    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GRST_REG, dat);
    grst = dat[0];
    switch (one->port.iobase)
    {
    case 1:
        grst |= WK2XXX_GRST_UT1RST_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GRST_REG, grst);
        break;
    case 2:
        grst |= WK2XXX_GRST_UT2RST_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GRST_REG, grst);
        break;
    case 3:
        grst |= WK2XXX_GRST_UT3RST_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GRST_REG, grst);
        break;
    case 4:
        grst |= WK2XXX_GRST_UT4RST_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GRST_REG, grst);
        break;
    default:
        printk(KERN_ALERT "%s!! bad iobase %d\n", __func__, (uint8_t)one->port.iobase);
        break;
    }

    wk2xxx_read_global_reg(s->spi_wk, WK2XXX_GENA_REG, dat);
    gena = dat[0];
    switch (one->port.iobase)
    {
    case 1:
        gena &= ~WK2XXX_GENA_UT1EN_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GENA_REG, gena);
        break;
    case 2:
        gena &= ~WK2XXX_GENA_UT2EN_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GENA_REG, gena);
        break;
    case 3:
        gena &= ~WK2XXX_GENA_UT3EN_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GENA_REG, gena);
        break;
    case 4:
        gena &= ~WK2XXX_GENA_UT4EN_BIT;
        wk2xxx_write_global_reg(s->spi_wk, WK2XXX_GENA_REG, gena);
        break;
    default:
        printk(KERN_ALERT "%s!! bad iobase %d\n", __func__, (uint8_t)one->port.iobase);
        ;
        break;
    }

    mutex_unlock(&wk2xxxs_global_lock);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--exit--\n", __func__, one->port.iobase);
#endif
}

static void conf_wk2xxx_subport(struct uart_port *port) // i
{
    struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
    uint8_t sier = 0, fwcr = 0, lcr = 0, scr = 0, dat[1], baud0 = 0, baud1 = 0, pres = 0, count = 200;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
#endif
    lcr = one->new_lcr_reg;
    // scr = s->new_scr_reg;
    baud0 = one->new_baud0_reg;
    baud1 = one->new_baud1_reg;
    pres = one->new_pres_reg;
    fwcr = one->new_fwcr_reg;
    /* Disable Uart all interrupts */
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, dat);
    sier = dat[0];
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, 0X0);

    do
    {
        wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FSR_REG, dat);
    } while ((dat[0] & WK2XXX_FSR_TBUSY_BIT) && (count--));
    // then, disable tx and rx
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SCR_REG, dat);
    scr = dat[0];
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SCR_REG, scr & (~(WK2XXX_SCR_RXEN_BIT | WK2XXX_SCR_TXEN_BIT)));
    // set the parity, stop bits and data size //
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_LCR_REG, lcr);
#ifdef WK_FlowControl_FUNCTION
    if (fwcr > 0)
    {
        printk(KERN_ALERT "%s!!---Flow Control  fwcr=0x%X\n", __func__, fwcr);
        // Configure flow control levels
        wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FWCR_REG, fwcr);
        // Flow control halt level 0XF0, resume level 0X80
        wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SPAGE_REG, 1);
        wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FWTH_REG, 0XF0);
        wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_FWTL_REG, 0X80);
        wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SPAGE_REG, 0);
    }
#endif
    /* Setup baudrate generator */
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SPAGE_REG, 1);
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_BAUD0_REG, baud0);
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_BAUD1_REG, baud1);
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_PRES_REG, pres);
#ifdef _DEBUG_WK_FUNCTION
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_BAUD0_REG, &baud1);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_BAUD1_REG, &baud0);
    wk2xxx_read_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_PRES_REG, &pres);
    printk(KERN_ALERT "%s!!---baud1:0x%x;baud0:0x%x;pres=0x%X.---\n", __func__, baud1, baud0, pres);
#endif
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SPAGE_REG, 0);
    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SCR_REG, scr | (WK2XXX_SCR_RXEN_BIT | WK2XXX_SCR_TXEN_BIT));

    wk2xxx_write_slave_reg(s->spi_wk, one->port.iobase, WK2XXX_SIER_REG, sier);

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--exit--\n", __func__, one->port.iobase);
#endif
}

static void wk2xxx_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{

    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
    int baud = 0;
    //uint32_t temp = 0, freq = 0;
    uint8_t lcr = 0, fwcr = 0, baud1 = 0, baud0 = 0, pres = 0, bParityType = 0;

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--in--\n", __func__, one->port.iobase);
    printk(KERN_ALERT "%s!!---c_cflag:0x%x,c_iflag:0x%x.\n", __func__, termios->c_cflag, termios->c_iflag);
#endif
    baud1 = 0;
    baud0 = 0;
    pres = 0;
    baud = tty_termios_baud_rate(termios);

    /*freq = one->port.uartclk;
    if (freq >= (baud * 16))
    {
        temp = (freq) / (baud * 16);
        temp = temp - 1;
        baud1 = (uint8_t)((temp >> 8) & 0xff);
        baud0 = (uint8_t)(temp & 0xff);
        temp = (((freq % (baud * 16)) * 100) / (baud));
        pres = (temp + 100 / 2) / 100;
        printk(KERN_ALERT "%s!!---freq:%d,baudrate:%d\n", __func__, freq, baud);
        printk(KERN_ALERT "%s!!---baud1:%x,baud0:%x,pres:%x\n", __func__, baud1, baud0, pres);
    }
    else
    {
        printk(KERN_ALERT "the baud rate:%d is too high！ \n", baud);
    }*/
	
#ifdef OSC_11MHZ
    switch (baud)
    {
    case 600:
        baud1 = 0x4;
        baud0 = 0x7f;
        pres = 0;
        break;
    case 1200:
        baud1 = 0x2;
        baud0 = 0x3F;
        pres = 0;
        break;
    case 2400:
        baud1 = 0x1;
        baud0 = 0x1f;
        pres = 0;
        break;
    case 4800:
        baud1 = 0x00;
        baud0 = 0x8f;
        pres = 0;
        break;
    case 9600:
        baud1 = 0x00;
        baud0 = 0x47;
        pres = 0;
        break;
    case 19200:
        baud1 = 0x00;
        baud0 = 0x23;
        pres = 0;
        break;
    case 38400:
        baud1 = 0x00;
        baud0 = 0x11;
        pres = 0;
        break;
    case 76800:
        baud1 = 0x00;
        baud0 = 0x08;
        pres = 0;
        break;
    case 1800:
        baud1 = 0x01;
        baud0 = 0x7f;
        pres = 0;
        break;
    case 3600:
        baud1 = 0x00;
        baud0 = 0xbf;
        pres = 0;
        break;
    case 7200:
        baud1 = 0x00;
        baud0 = 0x5f;
        pres = 0;
        break;
    case 14400:
        baud1 = 0x00;
        baud0 = 0x2f;
        pres = 0;
        break;
    case 28800:
        baud1 = 0x00;
        baud0 = 0x17;
        pres = 0;
        break;
    case 57600:
        baud1 = 0x00;
        baud0 = 0x0b;
        pres = 0;
        break;
    case 115200:
        baud1 = 0x00;
        baud0 = 0x05;
        pres = 0;
        break;
    case 230400:
        baud1 = 0x00;
        baud0 = 0x02;
        pres = 0;
        break;
    default: //默认\E4\B8?91200
        baud1 = 0x00;
        baud0 = 0x00;
        pres = 0;
        break;
    }
#else
#if 0
    //3.68mhz
        switch (baud) {
            case 600:
                baud1=0x1;
                baud0=0x7f;
                pres=0;
                break;
            case 1200:
                baud1=0x0;
                baud0=0xbF;
                pres=0;
                break;
            case 2400:
                baud1=0x0;
                baud0=0x5f;
                pres=0;
                break;
            case 4800:
                baud1=0x00;
                baud0=0x2f;
                pres=0;
                break;
            case 9600:
                baud1=0x00;
                baud0=0x17;
                pres=0;
                break;
            case 19200:
                baud1=0x00;
                baud0=0x0b;
                pres=0;
                break;
            case 38400:
                baud1=0x00;
                baud0=0x05;
                pres=0;
                break;
            case 76800:
                baud1=0x00;
                baud0=0x02;
                pres=0;
                break;
            case 1800:
                baud1=0x00;
                baud0=0x7f;
                pres=0;
                break;
            case 3600:
                baud1=0x00;
                baud0=0x3f;
                pres=0;
                break;
            case 7200:
                baud1=0x00;
                baud0=0x1f;
                pres=0;
                break;
            case 14400:
                baud1=0x00;
                baud0=0x0f;
                pres=0;
                break;
            case 28800:
                baud1=0x00;
                baud0=0x07;
                pres=0;
                break;
            case 57600:
                baud1=0x00;
                baud0=0x03;
                pres=0;
                break;
            case 115200:
                baud1=0x00;
                baud0=0x01;
                pres=0;
                break;
            case 230400:
                baud1=0x00;
                baud0=0x00;
                pres=0;
                break;
            default:     //默认\9600
                baud1=0x00;
                baud0=0x17;
                pres=0;
                break;
        }
#else
    // OSC_24MHZ
    switch (baud)
    {
    case 600:
        baud1 = 0x9;
        baud0 = 0xC3;
        pres = 0;
        break;
    case 1200:
        baud1 = 0x4;
        baud0 = 0xE1;
        pres = 0;
        break;
    case 2400:
        baud1 = 0x2;
        baud0 = 0x70;
        pres = 0;
        break;
    case 4800:
        baud1 = 0x01;
        baud0 = 0x37;
        pres = 0x05;
        break;
    case 9600:
        baud1 = 0x00;
        baud0 = 0x9B;
        pres = 0x02;
        break;
    case 19200:
        baud1 = 0x00;
        baud0 = 0x4D;
        pres = 0x01;
        break;
    case 38400:
        baud1 = 0x00;
        baud0 = 0x26;
        pres = 0;
        break;
    case 76800:
        baud1 = 0x00;
        baud0 = 0x12;
        pres = 0x05;
        break;
    case 1800:
        baud1 = 0x03;
        baud0 = 0x40;
        pres = 0x03;
        break;
    case 3600:
        baud1 = 0x01;
        baud0 = 0x9f;
        pres = 0x06;
        break;
    case 7200:
        baud1 = 0x00;
        baud0 = 0xcf;
        pres = 0x03;
        break;
    case 14400:
        baud1 = 0x00;
        baud0 = 0x67;
        pres = 0x01;
        break;
    case 28800:
        baud1 = 0x00;
        baud0 = 0x33;
        pres = 0;
        break;
    case 57600:
        baud1 = 0x00;
        baud0 = 0x19;
        pres = 0;
        break;
    case 115200:
        baud1 = 0x00;
        baud0 = 0x0c;
        pres = 0;
        break;
    case 230400:
        baud1 = 0x00;
        baud0 = 0x05;
        pres = 0x05;
        break;
    default: //默认\E4\B8?91200
        baud1 = 0x00;
        baud0 = 0x00;
        pres = 0;
        break;
    }

#endif

#endif
    tty_termios_encode_baud_rate(termios, baud, baud);

    lcr = 0;
    if (termios->c_cflag & CSTOPB)
        lcr |= WK2XXX_LCR_STPL_BIT; // two  stop_bits
    else
        lcr &= ~WK2XXX_LCR_STPL_BIT; // one  stop_bits

    bParityType = termios->c_cflag & PARENB ? (termios->c_cflag & PARODD ? 1 : 2) + (termios->c_cflag & CMSPAR ? 2 : 0) : 0;
    if (termios->c_cflag & PARENB)
    {
        lcr |= WK2XXX_LCR_PAEN_BIT; // enbale spa
        switch (bParityType)
        {
        case 0x01: // ODD
            lcr |= WK2XXX_LCR_PAM0_BIT;
            lcr &= ~WK2XXX_LCR_PAM1_BIT;
            break;
        case 0x02: // EVEN
            lcr |= WK2XXX_LCR_PAM1_BIT;
            lcr &= ~WK2XXX_LCR_PAM0_BIT;
            break;
        case 0x03: // MARK--1
            lcr |= WK2XXX_LCR_PAM1_BIT | WK2XXX_LCR_PAM0_BIT;
            break;
        case 0x04: // SPACE--0
            lcr &= ~WK2XXX_LCR_PAM1_BIT;
            lcr &= ~WK2XXX_LCR_PAM0_BIT;
            break;
        default:
            lcr &= ~WK2XXX_LCR_PAEN_BIT;
            break;
        }
    }

    /* Set read status mask */
    port->read_status_mask = WK2XXX_LSR_OE_BIT;
    if (termios->c_iflag & INPCK)
        port->read_status_mask |= WK2XXX_LSR_PE_BIT |
                                  WK2XXX_LSR_FE_BIT;
    if (termios->c_iflag & (BRKINT | PARMRK))
        port->read_status_mask |= WK2XXX_LSR_BI_BIT;

    /* Set status ignore mask */
    port->ignore_status_mask = 0;
    if (termios->c_iflag & IGNBRK)
        port->ignore_status_mask |= WK2XXX_LSR_BI_BIT;
    if (!(termios->c_cflag & CREAD))
        port->ignore_status_mask |= WK2XXX_LSR_BRK_ERROR_MASK;

#ifdef WK_FlowControl_FUNCTION
    /* Configure flow control */
    if (termios->c_cflag & CRTSCTS)
    {
        fwcr = 0X30;
        printk(KERN_ALERT "wk2xxx_termios(2)----port:%lx;lcr:0x%x;fwcr:0x%x---\n", one->port.iobase, lcr, fwcr);
    }

    if (termios->c_iflag & IXON)
    {
        printk(KERN_ALERT "%s!!---c_cflag:0x%x,IXON:0x%x.\n", __func__, termios->c_cflag, IXON);
    }
    if (termios->c_iflag & IXOFF)
    {
        printk(KERN_ALERT "%s!!---c_cflag:0x%x,IXOFF:0x%x.\n", __func__, termios->c_cflag, IXOFF);
    }
#endif

    one->new_baud1_reg = baud1;
    one->new_baud0_reg = baud0;
    one->new_pres_reg = pres;
    one->new_lcr_reg = lcr;
    one->new_fwcr_reg = fwcr;

#ifdef _DEBUG_WK_VALUE
    printk(KERN_ALERT "wk2xxx_termios()----port:%lx;lcr:0x%x;fwcr:0x%x---\n", one->port.iobase, lcr, fwcr);
#endif

    conf_wk2xxx_subport(&one->port);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!-port:%ld;--exit--\n", __func__, one->port.iobase);
#endif
}

static const char *wk2xxx_type(struct uart_port *port)
{

#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif
    return port->type == PORT_WK2XXX ? "wk2xxx" : NULL;
}

static void wk2xxx_release_port(struct uart_port *port)
{
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif
}

static int wk2xxx_request_port(struct uart_port *port)
{
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif
    return 0;
}

static void wk2xxx_config_port(struct uart_port *port, int flags)
{
    // struct wk2xxx_port *s = dev_get_drvdata(port->dev);
    struct wk2xxx_one *one = to_wk2xxx_one(port, port);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!\n", __func__);
#endif

    if (flags & UART_CONFIG_TYPE && wk2xxx_request_port(port) == 0)
        one->port.type = PORT_WK2XXX;
}

static int wk2xxx_verify_port(struct uart_port *port, struct serial_struct *ser)
{

    int ret = 0;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!---in--\n", __func__);
#endif
    if (ser->type != PORT_UNKNOWN && ser->type != PORT_WK2XXX)
        ret = -EINVAL;
    if (port->irq != ser->irq)
        ret = -EINVAL;
    if (ser->io_type != SERIAL_IO_PORT)
        ret = -EINVAL;
    // if (port->uartclk / 16 != ser->baud_base)
    //      ret = -EINVAL;
    if (port->iobase != ser->port)
        ret = -EINVAL;
    if (ser->hub6 != 0)
        ret = -EINVAL;
    return ret;
}

static struct uart_ops wk2xxx_pops = {
    tx_empty : wk2xxx_tx_empty,
    set_mctrl : wk2xxx_set_mctrl,
    get_mctrl : wk2xxx_get_mctrl,
    stop_tx : wk2xxx_stop_tx,
    start_tx : wk2xxx_start_tx,
    stop_rx : wk2xxx_stop_rx,
    enable_ms : wk2xxx_enable_ms,
    break_ctl : wk2xxx_break_ctl,
    startup : wk2xxx_startup,
    shutdown : wk2xxx_shutdown,
    set_termios : wk2xxx_termios,
    type : wk2xxx_type,
    release_port : wk2xxx_release_port,
    request_port : wk2xxx_request_port,
    config_port : wk2xxx_config_port,
    verify_port : wk2xxx_verify_port,

};
static struct uart_driver wk2xxx_uart_driver = {

    owner : THIS_MODULE,
    major : SERIAL_WK2XXX_MAJOR,

    driver_name : "ttyWK",
    dev_name : "ttyWK",

    minor : MINOR_START,
    nr : NR_PORTS,
    cons : NULL
};

static int uart_driver_registered;
static struct spi_driver wk2xxx_driver;

#ifdef WK_RSTGPIO_FUNCTION
static int wk2xxx_spi_rstgpio_parse_dt(struct device *dev, int *rst_gpio)
{

    enum of_gpio_flags rst_flags;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!--in--\n", __func__);
#endif
    *rst_gpio = of_get_named_gpio_flags(dev->of_node, "reset_gpio", 0, &rst_flags);
    if (!gpio_is_valid(*rst_gpio))
    {
        printk(KERN_ERR "invalid wk2xxx_rst_gpio: %d\n", *rst_gpio);
        return -1;
    }

    if (*rst_gpio)
    {
        if (gpio_request(*rst_gpio, "rst_gpio"))
        {
            printk(KERN_ERR "gpio_request failed!! rst_gpio: %d!\n", *rst_gpio);
            gpio_free(*rst_gpio);
            return IRQ_NONE;
        }
    }
    gpio_direction_output(*rst_gpio, 1); // output high
    printk(KERN_ERR "wk2xxx_rst_gpio: %d", *rst_gpio);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!--exit--\n", __func__);
#endif
    return 0;
}

#endif

#ifdef WK_CSGPIO_FUNCTION

static int wk2xxx_spi_csgpio_parse_dt(struct device *dev, int *cs_gpio)
{

    enum of_gpio_flags cs_flags;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!--in--\n", __func__);
#endif
    *cs_gpio = of_get_named_gpio_flags(dev->of_node, "cs-gpios", 0, &cs_flags);
    if (!gpio_is_valid(*cs_gpio))
    {
        printk(KERN_ERR "invalid wk2xxx_cs_gpio: %d\n", *cs_gpio);
        return -1;
    }

    if (*cs_gpio)
    {
        if (gpio_request(*cs_gpio, "cs-gpios"))
        {
            printk(KERN_ERR "gpio_request failed!! cs_gpio: %d!\n", *cs_gpio);
            gpio_free(*cs_gpio);
            return IRQ_NONE;
        }
    }
    printk(KERN_ERR "wk2xxx_cs_gpio: %d", *cs_gpio);
    gpio_direction_output(*cs_gpio, 1); // output high
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!--exit--\n", __func__);
#endif
    return 0;
}
#endif

static int wk2xxx_spi_irq_parse_dt(struct device *dev, int *irq_gpio)
{

    enum of_gpio_flags irq_flags;
    int irq;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!--in--\n", __func__);
#endif
    *irq_gpio = of_get_named_gpio_flags(dev->of_node, "irq-gpio", 0, &irq_flags);
    if (!gpio_is_valid(*irq_gpio))
    {
        printk(KERN_ERR "invalid wk2xxx_irq_gpio: %d\n", *irq_gpio);
        return -1;
    }

    irq = gpio_to_irq(*irq_gpio);

    if (irq)
    {
        if (gpio_request(*irq_gpio, "irq-gpio"))
        {
            printk(KERN_ERR "gpio_request failed!! irq_gpio: %d!\n", irq);
            gpio_free(*irq_gpio);
            return IRQ_NONE;
        }
    }
    else
    {
        printk(KERN_ERR "gpio_to_irq failed! irq: %d !\n", irq);
        return -ENODEV;
    }

    printk(KERN_ERR "wk2xxx_irq_gpio: %d, irq: %d", *irq_gpio, irq);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!--exit--\n", __func__);
#endif
    return irq;
}

static int wk2xxx_probe(struct spi_device *spi)
{
    const struct sched_param sched_param = {.sched_priority = MAX_RT_PRIO / 2};
    // const struct sched_param sched_param = { .sched_priority = 100 / 2 };
    uint8_t i;
	int time = 2;
    int ret, irq;
    uint8_t dat[1];
    static struct wk2xxx_port *s;
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!--in--\n", __func__);
#endif

    /* Setup SPI bus */
    spi->bits_per_word = 8;
    /* only supports mode 0 on WK2124 */
    spi->mode = spi->mode ?: SPI_MODE_0;
    spi->max_speed_hz = spi->max_speed_hz ?: 10000000;
    ret = spi_setup(spi);
    if (ret)
        return ret;
    /* Alloc port structure */
    s = devm_kzalloc(&spi->dev, sizeof(*s) + sizeof(struct wk2xxx_one) * NR_PORTS, GFP_KERNEL);
    if (!s)
    {
        printk(KERN_ALERT "wk2xxx_probe(devm_kzalloc) fail.\n");
        return -ENOMEM;
    }
    s->spi_wk = spi;
    s->devtype = &wk2124_devtype;
    dev_set_drvdata(&spi->dev, s);
    ret = wk2xxx_mmio_init(spi, s);
    if (ret)
    {
        printk(KERN_ALERT "wk2xxx_probe(mmio_init) fail. ret=%d\n", ret);
        goto out_gpio;
    }
    wk2xxx_dbg(&spi->dev,
               "probe start chip_select=%u max_speed=%u mode=%u irq_gpio_prop pending\n",
               spi->chip_select, spi->max_speed_hz, spi->mode);
#ifdef WK_RSTGPIO_FUNCTION
    // Obtain the GPIO number of RST signal
    ret = wk2xxx_spi_rstgpio_parse_dt(&spi->dev, &s->rst_gpio_num);
    if (ret != 0)
    {
        printk(KERN_ALERT "wk2xxx_probe(rst_gpio_num)  rst_gpio_num= 0x%d\n", s->rst_gpio_num);
        ret = s->rst_gpio_num;
        goto out_gpio;
    }
    /*reset wk2xxx*/
    mdelay(10);
    gpio_set_value(s->rst_gpio_num, 0);
    mdelay(10);
    gpio_set_value(s->rst_gpio_num, 1);
    mdelay(10);
#endif

#ifdef WK_CSGPIO_FUNCTION
    // Obtain the GPIO number of CS signal
    ret = wk2xxx_spi_csgpio_parse_dt(&spi->dev, &cs_gpio_num);
    if (ret != 0)
    {
        printk(KERN_ALERT "wk2xxx_probe(cs_gpio)  cs_gpio_num = 0x%d\n", cs_gpio_num);
        ret = cs_gpio_num;
        goto out_gpio;
    }
#endif

    // Obtain the IRQ signal GPIO number and interrupt number
    irq = wk2xxx_spi_irq_parse_dt(&spi->dev, &s->irq_gpio_num);
    if (irq < 0)
    {
        printk(KERN_ALERT "wk2xxx_probe(irq_gpio)  irq = 0x%d\n", irq);
        ret = irq;
        goto out_gpio;
    }
    s->irq_gpio = irq;

    /**********************test spi **************************************/

    do
    {
        wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
        wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
        wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, 0xf5);
        wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
        wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, 0xff);
        wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
        wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, 0xf0);
		if (dat[0] == 0)
		{
			msleep(100);
        	time--;			
		} else {
			break;
		}
    } while (time > 0);
    
    /*Get interrupt number*/
    wk2xxx_write_global_reg(spi, WK2XXX_GENA_REG, 0x0);
    wk2xxx_read_global_reg(spi, WK2XXX_GENA_REG, dat);
    if ((dat[0] & 0xf0) != 0x30)
    {
        printk(KERN_ALERT "wk2xxx_probe(0x30)  GENA = 0x%X\n", dat[0]);
        printk(KERN_ALERT "The spi failed to read the register.!!!!\n");
        //ret = -1;
       // goto out_gpio;
    }
    /*Init kthread_worker  and kthread_work */

#ifdef WK_WORK_KTHREAD
    kthread_init_worker(&(s->kworker));
    kthread_init_work(&s->irq_work, wk2xxx_ist);
#else
    init_kthread_worker(&(s->kworker));
    init_kthread_work(&s->irq_work, wk2xxx_ist);
#endif
    s->kworker_task = kthread_run(kthread_worker_fn, &s->kworker,
                                  "wk2xxx");
    if (IS_ERR(s->kworker_task))
    {
        ret = PTR_ERR(s->kworker_task);
        goto out_clk;
    }
    sched_setscheduler(s->kworker_task, SCHED_FIFO, &sched_param);

    /**/
    mutex_lock(&wk2xxxs_lock);
    if (!uart_driver_registered)
    {
        uart_driver_registered = 1;
        ret = uart_register_driver(&wk2xxx_uart_driver);
        if (ret)
        {
            printk(KERN_ERR "Couldn't register Wk2xxx uart driver\n");
            mutex_unlock(&wk2xxxs_lock);
            //goto out_clk;
        }
    }

    for (i = 0; i < NR_PORTS; i++)
    {
        s->p[i].line = i;
        s->p[i].port.dev = &spi->dev;
        s->p[i].port.line = i;
        s->p[i].port.ops = &wk2xxx_pops;
        s->p[i].port.uartclk = WK_CRASTAL_CLK;
        s->p[i].port.fifosize = 256;
        s->p[i].port.iobase = i + 1;
        // s->p[i].port.irq      = irq;
        s->p[i].port.iotype = SERIAL_IO_PORT;
        s->p[i].port.flags = UPF_BOOT_AUTOCONF;
        // s->p[i].port.flags    = ASYNC_BOOT_AUTOCONF;
        // s->p[i].port.iotype   = UPIO_PORT;
        // s->p[i].port.flags    = UPF_FIXED_TYPE | UPF_LOW_LATENCY;
        s->p[i].port.rs485_config = wk2xxx_rs485_config; // add by linx
#ifdef WK_WORK_KTHREAD
        kthread_init_work(&s->p[i].start_tx_work, wk2xxx_start_tx_proc);
        kthread_init_work(&s->p[i].stop_rx_work, wk2xxx_stop_rx_proc);
#else
        init_kthread_work(&s->p[i].start_tx_work, wk2xxx_start_tx_proc);
        init_kthread_work(&s->p[i].stop_rx_work, wk2xxx_stop_rx_proc);
#endif
        /* Register uart port */
        ret = uart_add_one_port(&wk2xxx_uart_driver, &s->p[i].port);
        if (ret < 0)
        {
            printk(KERN_ALERT "uart_add_one_port failed for line i:= %d with error %d\n", i, ret);
            mutex_unlock(&wk2xxxs_lock);
            goto out_port;
        }

    }

    mutex_unlock(&wk2xxxs_lock);

    /* Setup interrupt */
    ret = devm_request_irq(&spi->dev, irq, wk2xxx_irq, IRQF_TRIGGER_FALLING, dev_name(&spi->dev), s);

    if (!ret)
    {
        return 0;
    }

out_port:
    for (i = 0; i < NR_PORTS; i++)
    {
        printk(KERN_ALERT "uart_remove_one_port：%ld. status= 0x%d\n", s->p[i].port.iobase, ret);
        uart_remove_one_port(&wk2xxx_uart_driver, &s->p[i].port);
    }
out_clk:
    kthread_stop(s->kworker_task);
out_gpio:
    wk2xxx_mmio_exit(s);
    if (s->irq_gpio_num > 0)
    {
        printk(KERN_ALERT "gpio_free(s->irq_gpio_num)= 0x%d,ret=0x%d\n", s->irq_gpio_num, ret);
        gpio_free(s->irq_gpio_num);
        s->irq_gpio_num = 0;
    }
    if (s->rst_gpio_num > 0)
    {
        printk(KERN_ALERT "gpio_free(s->rst_gpio_num)= 0x%d,ret=0x%d\n", s->rst_gpio_num, ret);
        gpio_free(s->rst_gpio_num);
        s->rst_gpio_num = 0;
    }
#ifdef WK_CSGPIO_FUNCTION
    if (cs_gpio_num > 0)
    {
        printk(KERN_ALERT "gpio_free(cs_gpio_num)= 0x%d,ret=0x%d\n", cs_gpio_num, ret);
        gpio_free(cs_gpio_num);
        cs_gpio_num = 0;
    }
#endif
    return ret;
}

static int wk2xxx_remove(struct spi_device *spi)
{

    int i;
    struct wk2xxx_port *s = dev_get_drvdata(&spi->dev);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!--in--\n", __func__);
#endif

    mutex_lock(&wk2xxxs_lock);
    for (i = 0; i < NR_PORTS; i++)
    {
        uart_remove_one_port(&wk2xxx_uart_driver, &s->p[i].port);
        printk(KERN_ALERT "%s!--uart_remove_one_port：%d.\n", __func__, i);
    }
#ifdef WK_WORK_KTHREAD
    kthread_flush_worker(&s->kworker);
#else
    flush_kthread_worker(&s->kworker);
#endif
    kthread_stop(s->kworker_task);
    /*
    if (s->irq_gpio){
        free_irq(s->irq_gpio, s);
        printk(KERN_ALERT "%s!--,free_irq(s->irq_gpio, s);\n", __func__);
    }
    */
    if (s->irq_gpio_num > 0)
    {
        gpio_free(s->irq_gpio_num);
        printk(KERN_ALERT "%s!--,gpio_free(s->irq_gpio_num);\n", __func__);
    }
    if (s->rst_gpio_num > 0)
    {
        gpio_free(s->rst_gpio_num);
        printk(KERN_ALERT "%s!--,gpio_free(s->rst_gpio_num);\n", __func__);
    }

#ifdef WK_CSGPIO_FUNCTION
    if (cs_gpio_num > 0)
    {
        gpio_free(cs_gpio_num);
        printk(KERN_ALERT "%s!--,gpio_free(cs_gpio_num); ;\n", __func__);
    }
#endif

    printk(KERN_ERR "removing wk2xxx_uart_driver\n");
    uart_unregister_driver(&wk2xxx_uart_driver);
    mutex_unlock(&wk2xxxs_lock);
    wk2xxx_mmio_exit(s);
    devm_kfree(&spi->dev, s);
#ifdef _DEBUG_WK_FUNCTION
    printk(KERN_ALERT "%s!!--exit--\n", __func__);
#endif
    return 0;
}

static const struct of_device_id wkmic_spi_dt_match[] = {
    //{ .compatible = "wkmic,wk2124_spi",  },
    {
        .compatible = "wk2xxxspi,2124",
    },
    {},
};

MODULE_DEVICE_TABLE(of, wkmic_spi_dt_match);

static struct spi_driver wk2xxx_driver = {
    .driver = {
        .name = "wk2xxxspi",
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(wkmic_spi_dt_match),
    },

    .probe = wk2xxx_probe,
    .remove = wk2xxx_remove,
};

static int __init wk2xxx_init(void)
{

    int ret;
    printk(KERN_ALERT "%s: " DRIVER_DESC "\n", __func__);
    printk(KERN_ALERT "%s: " VERSION_DESC "\n", __func__);
    ret = spi_register_driver(&wk2xxx_driver);
    if (ret < 0)
    {
        printk(KERN_ALERT "%s,failed to init wk2xxx spi;ret= :%d\n", __func__, ret);
    }
    return ret;
}

static void __exit wk2xxx_exit(void)
{

    printk(KERN_ALERT "%s!!--in--\n", __func__);
    return spi_unregister_driver(&wk2xxx_driver);
}

module_init(wk2xxx_init);
module_exit(wk2xxx_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");