/**
  * Copyright (c) 2022 Rockchip Electronics Co., Ltd
  */
#include <getopt.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <stdbool.h>
#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <unistd.h>
#include <sys/eventfd.h>
#include <semaphore.h>
#include <time.h>

typedef unsigned char u8;
typedef unsigned int u32;
#define min(x, y) ({ typeof(x) _x = (x); typeof(y) _y = (y); (void) (&_x == &_y); _x < _y ? _x : _y; })

/* Change to 1 to output registers at the start of each transaction */
#define DEBUG_RK_SPI	0

#define ROCKCHIP_SPI_VER2_TYPE1			0x05EC0002
#define ROCKCHIP_SPI_VER2_TYPE2			0x00110002

struct rockchip_spi {
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
	u32 dmardlr;		/* 0x44 */
	u32 ver;
	u32 reserved[0xee];
	u32 txdr[0x100];	/* 0x400 */
	u32 rxdr[0x100];	/* 0x800 */
};


/* CTRLR0 */
enum {
	DFS_SHIFT	= 0,	/* Data Frame Size */
	DFS_MASK	= 3,
	DFS_4BIT	= 0,
	DFS_8BIT,
	DFS_16BIT,
	DFS_RESV,

	CFS_SHIFT	= 2,	/* Control Frame Size */
	CFS_MASK	= 0xf,

	SCPH_SHIFT	= 6,	/* Serial Clock Phase */
	SCPH_MASK	= 1,
	SCPH_TOGMID	= 0,	/* SCLK toggles in middle of first data bit */
	SCPH_TOGSTA,		/* SCLK toggles at start of first data bit */

	SCOL_SHIFT	= 7,	/* Serial Clock Polarity */
	SCOL_MASK	= 1,
	SCOL_LOW	= 0,	/* Inactive state of serial clock is low */
	SCOL_HIGH,		/* Inactive state of serial clock is high */

	CSM_SHIFT	= 8,	/* Chip Select Mode */
	CSM_MASK	= 0x3,
	CSM_KEEP	= 0,	/* ss_n stays low after each frame  */
	CSM_HALF,		/* ss_n high for half sclk_out cycles */
	CSM_ONE,		/* ss_n high for one sclk_out cycle */
	CSM_RESV,

	SSN_DELAY_SHIFT	= 10,	/* SSN to Sclk_out delay */
	SSN_DELAY_MASK	= 1,
	SSN_DELAY_HALF	= 0,	/* 1/2 sclk_out cycle */
	SSN_DELAY_ONE	= 1,	/* 1 sclk_out cycle */

	SEM_SHIFT	= 11,	/* Serial Endian Mode */
	SEM_MASK	= 1,
	SEM_LITTLE	= 0,	/* little endian */
	SEM_BIG,		/* big endian */

	FBM_SHIFT	= 12,	/* First Bit Mode */
	FBM_MASK	= 1,
	FBM_MSB		= 0,	/* first bit is MSB */
	FBM_LSB,		/* first bit in LSB */

	HALF_WORD_TX_SHIFT = 13,	/* Byte and Halfword Transform */
	HALF_WORD_MASK	= 1,
	HALF_WORD_ON	= 0,	/* apb 16bit write/read, spi 8bit write/read */
	HALF_WORD_OFF,		/* apb 8bit write/read, spi 8bit write/read */

	RXDSD_SHIFT	= 14,	/* Rxd Sample Delay, in cycles */
	RXDSD_MASK	= 3,

	FRF_SHIFT	= 16,	/* Frame Format */
	FRF_MASK	= 3,
	FRF_SPI		= 0,	/* Motorola SPI */
	FRF_SSP,			/* Texas Instruments SSP*/
	FRF_MICROWIRE,		/* National Semiconductors Microwire */
	FRF_RESV,

	TMOD_SHIFT	= 18,	/* Transfer Mode */
	TMOD_MASK	= 3,
	TMOD_TR		= 0,	/* xmit & recv */
	TMOD_TO,		/* xmit only */
	TMOD_RO,		/* recv only */
	TMOD_RESV,

	OMOD_SHIFT	= 20,	/* Operation Mode */
	OMOD_MASK	= 1,
	OMOD_MASTER	= 0,	/* Master Mode */
	OMOD_SLAVE,		/* Slave Mode */
};

/* SR */
enum {
	SR_MASK		= 0x7f,
	SR_BUSY		= 1 << 0,
	SR_TF_FULL	= 1 << 1,
	SR_TF_EMPT	= 1 << 2,
	SR_RF_EMPT	= 1 << 3,
	SR_RF_FULL	= 1 << 4,
};

#define ROCKCHIP_SPI_TIMEOUT_US		1000000

#define SPI_XFER_BEGIN		(1 << 0)	/* Assert CS before transfer */
#define SPI_XFER_END		(1 << 1)	/* Deassert CS after transfer */
#define SPI_XFER_ONCE		(SPI_XFER_BEGIN | SPI_XFER_END)

/* SPI mode flags */
#define SPI_CPHA	(1 << 0)			/* clock phase */
#define SPI_CPOL	(1 << 1)			/* clock polarity */
#define SPI_MODE_0	(0 | 0)			/* (original MicroWire) */
#define SPI_MODE_1	(0 | SPI_CPHA)
#define SPI_MODE_2	(SPI_CPOL | 0)
#define SPI_MODE_3	(SPI_CPOL | SPI_CPHA)

struct rockchip_spi_priv {
	struct rockchip_spi *regs;
	unsigned int mode;
	u8 bits_per_word;		/* max 16 bits per word */
	u8 n_bytes;
	unsigned int clock_div;
	uint cr0;
	u32 rsd;			/* Rx sample delay cycles */
	u32 fifo_len;
	u32 fifo_rx_th;			/* wait for rx thethold then read fifo */
	sem_t	*lock;			/* Lock for thread */
};

static struct rockchip_spi_priv *spi_bus;

#define SPI_FIFO_DEPTH		32
#define SPI_CR0_RSD_MAX		0x3

static uint64_t nano_time(void)
{
	struct timespec t;

	t.tv_sec = t.tv_nsec = 0;
	clock_gettime(CLOCK_MONOTONIC, &t);

	return (uint64_t)(t.tv_sec) * 1000000000 + t.tv_nsec;
}

static inline void writel(u32 val, void *addr)
{
	*(volatile u32 *)addr = val;
}

static inline u32 readl(void *addr)
{
	return *(volatile u32 *)addr;
}

static void rkspi_dump_regs(struct rockchip_spi *regs)
{
	printf("ctrl0: \t\t0x%08x\n", readl(&regs->ctrlr0));
	printf("ctrl1: \t\t0x%08x\n", readl(&regs->ctrlr1));
	printf("ssienr: \t\t0x%08x\n", readl(&regs->enr));
	printf("ser: \t\t0x%08x\n", readl(&regs->ser));
	printf("baudr: \t\t0x%08x\n", readl(&regs->baudr));
	printf("txftlr: \t\t0x%08x\n", readl(&regs->txftlr));
	printf("rxftlr: \t\t0x%08x\n", readl(&regs->rxftlr));
	printf("txflr: \t\t0x%08x\n", readl(&regs->txflr));
	printf("rxflr: \t\t0x%08x\n", readl(&regs->rxflr));
	printf("sr: \t\t0x%08x\n", readl(&regs->sr));
	printf("imr: \t\t0x%08x\n", readl(&regs->imr));
	printf("isr: \t\t0x%08x\n", readl(&regs->isr));
	printf("dmacr: \t\t0x%08x\n", readl(&regs->dmacr));
	printf("dmatdlr: \t0x%08x\n", readl(&regs->dmatdlr));
	printf("dmardlr: \t0x%08x\n", readl(&regs->dmardlr));
}

static void rkspi_enable_chip(struct rockchip_spi *regs, bool enable)
{
	writel(enable ? 1 : 0, &regs->enr);
}

static void rkspi_set_baudr(struct rockchip_spi_priv *priv, uint clk_div)
{
	writel(clk_div, &priv->regs->baudr);
}

static int rkspi_wait_till_not_busy(struct rockchip_spi *regs)
{
	unsigned long start;
	unsigned long long start_us, gap;

	start_us = nano_time();
	while (readl(&regs->sr) & SR_BUSY) {
		gap = nano_time() - start_us;
		if (gap > ROCKCHIP_SPI_TIMEOUT_US) {
			printf("RK SPI: Status keeps busy for 1000us after a read/write!\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static void spi_cs_activate(u8 cs)
{
	struct rockchip_spi_priv *priv = spi_bus;
	struct rockchip_spi *regs = priv->regs;

	//printf("activate cs%u\n", cs);
	writel(1 << cs, &regs->ser);
}

static void spi_cs_deactivate(u8 cs)
{
	struct rockchip_spi_priv *priv = spi_bus;
	struct rockchip_spi *regs = priv->regs;

	//printf("deactivate cs%u\n", cs);
	writel(0, &regs->ser);
}

static int rockchip_spi_claim_bus(void)
{
	struct rockchip_spi_priv *priv = spi_bus;
	struct rockchip_spi *regs = priv->regs;
	u8 spi_dfs, spi_tf;
	uint ctrlr0;

	/* Disable the SPI hardware */
	rkspi_enable_chip(regs, false);

	switch (priv->bits_per_word) {
	case 8:
		priv->n_bytes = 1;
		spi_dfs = DFS_8BIT;
		spi_tf = HALF_WORD_OFF;
		break;
	case 16:
		priv->n_bytes = 2;
		spi_dfs = DFS_16BIT;
		spi_tf = HALF_WORD_ON;
		break;
	default:
		printf("%s: unsupported bits: %dbits\n", __func__,
		      priv->bits_per_word);
		return -EPROTONOSUPPORT;
	}

	rkspi_set_baudr(priv, priv->clock_div);

	/* Operation Mode */
	ctrlr0 = OMOD_MASTER << OMOD_SHIFT;

	/* Data Frame Size */
	ctrlr0 |= spi_dfs << DFS_SHIFT;

	/* set SPI mode 0..3 */
	if (priv->mode & SPI_CPOL)
		ctrlr0 |= SCOL_HIGH << SCOL_SHIFT;
	if (priv->mode & SPI_CPHA)
		ctrlr0 |= SCPH_TOGSTA << SCPH_SHIFT;

	/* Chip Select Mode */
	ctrlr0 |= CSM_KEEP << CSM_SHIFT;

	/* SSN to Sclk_out delay */
	ctrlr0 |= SSN_DELAY_ONE << SSN_DELAY_SHIFT;

	/* Serial Endian Mode */
	ctrlr0 |= SEM_LITTLE << SEM_SHIFT;

	/* First Bit Mode */
	ctrlr0 |= FBM_MSB << FBM_SHIFT;

	/* Byte and Halfword Transform */
	ctrlr0 |= spi_tf << HALF_WORD_TX_SHIFT;

	/* Rxd Sample Delay */
	ctrlr0 |= priv->rsd << RXDSD_SHIFT;

	/* Frame Format */
	ctrlr0 |= FRF_SPI << FRF_SHIFT;

	/* Save static configuration */
	priv->cr0 = ctrlr0;

	writel(ctrlr0, &regs->ctrlr0);

	return 0;
}

static int rockchip_spi_config(struct rockchip_spi_priv *priv, const void *dout, void *din, int len)
{
	struct rockchip_spi *regs = priv->regs;
	uint ctrlr0 = priv->cr0;
	u32 tmod;

	if (dout && din)
		tmod = TMOD_TR;
	else if (dout)
		tmod = TMOD_TO;
	else
		tmod = TMOD_RO;

	ctrlr0 |= (tmod & TMOD_MASK) << TMOD_SHIFT;
	writel(ctrlr0, &regs->ctrlr0);
	if (tmod == TMOD_RO)
		writel(len, &regs->ctrlr1);

	return 0;
}

static u32 rockchip_get_fifo_len(struct rockchip_spi_priv *priv)
{
	struct rockchip_spi *regs = priv->regs;
	u32 ver;

	ver = readl(&regs->ver);

	switch (ver) {
	case ROCKCHIP_SPI_VER2_TYPE1:
	case ROCKCHIP_SPI_VER2_TYPE2:
		return 64;
	default:
		return 32;
	}
}

static u32 rockchip_get_tx_max(struct rockchip_spi_priv *priv, u32 tx_left)
{
	struct rockchip_spi *regs = priv->regs;

	return min(priv->fifo_len - readl(&regs->txflr), tx_left);
}

static u32 rockchip_get_rx_max(struct rockchip_spi_priv *priv, u32 rx_left)
{
	struct rockchip_spi *regs = priv->regs;

	return min(readl(&regs->rxflr), rx_left);
}

int rockchip_spi_probe(void *base_addr, u32 rsd, u32 clock_div, u32 mode)
{
	struct rockchip_spi_priv *priv = spi_bus;

	priv->regs = (struct rockchip_spi *)base_addr;
	priv->rsd = rsd;
	priv->mode = mode;
	if (clock_div % 2 || clock_div < 2) {
		printf("%s div should be even num, and at least 2\n", __func__);

		return -1;
	}
	priv->clock_div = clock_div;
	priv->bits_per_word = 8;
	priv->fifo_len = rockchip_get_fifo_len(priv);
	priv->fifo_rx_th = priv->fifo_len * 3 / 4;

	printf("%s %p clock_divide=%d spi_mode=%d\n", __func__, base_addr, clock_div, mode);

	sem_wait(priv->lock);
	rockchip_spi_claim_bus();
	sem_post(priv->lock);

	return 0;
}

int rockchip_spi_xfer(u8 cs, int len, const void *dout, void *din, unsigned long flags)
{
	struct rockchip_spi_priv *priv = spi_bus;
	struct rockchip_spi *regs = priv->regs;
	const u8 *out = dout;
	u8 *in = din;
	int toread, towrite;
	int ret, i, xfer_room;

	sem_wait(priv->lock);
	rockchip_spi_config(priv, dout, din, len);

	//printf("%s: dout=%p, din=%p, len=%x, flags=%lx\n", __func__, dout, din, len, flags);
	if (DEBUG_RK_SPI)
		rkspi_dump_regs(regs);

	/* Assert CS before transfer */
	if (flags & SPI_XFER_BEGIN)
		spi_cs_activate(cs);

	toread = din ? len : 0;
	towrite = dout ? len : 0;

	rkspi_enable_chip(regs, true);
	while (toread || towrite) {
		if (towrite) {
			xfer_room = rockchip_get_tx_max(priv, towrite);
			for (i = 0; i < xfer_room; i++) {
				writel(out ? *out++ : 0, regs->txdr);
			}
			towrite-=xfer_room;
		}

		if (toread) {
			u32 byte;
			
			xfer_room = rockchip_get_rx_max(priv, toread);

			/* Using fifo to improve the continuity of data transmission */
			if (xfer_room < priv->fifo_rx_th && xfer_room < toread)
				continue;

			for (i = 0; i < xfer_room; i++) {
				byte = readl(regs->rxdr);
				if (in)
					*in++ = byte;
			}
			toread-=xfer_room;
		}
	}
	ret = rkspi_wait_till_not_busy(regs);

	/* Deassert CS after transfer */
	if (flags & SPI_XFER_END)
		spi_cs_deactivate(cs);

	rkspi_enable_chip(regs, false);
	sem_post(priv->lock);

	return ret;
}

void dbg_print_hex(char *s, void *buf, u32 width, u32 len)
{
	u32 i, j;
	unsigned char *p8 = (unsigned char *)buf;
	unsigned short *p16 = (unsigned short *)buf;
	u32 *p32 = (u32 *)buf;

	j = 0;
	for (i = 0; i < len; i++) {
		if (j == 0)
			printf("%s %p + 0x%x:", s, buf, i * width);
		if (width == 4)
			printf("0x%08x,", p32[i]);
		else if (width == 2)
			printf("0x%04x,", p16[i]);
		else
			printf("0x%02x,", p8[i]);
		if (++j >= (16 / width)) {
			j = 0;
			printf("\n");
		}
	}
	printf("\n");
}

int spi_test(void)
{
	unsigned char *pread, *pwrite;
	u32 test_size = 0x100;
	int ret, i;

	/*
	 * SPI duplex
	 */
	pread = malloc(test_size);
	if (!pread)
		printf("%s pread malloc fail\n", __func__);
	pwrite = malloc(test_size);
	if (!pwrite) {
		printf("%s pwrite malloc fail\n", __func__);
		free(pwrite);
		return -1;
	}

	for (i = 0; i < test_size; i++)
		pwrite[i] = i;

	/* spi duplex test */
	ret = rockchip_spi_xfer(0, test_size, pwrite, pread, SPI_XFER_ONCE);
	if (ret) {
		printf("rockchip_spi_xfer fail ret=%d\n", ret);
		return ret;
	}
	if (memcmp(pwrite, pread, test_size)) {
		dbg_print_hex("spi_duplex w:", pwrite, 4, test_size / 4);
		dbg_print_hex("spi_duplex r:", pread, 4, test_size / 4);
		printf("%s duplex test fail, connect miso and mosi for hw test\n", __func__);
	} else {
		printf("%s duplex test success\n", __func__);
	}

	free(pread);
	free(pwrite);

	return 0;
}

static unsigned long long time_used(uint64_t start, uint64_t end)
{
	return (unsigned long long)(end - start) / 1000;
}

void spi_speed_test(unsigned long long test_size, unsigned long long loops)
{
	unsigned char *pread, *pwrite;
	int ret, i;
	unsigned long long t1, t2;

	/*
	 * SPI duplex
	 */
	pread = malloc(test_size);
	if (!pread)
		printf("%s pread malloc fail\n", __func__);
	pwrite = malloc(test_size);
	if (!pwrite) {
		printf("%s pwrite malloc fail\n", __func__);
		free(pread);
		return;
	}

	for (i = 0; i < test_size; i++)
		pwrite[i] = i;

	t1 = nano_time();
	for (i = 0; i < loops; i++) {
		ret = rockchip_spi_xfer(0, test_size, pwrite, NULL, SPI_XFER_ONCE);
		if (ret) {
			printf("rockchip_spi wr test fail ret=%d\n", ret);
			return;
		}
	}
	t2 = nano_time();
	printf("SPI: wr 0x%08xB %lluKB/s\n", test_size, test_size * loops * 1000000 / time_used(t1, t2) / 1024);
	t1 = nano_time();
	for (i = 0; i < loops; i++) {
		ret = rockchip_spi_xfer(0, test_size, NULL, pread, SPI_XFER_ONCE);
		if (ret) {
			printf("rockchip_spi rd test fail ret=%d\n", ret);
			return;
		}
	}
	t2 = nano_time();
	printf("SPI: rd 0x%08xB %lluKB/s\n", test_size, test_size * loops * 1000000 / time_used(t1, t2) / 1024);


	free(pread);
	free(pwrite);
}

int main(int argc, char *argv[])
{
	int fd, ret;
	void *spi_base;
	char *device;

	device = argv[1];

	fd = open(device, O_RDWR);

	if (fd < 0) {
		printf("open %s failed!\n", device);
		return fd;
	}

	spi_bus = malloc(sizeof(struct rockchip_spi_priv));
	if (!spi_bus) {
		printf("%s spi_bus malloc failed\n", __func__);
		return -1;
	}

	spi_base = (struct rockchip_spi *)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (spi_base == MAP_FAILED) {
		printf("mmap failed: %s\n", strerror(errno));
		free(spi_bus);
		return -errno;
	}

	spi_bus->lock = malloc(sizeof(sem_t));
	sem_init(spi_bus->lock, 0, 1);

	rockchip_spi_probe(spi_base, 0, 8, SPI_MODE_0);

	spi_speed_test(0x0020, 10000);
	spi_speed_test(0x0080, 10000);
	spi_speed_test(0x0400, 1000);
	spi_speed_test(0x2000, 1000);
	spi_speed_test(0x100000, 10);
	spi_test();

	free(spi_bus);
	close(fd);

	return 0;
}
