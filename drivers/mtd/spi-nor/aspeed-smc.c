/*
 * ASPEED Static Memory Controller driver
 *
 * Copyright (c) 2015-2016, IBM Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#define DEVICE_NAME	"aspeed-smc"

/*
 * The driver only support SPI flash
 */
enum aspeed_smc_flash_type {
	smc_type_nor  = 0,
	smc_type_nand = 1,
	smc_type_spi  = 2,
};

struct aspeed_smc_chip;

struct aspeed_smc_info {
	u32 maxsize;		/* maximum size of chip window */
	u8 nce;			/* number of chip enables */
	bool hastype;		/* flash type field exists in config reg */
	u8 we0;			/* shift for write enable bit for CE0 */
	u8 ctl0;		/* offset in regs of ctl for CE0 */
	u8 timing;		/* offset in regs of timing */
	bool has_dma;

	void (*set_4b)(struct aspeed_smc_chip *chip);
	int (*optimize_read)(struct aspeed_smc_chip *chip, u32 max_freq);
};

static void aspeed_smc_chip_set_4b_spi_2400(struct aspeed_smc_chip *chip);
static void aspeed_smc_chip_set_4b(struct aspeed_smc_chip *chip);
static int aspeed_smc_optimize_read(struct aspeed_smc_chip *chip,
				     u32 max_freq);

static const struct aspeed_smc_info fmc_2400_info = {
	.maxsize = 256 * 1024 * 1024,
	.nce = 5,
	.hastype = true,
	.we0 = 16,
	.ctl0 = 0x10,
	.timing = 0x94,
	.has_dma = true,
	.set_4b = aspeed_smc_chip_set_4b,
};

static const struct aspeed_smc_info spi_2400_info = {
	.maxsize = 64 * 1024 * 1024,
	.nce = 1,
	.hastype = false,
	.we0 = 0,
	.ctl0 = 0x04,
	.timing = 0x94,
	.has_dma = false,
	.set_4b = aspeed_smc_chip_set_4b_spi_2400,
	.optimize_read = aspeed_smc_optimize_read,
};

static const struct aspeed_smc_info fmc_2500_info = {
	.maxsize = 256 * 1024 * 1024,
	.nce = 3,
	.hastype = true,
	.we0 = 16,
	.ctl0 = 0x10,
	.timing = 0x94,
	.has_dma = true,
	.set_4b = aspeed_smc_chip_set_4b,
};

static const struct aspeed_smc_info spi_2500_info = {
	.maxsize = 128 * 1024 * 1024,
	.nce = 2,
	.hastype = false,
	.we0 = 16,
	.ctl0 = 0x10,
	.timing = 0x94,
	.has_dma = false,
	.set_4b = aspeed_smc_chip_set_4b,
	.optimize_read = aspeed_smc_optimize_read,
};

enum aspeed_smc_ctl_reg_value {
	smc_base,		/* base value without mode for other commands */
	smc_read,		/* command reg for (maybe fast) reads */
	smc_write,		/* command reg for writes */
	smc_max,
};

struct aspeed_smc_controller;

struct aspeed_smc_chip {
	int cs;
	struct aspeed_smc_controller *controller;
	void __iomem *ctl;			/* control register */
	void __iomem *ahb_base;			/* base of chip window */
	u32 ahb_window_size;			/* chip window size */
	unsigned long phys_base;		/* physical address of window */
	u32 ctl_val[smc_max];			/* control settings */
	enum aspeed_smc_flash_type type;	/* what type of flash */
	struct spi_nor nor;
};

struct aspeed_smc_controller {
	struct device *dev;

	struct mutex mutex;			/* controller access mutex */
	const struct aspeed_smc_info *info;	/* type info of controller */
	void __iomem *regs;			/* controller registers */
	void __iomem *ahb_base;			/* per-chip windows resource */

	struct clk *ahb_clk;

	/* interrupt handling */
	int irq;

	/* dma */
	bool dma_enabled;
	struct completion dma_done;

	/* dma logging */
	size_t dma_length;
	dma_addr_t dma_addr;			/* bus address of buffer */
	dma_addr_t flash_addr;			/* flash address */

	struct aspeed_smc_chip *chips[0];	/* pointers to attached chips */
};

/*
 * SPI Flash Configuration Register (AST2500 SPI)
 *     or
 * Type setting Register (AST2500 FMC).
 * CE0 and CE1 can only be of type SPI. CE2 can be of type NOR but the
 * driver does not support it.
 */
#define CONFIG_REG			0x0
#define CONFIG_DISABLE_LEGACY		BIT(31) /* 1 */

#define CONFIG_CE2_WRITE		BIT(18)
#define CONFIG_CE1_WRITE		BIT(17)
#define CONFIG_CE0_WRITE		BIT(16)

#define CONFIG_CE2_TYPE			BIT(4) /* AST2500 FMC only */
#define CONFIG_CE1_TYPE			BIT(2) /* AST2500 FMC only */
#define CONFIG_CE0_TYPE			BIT(0) /* AST2500 FMC only */

/*
 * CE Control Register
 */
#define CE_CONTROL_REG			0x4

/*
 * CEx Control Register
 */
#define CONTROL_AAF_MODE		BIT(31)
#define CONTROL_IO_MODE_MASK		GENMASK(30, 28)
#define CONTROL_IO_DUAL_DATA		BIT(29)
#define CONTROL_IO_DUAL_ADDR_DATA	(BIT(29) | BIT(28))
#define CONTROL_IO_QUAD_DATA		BIT(30)
#define CONTROL_IO_QUAD_ADDR_DATA	(BIT(30) | BIT(28))
#define CONTROL_CE_INACTIVE_SHIFT	24
#define CONTROL_CE_INACTIVE_MASK	GENMASK(27, \
					CONTROL_CE_INACTIVE_SHIFT)
/* 0 = 16T ... 15 = 1T   T=HCLK */
#define CONTROL_COMMAND_SHIFT		16
#define CONTROL_DUMMY_COMMAND_OUT	BIT(15)
#define CONTROL_IO_DUMMY_HI		BIT(14)
#define CONTROL_IO_DUMMY_HI_SHIFT	14
#define CONTROL_CLK_DIV4		BIT(13) /* others */
#define CONTROL_IO_ADDRESS_4B		BIT(13) /* AST2400 SPI */
#define CONTROL_RW_MERGE		BIT(12)
#define CONTROL_IO_DUMMY_LO_SHIFT	6
#define CONTROL_IO_DUMMY_LO		GENMASK(7, \
						CONTROL_IO_DUMMY_LO_SHIFT)
#define CONTROL_IO_DUMMY_MASK		(CONTROL_IO_DUMMY_HI | \
					 CONTROL_IO_DUMMY_LO)
#define CONTROL_IO_DUMMY_SET(dummy)				 \
	(((((dummy) >> 2) & 0x1) << CONTROL_IO_DUMMY_HI_SHIFT) | \
	 (((dummy) & 0x3) << CONTROL_IO_DUMMY_LO_SHIFT))

#define CONTROL_CLOCK_FREQ_SEL_SHIFT	8
#define CONTROL_CLOCK_FREQ_SEL_MASK	GENMASK(11, \
						CONTROL_CLOCK_FREQ_SEL_SHIFT)
#define CONTROL_LSB_FIRST		BIT(5)
#define CONTROL_CLOCK_MODE_3		BIT(4)
#define CONTROL_IN_DUAL_DATA		BIT(3)
#define CONTROL_CE_STOP_ACTIVE_CONTROL	BIT(2)
#define CONTROL_COMMAND_MODE_MASK	GENMASK(1, 0)
#define CONTROL_COMMAND_MODE_NORMAL	0
#define CONTROL_COMMAND_MODE_FREAD	1
#define CONTROL_COMMAND_MODE_WRITE	2
#define CONTROL_COMMAND_MODE_USER	3

#define CONTROL_KEEP_MASK						\
	(CONTROL_AAF_MODE | CONTROL_CE_INACTIVE_MASK | CONTROL_CLK_DIV4 | \
	 CONTROL_CLOCK_FREQ_SEL_MASK | CONTROL_LSB_FIRST | CONTROL_CLOCK_MODE_3)

/* Interrupt Control and Status Register */
#define INTERRUPT_STATUS_REG		0x08
#define     INTERRUPT_DMA_ENABLE	BIT(3)
#define     INTERRUPT_DMA_STATUS	BIT(11)

/*
 * The Segment Register uses a 8MB unit to encode the start address
 * and the end address of the mapping window of a flash SPI slave :
 *
 *        | byte 1 | byte 2 | byte 3 | byte 4 |
 *        +--------+--------+--------+--------+
 *        |  end   |  start |   0    |   0    |
 */
#define SEGMENT_ADDR_REG0		0x30
#define SEGMENT_ADDR_START(_r)		((((_r) >> 16) & 0xFF) << 23)
#define SEGMENT_ADDR_END(_r)		((((_r) >> 24) & 0xFF) << 23)
#define SEGMENT_ADDR_VALUE(start, end)					\
	(((((start) >> 23) & 0xFF) << 16) | ((((end) >> 23) & 0xFF) << 24))
#define SEGMENT_ADDR_REG(controller, cs)	\
	((controller)->regs + SEGMENT_ADDR_REG0 + (cs) * 4)

/* DMA Registers */
#define DMA_CONTROL_REG			0x80
#define     DMA_ENABLE			BIT(0)
#define     DMA_WRITE			BIT(1)

#define DMA_FLASH_BASE_REG		0x84
#define DMA_DRAM_BASE_REG		0x88
#define DMA_LENGTH_REG			0x8c

/*
 * DMAs do not seem to be that fast, so disable by default
 */
static bool use_dma;
module_param(use_dma, bool, 0644);

static unsigned int min_dma_size = 256;
module_param(min_dma_size, uint, 0644);

/* with 100ms we had a couple of timeouts */
static unsigned int dma_timeout = 200;
module_param(dma_timeout, uint, 0644);

/*
 * Switch to turn off read optimisation if needed
 */
static bool optimize_read = true;
module_param(optimize_read, bool, 0644);

static void aspeed_smc_dma_done(struct aspeed_smc_controller *controller)
{
	writel(0, controller->regs + INTERRUPT_STATUS_REG);
	writel(0, controller->regs + DMA_CONTROL_REG);
}

#define DMA_LENGTH(x) (((x) - 4) & ~0xFE000003)
#define DMA_ADDR(x) ((x) & ~0x00000003)

static inline void aspeed_smc_chip_configure(struct aspeed_smc_chip *chip,
					     u32 ctl)
{
	ctl |= CONTROL_CE_STOP_ACTIVE_CONTROL;
	writel(ctl, chip->ctl);

	ctl &= ~CONTROL_CE_STOP_ACTIVE_CONTROL;
	writel(ctl, chip->ctl);
}

static int aspeed_smc_dma_start(struct aspeed_smc_chip *chip,
				u32 offset, void *buf, size_t length,
				int is_write)
{
	struct aspeed_smc_controller *controller = chip->controller;
	dma_addr_t dma_addr, flash_addr;
	int ret = 0;

	aspeed_smc_chip_configure(chip, is_write ? chip->ctl_val[smc_write] :
		chip->ctl_val[smc_read]);

	dev_dbg(chip->nor.dev, "DMA %s to=0x%08x len=0x%08x\n",
		is_write ? "write" : "read", offset, length);

	dma_addr = dma_map_single(chip->nor.dev, buf, length,
				  (is_write ? DMA_TO_DEVICE : DMA_FROM_DEVICE));

	if (unlikely(dma_mapping_error(chip->nor.dev, dma_addr))) {
		dev_err(chip->nor.dev, "Failed to dma_map_single()\n");
		ret = -ENOMEM;
		goto out;
	}
	flash_addr = chip->phys_base + offset;

	controller->dma_length = length;
	controller->dma_addr = dma_addr;
	controller->flash_addr = flash_addr;

	reinit_completion(&controller->dma_done);

	writel(0, controller->regs + DMA_CONTROL_REG);
	writel(DMA_ADDR(flash_addr), controller->regs +
	       DMA_FLASH_BASE_REG);
	writel(DMA_ADDR(dma_addr), controller->regs + DMA_DRAM_BASE_REG);
	writel(DMA_LENGTH(length), controller->regs + DMA_LENGTH_REG);

	writel(INTERRUPT_DMA_ENABLE,
	       controller->regs + INTERRUPT_STATUS_REG);

	writel(DMA_ENABLE | (is_write << 1),
	       controller->regs + DMA_CONTROL_REG);

	if (!wait_for_completion_timeout(&controller->dma_done,
					 msecs_to_jiffies(dma_timeout))) {
		dev_err(chip->nor.dev,
			"DMA timeout addr@%.8x faddr@%.8x size=%x\n",
			controller->dma_addr,
			controller->flash_addr,
			controller->dma_length);
		ret = -ETIMEDOUT;
		aspeed_smc_dma_done(controller);
	}

	dma_unmap_single(chip->nor.dev,
			 controller->dma_addr, controller->dma_length,
			 (is_write ? DMA_TO_DEVICE : DMA_FROM_DEVICE));
out:
	aspeed_smc_chip_configure(chip, chip->ctl_val[smc_read]);
	return ret;
}

/*
 * In user mode all data bytes read or written to the chip decode address
 * range are transferred to or from the SPI bus. The range is treated as a
 * fifo of arbitratry 1, 2, or 4 byte width but each write has to be aligned
 * to its size. The address within the multiple 8kB range is ignored when
 * sending bytes to the SPI bus.
 *
 * On the arm architecture, as of Linux version 4.3, memcpy_fromio and
 * memcpy_toio on little endian targets use the optimized memcpy routines
 * that were designed for well behavied memory storage. These routines
 * have a stutter if the source and destination are not both word aligned,
 * once with a duplicate access to the source after aligning to the
 * destination to a word boundary, and again with a duplicate access to
 * the source when the final byte count is not word aligned.
 *
 * When writing or reading the fifo this stutter discards data or sends
 * too much data to the fifo and can not be used by this driver.
 *
 * While the low level io string routines that implement the insl family do
 * the desired accesses and memory increments, the cross architecture io
 * macros make them essentially impossible to use on a memory mapped address
 * instead of a a token from the call to iomap of an io port.
 *
 * These fifo routines use readl and friends to a constant io port and update
 * the memory buffer pointer and count via explicit code. The final updates
 * to len are optimistically suppressed.
 */
static int aspeed_smc_read_from_ahb(void *buf, void __iomem *src, size_t len)
{
	size_t offset = 0;

	if (IS_ALIGNED((uintptr_t)src, sizeof(uintptr_t)) &&
	    IS_ALIGNED((uintptr_t)buf, sizeof(uintptr_t))) {
		ioread32_rep(src, buf, len >> 2);
		offset = len & ~0x3;
		len -= offset;
	}
	ioread8_rep(src, (u8 *)buf + offset, len);
	return 0;
}

static int aspeed_smc_write_to_ahb(void __iomem *dst, const void *buf,
				   size_t len)
{
	size_t offset = 0;

	if (IS_ALIGNED((uintptr_t)dst, sizeof(uintptr_t)) &&
	    IS_ALIGNED((uintptr_t)buf, sizeof(uintptr_t))) {
		iowrite32_rep(dst, buf, len >> 2);
		offset = len & ~0x3;
		len -= offset;
	}
	iowrite8_rep(dst, (const u8 *)buf + offset, len);
	return 0;
}

static inline u32 aspeed_smc_chip_write_bit(struct aspeed_smc_chip *chip)
{
	return BIT(chip->controller->info->we0 + chip->cs);
}

static void aspeed_smc_chip_check_config(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32 reg;

	reg = readl(controller->regs + CONFIG_REG);

	if (reg & aspeed_smc_chip_write_bit(chip))
		return;

	dev_dbg(controller->dev, "config write is not set ! @%p: 0x%08x\n",
		controller->regs + CONFIG_REG, reg);
	reg |= aspeed_smc_chip_write_bit(chip);
	writel(reg, controller->regs + CONFIG_REG);
}

static void aspeed_smc_start_user(struct spi_nor *nor)
{
	struct aspeed_smc_chip *chip = nor->priv;
	u32 ctl = chip->ctl_val[smc_base];

	/*
	 * When the chip is controlled in user mode, we need write
	 * access to send the opcodes to it. So check the config.
	 */
	aspeed_smc_chip_check_config(chip);

	ctl |= CONTROL_COMMAND_MODE_USER |
		CONTROL_CE_STOP_ACTIVE_CONTROL;
	writel(ctl, chip->ctl);

	ctl &= ~CONTROL_CE_STOP_ACTIVE_CONTROL;
	writel(ctl, chip->ctl);
}

static void aspeed_smc_stop_user(struct spi_nor *nor)
{
	struct aspeed_smc_chip *chip = nor->priv;

	u32 ctl = chip->ctl_val[smc_read];
	u32 ctl2 = ctl | CONTROL_COMMAND_MODE_USER |
		CONTROL_CE_STOP_ACTIVE_CONTROL;

	writel(ctl2, chip->ctl);	/* stop user CE control */
	writel(ctl, chip->ctl);		/* default to fread or read mode */
}

static int aspeed_smc_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct aspeed_smc_chip *chip = nor->priv;

	mutex_lock(&chip->controller->mutex);
	return 0;
}

static void aspeed_smc_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct aspeed_smc_chip *chip = nor->priv;

	mutex_unlock(&chip->controller->mutex);
}

static int aspeed_smc_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_user(nor);
	aspeed_smc_write_to_ahb(chip->ahb_base, &opcode, 1);
	aspeed_smc_read_from_ahb(buf, chip->ahb_base, len);
	aspeed_smc_stop_user(nor);
	return 0;
}

static int aspeed_smc_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				int len)
{
	struct aspeed_smc_chip *chip = nor->priv;

	aspeed_smc_start_user(nor);
	aspeed_smc_write_to_ahb(chip->ahb_base, &opcode, 1);
	aspeed_smc_write_to_ahb(chip->ahb_base, buf, len);
	aspeed_smc_stop_user(nor);
	return 0;
}

static void aspeed_smc_send_cmd_addr(struct spi_nor *nor, u8 cmd, u32 addr)
{
	struct aspeed_smc_chip *chip = nor->priv;
	__be32 temp;
	u32 cmdaddr;

	switch (nor->addr_width) {
	default:
		WARN_ONCE(1, "Unexpected address width %u, defaulting to 3\n",
			  nor->addr_width);
		/* FALLTHROUGH */
	case 3:
		cmdaddr = addr & 0xFFFFFF;
		cmdaddr |= cmd << 24;

		temp = cpu_to_be32(cmdaddr);
		aspeed_smc_write_to_ahb(chip->ahb_base, &temp, 4);
		break;
	case 4:
		temp = cpu_to_be32(addr);
		aspeed_smc_write_to_ahb(chip->ahb_base, &cmd, 1);
		aspeed_smc_write_to_ahb(chip->ahb_base, &temp, 4);
		break;
	}
}

/*
 * Try DMA transfer when size and alignment are correct. In case
 * of failure, just restart using the IO mode.
 */
static int aspeed_smc_dma_check(struct aspeed_smc_chip *chip, loff_t off,
				size_t len)
{
	return (IS_ALIGNED(off, 4) && IS_ALIGNED(len, 4) &&
		len >= min_dma_size && chip->controller->dma_enabled &&
		use_dma);
}

static ssize_t aspeed_smc_read_user(struct spi_nor *nor, loff_t from,
				    size_t len, u_char *read_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;
	int i;
	u8 dummy = 0xFF;
	u32 ctl;

	aspeed_smc_start_user(nor);
	aspeed_smc_send_cmd_addr(nor, nor->read_opcode, from);
	for (i = 0; i < chip->nor.read_dummy / 8; i++)
		aspeed_smc_write_to_ahb(chip->ahb_base, &dummy, sizeof(dummy));

	if (chip->nor.flash_read == SPI_NOR_DUAL) {
		/* Switch to dual I/O mode for data cycle */
		ctl = readl(chip->ctl) & ~CONTROL_IO_MODE_MASK;
		ctl |= CONTROL_IO_DUAL_DATA;
		writel(ctl, chip->ctl);
	}

	aspeed_smc_read_from_ahb(read_buf, chip->ahb_base, len);
	aspeed_smc_stop_user(nor);
	return 0;
}

static ssize_t aspeed_smc_read(struct spi_nor *nor, loff_t from, size_t len,
			       u_char *read_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;
	int ret;

	/* The segment window configured for the chip is too small for
	 * the read offset. Use the "User mode" of the controller to
	 * perform the read.
	 */
	if (from >= chip->ahb_window_size) {
		aspeed_smc_read_user(nor, from, len, read_buf);
		goto out;
	}

	/* Then, try DMA if the driver allows them. */
	if (aspeed_smc_dma_check(chip, from, len)) {
		ret = aspeed_smc_dma_start(chip, from, read_buf, len, 0);
		if (!ret)
			goto out;
		dev_err(chip->nor.dev, "DMA read failed: %d", ret);
	}

	/* Last, and this should be the default, use the "Command
	 * mode" of the controller which does the read from the
	 * segment window configured for the chip on the AHB bus.
	 */
	memcpy_fromio(read_buf, chip->ahb_base + from, len);

out:
	return len;
}

static ssize_t aspeed_smc_write_user(struct spi_nor *nor, loff_t to,
				     size_t len, const u_char *write_buf)
{
	struct aspeed_smc_chip *chip = nor->priv;
	int ret;

	if (aspeed_smc_dma_check(chip, to, len)) {
		ret = aspeed_smc_dma_start(chip, to, (void *)write_buf,
					   len, 1);
		if (!ret)
			goto out;
		dev_err(chip->nor.dev, "DMA write failed: %d", ret);
	}

	aspeed_smc_start_user(nor);
	aspeed_smc_send_cmd_addr(nor, nor->program_opcode, to);
	aspeed_smc_write_to_ahb(chip->ahb_base, write_buf, len);
	aspeed_smc_stop_user(nor);
out:
	return len;
}

static int aspeed_smc_unregister(struct aspeed_smc_controller *controller)
{
	struct aspeed_smc_chip *chip;
	int n;

	for (n = 0; n < controller->info->nce; n++) {
		chip = controller->chips[n];
		if (chip)
			mtd_device_unregister(&chip->nor.mtd);
	}

	return 0;
}

static int aspeed_smc_remove(struct platform_device *dev)
{
	return aspeed_smc_unregister(platform_get_drvdata(dev));
}

static const struct of_device_id aspeed_smc_matches[] = {
	{ .compatible = "aspeed,ast2400-fmc", .data = &fmc_2400_info },
	{ .compatible = "aspeed,ast2400-spi", .data = &spi_2400_info },
	{ .compatible = "aspeed,ast2500-fmc", .data = &fmc_2500_info },
	{ .compatible = "aspeed,ast2500-spi", .data = &spi_2500_info },
	{ }
};
MODULE_DEVICE_TABLE(of, aspeed_smc_matches);

/*
 * Each chip has a mapping window defined by a segment address
 * register defining a start and an end address on the AHB bus. These
 * addresses can be configured to fit the chip size and offer a
 * contiguous memory region across chips. For the moment, we only
 * check that each chip segment is valid.
 */
static void __iomem *aspeed_smc_chip_base(struct aspeed_smc_chip *chip,
					  struct resource *res)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32 offset = 0;
	u32 reg;

	if (controller->info->nce > 1) {
		reg = readl(SEGMENT_ADDR_REG(controller, chip->cs));

		if (SEGMENT_ADDR_START(reg) >= SEGMENT_ADDR_END(reg))
			return NULL;

		offset = SEGMENT_ADDR_START(reg) - res->start;
	}

	return controller->ahb_base + offset;
}

static u32 chip_set_segment(struct aspeed_smc_chip *chip, u32 cs, u32 start,
			    u32 size)
{
	struct aspeed_smc_controller *controller = chip->controller;
	void __iomem *seg_reg;
	u32 oldval, newval, val0, start0, end;

	val0 = readl(SEGMENT_ADDR_REG(controller, 0));
	start0 = SEGMENT_ADDR_START(val0);

	seg_reg = SEGMENT_ADDR_REG(controller, cs);
	oldval = readl(seg_reg);

	/* If the chip size is not specified, use the default segment
	 * size, but take into account the possible overlap with the
	 * previous segment
	 */
	if (!size)
		size = SEGMENT_ADDR_END(oldval) - start;

	/* The segment cannot exceed the maximum window size of the
	 * controller.
	 */
	if (start + size > start0 + controller->info->maxsize) {
		size = start0 + controller->info->maxsize - start;
		dev_warn(chip->nor.dev, "CE%d window resized to %dMB",
			 cs, size >> 20);
	}

	end = start + size;
	newval = SEGMENT_ADDR_VALUE(start, end);
	writel(newval, seg_reg);

	/* Restore default value if something goes wrong. The chip
	 * might have set some bogus value and we would loose access
	 * to the chip.
	 */
	if (newval != readl(seg_reg)) {
		dev_err(chip->nor.dev, "CE%d window invalid", cs);
		writel(oldval, seg_reg);
		start = SEGMENT_ADDR_START(oldval);
		end = SEGMENT_ADDR_END(oldval);
		size = end - start;
	}

	dev_info(chip->nor.dev, "CE%d window [ 0x%.8x - 0x%.8x ] %dMB",
		 cs, start, end, size >> 20);

	return size;
}

/*
 * This is expected to be called in increasing CE order
 */
static u32 aspeed_smc_chip_set_segment(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32  val0, start0, start;
	u32 size = chip->nor.mtd.size;

	/* The AST2500 SPI controller has a bug when the CE0 chip size
	 * exceeds 120MB.
	 */
	if (chip->cs == 0 && controller->info == &spi_2500_info &&
	    size == (128 << 20)) {
		size = 120 << 20;
		dev_info(chip->nor.dev,
			 "CE%d window resized to %dMB (AST2500 HW quirk)",
			 chip->cs, size >> 20);
	}

	val0 = readl(SEGMENT_ADDR_REG(controller, 0));
	start0 = SEGMENT_ADDR_START(val0);

	/* As a start address for the current segment, use the default
	 * start address if we are handling CE0 or use the previous
	 * segment ending address
	 */
	if (chip->cs) {
		u32 prev = readl(SEGMENT_ADDR_REG(controller, chip->cs - 1));

		start = SEGMENT_ADDR_END(prev);
	} else
		start = start0;

	size = chip_set_segment(chip, chip->cs, start, size);

	/* Update chip base address on the AHB bus */
	chip->ahb_base = controller->ahb_base + (start - start0);

	if (size < chip->nor.mtd.size)
		dev_warn(chip->nor.dev,
			 "CE%d window too small for chip %dMB",
			 chip->cs, (u32) chip->nor.mtd.size >> 20);

	/* Make sure the next segment does not overlap with the
	 * current one we just configured even if there is no
	 * available chip. That could break access in Command Mode.
	 */
	if (chip->cs < controller->info->nce - 1)
		chip_set_segment(chip, chip->cs + 1, start + size, 0);

	return size;
}

static void aspeed_smc_chip_enable_write(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32 reg;

	reg = readl(controller->regs + CONFIG_REG);

	reg |= aspeed_smc_chip_write_bit(chip);
	writel(reg, controller->regs + CONFIG_REG);
}

static void aspeed_smc_chip_set_type(struct aspeed_smc_chip *chip, int type)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32 reg;

	chip->type = type;

	reg = readl(controller->regs + CONFIG_REG);
	reg &= ~(3 << (chip->cs * 2));
	reg |= chip->type << (chip->cs * 2);
	writel(reg, controller->regs + CONFIG_REG);
}

/*
 * The first chip of the AST2500 FMC flash controller is strapped by
 * hardware, or autodetected, but other chips need to be set. Enforce
 * the 4B setting for all chips.
 */
static void aspeed_smc_chip_set_4b(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;
	u32 reg;

	reg = readl(controller->regs + CE_CONTROL_REG);
	reg |= 1 << chip->cs;
	writel(reg, controller->regs + CE_CONTROL_REG);
}

/*
 * The AST2400 SPI flash controller does not have a CE Control
 * register. It uses the CE0 control register to set 4Byte mode at the
 * controller level.
 */
static void aspeed_smc_chip_set_4b_spi_2400(struct aspeed_smc_chip *chip)
{
	chip->ctl_val[smc_base] |= CONTROL_IO_ADDRESS_4B;
	chip->ctl_val[smc_read] |= CONTROL_IO_ADDRESS_4B;
}

static int aspeed_smc_chip_setup_init(struct aspeed_smc_chip *chip,
				      struct resource *res)
{
	struct aspeed_smc_controller *controller = chip->controller;
	const struct aspeed_smc_info *info = controller->info;
	u32 reg, base_reg;

	/*
	 * Always turn on the write enable bit to allow opcodes to be
	 * sent in user mode.
	 */
	aspeed_smc_chip_enable_write(chip);

	/* The driver only supports SPI type flash */
	if (info->hastype)
		aspeed_smc_chip_set_type(chip, smc_type_spi);

	/*
	 * Configure chip base address in memory
	 */
	chip->ahb_base = aspeed_smc_chip_base(chip, res);
	if (!chip->ahb_base) {
		dev_warn(chip->nor.dev, "CE%d window closed", chip->cs);
		return -EINVAL;
	}

	chip->phys_base = res->start + (chip->ahb_base - controller->ahb_base);

	/*
	 * Get value of the inherited control register. U-Boot usually
	 * does some timing calibration on the FMC chip, so it's good
	 * to keep them. In the future, we should handle calibration
	 * from Linux.
	 */
	reg = readl(chip->ctl);
	dev_dbg(controller->dev, "control register: %08x\n", reg);

	base_reg = reg & CONTROL_KEEP_MASK;
	if (base_reg != reg) {
		dev_dbg(controller->dev,
			"control register changed to: %08x\n",
			base_reg);
	}
	chip->ctl_val[smc_base] = base_reg;

	/*
	 * Retain the prior value of the control register as the
	 * default if it was normal access mode. Otherwise start with
	 * the sanitized base value set to read mode.
	 */
	if ((reg & CONTROL_COMMAND_MODE_MASK) ==
	    CONTROL_COMMAND_MODE_NORMAL)
		chip->ctl_val[smc_read] = reg;
	else
		chip->ctl_val[smc_read] = chip->ctl_val[smc_base] |
			CONTROL_COMMAND_MODE_NORMAL;

	dev_dbg(controller->dev, "default control register: %08x\n",
		chip->ctl_val[smc_read]);
	return 0;
}


#define CALIBRATE_BUF_SIZE 16384

static bool aspeed_smc_check_reads(struct aspeed_smc_chip *chip,
				  const u8 *golden_buf, u8 *test_buf)
{
	int i;

	for (i = 0; i < 10; i++) {
		aspeed_smc_read_from_ahb(test_buf, chip->ahb_base,
					 CALIBRATE_BUF_SIZE);
		if (memcmp(test_buf, golden_buf, CALIBRATE_BUF_SIZE) != 0)
			return false;
	}
	return true;
}

static int aspeed_smc_calibrate_reads(struct aspeed_smc_chip *chip, u32 hdiv,
				      const u8 *golden_buf, u8 *test_buf)
{
	struct aspeed_smc_controller *controller = chip->controller;
	const struct aspeed_smc_info *info = controller->info;
	int i;
	int good_pass = -1, pass_count = 0;
	u32 shift = (hdiv - 1) << 2;
	u32 mask = ~(0xfu << shift);
	u32 fread_timing_val = 0;

#define FREAD_TPASS(i)	(((i) / 2) | (((i) & 1) ? 0 : 8))

	/* Try HCLK delay 0..5, each one with/without delay and look for a
	 * good pair.
	 */
	for (i = 0; i < 12; i++) {
		bool pass;

		fread_timing_val &= mask;
		fread_timing_val |= FREAD_TPASS(i) << shift;

		writel(fread_timing_val, controller->regs + info->timing);
		pass = aspeed_smc_check_reads(chip, golden_buf, test_buf);
		dev_dbg(chip->nor.dev,
			"  * [%08x] %d HCLK delay, %dns DI delay : %s",
			fread_timing_val, i/2, (i & 1) ? 0 : 4,
			pass ? "PASS" : "FAIL");
		if (pass) {
			pass_count++;
			if (pass_count == 3) {
				good_pass = i - 1;
				break;
			}
		} else
			pass_count = 0;
	}

	/* No good setting for this frequency */
	if (good_pass < 0)
		return -1;

	/* We have at least one pass of margin, let's use first pass */
	fread_timing_val &= mask;
	fread_timing_val |= FREAD_TPASS(good_pass) << shift;
	writel(fread_timing_val, controller->regs + info->timing);
	dev_dbg(chip->nor.dev, " * -> good is pass %d [0x%08x]",
		good_pass, fread_timing_val);
	return 0;
}

static bool aspeed_smc_check_calib_data(const u8 *test_buf, u32 size)
{
	const u32 *tb32 = (const u32 *) test_buf;
	u32 i, cnt = 0;

	/* We check if we have enough words that are neither all 0
	 * nor all 1's so the calibration can be considered valid.
	 *
	 * I use an arbitrary threshold for now of 64
	 */
	size >>= 2;
	for (i = 0; i < size; i++) {
		if (tb32[i] != 0 && tb32[i] != 0xffffffff)
			cnt++;
	}
	return cnt >= 64;
}

static const uint32_t aspeed_smc_hclk_divs[] = {
	0xf, /* HCLK */
	0x7, /* HCLK/2 */
	0xe, /* HCLK/3 */
	0x6, /* HCLK/4 */
	0xd, /* HCLK/5 */
};
#define ASPEED_SMC_HCLK_DIV(i) (aspeed_smc_hclk_divs[(i) - 1] << 8)

static int aspeed_smc_optimize_read(struct aspeed_smc_chip *chip,
				     u32 max_freq)
{
	u8 *golden_buf, *test_buf;
	int i, rc, best_div = -1;
	u32 save_read_val = chip->ctl_val[smc_read];
	u32 ahb_freq = clk_get_rate(chip->controller->ahb_clk);

	dev_dbg(chip->nor.dev, "AHB frequency: %d MHz", ahb_freq / 1000000);

	test_buf = kmalloc(CALIBRATE_BUF_SIZE * 2, GFP_KERNEL);
	golden_buf = test_buf + CALIBRATE_BUF_SIZE;

	/* We start with the dumbest setting (keep 4Byte bit) and read
	 * some data
	 */
	chip->ctl_val[smc_read] = (chip->ctl_val[smc_read] & 0x2000) |
		(0x00 << 28) | /* Single bit */
		(0x00 << 24) | /* CE# max */
		(0x03 << 16) | /* use normal reads */
		(0x00 <<  8) | /* HCLK/16 */
		(0x00 <<  6) | /* no dummy cycle */
		(0x00);        /* normal read */

	writel(chip->ctl_val[smc_read], chip->ctl);

	aspeed_smc_read_from_ahb(golden_buf, chip->ahb_base,
				 CALIBRATE_BUF_SIZE);

	/* Establish our read mode with freq field set to 0 (HCLK/16) */
	chip->ctl_val[smc_read] = save_read_val & 0xfffff0ff;

	/* Check if calibration data is suitable */
	if (!aspeed_smc_check_calib_data(golden_buf, CALIBRATE_BUF_SIZE)) {
		dev_info(chip->nor.dev,
			 "Calibration area too uniform, using low speed");
		writel(chip->ctl_val[smc_read], chip->ctl);
		kfree(test_buf);
		return 0;
	}

	/* Now we iterate the HCLK dividers until we find our breaking point */
	for (i = ARRAY_SIZE(aspeed_smc_hclk_divs); i > 0; i--) {
		u32 tv, freq;

		/* Compare timing to max */
		freq = ahb_freq / i;
		if (freq >= max_freq)
			continue;

		/* Set the timing */
		tv = chip->ctl_val[smc_read] | ASPEED_SMC_HCLK_DIV(i);
		writel(tv, chip->ctl);
		dev_dbg(chip->nor.dev, "Trying HCLK/%d...", i);
		rc = aspeed_smc_calibrate_reads(chip, i, golden_buf, test_buf);
		if (rc == 0)
			best_div = i;
	}
	kfree(test_buf);

	/* Nothing found ? */
	if (best_div < 0)
		dev_warn(chip->nor.dev, "No good frequency, using dumb slow");
	else {
		dev_dbg(chip->nor.dev, "Found good read timings at HCLK/%d",
			best_div);
		chip->ctl_val[smc_read] |= ASPEED_SMC_HCLK_DIV(best_div);
	}

	writel(chip->ctl_val[smc_read], chip->ctl);
	return 0;
}

static int aspeed_smc_chip_setup_finish(struct aspeed_smc_chip *chip)
{
	struct aspeed_smc_controller *controller = chip->controller;
	const struct aspeed_smc_info *info = controller->info;
	u32 cmd;

	if (chip->nor.addr_width == 4 && info->set_4b)
		info->set_4b(chip);

	/* This is for direct AHB access when using Command Mode. For
	 * the AST2400 SPI controller which only handles one chip,
	 * let's use the full window.
	 */
	if (controller->info->nce == 1)
		chip->ahb_window_size = info->maxsize;
	else
		chip->ahb_window_size = aspeed_smc_chip_set_segment(chip);

	/*
	 * base mode has not been optimized yet. use it for writes.
	 */
	chip->ctl_val[smc_write] = chip->ctl_val[smc_base] |
		chip->nor.program_opcode << CONTROL_COMMAND_SHIFT |
		CONTROL_COMMAND_MODE_WRITE;

	dev_dbg(controller->dev, "write control register: %08x\n",
		chip->ctl_val[smc_write]);

	/*
	 * TODO: Adjust clocks if fast read is supported and interpret
	 * SPI-NOR flags to adjust controller settings.
	 */
	switch (chip->nor.flash_read) {
	case SPI_NOR_NORMAL:
		cmd = CONTROL_COMMAND_MODE_NORMAL;
		break;
	case SPI_NOR_FAST:
		cmd = CONTROL_COMMAND_MODE_FREAD;
		break;
	case SPI_NOR_DUAL:
		cmd = CONTROL_COMMAND_MODE_FREAD | CONTROL_IO_DUAL_DATA;
		break;
	default:
		dev_err(chip->nor.dev, "unsupported SPI read mode\n");
		return -EINVAL;
	}

	chip->ctl_val[smc_read] |= cmd |
		chip->nor.read_opcode << CONTROL_COMMAND_SHIFT |
		CONTROL_IO_DUMMY_SET(chip->nor.read_dummy / 8);

	dev_dbg(controller->dev, "read control register: %08x\n",
		chip->ctl_val[smc_read]);

	/*
	 * TODO: get max freq from chip
	 */
	if (optimize_read && info->optimize_read)
		info->optimize_read(chip, 104000000);
	return 0;
}

static int aspeed_smc_setup_flash(struct aspeed_smc_controller *controller,
				  struct device_node *np, struct resource *r)
{
	const struct aspeed_smc_info *info = controller->info;
	struct device *dev = controller->dev;
	struct device_node *child;
	unsigned int cs;
	int ret = -ENODEV;

	for_each_available_child_of_node(np, child) {
		struct aspeed_smc_chip *chip;
		struct spi_nor *nor;
		struct mtd_info *mtd;

		/* This driver does not support NAND or NOR flash devices. */
		if (!of_device_is_compatible(child, "jedec,spi-nor"))
			continue;

		ret = of_property_read_u32(child, "reg", &cs);
		if (ret) {
			dev_err(dev, "Couldn't not read chip select.\n");
			break;
		}

		if (cs >= info->nce) {
			dev_err(dev, "Chip select %d out of range.\n",
				cs);
			ret = -ERANGE;
			break;
		}

		if (controller->chips[cs]) {
			dev_err(dev, "Chip select %d already in use by %s\n",
				cs, dev_name(controller->chips[cs]->nor.dev));
			ret = -EBUSY;
			break;
		}

		chip = devm_kzalloc(controller->dev, sizeof(*chip), GFP_KERNEL);
		if (!chip) {
			ret = -ENOMEM;
			break;
		}

		chip->controller = controller;
		chip->ctl = controller->regs + info->ctl0 + cs * 4;
		chip->cs = cs;

		nor = &chip->nor;
		mtd = &nor->mtd;

		nor->dev = dev;
		nor->priv = chip;
		spi_nor_set_flash_node(nor, child);
		nor->read = aspeed_smc_read;
		nor->write = aspeed_smc_write_user;
		nor->read_reg = aspeed_smc_read_reg;
		nor->write_reg = aspeed_smc_write_reg;
		nor->prepare = aspeed_smc_prep;
		nor->unprepare = aspeed_smc_unprep;

		ret = aspeed_smc_chip_setup_init(chip, r);
		if (ret)
			break;

		ret = spi_nor_scan(nor, NULL, SPI_NOR_DUAL);
		if (ret)
			break;

		ret = aspeed_smc_chip_setup_finish(chip);
		if (ret)
			break;

		ret = mtd_device_register(mtd, NULL, 0);
		if (ret)
			break;

		controller->chips[cs] = chip;
	}

	if (ret)
		aspeed_smc_unregister(controller);

	return ret;
}

static irqreturn_t aspeed_smc_irq(int irq, void *arg)
{
	struct aspeed_smc_controller *controller = arg;
	struct device *dev = controller->dev;
	irqreturn_t ret = IRQ_NONE;
	u32 dma_ctl = readl(controller->regs + DMA_CONTROL_REG);
	u32 status = readl(controller->regs + INTERRUPT_STATUS_REG);

	dev_dbg(dev, "received IRQ. status: %x", status);

	if (!(status & INTERRUPT_DMA_ENABLE) || !(dma_ctl & DMA_ENABLE)) {
		dev_err(dev, "No DMA. bad IRQ status: %x", status);
		goto out;
	}

	if (!(status & INTERRUPT_DMA_STATUS)) {
		dev_err(dev, "DMA still in progress. length %d\n",
			readl(controller->regs + DMA_LENGTH_REG));
		goto out;
	}

	ret = IRQ_HANDLED;
	aspeed_smc_dma_done(controller);
	complete(&controller->dma_done);

out:
	return ret;
}

static int aspeed_smc_config_irq(struct aspeed_smc_controller *controller,
				 struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc;

	controller->irq = platform_get_irq(pdev, 0);
	if (!controller->irq)
		return -ENODEV;

	rc = devm_request_irq(dev, controller->irq, aspeed_smc_irq, IRQF_SHARED,
			      DEVICE_NAME, controller);
	if (rc < 0) {
		dev_warn(dev, "Unable to request IRQ %d\n", controller->irq);
		controller->irq = 0;
		return rc;
	}

	dev_info(dev, "Using IRQ %d\n", controller->irq);
	return 0;
}

static void aspeed_smc_dma_setup(struct aspeed_smc_controller *controller,
				 struct platform_device *pdev)
{
	const struct aspeed_smc_info *info = controller->info;

	init_completion(&controller->dma_done);

	controller->dma_enabled = false;
	if (info->has_dma)
		controller->dma_enabled = !aspeed_smc_config_irq(controller,
								 pdev);

	if (controller->dma_enabled)
		dev_info(controller->dev, "DMA support %s.\n",
			 use_dma ? "enabled" : "disabled");
}

static int aspeed_smc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct aspeed_smc_controller *controller;
	const struct of_device_id *match;
	const struct aspeed_smc_info *info;
	struct resource *res;
	int ret;

	match = of_match_device(aspeed_smc_matches, &pdev->dev);
	if (!match || !match->data)
		return -ENODEV;
	info = match->data;

	controller = devm_kzalloc(&pdev->dev, sizeof(*controller) +
		info->nce * sizeof(controller->chips[0]), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;
	controller->info = info;
	controller->dev = dev;

	mutex_init(&controller->mutex);
	platform_set_drvdata(pdev, controller);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	controller->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(controller->regs))
		return PTR_ERR(controller->regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	controller->ahb_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(controller->ahb_base))
		return PTR_ERR(controller->ahb_base);

	controller->ahb_clk = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(controller->ahb_clk))
		return PTR_ERR(controller->ahb_clk);

	aspeed_smc_dma_setup(controller, pdev);

	ret = aspeed_smc_setup_flash(controller, np, res);
	if (ret)
		dev_err(dev, "Aspeed SMC probe failed %d\n", ret);

	return ret;
}

static struct platform_driver aspeed_smc_driver = {
	.probe = aspeed_smc_probe,
	.remove = aspeed_smc_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = aspeed_smc_matches,
	}
};

module_platform_driver(aspeed_smc_driver);

MODULE_DESCRIPTION("ASPEED Static Memory Controller Driver");
MODULE_AUTHOR("Cedric Le Goater <clg@kaod.org>");
MODULE_LICENSE("GPL v2");
