/**
 * drivers/net/phy/ksz8463.c
 *
 * Copyright (c) 2010-2012 Micrel, Inc.
 *
 * Copyright 2009 Simtec Electronics
 *	http://www.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if 0
#define DEBUG
#define DBG
#endif

#ifndef CONFIG_MICREL_SWITCH_EMBEDDED
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#endif
#include <linux/spi/spi.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "../ks846xReg.h"
#include "../ks886xReg.h"
#include "ksz846x.h"

#ifndef DEVINIT
#define DEVINIT __devinit
#endif

#ifdef CONFIG_KSZ_STP
#include <../net/bridge/br_private.h>

#ifndef br_port_exists
#define br_port_get_rcu(dev)		(dev->br_port)
#endif

static u8 get_port_state(struct net_device *dev, struct net_device **br_dev);
#endif
#ifndef br_port_exists
#define br_port_exists(dev)		(dev->priv_flags & IFF_BRIDGE_PORT)
#endif


#if 1
#define USE_LEVEL_INTR
#endif


#define MAX_MULTICAST_LIST		32

#define MAC_ADDR_LEN			6
#define MAC_ADDR_ORDER(i)		(MAC_ADDR_LEN - 1 - (i))

/* -------------------------------------------------------------------------- */

#ifndef CONFIG_MICREL_SWITCH_EMBEDDED
enum {
	PHY_NO_FLOW_CTRL,
	PHY_FLOW_CTRL,
	PHY_TX_ONLY,
	PHY_RX_ONLY
};

enum {
	media_connected,
	media_disconnected
};

/* -------------------------------------------------------------------------- */

/**
 * struct ksz_timer_info - Timer information data structure
 * @timer:	Kernel timer.
 * @cnt:	Running timer counter.
 * @max:	Number of times to run timer; -1 for infinity.
 * @period:	Timer period in jiffies.
 */
struct ksz_timer_info {
	struct timer_list timer;
	int cnt;
	int max;
	int period;
};

/**
 * struct ksz_counter_info - OS dependent counter information data structure
 * @counter:	Wait queue to wakeup after counters are read.
 * @time:	Next time in jiffies to read counter.
 * @read:	Indication of counters read in full or not.
 */
struct ksz_counter_info {
	wait_queue_head_t counter;
	unsigned long time;
	int read;
};

#define DEV_NAME_SIZE			20

struct ksz_dev_attr {
	struct device_attribute dev_attr;
	char dev_name[DEV_NAME_SIZE];
};
#endif

/* -------------------------------------------------------------------------- */

#include "../ksz_sw.h"

#define SW_R8(sw, reg)		spi_rdreg8(sw->dev, reg)
#define SW_W8(sw, reg, val)	spi_wrreg8(sw->dev, reg, val)
#define SW_R16(sw, reg)		spi_rdreg16(sw->dev, reg)
#define SW_W16(sw, reg, val)	spi_wrreg16(sw->dev, reg, val)
#define SW_R32(sw, reg)		spi_rdreg32(sw->dev, reg)
#define SW_W32(sw, reg, val)	spi_wrreg32(sw->dev, reg, val)
#define SW_LOCK(sw)				\
	do {					\
		mutex_lock(sw->hwlock);		\
		mutex_lock(sw->reglock); \
	} while (0)
#define SW_UNLOCK(sw)				\
	do {					\
		mutex_unlock(sw->reglock);  \
		mutex_unlock(sw->hwlock);	\
	} while (0)

#define HW_R(ks, reg)		spi_rdreg16(ks, reg)
#define HW_W(ks, reg, val)	spi_wrreg16(ks, reg, val)

#include "../ksz_sw_phy.h"

#include "../ksz_spi_net.h"

/* -------------------------------------------------------------------------- */

/* shift for byte-enable data */
#define BYTE_EN(_x)			((_x) << 2)

/* turn register number and byte-enable mask into data for start of packet */
#define MK_OP(_byteen, _reg)		\
	((_reg >> 2) << (8+6) | BYTE_EN(_byteen) << 8 | (_reg) >> 4)

#define MK_BYTE(reg)			(1 << ((reg) & 3))
#define MK_WORD(reg)			((reg) & 2 ? 0xC : 0x3)
#define MK_LONG(reg)			(0xf)

#define MK_ADDR(bank, reg)		(((bank) << 9) | (reg))

/* SPI frame opcodes */
#define KS_SPIOP_RD			(0 << 7)
#define KS_SPIOP_WR			(1 << 7)

/*
 * SPI register read/write calls.
 *
 * All these calls issue SPI transactions to access the chip's registers. They
 * all require that the necessary lock is held to prevent accesses when the
 * chip is busy transfering packet data (RX/TX FIFO accesses).
 */

/**
 * spi_wrreg - issue write register command
 * @ks:		The device structure.
 * @op:		The register address and byte enables in message format.
 * @val:	The value to write.
 * @rxl:	The length of data.
 *
 * This is the low level write call that issues the necessary spi message(s)
 * to write data to the register specified in @op.
 */
static void spi_wrreg(struct spi_priv *ks, unsigned op, unsigned val,
	unsigned txl)
{
	struct spi_transfer *xfer = &ks->spi_xfer1;
	struct spi_message *msg = &ks->spi_msg1;
	struct spi_device *spi = ks->spidev;
	__le16 txb[4];
	int ret;

	if (!mutex_is_locked(&ks->lock))
		pr_alert("W not locked in %i\n", current->pid);
	txb[0] = cpu_to_le16(op | KS_SPIOP_WR);
	txb[1] = cpu_to_le16(val);
	txb[2] = cpu_to_le16(val >> 16);

	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = txl + 2;

	ret = spi_sync(spi, msg);
	if (ret < 0)
		pr_alert("spi_sync() failed\n");
}

/**
 * spi_wrreg32 - write 32bit register value to chip
 * @ks:		The device structure.
 * @reg:	The register address.
 * @val:	The value to write.
 *
 * Issue a write to put the value @val into the register specified in @reg.
 */
static void spi_wrreg32(struct spi_priv *ks, unsigned reg, unsigned val)
{
	spi_wrreg(ks, MK_OP(MK_LONG(reg), reg), val, 4);
}

/**
 * spi_wrreg16 - write 16bit register value to chip
 * @ks:		The device structure.
 * @reg:	The register address.
 * @val:	The value to write.
 *
 * Issue a write to put the value @val into the register specified in @reg.
 */
static void spi_wrreg16(struct spi_priv *ks, unsigned reg, unsigned val)
{
	spi_wrreg(ks, MK_OP(MK_WORD(reg), reg), val, 2);
}

/**
 * spi_wrreg8 - write 8bit register value to chip
 * @ks:		The device structure.
 * @reg:	The register address.
 * @val:	The value to write.
 *
 * Issue a write to put the value @val into the register specified in @reg.
 */
static void spi_wrreg8(struct spi_priv *ks, unsigned reg, unsigned val)
{
	spi_wrreg(ks, MK_OP(MK_BYTE(reg), reg), val, 1);
}

/**
 * ksz_rx_1msg - select whether to use one or two messages for spi read
 * @ks:		The device structure.
 *
 * Return whether to generate a single message with a tx and rx buffer
 * supplied to spi_sync(), or alternatively send the tx and rx buffers
 * as separate messages.
 *
 * Depending on the hardware in use, a single message may be more efficient
 * on interrupts or work done by the driver.
 *
 * This currently always returns true until we add some per-device data passed
 * from the platform code to specify which mode is better.
 */
static inline bool ksz_rx_1msg(struct spi_priv *ks)
{
	return false;
}

/**
 * spi_rdreg - issue read register command and return the data
 * @ks:		The device structure.
 * @op:		The register address and byte enables in message format.
 * @rxb:	The RX buffer to return the result into.
 * @rxl:	The length of data expected.
 *
 * This is the low level read call that issues the necessary spi message(s)
 * to read data from the register specified in @op.
 */
static void spi_rdreg(struct spi_priv *ks, unsigned op, u8 *rxb, unsigned rxl)
{
	struct spi_transfer *xfer;
	struct spi_message *msg;
	struct spi_device *spi = ks->spidev;
	__le16 *txb = (__le16 *) ks->txd;
	u8 *trx = ks->rxd;
	int ret;

	if (!mutex_is_locked(&ks->lock))
		pr_alert("R not locked in %i\n", current->pid);
	txb[0] = cpu_to_le16(op | KS_SPIOP_RD);

	if (ksz_rx_1msg(ks)) {
		msg = &ks->spi_msg1;
		xfer = &ks->spi_xfer1;

		xfer->tx_buf = txb;
		xfer->rx_buf = trx;
		xfer->len = rxl + 2;
	} else {
		msg = &ks->spi_msg2;
		xfer = ks->spi_xfer2;

		xfer->tx_buf = txb;
		xfer->rx_buf = NULL;
		xfer->len = 2;

#if 0 /* ICH: Don't think we need this - seems to be specific, to FTDI SPI
              controller. I've already taken out some PEGASUS specific stuff here.
              Have to see whether the Designware driver needs cs_change set during
              integration.*/
		/* Set private information for KSZ8692 SPI master controller. */
		if (spi->master->bus_num != 0) {
			/* for FTDI SPI */
			xfer->cs_change = 1;
		}
#endif

		xfer++;
		xfer->tx_buf = NULL;
		xfer->rx_buf = trx;
		xfer->len = rxl;
	}

	ret = spi_sync(spi, msg);
	if (ret < 0)
		pr_alert("read: spi_sync() failed\n");
	else if (ksz_rx_1msg(ks))
		memcpy(rxb, trx + 2, rxl);
	else
		memcpy(rxb, trx, rxl);
}

/**
 * spi_rdreg8 - read 8 bit register from device
 * @ks:		The device structure.
 * @reg:	The register address.
 *
 * Read a 8bit register from the chip, returning the result.
 */
static u8 spi_rdreg8(struct spi_priv *ks, unsigned reg)
{
	u8 rxb[1];

	spi_rdreg(ks, MK_OP(MK_BYTE(reg), reg), rxb, 1);
	return rxb[0];
}

/**
 * spi_rdreg16 - read 16 bit register from device
 * @ks:		The device structure.
 * @reg:	The register address.
 *
 * Read a 16bit register from the chip, returning the result.
 */
static u16 spi_rdreg16(struct spi_priv *ks, unsigned reg)
{
	__le16 rx = 0;

	spi_rdreg(ks, MK_OP(MK_WORD(reg), reg), (u8 *) &rx, 2);
	return le16_to_cpu(rx);
}

/**
 * spi_rdreg32 - read 32 bit register from device
 * @ks:		The device structure.
 * @reg:	The register address.
 *
 * Read a 32bit register from the chip.
 *
 * Note, this read requires the address be aligned to 4 bytes.
 */
static u32 spi_rdreg32(struct spi_priv *ks, unsigned reg)
{
	__le32 rx = 0;

	WARN_ON(reg & 3);

	spi_rdreg(ks, MK_OP(MK_LONG(reg), reg), (u8 *) &rx, 4);
	return le32_to_cpu(rx);
}

/* -------------------------------------------------------------------------- */

#ifndef CONFIG_MICREL_SWITCH_EMBEDDED
/**
 * delay_micro - delay in microsecond
 * @microsec:	Number of microseconds to delay.
 *
 * This routine delays in microseconds.
 */
static inline void delay_micro(uint microsec)
{
	uint millisec = microsec / 1000;

	microsec %= 1000;
	if (millisec)
		mdelay(millisec);
	if (microsec)
		udelay(microsec);
}

/**
 * delay_milli - delay in millisecond
 * @millisec:	Number of milliseconds to delay.
 *
 * This routine delays in milliseconds.
 */
static void delay_milli(uint millisec)
{
	unsigned long ticks = millisec * HZ / 1000;

	if (!ticks || in_interrupt())
		mdelay(millisec);
	else {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(ticks);
	}
}

/**
 * ksz_start_timer - start kernel timer
 * @info:	Kernel timer information.
 * @time:	The time tick.
 *
 * This routine starts the kernel timer after the specified time tick.
 */
static void ksz_start_timer(struct ksz_timer_info *info, int time)
{
	info->cnt = 0;
	info->timer.expires = jiffies + time;
	add_timer(&info->timer);

	/* infinity */
	info->max = -1;
}

/**
 * ksz_stop_timer - stop kernel timer
 * @info:	Kernel timer information.
 *
 * This routine stops the kernel timer.
 */
static void ksz_stop_timer(struct ksz_timer_info *info)
{
	if (info->max) {
		info->max = 0;
		del_timer_sync(&info->timer);
	}
}

static void ksz_init_timer(struct ksz_timer_info *info, int period,
	void (*function)(unsigned long), void *data)
{
	info->max = 0;
	info->period = period;
	init_timer(&info->timer);
	info->timer.function = function;
	info->timer.data = (unsigned long) data;
}

static void ksz_update_timer(struct ksz_timer_info *info)
{
	++info->cnt;
	if (info->max > 0) {
		if (info->cnt < info->max) {
			info->timer.expires = jiffies + info->period;
			add_timer(&info->timer);
		} else
			info->max = 0;
	} else if (info->max < 0) {
		info->timer.expires = jiffies + info->period;
		add_timer(&info->timer);
	}
}

static int get_num_val(const char *buf)
{
	int num = -1;

	if ('0' == buf[0] && 'x' == buf[1])
		sscanf(&buf[2], "%x", (unsigned int *) &num);
	else if ('0' == buf[0] && 'b' == buf[1]) {
		int i = 2;

		num = 0;
		while (buf[i]) {
			num <<= 1;
			num |= buf[i] - '0';
			i++;
		}
	} else if ('0' == buf[0] && 'd' == buf[1])
		sscanf(&buf[2], "%u", &num);
	else
		sscanf(buf, "%d", &num);
	return num;
}  /* get_num_val */
#endif

/* -------------------------------------------------------------------------- */

static inline void sw_acquire(struct ksz_sw *sw, struct mutex *hwlock)
{
	mutex_lock(&sw->lock);
	if (hwlock) {
		mutex_lock(hwlock);
		sw->locked = true;
	}
}

static inline void sw_release(struct ksz_sw *sw, struct mutex *hwlock)
{
	if (hwlock) {
		sw->locked = false;
		mutex_unlock(hwlock);
	}
	mutex_unlock(&sw->lock);
}

static int exit_mib_read(struct ksz_sw *sw)
{
	struct spi_priv *hw_priv = container_of(sw, struct spi_priv, sw);
	struct phy_priv *priv = hw_priv->phydev->priv;

	if (priv->ptp_irq || priv->ptp_using)
		return true;
	return false;
}  /* exit_mib_read */

#include "../ksz_sw.c"

#ifdef CONFIG_KSZ_STP
static u8 get_port_state(struct net_device *dev, struct net_device **br_dev)
{
	struct net_bridge_port *p;
	u8 state;

	/* This state is not defined in kernel. */
	state = STP_STATE_SIMPLE;
	if (br_port_exists(dev)) {
		p = br_port_get_rcu(dev);
		state = p->state;

		/* Port is under bridge. */
		*br_dev = p->br->dev;
	}
	return state;
}  /* get_port_state */
#endif

/* -------------------------------------------------------------------------- */

/* debugfs code */
static int state_show(struct seq_file *seq, void *v)
{
	int i;
	int j;
	SW_D data[16 / SW_SIZE];
	struct spi_priv *ks = seq->private;

	for (i = 0; i < 0x100; i += 16) {
		seq_printf(seq, SW_SIZE_STR":\t", i);
		mutex_lock(&ks->lock);
		for (j = 0; j < 16 / SW_SIZE; j++)
			data[j] = HW_R(ks, i + j * SW_SIZE);
		mutex_unlock(&ks->lock);
		for (j = 0; j < 16 / SW_SIZE; j++)
			seq_printf(seq, SW_SIZE_STR" ", data[j]);
		seq_printf(seq, "\n");
	}
	return 0;
}

static int state_open(struct inode *inode, struct file *file)
{
	return single_open(file, state_show, inode->i_private);
}

static const struct file_operations state_fops = {
	.owner	= THIS_MODULE,
	.open	= state_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release = single_release,
};

/**
 * spi_create_debugfs - create debugfs directory and files
 * @ks:		The device structure.
 *
 * Create the debugfs entries for the specific device.
 */
static void DEVINIT spi_create_debugfs(struct spi_priv *ks)
{
	struct dentry *root;
	char root_name[32];

	snprintf(root_name, sizeof(root_name), "spi_%s",
		 dev_name(&ks->spidev->dev));

	root = debugfs_create_dir(root_name, NULL);
	if (IS_ERR(root)) {
		pr_err("cannot create debugfs root\n");
		return;
	}

	ks->debug_root = root;
	ks->debug_file = debugfs_create_file("state", 0444, root,
		ks, &state_fops);
	if (IS_ERR(ks->debug_file))
		pr_err("cannot create debugfs state file\n");
}

static void __devexit spi_delete_debugfs(struct spi_priv *ks)
{
	debugfs_remove(ks->debug_file);
	debugfs_remove(ks->debug_root);
}

/* -------------------------------------------------------------------------- */

#define USE_SPEED_LINK
#define USE_MIB
#include "../ksz_sw_sysfs.c"

static irqreturn_t spi_interrupt(int irq, void *phy_dat)
{
	struct phy_device *phydev = phy_dat;
	struct phy_priv *priv = phydev->priv;

#ifdef USE_LEVEL_INTR
	disable_irq_nosync(irq);
#endif
	atomic_inc(&phydev->irq_disable);
	priv->ptp_irq = true;
	schedule_work(&phydev->phy_queue);

	return IRQ_HANDLED;
}  /* spi_interrupt */

static void spi_change(struct work_struct *work)
{
	struct phy_device *phydev =
		container_of(work, struct phy_device, phy_queue);
	struct spi_priv *ks = phydev->bus->priv;
	struct phy_priv *priv = phydev->priv;
	struct phy_device *dev = ks->phydev;
	SW_D status;

	ks->intr_working = true;
	mutex_lock(&ks->hwlock);
	mutex_lock(&ks->lock);
	status = HW_R(ks, REG_INT_STATUS);
	status &= ks->intr_mask;
	if (status & INT_PHY) {
		HW_W(ks, REG_INT_STATUS, INT_PHY);
		status &= ~INT_PHY;
		schedule_delayed_work(&ks->link_read, 0);
	}
	mutex_unlock(&ks->lock);
	if (dev->adjust_state)
		dev->adjust_state(dev->attached_dev);
	else if (status) {
		mutex_lock(&ks->lock);
		HW_W(ks, REG_INT_STATUS, status);
		mutex_unlock(&ks->lock);
	}
	priv->ptp_irq = false;
	mutex_unlock(&ks->hwlock);

	atomic_dec(&phydev->irq_disable);
#ifdef USE_LEVEL_INTR
	enable_irq(phydev->irq);
#endif
}  /* spi_change */

static int spi_start_interrupt(struct phy_device *phydev, const char *name)
{
	int err = 0;

	INIT_WORK(&phydev->phy_queue, spi_change);

	atomic_set(&phydev->irq_disable, 0);
	if (request_irq(phydev->irq, spi_interrupt,
#ifdef USE_LEVEL_INTR
			IRQF_TRIGGER_LOW,
#else
			IRQF_TRIGGER_FALLING,
#endif
			name,
			phydev) < 0) {
		printk(KERN_WARNING "%s: Can't get IRQ %d (PHY)\n",
			phydev->bus->name,
			phydev->irq);
		phydev->irq = PHY_POLL;
		return 0;
	}
#if 0
	phydev->state = PHY_RUNNING;
#endif

	return err;
}  /* spi_start_interrupt */

static void spi_stop_interrupt(struct phy_device *phydev)
{
	free_irq(phydev->irq, phydev);
	cancel_work_sync(&phydev->phy_queue);
	while (atomic_dec_return(&phydev->irq_disable) >= 0)
		enable_irq(phydev->irq);
}  /* spi_stop_interrupt */

static int ksz_mii_addr(int *reg, int *bank)
{
	int ret;

	ret = (*reg & 0xC000) >> ADDR_SHIFT;
	*bank = (*reg & 0x3000) >> BANK_SHIFT;
	*reg &= 0x0FFF;
	return ret;
}

/*
 * Tha  2011/03/11
 * The hardware register reads low word first of PHY id instead of high word.
 */
static inline int actual_reg(int regnum)
{
	if (2 == regnum)
		regnum = 3;
	else if (3 == regnum)
		regnum = 2;
	return regnum;
}

static int ksz_mii_read(struct mii_bus *bus, int phy_id, int regnum)
{
	struct spi_priv *ks = bus->priv;
	int addr;
	int bank;
	int ret = 0xffff;

	if (phy_id > SWITCH_PORT_NUM)
		return 0xffff;

	addr = ksz_mii_addr(&regnum, &bank);

	mutex_lock(&ks->lock);
	switch (addr) {
	case ADDR_8:
		ret = spi_rdreg8(ks, regnum);
		break;
	case ADDR_16:
		ret = spi_rdreg16(ks, regnum);
		break;
	case ADDR_32:
		ret = spi_rdreg32(ks, regnum);
		break;
	default:
		if (regnum < 6) {
			regnum = actual_reg(regnum);
			if (0 == phy_id)
				phy_id = ks->phy_id;
			if (2 == phy_id)
				ret = spi_rdreg16(ks,
					PHY2_REG_CTRL + regnum * 2);
			else
				ret = spi_rdreg16(ks,
					PHY1_REG_CTRL + regnum * 2);
		} else
			ret = 0;
	}
	mutex_unlock(&ks->lock);
	return ret;
}  /* ksz_mii_read */

static int ksz_mii_write(struct mii_bus *bus, int phy_id, int regnum, u16 val)
{
	static int last_reg;
	static int last_val;
	struct spi_priv *ks = bus->priv;
	int addr;
	int bank;
	int reg;

	if (phy_id > SWITCH_PORT_NUM)
		return -EINVAL;

	reg = regnum;
	addr = ksz_mii_addr(&regnum, &bank);

	mutex_lock(&ks->lock);
	switch (addr) {
	case ADDR_8:
		spi_wrreg8(ks, regnum, val);
		break;
	case ADDR_16:
		spi_wrreg16(ks, regnum, val);
		break;
	case ADDR_32:
		/*
		 * The phy_write interface allows only 16-bit value.  Break
		 * the 32-bit write into two calls for SPI efficiency.
		 */

		/* Previous write to high word. */
		if (last_reg == reg + 2) {
			last_val <<= 16;
			last_val |= val;
			spi_wrreg32(ks, regnum, last_val);
			last_reg = 0;
		} else {
			/* Somebody has written to different address! */
			if (last_reg) {
				int last_bank;

				addr = ksz_mii_addr(&last_reg, &last_bank);
				spi_wrreg16(ks, last_reg, last_val);
				last_reg = 0;
			}

			/* Cache the 16-bit write to high word. */
			if (reg & 3) {
				last_reg = reg;
				last_val = val;

			/* Did not find the previous write to high word.*/
			} else
				spi_wrreg16(ks, regnum, val);
		}
		break;
	default:
		if (regnum < 6) {
			if (2 == phy_id || 0 == phy_id)
				spi_wrreg16(ks,
					PHY2_REG_CTRL + regnum * 2, val);
			if (1 == phy_id || 0 == phy_id)
				spi_wrreg16(ks,
					PHY1_REG_CTRL + regnum * 2, val);
		}
		break;
	}
	mutex_unlock(&ks->lock);
	return 0;
}  /* ksz_mii_write */

static int DEVINIT ksz_mii_init(struct spi_priv *ks)
{
	struct platform_device *pdev;
	struct mii_bus *bus;
	int err;
	int i;

	pdev = platform_device_register_simple("SPI MII bus", ks->sw.id,
		NULL, 0);
	if (!pdev)
		return -ENOMEM;

	bus = mdiobus_alloc();
	if (bus == NULL) {
		err = -ENOMEM;
		goto mii_init_reg;
	}

	bus->name = "SPI MII bus",
	bus->read = ksz_mii_read;
	bus->write = ksz_mii_write;
	snprintf(bus->id, MII_BUS_ID_SIZE, "spi_mii.%d", ks->sw.id);
	bus->parent = &pdev->dev;
	bus->priv = ks;
	bus->irq = ks->irq;

	for (i = 0; i < PHY_MAX_ADDR; i++)
		bus->irq[i] = ks->spidev->irq;

	err = mdiobus_register(bus);
	if (err < 0)
		goto mii_init_free_mii_bus;

	if (!bus->phy_map[0]) {
		printk(KERN_WARNING "No PHY detected\n");
		mdiobus_unregister(bus);
		err = -ENODEV;
		goto mii_init_free_mii_bus;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		if (bus->phy_map[i]) {
			struct phy_priv *phydata;

			phydata = kzalloc(sizeof(struct phy_priv), GFP_KERNEL);
			if (!phydata) {
				err = -ENOMEM;
				goto mii_init_free_mii_bus;
			}
			phydata->port.sw = &ks->sw;
			bus->phy_map[i]->priv = phydata;
		}

	ks->bus = bus;
	ks->pdev = pdev;
	ks->phydev = bus->phy_map[0];
	ks->phy_id = 1;

	return 0;

mii_init_free_mii_bus:
	for (i = 0; i < PHY_MAX_ADDR; i++)
		if (bus->phy_map[i])
			kfree(bus->phy_map[i]->priv);
	mdiobus_free(bus);

mii_init_reg:
	platform_device_unregister(pdev);

	return err;
}  /* ksz_mii_init */

static void __devexit ksz_mii_exit(struct spi_priv *ks)
{
	int i;
	struct platform_device *pdev = ks->pdev;
	struct mii_bus *bus = ks->bus;

	if (ks->phydev->irq > 0) {
		mutex_lock(&ks->lock);
		spi_wrreg16(ks, TS_INT_ENABLE, 0);
		spi_wrreg16(ks, TRIG_INT_ENABLE, 0);
		spi_wrreg16(ks, REG_INT_MASK, 0);
		mutex_unlock(&ks->lock);
		spi_stop_interrupt(ks->phydev);
	}
	for (i = 0; i < PHY_MAX_ADDR; i++)
		if (bus->phy_map[i])
			kfree(bus->phy_map[i]->priv);
	mdiobus_unregister(bus);
	mdiobus_free(bus);
	platform_device_unregister(pdev);
}  /* ksz_mii_exit */

/* driver bus management functions */

static unsigned long next_jiffies;

static void ksz846x_mib_read_work(struct work_struct *work)
{
	struct spi_priv *hw_priv =
		container_of(work, struct spi_priv, mib_read);
	struct ksz_sw *sw = &hw_priv->sw;
	struct ksz_port_mib *mib;
	int i;

	next_jiffies = jiffies;
	for (i = 0; i < sw->mib_port_cnt; i++) {
		mib = &sw->port_mib[i];

		/* Reading MIB counters or requested to read. */
		if (mib->cnt_ptr || 1 == hw_priv->counter[i].read) {

			/* Need to process interrupt. */
			if (port_r_cnt(sw, i))
				return;
			hw_priv->counter[i].read = 0;

			/* Finish reading counters. */
			if (0 == mib->cnt_ptr) {
				hw_priv->counter[i].read = 2;
				wake_up_interruptible(
					&hw_priv->counter[i].counter);
			}
		} else if (jiffies >= hw_priv->counter[i].time) {
			/* Only read MIB counters when the port is connected. */
			if (media_connected == sw->port_state[i].state)
				hw_priv->counter[i].read = 1;
			next_jiffies += HZ * 1 * sw->mib_port_cnt;
			hw_priv->counter[i].time = next_jiffies;

		/* Port is just disconnected. */
		} else if (sw->port_state[i].link_down) {
			sw->port_state[i].link_down = 0;

			/* Read counters one last time after link is lost. */
			hw_priv->counter[i].read = 1;
		}
	}
}  /* ksz846x_mib_read_work */

static void link_read_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct spi_priv *hw_priv =
		container_of(dwork, struct spi_priv, link_read);
	struct ksz_sw *sw = &hw_priv->sw;
	struct phy_device *phydev;
	struct ksz_port *port = NULL;
	int i;
	int changes = 0;

	sw_acquire(sw, sw->hwlock);
	for (i = 0; i < sw->dev_count + sw->dev_offset; i++) {
		struct dev_priv *priv;
		struct phy_priv *phydata;
		struct net_device *dev = sw->netdev[i];

		phydev = sw->phy[i + (1 - sw->dev_offset)];
		phydata = phydev->priv;
		if (dev) {
			priv = netdev_priv(dev);
			port = &priv->port;
		} else
			port = &phydata->port;
		changes |= port_get_link_speed(port);

		/* Copy all port information for user access. */
		if (dev) {
			phydata->port = *port;
			phydata = hw_priv->phydev->priv;
			phydata->port = *port;
		}
	}
	sw_release(sw, sw->hwlock);

	/* Not to read PHY registers unnecessarily if no link change. */
	if (!changes)
		return;

	/* This only matters when one phy device is used for the switch. */
	if (1 == sw->dev_count)
		for (i = 0; i < SWITCH_PORT_NUM; i++)
			if (port->linked == &sw->port_info[i]) {
				hw_priv->phy_id = i + 1;
				break;
			}
	for (i = 0; i < PHY_MAX_ADDR; i++)
		if (hw_priv->bus->phy_map[i])
			phy_read_status(hw_priv->bus->phy_map[i]);
	phydev = hw_priv->phydev;
	if (phydev->adjust_link)
		phydev->adjust_link(phydev->attached_dev);
}  /* link_read_work */

static void stp_work(struct work_struct *work)
{
	struct spi_priv *hw_priv =
		container_of(work, struct spi_priv, stp_monitor);
	struct ksz_sw *sw = &hw_priv->sw;

	sw_acquire(sw, sw->hwlock);
	monitor_ports(sw);
	sw_release(sw, sw->hwlock);
}  /* stp_work */

/*
 * Hardware monitoring
 */

static void ksz846x_mib_monitor(unsigned long ptr)
{
	struct spi_priv *hw_priv = (struct spi_priv *) ptr;

	schedule_work(&hw_priv->mib_read);

	ksz_update_timer(&hw_priv->mib_timer_info);
}  /* ksz846x_mib_monitor */

static void ksz846x_dev_monitor(unsigned long ptr)
{
	struct spi_priv *hw_priv = (struct spi_priv *) ptr;

	if (hw_priv->intr_working && !(hw_priv->sw.features & STP_SUPPORT))
		return;
	if (!hw_priv->intr_working)
		schedule_delayed_work(&hw_priv->link_read, 0);
	if (hw_priv->sw.features & STP_SUPPORT)
		schedule_work(&hw_priv->stp_monitor);

	ksz_update_timer(&hw_priv->monitor_timer_info);
}  /* ksz846x_dev_monitor */

#define MAX_SPI_DEVICES		2

static int spi_device_present;

struct spi_info {
	struct spi_device *spidev;
	struct net_device *netdev[SWITCH_PORT_NUM];
};

struct spi_info spi_devices[MAX_SPI_DEVICES];
EXPORT_SYMBOL(spi_devices);

static int DEVINIT ksz846x_probe(struct spi_device *spi)
{
	struct spi_priv *ks;
	struct ksz_sw *sw;
	struct ksz_port *port;
	struct phy_device *phydev;
	struct phy_priv *priv;
	u16 id;
	int cnt;
	int i;
	int mib_port_count;
	int pi;
	int port_count;
	int ret;

	/* hook to this SPI master driver */
	spi->master->bus_num = 0;

	spi->bits_per_word = 8;

	ks = kzalloc(sizeof(struct spi_priv), GFP_KERNEL);
	if (!ks) {
		return -ENOMEM;
	}

	mutex_init(&ks->hwlock);
	mutex_init(&ks->lock);

	ks->spidev = spi;

	/* initialise pre-made spi transfer messages */

	spi_message_init(&ks->spi_msg1);
	spi_message_add_tail(&ks->spi_xfer1, &ks->spi_msg1);

	spi_message_init(&ks->spi_msg2);
	spi_message_add_tail(&ks->spi_xfer2[0], &ks->spi_msg2);
	spi_message_add_tail(&ks->spi_xfer2[1], &ks->spi_msg2);

	dev_set_drvdata(&spi->dev, ks);

	/* simple check for a valid chip being connected to the bus */
	mutex_lock(&ks->lock);
	id = spi_rdreg16(ks, REG_SWITCH_SIDER);
	mutex_unlock(&ks->lock);
	if ((id & CIDER_ID_MASK) != CIDER_ID_8463 &&
			(id & CIDER_ID_MASK) != CIDER_ID_8463_RLI) {
		dev_err(&spi->dev, "failed to read device ID(0x%x)\n", id);
		ret = -ENODEV;
		goto err_sw;
	}
	dev_info(&spi->dev, "chip id 0x%x, spi bus %d\n", id,
		spi->master->bus_num);

	sw = &ks->sw;
	mutex_init(&sw->lock);
	sw->hwlock = &ks->hwlock;
	sw->reglock = &ks->lock;

	sw->dev_count = 1;

	port_count = SWITCH_PORT_NUM;
	mib_port_count = SWITCH_PORT_NUM;

	sw->mib_cnt = TOTAL_SWITCH_COUNTER_NUM;
	sw->mib_port_cnt = TOTAL_PORT_NUM;

	sw->dev = ks;
	sw->id = spi_device_present;

	sw->info = kzalloc(sizeof(struct ksz_sw_info), GFP_KERNEL);
	if (!sw->info) {
		ret = -ENOMEM;
		goto err_sw;
	}

	INIT_DELAYED_WORK(&ks->link_read, link_read_work);
	INIT_WORK(&ks->stp_monitor, stp_work);
	ret = ksz_mii_init(ks);
	if (ret)
		goto err_mii;

	sw->phydev = ks->phydev;
	sw->counter = ks->counter;
	sw->monitor_timer_info = &ks->monitor_timer_info;
	sw->link_read = &ks->link_read;
	sw->stp_monitor = &ks->stp_monitor;

	for (i = 0; i < sw->mib_port_cnt; i++) {
		sw->port_mib[i].mib_start = 0;
		if (next_jiffies < jiffies)
			next_jiffies = jiffies + HZ * 2;
		else
			next_jiffies += HZ * 1;
		sw->counter[i].time = next_jiffies;
		sw->port_state[i].state = media_disconnected;
		port_init_cnt(sw, i);
	}
	sw->port_state[HOST_PORT].state = media_connected;

	for (i = 0; i < TOTAL_PORT_NUM; i++)
		init_waitqueue_head(&ks->counter[i].counter);

	spi_create_debugfs(ks);

	sw_acquire(sw, sw->reglock);
	sw_init(sw);
	sw_setup(sw);
	sw_release(sw, sw->reglock);

	init_sw_sysfs(&ks->sysfs, &ks->spidev->dev);
	sema_init(&ks->proc_sem, 1);

	i = 0;
	phydev = ks->phydev;
	priv = phydev->priv;
	port = &priv->port;
	port->port_cnt = port_count;
	port->mib_port_cnt = mib_port_count;
	port->first_port = i;
	port->flow_ctrl = PHY_FLOW_CTRL;

	port->sw = sw;
	port->linked = &sw->port_info[port->first_port];

	for (cnt = 0, pi = i; cnt < port_count; cnt++, pi++) {
		/*
		 * Initialize to invalid value so that link detection
		 * is done.
		 */
		sw->port_info[pi].partner = 0xFF;
		sw->port_info[pi].port_id = pi;
		sw->port_info[pi].state = media_disconnected;
	}
	for (i = 0; i <= SWITCH_PORT_NUM; i++) {
		struct phy_device *next_phydev;
		struct phy_priv *next_priv;

		sw->phy[i] = ks->bus->phy_map[i];
		next_phydev = sw->phy[i];
		if (!next_phydev)
			continue;
		next_priv = next_phydev->priv;
		next_priv->port = priv->port;
	}

	INIT_WORK(&ks->mib_read, ksz846x_mib_read_work);

	/* 500 ms timeout */
	ksz_init_timer(&ks->mib_timer_info, 500 * HZ / 1000,
		ksz846x_mib_monitor, ks);
	ksz_init_timer(&ks->monitor_timer_info, 500 * HZ / 1000,
		ksz846x_dev_monitor, ks);

	ksz_start_timer(&ks->mib_timer_info, ks->mib_timer_info.period);

	spi_devices[spi_device_present++].spidev = spi;

	if (ks->phydev->irq <= 0)
		return 0;
	ks->intr_mask = INT_PHY | INT_TIMESTAMP | INT_TRIG_OUTPUT;
	mutex_lock(&ks->lock);
	spi_wrreg16(ks, TS_INT_ENABLE, 0);
	spi_wrreg16(ks, TS_INT_STATUS, 0xffff);
	mutex_unlock(&ks->lock);
	ret = spi_start_interrupt(ks->phydev, dev_name(&ks->spidev->dev));
	if (ret < 0)
		printk(KERN_WARNING "No SPI interrupt\n");
	else {
		mutex_lock(&ks->lock);
		spi_wrreg16(ks, REG_INT_MASK, ks->intr_mask);
		mutex_unlock(&ks->lock);
	}

	return 0;

err_mii:
	kfree(sw->info);

err_sw:
	kfree(ks);

	return ret;
}

static int __devexit ksz846x_remove(struct spi_device *spi)
{
	struct spi_priv *ks = dev_get_drvdata(&spi->dev);

	ksz_stop_timer(&ks->mib_timer_info);
	flush_work(&ks->mib_read);

	exit_sw_sysfs(&ks->sysfs, &spi->dev);
	cancel_delayed_work_sync(&ks->link_read);
	flush_work(&ks->stp_monitor);
	spi_delete_debugfs(ks);
	kfree(ks->sw.info);
	ksz_mii_exit(ks);
	kfree(ks);

	return 0;
}

static struct spi_driver ksz846x_driver = {
	.driver = {
		.name = "ksz8463",
		.owner = THIS_MODULE,
	},
	.probe = ksz846x_probe,
	.remove = __devexit_p(ksz846x_remove),
};

static int __init ksz846x_init(void)
{
	return spi_register_driver(&ksz846x_driver);
}

static void __exit ksz846x_exit(void)
{
	spi_unregister_driver(&ksz846x_driver);
}

#ifndef CONFIG_MICREL_KSZ8463_EMBEDDED
module_init(ksz846x_init);
module_exit(ksz846x_exit);
#endif

#ifndef CONFIG_MICREL_KSZ8463_EMBEDDED
MODULE_DESCRIPTION("Micrel KSZ8463 MLI Switch Driver");
MODULE_AUTHOR("Tristram Ha <Tristram.Ha@micrel.com>");
MODULE_LICENSE("GPL");

MODULE_ALIAS("spi:ksz8463");
#endif
