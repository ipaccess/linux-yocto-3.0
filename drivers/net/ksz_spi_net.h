/* drivers/net/ksz_spi_net.h - Micrel SPI switch common header
 *
 * Copyright (c) 2012 Micrel, Inc.
 *	Tristram Ha <Tristram.Ha@micrel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef KSZ_SPI_NET_H
#define KSZ_SPI_NET_H


/**
 * struct spi_priv - SPI device private data structure
 * @spidev:		Adapter device information.
 * @sw:			Virtual switch structure.
 * @proc_sem:		Semaphore for sysfs accessing.
 */
struct spi_priv {
	struct spi_device *spidev;
	struct spi_message spi_msg1;
	struct spi_message spi_msg2;
	struct spi_transfer spi_xfer1;
	struct spi_transfer spi_xfer2[2];

	struct ksz_sw sw;

	struct ksz_sw_sysfs sysfs;
	struct semaphore proc_sem;

	struct mutex hwlock;
	struct mutex lock;

	struct delayed_work link_read;
	struct work_struct mib_read;
	struct work_struct stp_monitor;
	struct ksz_timer_info mib_timer_info;
	struct ksz_timer_info monitor_timer_info;
	struct ksz_counter_info counter[TOTAL_PORT_NUM];

	struct dentry *debug_root;
	struct dentry *debug_file;

	int phy_id;
	int intr_working;
	uint intr_mask;

	u8 rxd[8];
	u8 txd[8];

	struct platform_device *pdev;
	struct mii_bus *bus;
	int irq[PHY_MAX_ADDR];
	char name[40];
	struct phy_device *phydev;
	struct phy_priv *phypriv;
};

/**
 * struct dev_priv - Network device private data structure
 * @adapter:		Adapter device information.
 * @stats:		Network statistics.
 * @phydev:		The PHY device associated with the device.
 * @phy_pause:		Workqueue to pause the PHY state machine.
 * @id:			Device ID.
 * @mii_if:		MII interface information.
 * @advertising:	Temporary variable to store advertised settings.
 * @msg_enable:		The message flags controlling driver output.
 * @media_state:	The connection status of the device.
 * @multicast:		The all multicast state of the device.
 * @promiscuous:	The promiscuous state of the device.
 */
struct dev_priv {
	void *adapter;
	struct net_device *dev;
	void *parent;
	struct ksz_port port;
	struct ksz_timer_info monitor_timer_info;
	struct net_device_stats stats;

	struct phy_device *phydev;
	struct work_struct phy_pause;

	int id;

	struct mii_if_info mii_if;
	u32 advertising;

	u32 msg_enable;
	int media_state;
	int multicast;
	int promiscuous;
	u8 phy_addr;
	u8 state;
	u8 multi_list_size;
	u8 multi_list[MAX_MULTICAST_LIST][MAC_ADDR_LEN];

#ifdef DEBUG_COUNTER
	unsigned long query_jiffies;
#endif
};

static void get_private_data_2(struct device *d, struct semaphore **proc_sem,
	struct ksz_sw **sw)
{
	struct spi_device *spi;
	struct net_device *dev;
	struct dev_priv *priv;
	struct spi_priv *hw_priv;

	if (d->bus && d->bus == &spi_bus_type) {
		spi = to_spi_device(d);
		hw_priv = dev_get_drvdata(&spi->dev);
	} else {
		dev = to_net_dev(d);
		priv = netdev_priv(dev);
		hw_priv = priv->parent;
	}
	*proc_sem = &hw_priv->proc_sem;
	*sw = &hw_priv->sw;
}

static void get_private_data_3(struct device *d, struct semaphore **proc_sem,
	struct ksz_sw **sw, struct mutex **hwlock)
{
	struct spi_device *spi;
	struct net_device *dev;
	struct dev_priv *priv;
	struct spi_priv *hw_priv;

	if (d->bus && d->bus == &spi_bus_type) {
		spi = to_spi_device(d);
		hw_priv = dev_get_drvdata(&spi->dev);
	} else {
		dev = to_net_dev(d);
		priv = netdev_priv(dev);
		hw_priv = priv->parent;
	}
	*proc_sem = &hw_priv->proc_sem;
	*sw = &hw_priv->sw;
	*hwlock = &hw_priv->lock;
}

static void get_private_data(struct device *d, struct semaphore **proc_sem,
	struct ksz_sw **sw, struct mutex **hwlock, struct ksz_port **port)
{
	struct spi_device *spi;
	struct net_device *dev;
	struct dev_priv *priv = NULL;
	struct spi_priv *hw_priv;

	if (d->bus && d->bus == &spi_bus_type) {
		struct phy_device *phydev;
		struct phy_priv *phydata;

		spi = to_spi_device(d);
		hw_priv = dev_get_drvdata(&spi->dev);
		phydev = hw_priv->phydev;
		phydata = hw_priv->phydev->priv;
		*port = &phydata->port;
	} else {
		dev = to_net_dev(d);
		priv = netdev_priv(dev);
		hw_priv = priv->parent;
		*port = &priv->port;
	}
	*proc_sem = &hw_priv->proc_sem;
	*sw = &hw_priv->sw;
	*hwlock = &hw_priv->lock;
}
#endif

