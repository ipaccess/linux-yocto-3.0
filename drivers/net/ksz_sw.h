/**
 * drivers/net/ksz_sw.h - Micrel switch common header
 *
 * Copyright (c) 2010-2012 Micrel, Inc.
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

#ifndef KSZ_SW_H
#define KSZ_SW_H


/* These definitions should be defined before this header file. */
#ifndef PRIO_QUEUES
#define PRIO_QUEUES			4
#endif

#ifndef KS_PRIORITY_IN_REG
#define KS_PRIORITY_IN_REG		4
#endif

#ifndef SWITCH_PORT_NUM
#define SWITCH_PORT_NUM			2
#endif

#ifndef SWITCH_COUNTER_NUM
#define SWITCH_COUNTER_NUM		0x20
#endif
#ifndef TOTAL_SWITCH_COUNTER_NUM
#define TOTAL_SWITCH_COUNTER_NUM	(SWITCH_COUNTER_NUM + 2)
#endif

#ifndef SW_D
#error "SW_D and other data bus parameters need to be defined."
#endif

#define KS_PRIORITY_MASK		SWITCH_802_1P_MAP_MASK
#define KS_PRIORITY_SHIFT		SWITCH_802_1P_MAP_SHIFT

#define LEARNED_MAC_TABLE_ENTRIES	1024
#define STATIC_MAC_TABLE_ENTRIES	8

/**
 * struct ksz_mac_table - Static MAC table data structure
 * @mac_addr:	MAC address to filter.
 * @vid:	VID value.
 * @fid:	FID value.
 * @ports:	Port membership.
 * @override:	Override setting.
 * @use_fid:	FID use setting.
 * @valid:	Valid setting indicating the entry is being used.
 */
struct ksz_mac_table {
	u8 mac_addr[MAC_ADDR_LEN];
	u16 vid;
	u8 fid;
	u8 ports;
	u8 override:1;
	u8 use_fid:1;
	u8 valid:1;
};

#define VLAN_TABLE_ENTRIES		16

/**
 * struct ksz_vlan_table - VLAN table data structure
 * @vid:	VID value.
 * @fid:	FID value.
 * @member:	Port membership.
 * @valid:	Valid setting indicating the entry is being used.
 */
struct ksz_vlan_table {
	u16 vid;
	u8 fid;
	u8 member;
	u8 valid:1;
};

#define PRIO_802_1P_ENTRIES		8

#define DIFFSERV_ENTRIES		64

#define TOTAL_PORT_NUM			(SWITCH_PORT_NUM + 1)
#define HOST_MASK			(1 << SWITCH_PORT_NUM)
#define PORT_MASK			((1 << (SWITCH_PORT_NUM + 1)) - 1)

#define HOST_PORT			SWITCH_PORT_NUM

/**
 * struct ksz_port_mib - Port MIB data structure
 * @cnt_ptr:	Current pointer to MIB counter index.
 * @mib_start:	The starting counter index.  Some ports do not start at 0.
 * @counter:	64-bit MIB counter value.
 * @dropped:	Temporary buffer to remember last read packet dropped values.
 *
 * MIB counters needs to be read periodically so that counters do not get
 * overflowed and give incorrect values.  A right balance is needed to
 * satisfy this condition and not waste too much CPU time.
 */
struct ksz_port_mib {
	u8 cnt_ptr;
	u8 mib_start;
	u8 reserved[2];

	u64 counter[TOTAL_SWITCH_COUNTER_NUM];
	u32 dropped[2];
};

/**
 * struct ksz_port_cfg - Port configuration data structure
 * @vid:	VID value.
 * @member:	Port membership.
 * @port_prio:	Port priority.
 * @rate_ctrl:	Priority rate control.
 * @rx_rate:	Receive priority rate.
 * @tx_rate:	Transmit priority rate.
 * @mac_addr:	MAC address of the port.
 * @rate_limit: Priority rate limit value.
 * @stp_state:	Current Spanning Tree Protocol state.
 */
struct ksz_port_cfg {
	u16 vid;
	u8 member;
	u8 port_prio;
	u8 rate_ctrl[PRIO_QUEUES];
	u32 rx_rate[PRIO_QUEUES];
	u32 tx_rate[PRIO_QUEUES];
	u8 mac_addr[MAC_ADDR_LEN];
	u8 rate_limit;
	int stp_state;
};

/**
 * struct ksz_sw_info - KSZ8463 switch information data structure
 * @mac_table:	MAC table entries information.
 * @vlan_table:	VLAN table entries information.
 * @port_cfg:	Port configuration information.
 * @diffserv:	DiffServ priority settings.  Possible values from 6-bit of ToS
 *		(bit7 ~ bit2) field.
 * @p_802_1p:	802.1P priority settings.  Possible values from 3-bit of 802.1p
 *		Tag priority field.
 * @br_addr:	Bridge address.  Used for STP.
 * @mac_addr:	Switch MAC address.
 * @broad_per:	Broadcast storm percentage.
 * @member:	Current port membership.  Used for STP.
 * @stp:	STP port membership.  Used for STP.
 * @phy_addr:	PHY address used by first port.
 */
struct ksz_sw_info {
	struct ksz_mac_table mac_table[STATIC_MAC_TABLE_ENTRIES];
	struct ksz_vlan_table vlan_table[VLAN_TABLE_ENTRIES];
	struct ksz_port_cfg port_cfg[TOTAL_PORT_NUM];

	SW_D diffserv[DIFFSERV_ENTRIES / KS_PRIORITY_IN_REG];
	SW_D p_802_1p[PRIO_802_1P_ENTRIES / KS_PRIORITY_IN_REG];

	u8 br_addr[MAC_ADDR_LEN];
	u8 mac_addr[MAC_ADDR_LEN];

	u8 broad_per;
	u8 member;
	u8 stp;
	u8 phy_addr;
};

/**
 * struct ksz_port_state - Port state information data structure
 * @state:	Connection status of the port.
 * @link_down:	Indication the link has just gone down.
 *
 * It is pointless to read MIB counters when the port is disconnected.  The
 * @state provides the connection status so that MIB counters are read only
 * when the port is connected.  The @link_down indicates the port is just
 * disconnected so that all MIB counters are read one last time to update the
 * information.
 */
struct ksz_port_state {
	uint state;
	u8 link_down;
};

#define TX_RATE_UNIT			10000

/**
 * struct ksz_port_info - Port information data structure
 * @state:	Connection status of the port.
 * @tx_rate:	Transmit rate divided by 10000 to get Mbit.
 * @duplex:	Duplex mode.
 * @advertised:	Advertised auto-negotiation setting.  Used to determine link.
 * @partner:	Auto-negotiation partner setting.  Used to determine link.
 * @port_id:	Port index to access actual hardware register.
 * @status:	LinkMD status values.
 * @length:	LinkMD length values.
 */
struct ksz_port_info {
	uint state;
	uint tx_rate;
	u8 duplex;
	u8 advertised;
	u8 partner;
	u8 port_id;
	u32 status[3];
	u32 length[3];
};

/* Switch features and bug fixes. */
#define STP_SUPPORT			(1 << 0)
#define VLAN_PORT			(1 << 1)
#define VLAN_PORT_REMOVE_TAG		(1 << 2)
#define VLAN_PORT_START			200
#define DIFF_MAC_ADDR			(1 << 31)

/* Software overrides. */
#define PAUSE_FLOW_CTRL			(1 << 0)
#define FAST_AGING			(1 << 1)
#define TAG_REMOVE			(1 << 30)
#define TAIL_TAGGING			(1 << 31)

/**
 * struct ksz_sw - Virtual switch data structure
 * @dev:		Pointer to hardware device.
 * @phydev:		Pointer to PHY device interface.
 * @hwlock:		Pointer to hardware lock.
 * @reglock:		Pointer to register lock.
 * @lock		Software lock to switch structure.
 * @locked:		locked status.
 * @info:		Pointer to switch information structure.
 * @port_info:		Port information.
 * @netdev:		Pointer to OS dependent network devices.
 * @phy:		Pointer to OS dependent PHY devices.
 * @dev_offset:		Indication of a switch associated network device.
 * @port_state:		Port state information.
 * @port_mib:		Port MIB information.
 * @mib_cnt:		Number of MIB counters this switch has.
 * @mib_port_cnt:	Number of ports with MIB counters.
 * @monitor_timer_info:	Timer information for monitoring link.
 * @counter:		Pointer to OS dependent MIB counter information.
 * @link_read:		Work for link monitoring.
 * @stp_monitor:	Work for STP monitoring.
 * @dev_count:		Number of network devices this switch supports.
 * @id:			Hardware ID.  Used for display only.
 * @vlan_id		Used for the VLAN port forwarding feature.
 * @features:		Switch features to enable.
 * @overrides:		Switch features to override.
 */
struct ksz_sw {
	void *dev;
	void *phydev;
	struct mutex *hwlock;
	struct mutex *reglock;
	struct mutex lock;
	int locked;

	struct ksz_sw_info *info;
	struct ksz_port_info port_info[SWITCH_PORT_NUM];
	struct net_device *netdev[TOTAL_PORT_NUM];
	struct phy_device *phy[TOTAL_PORT_NUM];
	int dev_offset;
	struct ksz_port_state port_state[TOTAL_PORT_NUM];
	struct ksz_port_mib port_mib[TOTAL_PORT_NUM];
	int mib_cnt;
	int mib_port_cnt;
	struct ksz_timer_info *monitor_timer_info;
	struct ksz_counter_info *counter;
	struct delayed_work *link_read;
	struct work_struct *stp_monitor;

	int dev_count;
	int id;
	u32 vlan_id;
	u16 vid;

	uint features;
	uint overrides;
};

struct ksz_sw_sysfs {
	struct ksz_dev_attr *ksz_port_attrs[TOTAL_PORT_NUM];
	struct attribute **port_attrs[TOTAL_PORT_NUM];
	struct ksz_dev_attr *ksz_mac_attrs[STATIC_MAC_TABLE_ENTRIES];
	struct attribute **mac_attrs[STATIC_MAC_TABLE_ENTRIES];
	struct ksz_dev_attr *ksz_vlan_attrs[VLAN_TABLE_ENTRIES];
	struct attribute **vlan_attrs[VLAN_TABLE_ENTRIES];
};

/**
 * struct ksz_port - Virtual port data structure
 * @duplex:		Duplex mode setting.  1 for half duplex, 2 for full
 *			duplex, and 0 for auto, which normally results in full
 *			duplex.
 * @speed:		Speed setting.  10 for 10 Mbit, 100 for 100 Mbit, and
 *			0 for auto, which normally results in 100 Mbit.
 * @force_link:		Force link setting.  0 for auto-negotiation, and 1 for
 *			force.
 * @flow_ctrl:		Flow control setting.  PHY_NO_FLOW_CTRL for no flow
 *			control, and PHY_FLOW_CTRL for flow control.
 *			PHY_TX_ONLY and PHY_RX_ONLY are not supported for 100
 *			Mbit PHY.
 * @first_port:		Index of first port this port supports.
 * @mib_port_cnt:	Number of ports with MIB counters.
 * @port_cnt:		Number of ports this port supports.
 * @sw:			Pointer to virtual switch structure.
 * @linked:		Pointer to port information linked to this port.
 */
struct ksz_port {
	u8 duplex;
	u8 speed;
	u8 force_link;
	u8 flow_ctrl;

	int first_port;
	int mib_port_cnt;
	int port_cnt;

	struct ksz_sw *sw;
	struct ksz_port_info *linked;
};

#endif
