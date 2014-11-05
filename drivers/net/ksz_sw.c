/* drivers/net/ksz_sw.c - Micrel switch common code
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


/* -------------------------------------------------------------------------- */

enum {
	PROC_SW_INFO,

	PROC_SET_SW_DUPLEX,
	PROC_SET_SW_SPEED,
	PROC_SET_SW_FORCE,
	PROC_SET_SW_FLOW_CTRL,

	PROC_SET_SW_FEATURES,
	PROC_SET_SW_OVERRIDES,
	PROC_SET_SW_MIB,

	PROC_SET_SW_REG,
	PROC_SET_SW_VID,

	PROC_DYNAMIC,
	PROC_STATIC,
	PROC_VLAN,

	PROC_SET_AGING,
	PROC_SET_FAST_AGING,
	PROC_SET_LINK_AGING,

	PROC_SET_BROADCAST_STORM,
	PROC_SET_MULTICAST_STORM,
	PROC_SET_DIFFSERV,
	PROC_SET_802_1P,

	PROC_ENABLE_VLAN,
	PROC_SET_REPLACE_NULL_VID,
	PROC_SET_MAC_ADDR,
	PROC_SET_MIRROR_MODE,
	PROC_SET_TAIL_TAG,

	PROC_SET_IGMP_SNOOP,
	PROC_SET_IPV6_MLD_SNOOP,
	PROC_SET_IPV6_MLD_OPTION,

	PROC_SET_AGGR_BACKOFF,
	PROC_SET_NO_EXC_DROP,
	PROC_SET_BUFFER_RESERVE,

	PROC_SET_HUGE_PACKET,
	PROC_SET_LEGAL_PACKET,
	PROC_SET_LENGTH_CHECK,

	PROC_SET_BACK_PRESSURE_MODE,
	PROC_SET_SWITCH_FLOW_CTRL,
	PROC_SET_SWITCH_HALF_DUPLEX,
	PROC_SET_SWITCH_10_MBIT,

	PROC_SET_RX_FLOW_CTRL,
	PROC_SET_TX_FLOW_CTRL,
	PROC_SET_FAIR_FLOW_CTRL,
	PROC_SET_VLAN_BOUNDARY,

	PROC_SET_FORWARD_UNKNOWN_DEST,
	PROC_SET_INS_TAG_0_1,
	PROC_SET_INS_TAG_0_2,
	PROC_SET_INS_TAG_1_0,
	PROC_SET_INS_TAG_1_2,
	PROC_SET_INS_TAG_2_0,
	PROC_SET_INS_TAG_2_1,

	PROC_SET_PASS_ALL,
	PROC_SET_PASS_PAUSE,

	PROC_SET_PHY_ADDR,
};

enum {
	PROC_SET_PORT_MIB,

	PROC_SET_DEF_VID,
	PROC_SET_MEMBER,

	PROC_ENABLE_BROADCAST_STORM,
	PROC_ENABLE_DIFFSERV,
	PROC_ENABLE_802_1P,

	PROC_SET_PORT_BASED,

	PROC_SET_DIS_NON_VID,
	PROC_SET_INGRESS,
	PROC_SET_INSERT_TAG,
	PROC_SET_REMOVE_TAG,
	PROC_SET_DOUBLE_TAG,
	PROC_SET_DROP_TAG,
	PROC_SET_REPLACE_PRIO,

	PROC_SET_RX,
	PROC_SET_TX,
	PROC_SET_LEARN,

	PROC_ENABLE_PRIO_QUEUE,
	PROC_SET_TX_P0_CTRL,
	PROC_SET_TX_P1_CTRL,
	PROC_SET_TX_P2_CTRL,
	PROC_SET_TX_P3_CTRL,
	PROC_SET_TX_P0_RATIO,
	PROC_SET_TX_P1_RATIO,
	PROC_SET_TX_P2_RATIO,
	PROC_SET_TX_P3_RATIO,

	PROC_ENABLE_PRIO_RATE,
	PROC_SET_RX_LIMIT,
	PROC_SET_CNT_IFG,
	PROC_SET_CNT_PRE,
	PROC_SET_RX_P0_RATE,
	PROC_SET_RX_P1_RATE,
	PROC_SET_RX_P2_RATE,
	PROC_SET_RX_P3_RATE,
	PROC_SET_TX_P0_RATE,
	PROC_SET_TX_P1_RATE,
	PROC_SET_TX_P2_RATE,
	PROC_SET_TX_P3_RATE,

	PROC_SET_MIRROR_PORT,
	PROC_SET_MIRROR_RX,
	PROC_SET_MIRROR_TX,

	PROC_SET_BACK_PRESSURE,
	PROC_SET_FORCE_FLOW_CTRL,

	PROC_SET_UNKNOWN_DEF_PORT,
	PROC_SET_FORWARD_INVALID_VID,

	PROC_GET_PORT_DUPLEX,
	PROC_GET_PORT_SPEED,
	PROC_SET_LINK_MD,

	PROC_SET_PORT_MAC_ADDR,
	PROC_SET_SRC_FILTER_0,
	PROC_SET_SRC_FILTER_1,
};

enum {
	PROC_SET_STATIC_FID,
	PROC_SET_STATIC_USE_FID,
	PROC_SET_STATIC_OVERRIDE,
	PROC_SET_STATIC_VALID,
	PROC_SET_STATIC_PORTS,
	PROC_SET_STATIC_MAC_ADDR,
};

enum {
	PROC_SET_VLAN_VALID,
	PROC_SET_VLAN_MEMBER,
	PROC_SET_VLAN_FID,
	PROC_SET_VLAN_VID,
};

/* -------------------------------------------------------------------------- */

/*
#define STATIC_MAC_TABLE_ADDR		00-0000FFFF-FFFFFFFF
#define STATIC_MAC_TABLE_FWD_PORTS	00-00070000-00000000
#define STATIC_MAC_TABLE_VALID		00-00080000-00000000
#define STATIC_MAC_TABLE_OVERRIDE	00-00100000-00000000
#define STATIC_MAC_TABLE_USE_FID	00-00200000-00000000
#define STATIC_MAC_TABLE_FID		00-03C00000-00000000
*/

#define STATIC_MAC_TABLE_ADDR		0x0000FFFF
#define STATIC_MAC_TABLE_FWD_PORTS	0x00070000
#define STATIC_MAC_TABLE_VALID		0x00080000
#define STATIC_MAC_TABLE_OVERRIDE	0x00100000
#define STATIC_MAC_TABLE_USE_FID	0x00200000
#define STATIC_MAC_TABLE_FID		0x03C00000

#define STATIC_MAC_FWD_PORTS_SHIFT	16
#define STATIC_MAC_FID_SHIFT		22

/*
#define VLAN_TABLE_VID			00-00000000-00000FFF
#define VLAN_TABLE_FID			00-00000000-0000F000
#define VLAN_TABLE_MEMBERSHIP		00-00000000-00070000
#define VLAN_TABLE_VALID		00-00000000-00080000
*/

#define VLAN_TABLE_VID			0x00000FFF
#define VLAN_TABLE_FID			0x0000F000
#define VLAN_TABLE_MEMBERSHIP		0x00070000
#define VLAN_TABLE_VALID		0x00080000

#define VLAN_TABLE_FID_SHIFT		12
#define VLAN_TABLE_MEMBERSHIP_SHIFT	16

/*
#define DYNAMIC_MAC_TABLE_ADDR		00-0000FFFF-FFFFFFFF
#define DYNAMIC_MAC_TABLE_FID		00-000F0000-00000000
#define DYNAMIC_MAC_TABLE_SRC_PORT	00-00300000-00000000
#define DYNAMIC_MAC_TABLE_TIMESTAMP	00-00C00000-00000000
#define DYNAMIC_MAC_TABLE_ENTRIES	03-FF000000-00000000
#define DYNAMIC_MAC_TABLE_MAC_EMPTY	04-00000000-00000000
#define DYNAMIC_MAC_TABLE_RESERVED	78-00000000-00000000
#define DYNAMIC_MAC_TABLE_NOT_READY	80-00000000-00000000
*/

#define DYNAMIC_MAC_TABLE_ADDR		0x0000FFFF
#define DYNAMIC_MAC_TABLE_FID		0x000F0000
#define DYNAMIC_MAC_TABLE_SRC_PORT	0x00300000
#define DYNAMIC_MAC_TABLE_TIMESTAMP	0x00C00000
#define DYNAMIC_MAC_TABLE_ENTRIES	0xFF000000

#define DYNAMIC_MAC_TABLE_ENTRIES_H	0x03
#define DYNAMIC_MAC_TABLE_MAC_EMPTY	0x04
#define DYNAMIC_MAC_TABLE_RESERVED	0x78
#define DYNAMIC_MAC_TABLE_NOT_READY	0x80

#define DYNAMIC_MAC_FID_SHIFT		16
#define DYNAMIC_MAC_SRC_PORT_SHIFT	20
#define DYNAMIC_MAC_TIMESTAMP_SHIFT	22
#define DYNAMIC_MAC_ENTRIES_SHIFT	24
#define DYNAMIC_MAC_ENTRIES_H_SHIFT	8

/*
#define MIB_COUNTER_VALUE		00-00000000-3FFFFFFF
#define MIB_COUNTER_VALID		00-00000000-40000000
#define MIB_COUNTER_OVERFLOW		00-00000000-80000000
*/

#ifndef MIB_COUNTER_OVERFLOW
#define MIB_COUNTER_OVERFLOW		(1 << 31)
#define MIB_COUNTER_VALID		(1 << 30)
#define MIB_COUNTER_VALUE		0x3FFFFFFF
#endif

#define KS_MIB_PACKET_DROPPED_TX_0	0x100
#define KS_MIB_PACKET_DROPPED_TX_1	0x101
#define KS_MIB_PACKET_DROPPED_TX	0x102
#define KS_MIB_PACKET_DROPPED_RX_0	0x103
#define KS_MIB_PACKET_DROPPED_RX_1	0x104
#define KS_MIB_PACKET_DROPPED_RX	0x105

#define MIB_PACKET_DROPPED		0x0000FFFF

/* -------------------------------------------------------------------------- */

/* Switch functions */

#define HW_DELAY(sw, reg)			\
	do {					\
		u16 dummy;			\
		dummy = SW_R(sw, reg);		\
	} while (0)

/**
 * sw_r_table - read 4 bytes of data from switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @data:	Buffer to store the read data.
 *
 * This routine reads 4 bytes of data from the table of the switch.
 * Hardware is locked to minimize corruption of read data.
 */
static void sw_r_table(struct ksz_sw *sw, int table, u16 addr, u32 *data)
{
	u16 ctrl_addr;
	int locked = sw->locked;

	ctrl_addr = IND_ACC_TABLE(table | TABLE_READ) | addr;

	if (!locked)
		SW_LOCK(sw);
	SW_W16(sw, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(sw, REG_IND_CTRL_0);
	*data = SW_R32(sw, REG_IND_DATA_LO);
	if (!locked)
		SW_UNLOCK(sw);
}

/**
 * sw_w_table - write 4 bytes of data to the switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @data:	Data to be written.
 *
 * This routine writes 4 bytes of data to the table of the switch.
 * Hardware is locked to minimize corruption of written data.
 */
static void sw_w_table(struct ksz_sw *sw, int table, u16 addr, u32 data)
{
	u16 ctrl_addr;
	int locked = sw->locked;

	ctrl_addr = IND_ACC_TABLE(table) | addr;

	if (!locked)
		SW_LOCK(sw);
	SW_W32(sw, REG_IND_DATA_LO, data);
	SW_W16(sw, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(sw, REG_IND_CTRL_0);
	if (!locked)
		SW_UNLOCK(sw);
}

/**
 * sw_r_table_64 - read 8 bytes of data from the switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @data_hi:	Buffer to store the high part of read data (bit63 ~ bit32).
 * @data_lo:	Buffer to store the low part of read data (bit31 ~ bit0).
 *
 * This routine reads 8 bytes of data from the table of the switch.
 * Hardware is locked to minimize corruption of read data.
 */
static void sw_r_table_64(struct ksz_sw *sw, int table, u16 addr, u32 *data_hi,
	u32 *data_lo)
{
	u16 ctrl_addr;
	int locked = sw->locked;

	ctrl_addr = IND_ACC_TABLE(table | TABLE_READ) | addr;

	if (!locked)
		SW_LOCK(sw);
	SW_W16(sw, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(sw, REG_IND_CTRL_0);
	*data_hi = SW_R32(sw, REG_IND_DATA_HI);
	*data_lo = SW_R32(sw, REG_IND_DATA_LO);
	if (!locked)
		SW_UNLOCK(sw);
}

/**
 * sw_w_table_64 - write 8 bytes of data to the switch table
 * @sw:		The switch instance.
 * @table:	The table selector.
 * @addr:	The address of the table entry.
 * @data_hi:	The high part of data to be written (bit63 ~ bit32).
 * @data_lo:	The low part of data to be written (bit31 ~ bit0).
 *
 * This routine writes 8 bytes of data to the table of the switch.
 * Hardware is locked to minimize corruption of written data.
 */
static void sw_w_table_64(struct ksz_sw *sw, int table, u16 addr, u32 data_hi,
	u32 data_lo)
{
	u16 ctrl_addr;
	int locked = sw->locked;

	ctrl_addr = IND_ACC_TABLE(table) | addr;

	if (!locked)
		SW_LOCK(sw);
	SW_W32(sw, REG_IND_DATA_HI, data_hi);
	SW_W32(sw, REG_IND_DATA_LO, data_lo);
	SW_W16(sw, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(sw, REG_IND_CTRL_0);
	if (!locked)
		SW_UNLOCK(sw);
}

static inline int valid_dyn_entry(struct ksz_sw *sw, u8 *data)
{
	int timeout = 100;

	do {
		*data = SW_R8(sw, REG_IND_DATA_CHECK);
		timeout--;
	} while ((*data & DYNAMIC_MAC_TABLE_NOT_READY) && timeout);

	/* Entry is not ready for accessing. */
	if (*data & DYNAMIC_MAC_TABLE_NOT_READY)
		return 1;

	/* Entry is ready for accessing. */
	else {
		/* There is no valid entry in the table. */
		if (*data & DYNAMIC_MAC_TABLE_MAC_EMPTY)
			return 2;
	}
	return 0;
}

/**
 * sw_r_dyn_mac_table - read from dynamic MAC table
 * @sw:		The switch instance.
 * @addr:	The address of the table entry.
 * @mac_addr:	Buffer to store the MAC address.
 * @fid:	Buffer to store the FID.
 * @src_port:	Buffer to store the source port number.
 * @timestamp:	Buffer to store the timestamp.
 * @entries:	Buffer to store the number of entries.  If this is zero, the
 *		table is empty and so this function should not be called again
 *		until later.
 *
 * This function reads an entry of the dynamic MAC table of the switch.
 * Hardware is locked to minimize corruption of read data.
 *
 * Return 0 if the entry is successfully read; otherwise -1.
 */
static int sw_r_dyn_mac_table(struct ksz_sw *sw, u16 addr, u8 *mac_addr,
	u8 *fid, u8 *src_port, u8 *timestamp, u16 *entries)
{
	u32 data_hi;
	u32 data_lo;
	u16 ctrl_addr;
	int rc;
	u8 data;
	int locked = sw->locked;

	ctrl_addr = IND_ACC_TABLE(TABLE_DYNAMIC_MAC | TABLE_READ) | addr;

	if (!locked)
		SW_LOCK(sw);
	SW_W16(sw, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(sw, REG_IND_CTRL_0);

	rc = valid_dyn_entry(sw, &data);
	if (1 == rc) {
		if (0 == addr)
			*entries = 0;
	} else if (2 == rc)
		*entries = 0;
	/* At least one valid entry in the table. */
	else {
		data_hi = SW_R32(sw, REG_IND_DATA_HI);

		/* Check out how many valid entry in the table. */
		*entries = (u16)(((((u16)
			data & DYNAMIC_MAC_TABLE_ENTRIES_H) <<
			DYNAMIC_MAC_ENTRIES_H_SHIFT) |
			(((data_hi & DYNAMIC_MAC_TABLE_ENTRIES) >>
			DYNAMIC_MAC_ENTRIES_SHIFT))) + 1);

		*fid = (u8)((data_hi & DYNAMIC_MAC_TABLE_FID) >>
			DYNAMIC_MAC_FID_SHIFT);
		*src_port = (u8)((data_hi & DYNAMIC_MAC_TABLE_SRC_PORT) >>
			DYNAMIC_MAC_SRC_PORT_SHIFT);
		*timestamp = (u8)((
			data_hi & DYNAMIC_MAC_TABLE_TIMESTAMP) >>
			DYNAMIC_MAC_TIMESTAMP_SHIFT);

		data_lo = SW_R32(sw, REG_IND_DATA_LO);

		mac_addr[5] = (u8) data_lo;
		mac_addr[4] = (u8)(data_lo >> 8);
		mac_addr[3] = (u8)(data_lo >> 16);
		mac_addr[2] = (u8)(data_lo >> 24);

		mac_addr[1] = (u8) data_hi;
		mac_addr[0] = (u8)(data_hi >> 8);
		rc = 0;
	}
	if (!locked)
		SW_UNLOCK(sw);

	return rc;
}

static void sw_d_dyn_mac_table(struct ksz_sw *sw)
{
	u16 entries = 0;
	u16 i;
	u8 mac_addr[MAC_ADDR_LEN];
	u8 ports = 0;
	u8 timestamp = 0;
	u8 fid = 0;

	memset(mac_addr, 0, MAC_ADDR_LEN);
	i = 0;
	do {
		if (!sw_r_dyn_mac_table(sw, i, mac_addr, &fid, &ports,
				&timestamp, &entries)) {
			printk(KERN_INFO
				"%02X:%02X:%02X:%02X:%02X:%02X %x %x %x %03x\n",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5],
				fid, ports, timestamp, entries);
		}
		i++;
	} while (i < entries);
}

/**
 * sw_r_sta_mac_table - read from static MAC table
 * @sw:		The switch instance.
 * @addr:	The address of the table entry.
 * @mac_addr:	Buffer to store the MAC address.
 * @ports:	Buffer to store the port members.
 * @override:	Buffer to store the override flag.
 * @use_fid:	Buffer to store the use FID flag which indicates the FID is
 *		valid.
 * @fid:	Buffer to store the FID.
 *
 * This function reads an entry of the static MAC table of the switch.  It
 * calls sw_r_table_64() to get the data.
 *
 * Return 0 if the entry is valid; otherwise -1.
 */
static int sw_r_sta_mac_table(struct ksz_sw *sw, u16 addr, u8 *mac_addr,
	u8 *ports, int *override, int *use_fid, u8 *fid)
{
	u32 data_hi;
	u32 data_lo;

	sw_r_table_64(sw, TABLE_STATIC_MAC, addr, &data_hi, &data_lo);
	if (data_hi & (STATIC_MAC_TABLE_VALID | STATIC_MAC_TABLE_OVERRIDE)) {
		mac_addr[5] = (u8) data_lo;
		mac_addr[4] = (u8)(data_lo >> 8);
		mac_addr[3] = (u8)(data_lo >> 16);
		mac_addr[2] = (u8)(data_lo >> 24);
		mac_addr[1] = (u8) data_hi;
		mac_addr[0] = (u8)(data_hi >> 8);
		*ports = (u8)((data_hi & STATIC_MAC_TABLE_FWD_PORTS) >>
			STATIC_MAC_FWD_PORTS_SHIFT);
		*override = (data_hi & STATIC_MAC_TABLE_OVERRIDE) ? 1 : 0;
		*use_fid = (data_hi & STATIC_MAC_TABLE_USE_FID) ? 1 : 0;
		*fid = (u8)((data_hi & STATIC_MAC_TABLE_FID) >>
			STATIC_MAC_FID_SHIFT);
		return 0;
	}
	return -1;
}

/**
 * sw_w_sta_mac_table - write to the static MAC table
 * @sw:		The switch instance.
 * @addr:	The address of the table entry.
 * @mac_addr:	The MAC address.
 * @ports:	The port members.
 * @override:	The flag to override the port receive/transmit settings.
 * @valid:	The flag to indicate entry is valid.
 * @use_fid:	The flag to indicate the FID is valid.
 * @fid:	The FID value.
 *
 * This routine writes an entry of the static MAC table of the switch.  It
 * calls sw_w_table_64() to write the data.
 */
static void sw_w_sta_mac_table(struct ksz_sw *sw, u16 addr, u8 *mac_addr,
	u8 ports, int override, int valid, int use_fid, u8 fid)
{
	u32 data_hi;
	u32 data_lo;

	data_lo = ((u32) mac_addr[2] << 24) |
		((u32) mac_addr[3] << 16) |
		((u32) mac_addr[4] << 8) | mac_addr[5];
	data_hi = ((u32) mac_addr[0] << 8) | mac_addr[1];
	data_hi |= (u32) ports << STATIC_MAC_FWD_PORTS_SHIFT;

	if (override)
		data_hi |= STATIC_MAC_TABLE_OVERRIDE;
	if (use_fid) {
		data_hi |= STATIC_MAC_TABLE_USE_FID;
		data_hi |= (u32) fid << STATIC_MAC_FID_SHIFT;
	}
	if (valid)
		data_hi |= STATIC_MAC_TABLE_VALID;
	else
		data_hi &= ~STATIC_MAC_TABLE_OVERRIDE;

	sw_w_table_64(sw, TABLE_STATIC_MAC, addr, data_hi, data_lo);
}

static void sw_d_sta_mac_table(struct ksz_sw *sw)
{
	u16 i;
	u8 mac_addr[MAC_ADDR_LEN];
	u8 ports;
	int override;
	int use_fid;
	u8 fid;

	i = 0;
	do {
		if (!sw_r_sta_mac_table(sw, i, mac_addr, &ports, &override,
				&use_fid, &fid)) {
			printk(KERN_INFO
				"%d: %02X:%02X:%02X:%02X:%02X:%02X "
				"%x %u %u:%x\n",
				i, mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5],
				ports, override, use_fid, fid);
		}
		i++;
	} while (i < STATIC_MAC_TABLE_ENTRIES);
}

/* -------------------------------------------------------------------------- */

/**
 * sw_r_vlan_table - read from the VLAN table
 * @sw:		The switch instance.
 * @addr:	The address of the table entry.
 * @vid:	Buffer to store the VID.
 * @fid:	Buffer to store the VID.
 * @member:	Buffer to store the port membership.
 *
 * This function reads an entry of the VLAN table of the switch.  It calls
 * sw_r_table() to get the data.
 *
 * Return 0 if the entry is valid; otherwise -1.
 */
static int sw_r_vlan_table(struct ksz_sw *sw, u16 addr, u16 *vid, u8 *fid,
	u8 *member)
{
	u32 data;

	sw_r_table(sw, TABLE_VLAN, addr, &data);
	if (data & VLAN_TABLE_VALID) {
		*vid = (u16)(data & VLAN_TABLE_VID);
		*fid = (u8)((data & VLAN_TABLE_FID) >> VLAN_TABLE_FID_SHIFT);
		*member = (u8)((data & VLAN_TABLE_MEMBERSHIP) >>
			VLAN_TABLE_MEMBERSHIP_SHIFT);
		return 0;
	}
	return -1;
}

/**
 * sw_w_vlan_table - write to the VLAN table
 * @sw:		The switch instance.
 * @addr:	The address of the table entry.
 * @vid:	The VID value.
 * @fid:	The FID value.
 * @member:	The port membership.
 * @valid:	The flag to indicate entry is valid.
 *
 * This routine writes an entry of the VLAN table of the switch.  It calls
 * sw_w_table() to write the data.
 */
static void sw_w_vlan_table(struct ksz_sw *sw, u16 addr, u16 vid, u8 fid,
	u8 member, int valid)
{
	u32 data;
	int entry;
	struct ksz_sw_info *info = sw->info;

	data = vid;
	data |= (u32) fid << VLAN_TABLE_FID_SHIFT;
	data |= (u32) member << VLAN_TABLE_MEMBERSHIP_SHIFT;
	if (valid)
		data |= VLAN_TABLE_VALID;

	sw_w_table(sw, TABLE_VLAN, addr, data);

	entry = addr;
	if (entry >= VLAN_TABLE_ENTRIES)
		return;
	info->vlan_table[entry].vid = vid;
	info->vlan_table[entry].fid = fid;
	info->vlan_table[entry].member = member;
	info->vlan_table[entry].valid = valid;
}

static void sw_d_vlan_table(struct ksz_sw *sw)
{
	u16 i;
	u16 vid;
	u8 fid;
	u8 member;
	struct ksz_sw_info *info = sw->info;

	i = 0;
	do {
		if (!sw_r_vlan_table(sw, i, &vid, &fid, &member)) {
			info->vlan_table[i].vid = vid;
			info->vlan_table[i].fid = fid;
			info->vlan_table[i].member = member;
			info->vlan_table[i].valid = 1;
			printk(KERN_INFO
				"%x: 0x%03x %x %x\n", i, vid, fid, member);
		} else
			info->vlan_table[i].valid = 0;
		i++;
	} while (i < VLAN_TABLE_ENTRIES);
}

/* -------------------------------------------------------------------------- */

/**
 * port_r_mib_cnt - read MIB counter
 * @sw:		The switch instance.
 * @port:	The port index.
 * @addr:	The address of the counter.
 * @cnt:	Buffer to store the counter.
 *
 * This routine reads a MIB counter of the port.
 * Hardware is locked to minimize corruption of read data.
 */
static void port_r_mib_cnt(struct ksz_sw *sw, int port, u16 addr, u64 *cnt)
{
	u32 data;
	u16 ctrl_addr;
	int timeout;

	ctrl_addr = addr + SWITCH_COUNTER_NUM * port;

	SW_LOCK(sw);

	ctrl_addr |= IND_ACC_TABLE(TABLE_MIB | TABLE_READ);
	SW_W16(sw, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(sw, REG_IND_CTRL_0);

	for (timeout = 1; timeout > 0; timeout--) {
		data = SW_R32(sw, REG_IND_DATA_LO);

		if (data & MIB_COUNTER_VALID) {
			if (data & MIB_COUNTER_OVERFLOW)
				*cnt += MIB_COUNTER_VALUE + 1;
			*cnt += data & MIB_COUNTER_VALUE;
			break;
		}
	}

	SW_UNLOCK(sw);
}  /* port_r_mib_cnt */

/**
 * port_r_mib_pkt - read dropped packet counts
 * @sw:		The switch instance.
 * @port:	The port index.
 * @cnt:	Buffer to store the receive and transmit dropped packet counts.
 *
 * This routine reads the dropped packet counts of the port.
 * Hardware is locked to minimize corruption of read data.
 */
static void port_r_mib_pkt(struct ksz_sw *sw, int port, u32 *last, u64 *cnt)
{
	u32 cur;
	u32 data;
	u16 ctrl_addr;
	int index;

	index = KS_MIB_PACKET_DROPPED_RX_0 + port;
	do {
		ctrl_addr = index;

		SW_LOCK(sw);

		ctrl_addr |= IND_ACC_TABLE(TABLE_MIB | TABLE_READ);
		SW_W16(sw, REG_IND_CTRL_0, ctrl_addr);
		HW_DELAY(sw, REG_IND_CTRL_0);
		data = SW_R32(sw, REG_IND_DATA_LO);

		SW_UNLOCK(sw);

		data &= MIB_PACKET_DROPPED;
		cur = *last;
		if (data != cur) {
			*last = data;
			if (data < cur)
				data += MIB_PACKET_DROPPED + 1;
			data -= cur;
			*cnt += data;
		}
		++last;
		++cnt;
		index -= KS_MIB_PACKET_DROPPED_TX -
			KS_MIB_PACKET_DROPPED_TX_0 + 1;
	} while (index >= KS_MIB_PACKET_DROPPED_TX_0 + port);
}  /* port_r_mib_pkt */

/**
 * port_r_cnt - read MIB counters periodically
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine is used to read the counters of the port periodically to avoid
 * counter overflow.  The hardware should be acquired first before calling this
 * routine.
 *
 * Return non-zero when not all counters not read.
 */
static inline int port_r_cnt(struct ksz_sw *sw, int port)
{
	struct ksz_port_mib *mib = &sw->port_mib[port];

	if (mib->mib_start < SWITCH_COUNTER_NUM)
		while (mib->cnt_ptr < SWITCH_COUNTER_NUM) {
			if (exit_mib_read(sw))
				return mib->cnt_ptr;
			port_r_mib_cnt(sw, port, mib->cnt_ptr,
				&mib->counter[mib->cnt_ptr]);
			++mib->cnt_ptr;
		}
	if (sw->mib_cnt > SWITCH_COUNTER_NUM)
		port_r_mib_pkt(sw, port, mib->dropped,
			&mib->counter[SWITCH_COUNTER_NUM]);
	mib->cnt_ptr = 0;
	return 0;
}  /* port_r_cnt */

/**
 * port_init_cnt - initialize MIB counter values
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine is used to initialize all counters to zero if the hardware
 * cannot do it after reset.
 */
static inline void port_init_cnt(struct ksz_sw *sw, int port)
{
	struct ksz_port_mib *mib = &sw->port_mib[port];

	mib->cnt_ptr = 0;
	if (mib->mib_start < SWITCH_COUNTER_NUM)
		do {
			port_r_mib_cnt(sw, port, mib->cnt_ptr,
				&mib->counter[mib->cnt_ptr]);
			++mib->cnt_ptr;
		} while (mib->cnt_ptr < SWITCH_COUNTER_NUM);
	if (sw->mib_cnt > SWITCH_COUNTER_NUM)
		port_r_mib_pkt(sw, port, mib->dropped,
			&mib->counter[SWITCH_COUNTER_NUM]);
	memset((void *) mib->counter, 0, sizeof(u64) *
		TOTAL_SWITCH_COUNTER_NUM);
	mib->cnt_ptr = 0;
}  /* port_init_cnt */

/* -------------------------------------------------------------------------- */

/*
 * Port functions
 */

/**
 * port_chk - check port register bits
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the port register.
 * @bits:	The data bits to check.
 *
 * This function checks whether the specified bits of the port register are set
 * or not.
 *
 * Return 0 if the bits are not set.
 */
static int port_chk(struct ksz_sw *sw, int port, int offset, SW_D bits)
{
	u32 addr;
	SW_D data;

	PORT_CTRL_ADDR(port, addr);
	addr += offset;
	data = SW_R(sw, addr);
	return (data & bits) == bits;
}

/**
 * port_cfg - set port register bits
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the port register.
 * @bits:	The data bits to set.
 * @set:	The flag indicating whether the bits are to be set or not.
 *
 * This routine sets or resets the specified bits of the port register.
 */
static void port_cfg(struct ksz_sw *sw, int port, int offset, SW_D bits,
	int set)
{
	u32 addr;
	SW_D data;

	PORT_CTRL_ADDR(port, addr);
	addr += offset;
	data = SW_R(sw, addr);
	if (set)
		data |= bits;
	else
		data &= ~bits;
	SW_W(sw, addr, data);
}

/**
 * port_chk_shift - check port bit
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the register.
 * @shift:	Number of bits to shift.
 *
 * This function checks whether the specified port is set in the register or
 * not.
 *
 * Return 0 if the port is not set.
 */
static int port_chk_shift(struct ksz_sw *sw, int port, u32 addr, int shift)
{
	SW_D data;
	SW_D bit = 1 << port;

	data = SW_R(sw, addr);
	data >>= shift;
	return (data & bit) == bit;
}

/**
 * port_cfg_shift - set port bit
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the register.
 * @shift:	Number of bits to shift.
 * @set:	The flag indicating whether the port is to be set or not.
 *
 * This routine sets or resets the specified port in the register.
 */
static void port_cfg_shift(struct ksz_sw *sw, int port, u32 addr, int shift,
	int set)
{
	SW_D data;
	SW_D bits = 1 << port;

	data = SW_R(sw, addr);
	bits <<= shift;
	if (set)
		data |= bits;
	else
		data &= ~bits;
	SW_W(sw, addr, data);
}

/**
 * port_r8 - read byte from port register
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the port register.
 * @data:	Buffer to store the data.
 *
 * This routine reads a byte from the port register.
 */
static void port_r8(struct ksz_sw *sw, int port, int offset, u8 *data)
{
	u32 addr;

	PORT_CTRL_ADDR(port, addr);
	addr += offset;
	*data = SW_R8(sw, addr);
}

/**
 * port_w8 - write byte to port register
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the port register.
 * @data:	Data to write.
 *
 * This routine writes a byte to the port register.
 */
static void port_w8(struct ksz_sw *sw, int port, int offset, u8 data)
{
	u32 addr;

	PORT_CTRL_ADDR(port, addr);
	addr += offset;
	SW_W8(sw, addr, data);
}

/**
 * port_r16 - read word from port register.
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the port register.
 * @data:	Buffer to store the data.
 *
 * This routine reads a word from the port register.
 */
static void port_r16(struct ksz_sw *sw, int port, int offset, u16 *data)
{
	u32 addr;

	PORT_CTRL_ADDR(port, addr);
	addr += offset;
	*data = SW_R16(sw, addr);
}

/**
 * port_w16 - write word to port register.
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the port register.
 * @data:	Data to write.
 *
 * This routine writes a word to the port register.
 */
static void port_w16(struct ksz_sw *sw, int port, int offset, u16 data)
{
	u32 addr;

	PORT_CTRL_ADDR(port, addr);
	addr += offset;
	SW_W16(sw, addr, data);
}

/**
 * sw_chk - check switch register bits
 * @sw:		The switch instance.
 * @addr:	The address of the switch register.
 * @bits:	The data bits to check.
 *
 * This function checks whether the specified bits of the switch register are
 * set or not.
 *
 * Return 0 if the bits are not set.
 */
static int sw_chk(struct ksz_sw *sw, u32 addr, SW_D bits)
{
	SW_D data;

	data = SW_R(sw, addr);
	return (data & bits) == bits;
}

/**
 * sw_cfg - set switch register bits
 * @sw:		The switch instance.
 * @addr:	The address of the switch register.
 * @bits:	The data bits to set.
 * @set:	The flag indicating whether the bits are to be set or not.
 *
 * This function sets or resets the specified bits of the switch register.
 */
static void sw_cfg(struct ksz_sw *sw, u32 addr, SW_D bits, int set)
{
	SW_D data;

	data = SW_R(sw, addr);
	if (set)
		data |= bits;
	else
		data &= ~bits;
	SW_W(sw, addr, data);
}

#ifdef PORT_OUT_RATE_ADDR
/**
 * port_out_rate_r8 - read byte from port register
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the port register.
 * @data:	Buffer to store the data.
 *
 * This routine reads a byte from the port register.
 */
static void port_out_rate_r8(struct ksz_sw *sw, int port, int offset, u8 *data)
{
	u32 addr;

	PORT_OUT_RATE_ADDR(port, addr);
	addr += offset;
	*data = SW_R8(sw, addr);
}

/**
 * port_out_rate_w8 - write byte to port register
 * @sw:		The switch instance.
 * @port:	The port index.
 * @offset:	The offset of the port register.
 * @data:	Data to write.
 *
 * This routine writes a byte to the port register.
 */
static void port_out_rate_w8(struct ksz_sw *sw, int port, int offset, u8 data)
{
	u32 addr;

	PORT_OUT_RATE_ADDR(port, addr);
	addr += offset;
	SW_W8(sw, addr, data);
}
#endif

/* -------------------------------------------------------------------------- */

/* Bandwidth */

static inline void port_cfg_broad_storm(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_BCAST_STORM_CTRL, PORT_BROADCAST_STORM, set);
}

static inline int port_chk_broad_storm(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_BCAST_STORM_CTRL, PORT_BROADCAST_STORM);
}

/* Driver set switch broadcast storm protection at 10% rate. */
#define BROADCAST_STORM_PROTECTION_RATE	10

/* 148,800 frames * 67 ms / 100 */
#define BROADCAST_STORM_VALUE		9969

/**
 * sw_cfg_broad_storm - configure broadcast storm threshold
 * @sw:		The switch instance.
 * @percent:	Broadcast storm threshold in percent of transmit rate.
 *
 * This routine configures the broadcast storm threshold of the switch.
 */
static void sw_cfg_broad_storm(struct ksz_sw *sw, u8 percent)
{
	u16 data;
	u32 value = ((u32) BROADCAST_STORM_VALUE * (u32) percent / 100);

	if (value > BROADCAST_STORM_RATE)
		value = BROADCAST_STORM_RATE;

	data = SW_R16(sw, S_REPLACE_VID_CTRL);
#if (SW_SIZE == (2))
	data = ntohs(data);
#endif
	data &= ~BROADCAST_STORM_RATE;
	data |= value;
#if (SW_SIZE == (2))
	data = ntohs(data);
#endif
	SW_W16(sw, S_REPLACE_VID_CTRL, data);
}

/**
 * sw_get_board_storm - get broadcast storm threshold
 * @sw:		The switch instance.
 * @percent:	Buffer to store the broadcast storm threshold percentage.
 *
 * This routine retrieves the broadcast storm threshold of the switch.
 */
static void sw_get_broad_storm(struct ksz_sw *sw, u8 *percent)
{
	int num;
	u16 data;

	data = SW_R16(sw, S_REPLACE_VID_CTRL);
#if (SW_SIZE == (2))
	data = ntohs(data);
#endif
	num = (data & BROADCAST_STORM_RATE);
	num = (num * 100 + BROADCAST_STORM_VALUE / 2) / BROADCAST_STORM_VALUE;
	*percent = (u8) num;
}

/**
 * sw_dis_broad_storm - disable broadcast storm protection
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine disables the broadcast storm limit function of the switch.
 */
static void sw_dis_broad_storm(struct ksz_sw *sw, int port)
{
	port_cfg_broad_storm(sw, port, 0);
}

/**
 * sw_ena_broad_storm - enable broadcast storm protection
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine enables the broadcast storm limit function of the switch.
 */
static void sw_ena_broad_storm(struct ksz_sw *sw, int port)
{
	sw_cfg_broad_storm(sw, sw->info->broad_per);
	port_cfg_broad_storm(sw, port, 1);
}

/**
 * sw_init_broad_storm - initialize broadcast storm
 * @sw:		The switch instance.
 *
 * This routine initializes the broadcast storm limit function of the switch.
 */
static void sw_init_broad_storm(struct ksz_sw *sw)
{
	u8 percent;

	sw_get_broad_storm(sw, &percent);
	sw->info->broad_per = percent;
}

/**
 * hw_cfg_broad_storm - configure broadcast storm
 * @sw:		The switch instance.
 * @percent:	Broadcast storm threshold in percent of transmit rate.
 *
 * This routine configures the broadcast storm threshold of the switch.
 * It is called by user functions.  The hardware should be acquired first.
 */
static void hw_cfg_broad_storm(struct ksz_sw *sw, u8 percent)
{
	if (percent > 100)
		percent = 100;

	sw_cfg_broad_storm(sw, percent);
	sw_init_broad_storm(sw);
}

/**
 * sw_setup_broad_storm - setup broadcast storm
 * @sw:		The switch instance.
 *
 * This routine setup the broadcast storm limit function of the switch.
 */
static void sw_setup_broad_storm(struct ksz_sw *sw)
{
	int port;

	/* Enable switch broadcast storm protection at 10% percent rate. */
	hw_cfg_broad_storm(sw, BROADCAST_STORM_PROTECTION_RATE);
	for (port = 0; port < SWITCH_PORT_NUM; port++)
		sw_ena_broad_storm(sw, port);
	sw_cfg(sw, REG_SWITCH_CTRL_2, MULTICAST_STORM_DISABLE, 1);
}

/* -------------------------------------------------------------------------- */

/* Rate Control */

static inline int get_rate_ctrl_offset(int port, int prio)
{
	int offset;

#if (SW_SIZE == (2))
	offset = REG_PORT1_TXQ_RATE_CTRL1 + port * 4;
	offset += ((3 - prio) / 2) * 2;
	offset++;
	offset -= (prio & 1);
#else
	offset = REG_PORT1_TXQ3_RATE_CTRL + port * 4;
	offset += (3 - prio);
#endif
	return offset;
}

/**
 * hw_cfg_rate_ctrl - configure port rate control
 * @sw:		The switch instance.
 * @port:	The port index.
 * @prio:	The priority index to configure.
 * @ctrl:	The flag indicating whether the rate control bit is set or not.
 *
 * This routine configures the priority rate control of the port.
 */
static void hw_cfg_rate_ctrl(struct ksz_sw *sw, int port, int prio, int ctrl)
{
	int offset;
	u8 data;
	u8 saved;

	offset = get_rate_ctrl_offset(port, prio);

	data = SW_R8(sw, offset);
	saved = data;
	data &= ~RATE_CTRL_ENABLE;
	if (ctrl)
		data |= RATE_CTRL_ENABLE;
	if (data != saved)
		SW_W8(sw, offset, data);
	sw->info->port_cfg[port].rate_ctrl[prio] = data;
}

#ifdef RATE_RATIO_MASK
/**
 * hw_cfg_rate_ratio - configure port rate ratio
 * @sw:		The switch instance.
 * @port:	The port index.
 * @prio:	The priority index to configure.
 * @ratio:	The rate ratio.
 *
 * This routine configures the priority rate ratio of the port.
 */
static void hw_cfg_rate_ratio(struct ksz_sw *sw, int port, int prio, u8 ratio)
{
	int offset;
	u8 data;
	u8 saved;

	if (ratio >= RATE_CTRL_ENABLE)
		return;

	offset = get_rate_ctrl_offset(port, prio);

	data = SW_R8(sw, offset);
	saved = data;
	data &= RATE_CTRL_ENABLE;
	data |= ratio;
	if (data != saved)
		SW_W8(sw, offset, data);
	sw->info->port_cfg[port].rate_ctrl[prio] = data;
}
#endif

/**
 * hw_get_rate_ctrl - get port rate control
 * @sw:		The switch instance.
 * @port:	The port index.
 * @prio:	The priority index to retrieve.
 *
 * This routine retrieves the priority rate control of the port.
 */
static void hw_get_rate_ctrl(struct ksz_sw *sw, int port, int prio)
{
	int offset;
	u8 data;

	offset = get_rate_ctrl_offset(port, prio);

	data = SW_R8(sw, offset);
	sw->info->port_cfg[port].rate_ctrl[prio] = data;
}

/* -------------------------------------------------------------------------- */

/* Rate Limit */

/**
 * hw_cfg_rate_limit - configure port rate limit modes
 * @sw:		The switch instance.
 * @port:	The port index.
 * @mask:	The mask value.
 * @shift:	The shift position.
 * @mode:	The rate limit mode.
 *
 * This helper routine configures the rate limit modes of the port.
 */
static void hw_cfg_rate_limit(struct ksz_sw *sw, int port, u8 mask, u8 shift,
	u8 mode)
{
	u8 data;
	u8 saved;

	port_r8(sw, port, P_RATE_LIMIT_CTRL, &data);
	saved = data;
	data &= ~(mask << shift);
	data |= mode << shift;
	if (data != saved)
		port_w8(sw, port, P_RATE_LIMIT_CTRL, data);
	sw->info->port_cfg[port].rate_limit = data;
}

/**
 * hw_cfg_cnt_ifg - configure port rate limit count IFG control
 * @sw:		The switch instance.
 * @port:	The port index.
 * @set:	The flag indicating whether the count control is set or not.
 *
 * This routine configures the rate limit count IFG control of the port.
 */
static void hw_cfg_cnt_ifg(struct ksz_sw *sw, int port, int set)
{
	hw_cfg_rate_limit(sw, port, 1, 1, set != 0);
}

/**
 * hw_cfg_cnt_pre - configure port rate limit count preamble control
 * @sw:		The switch instance.
 * @port:	The port index.
 * @set:	The flag indicating whether the count control is set or not.
 *
 * This routine configures the rate limit count preamble control of the port.
 */
static void hw_cfg_cnt_pre(struct ksz_sw *sw, int port, int set)
{
	hw_cfg_rate_limit(sw, port, 1, 0, set != 0);
}

/**
 * hw_cfg_rx_limit - configure port rate limit mode
 * @sw:		The switch instance.
 * @port:	The port index.
 * @mode:	The rate limit mode.
 *
 * This routine configures the rate limit mode of the port.
 */
static void hw_cfg_rx_limit(struct ksz_sw *sw, int port, u8 mode)
{
	if (mode >= 4)
		return;

	hw_cfg_rate_limit(sw, port, 3, 2, mode);
}

/**
 * hw_get_rate_limit - get port rate limit control
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine retrieves the rate limit of the port.
 */
static void hw_get_rate_limit(struct ksz_sw *sw, int port)
{
	u8 data;

	port_r8(sw, port, P_RATE_LIMIT_CTRL, &data);
	sw->info->port_cfg[port].rate_limit = data;
}

/* -------------------------------------------------------------------------- */

static uint get_rate_from_val(u8 val)
{
	uint i;

	if (0 == val)
		i = 0;
	else if (val <= 0x64)
		i = 1000 * val;
	else
		i = 64 * (val - 0x64);
	return i;
}

static int get_rate_to_val(uint rate)
{
	int i;

	if (rate >= 1000) {
		i = (rate + 500) / 1000;
		if (i > 0x64)
			i = 0x64;
	} else if (0 == rate)
		i = 0;
	else {
		i = (rate + 32) / 64;
		if (0 == i)
			i = 1;
		else if (i > 15)
			i = 15;
		i += 0x64;
	}
	return i;
}

/**
 * port_cfg_rate - configure port priority rate
 * @sw:		The switch instance.
 * @port:	The port index.
 * @prio:	The priority index to configure.
 * @offset:	The receive or transmit rate offset.
 * @shift:	The shift position to set the value.
 * @rate:	The rate limit in number of Kbps.
 *
 * This helper routine configures the priority rate of the port.
 */
static void port_cfg_rate(struct ksz_sw *sw, int port, int prio, int offset,
	uint rate)
{
	u8 factor;

	factor = (u8) get_rate_to_val(rate);

#ifndef PORT_OUT_RATE_ADDR
	offset += (prio / 2) * 2;
	offset += (prio & 1);
	port_w8(sw, port, offset, factor);
#else
	if (REG_PORT_1_OUT_RATE_0 == offset)
		port_out_rate_w8(sw, port, REG_PORT_OUT_RATE_0 + prio, factor);
	else
		port_w8(sw, port, REG_PORT_IN_RATE_0 + prio, factor);
#endif
}

/**
 * port_get_rate - get port priority rate
 * @sw:		The switch instance.
 * @port:	The port index.
 * @prio:	The priority index to configure.
 * @offset:	The receive or transmit rate offset.
 * @shift:	The shift position to get the value.
 * @rate:	Buffer to store the data rate in number of Kbps.
 *
 * This helper routine retrieves the priority rate of the port.
 */
static void port_get_rate(struct ksz_sw *sw, int port, int prio, int offset,
	uint *rate)
{
	u8 data;

#ifndef PORT_OUT_RATE_ADDR
	offset += (prio / 2) * 2;
	offset += (prio & 1);
	port_r8(sw, port, offset, &data);
#else
	if (REG_PORT_1_OUT_RATE_0 == offset)
		port_out_rate_r8(sw, port, REG_PORT_OUT_RATE_0 + prio, &data);
	else
		port_r8(sw, port, REG_PORT_IN_RATE_0 + prio, &data);
#endif
	*rate = get_rate_from_val(data);
}

/**
 * hw_cfg_prio_rate - configure port priority
 * @sw:		The switch instance.
 * @port:	The port index.
 * @prio:	The priority index to configure.
 * @rate:	The rate limit in number of Kbps.
 * @offset:	The receive or transmit rate offset.
 * @result:	Buffer to store the data rate in number of Kbps.
 *
 * This helper routine configures the priority rate of the port and retrieves
 * the actual rate number.
 */
static void hw_cfg_prio_rate(struct ksz_sw *sw, int port, int prio, uint rate,
	int offset, uint *result)
{
	port_cfg_rate(sw, port, prio, offset, rate);
	port_get_rate(sw, port, prio, offset, result);
}

/**
 * hw_cfg_rx_prio_rate - configure port receive priority
 * @sw:		The switch instance.
 * @port:	The port index.
 * @prio:	The priority index to configure.
 * @rate:	The rate limit in number of Kbps.
 *
 * This routine configures the receive priority rate of the port.
 * It is called by user functions.  The hardware should be acquired first.
 */
static void hw_cfg_rx_prio_rate(struct ksz_sw *sw, int port, int prio,
	uint rate)
{
	hw_cfg_prio_rate(sw, port, prio, rate,
#ifdef PORT_OUT_RATE_ADDR
		REG_PORT_1_IN_RATE_0,
#else
		REG_PORT_IN_RATE_0,
#endif
		&sw->info->port_cfg[port].rx_rate[prio]);
}

/**
 * hw_cfg_tx_prio_rate - configure port transmit priority
 * @sw:		The switch instance.
 * @port:	The port index.
 * @prio:	The priority index to configure.
 * @rate:	The rate limit in number of Kbps.
 *
 * This routine configures the transmit priority rate of the port.
 * It is called by user functions.  The hardware should be acquired first.
 */
static void hw_cfg_tx_prio_rate(struct ksz_sw *sw, int port, int prio,
	uint rate)
{
	hw_cfg_prio_rate(sw, port, prio, rate,
#ifdef PORT_OUT_RATE_ADDR
		REG_PORT_1_OUT_RATE_0,
#else
		REG_PORT_OUT_RATE_0,
#endif
		&sw->info->port_cfg[port].tx_rate[prio]);
}

/**
 * sw_chk_prio_rate - check switch priority rate
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This function checks whether the priority rate function of the switch is
 * enabled.
 *
 * Return 0 if not enabled.
 */
static int sw_chk_prio_rate(struct ksz_sw *sw, int port)
{
	u32 addr;
	u32 rate_addr;
	u32 in_rate;
	u32 out_rate;

	PORT_CTRL_ADDR(port, addr);
	rate_addr = addr + REG_PORT_IN_RATE_0;
	in_rate = SW_R32(sw, rate_addr);
#ifdef PORT_OUT_RATE_ADDR
	PORT_OUT_RATE_ADDR(port, addr);
#endif
	rate_addr = addr + REG_PORT_OUT_RATE_0;
	out_rate = SW_R32(sw, rate_addr);
	return (in_rate | out_rate) != 0;
}

/**
 * sw_dis_prio_rate - disable switch priority rate
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine disables the priority rate function of the switch.
 */
static void sw_dis_prio_rate(struct ksz_sw *sw, int port)
{
	u32 addr;
	u32 rate_addr;

	PORT_CTRL_ADDR(port, addr);
	rate_addr = addr + REG_PORT_IN_RATE_0;
	SW_W32(sw, rate_addr, 0);
#ifdef PORT_OUT_RATE_ADDR
	PORT_OUT_RATE_ADDR(port, addr);
#endif
	rate_addr = addr + REG_PORT_OUT_RATE_0;
	SW_W32(sw, rate_addr, 0);
}

/**
 * sw_ena_prio_rate - enable switch priority rate
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine enables the priority rate function of the switch.
 */
static void sw_ena_prio_rate(struct ksz_sw *sw, int port)
{
	int prio;

	for (prio = 0; prio < PRIO_QUEUES; prio++) {
		hw_cfg_rx_prio_rate(sw, port, prio,
			sw->info->port_cfg[port].rx_rate[prio]);
		hw_cfg_tx_prio_rate(sw, port, prio,
			sw->info->port_cfg[port].tx_rate[prio]);
	}
}

/**
 * sw_init_prio_rate - initialize switch prioirty rate
 * @sw:		The switch instance.
 *
 * This routine initializes the priority rate function of the switch.
 */
static void sw_init_prio_rate(struct ksz_sw *sw)
{
	int offset;
	int port;
	int prio;

	for (port = 0; port < TOTAL_PORT_NUM; port++) {
		hw_get_rate_limit(sw, port);
		for (prio = 0; prio < PRIO_QUEUES; prio++) {
			hw_get_rate_ctrl(sw, port, prio);
#ifdef PORT_OUT_RATE_ADDR
			offset = REG_PORT_1_IN_RATE_0;
#else
			offset = REG_PORT_IN_RATE_0;
#endif
			port_get_rate(sw, port, prio, offset,
				&sw->info->port_cfg[port].rx_rate[prio]);
#ifdef PORT_OUT_RATE_ADDR
			offset = REG_PORT_1_OUT_RATE_0;
#else
			offset = REG_PORT_OUT_RATE_0;
#endif
			port_get_rate(sw, port, prio, offset,
				&sw->info->port_cfg[port].tx_rate[prio]);
		}
	}
}

/* -------------------------------------------------------------------------- */

/* Communication */

static inline void port_cfg_back_pressure(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_STP_CTRL, PORT_BACK_PRESSURE, set);
}

static inline void port_cfg_force_flow_ctrl(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_STP_CTRL, PORT_FORCE_FLOW_CTRL, set);
}

static inline int port_chk_back_pressure(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_STP_CTRL, PORT_BACK_PRESSURE);
}

static inline int port_chk_force_flow_ctrl(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_STP_CTRL, PORT_FORCE_FLOW_CTRL);
}

/* -------------------------------------------------------------------------- */

/* Spanning Tree */

static inline void port_cfg_dis_learn(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_STP_CTRL, PORT_LEARN_DISABLE, set);
}

static inline void port_cfg_rx(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_STP_CTRL, PORT_RX_ENABLE, set);
}

static inline void port_cfg_tx(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_STP_CTRL, PORT_TX_ENABLE, set);
}

static inline int port_chk_dis_learn(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_STP_CTRL, PORT_LEARN_DISABLE);
}

static inline int port_chk_rx(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_STP_CTRL, PORT_RX_ENABLE);
}

static inline int port_chk_tx(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_STP_CTRL, PORT_TX_ENABLE);
}

static inline void sw_cfg_fast_aging(struct ksz_sw *sw, int set)
{
	sw_cfg(sw, REG_SWITCH_CTRL_1, SWITCH_FAST_AGING, set);
}

static void sw_flush_dyn_mac_table(struct ksz_sw *sw, int port)
{
	int cnt;
	int first;
	int index;
	int learn_disable[TOTAL_PORT_NUM];

	if ((uint) port < TOTAL_PORT_NUM) {
		first = port;
		cnt = port + 1;
	} else {
		first = 0;
		cnt = TOTAL_PORT_NUM;
	}
	for (index = first; index < cnt; index++) {
		learn_disable[index] = port_chk_dis_learn(sw, index);
		if (!learn_disable[index])
			port_cfg_dis_learn(sw, index, 1);
	}
	sw_cfg(sw, S_FLUSH_TABLE_CTRL, SWITCH_FLUSH_DYN_MAC_TABLE, 1);
	for (index = first; index < cnt; index++) {
		if (!learn_disable[index])
			port_cfg_dis_learn(sw, index, 0);
	}
}

/* -------------------------------------------------------------------------- */

/* VLAN */

static inline void port_cfg_ins_tag(struct ksz_sw *sw, int p, int insert)
{
	port_cfg(sw, p,
		P_TAG_CTRL, PORT_INSERT_TAG, insert);
}

static inline void port_cfg_rmv_tag(struct ksz_sw *sw, int p, int remove)
{
	port_cfg(sw, p,
		P_TAG_CTRL, PORT_REMOVE_TAG, remove);
}

static inline int port_chk_ins_tag(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_TAG_CTRL, PORT_INSERT_TAG);
}

static inline int port_chk_rmv_tag(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_TAG_CTRL, PORT_REMOVE_TAG);
}

#ifdef PORT_DOUBLE_TAG
static inline void port_cfg_double_tag(struct ksz_sw *sw, int p, int remove)
{
	port_cfg(sw, p,
		P_MIRROR_CTRL, PORT_DOUBLE_TAG, remove);
}

static inline int port_chk_double_tag(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_MIRROR_CTRL, PORT_DOUBLE_TAG);
}
#endif

static inline void port_cfg_dis_non_vid(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_STP_CTRL, PORT_DISCARD_NON_VID, set);
}

static inline void port_cfg_drop_tag(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_SA_MAC_CTRL, PORT_DROP_TAG, set);
}

static inline void port_cfg_in_filter(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_STP_CTRL, PORT_INGRESS_FILTER, set);
}

static inline int port_chk_dis_non_vid(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_STP_CTRL, PORT_DISCARD_NON_VID);
}

static inline int port_chk_drop_tag(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_SA_MAC_CTRL, PORT_DROP_TAG);
}

static inline int port_chk_in_filter(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_STP_CTRL, PORT_INGRESS_FILTER);
}

/* -------------------------------------------------------------------------- */

/* Mirroring */

static inline void port_cfg_mirror_sniffer(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_MIRROR_CTRL, PORT_MIRROR_SNIFFER, set);
}

static inline void port_cfg_mirror_rx(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_MIRROR_CTRL, PORT_MIRROR_RX, set);
}

static inline void port_cfg_mirror_tx(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_MIRROR_CTRL, PORT_MIRROR_TX, set);
}

static inline void sw_cfg_mirror_rx_tx(struct ksz_sw *sw, int set)
{
	sw_cfg(sw, S_MIRROR_CTRL, SWITCH_MIRROR_RX_TX, set);
}

static inline int port_chk_mirror_sniffer(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_MIRROR_CTRL, PORT_MIRROR_SNIFFER);
}

static inline int port_chk_mirror_rx(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_MIRROR_CTRL, PORT_MIRROR_RX);
}

static inline int port_chk_mirror_tx(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_MIRROR_CTRL, PORT_MIRROR_TX);
}

static inline int sw_chk_mirror_rx_tx(struct ksz_sw *sw)
{
	return sw_chk(sw, S_MIRROR_CTRL, SWITCH_MIRROR_RX_TX);
}

static void sw_setup_mirror(struct ksz_sw *sw)
{
	int port;

	for (port = 0; port < TOTAL_PORT_NUM; port++) {
		port_cfg_mirror_sniffer(sw, port, 0);
		port_cfg_mirror_rx(sw, port, 0);
		port_cfg_mirror_tx(sw, port, 0);
	}
	sw_cfg_mirror_rx_tx(sw, 0);
}

/* -------------------------------------------------------------------------- */

static void sw_cfg_unk_dest(struct ksz_sw *sw, int set)
{
	u8 data;

	data = SW_R8(sw, S_UNKNOWN_DA_CTRL);
	if (set)
		data |= SWITCH_UNKNOWN_DA_ENABLE;
	else
		data &= ~SWITCH_UNKNOWN_DA_ENABLE;
	SW_W8(sw, S_UNKNOWN_DA_CTRL, data);
}

static int sw_chk_unk_dest(struct ksz_sw *sw)
{
	u8 data;

	data = SW_R8(sw, S_UNKNOWN_DA_CTRL);
	return (data & SWITCH_UNKNOWN_DA_ENABLE) == SWITCH_UNKNOWN_DA_ENABLE;
}

static void sw_cfg_unk_def_port(struct ksz_sw *sw, int port, int set)
{
	u8 data;
	u8 bits = 1 << port;

	data = SW_R8(sw, S_UNKNOWN_DA_CTRL);
	if (set)
		data |= bits;
	else
		data &= ~bits;
	SW_W8(sw, S_UNKNOWN_DA_CTRL, data);
}

static int sw_chk_unk_def_port(struct ksz_sw *sw, int port)
{
	u8 data;
	u8 bit = 1 << port;

	data = SW_R8(sw, S_UNKNOWN_DA_CTRL);
	return (data & bit) == bit;
}

static inline void sw_cfg_for_inv_vid(struct ksz_sw *sw, int port, int set)
{
	port_cfg_shift(sw, port, S_FORWARD_INVALID_VID_CTRL,
		FORWARD_INVALID_PORT_SHIFT, set);
}

static inline int sw_chk_for_inv_vid(struct ksz_sw *sw, int port)
{
	return port_chk_shift(sw, port, S_FORWARD_INVALID_VID_CTRL,
		FORWARD_INVALID_PORT_SHIFT);
}

/* -------------------------------------------------------------------------- */

/* Priority */

static inline void port_cfg_diffserv(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_PRIORITY_CTRL, PORT_DIFFSERV_ENABLE, set);
}

static inline void port_cfg_802_1p(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_PRIORITY_CTRL, PORT_802_1P_ENABLE, set);
}

static inline void port_cfg_replace_prio(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_MIRROR_CTRL, PORT_802_1P_REMAPPING, set);
}

static inline void port_cfg_prio(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_PRIORITY_CTRL, PORT_4_PRIORITIES_ENABLE, set);
}

static inline int port_chk_diffserv(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_PRIORITY_CTRL, PORT_DIFFSERV_ENABLE);
}

static inline int port_chk_802_1p(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_PRIORITY_CTRL, PORT_802_1P_ENABLE);
}

static inline int port_chk_replace_prio(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_MIRROR_CTRL, PORT_802_1P_REMAPPING);
}

static inline int port_chk_prio(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_PRIORITY_CTRL, PORT_4_PRIORITIES_ENABLE);
}

/* -------------------------------------------------------------------------- */

static inline void port_cfg_src_filter_0(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_SA_MAC_CTRL, PORT_SA_MAC1, set);
}

static inline void port_cfg_src_filter_1(struct ksz_sw *sw, int p, int set)
{
	port_cfg(sw, p,
		P_SA_MAC_CTRL, PORT_SA_MAC2, set);
}

static inline int port_chk_src_filter_0(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_SA_MAC_CTRL, PORT_SA_MAC1);
}

static inline int port_chk_src_filter_1(struct ksz_sw *sw, int p)
{
	return port_chk(sw, p,
		P_SA_MAC_CTRL, PORT_SA_MAC2);
}

/**
 * port_get_addr - get the port MAC address.
 * @sw:		The switch instance.
 * @port:	The port index.
 * @mac_addr:	Buffer to store the MAC address.
 *
 * This function retrieves the MAC address of the port.
 */
static inline void port_get_addr(struct ksz_sw *sw, int port, u8 *mac_addr)
{
	int i;
	SW_D reg;
#if (SW_SIZE == (2))
	u16 *addr = (u16 *) mac_addr;
#endif

	if (0 == port)
		reg = REG_PORT_0_MAC_ADDR_0;
	else
		reg = REG_PORT_1_MAC_ADDR_0;
#if (SW_SIZE == (2))
	for (i = 0; i < 6; i += 2) {
		*addr = SW_R16(sw, reg - i);
		*addr = ntohs(*addr);
		addr++;
	}
#else
	for (i = 0; i < 6; i++)
		mac_addr[i] = SW_R8(sw, reg + i);
#endif
	memcpy(sw->info->port_cfg[port].mac_addr, mac_addr, 6);
}

/**
 * port_set_addr - configure port MAC address
 * @sw:		The switch instance.
 * @port:	The port index.
 * @mac_addr:	The MAC address.
 *
 * This function configures the MAC address of the port.
 */
static void port_set_addr(struct ksz_sw *sw, int port, u8 *mac_addr)
{
	int i;
	SW_D reg;
#if (SW_SIZE == (2))
	u16 *addr = (u16 *) mac_addr;
#endif

	if (0 == port)
		reg = REG_PORT_0_MAC_ADDR_0;
	else
		reg = REG_PORT_1_MAC_ADDR_0;
#if (SW_SIZE == (2))
	for (i = 0; i < 6; i += 2) {
		SW_W16(sw, reg - i, htons(*addr));
		addr++;
	}
#else
	for (i = 0; i < 6; i++)
		SW_W8(sw, reg + i, mac_addr[i]);
#endif
	memcpy(sw->info->port_cfg[port].mac_addr, mac_addr, 6);
}

static inline void sw_setup_src_filter(struct ksz_sw *sw, u8 *mac_addr)
{
	int port;

	for (port = 0; port < SWITCH_PORT_NUM; port++) {
		port_set_addr(sw, port, mac_addr);
		port_cfg_src_filter_0(sw, port, 1);
		port_cfg_src_filter_1(sw, port, 1);
	}
}

/* -------------------------------------------------------------------------- */

/**
 * sw_set_tos_prio - program switch TOS priority
 * @sw:		The switch instance.
 * @tos:	ToS value from 6-bit (bit7 ~ bit2) of ToS field, ranging from 0
 *		to 63.
 * @prio:	Priority to be assigned.
 *
 * This routine programs the TOS priority into the switch registers.
 */
static void sw_set_tos_prio(struct ksz_sw *sw, u8 tos, SW_D prio)
{
	SW_W(sw, S_TOS_PRIORITY_CTRL + tos * SW_SIZE, prio);
}

/**
 * sw_dis_diffserv - disable switch DiffServ priority
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine disables the DiffServ priority function of the switch.
 */
static void sw_dis_diffserv(struct ksz_sw *sw, int port)
{
	port_cfg_diffserv(sw, port, 0);
}

/**
 * sw_ena_diffserv - enable switch DiffServ priority
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine enables the DiffServ priority function of the switch.
 */
static void sw_ena_diffserv(struct ksz_sw *sw, int port)
{
	port_cfg_diffserv(sw, port, 1);
}

/**
 * hw_cfg_tos_prio - configure TOS priority
 * @sw:		The switch instance.
 * @tos:	ToS value from 6-bit (bit7 ~ bit2) of ToS field, ranging from 0
 *		to 63.
 * @prio:	Priority to be assigned.
 *
 * This routine configures the TOS priority in the hardware.
 * DiffServ Value 0 ~ 63 is mapped to Priority Queue Number 0 ~ 3.
 * It is called by user functions.  The hardware should be acquired first.
 */
static void hw_cfg_tos_prio(struct ksz_sw *sw, u8 tos, SW_D prio)
{
	int shift;
	SW_D data = prio;
	SW_D mask = KS_PRIORITY_MASK;

	if (tos >= DIFFSERV_ENTRIES)
		return;

#if (SW_SIZE == (2))
	if (prio >= 0x100)
		mask = 0xffff;
	else
#endif
	if (prio >= 0x10)
		mask = 0xff;
	else if (prio > KS_PRIORITY_MASK)
		mask = 0xf;
	shift = (tos & (KS_PRIORITY_IN_REG - 1)) * KS_PRIORITY_SHIFT;
	prio = prio << shift;
	if (prio >> shift != data)
		return;
	mask <<= shift;
	tos /= KS_PRIORITY_IN_REG;

	sw->info->diffserv[tos] &= ~mask;
	sw->info->diffserv[tos] |= prio;

	sw_set_tos_prio(sw, tos, sw->info->diffserv[tos]);
}

/* -------------------------------------------------------------------------- */

/**
 * sw_set_802_1p_prio - program switch 802.1p priority
 * @sw:		The switch instance.
 * @prio:	Priority to be assigned.
 *
 * This routine programs the 802.1p priority into the switch register.
 */
static void sw_set_802_1p_prio(struct ksz_sw *sw, u8 tag, SW_D prio)
{
	SW_W(sw, S_802_1P_PRIORITY_CTRL + tag / SW_SIZE, prio);
}

/**
 * sw_dis_802_1p - disable switch 802.1p priority
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine disables the 802.1p priority function of the switch.
 */
static void sw_dis_802_1p(struct ksz_sw *sw, int port)
{
	port_cfg_802_1p(sw, port, 0);
}

/**
 * sw_ena_802_1p - enable switch 802.1p priority
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine enables the 802.1p priority function of the switch.
 */
static void sw_ena_802_1p(struct ksz_sw *sw, int port)
{
	port_cfg_802_1p(sw, port, 1);
}

/**
 * hw_cfg_802_1p_prio - configure 802.1p priority
 * @sw:		The switch instance.
 * @tag:	The 802.1p tag priority value, ranging from 0 to 7.
 * @prio:	Priority to be assigned.
 *
 * This routine configures the 802.1p priority in the hardware.
 * 802.1p Tag priority value 0 ~ 7 is mapped to Priority Queue Number 0 ~ 3.
 * It is called by user functions.  The hardware should be acquired first.
 */
static void hw_cfg_802_1p_prio(struct ksz_sw *sw, u8 tag, SW_D prio)
{
	int shift;
	SW_D data = prio;
	SW_D mask = KS_PRIORITY_MASK;

	if (tag >= PRIO_802_1P_ENTRIES)
		return;

#if (SW_SIZE == (2))
	if (prio >= 0x100)
		mask = 0xffff;
	else
#endif
	if (prio >= 0x10)
		mask = 0xff;
	else if (prio > KS_PRIORITY_MASK)
		mask = 0xf;
	shift = (tag & (KS_PRIORITY_IN_REG - 1)) * KS_PRIORITY_SHIFT;
	prio = prio << shift;
	if (prio >> shift != data)
		return;
	mask <<= shift;
	tag /= KS_PRIORITY_IN_REG;

	sw->info->p_802_1p[tag] &= ~mask;
	sw->info->p_802_1p[tag] |= prio;

	sw_set_802_1p_prio(sw, tag, sw->info->p_802_1p[tag]);
}

/**
 * sw_cfg_replace_null_vid -
 * @sw:		The switch instance.
 * @set:	The flag to disable or enable.
 *
 */
static void sw_cfg_replace_null_vid(struct ksz_sw *sw, int set)
{
	sw_cfg(sw, S_REPLACE_VID_CTRL, SWITCH_REPLACE_VID, set);
}

/**
 * sw_cfg_replace_prio - enable switch 802.10 priority re-mapping
 * @sw:		The switch instance.
 * @port:	The port index.
 * @set:	The flag to disable or enable.
 *
 * This routine enables the 802.1p priority re-mapping function of the switch.
 * That allows 802.1p priority field to be replaced with the port's default
 * tag's priority value if the ingress packet's 802.1p priority has a higher
 * priority than port's default tag's priority.
 */
static void sw_cfg_replace_prio(struct ksz_sw *sw, int port, int set)
{
	port_cfg_replace_prio(sw, port, set);
}

/* -------------------------------------------------------------------------- */

/**
 * sw_cfg_port_based - configure switch port based priority
 * @sw:		The switch instance.
 * @port:	The port index.
 * @prio:	The priority to set.
 *
 * This routine configures the port based priority of the switch.
 */
static void sw_cfg_port_based(struct ksz_sw *sw, int port, u8 prio)
{
	SW_D data;

	if (prio > PORT_BASED_PRIORITY_BASE)
		prio = PORT_BASED_PRIORITY_BASE;

	port_r(sw, port, P_PRIORITY_CTRL, &data);
	data &= ~PORT_BASED_PRIORITY_MASK;
	data |= prio << PORT_BASED_PRIORITY_SHIFT;
	port_w(sw, port, P_PRIORITY_CTRL, data);

	sw->info->port_cfg[port].port_prio = prio;
}

/* -------------------------------------------------------------------------- */

/**
 * sw_dis_multi_queue - disable transmit multiple queues
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine disables the transmit multiple queues selection of the switch
 * port.  Only single transmit queue on the port.
 */
static void sw_dis_multi_queue(struct ksz_sw *sw, int port)
{
	port_cfg_prio(sw, port, 0);
}

/**
 * sw_ena_multi_queue - enable transmit multiple queues
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine enables the transmit multiple queues selection of the switch
 * port.  The port transmit queue is split into four priority queues.
 */
static void sw_ena_multi_queue(struct ksz_sw *sw, int port)
{
	port_cfg_prio(sw, port, 1);
}

/**
 * sw_init_prio - initialize switch priority
 * @sw:		The switch instance.
 *
 * This routine initializes the switch QoS priority functions.
 */
static void sw_init_prio(struct ksz_sw *sw)
{
	int port;
	int tos;
	SW_D data;

	for (tos = 0; tos < PRIO_802_1P_ENTRIES / KS_PRIORITY_IN_REG; tos++)
		sw->info->p_802_1p[tos] =
			SW_R(sw, S_802_1P_PRIORITY_CTRL + tos * SW_SIZE);

	for (tos = 0; tos < DIFFSERV_ENTRIES / KS_PRIORITY_IN_REG; tos++)
		sw->info->diffserv[tos] =
			SW_R(sw, S_TOS_PRIORITY_CTRL + tos * SW_SIZE);

	for (port = 0; port < TOTAL_PORT_NUM; port++) {
		port_r(sw, port, P_PRIORITY_CTRL, &data);
		data &= PORT_BASED_PRIORITY_MASK;
		data >>= PORT_BASED_PRIORITY_SHIFT;
		sw->info->port_cfg[port].port_prio = data;
	}

#ifdef CONFIG_KSZ8463_SPI
/*
 * THa  2012/02/01
 * Port 3 sometimes cannot send frames out through the port where 100%
 * wire-rate 64 bytes traffic also goes through.  Solution is to assign
 * different port priorities.  It does not matter port 3 has higher priority,
 * just different from the port where heavy traffic comes in.
 */
	for (port = 0; port < TOTAL_PORT_NUM; port++)
		sw->info->port_cfg[port].port_prio = port;
#endif
}

/**
 * sw_setup_prio - setup switch priority
 * @sw:		The switch instance.
 *
 * This routine setup the switch QoS priority functions.
 */
static void sw_setup_prio(struct ksz_sw *sw)
{
	int port;

	/* All QoS functions disabled. */
	for (port = 0; port < TOTAL_PORT_NUM; port++) {
		sw_ena_multi_queue(sw, port);
		sw_dis_diffserv(sw, port);
		sw_dis_802_1p(sw, port);
		sw_cfg_replace_prio(sw, port, 0);
		sw_cfg_port_based(sw, port, sw->info->port_cfg[port].port_prio);
	}
	sw_cfg_replace_null_vid(sw, 0);
}

/* -------------------------------------------------------------------------- */

/**
 * port_cfg_def_vid - configure port default VID
 * @sw:		The switch instance.
 * @port:	The port index.
 * @vid:	The VID value.
 *
 * This routine configures the default VID of the port.
 */
static void port_cfg_def_vid(struct ksz_sw *sw, int port, u16 vid)
{
	port_w16(sw, port, REG_PORT_CTRL_VID, vid);
}

/**
 * port_get_def_vid - get port default VID.
 * @sw:		The switch instance.
 * @port:	The port index.
 * @vid:	Buffer to store the VID.
 *
 * This routine retrieves the default VID of the port.
 */
static void port_get_def_vid(struct ksz_sw *sw, int port, u16 *vid)
{
	port_r16(sw, port, REG_PORT_CTRL_VID, vid);
}

/**
 * sw_cfg_def_vid - configure switch port default VID
 * @sw:		The switch instance.
 * @port:	The port index.
 * @vid:	The VID value.
 *
 * This routine configures the default VID of the port.
 */
static void sw_cfg_def_vid(struct ksz_sw *sw, int port, u16 vid)
{
	sw->info->port_cfg[port].vid = vid;
	port_cfg_def_vid(sw, port, vid);
}

/**
 * sw_cfg_port_base_vlan - configure port-based VLAN membership
 * @sw:		The switch instance.
 * @port:	The port index.
 * @member:	The port-based VLAN membership.
 *
 * This routine configures the port-based VLAN membership of the port.
 */
static void sw_cfg_port_base_vlan(struct ksz_sw *sw, int port, u8 member)
{
	SW_D data;

	port_r(sw, port, P_MIRROR_CTRL, &data);
	data &= ~PORT_VLAN_MEMBERSHIP;
	data |= (member & PORT_MASK);
	port_w(sw, port, P_MIRROR_CTRL, data);

	sw->info->port_cfg[port].member = member;
}

/**
 * sw_vlan_cfg_dis_non_vid - configure discard non VID
 * @sw:		The switch instance.
 * @port:	The port index.
 * @set:	The flag to disable or enable.
 *
 * This routine configures Discard Non VID packets of the switch port.
 * If enabled, the device will discard packets whose VLAN id does not match
 * ingress port-based default VLAN id.
 */
static void sw_vlan_cfg_dis_non_vid(struct ksz_sw *sw, int port, int set)
{
	port_cfg_dis_non_vid(sw, port, set);
}

/**
 * sw_vlan_cfg_drop_tag - configure 802.1q tagged packet drop
 * @sw:		The switch instance.
 * @port:	The port index.
 * @set:	The flag to disable or enable.
 *
 */
static void sw_vlan_cfg_drop_tag(struct ksz_sw *sw, int port, int set)
{
	port_cfg_drop_tag(sw, port, set);
}

/**
 * sw_vlan_cfg_in_filter - configure ingress VLAN filtering
 * @sw:		The switch instance.
 * @port:	The port index.
 * @set:	The flag to disable or enable.
 *
 * This routine configures Ingress VLAN filtering of the switch port.
 * If enabled, the device will discard packets whose VLAN id membership	in the
 * VLAN table bits [18:16] does not include the ingress port that received this
 * packet.
 */
static void sw_vlan_cfg_in_filter(struct ksz_sw *sw, int port, int set)
{
	port_cfg_in_filter(sw, port, set);
}

/**
 * sw_vlan_cfg_ins_tag - configure 802.1q tag insertion
 * @sw:		The switch instance.
 * @port:	The port index.
 * @set:	The flag to disable or enable.
 *
 * This routine configures 802.1q Tag insertion to the switch port.
 * If enabled, the device will insert 802.1q tag to the transmit packet on this
 * port if received packet is an untagged packet.  The device will not insert
 * 802.1q tag if received packet is tagged packet.
 */
static void sw_vlan_cfg_ins_tag(struct ksz_sw *sw, int port, int set)
{
	port_cfg_ins_tag(sw, port, set);
}

/**
 * sw_vlan_cfg_rmv_tag - configure 802.1q tag removal
 * @sw:		The switch instance.
 * @port:	The port index.
 * @set:	The flag to disable or enable.
 *
 * This routine configures 802.1q Tag removal to the switch port.
 * If enabled, the device will removed 802.1q tag to the transmit packet on
 * this port if received packet is a tagged packet.  The device will not remove
 * 802.1q tag if received packet is untagged packet.
 */
static void sw_vlan_cfg_rmv_tag(struct ksz_sw *sw, int port, int set)
{
	port_cfg_rmv_tag(sw, port, set);
}

#ifdef PORT_DOUBLE_TAG
/**
 * sw_vlan_cfg_double_tag - configure 802.1q double tag
 * @sw:		The switch instance.
 * @port:	The port index.
 * @set:	The flag to disable or enable.
 *
 */
static void sw_vlan_cfg_double_tag(struct ksz_sw *sw, int port, int set)
{
	port_cfg_double_tag(sw, port, set);
}
#endif

/**
 * sw_dis_vlan - disable switch VLAN
 * @sw:		The switch instance.
 *
 * This routine disables the VLAN function of the switch.
 */
static void sw_dis_vlan(struct ksz_sw *sw)
{
	sw_cfg(sw, S_MIRROR_CTRL, SWITCH_VLAN_ENABLE, 0);
}

/**
 * sw_ena_vlan - enable switch VLAN
 * @sw:		The switch instance.
 *
 * This routine enables the VLAN function of the switch.
 */
static void sw_ena_vlan(struct ksz_sw *sw)
{
	int entry;
	struct ksz_sw_info *info = sw->info;

	/* Create 16 VLAN entries in the VLAN table. */
	for (entry = 0; entry < VLAN_TABLE_ENTRIES; entry++) {
		sw_w_vlan_table(sw, entry,
			info->vlan_table[entry].vid,
			info->vlan_table[entry].fid,
			info->vlan_table[entry].member,
			info->vlan_table[entry].valid);
	}

	/* Enable 802.1q VLAN mode. */
	sw_cfg(sw, REG_SWITCH_CTRL_2, UNICAST_VLAN_BOUNDARY, 1);
	sw_cfg(sw, S_MIRROR_CTRL, SWITCH_VLAN_ENABLE, 1);
}

/**
 * sw_init_vlan - initialize switch VLAN
 * @sw:		The switch instance.
 *
 * This routine initializes the VLAN function of the switch.
 */
static void sw_init_vlan(struct ksz_sw *sw)
{
	int port;
	int entry;
	SW_D data;
	struct ksz_sw_info *info = sw->info;

	/* Read 16 VLAN entries from device's VLAN table. */
	for (entry = 0; entry < VLAN_TABLE_ENTRIES; entry++) {
		if (!sw_r_vlan_table(sw, entry,
				&info->vlan_table[entry].vid,
				&info->vlan_table[entry].fid,
				&info->vlan_table[entry].member))
			info->vlan_table[entry].valid = 1;
		else
			info->vlan_table[entry].valid = 0;
	}

	for (port = 0; port < TOTAL_PORT_NUM; port++) {
		port_get_def_vid(sw, port, &info->port_cfg[port].vid);
		port_r(sw, port, P_MIRROR_CTRL, &data);
		data &= PORT_VLAN_MEMBERSHIP;
		info->port_cfg[port].member = data;
	}
	sw_cfg(sw, S_INS_SRC_PVID_CTRL,
		(SWITCH_INS_TAG_1_PORT_2 | SWITCH_INS_TAG_1_PORT_3 |
		SWITCH_INS_TAG_2_PORT_1 | SWITCH_INS_TAG_2_PORT_3 |
		SWITCH_INS_TAG_3_PORT_1 | SWITCH_INS_TAG_3_PORT_2),
		true);
}

/**
 * sw_get_addr - get the switch MAC address.
 * @sw:		The switch instance.
 * @mac_addr:	Buffer to store the MAC address.
 *
 * This function retrieves the MAC address of the switch.
 */
static inline void sw_get_addr(struct ksz_sw *sw, u8 *mac_addr)
{
	int i;
#if (SW_SIZE == (2))
	u16 *addr = (u16 *) mac_addr;

	for (i = 0; i < 6; i += 2) {
		*addr = SW_R16(sw, REG_SWITCH_MAC_ADDR_0 + i);
		*addr = ntohs(*addr);
		addr++;
	}
#else

	for (i = 0; i < 6; i++)
		mac_addr[i] = SW_R8(sw, REG_SWITCH_MAC_ADDR_0 + i);
#endif
	memcpy(sw->info->mac_addr, mac_addr, 6);
}

/**
 * sw_set_addr - configure switch MAC address
 * @sw:		The switch instance.
 * @mac_addr:	The MAC address.
 *
 * This function configures the MAC address of the switch.
 */
static void sw_set_addr(struct ksz_sw *sw, u8 *mac_addr)
{
	int i;
#if (SW_SIZE == (2))
	u16 *addr = (u16 *) mac_addr;

	for (i = 0; i < 6; i += 2) {
		SW_W16(sw, REG_SWITCH_MAC_ADDR_0 + i, htons(*addr));
		addr++;
	}
#else

	for (i = 0; i < 6; i++)
		SW_W8(sw, REG_SWITCH_MAC_ADDR_0 + i, mac_addr[i]);
#endif
	memcpy(sw->info->mac_addr, mac_addr, 6);
}

#ifdef SWITCH_PORT_PHY_ADDR_MASK
/**
 * sw_init_phy_addr - initialize switch PHY address
 * @sw:		The switch instance.
 *
 * This function initializes the PHY address of the switch.
 */
static void sw_init_phy_addr(struct ksz_sw *sw)
{
	u8 addr;

	addr = SW_R8(sw, REG_SWITCH_CTRL_13);
	addr >>= SWITCH_PORT_PHY_ADDR_SHIFT;
	addr &= SWITCH_PORT_PHY_ADDR_MASK;
	sw->info->phy_addr = addr;
}

/**
 * sw_set_phy_addr - configure switch PHY address
 * @sw:		The switch instance.
 * @addr:	The PHY address.
 *
 * This function configures the PHY address of the switch.
 */
static void sw_set_phy_addr(struct ksz_sw *sw, u8 addr)
{
	sw->info->phy_addr = addr;
	addr &= SWITCH_PORT_PHY_ADDR_MASK;
	addr <<= SWITCH_PORT_PHY_ADDR_SHIFT;
	SW_W8(sw, REG_SWITCH_CTRL_13, addr);
}
#endif

/**
 * sw_set_global_ctrl - set switch global control
 * @sw:		The switch instance.
 *
 * This routine sets the global control of the switch function.
 */
static void sw_set_global_ctrl(struct ksz_sw *sw)
{
	SW_D data;

	/* Enable switch MII flow control. */
	data = SW_R(sw, S_REPLACE_VID_CTRL);
	data |= SWITCH_FLOW_CTRL;
	SW_W(sw, S_REPLACE_VID_CTRL, data);

	data = SW_R(sw, S_LINK_AGING_CTRL);
	data |= SWITCH_LINK_AUTO_AGING;
	SW_W(sw, S_LINK_AGING_CTRL, data);

	data = SW_R(sw, REG_SWITCH_CTRL_1);

	/* Enable aggressive back off algorithm in half duplex mode. */
	data |= SWITCH_AGGR_BACKOFF;

	/* Enable automatic fast aging when link changed detected. */
	data |= SWITCH_AGING_ENABLE;

	if (sw->overrides & FAST_AGING)
		data |= SWITCH_FAST_AGING;
	else
		data &= ~SWITCH_FAST_AGING;

	SW_W(sw, REG_SWITCH_CTRL_1, data);

	data = SW_R(sw, REG_SWITCH_CTRL_2);

	/* Make sure unicast VLAN boundary is set as default. */
	if (sw->dev_count > 1)
		data |= UNICAST_VLAN_BOUNDARY;

	/* Enable no excessive collision drop. */
	data |= NO_EXC_COLLISION_DROP;
	SW_W(sw, REG_SWITCH_CTRL_2, data);
}

/* -------------------------------------------------------------------------- */

enum {
	STP_STATE_DISABLED = 0,
	STP_STATE_LISTENING,
	STP_STATE_LEARNING,
	STP_STATE_FORWARDING,
	STP_STATE_BLOCKED,
	STP_STATE_SIMPLE
};

/**
 * port_set_stp_state - configure port spanning tree state
 * @sw:		The switch instance.
 * @port:	The port index.
 * @state:	The spanning tree state.
 *
 * This routine configures the spanning tree state of the port.
 */
static void port_set_stp_state(struct ksz_sw *sw, int port, int state)
{
	SW_D data;

	port_r(sw, port, P_STP_CTRL, &data);
	switch (state) {
	case STP_STATE_DISABLED:
		data &= ~(PORT_TX_ENABLE | PORT_RX_ENABLE);
		data |= PORT_LEARN_DISABLE;
		break;
	case STP_STATE_LISTENING:
/*
 * No need to turn on transmit because of port direct mode.
 * Turning on receive is required if static MAC table is not setup.
 */
		data &= ~PORT_TX_ENABLE;
		data |= PORT_RX_ENABLE;
		data |= PORT_LEARN_DISABLE;
		break;
	case STP_STATE_LEARNING:
		data &= ~PORT_TX_ENABLE;
		data |= PORT_RX_ENABLE;
		data &= ~PORT_LEARN_DISABLE;
		break;
	case STP_STATE_FORWARDING:
		data |= (PORT_TX_ENABLE | PORT_RX_ENABLE);
		data &= ~PORT_LEARN_DISABLE;
		break;
	case STP_STATE_BLOCKED:
/*
 * Need to setup static MAC table with override to keep receiving BPDU
 * messages.  See sw_setup_stp routine.
 */
		data &= ~(PORT_TX_ENABLE | PORT_RX_ENABLE);
		data |= PORT_LEARN_DISABLE;
		break;
	case STP_STATE_SIMPLE:
		data |= (PORT_TX_ENABLE | PORT_RX_ENABLE);
		data |= PORT_LEARN_DISABLE;
		break;
	}
	port_w(sw, port, P_STP_CTRL, data);
	sw->info->port_cfg[port].stp_state = state;
}

#define STP_ENTRY			0
#define BROADCAST_ENTRY			1
#define BRIDGE_ADDR_ENTRY		2
#define IPV6_ADDR_ENTRY			3

/**
 * sw_clr_sta_mac_table - clear static MAC table
 * @sw:		The switch instance.
 *
 * This routine clears the static MAC table.
 */
static void sw_clr_sta_mac_table(struct ksz_sw *sw)
{
	struct ksz_mac_table *entry;
	int i;

	for (i = 0; i < STATIC_MAC_TABLE_ENTRIES; i++) {
		entry = &sw->info->mac_table[i];
		sw_w_sta_mac_table(sw, i,
			entry->mac_addr, entry->ports,
			0, 0,
			entry->use_fid, entry->fid);
	}
}

/**
 * sw_setup_stp - setup switch spanning tree support
 * @sw:		The switch instance.
 *
 * This routine initializes the spanning tree support of the switch.
 */
static void sw_setup_stp(struct ksz_sw *sw)
{
	struct ksz_mac_table *entry;

	entry = &sw->info->mac_table[STP_ENTRY];
	entry->mac_addr[0] = 0x01;
	entry->mac_addr[1] = 0x80;
	entry->mac_addr[2] = 0xC2;
	entry->mac_addr[3] = 0x00;
	entry->mac_addr[4] = 0x00;
	entry->mac_addr[5] = 0x00;
	entry->ports = HOST_MASK;
	entry->override = 1;
	entry->valid = 1;
	sw_w_sta_mac_table(sw, STP_ENTRY,
		entry->mac_addr, entry->ports,
		entry->override, entry->valid,
		entry->use_fid, entry->fid);
}

#ifdef CONFIG_KSZ_STP
/**
 * sw_block_addr - block certain packets from the host port
 * @sw:		The switch instance.
 *
 * This routine blocks certain packets from reaching to the host port.
 */
static void sw_block_addr(struct ksz_sw *sw)
{
	struct ksz_mac_table *entry;
	int i;

	for (i = BROADCAST_ENTRY; i <= IPV6_ADDR_ENTRY; i++) {
		entry = &sw->info->mac_table[i];
		entry->valid = 0;
		sw_w_sta_mac_table(sw, i,
			entry->mac_addr, entry->ports,
			0, entry->valid,
			entry->use_fid, entry->fid);
	}
}

static void bridge_change(struct ksz_sw *sw)
{
	int port;
	u8 member;
	struct ksz_sw_info *info = sw->info;

	/* No ports in forwarding state. */
	if (!info->member) {
		port_set_stp_state(sw, SWITCH_PORT_NUM, STP_STATE_SIMPLE);
		sw_block_addr(sw);
	}
	for (port = 0; port < SWITCH_PORT_NUM; port++) {
		if (STP_STATE_FORWARDING == info->port_cfg[port].stp_state)
			member = HOST_MASK | info->member;
		else
			member = HOST_MASK | (1 << port);
		if (member != info->port_cfg[port].member)
			sw_cfg_port_base_vlan(sw, port, member);
	}
}  /* bridge_change */

#ifndef NO_STP_MONITOR
/**
 * sw_pass_addr - allow certain packets to the host port
 * @sw:		The switch instance.
 *
 * This routine allows certain packets to reach the host port.
 */
static void sw_pass_addr(struct ksz_sw *sw)
{
	struct ksz_mac_table *entry;
	int i;
	struct ksz_sw_info *info = sw->info;

	for (i = BROADCAST_ENTRY; i <= IPV6_ADDR_ENTRY; i++) {
		entry = &sw->info->mac_table[i];
		switch (i) {
		case BROADCAST_ENTRY:
			memset(entry->mac_addr, 0xFF, MAC_ADDR_LEN);
			break;
		case BRIDGE_ADDR_ENTRY:
			memcpy(entry->mac_addr, info->br_addr, MAC_ADDR_LEN);
			break;
		case IPV6_ADDR_ENTRY:
			memcpy(entry->mac_addr, info->br_addr, MAC_ADDR_LEN);
			entry->mac_addr[0] = 0x33;
			entry->mac_addr[1] = 0x33;
			entry->mac_addr[2] = 0xFF;
			break;
		}
		entry->ports = HOST_MASK;
		entry->override = 1;
		entry->valid = 1;
		sw_w_sta_mac_table(sw, i,
			entry->mac_addr, entry->ports,
			entry->override, entry->valid,
			entry->use_fid, entry->fid);
	}
}

static void monitor_ports(struct ksz_sw *sw)
{
	int port;
	struct net_device *bridge_dev = NULL;
	struct ksz_sw_info *info = sw->info;
	u8 member = info->member;
	u8 stp = info->stp;
	u8 prev_stp = info->stp;
	u8 state;

	for (port = 0; port < SWITCH_PORT_NUM; port++) {
		struct net_device *dev = sw->netdev[port + sw->dev_offset];
		struct dev_priv *priv = netdev_priv(dev);

		state = get_port_state(dev, &bridge_dev);
		if (state != STP_STATE_SIMPLE)
			stp |= (1 << port);
		else {
			stp &= ~(1 << port);
			state = priv->state;
		}
		if (stp != info->stp) {
			info->stp = stp;
			if (!(stp & (1 << port))) {
				if (netif_running(dev))
					state = STP_STATE_SIMPLE;
			}
		}
		priv->state = state;

		if (info->port_cfg[port].stp_state != state) {
			if (STP_STATE_FORWARDING ==
					info->port_cfg[port].stp_state)
				member &= ~(1 << port);
			if (STP_STATE_FORWARDING == state)
				member |= (1 << port);
			port_set_stp_state(sw, port, state);
			if (STP_STATE_LEARNING == state)
				sw_flush_dyn_mac_table(sw, port);
		}
	}
	if (prev_stp != info->stp && !info->stp)
		memset(info->br_addr, 0, MAC_ADDR_LEN);

	if (member != info->member) {

		/* Have first member. */
		if (!info->member) {
			port_set_stp_state(sw, SWITCH_PORT_NUM,
				STP_STATE_BLOCKED);

			/* Force to program bridge address. */
			info->br_addr[0] = 0xFF;
		}
		info->member = member;
		bridge_change(sw);
	}

	/* At least one port in forwarding state. */
	if (info->member && bridge_dev && memcmp(bridge_dev->dev_addr,
			info->br_addr, MAC_ADDR_LEN)) {
		memcpy(info->br_addr, bridge_dev->dev_addr, MAC_ADDR_LEN);
		sw_pass_addr(sw);
	}
}  /* monitor_ports */
#endif
#else
#ifndef NO_STP_MONITOR
static void monitor_ports(struct ksz_sw *sw) {}
#endif
#endif

/*
 * Link detection routines
 */

static inline void dbp_link(struct ksz_port *port, struct ksz_sw *sw,
	int change)
{
	struct ksz_port_info *info;
	int i;
	int p;

	for (i = 0, p = port->first_port; i < port->port_cnt; i++, p++) {
		info = &sw->port_info[p];
		if (media_connected == info->state) {
			if (change & (1 << i)) {
				printk(KERN_INFO "link %d-%d: %d, %d\n",
					sw->id, i + port->first_port,
					info->tx_rate / TX_RATE_UNIT,
					info->duplex);
			}
		} else {
			if (change & (1 << i))
				printk(KERN_INFO "link %d-%d disconnected\n",
					sw->id, i + port->first_port);
		}
	}
}

static SW_D port_advertised_flow_ctrl(struct ksz_port *port, SW_D ctrl)
{
	ctrl &= ~PORT_AUTO_NEG_SYM_PAUSE;
	switch (port->flow_ctrl) {
	case PHY_FLOW_CTRL:
		ctrl |= PORT_AUTO_NEG_SYM_PAUSE;
		break;
	/* Not supported. */
	case PHY_TX_ONLY:
	case PHY_RX_ONLY:
	default:
		break;
	}
	return ctrl;
}

static void sw_determine_flow_ctrl(struct ksz_sw *sw, struct ksz_port *port,
	u8 local, u8 remote)
{
	int rx;
	int tx;

	if (sw->overrides & PAUSE_FLOW_CTRL)
		return;

	rx = tx = 0;
	if (port->force_link)
		rx = tx = 1;
	if (remote & PORT_REMOTE_SYM_PAUSE) {
		if (local & PORT_AUTO_NEG_SYM_PAUSE)
			rx = tx = 1;
	}
#ifdef DEBUG
	printk(KERN_INFO "pause: %d, %d; %02x %02x\n",
		rx, tx, local, remote);
#endif
}

/**
 * port_get_link_speed - get current link status
 * @port:	The port instance.
 *
 * This routine reads PHY registers to determine the current link status of the
 * switch ports.
 */
static int port_get_link_speed(struct ksz_port *port)
{
	struct ksz_port_info *info;
	struct ksz_port_info *linked = NULL;
	struct ksz_port_state *state;
	struct ksz_sw *sw = port->sw;
	SW_D data;
	SW_D status;
	u8 local;
	u8 remote;
	int i;
	int p;
	int change = 0;

	for (i = 0, p = port->first_port; i < port->port_cnt; i++, p++) {
		info = &sw->port_info[p];
		state = &sw->port_state[p];
		port_r(sw, p, P_PHY_CTRL, &data);
		port_r(sw, p, P_SPEED_STATUS, &status);

#if (SW_SIZE == (1))
		port_r(sw, p, P_LINK_STATUS, &remote);
#else
		remote = status;
#endif
		remote &= ~PORT_MDIX_STATUS;
		local = (u8) data;

		if (remote & PORT_STATUS_LINK_GOOD) {

			/* Remember the first linked port. */
			if (!linked)
				linked = info;
		}

		/* No change to status. */
		if (local == info->advertised && remote == info->partner)
			continue;

#ifdef DEBUG
		printk(KERN_INFO "advertised: %02X-%02X; partner: %02X-%02X\n",
			local, info->advertised, remote, info->partner);
#endif
		info->advertised = local;
		info->partner = remote;
		if (remote & PORT_STATUS_LINK_GOOD) {
			info->tx_rate = 10 * TX_RATE_UNIT;
			if (status & PORT_STAT_SPEED_100MBIT)
				info->tx_rate = 100 * TX_RATE_UNIT;

			info->duplex = 1;
			if (status & PORT_STAT_FULL_DUPLEX)
				info->duplex = 2;

#ifdef DEBUG
			printk(KERN_INFO "flow_ctrl: "SW_SIZE_STR"\n", status &
				(PORT_RX_FLOW_CTRL | PORT_TX_FLOW_CTRL));
#endif
			if (media_connected != info->state) {
				sw_determine_flow_ctrl(sw, port, local, remote);
				if (sw->info)
					port_cfg_back_pressure(sw, p,
						(1 == info->duplex));
				change |= 1 << i;
			}
			info->state = media_connected;
		} else {
			if (media_disconnected != info->state) {
				change |= 1 << i;

				/* Indicate the link just goes down. */
				state->link_down = 1;
			}
			info->state = media_disconnected;
		}
		state->state = info->state;
	}

	if (linked && media_disconnected == port->linked->state)
		port->linked = linked;

#ifdef DEBUG
	if (change)
		dbp_link(port, sw, change);
#endif
	return change;
}

/**
 * port_set_link_speed - set port speed
 * @port:	The port instance.
 *
 * This routine sets the link speed of the switch ports.
 */
static void port_set_link_speed(struct ksz_port *port)
{
	struct ksz_sw *sw = port->sw;
	SW_D data;
	SW_D cfg;
	SW_D status;
	int i;
	int p;

	for (i = 0, p = port->first_port; i < port->port_cnt; i++, p++) {
		port_r(sw, p, P_PHY_CTRL, &data);
		port_r(sw, p, P_LINK_STATUS, &status);

		cfg = 0;
		if (status & PORT_STATUS_LINK_GOOD)
			cfg = data;

		data |= PORT_AUTO_NEG_ENABLE;
		data = port_advertised_flow_ctrl(port, data);

		data |= PORT_AUTO_NEG_100BTX_FD | PORT_AUTO_NEG_100BTX |
			PORT_AUTO_NEG_10BT_FD | PORT_AUTO_NEG_10BT;

		/* Check if manual configuration is specified by the user. */
		if (port->speed || port->duplex) {
			if (10 == port->speed)
				data &= ~(PORT_AUTO_NEG_100BTX_FD |
					PORT_AUTO_NEG_100BTX);
			else if (100 == port->speed)
				data &= ~(PORT_AUTO_NEG_10BT_FD |
					PORT_AUTO_NEG_10BT);
			if (1 == port->duplex)
				data &= ~(PORT_AUTO_NEG_100BTX_FD |
					PORT_AUTO_NEG_10BT_FD);
			else if (2 == port->duplex)
				data &= ~(PORT_AUTO_NEG_100BTX |
					PORT_AUTO_NEG_10BT);
		}
		if (data != cfg) {
#if (SW_SIZE == (1))
			port_w(sw, p, P_PHY_CTRL, data);
			port_r(sw, p, P_NEG_RESTART_CTRL, &data);
#endif
			data |= PORT_AUTO_NEG_RESTART;
			port_w(sw, p, P_NEG_RESTART_CTRL, data);
		}
	}
}

/**
 * port_force_link_speed - force port speed
 * @port:	The port instance.
 *
 * This routine forces the link speed of the switch ports.
 */
static void port_force_link_speed(struct ksz_port *port)
{
	struct ksz_sw *sw = port->sw;
	SW_D data;
	int i;
	int p;

	for (i = 0, p = port->first_port; i < port->port_cnt; i++, p++) {
		port_r(sw, p, P_PHY_CTRL, &data);
		data &= ~PORT_AUTO_NEG_ENABLE;
		if (10 == port->speed)
			data &= ~PORT_FORCE_100_MBIT;
		else if (100 == port->speed)
			data |= PORT_FORCE_100_MBIT;
		if (1 == port->duplex)
			data &= ~PORT_FORCE_FULL_DUPLEX;
		else if (2 == port->duplex)
			data |= PORT_FORCE_FULL_DUPLEX;
		port_w(sw, p, P_PHY_CTRL, data);
	}
}

/**
 * sw_enable - enable the switch
 * @sw:		The switch instance.
 *
 */
static void sw_enable(struct ksz_sw *sw)
{
	int port;

	for (port = 0; port < SWITCH_PORT_NUM; port++) {
		if (sw->dev_count > 1) {
			/* Set port-base vlan membership with host port. */
			sw_cfg_port_base_vlan(sw, port,
				HOST_MASK | (1 << port));
			port_set_stp_state(sw, port, STP_STATE_DISABLED);
		} else {
			sw_cfg_port_base_vlan(sw, port, PORT_MASK);
			port_set_stp_state(sw, port, STP_STATE_FORWARDING);
		}
	}
	if (sw->dev_count > 1)
		port_set_stp_state(sw, SWITCH_PORT_NUM, STP_STATE_SIMPLE);
	else
		port_set_stp_state(sw, SWITCH_PORT_NUM, STP_STATE_FORWARDING);
}

/**
 * sw_init - initialize the switch
 * @sw:		The switch instance.
 *
 * This routine initializes the hardware switch engine for default operation.
 */
static void sw_init(struct ksz_sw *sw)
{
#ifdef SWITCH_PORT_PHY_ADDR_MASK
	sw_init_phy_addr(sw);
#endif

	sw_init_broad_storm(sw);

	sw_init_prio(sw);

	sw_init_prio_rate(sw);

	sw_init_vlan(sw);

	if (!sw_chk(sw, REG_SWITCH_CTRL_1,
			SWITCH_TX_FLOW_CTRL | SWITCH_RX_FLOW_CTRL))
		sw->overrides |= PAUSE_FLOW_CTRL;
}

/**
 * sw_setup - setup the switch
 * @sw:		The switch instance.
 *
 * This routine setup the hardware switch engine for default operation.
 */
static void sw_setup(struct ksz_sw *sw)
{
	int port;

	sw_set_global_ctrl(sw);
	for (port = 0; port < SWITCH_PORT_NUM; port++) {
		port_cfg_back_pressure(sw, port, 1);
		port_cfg_force_flow_ctrl(sw, port, 0);
	}

	sw_setup_broad_storm(sw);

	sw_setup_prio(sw);

	sw_setup_mirror(sw);

	if (sw->features & STP_SUPPORT)
		sw_setup_stp(sw);
	sw_enable(sw);
}

static inline void sw_reset(struct ksz_sw *sw)
{
	u8 data;

	data = SW_R8(sw, REG_SWITCH_RESET);
	data |= GLOBAL_SOFTWARE_RESET;
	SW_W8(sw, REG_SWITCH_RESET, data);
	data &= ~GLOBAL_SOFTWARE_RESET;
	SW_W8(sw, REG_SWITCH_RESET, data);
}

/* -------------------------------------------------------------------------- */

/*
 * Micrel LinkMD routines
 */

enum {
	CABLE_UNKNOWN,
	CABLE_GOOD,
	CABLE_CROSSED,
	CABLE_REVERSED,
	CABLE_CROSSED_REVERSED,
	CABLE_OPEN,
	CABLE_SHORT
};

#define STATUS_FULL_DUPLEX		0x01
#define STATUS_CROSSOVER		0x02
#define STATUS_REVERSED			0x04

#define LINK_10MBPS_FULL		0x00000001
#define LINK_10MBPS_HALF		0x00000002
#define LINK_100MBPS_FULL		0x00000004
#define LINK_100MBPS_HALF		0x00000008
#define LINK_1GBPS_FULL			0x00000010
#define LINK_1GBPS_HALF			0x00000020
#define LINK_10GBPS_FULL		0x00000040
#define LINK_10GBPS_HALF		0x00000080
#define LINK_SYM_PAUSE			0x00000100
#define LINK_ASYM_PAUSE			0x00000200

#define LINK_AUTO_MDIX			0x00010000
#define LINK_MDIX			0x00020000
#define LINK_AUTO_POLARITY		0x00040000

#define CABLE_LEN_MAXIMUM		15000
#define CABLE_LEN_MULTIPLIER		41

#define PHY_RESET_TIMEOUT		10

/**
 * hw_get_link_md -
 * @sw:		The switch instance.
 * @port:	The port index.
 *
 * This routine is used to get the LinkMD status.
 */
static void hw_get_link_md(struct ksz_sw *sw, int port)
{
	SW_D crossover;
	SW_D ctrl;
	SW_D data;
	SW_D link;
	u16 len;
	int i;
	int timeout;
	struct ksz_port_info *port_info = &sw->port_info[port];

	port_r(sw, port, P_SPEED_STATUS, &data);
#if (SW_SIZE == (1))
	port_r(sw, port, P_LINK_STATUS, &link);
#else
	link = data;
#endif
	port_info->status[0] = CABLE_UNKNOWN;
	if (link & PORT_STATUS_LINK_GOOD) {
		int stat = 0;

		port_info->status[0] = CABLE_GOOD;
		port_info->length[0] = 1;
		port_info->status[1] = CABLE_GOOD;
		port_info->length[1] = 1;
		port_info->status[2] = CABLE_GOOD;
		port_info->length[2] = 1;

		if (link & PORT_MDIX_STATUS)
			stat |= STATUS_CROSSOVER;
		if (data & PORT_REVERSED_POLARITY)
			stat |= STATUS_REVERSED;
		if ((stat & (STATUS_CROSSOVER | STATUS_REVERSED)) ==
				(STATUS_CROSSOVER | STATUS_REVERSED))
			port_info->status[0] = CABLE_CROSSED_REVERSED;
		else if ((stat & STATUS_CROSSOVER) == STATUS_CROSSOVER)
			port_info->status[0] = CABLE_CROSSED;
		else if ((stat & STATUS_REVERSED) == STATUS_REVERSED)
			port_info->status[0] = CABLE_REVERSED;
		return;
	}

	/* Put in 10 Mbps mode. */
	port_r(sw, port, P_PHY_CTRL, &ctrl);
	data = ctrl;
	data &= ~(PORT_AUTO_NEG_ENABLE | PORT_FORCE_100_MBIT |
		PORT_FORCE_FULL_DUPLEX);
#if (SW_SIZE == (1))
	port_w(sw, port, P_PHY_CTRL, data);

	port_r(sw, port, P_NEG_RESTART_CTRL, &data);
#endif
	crossover = data;

	for (i = 1; i <= 2; i++) {
		data = crossover;

		/* Disable auto MDIX. */
		data |= PORT_AUTO_MDIX_DISABLE;
		if (0 == i)
			data &= ~PORT_FORCE_MDIX;
		else
			data |= PORT_FORCE_MDIX;

		/* Disable transmitter. */
		data |= PORT_TX_DISABLE;
		port_w(sw, port, P_NEG_RESTART_CTRL, data);

		/* Wait at most 1 second.*/
		delay_milli(100);

		/* Enable transmitter. */
		data &= ~PORT_TX_DISABLE;
		port_w(sw, port, P_NEG_RESTART_CTRL, data);

		/* Start cable diagnostic test. */
		port_r(sw, port, REG_PORT_LINK_MD_CTRL, &data);
		data |= PORT_START_CABLE_DIAG;
		port_w(sw, port, REG_PORT_LINK_MD_CTRL, data);
		timeout = PHY_RESET_TIMEOUT;
		do {
			if (!--timeout)
				break;
			delay_milli(10);
			port_r(sw, port, REG_PORT_LINK_MD_CTRL, &data);
		} while ((data & PORT_START_CABLE_DIAG));

		port_info->length[i] = 0;
		port_info->status[i] = CABLE_UNKNOWN;

		if (!(data & PORT_START_CABLE_DIAG)) {
#if (SW_SIZE == (1))
			port_r8(sw, port, REG_PORT_LINK_MD_RESULT, &link);
			len = data & PORT_CABLE_FAULT_COUNTER_H;
			len <<= 16;
			len |= link;
#else
			len = data & PORT_CABLE_FAULT_COUNTER;
#endif
			port_info->length[i] = len *
				CABLE_LEN_MULTIPLIER;
			if (data & PORT_CABLE_10M_SHORT)
				port_info->length[i] = 1;
			data &= PORT_CABLE_DIAG_RESULT;
			switch (data) {
			case PORT_CABLE_STAT_NORMAL:
				port_info->status[i] = CABLE_GOOD;
				port_info->length[i] = 1;
				break;
			case PORT_CABLE_STAT_OPEN:
				port_info->status[i] = CABLE_OPEN;
				break;
			case PORT_CABLE_STAT_SHORT:
				port_info->status[i] = CABLE_SHORT;
				break;
			}
		}
	}

	port_w(sw, port, P_PHY_CTRL, ctrl);
	if (ctrl & PORT_AUTO_NEG_ENABLE) {
		crossover |= PORT_AUTO_NEG_RESTART;
		port_w(sw, port, P_NEG_RESTART_CTRL, crossover);
	}

	port_info->length[0] = port_info->length[1];
	port_info->status[0] = port_info->status[1];
	for (i = 2; i < 3; i++) {
		if (CABLE_GOOD == port_info->status[0]) {
			if (port_info->status[i] != CABLE_GOOD) {
				port_info->status[0] = port_info->status[i];
				port_info->length[0] = port_info->length[i];
				break;
			}
		}
	}
}

/* -------------------------------------------------------------------------- */

static void get_sw_mib_counters(struct ksz_sw *sw, int first, int cnt,
	u64 *counter)
{
	int i;
	int mib;
	int port;
	struct ksz_port_mib *port_mib;

	memset(counter, 0, sizeof(u64) * TOTAL_SWITCH_COUNTER_NUM);
	for (i = 0, port = first; i < cnt; i++, port++) {
		port_mib = &sw->port_mib[port];
		for (mib = port_mib->mib_start; mib < sw->mib_cnt; mib++)
			counter[mib] += port_mib->counter[mib];
	}
}

#define MIB_8863_RX_LO_PRIORITY		0x00
#define MIB_8863_RX_HI_PRIORITY		0x01
#define MIB_8863_RX_UNDERSIZE		0x02
#define MIB_8863_RX_FRAGMENT		0x03
#define MIB_8863_RX_OVERSIZE		0x04
#define MIB_8863_RX_JABBER		0x05
#define MIB_8863_RX_SYMBOL_ERR		0x06
#define MIB_8863_RX_CRC_ERR		0x07
#define MIB_8863_RX_ALIGNMENT_ERR	0x08
#define MIB_8863_RX_CTRL_8808		0x09
#define MIB_8863_RX_PAUSE		0x0A
#define MIB_8863_RX_BROADCAST		0x0B
#define MIB_8863_RX_MULTICAST		0x0C
#define MIB_8863_RX_UNICAST		0x0D
#define MIB_8863_RX_OCTET_64		0x0E
#define MIB_8863_RX_OCTET_65_127	0x0F
#define MIB_8863_RX_OCTET_128_255	0x10
#define MIB_8863_RX_OCTET_256_511	0x11
#define MIB_8863_RX_OCTET_512_1023	0x12
#define MIB_8863_RX_OCTET_1024_1522	0x13
#define MIB_8863_TX_LO_PRIORITY		0x14
#define MIB_8863_TX_HI_PRIORITY		0x15
#define MIB_8863_TX_LATE_COLLISION	0x16
#define MIB_8863_TX_PAUSE		0x17
#define MIB_8863_TX_BROADCAST		0x18
#define MIB_8863_TX_MULTICAST		0x19
#define MIB_8863_TX_UNICAST		0x1A
#define MIB_8863_TX_DEFERRED		0x1B
#define MIB_8863_TX_TOTAL_COLLISION	0x1C
#define MIB_8863_TX_EXCESS_COLLISION	0x1D
#define MIB_8863_TX_SINGLE_COLLISION	0x1E
#define MIB_8863_TX_MULTI_COLLISION	0x1F

#define MIB_8863_RX_DROPPED_PACKET	0x20
#define MIB_8863_TX_DROPPED_PACKET	0x21

static struct {
	char string[20];
} mib_names[TOTAL_SWITCH_COUNTER_NUM] = {
	{ "rx_lo        " },
	{ "rx_hi        " },
	{ "rx_undersize" },
	{ "rx_fragments" },
	{ "rx_oversize" },
	{ "rx_jabbers" },
	{ "rx_symbol_err" },
	{ "rx_crc_err" },
	{ "rx_align_err" },
	{ "rx_mac_ctrl" },
	{ "rx_pause" },
	{ "rx_bcast" },
	{ "rx_mcast" },
	{ "rx_ucast" },
	{ "rx_64_or_less" },
	{ "rx_65_127" },
	{ "rx_128_255" },
	{ "rx_256_511" },
	{ "rx_512_1023" },
	{ "rx_1024_1522" },

	{ "tx_lo        " },
	{ "tx_hi        " },
	{ "tx_late_col" },
	{ "tx_pause" },
	{ "tx_bcast" },
	{ "tx_mcast" },
	{ "tx_ucast" },
	{ "tx_deferred" },
	{ "tx_total_col" },
	{ "tx_exc_col" },
	{ "tx_single_col" },
	{ "tx_mult_col" },

	{ "rx_discards" },
	{ "tx_discards" },
#if 0
	{ "1234567890123456789012345678901" }
#endif
};

static struct {
	int rx;
	int tx;
} mib_display[17] = {
	{ MIB_8863_RX_LO_PRIORITY, MIB_8863_TX_LO_PRIORITY },
	{ MIB_8863_RX_HI_PRIORITY, MIB_8863_TX_HI_PRIORITY },
	{ MIB_8863_RX_PAUSE, MIB_8863_TX_PAUSE },
	{ MIB_8863_RX_BROADCAST, MIB_8863_TX_BROADCAST },
	{ MIB_8863_RX_MULTICAST, MIB_8863_TX_MULTICAST },
	{ MIB_8863_RX_UNICAST, MIB_8863_TX_UNICAST },
	{ MIB_8863_RX_OCTET_64, MIB_8863_RX_OCTET_65_127 },
	{ MIB_8863_RX_OCTET_128_255, MIB_8863_RX_OCTET_256_511 },
	{ MIB_8863_RX_OCTET_512_1023, MIB_8863_RX_OCTET_1024_1522 },
	{ MIB_8863_RX_UNDERSIZE, MIB_8863_RX_OVERSIZE },
	{ MIB_8863_RX_FRAGMENT, MIB_8863_RX_JABBER },
	{ MIB_8863_RX_SYMBOL_ERR, MIB_8863_RX_CRC_ERR },
	{ MIB_8863_RX_ALIGNMENT_ERR, MIB_8863_RX_CTRL_8808 },
	{ MIB_8863_TX_LATE_COLLISION, MIB_8863_TX_DEFERRED },
	{ MIB_8863_TX_TOTAL_COLLISION, MIB_8863_TX_EXCESS_COLLISION },
	{ MIB_8863_TX_SINGLE_COLLISION, MIB_8863_TX_MULTI_COLLISION },
	{ MIB_8863_RX_DROPPED_PACKET, MIB_8863_TX_DROPPED_PACKET },
};

static int display_sw_mib_counters(struct ksz_sw *sw, int first, int cnt,
	char *buf)
{
	int mib;
	int len = 0;
	u64 counter[TOTAL_SWITCH_COUNTER_NUM];

	get_sw_mib_counters(sw, first, cnt, counter);
	for (mib = 0; mib < 17; mib++) {
		int rx = mib_display[mib].rx;
		int tx = mib_display[mib].tx;
		if (buf)
			len += sprintf(buf + len,
				"%s\t= %-10lu\t%s\t= %-10lu\n",
				mib_names[rx].string,
				(unsigned long) counter[rx],
				mib_names[tx].string,
				(unsigned long) counter[tx]);
		else
			printk(KERN_INFO "%s\t= %-10lu\t%s\t= %-10lu\n",
				mib_names[rx].string,
				(unsigned long) counter[rx],
				mib_names[tx].string,
				(unsigned long) counter[tx]);
	}
	return len;
}

/* -------------------------------------------------------------------------- */

struct lan_attributes {
	int info;
	int duplex;
	int speed;
	int force;
	int flow_ctrl;
	int features;
	int overrides;
	int mib;
	int reg;
	int vid;
	int dynamic_table;
	int static_table;
	int vlan_table;
	int aging;
	int fast_aging;
	int link_aging;
	int bcast_per;
	int mcast_storm;
	int diffserv_map;
	int p_802_1p_map;
	int vlan;
	int null_vid;
	int macaddr;
	int mirror_mode;
	int tail_tag;
	int igmp_snoop;
	int ipv6_mld_snoop;
	int ipv6_mld_option;
	int aggr_backoff;
	int no_exc_drop;
	int buf_reserve;
	int huge_packet;
	int legal_packet;
	int length_check;
	int back_pressure;
	int sw_flow_ctrl;
	int sw_half_duplex;
	int sw_10_mbit;
	int rx_flow_ctrl;
	int tx_flow_ctrl;
	int fair_flow_ctrl;
	int vlan_bound;
	int fw_unk_dest;
	int ins_tag_0_1;
	int ins_tag_0_2;
	int ins_tag_1_0;
	int ins_tag_1_2;
	int ins_tag_2_0;
	int ins_tag_2_1;
	int pass_all;
	int pass_pause;
	int phy_addr;
};

struct sw_attributes {
	int mib;
	int vid;
	int member;
	int bcast_storm;
	int diffserv;
	int p_802_1p;
	int port_based;
	int non_vid;
	int ingress;
	int ins_tag;
	int rmv_tag;
	int double_tag;
	int drop_tagged;
	int replace_prio;
	int rx;
	int tx;
	int learn;
	int prio_queue;
	int tx_p0_ctrl;
	int tx_p1_ctrl;
	int tx_p2_ctrl;
	int tx_p3_ctrl;
	int tx_p0_ratio;
	int tx_p1_ratio;
	int tx_p2_ratio;
	int tx_p3_ratio;
	int prio_rate;
	int rx_limit;
	int cnt_ifg;
	int cnt_pre;
	int rx_p0_rate;
	int rx_p1_rate;
	int rx_p2_rate;
	int rx_p3_rate;
	int tx_p0_rate;
	int tx_p1_rate;
	int tx_p2_rate;
	int tx_p3_rate;
	int mirror_port;
	int mirror_rx;
	int mirror_tx;
	int back_pressure;
	int force_flow_ctrl;
	int fw_unk_dest;
	int fw_inv_vid;

	int duplex;
	int speed;
	int linkmd;
	int macaddr;
	int src_filter_0;
	int src_filter_1;
};

struct static_mac_attributes {
	int fid;
	int use_fid;
	int override;
	int valid;
	int ports;
	int addr;
};

struct vlan_attributes {
	int valid;
	int member;
	int fid;
	int vid;
};

static ssize_t display_sw_info(int cnt, char *buf, ssize_t len)
{
	len += sprintf(buf + len, "duplex:\t\t");
	len += sprintf(buf + len, "0 for auto; ");
	len += sprintf(buf + len,
		"set to 1 for half-duplex; 2, full-duplex\n");
	len += sprintf(buf + len, "speed:\t\t");
	len += sprintf(buf + len,
		"0 for auto; set to 10 or 100\n");
	len += sprintf(buf + len, "force:\t\t");
	len += sprintf(buf + len,
		"set to 1 to force link to specific speed setting\n");
	len += sprintf(buf + len, "flow_ctrl:\t");
	len += sprintf(buf + len,
		"set to 0 to disable flow control\n");
	len += sprintf(buf + len, "mib:\t\t");
	len += sprintf(buf + len,
		"display the MIB table\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	if (TOTAL_PORT_NUM != cnt)
		return len;

	len += sprintf(buf + len, "\ndynamic_table:\t");
	len += sprintf(buf + len,
		"display the switch's dynamic MAC table\n");
	len += sprintf(buf + len, "static_table:\t");
	len += sprintf(buf + len,
		"display the switch's static MAC table\n");
	len += sprintf(buf + len, "vlan_table:\t");
	len += sprintf(buf + len,
		"display the switch's VLAN table\n");

	len += sprintf(buf + len, "\naging:\t\t");
	len += sprintf(buf + len,
		"disable/enable aging\n");
	len += sprintf(buf + len, "fast_aging:\t");
	len += sprintf(buf + len,
		"disable/enable fast aging\n");
	len += sprintf(buf + len, "link_aging:\t");
	len += sprintf(buf + len,
		"disable/enable link change auto aging\n");

	len += sprintf(buf + len, "\nbcast_per:\t");
	len += sprintf(buf + len,
		"set broadcast storm percentage\n");
	len += sprintf(buf + len, "mcast_storm:\t");
	len += sprintf(buf + len,
		"disable multicast storm protection\n");
	len += sprintf(buf + len, "diffserv_map:\t");
	len += sprintf(buf + len,
		"set DiffServ value.  Use \"decimal=hexadecimal\" format\n");
	len += sprintf(buf + len, "p_802_1p_map:\t");
	len += sprintf(buf + len,
		"set 802.1p value.  Use \"decimal=hexadecimal\" format\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	len += sprintf(buf + len, "\nvlan:\t\t");
	len += sprintf(buf + len,
		"disable/enable 802.1Q VLAN\n");
	len += sprintf(buf + len, "null_vid:\t");
	len += sprintf(buf + len,
		"set to 1 to replace null vid\n");
	len += sprintf(buf + len, "macaddr:\t");
	len += sprintf(buf + len,
		"set switch MAC address\n");
	len += sprintf(buf + len, "mirror_mode:\t");
	len += sprintf(buf + len,
		"set to 1 to use mirror rx AND tx mode\n");
	len += sprintf(buf + len, "tail_tag:\t");
	len += sprintf(buf + len,
		"disable/enable tail tagging\n");

	len += sprintf(buf + len, "\nigmp_snoop:\t");
	len += sprintf(buf + len,
		"disable/enable IGMP snooping\n");
	len += sprintf(buf + len, "ipv6_mld_snoop:\t");
	len += sprintf(buf + len,
		"disable/enable IPv6 MLD snooping\n");
	len += sprintf(buf + len, "ipv6_mld_option:");
	len += sprintf(buf + len,
		"disable/enable IPv6 MLD option snooping\n");

	len += sprintf(buf + len, "\naggr_backoff:\t");
	len += sprintf(buf + len,
		"disable/enable aggressive backoff in half-duplex mode\n");
	len += sprintf(buf + len, "no_exc_drop:\t");
	len += sprintf(buf + len,
		"disable/enable no excessive collision drop\n");
	len += sprintf(buf + len, "buf_reserve:\t");
	len += sprintf(buf + len,
		"disable/enable buffer reserve\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	len += sprintf(buf + len, "\nhuge_packet:\t");
	len += sprintf(buf + len,
		"disable/enable huge packet support\n");
	len += sprintf(buf + len, "legal_packet:\t");
	len += sprintf(buf + len,
		"disable/enable legal packet\n");
	len += sprintf(buf + len, "length_check:\t");
	len += sprintf(buf + len,
		"disable/enable legal packet length check\n");

	len += sprintf(buf + len, "\nback_pressure:\t");
	len += sprintf(buf + len,
		"set back pressure mode\n");
	len += sprintf(buf + len, "sw_flow_ctrl:\t");
	len += sprintf(buf + len,
		"disable/enable switch host port flow control\n");
	len += sprintf(buf + len, "sw_half_duplex:\t");
	len += sprintf(buf + len,
		"disable/enable switch host port half-duplex mode\n");
#ifdef SWITCH_10_MBIT
	len += sprintf(buf + len, "sw_10_mbit:\t");
	len += sprintf(buf + len,
		"disable/enable switch host port 10Mbit mode\n");
#endif
	len += sprintf(buf + len, "rx_flow_ctrl:\t");
	len += sprintf(buf + len,
		"disable/enable receive flow control\n");
	len += sprintf(buf + len, "tx_flow_ctrl:\t");
	len += sprintf(buf + len,
		"disable/enable transmit flow control\n");
	len += sprintf(buf + len, "fair_flow_ctrl:\t");
	len += sprintf(buf + len,
		"disable/enable fair flow control mode\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	len += sprintf(buf + len, "vlan_bound:\t");
	len += sprintf(buf + len,
		"disable/enable unicast VLAN boundary\n");
	len += sprintf(buf + len, "fw_unk_dest:\t");
	len += sprintf(buf + len,
		"disable/enable unknown destination address forwarding\n");

	len += sprintf(buf + len, "\nins_tag_0_1:\t");
	len += sprintf(buf + len,
		"set to 1 to insert tag from port 1 to 2\n");
	len += sprintf(buf + len, "ins_tag_0_2:\t");
	len += sprintf(buf + len,
		"set to 1 to insert tag from port 1 to 3\n");
	len += sprintf(buf + len, "ins_tag_1_0:\t");
	len += sprintf(buf + len,
		"set to 1 to insert tag from port 2 to 1\n");
	len += sprintf(buf + len, "ins_tag_1_2:\t");
	len += sprintf(buf + len,
		"set to 1 to insert tag from port 2 to 3\n");
	len += sprintf(buf + len, "ins_tag_2_0:\t");
	len += sprintf(buf + len,
		"set to 1 to insert tag from port 3 to 1\n");
	len += sprintf(buf + len, "ins_tag_2_1:\t");
	len += sprintf(buf + len,
		"set to 1 to insert tag from port 3 to 2\n");

	len += sprintf(buf + len, "\npass_all:\t");
	len += sprintf(buf + len,
		"set to 1 to pass all frames for debugging\n");
	len += sprintf(buf + len, "pass_pause:\t");
	len += sprintf(buf + len,
		"set to 1 to pass PAUSE frames for debugging\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	len += sprintf(buf + len, "\nswitch port settings:\n");
	len += sprintf(buf + len, "duplex:\t\t");
	len += sprintf(buf + len,
		"display the port's duplex setting\n");
	len += sprintf(buf + len, "speed:\t\t");
	len += sprintf(buf + len,
		"display the port's link speed\n");
	len += sprintf(buf + len, "linkmd:\t\t");
	len += sprintf(buf + len,
		"write to start LinkMD test.  read for result\n");
	len += sprintf(buf + len, "mib:\t\t");
	len += sprintf(buf + len,
		"display the port's MIB table\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	len += sprintf(buf + len, "vid:\t\t");
	len += sprintf(buf + len,
		"set default VID value\n");
	len += sprintf(buf + len, "member:\t\t");
	len += sprintf(buf + len,
		"set VLAN membership\n");

	len += sprintf(buf + len, "bcast_storm:\t");
	len += sprintf(buf + len,
		"disable/enable broadcast storm protection\n");
	len += sprintf(buf + len, "diffserv:\t");
	len += sprintf(buf + len,
		"disable/enable DiffServ priority\n");
	len += sprintf(buf + len, "p_802_1p:\t");
	len += sprintf(buf + len,
		"disable/enable 802.1p priority\n");

	len += sprintf(buf + len, "port_based:\t");
	len += sprintf(buf + len,
		"disable/enable port-based priority\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	len += sprintf(buf + len, "prio_queue:\t");
	len += sprintf(buf + len,
		"disable/enable priority queue\n");
	len += sprintf(buf + len, "tx_p0_ctrl:\t");
	len += sprintf(buf + len,
		"set priority queue 0 control\n");
	len += sprintf(buf + len, "tx_p1_ctrl:\t");
	len += sprintf(buf + len,
		"set priority queue 1 control\n");
	len += sprintf(buf + len, "tx_p2_ctrl:\t");
	len += sprintf(buf + len,
		"set priority queue 2 control\n");
	len += sprintf(buf + len, "tx_p3_ctrl:\t");
	len += sprintf(buf + len,
		"set priority queue 3 control\n");
	len += sprintf(buf + len, "tx_p0_ratio:\t");
	len += sprintf(buf + len,
		"set priority queue 0 ratio\n");
	len += sprintf(buf + len, "tx_p1_ratio:\t");
	len += sprintf(buf + len,
		"set priority queue 1 ratio\n");
	len += sprintf(buf + len, "tx_p2_ratio:\t");
	len += sprintf(buf + len,
		"set priority queue 2 ratio\n");
	len += sprintf(buf + len, "tx_p3_ratio:\t");
	len += sprintf(buf + len,
		"set priority queue 3 ratio\n");
	len += sprintf(buf + len, "prio_rate:\t");
	len += sprintf(buf + len,
		"disable/enable priority queue rate limiting\n");
	len += sprintf(buf + len, "rx_limit:\t");
	len += sprintf(buf + len,
		"set rx rate limiting mode\n");
	len += sprintf(buf + len, "cnt_ifg:\t");
	len += sprintf(buf + len,
		"set to 1 to count IPG\n");
	len += sprintf(buf + len, "cnt_pre:\t");
	len += sprintf(buf + len,
		"set to 1 to count preamble\n");
	len += sprintf(buf + len, "rx_p0_rate:\t");
	len += sprintf(buf + len,
		"set rx priority queue 0 rate in 64Kbps unit\n");
	len += sprintf(buf + len, "rx_p1_rate:\t");
	len += sprintf(buf + len,
		"set rx priority queue 1 rate in 64Kbps unit\n");
	len += sprintf(buf + len, "rx_p2_rate:\t");
	len += sprintf(buf + len,
		"set rx priority queue 2 rate in 64Kbps unit\n");
	len += sprintf(buf + len, "rx_p3_rate:\t");
	len += sprintf(buf + len,
		"set rx priority queue 3 rate in 64Kbps unit\n");
	len += sprintf(buf + len, "tx_p0_rate:\t");
	len += sprintf(buf + len,
		"set tx priority queue 0 rate in 64Kbps unit\n");
	len += sprintf(buf + len, "tx_p1_rate:\t");
	len += sprintf(buf + len,
		"set tx priority queue 1 rate in 64Kbps unit\n");
	len += sprintf(buf + len, "tx_p2_rate:\t");
	len += sprintf(buf + len,
		"set tx priority queue 2 rate in 64Kbps unit\n");
	len += sprintf(buf + len, "tx_p3_rate:\t");
	len += sprintf(buf + len,
		"set tx priority queue 3 rate in 64Kbps unit\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	len += sprintf(buf + len, "rx:\t\t");
	len += sprintf(buf + len,
		"disable/enable rx\n");
	len += sprintf(buf + len, "tx:\t\t");
	len += sprintf(buf + len,
		"disable/enable tx\n");
	len += sprintf(buf + len, "learn:\t\t");
	len += sprintf(buf + len,
		"disable/enable learning\n");

	len += sprintf(buf + len, "mirror_port:\t");
	len += sprintf(buf + len,
		"disable/enable mirror port\n");
	len += sprintf(buf + len, "mirror_rx:\t");
	len += sprintf(buf + len,
		"disable/enable mirror receive\n");
	len += sprintf(buf + len, "mirror_tx:\t");
	len += sprintf(buf + len,
		"disable/enable mirror transmit\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	len += sprintf(buf + len, "\nnon_vid:\t");
	len += sprintf(buf + len,
		"set to 1 to discard non-VID packets\n");
	len += sprintf(buf + len, "ingress:\t");
	len += sprintf(buf + len,
		"disable/enable ingress VLAN filtering\n");
	len += sprintf(buf + len, "ins_tag:\t");
	len += sprintf(buf + len,
		"disable/enable insert VLAN tag feature\n");
	len += sprintf(buf + len, "rmv_tag:\t");
	len += sprintf(buf + len,
		"disable/enable remove VLAN tag feature\n");
	len += sprintf(buf + len, "drop_tagged:\t");
	len += sprintf(buf + len,
		"disable/enable drop tagged packet feature\n");
	len += sprintf(buf + len, "replace_prio:\t");
	len += sprintf(buf + len,
		"disable/enable replace 802.1p priority feature\n");
	len += sprintf(buf + len, "back_pressure:\t");
	len += sprintf(buf + len,
		"disable/enable back pressure in half-duplex mode\n");
	len += sprintf(buf + len, "force_flow_ctrl:");
	len += sprintf(buf + len,
		"set to 1 to force flow control\n");
	len += sprintf(buf + len, "fw_unk_dest:\t");
	len += sprintf(buf + len,
		"set to 1 to forward unknown destination address packets\n");
	len += sprintf(buf + len, "fw_inv_vid:\t");
	len += sprintf(buf + len,
		"set to 1 to forward invalid VID packets\n");
	len += sprintf(buf + len, "\nmacaddr:\t");
	len += sprintf(buf + len,
		"set port MAC address\n");
	len += sprintf(buf + len, "src_filter_0:\t");
	len += sprintf(buf + len,
		"disable/enable source filtering port 1 MAC address\n");
	len += sprintf(buf + len, "src_filter_1:\t");
	len += sprintf(buf + len,
		"disable/enable source filtering port 2 MAC address\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	len += sprintf(buf + len, "\nstatic MAC table:\n");
	len += sprintf(buf + len, "addr:\t\t");
	len += sprintf(buf + len,
		"set MAC address\n");
	len += sprintf(buf + len, "ports:\t\t");
	len += sprintf(buf + len,
		"set destination ports\n");
	len += sprintf(buf + len, "override:\t");
	len += sprintf(buf + len,
		"set override bit\n");
	len += sprintf(buf + len, "use_fid:\t");
	len += sprintf(buf + len,
		"set use FID bit\n");
	len += sprintf(buf + len, "fid:\t\t");
	len += sprintf(buf + len,
		"set FID\n");
	len += sprintf(buf + len, "valid:\t\t");
	len += sprintf(buf + len,
		"set valid bit and write to table\n");
	len += sprintf(buf + len, "\nVLAN table:\n");
	len += sprintf(buf + len, "vid:\t\t");
	len += sprintf(buf + len,
		"set VID\n");
	len += sprintf(buf + len, "fid:\t\t");
	len += sprintf(buf + len,
		"set FID\n");
	len += sprintf(buf + len, "member:\t\t");
	len += sprintf(buf + len,
		"set membership\n");
	len += sprintf(buf + len, "valid:\t\t");
	len += sprintf(buf + len,
		"set valid bit and write to table\n");
	printk(KERN_INFO "%s", buf);
	len = 0;

	return len;
}

static ssize_t sysfs_sw_read(struct ksz_sw *sw, int proc_num,
	struct ksz_port *port, ssize_t len, char *buf)
{
	int i;
	int j;
	u16 map;
	struct ksz_sw_info *info = sw->info;

	switch (proc_num) {
	case PROC_SW_INFO:
		len = display_sw_info(sw->mib_port_cnt, buf, len);
		break;
	case PROC_SET_SW_DUPLEX:
		if (!port)
			break;
		len += sprintf(buf + len, "%u; ", port->duplex);
		if (media_connected == port->linked->state) {
			if (1 == port->linked->duplex)
				len += sprintf(buf + len, "half-duplex\n");
			else if (2 == port->linked->duplex)
				len += sprintf(buf + len, "full-duplex\n");
		} else
			len += sprintf(buf + len, "unlinked\n");
		break;
	case PROC_SET_SW_SPEED:
		if (!port)
			break;
		len += sprintf(buf + len, "%u; ", port->speed);
		if (media_connected == port->linked->state)
			len += sprintf(buf + len, "%u\n",
				port->linked->tx_rate / TX_RATE_UNIT);
		else
			len += sprintf(buf + len, "unlinked\n");
		break;
	case PROC_SET_SW_FORCE:
		if (!port)
			break;
		len += sprintf(buf + len, "%u\n", port->force_link);
		break;
	case PROC_SET_SW_FLOW_CTRL:
		if (!port)
			break;
		len += sprintf(buf + len, "%u; ", port->flow_ctrl);
		switch (port->flow_ctrl) {
		case PHY_FLOW_CTRL:
			len += sprintf(buf + len, "flow control\n");
			break;
		case PHY_TX_ONLY:
			len += sprintf(buf + len, "tx only\n");
			break;
		case PHY_RX_ONLY:
			len += sprintf(buf + len, "rx only\n");
			break;
		default:
			len += sprintf(buf + len, "no flow control\n");
			break;
		}
		break;
	case PROC_SET_SW_MIB:
		if (!port)
			break;
		len += display_sw_mib_counters(sw, port->first_port,
			port->mib_port_cnt, buf + len);
		break;
	case PROC_SET_BROADCAST_STORM:
		len += sprintf(buf + len, "%u%%\n", info->broad_per);
		break;
	case PROC_SET_DIFFSERV:
		for (i = 0; i < DIFFSERV_ENTRIES / KS_PRIORITY_IN_REG; i++) {
			len += sprintf(buf + len, "%2u=",
				i * KS_PRIORITY_IN_REG);
			map = info->diffserv[i];
			for (j = 0; j < KS_PRIORITY_IN_REG; j++) {
				len += sprintf(buf + len, "%u ",
					map & KS_PRIORITY_MASK);
				map >>= KS_PRIORITY_SHIFT;
			}
			len += sprintf(buf + len, "\t"SW_SIZE_STR"\n",
				info->diffserv[i]);
		}
		break;
	case PROC_SET_802_1P:
		for (i = 0; i < PRIO_802_1P_ENTRIES / KS_PRIORITY_IN_REG; i++) {
			len += sprintf(buf + len, "%u=",
				i * KS_PRIORITY_IN_REG);
			map = info->p_802_1p[i];
			for (j = 0; j < KS_PRIORITY_IN_REG; j++) {
				len += sprintf(buf + len, "%u ",
					map & KS_PRIORITY_MASK);
				map >>= KS_PRIORITY_SHIFT;
			}
			len += sprintf(buf + len, "\t"SW_SIZE_STR"\n",
				info->p_802_1p[i]);
		}
		break;
#ifdef SWITCH_PORT_PHY_ADDR_MASK
	case PROC_SET_PHY_ADDR:
		len += sprintf(buf + len, "%u\n",
			info->phy_addr);
		break;
#endif
	case PROC_SET_SW_VID:
		len += sprintf(buf + len, "0x%04x\n", sw->vid);
		break;
	case PROC_SET_SW_FEATURES:
		len += sprintf(buf + len, "%08x:\n", sw->features);
		len += sprintf(buf + len, "\t%08x = STP support\n",
			STP_SUPPORT);
		len += sprintf(buf + len, "\t%08x = VLAN port forwarding\n",
			VLAN_PORT);
		len += sprintf(buf + len, "\t%08x = VLAN port remove tag\n",
			VLAN_PORT_REMOVE_TAG);
		len += sprintf(buf + len, "\t%08x = different MAC addresses\n",
			DIFF_MAC_ADDR);
		break;
	case PROC_SET_SW_OVERRIDES:
		len += sprintf(buf + len, "%08x:\n", sw->overrides);
		len += sprintf(buf + len, "\t%08x = flow control\n",
			PAUSE_FLOW_CTRL);
		len += sprintf(buf + len, "\t%08x = fast aging\n",
			FAST_AGING);
		len += sprintf(buf + len, "\t%08x = tag is removed\n",
			TAG_REMOVE);
		len += sprintf(buf + len, "\t%08x = tail tagging\n",
			TAIL_TAGGING);
		break;
	}
	return len;
}

static ssize_t sysfs_sw_read_hw(struct ksz_sw *sw, int proc_num, ssize_t len,
	char *buf)
{
	u8 data[8];

	switch (proc_num) {
	case PROC_SET_AGING:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_1,
				SWITCH_AGING_ENABLE));
		break;
	case PROC_SET_FAST_AGING:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_1,
				SWITCH_FAST_AGING));
		break;
	case PROC_SET_LINK_AGING:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_LINK_AGING_CTRL,
				SWITCH_LINK_AUTO_AGING));
		break;
	case PROC_SET_MULTICAST_STORM:
		len += sprintf(buf + len, "%u\n",
			!sw_chk(sw, REG_SWITCH_CTRL_2,
				MULTICAST_STORM_DISABLE));
		break;
	case PROC_ENABLE_VLAN:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_MIRROR_CTRL,
				SWITCH_VLAN_ENABLE));
		break;
	case PROC_SET_REPLACE_NULL_VID:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_REPLACE_VID_CTRL,
				SWITCH_REPLACE_VID));
		break;
	case PROC_SET_MAC_ADDR:
		sw_get_addr(sw, data);
		len += sprintf(buf + len, "%02X:%02X:%02X:%02X:%02X:%02X\n",
			data[0], data[1], data[2], data[3], data[4], data[5]);
		break;
	case PROC_SET_MIRROR_MODE:
		len += sprintf(buf + len, "%u\n",
			sw_chk_mirror_rx_tx(sw));
		break;
	case PROC_SET_IGMP_SNOOP:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_MIRROR_CTRL,
				SWITCH_IGMP_SNOOP));
		break;
#ifdef SWITCH_IPV6_MLD_SNOOP
	case PROC_SET_IPV6_MLD_SNOOP:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_MIRROR_CTRL,
				SWITCH_IPV6_MLD_SNOOP));
		break;
	case PROC_SET_IPV6_MLD_OPTION:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_MIRROR_CTRL,
				SWITCH_IPV6_MLD_OPTION));
		break;
#endif
	case PROC_SET_TAIL_TAG:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_TAIL_TAG_CTRL,
				SWITCH_TAIL_TAG_ENABLE));
		break;
	case PROC_SET_AGGR_BACKOFF:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_1,
				SWITCH_AGGR_BACKOFF));
		break;
	case PROC_SET_NO_EXC_DROP:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_2,
				NO_EXC_COLLISION_DROP));
		break;
#ifdef SWITCH_BUF_RESERVE
	case PROC_SET_BUFFER_RESERVE:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_2,
				SWITCH_BUF_RESERVE));
		break;
#endif
	case PROC_SET_VLAN_BOUNDARY:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_2,
				UNICAST_VLAN_BOUNDARY));
		break;
	case PROC_SET_HUGE_PACKET:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_2,
				SWITCH_HUGE_PACKET));
		break;
	case PROC_SET_LEGAL_PACKET:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_2,
				SWITCH_LEGAL_PACKET));
		break;
	case PROC_SET_LENGTH_CHECK:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_1,
				SWITCH_CHECK_LENGTH));
		break;
	case PROC_SET_BACK_PRESSURE_MODE:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_2,
				SWITCH_BACK_PRESSURE));
		break;
	case PROC_SET_SWITCH_FLOW_CTRL:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_REPLACE_VID_CTRL,
				SWITCH_FLOW_CTRL));
		break;
	case PROC_SET_SWITCH_HALF_DUPLEX:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_REPLACE_VID_CTRL,
				SWITCH_HALF_DUPLEX));
		break;
#ifdef SWITCH_10_MBIT
	case PROC_SET_SWITCH_10_MBIT:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_REPLACE_VID_CTRL,
				SWITCH_10_MBIT));
		break;
#endif
	case PROC_SET_RX_FLOW_CTRL:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_1,
				SWITCH_RX_FLOW_CTRL));
		break;
	case PROC_SET_TX_FLOW_CTRL:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_1,
				SWITCH_TX_FLOW_CTRL));
		break;
	case PROC_SET_FAIR_FLOW_CTRL:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_2,
				FAIR_FLOW_CTRL));
		break;
	case PROC_SET_FORWARD_UNKNOWN_DEST:
		len += sprintf(buf + len, "%u\n",
			sw_chk_unk_dest(sw));
		break;
	case PROC_SET_INS_TAG_0_1:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_INS_SRC_PVID_CTRL,
				SWITCH_INS_TAG_1_PORT_2));
		break;
	case PROC_SET_INS_TAG_0_2:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_INS_SRC_PVID_CTRL,
				SWITCH_INS_TAG_1_PORT_3));
		break;
	case PROC_SET_INS_TAG_1_0:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_INS_SRC_PVID_CTRL,
				SWITCH_INS_TAG_2_PORT_1));
		break;
	case PROC_SET_INS_TAG_1_2:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_INS_SRC_PVID_CTRL,
				SWITCH_INS_TAG_2_PORT_3));
		break;
	case PROC_SET_INS_TAG_2_0:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_INS_SRC_PVID_CTRL,
				SWITCH_INS_TAG_3_PORT_1));
		break;
	case PROC_SET_INS_TAG_2_1:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_INS_SRC_PVID_CTRL,
				SWITCH_INS_TAG_3_PORT_2));
		break;
	case PROC_SET_PASS_ALL:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, REG_SWITCH_CTRL_1,
				SWITCH_PASS_ALL));
		break;
	case PROC_SET_PASS_PAUSE:
		len += sprintf(buf + len, "%u\n",
			sw_chk(sw, S_LINK_AGING_CTRL,
				SWITCH_PASS_PAUSE));
		break;
	case PROC_DYNAMIC:
		sw_d_dyn_mac_table(sw);
		break;
	case PROC_STATIC:
		sw_d_sta_mac_table(sw);
		break;
	case PROC_VLAN:
		sw_d_vlan_table(sw);
		break;
	}
	return len;
}

static int sysfs_sw_write(struct ksz_sw *sw, int proc_num,
	struct ksz_port *port, int num, const char *buf)
{
	int count;
	unsigned int val;
	u8 data[8];
	int processed = true;

	switch (proc_num) {
	case PROC_SW_INFO:
		sw_init(sw);
		break;
	case PROC_SET_SW_DUPLEX:
		if (!port)
			break;
		if (num <= 2)
			port->duplex = (u8) num;
		break;
	case PROC_SET_SW_SPEED:
		if (!port)
			break;
		if (0 == num || 10 == num || 100 == num)
			port->speed = (u8) num;
		break;
	case PROC_SET_SW_FORCE:
		if (!port)
			break;
		port->force_link = (u8) num;
		if (port->force_link)
			port_force_link_speed(port);
		else
			port_set_link_speed(port);
		break;
	case PROC_SET_SW_FLOW_CTRL:
		if (!port)
			break;
		if (num <= PHY_FLOW_CTRL)
			port->flow_ctrl = (u8) num;
		break;
	case PROC_SET_SW_MIB:
		for (count = 0; count < sw->mib_port_cnt; count++) {
			struct ksz_port_mib *mib = &sw->port_mib[count];

			memset((void *) mib->counter, 0, sizeof(u64) *
				TOTAL_SWITCH_COUNTER_NUM);
		}
		break;
	case PROC_SET_SW_REG:
		count = sscanf(buf, "%x=%x", (unsigned int *) &num, &val);
		if (1 == count)
			printk(KERN_INFO SW_SIZE_STR"\n",
				SW_R(sw, num));
		else if (2 == count)
			SW_W(sw, num, val);
		break;
	case PROC_SET_SW_VID:
		sw->vid = num;
		break;
	case PROC_SET_SW_FEATURES:
		if ('0' != buf[0] || 'x' != buf[1])
			sscanf(buf, "%x", &num);
		sw->features = num;
		break;
	case PROC_SET_SW_OVERRIDES:
		if ('0' != buf[0] || 'x' != buf[1])
			sscanf(buf, "%x", &num);
		sw->overrides = num;
		break;
	case PROC_DYNAMIC:
		sw_flush_dyn_mac_table(sw, TOTAL_PORT_NUM);
		break;
	case PROC_STATIC:
#if 0
		sw_cfg(sw, S_FLUSH_TABLE_CTRL, SWITCH_FLUSH_STA_MAC_TABLE, 1);
		sw_cfg(sw, S_FLUSH_TABLE_CTRL, SWITCH_FLUSH_STA_MAC_TABLE, 0);
#else
		sw_clr_sta_mac_table(sw);
#endif
		break;
	case PROC_SET_AGING:
		sw_cfg(sw, REG_SWITCH_CTRL_1, SWITCH_AGING_ENABLE, num);
		break;
	case PROC_SET_FAST_AGING:
		sw_cfg(sw, REG_SWITCH_CTRL_1, SWITCH_FAST_AGING, num);
		break;
	case PROC_SET_LINK_AGING:
		sw_cfg(sw, S_LINK_AGING_CTRL, SWITCH_LINK_AUTO_AGING, num);
		break;
	case PROC_SET_BROADCAST_STORM:
		hw_cfg_broad_storm(sw, num);
		break;
	case PROC_SET_MULTICAST_STORM:
		sw_cfg(sw, REG_SWITCH_CTRL_2, MULTICAST_STORM_DISABLE, !num);
		break;
	case PROC_SET_DIFFSERV:
		count = sscanf(buf, "%d=%x", (unsigned int *) &num, &val);
		if (2 == count)
			hw_cfg_tos_prio(sw, (u8) num, (u16) val);
		break;
	case PROC_SET_802_1P:
		count = sscanf(buf, "%d=%x", (unsigned int *) &num, &val);
		if (2 == count)
			hw_cfg_802_1p_prio(sw, (u8) num, (u16) val);
		break;
	case PROC_ENABLE_VLAN:
		if (!num)
			sw_dis_vlan(sw);
		else
			sw_ena_vlan(sw);
		break;
	case PROC_SET_REPLACE_NULL_VID:
		sw_cfg_replace_null_vid(sw, num);
		break;
	case PROC_SET_MAC_ADDR:
	{
		int i;
		int n[6];

		i = sscanf(buf, "%x:%x:%x:%x:%x:%x",
			&n[0], &n[1], &n[2], &n[3], &n[4], &n[5]);
		if (6 == i) {
			for (i = 0; i < 6; i++)
				data[i] = (u8) n[i];
			sw_set_addr(sw, data);
		}
		break;
	}
	case PROC_SET_MIRROR_MODE:
		sw_cfg_mirror_rx_tx(sw, num);
		break;
	case PROC_SET_IGMP_SNOOP:
		sw_cfg(sw, S_MIRROR_CTRL, SWITCH_IGMP_SNOOP, num);
		break;
#ifdef SWITCH_IPV6_MLD_SNOOP
	case PROC_SET_IPV6_MLD_SNOOP:
		sw_cfg(sw, S_MIRROR_CTRL, SWITCH_IPV6_MLD_SNOOP, num);
		break;
	case PROC_SET_IPV6_MLD_OPTION:
		sw_cfg(sw, S_MIRROR_CTRL, SWITCH_IPV6_MLD_OPTION, num);
		break;
#endif
	case PROC_SET_TAIL_TAG:
		sw_cfg(sw, S_TAIL_TAG_CTRL, SWITCH_TAIL_TAG_ENABLE, num);
		if (num)
			sw->overrides |= TAIL_TAGGING;
		else
			sw->overrides &= ~TAIL_TAGGING;
		break;
	case PROC_SET_AGGR_BACKOFF:
		sw_cfg(sw, REG_SWITCH_CTRL_1, SWITCH_AGGR_BACKOFF, num);
		break;
	case PROC_SET_NO_EXC_DROP:
		sw_cfg(sw, REG_SWITCH_CTRL_2, NO_EXC_COLLISION_DROP, num);
		break;
#ifdef SWITCH_BUF_RESERVE
	case PROC_SET_BUFFER_RESERVE:
		sw_cfg(sw, REG_SWITCH_CTRL_2, SWITCH_BUF_RESERVE, num);
		break;
#endif
	case PROC_SET_VLAN_BOUNDARY:
		sw_cfg(sw, REG_SWITCH_CTRL_2, UNICAST_VLAN_BOUNDARY, num);
		break;
	case PROC_SET_HUGE_PACKET:
		sw_cfg(sw, REG_SWITCH_CTRL_2, SWITCH_HUGE_PACKET, num);
		break;
	case PROC_SET_LEGAL_PACKET:
		sw_cfg(sw, REG_SWITCH_CTRL_2, SWITCH_LEGAL_PACKET, num);
		break;
	case PROC_SET_LENGTH_CHECK:
		sw_cfg(sw, REG_SWITCH_CTRL_1, SWITCH_CHECK_LENGTH, num);
		break;
	case PROC_SET_BACK_PRESSURE_MODE:
		sw_cfg(sw, REG_SWITCH_CTRL_2, SWITCH_BACK_PRESSURE, num);
		break;
	case PROC_SET_SWITCH_FLOW_CTRL:
		sw_cfg(sw, S_REPLACE_VID_CTRL, SWITCH_FLOW_CTRL, num);
		break;
	case PROC_SET_SWITCH_HALF_DUPLEX:
		sw_cfg(sw, S_REPLACE_VID_CTRL, SWITCH_HALF_DUPLEX, num);
		break;
#ifdef SWITCH_10_MBIT
	case PROC_SET_SWITCH_10_MBIT:
		sw_cfg(sw, S_REPLACE_VID_CTRL, SWITCH_10_MBIT, num);
		break;
#endif
	case PROC_SET_RX_FLOW_CTRL:
		sw_cfg(sw, REG_SWITCH_CTRL_1, SWITCH_RX_FLOW_CTRL, num);
		break;
	case PROC_SET_TX_FLOW_CTRL:
		sw_cfg(sw, REG_SWITCH_CTRL_1, SWITCH_TX_FLOW_CTRL, num);
		break;
	case PROC_SET_FAIR_FLOW_CTRL:
		sw_cfg(sw, REG_SWITCH_CTRL_2, FAIR_FLOW_CTRL, num);
		break;
	case PROC_SET_FORWARD_UNKNOWN_DEST:
		sw_cfg_unk_dest(sw, num);
		break;
	case PROC_SET_INS_TAG_0_1:
		sw_cfg(sw, S_INS_SRC_PVID_CTRL, SWITCH_INS_TAG_1_PORT_2, num);
		break;
	case PROC_SET_INS_TAG_0_2:
		sw_cfg(sw, S_INS_SRC_PVID_CTRL, SWITCH_INS_TAG_1_PORT_3, num);
		break;
	case PROC_SET_INS_TAG_1_0:
		sw_cfg(sw, S_INS_SRC_PVID_CTRL, SWITCH_INS_TAG_2_PORT_1, num);
		break;
	case PROC_SET_INS_TAG_1_2:
		sw_cfg(sw, S_INS_SRC_PVID_CTRL, SWITCH_INS_TAG_2_PORT_3, num);
		break;
	case PROC_SET_INS_TAG_2_0:
		sw_cfg(sw, S_INS_SRC_PVID_CTRL, SWITCH_INS_TAG_3_PORT_1, num);
		break;
	case PROC_SET_INS_TAG_2_1:
		sw_cfg(sw, S_INS_SRC_PVID_CTRL, SWITCH_INS_TAG_3_PORT_2, num);
		break;
	case PROC_SET_PASS_ALL:
		sw_cfg(sw, REG_SWITCH_CTRL_1, SWITCH_PASS_ALL, num);
		break;
	case PROC_SET_PASS_PAUSE:
		sw_cfg(sw, S_LINK_AGING_CTRL, SWITCH_PASS_PAUSE, num);
		break;
#ifdef SWITCH_PORT_PHY_ADDR_MASK
	case PROC_SET_PHY_ADDR:
		sw_set_phy_addr(sw, num);
		break;
#endif
	default:
		processed = false;
		break;
	}
	return processed;
}

static ssize_t sysfs_port_read(struct ksz_sw *sw, int proc_num, int port,
	ssize_t len, char *buf)
{
	struct ksz_port_cfg *port_cfg;
	struct ksz_port_info *port_info;

	port_cfg = &sw->info->port_cfg[port];
	port_info = &sw->port_info[port];
	switch (proc_num) {
	case PROC_GET_PORT_DUPLEX:
		if (media_connected == port_info->state) {
			if (1 == port_info->duplex)
				len += sprintf(buf + len, "half-duplex\n");
			else if (2 == port_info->duplex)
				len += sprintf(buf + len, "full-duplex\n");
		} else
			len += sprintf(buf + len, "unlinked\n");
		break;
	case PROC_GET_PORT_SPEED:
		if (media_connected == port_info->state)
			len += sprintf(buf + len, "%u\n",
				port_info->tx_rate / TX_RATE_UNIT);
		else
			len += sprintf(buf + len, "unlinked\n");
		break;
	case PROC_SET_PORT_MIB:
		len += display_sw_mib_counters(sw, port, 1, buf + len);
		break;
	case PROC_SET_LINK_MD:
		len += sprintf(buf + len, "%u:%u %u:%u %u:%u\n",
			port_info->length[0], port_info->status[0],
			port_info->length[1], port_info->status[1],
			port_info->length[2], port_info->status[2]);
		break;
	case PROC_SET_PORT_BASED:
		len += sprintf(buf + len, "%u\n",
			port_cfg->port_prio);
		break;
	case PROC_SET_DEF_VID:
		len += sprintf(buf + len, "0x%04x\n",
			port_cfg->vid);
		break;
	case PROC_SET_MEMBER:
		len += sprintf(buf + len, "0x%02x\n",
			port_cfg->member);
		break;
	case PROC_SET_TX_P0_CTRL:
		len += sprintf(buf + len, "%u\n", (int)
			(port_cfg->rate_ctrl[0] & RATE_CTRL_ENABLE) != 0);
		break;
	case PROC_SET_TX_P1_CTRL:
		len += sprintf(buf + len, "%u\n", (int)
			(port_cfg->rate_ctrl[1] & RATE_CTRL_ENABLE) != 0);
		break;
	case PROC_SET_TX_P2_CTRL:
		len += sprintf(buf + len, "%u\n", (int)
			(port_cfg->rate_ctrl[2] & RATE_CTRL_ENABLE) != 0);
		break;
	case PROC_SET_TX_P3_CTRL:
		len += sprintf(buf + len, "%u\n", (int)
			(port_cfg->rate_ctrl[3] & RATE_CTRL_ENABLE) != 0);
		break;
#ifdef RATE_RATIO_MASK
	case PROC_SET_TX_P0_RATIO:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->rate_ctrl[0] & RATE_RATIO_MASK);
		break;
	case PROC_SET_TX_P1_RATIO:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->rate_ctrl[1] & RATE_RATIO_MASK);
		break;
	case PROC_SET_TX_P2_RATIO:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->rate_ctrl[2] & RATE_RATIO_MASK);
		break;
	case PROC_SET_TX_P3_RATIO:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->rate_ctrl[3] & RATE_RATIO_MASK);
		break;
#endif
	case PROC_SET_RX_LIMIT:
		len += sprintf(buf + len, "%u\n", (int)
			(port_cfg->rate_limit >> 2) & 3);
		break;
	case PROC_SET_CNT_IFG:
		len += sprintf(buf + len, "%u\n", (int)
			(port_cfg->rate_limit & 2) != 0);
		break;
	case PROC_SET_CNT_PRE:
		len += sprintf(buf + len, "%u\n", (int)
			(port_cfg->rate_limit & 1) != 0);
		break;
	case PROC_SET_RX_P0_RATE:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->rx_rate[0]);
		break;
	case PROC_SET_RX_P1_RATE:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->rx_rate[1]);
		break;
	case PROC_SET_RX_P2_RATE:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->rx_rate[2]);
		break;
	case PROC_SET_RX_P3_RATE:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->rx_rate[3]);
		break;
	case PROC_SET_TX_P0_RATE:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->tx_rate[0]);
		break;
	case PROC_SET_TX_P1_RATE:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->tx_rate[1]);
		break;
	case PROC_SET_TX_P2_RATE:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->tx_rate[2]);
		break;
	case PROC_SET_TX_P3_RATE:
		len += sprintf(buf + len, "%u\n", (int)
			port_cfg->tx_rate[3]);
		break;
	}
	return len;
}

static ssize_t sysfs_port_read_hw(struct ksz_sw *sw, int proc_num, int port,
	ssize_t len, char *buf)
{
	u8 data[8];

	switch (proc_num) {
	case PROC_ENABLE_BROADCAST_STORM:
		len += sprintf(buf + len, "%d\n",
			port_chk_broad_storm(sw, port));
		break;
	case PROC_ENABLE_DIFFSERV:
		len += sprintf(buf + len, "%d\n",
			port_chk_diffserv(sw, port));
		break;
	case PROC_ENABLE_802_1P:
		len += sprintf(buf + len, "%d\n",
			port_chk_802_1p(sw, port));
		break;
	case PROC_ENABLE_PRIO_QUEUE:
		len += sprintf(buf + len, "%d\n",
			port_chk_prio(sw, port));
		break;
	case PROC_SET_MIRROR_PORT:
		len += sprintf(buf + len, "%u\n",
			port_chk_mirror_sniffer(sw, port));
		break;
	case PROC_SET_MIRROR_RX:
		len += sprintf(buf + len, "%u\n",
			port_chk_mirror_rx(sw, port));
		break;
	case PROC_SET_MIRROR_TX:
		len += sprintf(buf + len, "%u\n",
			port_chk_mirror_tx(sw, port));
		break;
	case PROC_SET_LEARN:
		len += sprintf(buf + len, "%u\n",
			!port_chk_dis_learn(sw, port));
		break;
	case PROC_SET_RX:
		len += sprintf(buf + len, "%u\n",
			port_chk_rx(sw, port));
		break;
	case PROC_SET_TX:
		len += sprintf(buf + len, "%u\n",
			port_chk_tx(sw, port));
		break;
	case PROC_SET_INSERT_TAG:
		len += sprintf(buf + len, "%u\n",
			port_chk_ins_tag(sw, port));
		break;
	case PROC_SET_REMOVE_TAG:
		len += sprintf(buf + len, "%u\n",
			port_chk_rmv_tag(sw, port));
		break;
#ifdef PORT_DOUBLE_TAG
	case PROC_SET_DOUBLE_TAG:
		len += sprintf(buf + len, "%u\n",
			port_chk_double_tag(sw, port));
		break;
#endif
	case PROC_SET_DROP_TAG:
		len += sprintf(buf + len, "%u\n",
			port_chk_drop_tag(sw, port));
		break;
	case PROC_SET_REPLACE_PRIO:
		len += sprintf(buf + len, "%u\n",
			port_chk_replace_prio(sw, port));
		break;
	case PROC_ENABLE_PRIO_RATE:
		len += sprintf(buf + len, "%d\n",
			sw_chk_prio_rate(sw, port));
		break;
	case PROC_SET_DIS_NON_VID:
		len += sprintf(buf + len, "%u\n",
			port_chk_dis_non_vid(sw, port));
		break;
	case PROC_SET_INGRESS:
		len += sprintf(buf + len, "%u\n",
			port_chk_in_filter(sw, port));
		break;
	case PROC_SET_BACK_PRESSURE:
		len += sprintf(buf + len, "%u\n",
			port_chk_back_pressure(sw, port));
		break;
	case PROC_SET_FORCE_FLOW_CTRL:
		len += sprintf(buf + len, "%u\n",
			port_chk_force_flow_ctrl(sw, port));
		break;
	case PROC_SET_UNKNOWN_DEF_PORT:
		len += sprintf(buf + len, "%u\n",
			sw_chk_unk_def_port(sw, port));
		break;
	case PROC_SET_FORWARD_INVALID_VID:
		len += sprintf(buf + len, "%u\n",
			sw_chk_for_inv_vid(sw, port));
		break;
	case PROC_SET_PORT_MAC_ADDR:
		port_get_addr(sw, port, data);
		len += sprintf(buf + len, "%02X:%02X:%02X:%02X:%02X:%02X\n",
			data[0], data[1], data[2], data[3], data[4], data[5]);
		break;
	case PROC_SET_SRC_FILTER_0:
		len += sprintf(buf + len, "%u\n",
			port_chk_src_filter_0(sw, port));
		break;
	case PROC_SET_SRC_FILTER_1:
		len += sprintf(buf + len, "%u\n",
			port_chk_src_filter_1(sw, port));
		break;
	}
	return len;
}

static int sysfs_port_write(struct ksz_sw *sw, int proc_num, int port,
	int num, const char *buf)
{
	u8 data[8];
	int processed = true;

	switch (proc_num) {
	case PROC_SET_PORT_MIB:
	{
		struct ksz_port_mib *mib = &sw->port_mib[port];

		memset((void *) mib->counter, 0, sizeof(u64) *
			TOTAL_SWITCH_COUNTER_NUM);
		break;
	}
	case PROC_ENABLE_BROADCAST_STORM:
		if (!num)
			sw_dis_broad_storm(sw, port);
		else
			sw_ena_broad_storm(sw, port);
		break;
	case PROC_ENABLE_DIFFSERV:
		if (!num)
			sw_dis_diffserv(sw, port);
		else
			sw_ena_diffserv(sw, port);
		break;
	case PROC_ENABLE_802_1P:
		if (!num)
			sw_dis_802_1p(sw, port);
		else
			sw_ena_802_1p(sw, port);
		break;
	case PROC_SET_PORT_BASED:
		sw_cfg_port_based(sw, port, num);
		break;
	case PROC_SET_DEF_VID:
		sw_cfg_def_vid(sw, port, num);
		break;
	case PROC_SET_MEMBER:
		sw_cfg_port_base_vlan(sw, port, (u8) num);
		break;
	case PROC_ENABLE_PRIO_QUEUE:
		if (!num)
			sw_dis_multi_queue(sw, port);
		else
			sw_ena_multi_queue(sw, port);
		break;
	case PROC_SET_TX_P0_CTRL:
		hw_cfg_rate_ctrl(sw, port, 0, num);
		break;
	case PROC_SET_TX_P1_CTRL:
		hw_cfg_rate_ctrl(sw, port, 1, num);
		break;
	case PROC_SET_TX_P2_CTRL:
		hw_cfg_rate_ctrl(sw, port, 2, num);
		break;
	case PROC_SET_TX_P3_CTRL:
		hw_cfg_rate_ctrl(sw, port, 3, num);
		break;
#ifdef RATE_RATIO_MASK
	case PROC_SET_TX_P0_RATIO:
		hw_cfg_rate_ratio(sw, port, 0, (u8) num);
		break;
	case PROC_SET_TX_P1_RATIO:
		hw_cfg_rate_ratio(sw, port, 1, (u8) num);
		break;
	case PROC_SET_TX_P2_RATIO:
		hw_cfg_rate_ratio(sw, port, 2, (u8) num);
		break;
	case PROC_SET_TX_P3_RATIO:
		hw_cfg_rate_ratio(sw, port, 3, (u8) num);
		break;
#endif
	case PROC_SET_RX_LIMIT:
		hw_cfg_rx_limit(sw, port, (u8) num);
		break;
	case PROC_SET_CNT_IFG:
		hw_cfg_cnt_ifg(sw, port, num);
		break;
	case PROC_SET_CNT_PRE:
		hw_cfg_cnt_pre(sw, port, num);
		break;
	case PROC_SET_RX_P0_RATE:
		hw_cfg_rx_prio_rate(sw, port, 0, num);
		break;
	case PROC_SET_RX_P1_RATE:
		hw_cfg_rx_prio_rate(sw, port, 1, num);
		break;
	case PROC_SET_RX_P2_RATE:
		hw_cfg_rx_prio_rate(sw, port, 2, num);
		break;
	case PROC_SET_RX_P3_RATE:
		hw_cfg_rx_prio_rate(sw, port, 3, num);
		break;
	case PROC_SET_TX_P0_RATE:
		hw_cfg_tx_prio_rate(sw, port, 0, num);
		break;
	case PROC_SET_TX_P1_RATE:
		hw_cfg_tx_prio_rate(sw, port, 1, num);
		break;
	case PROC_SET_TX_P2_RATE:
		hw_cfg_tx_prio_rate(sw, port, 2, num);
		break;
	case PROC_SET_TX_P3_RATE:
		hw_cfg_tx_prio_rate(sw, port, 3, num);
		break;
	case PROC_SET_MIRROR_PORT:
		port_cfg_mirror_sniffer(sw, port, num);
		break;
	case PROC_SET_MIRROR_RX:
		port_cfg_mirror_rx(sw, port, num);
		break;
	case PROC_SET_MIRROR_TX:
		port_cfg_mirror_tx(sw, port, num);
		break;
	case PROC_SET_LEARN:
		port_cfg_dis_learn(sw, port, !num);
		if (!num)
			sw_cfg(sw, S_FLUSH_TABLE_CTRL,
				SWITCH_FLUSH_DYN_MAC_TABLE, 1);
		break;
	case PROC_SET_RX:
		port_cfg_rx(sw, port, num);
		break;
	case PROC_SET_TX:
		port_cfg_tx(sw, port, num);
		break;
	case PROC_SET_INSERT_TAG:
		sw_vlan_cfg_ins_tag(sw, port, num);
		break;
	case PROC_SET_REMOVE_TAG:
		sw_vlan_cfg_rmv_tag(sw, port, num);
		break;
#ifdef PORT_DOUBLE_TAG
	case PROC_SET_DOUBLE_TAG:
		sw_vlan_cfg_double_tag(sw, port, num);
		break;
#endif
	case PROC_SET_DROP_TAG:
		sw_vlan_cfg_drop_tag(sw, port, num);
		break;
	case PROC_SET_REPLACE_PRIO:
		sw_cfg_replace_prio(sw, port, num);
		break;
	case PROC_ENABLE_PRIO_RATE:
		if (!num)
			sw_dis_prio_rate(sw, port);
		else
			sw_ena_prio_rate(sw, port);
		break;
	case PROC_SET_DIS_NON_VID:
		sw_vlan_cfg_dis_non_vid(sw, port, num);
		break;
	case PROC_SET_INGRESS:
		sw_vlan_cfg_in_filter(sw, port, num);
		break;
	case PROC_SET_BACK_PRESSURE:
		port_cfg_back_pressure(sw, port, num);
		break;
	case PROC_SET_FORCE_FLOW_CTRL:
		port_cfg_force_flow_ctrl(sw, port, num);
		break;
	case PROC_SET_UNKNOWN_DEF_PORT:
		sw_cfg_unk_def_port(sw, port, num);
		break;
	case PROC_SET_FORWARD_INVALID_VID:
		sw_cfg_for_inv_vid(sw, port, num);
		break;
	case PROC_SET_LINK_MD:
		hw_get_link_md(sw, port);
		break;
	case PROC_SET_PORT_MAC_ADDR:
	{
		int i;
		int n[6];

		i = sscanf(buf, "%x:%x:%x:%x:%x:%x",
			&n[0], &n[1], &n[2], &n[3], &n[4], &n[5]);
		if (6 == i) {
			for (i = 0; i < 6; i++)
				data[i] = (u8) n[i];
			port_set_addr(sw, port, data);
		}
		break;
	}
	case PROC_SET_SRC_FILTER_0:
		port_cfg_src_filter_0(sw, port, num);
		break;
	case PROC_SET_SRC_FILTER_1:
		port_cfg_src_filter_1(sw, port, num);
		break;
	default:
		processed = false;
		break;
	}
	return processed;
}

static ssize_t sysfs_mac_read(struct ksz_sw *sw, int proc_num, int index,
	ssize_t len, char *buf)
{
	struct ksz_mac_table *entry;

	entry = &sw->info->mac_table[index];
	switch (proc_num) {
	case PROC_SET_STATIC_FID:
		len += sprintf(buf + len, "%u\n", entry->fid);
		break;
	case PROC_SET_STATIC_USE_FID:
		len += sprintf(buf + len, "%u\n", entry->use_fid);
		break;
	case PROC_SET_STATIC_OVERRIDE:
		len += sprintf(buf + len, "%u\n", entry->override);
		break;
	case PROC_SET_STATIC_VALID:
		len += sprintf(buf + len, "%u\n", entry->valid);
		break;
	case PROC_SET_STATIC_PORTS:
		len += sprintf(buf + len, "%u\n", entry->ports);
		break;
	case PROC_SET_STATIC_MAC_ADDR:
		len += sprintf(buf + len, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			entry->mac_addr[0], entry->mac_addr[1],
			entry->mac_addr[2], entry->mac_addr[3],
			entry->mac_addr[4], entry->mac_addr[5]);
		break;
	}
	return len;
}

static int sysfs_mac_write(struct ksz_sw *sw, int proc_num, int index,
	int num, const char *buf)
{
	struct ksz_mac_table *entry;
	int processed = true;

	entry = &sw->info->mac_table[index];
	switch (proc_num) {
	case PROC_SET_STATIC_FID:
		if (0 <= num && num < 16)
			entry->fid = num;
		break;
	case PROC_SET_STATIC_USE_FID:
		if (num)
			entry->use_fid = 1;
		else
			entry->use_fid = 0;
		break;
	case PROC_SET_STATIC_OVERRIDE:
		if (num)
			entry->override = 1;
		else
			entry->override = 0;
		break;
	case PROC_SET_STATIC_VALID:
		if (num)
			entry->valid = 1;
		else
			entry->valid = 0;
		sw_w_sta_mac_table(sw, index,
			entry->mac_addr, entry->ports,
			entry->override, entry->valid,
			entry->use_fid, entry->fid);
		break;
	case PROC_SET_STATIC_PORTS:
		if (0 <= num && num <= PORT_MASK)
			entry->ports = num;
		break;
	case PROC_SET_STATIC_MAC_ADDR:
	{
		int i;
		int n[6];

		i = sscanf(buf, "%x:%x:%x:%x:%x:%x",
			&n[0], &n[1], &n[2], &n[3], &n[4], &n[5]);
		if (6 == i) {
			for (i = 0; i < 6; i++)
				entry->mac_addr[i] = (u8) n[i];
		}
		break;
	}
	default:
		processed = false;
		break;
	}
	return processed;
}

static ssize_t sysfs_vlan_read(struct ksz_sw *sw, int proc_num, int index,
	ssize_t len, char *buf)
{
	struct ksz_vlan_table *entry;

	entry = &sw->info->vlan_table[index];
	switch (proc_num) {
	case PROC_SET_VLAN_VALID:
		len += sprintf(buf + len, "%u\n", entry->valid);
		break;
	case PROC_SET_VLAN_MEMBER:
		len += sprintf(buf + len, "%u\n", entry->member);
		break;
	case PROC_SET_VLAN_FID:
		len += sprintf(buf + len, "%u\n", entry->fid);
		break;
	case PROC_SET_VLAN_VID:
		len += sprintf(buf + len, "0x%04x\n", entry->vid);
		break;
	}
	return len;
}

static int sysfs_vlan_write(struct ksz_sw *sw, int proc_num, int index,
	int num)
{
	struct ksz_vlan_table *entry;
	int processed = true;

	entry = &sw->info->vlan_table[index];
	switch (proc_num) {
	case PROC_SET_VLAN_VALID:
		if (num)
			entry->valid = 1;
		else
			entry->valid = 0;
		sw_w_vlan_table(sw, index, entry->vid, entry->fid,
			entry->member, entry->valid);
		break;
	case PROC_SET_VLAN_MEMBER:
		if (0 <= num && num <= PORT_MASK)
			entry->member = num;
		break;
	case PROC_SET_VLAN_FID:
		if (0 <= num && num < 16)
			entry->fid = num;
		break;
	case PROC_SET_VLAN_VID:
		if (0 <= num && num < 0x1000)
			entry->vid = num;
		break;
	default:
		processed = false;
		break;
	}
	return processed;
}

static char *sw_name[TOTAL_PORT_NUM] = {
	"sw0",
	"sw1",
	"sw2",
};

static char *mac_name[STATIC_MAC_TABLE_ENTRIES] = {
	"mac0",
	"mac1",
	"mac2",
	"mac3",
	"mac4",
	"mac5",
	"mac6",
	"mac7",
};

static char *vlan_name[VLAN_TABLE_ENTRIES] = {
	"vlan0",
	"vlan1",
	"vlan2",
	"vlan3",
	"vlan4",
	"vlan5",
	"vlan6",
	"vlan7",
	"vlan8",
	"vlan9",
	"vlana",
	"vlanb",
	"vlanc",
	"vland",
	"vlane",
	"vlanf",
};

static int alloc_dev_attr(struct attribute **attrs, size_t attr_size, int item,
	struct ksz_dev_attr **ksz_attrs, struct attribute ***item_attrs,
	char *item_name, struct ksz_dev_attr **attrs_ptr)
{
	struct attribute **attr_ptr;
	struct device_attribute *dev_attr;
	struct ksz_dev_attr *new_attr;

	*item_attrs = kmalloc(attr_size * sizeof(void *), GFP_KERNEL);
	if (!*item_attrs)
		return -ENOMEM;

	attr_size--;
	attr_size *= sizeof(struct ksz_dev_attr);
	*ksz_attrs = *attrs_ptr;
	*attrs_ptr += attr_size / sizeof(struct ksz_dev_attr);

	new_attr = *ksz_attrs;
	attr_ptr = *item_attrs;
	while (*attrs != NULL) {
		if (item_name && !strcmp((*attrs)->name, item_name))
			break;
		dev_attr = container_of(*attrs, struct device_attribute, attr);
		memcpy(new_attr, dev_attr, sizeof(struct device_attribute));
		strncpy(new_attr->dev_name, (*attrs)->name, DEV_NAME_SIZE);
		if (10 <= item && item <= 15)
			new_attr->dev_name[0] = item - 10 + 'a';
		else
			new_attr->dev_name[0] = item + '0';
		new_attr->dev_attr.attr.name = new_attr->dev_name;
		*attr_ptr = &new_attr->dev_attr.attr;
		new_attr++;
		attr_ptr++;
		attrs++;
	}
	*attr_ptr = NULL;
	return 0;
}

