/**
 * drivers/net/ksz_ptp.c - Micrel PTP common code
 *
 * Copyright (c) 2009-2012 Micrel, Inc.
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


#if 0
#define ENABLE_10_MHZ_CLK
#endif


#define FMT_NSEC_SIZE			12

static char *format_nsec(char *str, u32 nsec)
{
	u32 nsec0;
	u32 nsec1;
	u32 nsec2;
	char str0[4];

	nsec0 = nsec % 1000;
	nsec1 = (nsec / 1000) % 1000;
	nsec2 = (nsec / 1000000) % 1000;
	sprintf(str0, "%03u", nsec0);
	if (nsec2)
		sprintf(str, "%3u.%03u.%s", nsec2, nsec1, str0);
	else if (nsec1)
		sprintf(str, "    %3u.%s", nsec1, str0);
	else
		sprintf(str, "        %3u", nsec0);
	return str;
}  /* format_nsec */

struct pseudo_iphdr {
	__u8 ttl;
	__u8 protocol;
	__be16 tot_len;
	__be32 saddr;
	__be32 daddr;
};

struct pseudo_ip6hdr {
	__be16 payload_len;
	__u8 hop_limit;
	__u8 nexthdr;
	struct in6_addr saddr;
	struct in6_addr daddr;
};

#ifdef VERIFY_PTP_CSUM
static u_short in_cksum(u_short *addr, int len)
{
	int nleft = len;
	int sum = 0;
	u_short *w = addr;
	u_short answer = 0;

	while (nleft > 1) {
		sum += *w++;
		nleft -= 2;
	}

	if (nleft == 1) {
		*(u_char *)(&answer) = *(u_char *) w;
		sum += answer;
	}

	sum = (sum >> 16) + (sum & 0xffff);
	sum += (sum >> 16);
	answer = (u_short) ~sum;
	return answer;
}  /* in_cksum */
#endif

static u32 timestamp_val(u32 timestamp, u8 *sec)
{
	*sec = timestamp >> 30;
	timestamp <<= 2;
	timestamp >>= 2;
	return timestamp;
}  /* timestamp_val */

static void calc_diff(struct ptp_time *prev, struct ptp_time *cur,
	struct ptp_time *result)
{
	struct ptp_time diff;
	int prev_nsec = prev->nsec;
	int cur_nsec = cur->nsec;

	if (prev->sec < 0)
		prev_nsec = -prev_nsec;
	if (cur->sec < 0)
		cur_nsec = -cur_nsec;
	diff.sec = cur->sec - prev->sec;
	diff.nsec = cur_nsec - prev_nsec;
	if (diff.nsec >= NANOSEC_IN_SEC) {
		diff.nsec -= NANOSEC_IN_SEC;
		diff.sec++;
	} else if (diff.nsec <= -NANOSEC_IN_SEC) {
		diff.nsec += NANOSEC_IN_SEC;
		diff.sec--;
	}
	if (diff.sec > 0 && diff.nsec < 0) {
		diff.nsec += NANOSEC_IN_SEC;
		diff.sec--;
	} else if (diff.sec < 0 && diff.nsec > 0) {
		diff.nsec -= NANOSEC_IN_SEC;
		diff.sec++;
	}
	if (diff.nsec < 0 && diff.sec < 0)
		diff.nsec = -diff.nsec;
	result->sec = diff.sec;
	result->nsec = diff.nsec;
}  /* calc_diff */

static void calc_udiff(struct ptp_utime *prev, struct ptp_utime *cur,
	struct ptp_time *result)
{
	struct ptp_time t1;
	struct ptp_time t2;

	t1.sec = prev->sec;
	t1.nsec = prev->nsec;
	t2.sec = cur->sec;
	t2.nsec = cur->nsec;
	calc_diff(&t1, &t2, result);
}  /* calc_udiff */

#ifdef PTP_PROCESS
static void calc_diff64(struct ptp_ltime *prev, struct ptp_ltime *cur,
	struct ptp_ltime *result)
{
	struct ptp_ltime diff;
	s64 prev_nsec = prev->nsec;
	s64 cur_nsec = cur->nsec;
	s64 scaled_nsec = (s64) NANOSEC_IN_SEC << SCALED_NANOSEC_SHIFT;

	if (prev->sec < 0)
		prev_nsec = -prev_nsec;
	if (cur->sec < 0)
		cur_nsec = -cur_nsec;
	diff.sec = cur->sec - prev->sec;
	diff.nsec = cur_nsec - prev_nsec;
	if (diff.nsec >= scaled_nsec) {
		diff.nsec -= scaled_nsec;
		diff.sec++;
	} else if (diff.nsec <= -scaled_nsec) {
		diff.nsec += scaled_nsec;
		diff.sec--;
	}
	if (diff.sec > 0 && diff.nsec < 0) {
		diff.nsec += scaled_nsec;
		diff.sec--;
	} else if (diff.sec < 0 && diff.nsec > 0) {
		diff.nsec -= scaled_nsec;
		diff.sec++;
	}
	if (diff.nsec < 0 && diff.sec < 0)
		diff.nsec = -diff.nsec;
	result->sec = diff.sec;
	result->nsec = diff.nsec;
}  /* calc_diff64 */
#endif

#if 0
static void test_calc_diff(int t1_sec, int t1_nsec, int t2_sec, int t2_nsec)
{
	struct ptp_time diff;
	struct ptp_time t1;
	struct ptp_time t2;
	struct ptp_ltime t;
	struct ptp_ltime t3;
	struct ptp_ltime t4;

	t1.sec = t1_sec;
	t1.nsec = t1_nsec;
	t2.sec = t2_sec;
	t2.nsec = t2_nsec;
	t3.sec = t1_sec;
	t3.nsec = (s64) t1_nsec * SCALED_NANOSEC_MULT + 33000;
	t4.sec = t2_sec;
	t4.nsec = (s64) t2_nsec * SCALED_NANOSEC_MULT + 33000;
	calc_diff(&t1, &t2, &diff);
	calc_diff64(&t3, &t4, &t);
	printk(KERN_INFO "diff: %d:%9u - %d:%9u = %d:%d %lld:%lld\n",
		t2_sec, t2_nsec, t1_sec, t1_nsec, diff.sec, diff.nsec,
		t.sec, t.nsec >> SCALED_NANOSEC_SHIFT);
}  /* test_calc_diff */

static void verify_calc_diff(void)
{
	struct ptp_time t1;
	struct ptp_time t2;

	t1.sec = 1234;
	t1.nsec = 5678;
	t2.sec = 1234;
	t2.nsec = 5678;
	test_calc_diff(t1.sec, t1.nsec, t2.sec, t2.nsec);
	test_calc_diff(t1.sec, t1.nsec, -t2.sec, t2.nsec);
	test_calc_diff(-t1.sec, t1.nsec, t2.sec, t2.nsec);
	test_calc_diff(-t1.sec, t1.nsec, -t2.sec, t2.nsec);
	t2.sec = 1235;
	t2.nsec = 505678;
	test_calc_diff(t1.sec, t1.nsec, t2.sec, t2.nsec);
	test_calc_diff(t1.sec, t1.nsec, -t2.sec, t2.nsec);
	test_calc_diff(-t1.sec, t1.nsec, t2.sec, t2.nsec);
	test_calc_diff(-t1.sec, t1.nsec, -t2.sec, t2.nsec);
	test_calc_diff(t2.sec, t2.nsec, t1.sec, t1.nsec);
	test_calc_diff(t2.sec, t2.nsec, -t1.sec, t1.nsec);
	test_calc_diff(-t2.sec, t2.nsec, t1.sec, t1.nsec);
	test_calc_diff(-t2.sec, t2.nsec, -t1.sec, t1.nsec);
	t1.nsec = 50505678;
	test_calc_diff(t1.sec, t1.nsec, t2.sec, t2.nsec);
	test_calc_diff(t1.sec, t1.nsec, -t2.sec, t2.nsec);
	test_calc_diff(-t1.sec, t1.nsec, t2.sec, t2.nsec);
	test_calc_diff(-t1.sec, t1.nsec, -t2.sec, t2.nsec);
	test_calc_diff(t2.sec, t2.nsec, t1.sec, t1.nsec);
	test_calc_diff(t2.sec, t2.nsec, -t1.sec, t1.nsec);
	test_calc_diff(-t2.sec, t2.nsec, t1.sec, t1.nsec);
	test_calc_diff(-t2.sec, t2.nsec, -t1.sec, t1.nsec);
}  /* verify_calc_diff */
#endif

static void add_nsec(struct ptp_utime *t, u32 nsec)
{
	t->nsec += nsec;
	if (t->nsec >= NANOSEC_IN_SEC) {
		t->nsec -= NANOSEC_IN_SEC;
		t->sec++;
	}
}  /* add_nsec */

static void get_ptp_time(void *ptpdev, struct ptp_utime *t)
{
	u16 data;
	u8 subnsec;

	data = ptp_read(ptpdev, ADDR_16, PTP_CLK_CTRL);
	data |= PTP_READ_RTC;
	ptp_write(ptpdev, ADDR_16, PTP_CLK_CTRL, data);
	t->sec = ptp_read(ptpdev, ADDR_32, PTP_RTC_SEC_L);
	t->nsec = ptp_read(ptpdev, ADDR_32, PTP_RTC_NANOSEC_L);
	subnsec = ptp_read(ptpdev, ADDR_8, PTP_RTC_SUB_NANOSEC);
	add_nsec(t, subnsec * 8);
}  /* get_ptp_time */

static void update_ts(struct ptp_ts *ts, u32 cur_sec)
{
	int sec;
	u8 sec_chk;

	ts->t.nsec = timestamp_val(ts->timestamp, &sec_chk);
	if (ts->timestamp)
		sec = (cur_sec - sec_chk) & 3;
	else
		sec = 0;
	if (sec >= 3)
		sec -= 4;
	ts->t.sec = cur_sec - sec;
}  /* update_ts */

#ifdef PTP_MONITOR
static void get_cur_time(struct ptp_info *ptp, u8 msg, u8 port,
	struct ptp_ts *ts)
{
	u32 timestamp;
	u8 overflow;
	u8 sec;

	timestamp = ts->timestamp;
	timestamp = timestamp_val(timestamp, &overflow);
	ts->t.sec = ptp->cur_time.sec;
	ts->t.nsec = ptp->cur_time.nsec;
	if (ts->timestamp)
		sec = (ts->t.sec - overflow) & 3;
	else
		sec = 0;
	ts->t.sec -= sec;
}  /* get_cur_time */
#endif

static void set_ptp_time(void *ptpdev, struct ptp_utime *t)
{
	u16 data;

	data = ptp_read(ptpdev, ADDR_16, PTP_CLK_CTRL);
	ptp_write(ptpdev, ADDR_8, PTP_RTC_SUB_NANOSEC, 0);
	ptp_write(ptpdev, ADDR_32, PTP_RTC_SEC_H, t->sec >> 16);
	ptp_write(ptpdev, ADDR_32, PTP_RTC_SEC_L, (u16) t->sec);
	ptp_write(ptpdev, ADDR_32, PTP_RTC_NANOSEC_H, t->nsec >> 16);
	ptp_write(ptpdev, ADDR_32, PTP_RTC_NANOSEC_L, (u16) t->nsec);
	data |= PTP_LOAD_TIME;
	ptp_write(ptpdev, ADDR_16, PTP_CLK_CTRL, data);
}  /* set_ptp_time */

static void set_ptp_domain(void *ptpdev, u8 domain)
{
	u16 data;

	data = ptp_read(ptpdev, ADDR_16, PTP_DOMAIN_VERSION) &
		~PTP_DOMAIN_MASK;
	data |= domain;
	ptp_write(ptpdev, ADDR_16, PTP_DOMAIN_VERSION, data);
}  /* set_ptp_domain */

static void set_ptp_mode(void *ptpdev, u16 mode)
{
	u16 val;
	u16 sav;

	val = ptp_read(ptpdev, ADDR_16, PTP_MSG_CONF1);
	sav = val;
	val &= ~(PTP_1STEP | PTP_TC_P2P | PTP_MASTER);
	val |= mode;
	if (val != sav)
		ptp_write(ptpdev, ADDR_16, PTP_MSG_CONF1, val);
}  /* set_ptp_mode */

static void adjust_ptp_time(void *ptpdev, int add, u32 sec, u32 nsec,
	int adj_hack)
{
	u16 data;
	u16 adj = 0;
	u32 val = nsec;

	data = ptp_read(ptpdev, ADDR_16, PTP_CLK_CTRL);
	if (adj_hack) {
		adj = data;
		data &= ~PTP_CLK_ADJ_ENABLE;
	}
	data |= PTP_STEP_TIME;
	if (add)
		data |= PTP_STEP_DIR;
	else
		data &= ~PTP_STEP_DIR;
	ptp_write(ptpdev, ADDR_32, PTP_RTC_SEC_H, sec >> 16);
	ptp_write(ptpdev, ADDR_32, PTP_RTC_SEC_L, (u16) sec);
	do {
		if (nsec > NANOSEC_IN_SEC - 1)
			nsec = NANOSEC_IN_SEC - 1;
		ptp_write(ptpdev, ADDR_32, PTP_RTC_NANOSEC_H, nsec >> 16);
		ptp_write(ptpdev, ADDR_32, PTP_RTC_NANOSEC_L, (u16) nsec);
		ptp_write(ptpdev, ADDR_16, PTP_CLK_CTRL, data);
		val -= nsec;
		nsec = val;
	} while (val);
	if (adj_hack && (adj & PTP_CLK_ADJ_ENABLE))
		ptp_write(ptpdev, ADDR_16, PTP_CLK_CTRL, adj);
}  /* adjust_ptp_time */

static void synchronize_clk(struct ptp_info *ptp)
{
	if (ptp->adjust_offset < 0)
		adjust_ptp_time(ptp->ptpdev, false, -ptp->adjust_sec,
			-ptp->adjust_offset, ptp->features & PTP_ADJ_HACK);
	else
		adjust_ptp_time(ptp->ptpdev, true, ptp->adjust_sec,
			ptp->adjust_offset, ptp->features & PTP_ADJ_HACK);
	ptp->offset_changed = ptp->adjust_offset;
	ptp->adjust_offset = 0;
}  /* synchronize_clk */

#define MAX_DRIFT_CORR			6250000
#define LOW_DRIFT_CORR			2499981
#define MAX_U32_SHIFT			32
#define MAX_DIVIDER_SHIFT		31

static u32 drift_in_sec(u32 abs_offset, u64 interval64)
{
	u32 abs_drift;
	u32 divider;
	u32 interval;
	u64 drift64;

	/* 2^32 / 1000 */
	interval64 *= 4294967;
	interval64 += (u32)(1 << (MAX_U32_SHIFT - 1));
	interval64 >>= MAX_U32_SHIFT;

	/* interval64 / 1000 */
	interval = (u32) interval64;
	divider = (1 << MAX_DIVIDER_SHIFT);
	divider += interval / 2;
	divider /= interval;

	drift64 = abs_offset;

	/* NANOSEC_IN_SEC / 1000 */
	drift64 *= 1000000;
	drift64 *= divider;
	drift64 += (u32)(1 << (MAX_DIVIDER_SHIFT - 1));
	drift64 >>= MAX_DIVIDER_SHIFT;
	if (drift64 > 0xffffffff)
		abs_drift = 0xffffffff;
	else
		abs_drift = (u32) drift64;
	return abs_drift;
}

static u32 clk_adjust_val(int diff, u32 interval)
{
	u32 adjust;
	u64 adjust64;

	if (0 == diff)
		return 0;
	if (diff < 0)
		adjust = -diff;
	else
		adjust = diff;

	if (interval != NANOSEC_IN_SEC)
		adjust = drift_in_sec(adjust, interval);

	/* 2^32 * adjust * 1000000000 / interval / 25000000 */
	if (adjust >= MAX_DRIFT_CORR)
		adjust = 0x3fffffff;
	else {
		adjust64 = adjust;

		/* 2^32 * 10 / 25 */
		adjust64 *= 1717986918;

		/* 2^32 / 10000000 */
		if (adjust < LOW_DRIFT_CORR)
			adjust64 *= 4295;
		else
			adjust64 *= 429;
		adjust64 += (u32)(1 << (MAX_U32_SHIFT - 1));
		adjust64 >>= MAX_U32_SHIFT;
		if (adjust < LOW_DRIFT_CORR) {
			adjust = (u32) adjust64;
			adjust /= 10;
		} else {
			adjust = (u32) adjust64;
			if (adjust < 0x199998f3)
				adjust = 0x199998f3;
		}
	}
	if (diff < 0)
		adjust |= PTP_RATE_DIR << 16;
	return adjust;
}  /* clk_adjust_val */

static void set_ptp_adjust(void *ptpdev, u32 adjust)
{
	ptp_write(ptpdev, ADDR_32, PTP_SUBNANOSEC_RATE_H, adjust >> 16);
	ptp_write(ptpdev, ADDR_32, PTP_SUBNANOSEC_RATE_L, adjust);
}  /* set_ptp_adjust */

static inline void adjust_sync_time(void *ptpdev, int diff, u32 interval,
	u32 duration)
{
	u32 adjust;

	adjust = clk_adjust_val(diff, interval);
	adjust |= PTP_TMP_RATE_ENABLE << 16;
	ptp_write(ptpdev, ADDR_32, PTP_RATE_DURATION_H, duration >> 16);
	ptp_write(ptpdev, ADDR_32, PTP_RATE_DURATION_L, duration);
	ptp_write(ptpdev, ADDR_32, PTP_SUBNANOSEC_RATE_H, adjust >> 16);
	ptp_write(ptpdev, ADDR_32, PTP_SUBNANOSEC_RATE_L, adjust);
}  /* adjust_sync_time */

static inline void unsyntonize_clk(void *ptpdev)
{
	u16 data;

	data = ptp_read(ptpdev, ADDR_16, PTP_CLK_CTRL);
	data &= ~PTP_CLK_ADJ_ENABLE;
	ptp_write(ptpdev, ADDR_16, PTP_CLK_CTRL, data);
}  /* unsyntonize_clk */

static void syntonize_clk(void *ptpdev)
{
	u16 data;

	data = ptp_read(ptpdev, ADDR_16, PTP_CLK_CTRL);
	data |= PTP_CLK_ADJ_ENABLE;
	ptp_write(ptpdev, ADDR_16, PTP_CLK_CTRL, data);
}  /* syntonize_clk */

static u16 get_ptp_delay(void *ptpdev, int port, int reg)
{
	reg += PTP_PORT_INTERVAL(port);
	return ptp_read(ptpdev, ADDR_16, reg);
}  /* get_ptp_delay */

static void set_ptp_delay(void *ptpdev, int port, int reg, u16 nsec)
{
	reg += PTP_PORT_INTERVAL(port);
	ptp_write(ptpdev, ADDR_16, reg, nsec);
}  /* set_ptp_delay */

static u16 get_ptp_ingress(void *ptpdev, int port)
{
	return get_ptp_delay(ptpdev, port, PTP_PORT1_RX_MAC2PHY_DELAY);
}

static u16 get_ptp_egress(void *ptpdev, int port)
{
	return get_ptp_delay(ptpdev, port, PTP_PORT1_TX_MAC2PHY_DELAY);
}

static short get_ptp_asym(void *ptpdev, int port)
{
	short val;

	val = get_ptp_delay(ptpdev, port, PTP_PORT1_ASYM_DELAY);
	if (val & 0x8000)
		val = -(val & ~0x8000);
	return val;
}

static u16 get_ptp_link(void *ptpdev, int port)
{
	return get_ptp_delay(ptpdev, port, PTP_PORT1_LINK_DELAY);
}

static void set_ptp_ingress(void *ptpdev, int port, u16 nsec)
{
	set_ptp_delay(ptpdev, port, PTP_PORT1_RX_MAC2PHY_DELAY, nsec);
}

static void set_ptp_egress(void *ptpdev, int port, u16 nsec)
{
	set_ptp_delay(ptpdev, port, PTP_PORT1_TX_MAC2PHY_DELAY, nsec);
}

static void set_ptp_asym(void *ptpdev, int port, short nsec)
{
	if (nsec < 0)
		nsec = -nsec | 0x8000;
	set_ptp_delay(ptpdev, port, PTP_PORT1_ASYM_DELAY, nsec);
}

static void set_ptp_link(void *ptpdev, int port, u16 nsec)
{
	set_ptp_delay(ptpdev, port, PTP_PORT1_LINK_DELAY, nsec);
}

static inline void dbp_tx_ts(char *name, u8 port, u32 timestamp)
{
	u8 overflow;
	char ts[FMT_NSEC_SIZE];

	timestamp = timestamp_val(timestamp, &overflow);
	format_nsec(ts, timestamp);
	dbg_msg("%s p:%d c:%u %08x:%s\n", name, port, overflow, timestamp, ts);
}  /* dbp_tx_ts */

static void ptp_setup_udp_msg(struct ptp_dev_info *info, u8 *data, int len,
	void (*func)(u8 *data, void *param), void *param)
{
	u8 buf[MAX_TSM_UDP_LEN];
	int in_intr = in_interrupt();

	if (len > MAX_TSM_UDP_LEN)
		len = MAX_TSM_UDP_LEN;
	if (!in_intr)
		mutex_lock(&info->lock);
	memcpy(buf, data, len);
	if (func)
		func(buf, param);
	len += 2;
	if (info->read_len + len <= info->read_max) {
		u16 *udp_len = (u16 *) &info->read_buf[info->read_len];

		*udp_len = len;
		udp_len++;
		memcpy(udp_len, buf, len - 2);
		info->read_len += len;
	}
	if (!in_intr)
		mutex_unlock(&info->lock);
	wake_up_interruptible(&info->wait_udp);
}  /* ptp_setup_udp_msg */

static void ptp_tsm_resp(u8 *data, void *param)
{
	struct tsm_db *db = (struct tsm_db *) data;
	struct ptp_ts *ts = param;
	u32 timestamp;
	u8 sec_chk;

	db->cmd |= TSM_CMD_RESP;
	db->cur_sec = htonl(ts->t.sec);
	db->cur_nsec = htonl(ts->t.nsec);
	timestamp = timestamp_val(ts->timestamp, &sec_chk);
	db->timestamp = htonl(timestamp);
	db->cur_nsec = db->timestamp;
}  /* ptp_tsm_resp */

static void ptp_tsm_get_time_resp(u8 *data, void *param)
{
	struct tsm_get_time *get = (struct tsm_get_time *) data;
	struct ptp_utime *t = param;

	get->cmd |= TSM_CMD_GET_TIME_RESP;
	get->sec = htonl(t->sec);
	get->nsec = htonl(t->nsec);
}  /* ptp_tsm_get_time_resp */

static void add_tx_delay(struct ptp_ts *ts, int delay, u32 cur_sec)
{
	update_ts(ts, cur_sec);

	/*
	 * Save timestamp without transmit latency for PTP stack that adjusts
	 * transmit latency itself.
	 */
	ts->r = ts->t;
	add_nsec(&ts->t, delay);
	ts->timestamp = ts->t.nsec;
}  /* add_tx_delay */

static void save_tx_ts(struct ptp_info *ptp, struct ptp_tx_ts *tx,
	struct ptp_hw_ts *htx, int delay, int port)
{
	unsigned long diff = 0;

	add_tx_delay(&htx->ts, delay, ptp->cur_time.sec);
	if (!htx->sim_2step) {
		struct tsm_db *db = (struct tsm_db *) tx->data.buf;
		u8 msg = tx->data.buf[0] & 3;

		tx->ts = htx->ts;
		tx->resp_time = jiffies;
		if (tx->req_time)
			diff = tx->resp_time - tx->req_time;
		if (diff < 4 * ptp->delay_ticks) {
			if (tx->missed) {
				if (diff > 2 * ptp->delay_ticks)
					dbg_msg("  caught: %d, %lu; %x=%04x\n",
						port, diff, msg,
						ntohs(db->seqid));
				if (tx->dev) {
					ptp_setup_udp_msg(tx->dev,
						tx->data.buf, tx->data.len,
						ptp_tsm_resp, &tx->ts);
					tx->dev = NULL;
				}

				/* Invalidate the timestamp. */
				tx->ts.timestamp = 0;
				tx->req_time = 0;
			}
		} else {
			dbg_msg("  new: %d, %lu; %x=%04x\n", port, diff,
				msg, ntohs(db->seqid));
		}
		tx->missed = false;
		if (tx->skb) {
			u64 ns;
			struct skb_shared_hwtstamps shhwtstamps;

			ns = (u64) tx->ts.r.sec * NANOSEC_IN_SEC +
				tx->ts.r.nsec;
			memset(&shhwtstamps, 0, sizeof(shhwtstamps));
			shhwtstamps.hwtstamp = ns_to_ktime(ns);

			/* Indicate which port message is sent out. */
			tx->msg->hdr.reserved2 = (1 << port);
			skb_tstamp_tx(tx->skb, &shhwtstamps);

			/* buffer not released yet. */
			if (skb_shinfo(tx->skb)->tx_flags & SKBTX_HW_TSTAMP)
				skb_shinfo(tx->skb)->tx_flags &=
					~SKBTX_IN_PROGRESS;
			else
				dev_kfree_skb_irq(tx->skb);
			tx->skb = NULL;
		}
	}
	htx->sending = false;
#ifdef PTP_MONITOR
	htx->update = true;
#endif
}  /* save_tx_ts */

static int get_tx_time(struct ptp_info *ptp, u16 status)
{
	int reg = 0;
	int port;
	int delay;
	u32 xts;
	u32 *pts;
#ifdef PTP_MONITOR
	struct ptp_ts org;
#endif
	struct ptp_tx_ts *tx = NULL;
	struct ptp_hw_ts *htx = NULL;
	void *ptpdev = ptp->ptpdev;

#ifdef DEBUG_MSG
	if (status) {
		u16 data;
		data = ptp_read(ptpdev, ADDR_16, TS_INT_STATUS);
		if (data & status)
			dbg_msg(" ? get_tx_time %x %x\n", data, status);
	}
#endif
	while (status) {
		/* Do port 1 first. */
		if (status & (TS_PORT1_INT_XDELAY | TS_PORT1_INT_SYNC))
			port = 0;
		else if (status & (TS_PORT2_INT_XDELAY | TS_PORT2_INT_SYNC))
			port = 1;
		else
			break;
		xts = 0;
		pts = NULL;
		delay = ptp->tx_latency[port];
		if (status & TS_PORT1_INT_XDELAY) {
			reg = PTP_PORT1_XDELAY_TIMESTAMP_L;
			pts = &ptp->xdelay_ts[port];
			tx = &ptp->tx_dreq[port];
			htx = &ptp->hw_dreq[port];
			status &= ~TS_PORT1_INT_XDELAY;
		} else if (status & TS_PORT1_INT_SYNC) {
			reg = PTP_PORT1_SYNC_TIMESTAMP_L;
			tx = &ptp->tx_sync[port];
			htx = &ptp->hw_sync[port];
			status &= ~TS_PORT1_INT_SYNC;
		} else if (status & TS_PORT2_INT_XDELAY) {
			reg = PTP_PORT1_XDELAY_TIMESTAMP_L;
			pts = &ptp->xdelay_ts[port];
			tx = &ptp->tx_dreq[port];
			htx = &ptp->hw_dreq[port];
			status &= ~TS_PORT2_INT_XDELAY;
		} else if (status & TS_PORT2_INT_SYNC) {
			reg = PTP_PORT1_SYNC_TIMESTAMP_L;
			tx = &ptp->tx_sync[port];
			htx = &ptp->hw_sync[port];
			status &= ~TS_PORT2_INT_SYNC;
		}

		/* PDELAY_REQ and PDELAY_RESP share same interrupt. */
		if (pts) {
			reg += PTP_PORT_INTERVAL(port);
			xts = ptp_read(ptpdev, ADDR_32, reg);

			if (xts != *pts) {
				*pts = xts;
				htx->ts.timestamp = xts;
#ifdef PTP_MONITOR
				org.timestamp = htx->ts.timestamp;
				update_ts(&org, ptp->cur_time.sec);
#endif
				save_tx_ts(ptp, tx, htx, delay, port);
			}

			reg = PTP_PORT1_PDRESP_TIMESTAMP_L;
			pts = &ptp->pdresp_ts[port];
			tx = &ptp->tx_resp[port];
			htx = &ptp->hw_resp[port];

			reg += PTP_PORT_INTERVAL(port);
			xts = ptp_read(ptpdev, ADDR_32, reg);
			if (xts != *pts) {
				delay = ptp->tx_latency[port];
				*pts = xts;
				htx->ts.timestamp = xts;
				save_tx_ts(ptp, tx, htx, delay, port);
#ifdef VERIFY_PDELAY_RESP_DIFF
{
	struct ptp_time diff1;
	struct ptp_time diff2;
	struct ptp_time t1;
	struct ptp_time t2;

	t1.sec = ptp->pdelay_t2[port].sec & 3;
	t1.nsec = ptp->pdelay_t2[port].nsec;
	t2.sec = htx->ts.t.sec & 3;
	t2.nsec = htx->ts.t.nsec;
	calc_diff(&t1, &t2, &diff1);
	t1.sec = 0;
	calc_diff(&t1, &t2, &diff2);
	while (diff2.sec) {
		diff2.nsec += NANOSEC_IN_SEC;
		--diff2.sec;
	}
}
#endif
			}
		} else {
			reg += PTP_PORT_INTERVAL(port);
			htx->ts.timestamp = ptp_read(ptpdev, ADDR_32, reg);
			save_tx_ts(ptp, tx, htx, delay, port);
		}
	}
	if (!htx)
		return false;

#ifdef PTP_MONITOR
	for (port = 0; port < MAX_PTP_PORT; port++) {
		htx = &ptp->hw_sync[port];
#ifdef VERIFY_PTP_TX
		if (htx->update)
			dbp_tx_ts("sync", port + 1, htx->ts.timestamp);
#endif
		htx->update = false;
		htx = &ptp->hw_dreq[port];
#ifdef PTP_PROCESS
#endif
#ifdef VERIFY_PTP_TX
		if (htx->update)
			dbp_tx_ts("dreq", port + 1, htx->ts.timestamp);
#endif
		htx->update = false;
		htx = &ptp->hw_resp[port];
#ifdef VERIFY_PTP_TX
		if (htx->update)
			dbp_tx_ts("resp", port + 1, htx->ts.timestamp);
#endif
		htx->update = false;
	}
#endif

	return true;
}  /* get_tx_time */

static inline void ptp_rx_reset(void *ptpdev, u16 tsi_bit)
{
	ptp_write(ptpdev, ADDR_16, TS_RESET, tsi_bit);
}  /* ptp_rx_reset */

static void ptp_rx_off(struct ptp_info *ptp, u8 tsi)
{
	u16 data;
	u16 tsi_bit = (1 << tsi);
	void *ptpdev = ptp->ptpdev;

	/* Disable previous timestamp interrupt. */
	if (ptp->ts_intr & tsi_bit) {
		ptp->ts_intr &= ~tsi_bit;
		ptp_write(ptpdev, ADDR_16, TS_INT_ENABLE, ptp->ts_intr);
	}

	/* Disable previous timestamp detection. */
	data = ptp_read(ptpdev, ADDR_16, TS_ENABLE);
	if (data & tsi_bit) {
		data &= ~tsi_bit;
		ptp_write(ptpdev, ADDR_16, TS_ENABLE, data);
	}

	/*
	 * Need to turn off cascade mode if it is used previously; otherwise,
	 * event counter keeps increasing.
	 */
	if (ptp->cascade_rx & tsi_bit) {
		ptp_rx_reset(ptpdev, tsi_bit);
		ptp->cascade_rx &= ~tsi_bit;
	}
}  /* ptp_rx_off */

static inline void ptp_rx_intr(struct ptp_info *ptp, u16 tsi_bit)
{
	ptp->ts_intr |= tsi_bit;
	ptp_write(ptp->ptpdev, ADDR_16, TS_INT_ENABLE, ptp->ts_intr);
}  /* ptp_rx_intr */

static inline void ptp_rx_on(void *ptpdev, u16 tsi_bit)
{
	u16 data;

	data = ptp_read(ptpdev, ADDR_16, TS_ENABLE);
	data |= tsi_bit;
	ptp_write(ptpdev, ADDR_16, TS_ENABLE, data);
}  /* ptp_rx_on */

static void ptp_rx_restart(void *ptpdev, u16 tsi_bit)
{
	u16 data;

	data = ptp_read(ptpdev, ADDR_16, TS_ENABLE);
	data &= ~tsi_bit;
	ptp_write(ptpdev, ADDR_16, TS_ENABLE, data);
	data |= tsi_bit;
	ptp_write(ptpdev, ADDR_16, TS_ENABLE, data);
}  /* ptp_rx_restart */

static void ptp_rx_event(struct ptp_info *ptp, u8 tsi, u8 gpi, u8 event,
	int intr)
{
	int reg;
	u16 data;
	u16 tsi_bit = (1 << tsi);
	void *ptpdev = ptp->ptpdev;

	/* Config pattern. */
	reg = TSn_CONF(tsi);
	data = event | ((gpi & 0xf) << 8);
	ptp_write(ptpdev, ADDR_16, reg, data);

	/* Enable timestamp interrupt. */
	if (intr)
		ptp_rx_intr(ptp, tsi_bit);

	/* Enable timestamp detection. */
	ptp_rx_on(ptpdev, tsi_bit);
}  /* ptp_rx_event */

static void ptp_rx_cascade_event(struct ptp_info *ptp, u8 first, u8 total,
	u8 gpi, u8 event, int intr)
{
	int last;
	int tsi;
	int reg;
	u16 data;
	u16 tail;
	int i;
	int prev;
	void *ptpdev = ptp->ptpdev;

	last = (first + total - 1) % MAX_TIMESTAMP_UNIT;
	tsi = last;
	tail = TS_CASCADE_TAIL;
	for (i = 1; i < total; i++) {
		reg = TSn_CONF(tsi);
		prev = tsi - 1;
		if (prev < 0)
			prev = MAX_TIMESTAMP_UNIT - 1;
		data = event | ((gpi & 0xf) << 8);
		data |= TS_CASCADE_EN | ((prev & 0xf) << 1);
		data |= tail;
		ptp->cascade_rx |= (1 << tsi);
		ptp_write(ptpdev, ADDR_16, reg, data);

		/* Enable timestamp interrupt. */
		if (intr)
			ptp->ts_intr |= (1 << tsi);
		--tsi;
		if (tsi < 0)
			tsi = MAX_TIMESTAMP_UNIT - 1;
		tail = 0;
	}
	reg = TSn_CONF(first);
	data = event | ((gpi & 0xf) << 8);
	data |= TS_CASCADE_EN | ((last & 0xf) << 1);
	ptp->cascade_rx |= (1 << first);
	ptp_write(ptpdev, ADDR_16, reg, data);

	/* Enable timestamp interrupt. */
	if (intr)
		ptp_rx_intr(ptp, (1 << first));

	/* Enable timestamp detection. */
	ptp_rx_on(ptpdev, (1 << first));
}  /* ptp_rx_cascade_event */

static void ptp_read_event(struct ptp_info *ptp, u8 tsi)
{
	int reg;
	u16 data;
	u16 tsi_bit = (1 << tsi);

	int reg_ns;
	int reg_s;
	int reg_sub;
	struct ptp_utime t;
	u16 sub;
	int max_ts;
	int num;
	int i;
	int edge;
	void *ptpdev = ptp->ptpdev;
	struct ptp_event *event = &ptp->events[tsi];
	int last = event->num;

	reg = TSn_EVENT_STATUS(tsi);
	data = ptp_read(ptpdev, ADDR_16, reg);
	num = (data & TS_NO_EVENT_DET_MASK) >> 1;
	max_ts = (num <= event->max) ? num : event->max;
	i = event->num;

	reg_ns = TSn_0_EVENT_NANOSEC_L(tsi) + 0x10 * i;
	reg_s = TSn_0_EVENT_SEC_L(tsi) + 0x10 * i;
	reg_sub = TSn_0_EVENT_SUB_NANOSEC(tsi) + 0x10 * i;
	for (; i < max_ts; i++) {
		t.nsec = ptp_read(ptpdev, ADDR_32, reg_ns);
		t.sec = ptp_read(ptpdev, ADDR_32, reg_s);
		sub = ptp_read(ptpdev, ADDR_16, reg_sub);
		edge = ((t.nsec >> 30) & 1);
#if 1
/*
 * THa  2011/10/06
 * Unit sometimes detects rising edge when it is configured to detect falling
 * edge only.  This happens in the case of hooking up the output pin to an
 * input pin and using two units running opposite cycle in cascade mode.  The
 * 8 ns switch pulse before the cycle is too short to detect properly,
 * resulting in missing edges.
 * When detecting events directly from the output pin, the minimum pulse time
 * is 24 ns for proper detection without missing any edge.
 */
		if (event->event < 2 && edge != event->event)
			edge = event->event;
#endif
		event->edge |= edge << i;
		t.nsec <<= 2;
		t.nsec >>= 2;
		add_nsec(&t, sub * 8);
		event->t[i] = t;
		reg_ns += 0x10;
		reg_s += 0x10;
		reg_sub += 0x10;
	}
	event->num = max_ts;

	/* Indicate there is new event. */
	if (event->num > last)
		ptp->ts_status |= tsi_bit;
}  /* ptp_read_event */

static int ptp_poll_event(struct ptp_info *ptp, u8 tsi)
{
	int max_ts;
	int num;
	u16 status;
	u16 tsi_bit = (1 << tsi);
	int reg = TSn_EVENT_STATUS(tsi);
	struct ptp_event *event = &ptp->events[tsi];

	status = ptp_read(ptp->ptpdev, ADDR_16, reg);
	num = (status & TS_NO_EVENT_DET_MASK) >> 1;
	max_ts = (num <= event->max) ? num : event->max;
	if (max_ts > event->num) {
		ptp_acquire(ptp);
		status = ptp_read(ptp->ptpdev, ADDR_16, TS_INT_STATUS);
		if (status & tsi_bit)
			ptp_write(ptp->ptpdev, ADDR_16, TS_INT_STATUS,
				tsi_bit);
		ptp_read_event(ptp, tsi);
		ptp->ts_status = 0;
		ptp_release(ptp);
		return true;
	}
	return false;
}  /* ptp_poll_event */

static inline void ptp_tx_reset(void *ptpdev, u16 tso_bit)
{
	ptp_write(ptpdev, ADDR_16, TRIG_RESET, tso_bit);
}  /* ptp_tx_reset */

static inline void ptp_gpo_reset(struct ptp_info *ptp, int gpo, u16 tso_bit)
{
	ptp_tx_reset(ptp->ptpdev, tso_bit);
	ptp->cascade_gpo[gpo].tso &= ~tso_bit;
}  /* ptp_gpo_reset */

static void ptp_tx_off(struct ptp_info *ptp, u8 tso)
{
	u16 data;
	u16 tso_bit = (1 << tso);
	void *ptpdev = ptp->ptpdev;

	/* Disable previous trigger out if not already completed. */
	data = ptp_read(ptpdev, ADDR_16, TRIG_EN);
	if (data & tso_bit) {
		data &= ~tso_bit;
		ptp_write(ptpdev, ADDR_16, TRIG_EN, data);
	}

	/*
	 * Using cascade mode previously need to reset the trigger output so
	 * that an errorneous output will not be generated during next
	 * cascade mode setup.
	 */
	if (ptp->cascade_tx & tso_bit) {
		ptp_gpo_reset(ptp, ptp->outputs[tso].gpo, tso_bit);
		ptp->cascade_tx &= ~tso_bit;
	} else {
		int reg = TRIGn_CONF_1(tso);

		data = ptp_read(ptpdev, ADDR_16, reg);
		if (data & TRIG_CASCADE_EN) {
			data &= ~TRIG_CASCADE_EN;
			data &= ~TRIG_CASCADE_TAIL;
			data |= TRIG_CASCADE_UPS_MASK;
			ptp_write(ptpdev, ADDR_16, reg, data);
		}
	}
}  /* ptp_tx_off */

static void ptp_tso_off(struct ptp_info *ptp, u8 tso, u16 tso_bit)
{
	ptp_tx_off(ptp, tso);
	ptp->tso_intr &= ~tso_bit;
	ptp->tso_used &= ~tso_bit;
	ptp->tso_dev[tso] = NULL;
}  /* ptp_tso_off */

static void ptp_tx_on(void *ptpdev, u8 tso)
{
	u16 data;
	u16 tso_bit = (1 << tso);

	data = ptp_read(ptpdev, ADDR_16, TRIG_EN);
	data |= tso_bit;
	ptp_write(ptpdev, ADDR_16, TRIG_EN, data);
}  /* ptp_tx_on */

static void ptp_tx_trigger_time(void *ptpdev, u8 tso, u32 sec, u32 nsec)
{
	int reg;

	reg = TRIGn_TARGET_SEC_L(tso);
	ptp_write(ptpdev, ADDR_32, reg + 2, sec >> 16);
	ptp_write(ptpdev, ADDR_32, reg, sec);
	reg = TRIGn_TARGET_NANOSEC_L(tso);
	ptp_write(ptpdev, ADDR_32, reg + 2, nsec >> 16);
	ptp_write(ptpdev, ADDR_32, reg, nsec);
}  /* ptp_tx_trigger_time */

#define INIT_NSEC			40
#define MIN_CYCLE_NSEC			8
#define MIN_GAP_NSEC			120
#define PULSE_NSEC			8

static int check_cascade(struct ptp_info *ptp, int first, int total,
	u16 *repeat, u32 sec, u32 nsec)
{
	struct ptp_output *cur;
	struct ptp_output *next;
	struct ptp_output *prev;
	int diff;
	int i;
	int tso;
	int min_cnt;
	int cnt;

	tso = first;
	cur = &ptp->outputs[tso];
	next = &ptp->outputs[first + total];
	next->start = cur->start;
	add_nsec(&next->start, cur->iterate);
	for (i = 0; i < total; i++, tso++) {
		cur = &ptp->outputs[tso];
		cur->stop = cur->start;
		add_nsec(&cur->stop, cur->len);
		next = &ptp->outputs[tso + 1];
		calc_udiff(&cur->stop, &next->start, &cur->gap);
		if ((cur->gap.sec < 0 || (!cur->gap.sec && cur->gap.nsec < 0))
				&& (i < total - 1 || 1 != *repeat)) {
			dbg_msg("gap too small: %d=%d\n", i, cur->gap.nsec);
			return 1;
		}
	}
	if (1 == *repeat)
		goto check_cascade_done;

	min_cnt = *repeat;
	tso = first + 1;
	for (i = 1; i < total; i++, tso++) {
		cur = &ptp->outputs[tso];
		prev = &ptp->outputs[tso - 1];
		if (cur->iterate < prev->iterate) {
			diff = prev->iterate - cur->iterate;
			cnt = prev->gap.nsec / diff + 1;
		} else if (cur->iterate > prev->iterate) {
			diff = cur->iterate - prev->iterate;
			cnt = cur->gap.nsec / diff + 1;
		} else
			cnt = *repeat;
		if (min_cnt > cnt)
			min_cnt = cnt;
	}
	if (*repeat > min_cnt)
		*repeat = min_cnt;
	prev = &ptp->outputs[first + tso];
	for (cnt = 0; cnt < *repeat; cnt++) {
		tso = first;
		for (i = 0; i < total; i++, tso++) {
			cur = &ptp->outputs[tso];
			next = &ptp->outputs[tso + 1];
			dbg_msg("%d: %d:%9d %d %d:%9d %d: %d:%9d\n",
				i, cur->start.sec, cur->start.nsec, cur->len,
				cur->gap.sec, cur->gap.nsec, cur->iterate,
				cur->stop.sec, cur->stop.nsec);
			if (cur->stop.sec > next->start.sec ||
					(cur->stop.sec == next->start.sec &&
					cur->stop.nsec > next->stop.nsec))
				dbg_msg("> %d %d:%9d %d:%9d\n", i,
					cur->stop.sec, cur->stop.nsec,
					next->start.sec, next->start.nsec);
			add_nsec(&cur->start, cur->iterate);
			cur->stop = cur->start;
			add_nsec(&cur->stop, cur->len);
			if (!i)
				prev->start = cur->start;
		}
		dbg_msg("%d:%9d\n", prev->start.sec, prev->start.nsec);
	}

check_cascade_done:
	tso = first;
	cur = &ptp->outputs[tso];
	if (cur->trig.sec >= sec)
		return 0;

	for (i = 0; i < total; i++, tso++) {
		cur = &ptp->outputs[tso];
		cur->trig.sec += sec;
		add_nsec(&cur->trig, nsec);
	}
	return 0;
}

static void ptp_tx_event(struct ptp_info *ptp, u8 tso, u8 gpo, u8 event,
	u32 pulse, u32 cycle, u16 cnt, u32 sec, u32 nsec, u32 iterate,
	int intr, int now, int opt)
{
	int reg;
	u16 data;
	u16 tso_bit = (1 << tso);
	void *ptpdev = ptp->ptpdev;
	struct ptp_output *cur = &ptp->outputs[tso];

	/* Hardware immediately keeps level high on new GPIO if not reset. */
	if (cur->level && gpo != cur->gpo)
		ptp_gpo_reset(ptp, cur->gpo, tso_bit);

	/* Config pattern. */
	reg = TRIGn_CONF_1(tso);
	data = ((event & 0x7) << 4);
	data |= (gpo & 0xf);
	if (intr)
		data |= TRIG_NOTIFY;
	if (now)
		data |= TRIG_NOW;
	if (opt)
		data |= TRIG_CLK_OPT;
	data |= TRIG_CASCADE_UPS_MASK;
	ptp_write(ptpdev, ADDR_16, reg, data);

	/* Config pulse width. */
	if (TRIG_REG_OUTPUT == event) {
		reg = TRIGn_BIT_PATTERN(tso);
		ptp_write(ptpdev, ADDR_16, reg, (u16) pulse);
		cur->level = 0;
		if (cnt) {
			reg = cnt - 1;
			reg %= 16;
			while (reg) {
				pulse >>= 1;
				reg--;
			}
			if (pulse & 1)
				cur->level = 1;
		}
		pulse = 0;
	} else if (event >= TRIG_NEG_PULSE) {
		if (0 == pulse)
			pulse = 1;
		else if (tso != 11 && pulse > 0xffff)
			pulse = 0xffff;
		reg = TRIGn_PULSE_WIDTH(tso);
		ptp_write(ptpdev, ADDR_16, reg, (u16) pulse);
		if (11 == tso) {
			if (pulse > 0xffffff)
				pulse = 0xffffff;
			data = ptp_read(ptpdev, ADDR_16, TRIG_PPS_WS);
			data &= ~TRIG_PPS_WS_MASK;
			data |= ((pulse >> 16) & TRIG_PPS_WS_MASK);
			ptp_write(ptpdev, ADDR_16, TRIG_PPS_WS, data);
		}
	}

	/* Config cycle width. */
	if (event >= TRIG_NEG_PERIOD) {
		int min_cycle = pulse * PULSE_NSEC + MIN_CYCLE_NSEC;

		if (cycle < min_cycle)
			cycle = min_cycle;
		reg = TRIGn_CYCLE_WIDTH_L(tso);
		ptp_write(ptpdev, ADDR_32, reg + 2, cycle >> 16);
		ptp_write(ptpdev, ADDR_32, reg, cycle);

		/* Config trigger count. */
		reg = TRIGn_PER_OCCUR(tso);
		ptp_write(ptpdev, ADDR_16, reg, cnt);
	}

	cur->len = 0;
	if (event >= TRIG_NEG_PERIOD) {
		if (cnt)
			cur->len += cycle * cnt;
		else
			cur->len += 0xF0000000;
	} else if (event >= TRIG_NEG_PULSE)
		cur->len += pulse * PULSE_NSEC;
	else
		cur->len += MIN_CYCLE_NSEC;

	cur->start.sec = sec;
	cur->start.nsec = nsec;
	cur->iterate = iterate;
	cur->trig = cur->start;
	cur->stop = cur->start;
	add_nsec(&cur->stop, cur->len);
	cur->gpo = gpo;

	switch (event) {
	case TRIG_POS_EDGE:
	case TRIG_NEG_PULSE:
	case TRIG_NEG_PERIOD:
		cur->level = 1;
		break;
	case TRIG_REG_OUTPUT:
		break;
	default:
		cur->level = 0;
		break;
	}

	if (ptp->cascade)
		return;

	/*
	 * Need to reset after completion.  Otherwise, this output pattern
	 * does not behave consistently in cascade mode.
	 */
	if (TRIG_NEG_EDGE == event)
		ptp->cascade_tx |= tso_bit;

	ptp->cascade_gpo[gpo].total = 0;
	if (cur->level)
		ptp->cascade_gpo[gpo].tso |= tso_bit;
	else
		ptp->cascade_gpo[gpo].tso &= ~tso_bit;

	/* Config trigger time. */
	ptp_tx_trigger_time(ptpdev, tso, sec, nsec);

	/* Enable trigger. */
	ptp_tx_on(ptpdev, tso);
}  /* ptp_tx_event */

static void ptp_pps_event(struct ptp_info *ptp, u8 gpo, u32 sec)
{
	int reg;
	u16 data;
	u32 nsec;
	u32 pulse = (20000000 / 8);	/* 20 ms */
	u32 cycle = 1000000000;
	u16 cnt = 0;
	u8 tso = ptp->pps_tso;
	u8 event = TRIG_POS_PERIOD;
	void *ptpdev = ptp->ptpdev;

	ptp_tx_off(ptp, tso);

	/* Config pattern. */
	reg = TRIGn_CONF_1(tso);
	data = ((event & 0x7) << 4);
	data |= (gpo & 0xf);
	data |= TRIG_NOTIFY;
	data |= TRIG_NOW;
	data |= TRIG_CASCADE_UPS_MASK;
	ptp_write(ptpdev, ADDR_16, reg, data);

	/* Config pulse width. */
	reg = TRIGn_PULSE_WIDTH(tso);
	if (11 != tso && pulse > 0xffff)
		pulse = 0xffff;
	ptp_write(ptpdev, ADDR_16, reg, (u16) pulse);
	if (11 == tso) {
		if (pulse > 0xffffff)
			pulse = 0xffffff;
		data = ptp_read(ptpdev, ADDR_16, TRIG_PPS_WS);
		data &= ~TRIG_PPS_WS_MASK;
		data |= ((pulse >> 16) & TRIG_PPS_WS_MASK);
		ptp_write(ptpdev, ADDR_16, TRIG_PPS_WS, data);
	}

	/* Config cycle width. */
	reg = TRIGn_CYCLE_WIDTH_L(tso);
	ptp_write(ptpdev, ADDR_32, reg + 2, cycle >> 16);
	ptp_write(ptpdev, ADDR_32, reg, cycle);

	/* Config trigger count. */
	reg = TRIGn_PER_OCCUR(tso);
	ptp_write(ptpdev, ADDR_16, reg, cnt);

	/* Config trigger time. */
	if (ptp->pps_offset >= 0)
		nsec = ptp->pps_offset;
	else {
		nsec = NANOSEC_IN_SEC + ptp->pps_offset;
		sec--;
	}
	ptp_tx_trigger_time(ptpdev, tso, sec, nsec);

	/* Enable trigger. */
	ptp_tx_on(ptpdev, tso);
}  /* ptp_pps_event */

#ifdef ENABLE_10_MHZ_CLK
static void ptp_10MHz(struct ptp_info *ptp, u8 tso, u8 gpo, u32 sec)
{
	int i;
	int reg;
	u16 data;
	u32 nsec;
	u32 pulse = 6;
	u32 cycle = 200;
	u16 cnt = 0;
	u8 event = TRIG_POS_PERIOD;
	void *ptpdev = ptp->ptpdev;

	/* Config trigger time. */
	if (ptp->pps_offset >= 0)
		nsec = ptp->pps_offset;
	else {
		nsec = NANOSEC_IN_SEC + ptp->pps_offset;
		sec--;
	}
	for (i = 0; i < 2; i++) {
		ptp_tx_off(ptp, tso);

		/* Config pattern. */
		reg = TRIGn_CONF_1(tso);
		data = ((event & 0x7) << 4);
		data |= (gpo & 0xf);
		data |= TRIG_NOTIFY;
		data |= TRIG_CASCADE_UPS_MASK;
		if (1 == tso)
			data |= TRIG_CLK_OPT;
		ptp_write(ptpdev, ADDR_16, reg, data);

		/* Config pulse width. */
		reg = TRIGn_PULSE_WIDTH(tso);
		if (11 != tso && pulse > 0xffff)
			pulse = 0xffff;
		ptp_write(ptpdev, ADDR_16, reg, (u16) pulse);
		if (11 == tso) {
			if (pulse > 0xffffff)
				pulse = 0xffffff;
			data = ptp_read(ptpdev, ADDR_16, TRIG_PPS_WS);
			data &= ~TRIG_PPS_WS_MASK;
			data |= ((pulse >> 16) & TRIG_PPS_WS_MASK);
			ptp_write(ptpdev, ADDR_16, TRIG_PPS_WS, data);
		}

		/* Config cycle width. */
		reg = TRIGn_CYCLE_WIDTH_L(tso);
		ptp_write(ptpdev, ADDR_32, reg + 2, cycle >> 16);
		ptp_write(ptpdev, ADDR_32, reg, cycle);

		/* Config trigger count. */
		reg = TRIGn_PER_OCCUR(tso);
		ptp_write(ptpdev, ADDR_16, reg, cnt);

		ptp_tx_trigger_time(ptpdev, tso, sec, nsec);

		/* Enable trigger. */
		ptp_tx_on(ptpdev, tso);

		tso = 1;
#if 0
		cycle -= 8;
#endif
		nsec += 12 * 8;
	}
}  /* ptp_10MHz */
#endif

static void ptp_tx_cascade_on(void *ptpdev, u8 tso, u8 first, u8 last,
	u16 repeat)
{
	int reg;
	u16 data;
	int repeat_reg = 0;

	reg = TRIGn_CONF_1(tso);
	data = ptp_read(ptpdev, ADDR_16, reg);
	data |= TRIG_CASCADE_EN;
	data &= ~TRIG_CASCADE_UPS_MASK;
	if (tso == first)
		data |= ((last & 0xf) << 10);
	else
		data |= (((tso - 1) & 0xf) << 10);
	if (repeat && tso == last) {
		data |= TRIG_CASCADE_TAIL;
		if (((data >> 4) & 0xf) != TRIG_REG_OUTPUT)
			repeat_reg = TRIGn_BIT_PATTERN(tso);
		else
			repeat_reg = TRIGn_PULSE_WIDTH(tso);
	}
	ptp_write(ptpdev, ADDR_16, reg, data);
	if (repeat_reg)
		ptp_write(ptpdev, ADDR_16, repeat_reg, repeat - 1);
}  /* ptp_tx_cascade_on */

static void ptp_tx_cascade_cycle(void *ptpdev, u8 tso, u32 nsec)
{
	int reg;

	reg = TRIGn_ITERATE_TIME_L(tso);
	ptp_write(ptpdev, ADDR_32, reg + 2, nsec >> 16);
	ptp_write(ptpdev, ADDR_32, reg, nsec);
}  /* ptp_tx_cascade_cycle */

static int ptp_tx_cascade(struct ptp_info *ptp, u8 first, u8 total,
	u16 repeat, u32 sec, u32 nsec, int intr)
{
	int i;
	u8 tso;
	u8 last;
	void *ptpdev = ptp->ptpdev;
	struct ptp_output *cur;

	last = first + total - 1;
	if (last >= MAX_TRIG_UNIT)
		return 1;
	if (check_cascade(ptp, first, total, &repeat, sec, nsec)) {
		dbg_msg("cascade repeat timing is not right\n");
		return 1;
	}
	tso = first;
	for (i = 0; i < total; i++, tso++) {
		cur = &ptp->outputs[tso];
		ptp_tx_trigger_time(ptpdev, tso, cur->trig.sec,
			cur->trig.nsec);
		ptp_tx_cascade_cycle(ptpdev, tso, cur->iterate);
		ptp_tx_cascade_on(ptpdev, tso, first, last, repeat);
		ptp->cascade_tx |= (1 << tso);
	}

	/* Do not reset last unit to keep level high. */
	if (ptp->outputs[last].level) {
		ptp->cascade_tx &= ~(1 << last);
		ptp->cascade_gpo[ptp->outputs[last].gpo].tso |= (1 << last);
	} else
		ptp->cascade_gpo[ptp->outputs[last].gpo].tso &= ~(1 << last);
	ptp_tx_on(ptpdev, first);
	return 0;
}  /* ptp_tx_cascade */

#ifdef ENABLE_IRIG
static int irig_cnt = 20;
static struct ptp_utime irig_up[20];
static struct ptp_utime irig_dn[20];
static int irig_up_cnt;
static int irig_dn_cnt;

static void set_irig(struct ptp_info *ptp)
{
	ptp->irig.pulse[0] = 8;
	ptp->irig.pulse[1] = 5;
	ptp->irig.pulse[2] = 5;
	ptp->irig.pulse[3] = 5;
	ptp->irig.pulse[4] = 5;
	ptp->irig.pulse[5] = 2;
	ptp->irig.pulse[6] = 2;
	ptp->irig.pulse[7] = 2;
	ptp->irig.pulse[8] = 2;
	ptp->irig.pulse[9] = 8;
	ptp->irig.index = 0;
}  /* set_irig */

static void irig(struct ptp_info *ptp)
{
	u32 pulse;

	if (10 == ptp->irig.index)
		set_irig(ptp);
	pulse = ptp->irig.pulse[ptp->irig.index++];
	add_nsec(&ptp->irig.t, IRIG_INTERVAL);
	if (8 == pulse) {
		pulse *= IRIG_INTERVAL / 10;
		pulse /= 8;
		ptp_tx_event(ptp, 11, IRIG_GPIO, TRIG_POS_PULSE, pulse, 0, 1,
			ptp->irig.t.sec, ptp->irig.t.nsec, 0, 1, 1, 0);
	} else {
		pulse *= IRIG_INTERVAL / 10;
		pulse /= 8;
		ptp_tx_event(ptp, 11, IRIG_GPIO, TRIG_POS_PULSE, pulse, 0, 1,
			ptp->irig.t.sec, ptp->irig.t.nsec, 0, 1, 1, 0);
	}
}  /* irig */
#endif

static void generate_tx_event(struct ptp_info *ptp, int gpo)
{
	struct ptp_utime t;

	get_ptp_time(ptp->ptpdev, &t);
	t.sec += 1;
	if (t.nsec >= (NANOSEC_IN_SEC - ptp->delay_ticks * 50000000))
		t.sec += 1;
	ptp_pps_event(ptp, gpo, t.sec);
#ifdef ENABLE_10_MHZ_CLK
	ptp_10MHz(ptp, ptp->mhz_tso, ptp->mhz_gpo, t.sec);
#endif
}  /* generate_tx_event */

static void prepare_gps(struct ptp_info *ptp)
{
	ptp_acquire(ptp);
	ptp->tsi_used |= (1 << ptp->gps_tsi);
	ptp->events[ptp->gps_tsi].event = 1;
	ptp->events[ptp->gps_tsi].timeout = 0;
	ptp_rx_event(ptp, ptp->gps_tsi, ptp->gps_gpi, TS_DETECT_RISE, true);
	ptp_release(ptp);
}  /* prepare_gps */

static void prepare_pps(struct ptp_info *ptp)
{
#ifdef ENABLE_IRIG
	struct ptp_utime t;
	void *ptpdev = ptp->ptpdev;
#endif

	ptp_acquire(ptp);
	ptp->tso_used |= (1 << ptp->pps_tso);
#ifdef ENABLE_10_MHZ_CLK
	ptp->tso_used |= (1 << ptp->mhz_tso);
	ptp->tso_used |= (1 << 1);
#endif
	generate_tx_event(ptp, ptp->pps_gpo);
#ifdef ENABLE_IRIG
	get_ptp_time(ptpdev, &t);
	ptp->irig.t.sec = t.sec;
	ptp->irig.t.nsec = NANOSEC_IN_SEC - IRIG_INTERVAL;
	ptp->irig.index = 10;
	irig(ptp);
	ptp->tsi_used |= (1 << 7);
	ptp->events[7].event = 1;
	ptp->events[7].timeout = 1000;
	ptp_rx_event(ptp, 7, IRIG_GPIO, TS_DETECT_RISE, true);
	ptp->tsi_used |= (1 << 8);
	ptp->events[8].event = 1;
	ptp->events[8].timeout = 1000;
	ptp_rx_event(ptp, 8, IRIG_GPIO, TS_DETECT_FALL, true);
#endif
	ptp->tsi_used |= (1 << ptp->pps_tsi);
	ptp->events[ptp->pps_tsi].event = 1;
	ptp_rx_event(ptp, ptp->pps_tsi, ptp->pps_gpo, TS_DETECT_RISE, true);
	ptp_release(ptp);
}  /* prepare_pps */

#ifdef PTP_MONITOR
static void get_rx_ts(struct work_struct *work)
{
	struct ptp_info *ptp = container_of(work, struct ptp_info, ts_rx);
	struct ptp_rx_ts *rx;
	int head;

	head = ptp->rx.read;
	while (head != ptp->rx.tail) {
		rx = &ptp->rx.rx[head];
		get_cur_time(ptp, rx->id.msg, rx->id.port, &rx->ts);
		head = (head + 1) & (NUM_OF_TIMESTAMP - 1);
	}
	ptp->rx.read = head;
}  /* get_rx_ts */
#endif

static void convert_scaled_nsec(s64 scaled_nsec, int s, int *sec, int *nsec)
{
	int sign;
	s32 quot;
	s32 rem;

	/* Convert to positive number first. */
	if (scaled_nsec < 0) {
		sign = -1;
		scaled_nsec = -scaled_nsec;
	} else
		sign = 1;
	scaled_nsec >>= s;
	quot = div_s64_rem(scaled_nsec, NSEC_PER_SEC, &rem);

	/* Positive number means clock is faster. */
	if (1 == sign) {
		quot = -quot;
		rem = -rem;
	}
	*sec = quot;
	*nsec = rem;
}  /* convert_scaled_nsec */

static void adj_cur_time(struct ptp_info *ptp)
{
	if (ptp->adjust_offset || ptp->adjust_sec)
		synchronize_clk(ptp);
	generate_tx_event(ptp, ptp->pps_gpo);
	if (ptp->adjust_sec) {
		struct timespec ts;
		struct ptp_utime cur;

		get_ptp_time(ptp->ptpdev, &ptp->cur_time);
		ts = ktime_to_timespec(ktime_get_real());
		cur.sec = ts.tv_sec;
		cur.nsec = ts.tv_nsec;
		calc_udiff(&ptp->cur_time, &cur, &ptp->time_diff);
		ptp->adjust_sec = 0;
	}
}  /* adj_cur_time */

static void set_before_adj(struct ptp_info *ptp, struct ptp_utime *cur)
{
	ptp->adjust_offset += cur->nsec;
	ptp->adjust_offset += ptp->set_delay;
	ptp->adjust_offset += ptp->get_delay;
	cur->nsec = 0;
	if (ptp->adjust_offset > NANOSEC_IN_SEC) {
		ptp->adjust_offset -= NANOSEC_IN_SEC;
		cur->sec++;
	}
	set_ptp_time(ptp->ptpdev, cur);
}   /* set_before_adj */

static void set_cur_time(struct ptp_info *ptp, struct ptp_ts *ts)
{
	struct ptp_utime cur;
	int diff_sec;
	int diff_nsec;

	ptp->adjust_offset = ts->t.nsec - ts->timestamp;
	get_ptp_time(ptp->ptpdev, &cur);
	diff_nsec = ts->t.nsec - ts->timestamp;
	diff_sec = ts->t.sec - cur.sec;
	if (ptp->features & PTP_ADJ_SEC) {
		if (diff_sec) {
			s64 nsec;

			nsec = diff_sec;
			nsec *= NANOSEC_IN_SEC;
			nsec += diff_nsec;
			convert_scaled_nsec(-nsec, 0, &ptp->adjust_sec,
				&ptp->adjust_offset);
		} else {
			ptp->adjust_offset = diff_nsec;
			ptp->adjust_sec = 0;
		}
	} else {
		if (abs(diff_sec) <= 1) {
			diff_nsec += diff_sec * NANOSEC_IN_SEC;
			if (abs(diff_nsec) < NANOSEC_IN_SEC) {
				ptp->adjust_offset = diff_nsec;
				diff_sec = 0;
			}
		}
		if (diff_sec) {
			cur.sec = ts->t.sec;
			set_before_adj(ptp, &cur);
		}
	}
	adj_cur_time(ptp);
}  /* set_cur_time */

static void adj_clock(struct work_struct *work)
{
	struct ptp_info *ptp = container_of(work, struct ptp_info, adj_clk);
	struct ptp_utime cur;

	ptp_acquire(ptp);

	if (!(ptp->features & PTP_ADJ_SEC)) {
		/* Need to adjust second. */
		if (abs(ptp->adjust_sec) > 1) {
			get_ptp_time(ptp->ptpdev, &cur);
			cur.sec += ptp->adjust_sec;
			set_before_adj(ptp, &cur);
		} else {
			ptp->adjust_offset += ptp->adjust_sec * NANOSEC_IN_SEC;
			ptp->adjust_sec = 0;
		}
	}
	adj_cur_time(ptp);
	ptp_release(ptp);
}  /* adj_clock */

static void set_clock(struct work_struct *work)
{
	struct ptp_info *ptp = container_of(work, struct ptp_info, set_clk);
	struct timespec ts;
	struct ptp_utime sys_time;

	ptp_acquire(ptp);
	ts = ktime_to_timespec(ktime_get_real());
	sys_time.sec = ts.tv_sec;
	sys_time.nsec = ts.tv_nsec;
	set_ptp_time(ptp->ptpdev, &ptp->time_set);
	ptp->cur_time = ptp->time_set;
	calc_udiff(&ptp->cur_time, &sys_time, &ptp->time_diff);
#ifdef PTP_MONITOR
	if (ptp->sim)
		ptp->sim = 3;
	ptp->state = 2;
#endif
	generate_tx_event(ptp, ptp->pps_gpo);
	ptp_release(ptp);
}  /* set_clock */

static void synchronize(struct work_struct *work)
{
	struct ptp_info *ptp = container_of(work, struct ptp_info, sync_clk);

	ptp_acquire(ptp);
	synchronize_clk(ptp);
	generate_tx_event(ptp, ptp->pps_gpo);
	ptp_release(ptp);
}  /* synchronize */

static void syntonize(struct work_struct *work)
{
	struct ptp_info *ptp = container_of(work, struct ptp_info, synt_clk);

	if (ptp_acquire(ptp)) {
#ifdef PTP_SPI
		queue_work(ptp->access, &ptp->synt_clk);
#endif
		return;
	}
	set_ptp_adjust(ptp->ptpdev, ptp->adjust);
	if (!ptp->ptp_synt) {
		syntonize_clk(ptp->ptpdev);
		ptp->ptp_synt = true;
	}
	ptp_release(ptp);
}  /* syntonize */

static void execute(struct ptp_info *ptp, struct work_struct *work)
{
#ifdef PTP_SPI
	queue_work(ptp->access, work);
#else
	work->func(work);
#endif
}  /* execute */

static inline void check_ptp_pkt(struct work_struct *work) {}

static void ptp_start(struct ptp_info *ptp, int init)
{
	u16 data;
	void *ptpdev = ptp->ptpdev;
#if 1
	struct timespec ts;
	struct ptp_utime t;
#endif

	mutex_lock(ptp->hwlock);
	data = ptp_read(ptpdev, ADDR_16, PTP_MSG_CONF1);
	if (data == ptp->mode) {
		ptp->cfg = ptp_read(ptpdev, ADDR_16, PTP_MSG_CONF2);
		ptp->domain = ptp_read(ptpdev, ADDR_16, PTP_DOMAIN_VERSION) &
			PTP_DOMAIN_MASK;
		if (!init) {
			mutex_unlock(ptp->hwlock);
			return;
		}
	} else if (!init)
		ptp->mode = data;
	if (ptp->mode != ptp->def_mode) {
		dbg_msg("mode changed: %04x %04x; %04x %04x\n",
			ptp->mode, ptp->def_mode, ptp->cfg, ptp->def_cfg);
		ptp->mode = ptp->def_mode;
		ptp->cfg = ptp->def_cfg;
		ptp->ptp_synt = false;
	}
	dbg_msg("ptp_start: %04x %04x\n",
		ptp->mode, ptp->cfg);
	ptp_write(ptpdev, ADDR_16, PTP_MSG_CONF1, ptp->mode);
	ptp_write(ptpdev, ADDR_16, PTP_MSG_CONF2, ptp->cfg);
	ptp_write(ptpdev, ADDR_16, TRIG_INT_ENABLE, ptp->trig_intr);
	ptp_write(ptpdev, ADDR_16, TS_INT_ENABLE, ptp->ts_intr);
	mutex_unlock(ptp->hwlock);
#if 1
	ts = ktime_to_timespec(ktime_get_real());
	t.sec = ts.tv_sec;
	t.nsec = ts.tv_nsec;
	ptp_acquire(ptp);
	set_ptp_time(ptp->ptpdev, &t);
	ptp->cur_time = t;
	ptp_release(ptp);
#endif
	prepare_pps(ptp);
}  /* ptp_start */

static void ptp_stop(struct ptp_info *ptp)
{
#ifdef PTP_SPI
#ifdef PTP_MONITOR
	flush_work(&ptp->ts_rx);
#ifdef PTP_PROCESS
	flush_work(&ptp->exit_state);
#endif
#endif
	flush_work(&ptp->adj_clk);
	flush_work(&ptp->set_clk);
	flush_work(&ptp->sync_clk);
	flush_work(&ptp->synt_clk);
	flush_workqueue(ptp->access);
#endif
	mutex_lock(ptp->hwlock);
	ptp_write(ptp->ptpdev, ADDR_16, REG_RESET_CTRL,
		GLOBAL_SOFTWARE_RESET);
	ptp_write(ptp->ptpdev, ADDR_16, REG_RESET_CTRL, 0);
	ptp->ptp_synt = false;
	mutex_unlock(ptp->hwlock);
}  /* ptp_stop */

static void init_tx_ts(struct ptp_tx_ts *ts)
{
	ts->ts.timestamp = 0;
	ts->req_time = 0;
	ts->resp_time = 0;
	ts->missed = false;
}  /* init_tx_ts */

static struct ptp_dev_info *find_minor_dev(struct ptp_dev_info *info)
{
	struct ptp_info *ptp = info->ptp;
	struct ptp_dev_info *dev;
	struct ptp_dev_info *prev;

	dev = ptp->dev[info->minor ^ 1];
	prev = ptp->dev[info->minor];
	while (prev != info && prev && dev) {
		prev = prev->next;
		dev = dev->next;
	}
	if (prev != info)
		dev = NULL;
	return dev;
}  /* find_minor_dev */

static void ptp_init_state(struct ptp_info *ptp)
{
	void *ptpdev = ptp->ptpdev;
	int port;
	int reg;
	struct ptp_utime t;

	mutex_lock(&ptp->lock);
	ptp->udp_head = ptp->udp_tail = 0;
	for (reg = 0; reg < MAX_TSM_UDP_CNT; reg++)
		ptp->udp[reg].len = 0;
	mutex_unlock(&ptp->lock);

	ptp_start(ptp, false);

	mutex_lock(&ptp->lock);
	for (port = 0; port < MAX_PTP_PORT; port++) {
		ptp->hw_sync[port].ts.timestamp = 0;
		ptp->hw_sync[port].sending = false;
		ptp->hw_dreq[port].ts.timestamp = 0;
		ptp->hw_dreq[port].sending = false;
		ptp->hw_resp[port].ts.timestamp = 0;
		ptp->hw_resp[port].sending = false;
		init_tx_ts(&ptp->tx_sync[port]);
		init_tx_ts(&ptp->tx_dreq[port]);
		init_tx_ts(&ptp->tx_resp[port]);
		mutex_lock(ptp->hwlock);
		reg = PTP_PORT1_XDELAY_TIMESTAMP_L + PTP_PORT_INTERVAL(port);
		ptp->xdelay_ts[port] = ptp_read(ptpdev, ADDR_32, reg);
		reg = PTP_PORT1_PDRESP_TIMESTAMP_L + PTP_PORT_INTERVAL(port);
		ptp->pdresp_ts[port] = ptp_read(ptpdev, ADDR_32, reg);
		ptp->rx_latency[port] = get_ptp_ingress(ptpdev, port);
		ptp->tx_latency[port] = get_ptp_egress(ptpdev, port);
		ptp->asym_delay[port] = get_ptp_asym(ptpdev, port);
		ptp->peer_delay[port] = get_ptp_link(ptpdev, port);
		set_ptp_link(ptpdev, port, 0);
		mutex_unlock(ptp->hwlock);
		dbg_msg("%d = %d %d %d; %d\n", port,
			ptp->rx_latency[port],
			ptp->tx_latency[port],
			ptp->asym_delay[port],
			ptp->peer_delay[port]);
	}

#ifdef PTP_MONITOR
	ptp->sync_init_cnt = 8;
	for (port = 0; port < MAX_DELAY_REQ_CNT; port++)
		ptp->e2e.delay_resp[port].seqid = -1;
	ptp->e2e.index = ptp->e2e.mask;
	ptp->path_delay = 0;
	ptp->get_time_jiffies = 0;
#endif

	ptp->adjust_offset = 0;
	ptp->offset_changed = 0;
	ptp->sync_diff = 0;
	ptp->sync_time.nsec = 0;
	ptp->last_offset = 0;

	ptp->sim = 0;
	ptp->state = 1;
	mutex_unlock(&ptp->lock);

	ptp_acquire(ptp);
	if (!ptp->ptp_synt) {
		syntonize_clk(ptpdev);
		ptp->ptp_synt = true;
	}
	get_ptp_time(ptpdev, &t);
	ptp->cur_time = t;
	ptp_release(ptp);
#ifdef PTP_MONITOR
	ptp->features |= PTP_MON;
#endif
}  /* ptp_init_state */

static void ptp_exit_state(struct ptp_info *ptp)
{
#ifdef PTP_MONITOR
	ptp->features &= ~PTP_MON;
	ptp->path_delay = 0;
#endif
	if (ptp->mode & PTP_MASTER) {
		u16 data;

		mutex_lock(ptp->hwlock);
		data = ptp_read(ptp->ptpdev, ADDR_16, PTP_MSG_CONF1);
		data &= ~PTP_MASTER;
		ptp_write(ptp->ptpdev, ADDR_16, PTP_MSG_CONF1, data);
		mutex_unlock(ptp->hwlock);
		ptp->mode &= ~PTP_MASTER;
		ptp->def_mode &= ~PTP_MASTER;
	}
	ptp->adjust_offset = 0;
	ptp->offset_changed = 0;
	ptp->sync_diff = 0;
	ptp->sync_time.nsec = 0;
	ptp->last_offset = 0;
	ptp->state = 0;
	ptp->sim = 0;
}  /* ptp_exit_state */

#ifdef PTP_MONITOR
#ifdef PTP_PROCESS
static void reset_state(struct work_struct *work)
{
	struct ptp_info *ptp = container_of(work, struct ptp_info, exit_state);

	ptp_exit_state(ptp);
}  /* reset_state */

static int calc_pi(struct ptp_info *ptp, int offset, int *corr)
{
	int abs_offset;
	int i;
	int P;

	abs_offset = abs(offset);
	P = (abs_offset * ptp->KP + 50) / 100;
	i = (abs_offset * ptp->KI + 50) / 100;
	if (offset < 0) {
		P = -P;
		i = -i;
	}
	ptp->I += i;
	*corr = P + ptp->I;
	if (abs(*corr) >= abs_offset)
		return -1;
	return 0;
}  /* calc_pi */

#define MIN_SYNC_INTERVAL		40000000
#define STABLE_OFFSET			(3 * 1000 / \
	(MIN_SYNC_INTERVAL / 1000000))

static void syntonize_proc(struct ptp_info *ptp, u32 sec_hi,
	struct ptp_utime *t)
{
	static struct ptp_utime last;

	struct ptp_time diff_t;
	u32 abs_drift;
	u64 interval64;
	int drift;
	int abs_diff;
	int abs_offset;
	int diff;
	int offset;
	int offset_diff;
	int rc;
	char a;
	char b;
	s64 corr;
	struct ptp_ltime t1;
	struct ptp_ltime t2;
	struct ptp_ltime offset_64;

	if (ptp->sim) {
		diff = ptp->sync_time.sec - t->sec;
		abs_diff = abs(diff);
		if (abs_diff > 1) {
			if (ptp->features & PTP_ADJ_SEC) {
				s64 nsec;

				nsec = diff;
				nsec *= NANOSEC_IN_SEC;
				nsec += (s64) ptp->sync_time.nsec - t->nsec;
				convert_scaled_nsec(nsec, 0, &ptp->adjust_sec,
					&ptp->adjust_offset);
				execute(ptp, &ptp->adj_clk);
			} else {
				ptp->time_set.sec = t->sec;
				if (t->nsec >= 500000000)
					ptp->time_set.nsec = 500000000;
				else
					ptp->time_set.nsec = 0;
				execute(ptp, &ptp->set_clk);
			}
			ptp->sim = 1;
			ptp->state = 1;
			ptp->sync_init_cnt = 8;
		} else if (1 == ptp->sim) {
			ptp->sim = 3;
			ptp->state = 2;
		}
	}

	calc_udiff(&last, t, &diff_t);
	last.nsec = t->nsec;
	last.sec = t->sec;

	interval64 = diff_t.sec;
	interval64 *= NANOSEC_IN_SEC;
	interval64 += diff_t.nsec;

	/*
	 * offsetFromMaster =
	 * t2 - preciseOriginTimestamp - meanPathDelay -
	 * (correctionField_Sync + delayAsymmetry) - correctionField_Follow_Up
	 * t2 - t1 - meanPathDelay -
	 * (corrSync + delayAsymmetry + corrFollow_Up)
	 */
	t2.sec = ((s64) ptp->sec_hi << 32) | ptp->sync_time.sec;
	t2.nsec = (s64) ptp->sync_time.nsec << SCALED_NANOSEC_SHIFT;
	t1.sec = ((s64) sec_hi << 32) | t->sec;
	t1.nsec = (s64) t->nsec << SCALED_NANOSEC_SHIFT;
	corr = ptp->corr_sync + ptp->corr_follow_up;
	calc_diff64(&t1, &t2, &offset_64);
	t1.sec = 0;
	t1.nsec = corr;
	t2.sec = offset_64.sec;
	t2.nsec = offset_64.nsec;
	calc_diff64(&t1, &t2, &offset_64);
	offset = (int)((offset_64.nsec + SCALED_NANOSEC_MULT / 2) >>
		SCALED_NANOSEC_SHIFT);

	abs_offset = abs(offset);

	offset_diff = offset - ptp->last_offset;
	offset_diff -= ptp->offset_changed;
	ptp->last_offset = offset;

	diff = offset_diff;
	abs_diff = abs(diff);
	ptp->offset_changed = 0;

	if (2 == ptp->state && 0 == (int) offset_64.sec)
		ptp->state = 3;
	else if (3 == ptp->state) {
		ptp->state = 4;
		ptp->sync_init_cnt = 2;
	}

	if (ptp->sync_init_cnt) {
		--ptp->sync_init_cnt;
		dbg_msg(" sync: %u:%9u in: %u:%9u\n",
			ptp->sync_time.sec, ptp->sync_time.nsec,
			t->sec, t->nsec);
		dbg_msg(" o:%d, i:%llu\n",
			offset, interval64);
		dbg_msg(" d:%d sd:%lld s=%d\n", diff_t.sec, ptp->sync_diff,
			ptp->state);
	}

	/* Calculation does not work for big interval. */
	if (ptp->state < 4)
		return;

	ptp->path_offset = offset - ptp->path_delay;

	a = ' ';

	if (5 <= ptp->sim) {
		/* Calculate drift per second. */
		abs_drift = drift_in_sec(abs_offset, interval64);
		if (abs_drift < 10000) {
			if (offset < 0)
				drift = -abs_drift;
			else
				drift = abs_drift;
			if (abs_diff < 10 && abs_drift < STABLE_OFFSET) {
				if (++ptp->stable_cnt >= 3) {
					ptp->stable_cnt = 0;
					ptp->drift = ptp->drift_set;
					ptp->I = 0;
				}
			} else
				ptp->stable_cnt = 0;
			rc = calc_pi(ptp, drift, &drift);
			drift += ptp->drift;
			ptp->drift_set = drift;
			if (rc && abs_drift > 1000) {
				ptp->I = 0;
				ptp->drift = ptp->drift_set;
			}
			ptp->adjust = clk_adjust_val(drift, NANOSEC_IN_SEC);
			execute(ptp, &ptp->synt_clk);
			a = ptp->sim + '0';
		} else
			ptp->sim = 3;
	}
	ptp->adjust_offset = 0;
	if (3 == ptp->sim) {
		/* Calculate drift per second. */
		abs_drift = drift_in_sec(abs_diff, interval64);
		if (diff < 0)
			drift = -abs_drift;
		else
			drift = abs_drift;
		ptp->drift += drift;
		if (abs(ptp->drift) >= MAX_DRIFT_CORR) {
			if (diff < 0)
				ptp->drift = -MAX_DRIFT_CORR;
			else
				ptp->drift = MAX_DRIFT_CORR;
		}
		ptp->drift_set = ptp->drift;
		ptp->adjust = clk_adjust_val(ptp->drift, NANOSEC_IN_SEC);
		execute(ptp, &ptp->synt_clk);
		a = ptp->sim + '0';
		if (abs_diff < 40) {
			if (++ptp->stable_cnt >= 3) {
				ptp->stable_cnt = 0;
				ptp->adjust_offset = -offset;
				ptp->I = 0;
				ptp->sim = 5;
			}
		} else {
			ptp->stable_cnt = 0;
		}
	}
	b = ' ';
	abs_diff = abs(diff);

	if (ptp->adjust_offset) {
		b = '<';
		ptp->adjust_offset = -offset;
		execute(ptp, &ptp->sync_clk);
	}
	if (4 == ptp->state) {
		if ((diff == offset && abs_diff < 500) ||
				(abs_offset < 40 &&
				abs(ptp->drift - ptp->drift_set) < 10))
			ptp->state = 5;
	}
	if (b == ' ')
		b = ptp->state + '0';
	if (jiffies - ptp->last_dbg_jiffies >= 50) {
		dbg_msg("\n");
		dbg_msg("%csynt: %10lld - %10lld = <%d>\n", a,
			ptp->sync_diff, interval64, diff);
		dbg_msg("%csync: %10u - %10u = <%d> - %d = [%d]\n", b,
			ptp->sync_time.nsec, t->nsec, offset, ptp->path_delay,
			ptp->path_offset);
		dbg_msg(" %08x:%9llu; drift:%6d - %6d = %6d\n",
			ptp->cur_time.sec, interval64,
			ptp->drift, ptp->drift_set,
			ptp->drift - ptp->drift_set);
#if 1
		if (abs(ptp->drift - ptp->drift_set) > 100)
			dbg_msg("I = %d\n", ptp->I);
#endif
		ptp->last_dbg_jiffies = jiffies;
		if (ptp->get_time_jiffies &&
				jiffies - ptp->get_time_jiffies >= 200) {
			if (ptp->software_running)
				ptp->software_running = 0;
			else
				execute(ptp, &ptp->exit_state);
		}
	}
}  /* syntonize_proc */

static void proc_sync_msg(struct ptp_info *ptp)
{
	struct ptp_time diff;
	struct ptp_utime corr;
	struct ptp_utime in_time;

	in_time = ptp->sync_in_time;
	if (ptp->corr_sync || ptp->corr_follow_up) {
		corr.sec = 0;
		corr.nsec = (ptp->corr_sync + ptp->corr_follow_up +
			SCALED_NANOSEC_MULT / 2) >> SCALED_NANOSEC_SHIFT;
		calc_udiff(&corr, &in_time, &diff);
		in_time.sec = diff.sec;
		in_time.nsec = diff.nsec;
	}
	ptp->proc_sync = false;
	if (ptp->sync_time.nsec) {
		calc_udiff(&ptp->sync_time_adj, &in_time, &diff);
		ptp->sync_diff = diff.sec;
		ptp->sync_diff *= NANOSEC_IN_SEC;
		ptp->sync_diff += diff.nsec;
		if (abs(ptp->sync_diff) < MIN_SYNC_INTERVAL)
			return;
	}
	ptp->sync_time_adj = in_time;
	ptp->sync_time = ptp->sync_in_time;
	ptp->proc_sync = true;
}  /* proc_sync_msg */

static void adjust_rx_latency(struct ptp_info *ptp, struct ptp_utime *in)
{
	int nsec = in->nsec;

	nsec -= ptp->rx_latency[ptp->in_port];
	while (nsec < 0) {
		nsec += NANOSEC_IN_SEC;
		(in->sec)--;
	}
	in->nsec = nsec;
}  /* adjust_rx_latency */

static void handle_sync(struct ptp_info *ptp, struct ptp_msg *msg,
	s64 corr64)
{
	struct ptp_utime rx;
	u32 sec_hi;
	int asym_delay;

	rx.nsec = ntohl(msg->data.sync.originTimestamp.nsec);
	rx.sec = ntohl(msg->data.sync.originTimestamp.sec_lo);
	sec_hi = ntohl(msg->data.sync.originTimestamp.sec_hi);

	ptp->master_port = ptp->in_port;

	adjust_rx_latency(ptp, &ptp->in_time);

	asym_delay = ptp->asym_delay[ptp->in_port] * SCALED_NANOSEC_MULT;
	corr64 += asym_delay;
	ptp->corr_sync = corr64;
	ptp->corr_follow_up = 0;

	ptp->sync_in_time = ptp->in_time;
	if (!(msg->hdr.flagField.flag.twoStepFlag)) {
		proc_sync_msg(ptp);
		if (ptp->proc_sync)
			syntonize_proc(ptp, sec_hi, &rx);
	}
}  /* handle_sync */

static void handle_follow_up(struct ptp_info *ptp, struct ptp_msg *msg,
	s64 corr64)
{
	struct ptp_utime rx;
	u32 sec_hi;

	rx.nsec = ntohl(msg->data.follow_up.preciseOriginTimestamp.nsec);
	rx.sec = ntohl(msg->data.follow_up.preciseOriginTimestamp.sec_lo);
	sec_hi = ntohl(msg->data.follow_up.preciseOriginTimestamp.sec_hi);

	ptp->corr_follow_up = corr64;

	proc_sync_msg(ptp);
	if (ptp->proc_sync)
		syntonize_proc(ptp, sec_hi, &rx);
}  /* handle_follow_up */

static void handle_delay_req(struct ptp_info *ptp, struct ptp_msg *msg,
	s64 corr64)
{
	ptp->corr_delay_req = corr64;
}  /* handle_delay_req */

static void handle_delay_resp(struct ptp_info *ptp, struct ptp_msg *msg,
	s64 corr64)
{
#if 0
	struct ptp_utime rx;

	rx.nsec = ntohl(msg->data.delay_resp.receiveTimestamp.nsec);
	rx.sec = ntohl(msg->data.delay_resp.receiveTimestamp.sec_lo);
#endif
}  /* handle_delay_resp */

static void handle_pdelay_resp(struct ptp_info *ptp, struct ptp_msg *msg,
	s64 corr64)
{
#if 0
	struct ptp_utime rx;
	int asym_delay;

	rx.nsec = ntohl(msg->data.pdelay_resp.requestReceiptTimestamp.nsec);
	rx.sec = ntohl(msg->data.pdelay_resp.requestReceiptTimestamp.sec_lo);

	adjust_rx_latency(ptp, &ptp->in_time);

	asym_delay = ptp->asym_delay[ptp->in_port] * SCALED_NANOSEC_MULT;
	corr64 += asym_delay;
#endif
}  /* handle_pdelay_resp */

static void handle_pdelay_resp_follow_up(struct ptp_info *ptp,
	struct ptp_msg *msg, s64 corr64)
{
#if 0
	struct ptp_utime rx;

	rx.nsec = ntohl(msg->data.pdelay_resp_follow_up.
		responseOriginTimestamp.nsec);
	rx.sec = ntohl(msg->data.pdelay_resp_follow_up.
		responseOriginTimestamp.sec_lo);

	/*
	 * Pdelay_Resp_Follow_Up may have t3 in data or (t3 - t2) in
	 * correctionField.
	 */
#endif
}  /* handle_pdelay_resp_follow_up */

static void proc_ptp_rx(struct ptp_info *ptp, struct ptp_msg *msg)
{
	int corr;
	s64 corr64;
	int nsec_hi;
	u32 nsec_lo;
	struct ptp_ts ts;

	ts.timestamp = ntohl(msg->hdr.reserved3);
	update_ts(&ts, ptp->cur_time.sec);
	ptp->in_time.sec = ts.t.sec;
	ptp->in_time.nsec = ts.t.nsec;
	ptp->in_time_org.sec = ts.t.sec;
	ptp->in_time_org.nsec = ts.t.nsec;
	ptp->in_port = msg->hdr.reserved2 - 1;

	nsec_hi = ntohl(msg->hdr.correctionField.scaled_nsec_hi);
	nsec_lo = ntohl(msg->hdr.correctionField.scaled_nsec_lo);
	corr = (nsec_hi << 16) | (nsec_lo >> 16);
	corr64 = ((s64) nsec_hi << 32) | nsec_lo;

	switch (msg->hdr.messageType) {
	case SYNC_MSG:
		handle_sync(ptp, msg, corr64);
		break;
	case FOLLOW_UP_MSG:
		handle_follow_up(ptp, msg, corr64);
		break;
	case DELAY_RESP_MSG:
		if (!memcmp(&ptp->clockIdentity,
				&msg->data.delay_resp.requestingPortIdentity.
				clockIdentity,
				sizeof(struct ptp_clock_identity)))
			handle_delay_resp(ptp, msg, corr64);
		break;
	case PDELAY_RESP_MSG:
		if (!memcmp(&ptp->clockIdentity,
				&msg->data.pdelay_resp.requestingPortIdentity.
				clockIdentity,
				sizeof(struct ptp_clock_identity)))
			handle_pdelay_resp(ptp, msg, corr64);
		break;
	case PDELAY_RESP_FOLLOW_UP_MSG:
		if (!memcmp(&ptp->clockIdentity,
				&msg->data.pdelay_resp_follow_up.
				requestingPortIdentity.clockIdentity,
				sizeof(struct ptp_clock_identity)))
			handle_pdelay_resp_follow_up(ptp, msg, corr64);
		break;
	case DELAY_REQ_MSG:
		break;
	case PDELAY_REQ_MSG:
		if (ptp->mode & PTP_TC_P2P)
			handle_delay_req(ptp, msg, corr64);
		break;
	}
}  /* proc_ptp_rx */

static int proc_ptp_tx(struct ptp_info *ptp, struct ptp_msg *msg)
{
#ifdef VERIFY_PDELAY_RESP_DIFF
	u32 nsec;
	u8 sec;
#endif
	u16 port = 0;
	struct ptp_hw_ts *htx = NULL;
	struct ptp_tx_ts *tx = NULL;

	switch (msg->hdr.messageType) {
	case SYNC_MSG:
		htx = ptp->hw_sync;
		tx = ptp->tx_sync;
		break;
	case DELAY_REQ_MSG:
		htx = ptp->hw_dreq;
		tx = ptp->tx_dreq;
		break;
	case PDELAY_REQ_MSG:
		htx = ptp->hw_dreq;
		tx = ptp->tx_dreq;
		break;
	case PDELAY_RESP_MSG:
		htx = ptp->hw_resp;
		tx = ptp->tx_resp;
		port = msg->hdr.reserved2 - 1;
		break;
	case DELAY_RESP_MSG:
		break;
	}
	switch (msg->hdr.messageType) {
	case DELAY_REQ_MSG:
		break;
	case PDELAY_REQ_MSG:
		break;
	case PDELAY_RESP_MSG:
#ifdef VERIFY_PDELAY_RESP_DIFF
		if (!(msg->hdr.flagField.flag.twoStepFlag)) {
			nsec = htonl(msg->hdr.reserved3);
			nsec = timestamp_val(nsec, &sec);
		} else {
			nsec = ntohl(msg->data.pdelay_resp.
				requestReceiptTimestamp.nsec);
			sec = ntohl(msg->data.pdelay_resp.
				requestReceiptTimestamp.sec_lo) & 3;
		}
		ptp->pdelay_t2[port].sec = sec;
		ptp->pdelay_t2[port].nsec = nsec;
#endif
		break;
	case PDELAY_RESP_FOLLOW_UP_MSG:
		break;
	}
	return true;
}  /* proc_ptp_tx */
#endif

static struct ptp_msg *parse_ptp_msg(struct net_device *dev,
	struct sk_buff *skb, struct ptp_id *id, int rx)
{
	struct dev_priv *priv = netdev_priv(dev);
	struct dev_info *hw_priv = priv->adapter;
	struct ptp_info *ptp = &hw_priv->ptp_hw;
	struct ethhdr *eth = (struct ethhdr *) skb->data;
	struct vlan_ethhdr *vlan = (struct vlan_ethhdr *) skb->data;
	struct iphdr *iph = NULL;
	struct ipv6hdr *ip6h = NULL;
	struct udphdr *udp;
	struct udphdr udph;
	struct ptp_msg *msg;
	int len;
	int ipv6;

	if (eth->h_proto == htons(0x88F7)) {
		udp = &udph;
		udp->dest = htons(319);
		msg = (struct ptp_msg *)(eth + 1);
		udp->check = 0xffff;
#ifdef VERIFY_PTP_MSG
		dbg_msg("hdr: m=%x i=%04x d:%u=%u p=%x; r1=%x r2=%x r3=%08x\n",
			msg->hdr.messageType, ntohs(msg->hdr.sequenceId),
			msg->hdr.domainNumber, ptp->domain,
			ntohs(msg->hdr.sourcePortIdentity.port),
			msg->hdr.reserved1, msg->hdr.reserved2,
			ntohl(msg->hdr.reserved3));
#endif
		goto parse_ptp_msg_hdr;
	}

	if (eth->h_proto == htons(ETH_P_8021Q)) {
		ipv6 = vlan->h_vlan_encapsulated_proto == htons(ETH_P_IPV6);
		if (vlan->h_vlan_encapsulated_proto != htons(ETH_P_IP) &&
				!ipv6)
			goto parse_ptp_msg_done;
		ip6h = (struct ipv6hdr *)(vlan + 1);
		iph = (struct iphdr *)(vlan + 1);
	} else {
		ipv6 = eth->h_proto == htons(ETH_P_IPV6);
		if (eth->h_proto != htons(ETH_P_IP) && !ipv6)
			goto parse_ptp_msg_done;
		ip6h = (struct ipv6hdr *)(eth + 1);
		iph = (struct iphdr *)(eth + 1);
	}

	if (ipv6) {
		if (ip6h->nexthdr != IPPROTO_UDP)
			goto parse_ptp_msg_done;

		udp = (struct udphdr *)(ip6h + 1);
	} else {
		if (iph->protocol != IPPROTO_UDP)
			goto parse_ptp_msg_done;

		udp = (struct udphdr *)(iph + 1);
	}

	len = ntohs(udp->len);
	if (udp->dest != htons(319) && udp->dest != htons(320))
		goto parse_ptp_msg_done;

	msg = (struct ptp_msg *)(udp + 1);
#ifdef VERIFY_PTP_MSG
#if 1
	if (rx && udp->check) {
#else
	if (rx) {
#endif
		dbg_msg("hdr: m=%x c=%04x i=%04x d:%u=%u p=%x\n",
			msg->hdr.messageType, ntohs(udp->check),
			ntohs(msg->hdr.sequenceId),
			msg->hdr.domainNumber, ptp->domain,
			ntohs(msg->hdr.sourcePortIdentity.port));
		dbg_msg("r1=%x r2=%x r3=%08x\n",
			msg->hdr.reserved1, msg->hdr.reserved2,
			ntohl(msg->hdr.reserved3));
		if (MANAGEMENT_MSG == msg->hdr.messageType)
			dbg_msg("mg: %d=%04x\n",
				msg->data.management.b.actionField,
				ntohs(msg->data.management.
				tlv.normal[0].managementId));
	}
#endif
#ifdef VERIFY_PTP_ZERO_CSUM
	if (rx) {
		if (udp->check) {
			if ((!(ptp->cfg & PTP_UDP_CHECKSUM) && !ipv6))
				dbg_msg("not zero csum: m=%x i=%04x %08x\n",
					msg->hdr.messageType,
					ntohs(msg->hdr.sequenceId),
					ntohl(msg->hdr.reserved3));
		} else {
			if (((ptp->cfg & PTP_UDP_CHECKSUM) || ipv6)) {
				dbg_msg("no csum: m=%x i=%04x %08x\n",
					msg->hdr.messageType,
					ntohs(msg->hdr.sequenceId),
					ntohl(msg->hdr.reserved3));
#ifdef VERIFY_PTP_CSUM
				udp->check = 0xffff;
#endif
			}
		}
	}
#endif
#ifdef VERIFY_PTP_CSUM
	if (udp->check) {
		u16 check = udp->check;
		u16 csum0;
		u16 csum1;
		u16 csum2;
		int csum_check = false;
		struct pseudo_iphdr p_iph;
		struct pseudo_ip6hdr p_ip6h;

		if (ipv6) {
			p_ip6h.payload_len = ip6h->payload_len;
			p_ip6h.nexthdr = ip6h->nexthdr;
			p_ip6h.hop_limit = 0;
			memcpy(&p_ip6h.saddr, &ip6h->saddr,
				sizeof(struct in6_addr));
			memcpy(&p_ip6h.daddr, &ip6h->daddr,
				sizeof(struct in6_addr));
			if (rx)
				csum_check = true;
		} else {
			p_iph.ttl = 0;
			p_iph.protocol = iph->protocol;
			p_iph.tot_len = udp->len;
			p_iph.saddr = iph->saddr;
			p_iph.daddr = iph->daddr;
		}

		udp->check = 0;
		if (ipv6)
			csum0 = in_cksum((u16 *) &p_ip6h,
				sizeof(struct pseudo_ip6hdr));
		else
			csum0 = in_cksum((u16 *) &p_iph,
				sizeof(struct pseudo_iphdr));
		csum1 = in_cksum((u16 *) udp, len);
		udp->check = ~csum0;
		csum2 = in_cksum((u16 *) udp, len);
		udp->check = check;
		if (rx && (check != csum2 || 0xffff == check)) {
#if 1
			dbg_msg("%d csum: %04x = %04x + %04x = %04x; %x\n", rx,
				check, csum0, csum1, csum2,
				msg->hdr.messageType);
#endif
			if (0 == csum2)
				csum2 = 0xffff;
			udp->check = csum2;
			if (check != 0xffff || !ipv6)
				csum_check = true;
			check = udp->check;
		}
#ifdef VERIFY_PTP_TX_CSUM
		else if (!rx && (ptp->cfg & PTP_UDP_CHECKSUM))
			csum_check = true;
#endif
		if (csum_check) {
			u16 src_port;
			u16 *src;
			u8 reserved1;
			u8 reserved2;
			u32 reserved3;

#ifdef VERIFY_PTP_TX_CSUM
#endif
			udp->check = ~csum0;
			src = &msg->hdr.sourcePortIdentity.port;
			src_port = *src;
			reserved1 = msg->hdr.reserved1;
			reserved2 = msg->hdr.reserved2;
			reserved3 = msg->hdr.reserved3;
			msg->hdr.reserved1 = 0;
#ifdef VERIFY_PTP_TX_CSUM
			switch (msg->hdr.messageType) {
			case SYNC_MSG:
				break;
			case PDELAY_REQ_MSG:
			*src = htons(2);
			csum0 = in_cksum((u16 *) udp, len);
			msg->hdr.reserved2 = 0;
			csum1 = in_cksum((u16 *) udp, len);
			msg->hdr.reserved3 = 0;
			csum2 = in_cksum((u16 *) udp, len);
			msg->hdr.reserved2 = reserved2;
			msg->hdr.reserved3 = reserved3;
				break;
			}
#endif
			*src = htons(1);
			csum0 = in_cksum((u16 *) udp, len);
			msg->hdr.reserved2 = 0;
			csum1 = in_cksum((u16 *) udp, len);
			msg->hdr.reserved3 = 0;
			csum2 = in_cksum((u16 *) udp, len);
			if (rx) {
				int i;
				u8 *data = (u8 *) udp;

				if (!ipv6 || 0 == csum2)
					dbg_msg("original: %04x m=%x i=%04x\n",
						csum2, msg->hdr.messageType,
						ntohs(msg->hdr.sequenceId));
				if (0 == csum2) {
					for (i = 0; i < len; i++) {
						dbg_msg("%02x ", data[i]);
						if ((i % 16) == 15)
							dbg_msg("\n");
					}
					dbg_msg("\n");
				}
			}
#ifdef VERIFY_PTP_TX_CSUM
#endif
			*src = src_port;
			msg->hdr.reserved1 = reserved1;
			msg->hdr.reserved2 = reserved2;
			msg->hdr.reserved3 = reserved3;
			udp->check = check;
		}
	}
#endif

parse_ptp_msg_hdr:
	if (udp->dest == htons(319)) {
		id->msg = msg->hdr.messageType;
		id->seq = msg->hdr.sequenceId;
		memcpy(&id->clock, &msg->hdr.sourcePortIdentity.clockIdentity,
			sizeof(struct ptp_clock_identity));
		id->mac[0] = msg->hdr.sourcePortIdentity.clockIdentity.addr[6];
		id->mac[1] = msg->hdr.sourcePortIdentity.clockIdentity.addr[7];
		id->port = msg->hdr.reserved2;
	}
#ifdef CHECK_DROPPED_MSG
	if (DELAY_REQ_MSG == msg->hdr.messageType) {
		static u16 req_seqids[CLOCK_ENTRIES + 1];
		static u8 last_gm_port;
		int i;
		u16 delay_req_seqid;
		u8 *data = msg->hdr.sourcePortIdentity.clockIdentity.addr;

		delay_req_seqid = ntohs(msg->hdr.sequenceId);
		if (rx) {
			for (i = 0; i < CLOCK_ENTRIES; i++)
				if (!memcmp(data, &ptp->clock_id[i],
					sizeof(struct ptp_clock_identity)))
					break;
		} else {
			i = 0;
			if (msg->hdr.reserved2 != last_gm_port) {
				dbg_msg(" ? gm: %d %d\n", last_gm_port,
					msg->hdr.reserved2);
				last_gm_port = msg->hdr.reserved2;
			}
		}
		if ((u16)(req_seqids[i] + 1) != delay_req_seqid)
			dbg_msg(" %d: m=%x %04x %04x\n", i,
				msg->hdr.messageType,
				delay_req_seqid, req_seqids[i]);
		req_seqids[i] = delay_req_seqid;
	} else if (DELAY_RESP_MSG == msg->hdr.messageType) {
		static u16 resp_seqids[CLOCK_ENTRIES + 1];
		int i;
		u16 delay_resp_seqid;
		u8 *data = msg->data.delay_resp.
			requestingPortIdentity.clockIdentity.addr;

		delay_resp_seqid = ntohs(msg->hdr.sequenceId);
		for (i = 0; i < CLOCK_ENTRIES; i++)
			if (!memcmp(data, &ptp->clock_id[i],
					sizeof(struct ptp_clock_identity)))
				break;
		if ((u16)(resp_seqids[i] + 1) != delay_resp_seqid) {
			dbg_msg(" %d: m=%x %04x %04x\n", i,
				msg->hdr.messageType,
				delay_resp_seqid, resp_seqids[i]);
		}
		resp_seqids[i] = delay_resp_seqid;
	} else if (SYNC_MSG == msg->hdr.messageType ||
			FOLLOW_UP_MSG == msg->hdr.messageType) {
		static u16 sync_seqids[2][MAX_PTP_PORT];
		int i;
		int port;
		u16 seqid;

		seqid = ntohs(msg->hdr.sequenceId);
		if (2 == msg->hdr.reserved2)
			port = 1;
		else
			port = 0;
		if (SYNC_MSG == msg->hdr.messageType)
			i = 0;
		else
			i = 1;
		if ((u16)(sync_seqids[i][port] + 1) != seqid)
			dbg_msg(" m=%x p=%d %04x %04x\n",
				msg->hdr.messageType, msg->hdr.reserved2,
				seqid, sync_seqids[i][port]);
		sync_seqids[i][port] = seqid;
		if ((1 << MAX_PTP_PORT) - 1 == msg->hdr.reserved2)
			sync_seqids[i][1] = seqid;
	} else if (ANNOUNCE_MSG == msg->hdr.messageType) {
		static u16 ann_seqids[MAX_PTP_PORT];
		int port;
		u16 seqid;

		seqid = ntohs(msg->hdr.sequenceId);
		if (2 == msg->hdr.reserved2)
			port = 1;
		else
			port = 0;
		if ((u16)(ann_seqids[port] + 1) != seqid)
			dbg_msg(" m=%x p=%d %04x %04x\n",
				msg->hdr.messageType, msg->hdr.reserved2,
				seqid, ann_seqids[port]);
		ann_seqids[port] = seqid;
	}
#endif
#ifdef PTP_PROCESS
	if (rx)
		proc_ptp_rx(ptp, msg);
	else
		len = proc_ptp_tx(ptp, msg);
#endif
	if (rx && udp->dest != htons(319))
		msg = NULL;
	return msg;

parse_ptp_msg_done:
	return NULL;
}  /* parse_ptp_msg */
#endif

static struct ptp_msg *check_ptp_msg(struct sk_buff *skb)
{
	struct ethhdr *eth = (struct ethhdr *) skb->data;
	struct vlan_ethhdr *vlan = (struct vlan_ethhdr *) skb->data;
	struct iphdr *iph = NULL;
	struct ipv6hdr *ip6h = NULL;
	struct udphdr *udp;
	int ipv6;

	if (eth->h_proto == htons(0x88F7))
		return (struct ptp_msg *)(eth + 1);

	if (eth->h_proto == htons(ETH_P_8021Q)) {
		ipv6 = vlan->h_vlan_encapsulated_proto == htons(ETH_P_IPV6);
		if (vlan->h_vlan_encapsulated_proto != htons(ETH_P_IP) &&
				!ipv6)
			return NULL;
		ip6h = (struct ipv6hdr *)(vlan + 1);
		iph = (struct iphdr *)(vlan + 1);
	} else {
		ipv6 = eth->h_proto == htons(ETH_P_IPV6);
		if (eth->h_proto != htons(ETH_P_IP) && !ipv6)
			return NULL;
		ip6h = (struct ipv6hdr *)(eth + 1);
		iph = (struct iphdr *)(eth + 1);
	}

	if (ipv6) {
		if (ip6h->nexthdr != IPPROTO_UDP)
			return NULL;

		udp = (struct udphdr *)(ip6h + 1);
	} else {
		if (iph->protocol != IPPROTO_UDP)
			return NULL;

		udp = (struct udphdr *)(iph + 1);
	}

	if (udp->dest != htons(319) && udp->dest != htons(320))
		return NULL;

	return (struct ptp_msg *)(udp + 1);
}  /* check_ptp_msg */

static struct ptp_msg *check_ptp_event(struct sk_buff *skb)
{
	struct ptp_msg *msg;

	msg = check_ptp_msg(skb);
	if (!msg)
		return NULL;
	switch (msg->hdr.messageType) {
	case SYNC_MSG:
		break;
	case DELAY_REQ_MSG:
		break;
	case PDELAY_REQ_MSG:
		break;
	case PDELAY_RESP_MSG:
		break;
	default:
		msg = NULL;
		break;
	}
	return msg;
}

static void update_ptp_msg(struct sk_buff *skb, u8 port)
{
	struct ethhdr *eth = (struct ethhdr *) skb->data;
	struct vlan_ethhdr *vlan = (struct vlan_ethhdr *) skb->data;
	struct iphdr *iph;
	struct ipv6hdr *ip6h;
	struct ptp_msg *msg;
	struct udphdr *udp = NULL;
	int ipv6 = 0;

	do {
		if (eth->h_proto == htons(0x88F7)) {
			msg = (struct ptp_msg *)(eth + 1);
			break;
		}

		if (eth->h_proto == htons(ETH_P_8021Q)) {
			ipv6 = vlan->h_vlan_encapsulated_proto ==
				htons(ETH_P_IPV6);
			if (vlan->h_vlan_encapsulated_proto != htons(ETH_P_IP)
					&& !ipv6)
				return;
			ip6h = (struct ipv6hdr *)(vlan + 1);
			iph = (struct iphdr *)(vlan + 1);
		} else {
			ipv6 = eth->h_proto == htons(ETH_P_IPV6);
			if (eth->h_proto != htons(ETH_P_IP) && !ipv6)
				return;
			ip6h = (struct ipv6hdr *)(eth + 1);
			iph = (struct iphdr *)(eth + 1);
		}

		if (ipv6) {
			if (ip6h->nexthdr != IPPROTO_UDP)
				return;

			udp = (struct udphdr *)(ip6h + 1);
		} else {
			if (iph->protocol != IPPROTO_UDP)
				return;

			udp = (struct udphdr *)(iph + 1);
		}

		if (udp->dest != htons(319) && udp->dest != htons(320))
			return;
		msg = (struct ptp_msg *)(udp + 1);
	} while (0);
	if (msg->hdr.reserved2 != 3 && msg->hdr.reserved2 != port) {
		u8 data = msg->hdr.reserved2;
		u16 check;

		/* Zero checksum in IPv4. */
		if (udp && !ipv6 && !udp->check)
			udp = NULL;
		if (udp) {
			check = ntohs(udp->check);
			check += data;
		}
		msg->hdr.reserved2 = port;
		if (udp) {
			check -= port;
			if (!check)
				check = -1;
			udp->check = htons(check);
		}
	}
}  /* update_ptp_msg */

static void get_rx_tstamp(struct ptp_info *ptp, struct sk_buff *skb)
{
	struct ptp_msg *msg;
	struct ptp_ts ts;
	u64 ns;
	struct skb_shared_hwtstamps *shhwtstamps = skb_hwtstamps(skb);

	msg = check_ptp_event(skb);
	if (!msg)
		return;
	ts.timestamp = ntohl(msg->hdr.reserved3);
	update_ts(&ts, ptp->cur_time.sec);
	if (shhwtstamps) {
		ns = (u64) ts.t.sec * NANOSEC_IN_SEC + ts.t.nsec;
		memset(shhwtstamps, 0, sizeof(*shhwtstamps));
		shhwtstamps->hwtstamp = ns_to_ktime(ns);
	}
}  /* get_rx_tstamp */

static void get_tx_tstamp(struct ptp_info *ptp, struct sk_buff *skb)
{
	int cnt;
	int p;
	struct ptp_msg *msg;
	u8 port;
	struct ptp_tx_ts *tx;
	struct sk_buff *orig_skb = skb;

	msg = check_ptp_event(skb);
	if (!msg)
		return;
	port = msg->hdr.reserved2;
	if (!port)
		port = (1 << MAX_PTP_PORT) - 1;
	cnt = 0;
	if (SYNC_MSG == msg->hdr.messageType)
		tx = ptp->tx_sync;
	else if (PDELAY_RESP_MSG == msg->hdr.messageType)
		tx = ptp->tx_resp;
	else
		tx = ptp->tx_dreq;
	for (p = 0; p < MAX_PTP_PORT; p++, tx++) {
		if (!(port & (1 << p)))
			continue;
		if (tx->skb) {
			if (skb_shinfo(tx->skb)->tx_flags & SKBTX_HW_TSTAMP)
				skb_shinfo(tx->skb)->tx_flags &=
					~SKBTX_IN_PROGRESS;
			else {
				dev_kfree_skb_irq(tx->skb);
				tx->skb = NULL;
			}
		}

		/* Need to create socket buffer for more than 1 port. */
		if (cnt++) {
			skb = skb_copy(orig_skb, GFP_ATOMIC);
			if (!skb)
				break;
			skb->sk = orig_skb->sk;
			msg = check_ptp_event(skb);
		}
		tx->skb = skb;
		tx->msg = msg;
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	}
}  /* get_tx_tstamp */

static int ptp_hwtstamp_ioctl(struct ptp_info *ptp, struct ifreq *ifr)
{
	struct hwtstamp_config config;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		ptp->tx_en &= ~1;
		break;
	case HWTSTAMP_TX_ON:
		ptp->tx_en |= 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		ptp->rx_en &= ~1;
		break;
	default:
		ptp->rx_en |= 1;
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	}

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

static void proc_ptp_get_cfg(struct ptp_info *ptp, u8 *data)
{
	struct ptp_cfg_options *cmd = (struct ptp_cfg_options *) data;

	ptp_acquire(ptp);
	ptp->mode = ptp_read(ptp->ptpdev, ADDR_16, PTP_MSG_CONF1);
	ptp->cfg = ptp_read(ptp->ptpdev, ADDR_16, PTP_MSG_CONF2);
	ptp->domain = ptp_read(ptp->ptpdev, ADDR_16, PTP_DOMAIN_VERSION) &
		PTP_DOMAIN_MASK;
	ptp_release(ptp);
	if (ptp->mode != ptp->def_mode) {
		dbg_msg("mode mismatched: %04x %04x; %04x %04x\n",
			ptp->mode, ptp->def_mode, ptp->cfg, ptp->def_cfg);
		ptp->mode = ptp->def_mode;
		ptp->cfg = ptp->def_cfg;
		ptp_start(ptp, false);
	}
	cmd->two_step = (ptp->mode & PTP_1STEP) ? 0 : 1;
	cmd->master = (ptp->mode & PTP_MASTER) ? 1 : 0;
	cmd->p2p = (ptp->mode & PTP_TC_P2P) ? 1 : 0;
	cmd->as = (ptp->mode & PTP_FORWARD_TO_PORT3) ? 1 : 0;
	cmd->unicast = (ptp->cfg & PTP_UNICAST_ENABLE) ? 1 : 0;
	cmd->alternate = (ptp->cfg & PTP_ALTERNATE_MASTER) ? 1 : 0;
	cmd->domain_check = (ptp->cfg & PTP_DOMAIN_CHECK) ? 1 : 0;
	cmd->udp_csum = (ptp->cfg & PTP_UDP_CHECKSUM) ? 1 : 0;
	cmd->delay_assoc = (ptp->cfg & PTP_DELAY_CHECK) ? 1 : 0;
	cmd->pdelay_assoc = (ptp->cfg & PTP_PDELAY_CHECK) ? 1 : 0;
	cmd->sync_assoc = (ptp->cfg & PTP_SYNC_CHECK) ? 1 : 0;
	cmd->drop_sync = (ptp->cfg & PTP_DROP_SYNC_DELAY_REQ) ? 1 : 0;
	cmd->priority = (ptp->cfg & PTP_ALL_HIGH_PRIORITY) ? 1 : 0;
	cmd->reserved = 0;
	cmd->domain = ptp->domain;
	cmd->access_delay = ptp->get_delay;
}  /* proc_ptp_get_cfg */

static int proc_ptp_set_cfg(struct ptp_info *ptp, u8 *data)
{
	struct ptp_cfg_options *cmd = (struct ptp_cfg_options *) data;
	u16 cfg;
	u16 mode;
	u8 domain;

	mode = ptp->mode;
	cfg = ptp->cfg;
	domain = ptp->domain;
	if (cmd->domain_set) {
		domain = cmd->domain;
	} else {
		if (cmd->two_step_set) {
			if (cmd->two_step)
				ptp->mode &= ~PTP_1STEP;
			else
				ptp->mode |= PTP_1STEP;
		}
		if (cmd->master_set) {
			if (cmd->master)
				ptp->mode |= PTP_MASTER;
			else
				ptp->mode &= ~PTP_MASTER;
		}
		if (cmd->p2p_set) {
			if (cmd->p2p)
				ptp->mode |= PTP_TC_P2P;
			else
				ptp->mode &= ~PTP_TC_P2P;
		}
		if (cmd->as_set) {
			if (cmd->as)
				ptp->mode |= PTP_FORWARD_TO_PORT3;
			else
				ptp->mode &= ~PTP_FORWARD_TO_PORT3;
		}
		if (cmd->unicast_set) {
			if (cmd->unicast)
				ptp->cfg |= PTP_UNICAST_ENABLE;
			else
				ptp->cfg &= ~PTP_UNICAST_ENABLE;
		}
		if (cmd->alternate_set) {
			if (cmd->alternate)
				ptp->cfg |= PTP_ALTERNATE_MASTER;
			else
				ptp->cfg &= ~PTP_ALTERNATE_MASTER;
		}
		if (cmd->domain_check_set) {
			if (cmd->domain_check)
				ptp->cfg |= PTP_DOMAIN_CHECK;
			else
				ptp->cfg &= ~PTP_DOMAIN_CHECK;
		}
		if (cmd->udp_csum_set) {
			if (cmd->udp_csum)
				ptp->cfg |= PTP_UDP_CHECKSUM;
			else
				ptp->cfg &= ~PTP_UDP_CHECKSUM;
		}
		if (cmd->delay_assoc_set) {
			if (cmd->delay_assoc)
				ptp->cfg |= PTP_DELAY_CHECK;
			else
				ptp->cfg &= ~PTP_DELAY_CHECK;
		}
		if (cmd->pdelay_assoc_set) {
			if (cmd->pdelay_assoc)
				ptp->cfg |= PTP_PDELAY_CHECK;
			else
				ptp->cfg &= ~PTP_PDELAY_CHECK;
		}
		if (cmd->sync_assoc_set) {
			if (cmd->sync_assoc)
				ptp->cfg |= PTP_SYNC_CHECK;
			else
				ptp->cfg &= ~PTP_SYNC_CHECK;
		}
		if (cmd->drop_sync_set) {
			if (cmd->drop_sync)
				ptp->cfg |= PTP_DROP_SYNC_DELAY_REQ;
			else
				ptp->cfg &= ~PTP_DROP_SYNC_DELAY_REQ;
		}
		if (cmd->priority_set) {
			if (cmd->priority)
				ptp->cfg |= PTP_ALL_HIGH_PRIORITY;
			else
				ptp->cfg &= ~PTP_ALL_HIGH_PRIORITY;
		}
	}
	ptp_acquire(ptp);
	if (mode != ptp->mode) {
		dbg_msg("mode: %x %x\n", mode, ptp->mode);
		ptp_write(ptp->ptpdev, ADDR_16, PTP_MSG_CONF1, ptp->mode);
		ptp->def_mode = ptp->mode;
	}
	if (cfg != ptp->cfg) {
		dbg_msg("cfg: %x %x\n", cfg, ptp->cfg);
		ptp_write(ptp->ptpdev, ADDR_16, PTP_MSG_CONF2, ptp->cfg);
	}
	if (domain != ptp->domain) {
		ptp->domain = domain;
		set_ptp_domain(ptp->ptpdev, ptp->domain);
	}
	ptp_release(ptp);
	return 0;
}  /* proc_ptp_set_cfg */

static void cancel_rx_unit(struct ptp_info *ptp, int tsi)
{
	int first;
	int last;
	int tsi_bit;
	struct ptp_event *events;

	ptp_acquire(ptp);
	first = tsi;
	events = &ptp->events[tsi];
	if (events->last) {
		first = events->first;
		last = events->last;
	} else
		last = first + 1;
	tsi = first;
	ptp->events[tsi].timeout = 0;
	do {
		if (tsi >= MAX_TIMESTAMP_UNIT)
			tsi = 0;
		events = &ptp->events[tsi];
		events->first = 0;
		events->last = 0;
		tsi_bit = 1 << tsi;
		if (ptp->tsi_used & tsi_bit) {
			if (events->num < events->max) {
				ptp_read_event(ptp, tsi);
				ptp->ts_status = 0;
			}
			ptp_rx_off(ptp, tsi);
			ptp->tsi_intr &= ~tsi_bit;
			ptp->tsi_used &= ~tsi_bit;
			ptp->tsi_dev[tsi] = NULL;
		}
		++tsi;
	} while (tsi != last);
	ptp_release(ptp);
}  /* cancel_rx_unit */

static int check_expired_rx_unit(struct ptp_info *ptp, int tsi)
{
	int first;
	int last;
	u32 expired;
	struct ptp_event *events;
	struct ptp_time diff;
	struct ptp_utime t;

	events = &ptp->events[tsi];
	first = tsi;
	if (events->last) {
		first = events->first;
		last = events->last;
	} else
		last = first + 1;
	events = &ptp->events[first];
	if (events->num && events->timeout) {
		ptp_acquire(ptp);
		get_ptp_time(ptp->ptpdev, &t);
		ptp_release(ptp);
		calc_udiff(events->t, &t, &diff);
		if (diff.sec >= 0 && diff.nsec >= 0) {
			expired = diff.sec * 1000 + diff.nsec / 1000000;
			expired = expired * HZ / 1000;
			if (expired > events->timeout) {
				cancel_rx_unit(ptp, first);
				return 1;
			}
		}
	}
	return 0;
}  /* check_expired_rx_unit */

static int proc_dev_rx_event(struct ptp_dev_info *info, u8 *data)
{
	struct ptp_info *ptp = info->ptp;
	struct ptp_tsi_options *cmd = (struct ptp_tsi_options *) data;
	u8 event;
	int first;
	int i;
	int intr;
	int tsi;
	int avail;
	int total;
	int last;
	int tsi_bit;
	struct ptp_event *events;

	tsi = cmd->tsi;
	total = cmd->total;
	if (!total)
		total = 1;
	first = tsi;

	/* Cancel operation. */
	if ((cmd->flags & PTP_CMD_CANCEL_OPER)) {
		if (tsi >= MAX_TIMESTAMP_UNIT)
			return DEV_IOC_UNIT_UNAVAILABLE;
		cancel_rx_unit(ptp, tsi);
		goto proc_ptp_rx_cascade_event_done;
	}
	if (tsi >= MAX_TIMESTAMP_UNIT) {
		first = 0;
		do {
			for (tsi = first; tsi < MAX_TIMESTAMP_UNIT; tsi++)
				if (!(ptp->tsi_used & (1 << tsi)) &&
						!ptp->events[tsi].last)
					break;
			if (tsi >= MAX_TIMESTAMP_UNIT)
				return DEV_IOC_UNIT_UNAVAILABLE;
			first = tsi;
			avail = 1;
			for (i = 1; i < total; i++)
				if (!(ptp->tsi_used & (1 << tsi)) &&
						!ptp->events[tsi].last) {
					++avail;
					++tsi;
					if (tsi >= MAX_TIMESTAMP_UNIT)
						tsi = 0;
				} else {
					++first;
					break;
				}
		} while (avail < total);
	} else {
		for (i = 0; i < total; i++) {
			if (ptp->tsi_used & (1 << tsi) ||
					ptp->events[tsi].last)
				if (!check_expired_rx_unit(ptp, tsi))
					return DEV_IOC_UNIT_USED;
			++tsi;
			if (tsi >= MAX_TIMESTAMP_UNIT)
				tsi = 0;
		}
	}
	if (cmd->gpi >= MAX_GPIO)
		return -EINVAL;
	if (0 == cmd->event)
		event = TS_DETECT_FALL;
	else if (1 == cmd->event)
		event = TS_DETECT_RISE;
	else {
		event = TS_DETECT_RISE | TS_DETECT_FALL;
		cmd->event = 2;
	}
	tsi = first;
	last = first + total;
	if (last > MAX_TIMESTAMP_UNIT)
		last -= MAX_TIMESTAMP_UNIT;
	intr = cmd->flags & PTP_CMD_INTR_OPER;
	ptp_acquire(ptp);
	for (i = 0; i < total; i++) {
		tsi_bit = 1 << tsi;
		ptp->tsi_used |= tsi_bit;
		if (intr & !(cmd->flags & PTP_CMD_SILENT_OPER)) {
			ptp->tsi_intr |= tsi_bit;
			ptp->tsi_dev[tsi] = info;
		}
		events = &ptp->events[tsi];
		events->num = 0;
		events->event = cmd->event;
		events->edge = 0;
		events->expired = 0;
		if (total > 1) {
			events->first = first;
			events->last = last;
		}
		++tsi;
		if (tsi >= MAX_TIMESTAMP_UNIT)
			tsi = 0;
	}
	tsi = first;
	ptp->events[tsi].timeout = cmd->timeout * HZ / 1000;

	/* Zero timeout means repeatable. */
	if (!ptp->events[tsi].timeout && cmd->timeout)
		ptp->events[tsi].timeout = 1;
	if (total > 1)
		ptp_rx_cascade_event(ptp, tsi, total, cmd->gpi, event, intr);
	else
		ptp_rx_event(ptp, tsi, cmd->gpi, event, intr);
	ptp_release(ptp);

proc_ptp_rx_cascade_event_done:
	*data = tsi;
	return 0;
}  /* proc_dev_rx_event */

static int find_avail_tx_unit(struct ptp_info *ptp, int total, int *unit)
{
	int avail;
	int first;
	int i;
	int tso;

	first = 0;
	do {
		for (tso = first; tso < MAX_TRIG_UNIT; tso++)
			if (!(ptp->tso_used & (1 << tso)))
				break;
		if (tso >= MAX_TRIG_UNIT)
			return DEV_IOC_UNIT_UNAVAILABLE;
		first = tso;
		avail = 1;
		for (i = 1; i < total; i++)
			if (!(ptp->tso_used & (1 << tso))) {
				++avail;
				++tso;
				if (tso >= MAX_TRIG_UNIT)
					return DEV_IOC_UNIT_UNAVAILABLE;
			} else {
				++first;
				break;
			}
	} while (avail < total);
	*unit = first;
	return 0;
}  /* find_avail_tx_unit */

static int proc_dev_tx_event(struct ptp_dev_info *info, u8 *data)
{
	struct ptp_info *ptp = info->ptp;
	struct ptp_tso_options *cmd = (struct ptp_tso_options *) data;
	int gpo;
	int intr;
	int tso;
	int tso_bit;
	struct ptp_utime t;
	u16 active;
	u16 status;
	void *ptpdev = ptp->ptpdev;
	int err = 0;

	gpo = cmd->gpo;
	if (gpo >= MAX_GPIO)
		return -EINVAL;
	if (cmd->event > TRIG_REG_OUTPUT)
		return -EINVAL;
	tso = cmd->tso;
	tso_bit = 1 << tso;

	/* Cancel operation. */
	if ((cmd->flags & PTP_CMD_CANCEL_OPER)) {
		if (tso >= MAX_TRIG_UNIT)
			return DEV_IOC_UNIT_UNAVAILABLE;
		ptp_acquire(ptp);

		/* Reset the tso. */
		ptp->cascade_tx |= tso_bit;
		ptp_tso_off(ptp, tso, tso_bit);
		ptp_release(ptp);
		goto proc_dev_tx_event_done;
	}
	if (ptp->cascade && (tso < ptp->cascade_gpo[gpo].first ||
			tso >= ptp->cascade_gpo[gpo].first +
			ptp->cascade_gpo[gpo].total))
		return DEV_IOC_UNIT_UNAVAILABLE;

	/* Find available unit for use. */
	if (tso >= MAX_TRIG_UNIT) {
		int rc = find_avail_tx_unit(ptp, 1, &tso);

		if (rc)
			return rc;
	} else if (!ptp->cascade && (ptp->tso_used & tso_bit)) {

		/* See whether previous operation is completed. */
		active = ptp_read(ptpdev, ADDR_16, TRIG_ACTIVE);
		if (active & tso_bit) {
			status = ptp_read(ptpdev, ADDR_16, TRIG_ERROR);
			if (!(status & tso_bit))
				return DEV_IOC_UNIT_USED;
			dbg_msg("trig err: %d\n", tso);
		}
		if (!(active & tso_bit)) {
			status = ptp_read(ptpdev, ADDR_16, TRIG_DONE);
			if (!(status & tso_bit)) {
				/* Reset the unit. */
				ptp->cascade_tx |= tso_bit;
				dbg_msg(" !? trig done: %d\n", tso);
			}
		}
		ptp_tso_off(ptp, tso, tso_bit);
	}
	ptp_acquire(ptp);
	if (!ptp->cascade && (cmd->flags & PTP_CMD_REL_TIME) &&
			cmd->sec < 100) {
		get_ptp_time(ptpdev, &t);
		if (0 == cmd->sec) {
			cmd->nsec += t.nsec;
			cmd->nsec += 500;
			cmd->nsec /= 1000;
			cmd->nsec *= 1000;
			if (cmd->nsec >= NANOSEC_IN_SEC) {
				cmd->nsec -= NANOSEC_IN_SEC;
				cmd->sec++;
			}
		}
		cmd->sec += t.sec;
	}
	intr = cmd->flags & PTP_CMD_INTR_OPER;
	if (intr & !(cmd->flags & PTP_CMD_SILENT_OPER)) {
		ptp->tso_intr |= tso_bit;
		ptp->tso_dev[tso] = info;
	}
	ptp->tso_used |= tso_bit;
	ptp_tx_event(ptp, tso, cmd->gpo, cmd->event, cmd->pulse, cmd->cycle,
		cmd->cnt, cmd->sec, cmd->nsec, cmd->iterate, intr,
		!(cmd->flags & PTP_CMD_ON_TIME),
		(cmd->flags & PTP_CMD_CLK_OPT));
	if (cmd->flags & PTP_CMD_ON_TIME) {
		status = ptp_read(ptpdev, ADDR_16, TRIG_ERROR);
		if (status & tso_bit)
			err = DEV_IOC_UNIT_ERROR;
	}
	ptp_release(ptp);

proc_dev_tx_event_done:
	*data = tso;
	return err;
}  /* proc_dev_tx_event */

static int proc_ptp_tx_cascade_init(struct ptp_info *ptp, u8 *data)
{
	struct ptp_tso_options *cmd = (struct ptp_tso_options *) data;
	int first;
	int gpo;
	int i;
	int tso;
	int total;
	u16 status;

	tso = cmd->tso;
	gpo = cmd->gpo;
	total = cmd->total;
	if (!total)
		return -EINVAL;
	if (gpo >= MAX_GPIO)
		return -EINVAL;
	first = tso;

	/* Cancel operation. */
	if ((cmd->flags & PTP_CMD_CANCEL_OPER)) {
		if (tso >= MAX_TRIG_UNIT)
			return DEV_IOC_UNIT_UNAVAILABLE;
		if (first != ptp->cascade_gpo[gpo].first ||
				total != ptp->cascade_gpo[gpo].total) {
			first = ptp->cascade_gpo[gpo].first;
			total = ptp->cascade_gpo[gpo].total;
		}

		/* Reset the last unit in case it is used to raise the level. */
		first = first + total - 1;
		if (ptp->outputs[first].level) {
			ptp->cascade_tx |= (1 << first);
			ptp->tso_used |= (1 << first);
		}
		ptp_acquire(ptp);
		for (i = 0; i < total; i++, tso++) {
			if (ptp->tso_used & (1 << tso))
				ptp_tso_off(ptp, tso, (1 << tso));
		}
		tso = total;
		ptp->cascade = false;
		ptp_release(ptp);
		goto proc_ptp_tx_cascade_init_done;
	}

	if (ptp->cascade)
		return DEV_IOC_UNIT_UNAVAILABLE;

	/* Find available units for use. */
	if (tso >= MAX_TRIG_UNIT) {
		int rc = find_avail_tx_unit(ptp, total, &first);

		if (rc)
			return rc;
	} else {
		for (i = 0; i < total; i++) {
			if (ptp->tso_used & (1 << tso))
				return DEV_IOC_UNIT_USED;
			++tso;
			if (tso >= MAX_TRIG_UNIT)
				return DEV_IOC_UNIT_UNAVAILABLE;
		}
	}

	if ((cmd->flags & PTP_CMD_CASCADE_RESET_OPER))
		goto proc_ptp_tx_cascade_init_set;

	/* Last operation was not in cascade mode. */
	if (!ptp->cascade_gpo[gpo].total)
		goto proc_ptp_tx_cascade_init_set;

	/* previous last unit. */
	i = ptp->cascade_gpo[gpo].first + ptp->cascade_gpo[gpo].total - 1;

	/* current last unit. */
	tso = first + total - 1;

	/* Last operation not ended high. */
	if (tso == i || !ptp->outputs[i].level)
		goto proc_ptp_tx_cascade_init_set;

	status = ptp_read(ptp->ptpdev, ADDR_16, GPIO_MONITOR);

	/* Current level is high. */
	if (status & (1 << gpo)) {
		ptp_acquire(ptp);

		/* Set unit to hold the level high. */
		ptp_tx_event(ptp, tso, gpo, TRIG_POS_EDGE, 0, 0, 1, 0, 1, 0,
			PTP_CMD_INTR_OPER, 1, 0);

		/* Release the signal from the previous last unit. */
		ptp_gpo_reset(ptp, ptp->outputs[i].gpo, (1 << i));
		ptp_release(ptp);
	}

proc_ptp_tx_cascade_init_set:
	ptp->cascade = true;
	ptp->cascade_gpo[gpo].first = first;
	ptp->cascade_gpo[gpo].total = total;
	tso = first;

proc_ptp_tx_cascade_init_done:
	*data = tso;
	return 0;
}  /* proc_ptp_tx_cascade_init */

static int proc_ptp_tx_cascade(struct ptp_info *ptp, u8 *data)
{
	struct ptp_tso_options *cmd = (struct ptp_tso_options *) data;
	int first;
	int gpo;
	int tso;
	int total;
	struct ptp_utime t;
	void *ptpdev = ptp->ptpdev;

	gpo = cmd->gpo;
	if (gpo >= MAX_GPIO)
		return -EINVAL;
	tso = cmd->tso;
	total = cmd->total;
	first = tso;
	if (!ptp->cascade || tso != ptp->cascade_gpo[gpo].first ||
			total != ptp->cascade_gpo[gpo].total)
		return DEV_IOC_UNIT_UNAVAILABLE;

	/* Cancel operation. */
	if ((cmd->flags & PTP_CMD_CANCEL_OPER)) {
		proc_ptp_tx_cascade_init(ptp, data);
		goto proc_ptp_tx_cascade_done;
	}
	ptp_acquire(ptp);
	if ((cmd->flags & PTP_CMD_REL_TIME) && cmd->sec < 100) {
		get_ptp_time(ptpdev, &t);
		cmd->sec += t.sec;
	}
	total = ptp_tx_cascade(ptp, tso, total, cmd->cnt, cmd->sec, cmd->nsec,
		cmd->flags & PTP_CMD_INTR_OPER);
	if (!total)
		ptp->cascade = false;
	ptp_release(ptp);

proc_ptp_tx_cascade_done:
	*data = tso;
	return 0;
}  /* proc_ptp_tx_cascade */

static void proc_tsm_get_gps(struct ptp_info *ptp, u8 *data)
{
	struct tsm_get_gps *get = (struct tsm_get_gps *) data;

	if (!ptp->gps_dev)
		return;

	get->cmd |= TSM_CMD_GET_TIME_RESP;
	get->seqid = htons(ptp->gps_seqid);
	get->sec = htonl(ptp->gps_time.sec);
	get->nsec = htonl(ptp->gps_time.nsec);
	ptp_setup_udp_msg(ptp->gps_dev, data, sizeof(struct tsm_get_gps),
		NULL, NULL);
	ptp->gps_dev = NULL;
}  /* proc_tsm_get_gps */

static int proc_dev_get_event(struct ptp_dev_info *info, u8 *data)
{
	struct ptp_info *ptp = info->ptp;
	int len;
	struct ptp_tsi_info *in = (struct ptp_tsi_info *) data;
	u8 buf[sizeof(struct ptp_utime) * MAX_TIMESTAMP_EVENT_UNIT +
		sizeof(struct ptp_tsi_info)];
	struct ptp_tsi_info *out = (struct ptp_tsi_info *) buf;

	if (in->unit >= MAX_TIMESTAMP_UNIT)
		return -EINVAL;
	if (!ptp->events[in->unit].num)
		return DEV_IOC_UNIT_UNAVAILABLE;
	out->cmd = in->cmd | PTP_CMD_RESP;
	out->unit = in->unit;
	out->event = ptp->events[in->unit].event;
	out->num = ptp->events[in->unit].num;
	out->edge = ptp->events[in->unit].edge;
	len = sizeof(struct ptp_utime) * out->num;
	memcpy(out->t, ptp->events[in->unit].t, len);
	len += sizeof(struct ptp_tsi_info);
	ptp_setup_udp_msg(info, buf, len, NULL, NULL);
	return 0;
}  /* proc_dev_get_event */

static int proc_ptp_get_event(struct ptp_info *ptp, u8 *data)
{
	struct ptp_tsi_info *in = (struct ptp_tsi_info *) data;
	int ret = -1;

	if (ptp->tsi_dev[in->unit])
		ret = proc_dev_get_event(ptp->tsi_dev[in->unit], data);
	return ret;
}  /* proc_ptp_get_event */

static int proc_ptp_get_trig(struct ptp_info *ptp, u8 *data, u16 done,
	u16 error)
{
	int len;
	struct ptp_tsi_info *in = (struct ptp_tsi_info *) data;
	u8 buf[sizeof(struct ptp_utime) + sizeof(struct ptp_tsi_info)];
	struct ptp_tsi_info *out = (struct ptp_tsi_info *) buf;
	struct ptp_output *cur;
	int tso = in->unit;
	int tso_bit = (1 << tso);

	out->cmd = in->cmd | PTP_CMD_RESP;
	out->unit = in->unit;
	cur = &ptp->outputs[tso];
	if (error & tso_bit)
		out->event = 1;
	else if (!(done & tso_bit))
		out->event = 2;
	else
		out->event = 0;
	out->num = 1;
	len = sizeof(struct ptp_utime) * out->num;
	memcpy(out->t, &cur->trig, len);
	len += sizeof(struct ptp_tsi_info);
	if (ptp->tso_dev[tso]) {
		ptp_setup_udp_msg(ptp->tso_dev[tso], buf, len, NULL, NULL);
		return 0;
	}
	return -1;
}  /* proc_ptp_get_trig */

static int proc_dev_poll_event(struct ptp_dev_info *info, u8 *data)
{
	struct ptp_info *ptp = info->ptp;
	struct ptp_tsi_info *in = (struct ptp_tsi_info *) data;

	if (in->unit >= MAX_TIMESTAMP_UNIT)
		return -EINVAL;
	if (!ptp_poll_event(ptp, in->unit))
		return DEV_IOC_UNIT_UNAVAILABLE;
	return proc_dev_get_event(info, data);
}  /* proc_dev_poll_event */

static int proc_ptp_get_output(struct ptp_info *ptp, u8 *data)
{
	int *output = (int *) data;
	struct ptp_tso_options *in = (struct ptp_tso_options *) data;

	if (in->gpo >= MAX_GPIO)
		return -EINVAL;
	*output = ptp->cascade_gpo[in->gpo].tso;
	return 0;
}  /* proc_ptp_get_output */

static void proc_ptp_get_clk(struct ptp_info *ptp, u8 *data)
{
	struct ptp_utime t;
	struct ptp_clk_options *cmd = (struct ptp_clk_options *) data;

	ptp_acquire(ptp);
	get_ptp_time(ptp->ptpdev, &t);
	ptp_release(ptp);
	cmd->sec = t.sec;
	cmd->nsec = t.nsec;
}  /* proc_ptp_get_clk */

static int proc_ptp_set_clk(struct ptp_info *ptp, u8 *data)
{
	struct ptp_utime t;
	struct ptp_clk_options *cmd = (struct ptp_clk_options *) data;
	struct timespec ts;
	struct ptp_utime sys_time;

	t.sec = cmd->sec;
	t.nsec = cmd->nsec;
	ptp_acquire(ptp);
	ts = ktime_to_timespec(ktime_get_real());
	sys_time.sec = ts.tv_sec;
	sys_time.nsec = ts.tv_nsec;
	set_ptp_time(ptp->ptpdev, &t);
	ptp->cur_time = t;
	calc_udiff(&ptp->cur_time, &sys_time, &ptp->time_diff);
	generate_tx_event(ptp, ptp->pps_gpo);
	ptp_release(ptp);
	dbg_msg(" set clk: %x:%09u\n", cmd->sec, cmd->nsec);
	return 0;
}  /* proc_ptp_set_clk */

static int proc_ptp_adj_clk(struct ptp_info *ptp, u8 *data, int adjust)
{
	struct ptp_clk_options *cmd = (struct ptp_clk_options *) data;

	adjust--;
	if (cmd->sec > 1) {
		if (adjust) {
			ptp->adjust_sec = cmd->sec;
			ptp->adjust_offset = cmd->nsec;
		} else {
			ptp->adjust_sec = -cmd->sec;
			ptp->adjust_offset = -cmd->nsec;
		}
		cmd->sec = cmd->nsec = 0;
		ptp->adj_clk.func(&ptp->adj_clk);
	}
	ptp_acquire(ptp);
	if (cmd->nsec || cmd->sec) {
		adjust_ptp_time(ptp->ptpdev, adjust, cmd->sec, cmd->nsec,
			ptp->features & PTP_ADJ_HACK);
		ptp->offset_changed = cmd->nsec;
		if (!adjust)
			ptp->offset_changed = -cmd->nsec;
		generate_tx_event(ptp, ptp->pps_gpo);
		dbg_msg(" adj clk: %d %u:%09u\n", adjust, cmd->sec, cmd->nsec);
	}
	if (cmd->interval) {
		ptp->drift = cmd->drift;
		ptp->drift_set = ptp->drift;
		ptp->adjust = clk_adjust_val(ptp->drift, cmd->interval);
		set_ptp_adjust(ptp->ptpdev, ptp->adjust);
		if (!ptp->ptp_synt) {
			syntonize_clk(ptp->ptpdev);
			ptp->ptp_synt = true;
		}
		dbg_msg(" adj drift: %d\n", cmd->drift);
	}
	ptp_release(ptp);
	return 0;
}  /* proc_ptp_adj_clk */

static int proc_ptp_get_delay(struct ptp_info *ptp, int port, u8 *data)
{
	struct ptp_delay_values *delay = (struct ptp_delay_values *) data;

	if (port >= MAX_PTP_PORT)
		return DEV_IOC_INVALID_CMD;
	ptp_acquire(ptp);
	delay->rx_latency = get_ptp_ingress(ptp->ptpdev, port);
	delay->tx_latency = get_ptp_egress(ptp->ptpdev, port);
	delay->asym_delay = get_ptp_asym(ptp->ptpdev, port);
	ptp_release(ptp);
	return 0;
}  /* proc_ptp_get_delay */

static int proc_ptp_set_delay(struct ptp_info *ptp, int port, u8 *data)
{
	struct ptp_delay_values *delay = (struct ptp_delay_values *) data;

	if (port >= MAX_PTP_PORT)
		return DEV_IOC_INVALID_CMD;
	ptp_acquire(ptp);
	set_ptp_ingress(ptp->ptpdev, port, delay->rx_latency);
	set_ptp_egress(ptp->ptpdev, port, delay->tx_latency);
	set_ptp_asym(ptp->ptpdev, port, delay->asym_delay);
	ptp->rx_latency[port] = delay->rx_latency;
	ptp->tx_latency[port] = delay->tx_latency;
	ptp->asym_delay[port] = delay->asym_delay;
	ptp_release(ptp);
	dbg_msg("set delay: %d = %d %d %d\n", port,
		ptp->rx_latency[port],
		ptp->tx_latency[port],
		ptp->asym_delay[port]);
	return 0;
}  /* proc_ptp_set_delay */

static int proc_ptp_get_peer_delay(struct ptp_info *ptp, int port, u8 *data)
{
	struct ptp_delay_values *delay = (struct ptp_delay_values *) data;

	if (port >= MAX_PTP_PORT)
		return DEV_IOC_INVALID_CMD;
	ptp_acquire(ptp);
	delay->rx_latency = 0;
	delay->tx_latency = 0;
	delay->asym_delay = 0;
	delay->reserved = get_ptp_link(ptp->ptpdev, port);
	ptp_release(ptp);
	return 0;
}  /* proc_ptp_get_peer_delay */

static int proc_ptp_set_peer_delay(struct ptp_info *ptp, int port, u8 *data)
{
	struct ptp_delay_values *delay = (struct ptp_delay_values *) data;

	if (port >= MAX_PTP_PORT)
		return DEV_IOC_INVALID_CMD;
	ptp_acquire(ptp);
	set_ptp_link(ptp->ptpdev, port, delay->reserved);
	ptp->peer_delay[port] = delay->reserved;
	ptp_release(ptp);
	dbg_msg("set delay: %d = %d\n", port,
		ptp->peer_delay[port]);
	return 0;
}  /* proc_ptp_set_peer_delay */

static void ptp_tx_done(struct ptp_info *ptp, int tso)
{
	int first;
	int last;
	int prev;
	int reg;
	u16 data;
	void *ptpdev = ptp->ptpdev;

	reg = TRIGn_CONF_1(tso);
	data = ptp_read(ptpdev, ADDR_16, reg);
	if (data & TRIG_CASCADE_EN) {
		last = tso;
		do {
			--tso;
			reg = TRIGn_CONF_1(tso);
			data = ptp_read(ptpdev, ADDR_16, reg);
			prev = (data & TRIG_CASCADE_UPS_MASK) >> 10;
			if (prev == last)
				break;
		} while (tso > 0);
		first = tso;
		for (tso = last; tso > first; tso--)
			ptp_tso_off(ptp, tso, (1 << tso));
	}
	ptp_tso_off(ptp, tso, (1 << tso));
#ifdef ENABLE_IRIG
	if (11 == tso) {
		if (irig_cnt > 1)
			irig(ptp);
		if (irig_cnt)
			irig_cnt--;
	}
#endif
}  /* ptp_tx_done */

static void proc_ptp_intr(struct work_struct *work)
{
	struct ptp_info *ptp =
		container_of(work, struct ptp_info, proc_ptp_irq);
	void *ptpdev = ptp->ptpdev;
	struct ptp_event *event;
	u16 done;
	u16 error;
	u16 status;
	u16 tsi_bit;
	u8 data[24];
	int i;
	int tsi;
	int last;
	struct timespec ts;

	ts = ktime_to_timespec(ktime_get_real());

proc_chk_trig_intr:
	status = ptp_read(ptpdev, ADDR_16, TRIG_INT_STATUS);
	if (!status)
		goto proc_chk_ts_intr;
#ifdef DBG_RX_INTR
	printk(KERN_DEBUG "ts_irq: %x\n", status);
#endif

	ptp_write(ptpdev, ADDR_16, TRIG_INT_STATUS, status);
	done = ptp_read(ptpdev, ADDR_16, TRIG_DONE);
	error = ptp_read(ptpdev, ADDR_16, TRIG_ERROR);
	for (i = 0; i < MAX_TRIG_UNIT; i++) {
		if (status & (1 << i)) {
			if (ptp->tso_intr & (1 << i)) {
				data[0] = PTP_CMD_GET_OUTPUT;
				data[1] = i;
				proc_ptp_get_trig(ptp, data, done, error);
			}
			ptp_tx_done(ptp, i);
		}
	}

proc_chk_ts_intr:
	status = ptp_read(ptpdev, ADDR_16, TS_INT_STATUS);
	if (!status)
		goto proc_ptp_intr_done;

	ptp_write(ptpdev, ADDR_16, TS_INT_STATUS, status);
	for (i = 0; i < MAX_TIMESTAMP_UNIT; i++) {
		tsi_bit = 1 << i;
		if (status & tsi_bit) {
			ptp_read_event(ptp, i);
			event = &ptp->events[i];
			if (event->timeout && (event->num < event->max ||
					event->last)) {
				unsigned long expired;

				expired = jiffies + event->timeout;
				event->expired = expired;
				if (event->last) {
					tsi = i + 1;
					do {
						if (tsi >= MAX_TIMESTAMP_UNIT)
							tsi = 0;
						ptp->events[tsi].expired =
							expired;
						++tsi;
					} while (tsi != event->last);
				}
			} else if (event->last && i != event->first) {
				tsi = i - 1;
				if (tsi < 0)
					tsi = MAX_TIMESTAMP_UNIT - 1;
				if (ptp->tsi_used & (1 << tsi))
					ptp->events[tsi].expired = jiffies;
			}
			if (i == ptp->pps_tsi) {
				struct ptp_utime sys_time;
#if defined(CONFIG_MICREL_FPGA_BOARD)
				u32 old_sec = ptp->cur_time.sec;
#endif

				ptp->cur_time.sec = event->t[0].sec;
				ptp->cur_time.nsec = event->t[0].nsec;
				sys_time.sec = ts.tv_sec;
				sys_time.nsec = ts.tv_nsec;
				calc_udiff(&ptp->cur_time, &sys_time,
					&ptp->time_diff);
#if defined(CONFIG_MICREL_FPGA_BOARD)
/*
 * THa  2011/08/12
 * The new FPGA has a problem that sometimes the data read is stripped off at
 * bit 7.
 */
				if (old_sec > ptp->cur_time.sec) {
#if 0
					dbg_msg("  ?sec: %x %x\n", old_sec,
						ptp->cur_time.sec);
#endif
					if ((ptp->cur_time.sec | 0x80) >
							old_sec)
						ptp->cur_time.sec |= 0x80;
				}
#if 0
				if (abs(ptp->cur_time.sec - old_sec) > 1)
					printk(KERN_INFO "  sec: %x %x\n",
						old_sec, ptp->cur_time.sec);
#endif
#endif
#if 1
				if ((abs(ptp->cur_time.nsec - ptp->pps_offset))
						>= 200)
					printk(KERN_INFO "  off: %d %u\n",
						ptp->pps_offset,
						ptp->cur_time.nsec);
#endif
			} else if (i == ptp->gps_tsi) {
				ptp->gps_time.sec = event->t[0].sec;
				ptp->gps_time.nsec = event->t[0].nsec;
				++ptp->gps_seqid;
				ptp->gps_resp_time = jiffies;
			}
#ifdef ENABLE_IRIG
			else if (7 == i) {
				if (irig_up_cnt < 20) {
					irig_up[irig_up_cnt] = event->t[0];
					irig_up_cnt++;
				}
				event->num = 0;
				if (irig_cnt > 1)
					ptp_rx_restart(ptpdev, tsi_bit);
				else
					ptp_rx_off(ptp, i);
			} else if (8 == i) {
				if (irig_dn_cnt < 20) {
					irig_dn[irig_dn_cnt] = event->t[0];
					irig_dn_cnt++;
				}
				event->num = 0;
				if (irig_cnt)
					ptp_rx_restart(ptpdev, tsi_bit);
				else {
					int dn = 0;
					int up = 0;

					while (dn < irig_dn_cnt ||
							up < irig_up_cnt) {
						if (up < irig_up_cnt)
							up++;
						if (dn < irig_dn_cnt)
							dn++;
					}
					ptp_rx_off(ptp, i);
				}
			}
#endif
		}
	}
	for (i = 0; i < MAX_TIMESTAMP_UNIT; i++) {
		int stop;

		stop = false;
		tsi_bit = 1 << i;
		event = &ptp->events[i];
		if (ptp->tsi_used & tsi_bit) {

			/* At least one event. */
			if (event->num || event->expired) {
				if (event->num >= event->max)
					stop = true;
				else if (event->expired &&
						jiffies >= event->expired) {
					ptp_read_event(ptp, i);
					stop = true;
				}
			}
		}
		if ((ptp->ts_status & ptp->ts_intr) & tsi_bit) {
			if (ptp->tsi_intr & tsi_bit) {
				data[0] = PTP_CMD_GET_EVENT;
				data[1] = i;
				proc_ptp_get_event(ptp, data);
			}
			if (i == ptp->gps_tsi && ptp->gps_req_time) {
				unsigned long diff = jiffies -
					ptp->gps_req_time;

				if (diff < 2 * ptp->delay_ticks) {
					data[0] = TSM_CMD_GET_GPS_TS;
					proc_tsm_get_gps(ptp, data);
					ptp->gps_time.sec = 0;
				}
				ptp->gps_req_time = 0;
			}

			/* Not used in cascade mode. */
			if (!event->timeout && !event->last) {
				event->num = 0;
				ptp_rx_restart(ptpdev, tsi_bit);
				stop = false;
			}
		}
		if (stop) {
			ptp_rx_off(ptp, i);
			ptp->tsi_intr &= ~tsi_bit;
			ptp->tsi_used &= ~tsi_bit;
			ptp->tsi_dev[i] = NULL;
			tsi = i;
			ptp->events[i].timeout = 0;
			if (i + 1 == event->last) {
				tsi = event->first;
				last = event->last;
				do {
					if (tsi >= MAX_TIMESTAMP_UNIT)
						tsi = 0;
					ptp->events[tsi].first = 0;
					ptp->events[tsi].last = 0;
					++tsi;
				} while (tsi != last);
			}
		}
	}
	ptp->ts_status = 0;
	if (!(status & 0xF000))
		goto proc_chk_trig_intr;

	if (get_tx_time(ptp, status & 0xF000))
		wake_up_interruptible(&ptp->wait_ts);
	goto proc_chk_trig_intr;

proc_ptp_intr_done:
	return;
}  /* proc_ptp_intr */

static struct ptp_tx_ts *proc_get_ts(struct ptp_info *ptp, u8 port, u8 msg,
	u16 seqid, u8 *mac, struct ptp_dev_info *info, int len)
{
	struct ptp_tx_ts *tx;
	int from_stack = false;
	u8 *data = NULL;

	if (info)
		data = info->write_buf;
	if (SYNC_MSG == msg)
		tx = &ptp->tx_sync[port];
	else if (PDELAY_RESP_MSG == msg)
		tx = &ptp->tx_resp[port];
	else
		tx = &ptp->tx_dreq[port];
	if (seqid || mac[0] || mac[1])
		from_stack = true;
	if (data && tx->req_time && ptp->linked[port])
		dbg_msg("  last %x=%04x: %d, %lu\n", msg, seqid, port,
			jiffies - tx->req_time);
	tx->missed = false;
	tx->req_time = jiffies;
	if (tx->ts.timestamp && from_stack) {
		unsigned long diff = tx->req_time - tx->resp_time;

		/* The timestamp is not valid. */
		if (diff >= 4 * ptp->delay_ticks) {
			dbg_msg("  invalid: %x=%04x: %d, %lu\n",
				msg, seqid, port, diff);
			tx->ts.timestamp = 0;
		}
#ifdef PTP_SPI
		else if (diff > 2 * ptp->delay_ticks)
			dbg_msg("  ready? %x=%04x: %d, %lu\n",
				msg, seqid, port, diff);
#endif
	}
	if (!tx->ts.timestamp && ptp->linked[port] && data) {
		int rc = wait_event_interruptible_timeout(ptp->wait_ts,
			0 != tx->ts.timestamp, ptp->delay_ticks);
		if (rc < 0)
			return NULL;
	}
	if (!tx->ts.timestamp) {
		if (from_stack && data) {
			tx->missed = true;
			memcpy(tx->data.buf, data, len);
			tx->data.len = len;
			tx->dev = info;
		}
#ifndef PTP_SPI
		if (ptp->linked[port])
			dbg_msg("  missed %x=%04x: %d, %lu\n",
				msg, seqid, port, jiffies - tx->req_time);
#endif
		tx = NULL;
	}
	return tx;
}  /* proc_get_ts */

static int proc_ptp_get_timestamp(struct ptp_info *ptp, u8 *data,
	struct ptp_dev_info *info)
{
	struct ptp_ts_options *opt = (struct ptp_ts_options *) data;

	if (opt->timestamp) {
		struct ptp_ts ts;

		ts.timestamp = opt->timestamp;
		update_ts(&ts, ptp->cur_time.sec);
		opt->sec = ts.t.sec;
		opt->nsec = ts.t.nsec;
	} else {
		struct ptp_tx_ts *tx;
		struct tsm_db *db;

		if (opt->port >= MAX_PTP_PORT)
			return DEV_IOC_INVALID_CMD;

		/* Save timestamp information for later reporting. */
		if (info) {
			db = (struct tsm_db *) info->write_buf;
			db->cmd = opt->msg;
			db->cmd |= TSM_CMD_DB_GET;
			db->index = opt->port << 1;
			db->seqid = htons(opt->seqid);
			db->mac[0] = opt->mac[0];
			db->mac[1] = opt->mac[1];
		}
		tx = proc_get_ts(ptp, opt->port, opt->msg,
			opt->seqid, opt->mac, info, sizeof(struct tsm_db));
		if (!tx)
			return -EAGAIN;
		opt->sec = tx->ts.r.sec;
		opt->nsec = tx->ts.r.nsec;
		tx->ts.timestamp = 0;
		tx->req_time = 0;
	}
	return 0;
}  /* proc_ptp_get_timestamp */

static int parse_tsm_msg(struct ptp_dev_info *info, int len)
{
	struct ptp_info *ptp = info->ptp;
	u8 *data = info->write_buf;
	void *ptpdev = ptp->ptpdev;
#ifdef PTP_MONITOR_ONTIME
	u32 timestamp;
	u8 sec_chk;
#endif
	u8 cmd = data[0] & 0xf0;
	u8 msg = data[0] & 0x03;
	int result = 0;

	switch (cmd) {
	case TSM_CMD_DB_GET_TIME:
	{
		struct tsm_get_time *get = (struct tsm_get_time *) data;
		struct ptp_ts ts;

		ts.timestamp = ntohl(get->nsec);
		if (ts.timestamp) {
			update_ts(&ts, ptp->cur_time.sec);
#ifdef PTP_MONITOR
			ptp->get_time_jiffies = jiffies;
			ptp->software_running = 1;
#endif
		} else {
			ptp_acquire(ptp);
			get_ptp_time(ptpdev, &ts.t);
			ptp_release(ptp);
		}
		ptp_setup_udp_msg(info, data, len, ptp_tsm_get_time_resp,
			&ts.t);
		break;
	}
	case TSM_CMD_DB_GET:
	{
		struct tsm_db *db = (struct tsm_db *) data;
#ifdef PTP_MONITOR_ONTIME
		struct ptp_ts *ts;
		struct ptp_rx_ts *rx;
		int head;
#endif

#if 0
		dbg_msg("get db: %x %x seqid=%04x mac=%02x:%02x %u:%u %u\n",
			msg,
			db->index, ntohs(db->seqid), db->mac[0], db->mac[1],
			db->cur_sec, db->cur_nsec, db->timestamp);
#endif
#ifdef PTP_MONITOR_ONTIME
		head = ptp->rx.head;
		while (head != ptp->rx.tail) {
			if (db->index < 0x10)
				break;
			rx = &ptp->rx.rx[head];
			ts = &rx->ts;
			if (msg == rx->id.msg && db->seqid == rx->id.seq &&
					db->mac[0] == rx->id.mac[0] &&
					db->mac[1] == rx->id.mac[1]) {
				get_cur_time(ptp, msg, rx->id.port, ts);
				timestamp = timestamp_val(ts->timestamp,
					&sec_chk);
				ptp_setup_udp_msg(info, data, len,
					ptp_tsm_resp, ts);
				break;
			}
			head = (head + 1) & (NUM_OF_TIMESTAMP - 1);
		}
#endif
		if (db->index <= 3) {
			struct ptp_tx_ts *tx;
			int port = db->index >> 1;

			tx = proc_get_ts(ptp, port, msg, ntohs(db->seqid),
				db->mac, info, len);
			if (tx) {
				ptp_setup_udp_msg(info, data, len,
					ptp_tsm_resp, &tx->ts);
				tx->ts.timestamp = 0;
				tx->req_time = 0;
			}
		}
		break;
	}
	case TSM_CMD_GET_GPS_TS:
	{
		/* First time to get GPS timestamp. */
		if (MAX_TIMESTAMP_UNIT == ptp->gps_tsi) {
			ptp->gps_tsi = DEFAULT_GPS_TSI;
			if (ptp->tsi_used & (1 << ptp->gps_tsi))
				ptp_rx_off(ptp, ptp->gps_tsi);
			prepare_gps(ptp);
			ptp->gps_seqid = 0;
		}
		ptp->gps_req_time = jiffies;
		ptp->gps_dev = info;
		if (ptp->gps_resp_time) {
			unsigned long diff = ptp->gps_req_time -
				ptp->gps_resp_time;

			/* The timestamp is not valid. */
			if (diff >= 2 * ptp->delay_ticks) {
				dbg_msg("  invalid gps: %lu\n", diff);
				ptp->gps_time.sec = 0;
			}
		}
		if (ptp->gps_time.sec) {
			proc_tsm_get_gps(ptp, data);
			ptp->gps_time.sec = 0;
			ptp->gps_req_time = 0;
		} else
			dbg_msg("  missed gps\n");
		break;
	}
	case TSM_CMD_CNF_SET:
	{
		struct tsm_cfg *cfg = (struct tsm_cfg *) data;
		u32 ingress = htonl(cfg->ingress_delay);
#ifdef PTP_MONITOR_ONTIME
		u16 egress = htons(cfg->egress_delay);

		dbg_msg("set cfg: p=%02x %02x gmp=%u ingress=%u egress=%u\n",
			cfg->port, cfg->enable, cfg->gmp,
			ingress, egress);
#endif

		ptp_acquire(ptp);
		if (0xFF == cfg->port) {
			if (cfg->enable & 0x04)
				ptp->mode |= PTP_TC_P2P;
			else
				ptp->mode &= ~PTP_TC_P2P;
			set_ptp_mode(ptpdev, ptp->mode);
			ptp->def_mode = ptp->mode;
		} else {
			u8 port = cfg->port - 1;

			if ((cfg->enable & 0x10) && port < MAX_PTP_PORT &&
					ptp->peer_delay[port] != ingress) {
				ptp->peer_delay[port] = ingress;
				set_ptp_link(ptpdev, port, ingress);
			}
		}
		ptp_release(ptp);
		break;
	}
	case TSM_CMD_CLOCK_SET:
	{
		struct tsm_clock_set *clk = (struct tsm_clock_set *) data;
		struct ptp_ts ts;
#ifdef PTP_MONITOR_ONTIME
		char n1[FMT_NSEC_SIZE];
		char n2[FMT_NSEC_SIZE];
#endif

		ts.t.sec = ntohl(clk->sec);
		ts.t.nsec = ntohl(clk->nsec);
		ts.timestamp = ntohl(clk->timestamp);
		ptp_acquire(ptp);
		set_cur_time(ptp, &ts);
		ptp_release(ptp);
		ptp->state = 2;
#ifdef PTP_MONITOR_ONTIME
		format_nsec(n1, ts.t.nsec);
		format_nsec(n2, ts.timestamp);
		dbg_msg("  set: s:%08x ns:%s t:%s\n", ts.t.sec, n1, n2);
#endif
		break;
	}
	case TSM_CMD_CLOCK_CORRECT:
	{
		struct tsm_clock_correct *clk = (struct tsm_clock_correct *)
			data;
		u32 drift;
		u32 nsec;
		int ptp_offset;
#ifdef PTP_MONITOR_ONTIME
		char n1[FMT_NSEC_SIZE];
		char n2[FMT_NSEC_SIZE];
		char a;
#endif

		drift = ntohl(clk->drift);
		nsec = ntohl(clk->nsec);
		ptp_offset = ntohl(clk->offset);
		if (2 == (clk->add >> 4)) {
#ifdef PTP_MONITOR_ONTIME
			dbg_msg("  starting sync\n");
			ptp->last_correct_jiffies = jiffies;
#endif
			break;
		}
#ifdef PTP_MONITOR_ONTIME
		a = ' ';
#endif
		ptp_acquire(ptp);
		if (nsec) {
			adjust_ptp_time(ptpdev, !ptp_offset, 0, nsec,
				ptp->features & PTP_ADJ_HACK);
			ptp->offset_changed = nsec;
			if (ptp_offset)
				ptp->offset_changed = -nsec;
			generate_tx_event(ptp, ptp->pps_gpo);
#ifdef PTP_MONITOR_ONTIME
			a = '>';
#endif
		}
		if (clk->add & 1)
			ptp->drift = drift;
		else
			ptp->drift = -drift;
		ptp->drift_set = ptp->drift;
		ptp->adjust = clk_adjust_val(ptp->drift,
			NANOSEC_IN_SEC);
		set_ptp_adjust(ptpdev, ptp->adjust);
		ptp_release(ptp);
#ifdef PTP_MONITOR_ONTIME
		if (jiffies - ptp->last_correct_jiffies < 20)
			break;
		format_nsec(n1, ntohl(clk->nsec));
		format_nsec(n2, ntohl(clk->drift));
		dbg_msg("\n");
		dbg_msg("%ccorrect: %d:%d s:%9u ns:%s d:%s o:%u\n", a,
			clk->add >> 4, clk->add & 1,
			ntohl(clk->sec), n1, n2, ntohl(clk->offset));
		ptp->last_correct_jiffies = jiffies;
#endif
		break;
	}
	default:
		dbg_msg("tsm cmd: %02X, %d\n", cmd, len);
	}
#ifdef DEBUG_MSG
	schedule_work(&db.dbg_print);
#endif
	return result;
}  /* parse_tsm_msg */

static struct ptp_info *ptp_priv;

static struct ptp_dev_info *alloc_dev_info(unsigned int minor)
{
	struct ptp_dev_info *info;

	info = kzalloc(sizeof(struct ptp_dev_info), GFP_KERNEL);
	if (info) {
		info->ptp = ptp_priv;
		sema_init(&info->sem, 1);
		mutex_init(&info->lock);
		init_waitqueue_head(&info->wait_udp);
		info->write_len = 1000;
		info->write_buf = kzalloc(info->write_len, GFP_KERNEL);
		info->read_max = 60000;
		info->read_buf = kzalloc(info->read_max, GFP_KERNEL);

		info->minor = minor;
		info->next = ptp_priv->dev[minor];
		ptp_priv->dev[minor] = info;
	}
	return info;
}  /* alloc_dev_info */

static void free_dev_info(struct ptp_dev_info *info)
{
	if (info) {
		struct ptp_info *ptp = info->ptp;
		unsigned int minor = info->minor;
		struct ptp_dev_info *prev = ptp->dev[minor];

		if (prev == info) {
			ptp->dev[minor] = info->next;
		} else {
			while (prev && prev->next != info)
				prev = prev->next;
			if (prev)
				prev->next = info->next;
		}
		kfree(info->read_buf);
		kfree(info->write_buf);
		kfree(info);
	}
}  /* free_dev_info */

static int ptp_dev_open(struct inode *inode, struct file *filp)
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;
	unsigned int minor = MINOR(inode->i_rdev);

	if (minor > 1)
		return -ENODEV;
	if (!info) {
		info = alloc_dev_info(minor);
		if (info)
			filp->private_data = info;
		else
			return -ENOMEM;
	}
	return 0;
}  /* ptp_dev_open */

static int ptp_dev_release(struct inode *inode, struct file *filp)
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;

	free_dev_info(info);
	filp->private_data = NULL;
	return 0;
}  /* ptp_dev_release */

#define DEV_IOC_MAGIC			0x92

enum {
	DEV_IO_PTP,
	DEV_IOC_MAX
};

#define DEV_IOC_PTP			\
	_IOW(DEV_IOC_MAGIC, DEV_IO_PTP, struct ksz_request)

#define PARAM_DATA_SIZE			40

static int chk_ioctl_size(int len, int size, int additional, int *req_size,
	int *result, void *param, u8 *data)
{
	if (len < size) {
		printk(KERN_INFO "wrong size: %d %d\n", len, size);
		*req_size = size + additional;
		*result = DEV_IOC_INVALID_LEN;
		return -1;
	}
	if (size >= PARAM_DATA_SIZE) {
		printk(KERN_INFO "large size: %d\n", size);
		*result = -EFAULT;
		return -1;
	}
	if (data && (!access_ok(VERIFY_READ, param, size) ||
			copy_from_user(data, param, size))) {
		*result = -EFAULT;
		return -1;
	}
	return 0;
}  /* chk_ioctl_size */

static u8 eth_pdelay[] = { 0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E };
static u8 eth_others[] = { 0x01, 0x1B, 0x19, 0x00, 0x00, 0x00 };

static int proc_ptp_eth_msg(struct dev_info *hw_priv, int len, int *req_size,
	void *param)
{
	int size;
	u32 overrides;
	struct sk_buff *skb;
	struct ptp_msg_hdr *hdr = param;
	int ret = 0;

	if (len < sizeof(struct ptp_msg_hdr)) {
		*req_size = sizeof(struct ksz_request) +
			sizeof(struct ptp_msg_hdr);
		ret = DEV_IOC_INVALID_LEN;
		goto proc_eth_msg_done;
	}
	size = ntohs(hdr->messageLength);
	if (size > len) {
		ret = -EFAULT;
		goto proc_eth_msg_done;
	}
	skb = dev_alloc_skb(len + 14 + 4);
	if (!skb) {
		ret = -ENOMEM;
		goto proc_eth_msg_done;
	}
	if (!access_ok(VERIFY_READ, param, len) ||
			copy_from_user(&skb->data[14], param, len)) {
		ret = -EFAULT;
		dev_kfree_skb(skb);
		goto proc_eth_msg_done;
	}
	skb->data[12] = 0x88;
	skb->data[13] = 0xF7;
	skb->dev = hw_priv->dev;
	skb->len = len + 14;
	memcpy(&skb->data[6], hw_priv->hw.mac_addr, 6);
	switch (hdr->messageType) {
	case PDELAY_REQ_MSG:
	case PDELAY_RESP_MSG:
	case PDELAY_RESP_FOLLOW_UP_MSG:
		memcpy(skb->data, eth_pdelay, 6);
		break;
	default:
		memcpy(skb->data, eth_others, 6);
		break;
	}
	overrides = hw_priv->hw.overrides;
	hw_priv->hw.overrides &= ~TX_DROP;
	if (netdev_tx(skb, skb->dev)) {
		dev_kfree_skb(skb);
		ret = -EFAULT;
	}
	hw_priv->hw.overrides = overrides;

proc_eth_msg_done:
	return ret;
}  /* proc_ptp_eth_msg */

static int execute_wait(struct ptp_work *work)
{
	int rc = 0;

	execute(work->ptp, &work->work);
#ifdef PTP_SPI
	wait_for_completion(&work->done);
#endif
	return rc;
}  /* execute_wait */

static void proc_ptp_work(struct work_struct *work)
{
	struct ptp_work *parent =
		container_of(work, struct ptp_work, work);
	struct ptp_info *ptp = parent->ptp;
	struct ptp_dev_info *info = parent->dev_info;
	u8 *data = parent->param.data;
	int port;
	int reg;
	int result = DEV_IOC_OK;

	switch (parent->cmd) {
	case DEV_CMD_INFO:
		switch (parent->subcmd) {
		case DEV_INFO_INIT:
			ptp_init_state(ptp);
			parent->output = ptp->drift_set;
			break;
		case DEV_INFO_EXIT:
			ptp_exit_state(ptp);
			break;
		case DEV_INFO_RESET:
			reg = parent->option;
			mutex_lock(ptp->hwlock);
			ptp_write(ptp->ptpdev, ADDR_16, REG_RESET_CTRL,
				1 << reg);
			ptp_write(ptp->ptpdev, ADDR_16, REG_RESET_CTRL, 0);
			mutex_unlock(ptp->hwlock);
			break;
		default:
			result = DEV_IOC_INVALID_CMD;
			break;
		}
		break;
	case DEV_CMD_PUT:
		switch (parent->subcmd) {
		case DEV_PTP_CFG:
			result = proc_ptp_set_cfg(ptp, data);
			break;
		case DEV_PTP_TEVT:
			result = proc_dev_rx_event(info, data);
			parent->output = *data;
			break;
		case DEV_PTP_TOUT:
			result = proc_dev_tx_event(info, data);
			parent->output = *data;
			break;
		case DEV_PTP_CASCADE:
			if (parent->option)
				result = proc_ptp_tx_cascade(ptp, data);
			else
				result = proc_ptp_tx_cascade_init(ptp, data);
			parent->output = *data;
			break;
		case DEV_PTP_CLK:
			if (parent->option)
				result = proc_ptp_adj_clk(ptp, data,
					parent->option);
			else
				result = proc_ptp_set_clk(ptp, data);
			break;
		case DEV_PTP_DELAY:
			port = parent->option;
			result = proc_ptp_set_delay(ptp, port, data);
			break;
		case DEV_PTP_REG:
			reg = parent->option;
			mutex_lock(ptp->hwlock);
			ptp_write(ptp->ptpdev, ADDR_16, reg & 0xffff,
				reg >> 16);
			mutex_unlock(ptp->hwlock);
			break;
		case DEV_PTP_PEER_DELAY:
			port = parent->option;
			result = proc_ptp_set_peer_delay(ptp, port, data);
			break;
		default:
			result = DEV_IOC_INVALID_CMD;
			break;
		}
		break;
	case DEV_CMD_GET:
		switch (parent->subcmd) {
		case DEV_PTP_CFG:
			proc_ptp_get_cfg(ptp, data);
			break;
		case DEV_PTP_TEVT:
			if (parent->option)
				result = proc_dev_poll_event(info, data);

			/* Not actually used. */
			else
				result = proc_dev_get_event(info, data);
			break;
		case DEV_PTP_TOUT:
			break;
		case DEV_PTP_CLK:
			proc_ptp_get_clk(ptp, data);
			break;
		case DEV_PTP_DELAY:
			port = parent->option;
			result = proc_ptp_get_delay(ptp, port, data);
			break;
		case DEV_PTP_REG:
			reg = parent->option;
			mutex_lock(ptp->hwlock);
			parent->output = ptp_read(ptp->ptpdev, ADDR_16, reg);
			mutex_unlock(ptp->hwlock);
			break;
		case DEV_PTP_PEER_DELAY:
			port = parent->option;
			result = proc_ptp_get_peer_delay(ptp, port, data);
			break;
		}
		break;
	default:
		result = DEV_IOC_INVALID_CMD;
		break;
	}
	parent->result = result;
	parent->used = false;
	complete(&parent->done);
}  /* proc_ptp_work */

static int proc_ptp_hw_access(struct ptp_info *ptp, int cmd, int subcmd,
	int option, void *data, size_t len, struct ptp_dev_info *info,
	int *output, int wait)
{
	struct ptp_access *access;
	struct ptp_work *work;
	int ret = 0;

	access = &ptp->hw_access;
	work = &access->works[access->index];
	if (work->used) {
		pr_alert("work full\n");
		return -EFAULT;
	}
	work->cmd = cmd;
	work->subcmd = subcmd;
	work->option = option;
	memcpy(work->param.data, data, len);
	work->dev_info = info;
	work->used = true;
	access->index++;
	access->index &= PTP_WORK_LAST;
#ifdef PTP_SPI
	init_completion(&work->done);
#endif
	if (!wait) {
		execute(ptp, &work->work);
		goto hw_access_end;
	}
	ret = execute_wait(work);

	/* Cannot continue if ERESTARTSYS. */
	if (ret < 0)
		return ret;

	ret = work->result;
	if (DEV_IOC_OK == ret && DEV_CMD_GET == work->cmd)
		memcpy(data, work->param.data, len);
	*output = work->output;

hw_access_end:
	return ret;
}  /* proc_ptp_hw_access */

static void init_ptp_work(struct ptp_info *ptp)
{
	struct ptp_access *access;
	struct ptp_work *work;
	int i;

	access = &ptp->hw_access;
	for (i = 0; i < PTP_WORK_NUM; i++) {
		work = &access->works[i];
		work->ptp = ptp;
		INIT_WORK(&work->work, proc_ptp_work);
		init_completion(&work->done);
	}
}  /* init_ptp_work */

#define PTP_ENABLE_TXTS		SIOCDEVPRIVATE
#define PTP_DISABLE_TXTS	(SIOCDEVPRIVATE + 1)
#define PTP_ENABLE_RXTS		(SIOCDEVPRIVATE + 2)
#define PTP_DISABLE_RXTS	(SIOCDEVPRIVATE + 3)
#define PTP_GET_TX_TIMESTAMP	(SIOCDEVPRIVATE + 4)
#define PTP_GET_RX_TIMESTAMP	(SIOCDEVPRIVATE + 5)
#define PTP_SET_TIME		(SIOCDEVPRIVATE + 6)
#define PTP_GET_TIME		(SIOCDEVPRIVATE + 7)
#define PTP_SET_FIPER_ALARM	(SIOCDEVPRIVATE + 8)
#define PTP_SET_ADJ		(SIOCDEVPRIVATE + 9)
#define PTP_GET_ADJ		(SIOCDEVPRIVATE + 10)
#define PTP_CLEANUP_TS		(SIOCDEVPRIVATE + 11)
#define PTP_ADJ_TIME		(SIOCDEVPRIVATE + 12)

struct ixxat_ptp_time {
	/* just 48 bit used */
	u64 sec;
	u32 nsec;
};

struct ixxat_ptp_ident {
	u8 vers;
	u8 mType;
	u16 netwProt;
	u16 seqId;
	struct ptp_port_identity portId;
} __packed;

/* needed for timestamp data over ioctl */
struct ixxat_ptp_data {
	struct ixxat_ptp_ident ident;
	struct ixxat_ptp_time ts;
};

static int ixxat_ptp_ioctl(struct ptp_info *ptp, unsigned int cmd,
	struct ifreq *ifr)
{
	struct ixxat_ptp_time ptp_time;
	struct ixxat_ptp_data ptp_data;
	struct ptp_clk_options clk_opt;
	int output;
	s64 scaled_nsec;
	struct ptp_ts ts;
	struct ptp_tx_ts *tx;
	int drift;
	int err = 0;
	int port;

	switch (cmd) {
	case PTP_ENABLE_TXTS:
		ptp->tx_en |= 2;
		break;
	case PTP_DISABLE_TXTS:
		ptp->tx_en &= ~2;
		break;
	case PTP_ENABLE_RXTS:
		ptp->rx_en |= 2;
		break;
	case PTP_DISABLE_RXTS:
		ptp->rx_en &= ~2;
		break;
	case PTP_GET_TX_TIMESTAMP:
		if (copy_from_user(&ptp_data, ifr->ifr_data, sizeof(ptp_data)))
			return -EFAULT;

		err = -EINVAL;
#if 0
		dbg_msg(" ts: %x %x %x %04x %02x:%02x:%02x:%02x:%02x:%02x-%d\n",
			ptp_data.ident.vers,
			ptp_data.ident.mType,
			ptp_data.ident.netwProt,
			ptp_data.ident.seqId,
			ptp_data.ident.portId.clockIdentity.addr[0],
			ptp_data.ident.portId.clockIdentity.addr[1],
			ptp_data.ident.portId.clockIdentity.addr[2],
			ptp_data.ident.portId.clockIdentity.addr[5],
			ptp_data.ident.portId.clockIdentity.addr[6],
			ptp_data.ident.portId.clockIdentity.addr[7],
			htons(ptp_data.ident.portId.port));
#endif
		port = htons(ptp_data.ident.portId.port);
		if (port < 1 || port > MAX_PTP_PORT)
			break;
		port--;
		tx = proc_get_ts(ptp, port, ptp_data.ident.mType,
			ptp_data.ident.seqId,
			ptp_data.ident.portId.clockIdentity.addr,
			NULL, 0);
		if (!tx)
			break;
		ptp_data.ts.sec = tx->ts.r.sec;
		ptp_data.ts.nsec = tx->ts.r.nsec;
		tx->ts.timestamp = 0;
		tx->req_time = 0;
		err = copy_to_user(ifr->ifr_data, &ptp_data, sizeof(ptp_data));
		break;
	case PTP_GET_RX_TIMESTAMP:
		if (copy_from_user(&ptp_data, ifr->ifr_data, sizeof(ptp_data)))
			return -EFAULT;

		ts.timestamp = ptp_data.ts.nsec;
		if (ts.timestamp)
			update_ts(&ts, ptp->cur_time.sec);
		else {
			ptp_acquire(ptp);
			get_ptp_time(ptp->ptpdev, &ts.t);
			ptp_release(ptp);
		}
		ptp_data.ts.sec = ts.t.sec;
		ptp_data.ts.nsec = ts.t.nsec;
		err = copy_to_user(ifr->ifr_data, &ptp_data, sizeof(ptp_data));
		break;
	case PTP_GET_TIME:
	{
		struct timespec ts;
		struct ptp_time cur_time;
		struct ptp_time sys_time;

		ts = ktime_to_timespec(ktime_get_real());
		sys_time.sec = ts.tv_sec;
		sys_time.nsec = ts.tv_nsec;
		calc_diff(&ptp->time_diff, &sys_time, &cur_time);
		ptp_time.sec = cur_time.sec;
		ptp_time.nsec = cur_time.nsec;
		err = proc_ptp_hw_access(ptp,
			DEV_CMD_GET, DEV_PTP_CLK, 0,
			&clk_opt, sizeof(clk_opt), NULL, &output,
			true);
		ptp_time.sec = clk_opt.sec;
		ptp_time.nsec = clk_opt.nsec;
		err = copy_to_user(ifr->ifr_data, &ptp_time, sizeof(ptp_time));
		break;
	}
	case PTP_SET_TIME:
		if (copy_from_user(&ptp_time, ifr->ifr_data, sizeof(ptp_time)))
			return -EFAULT;
		output = 0;
		clk_opt.sec = (u32) ptp_time.sec;
		clk_opt.nsec = ptp_time.nsec;
		err = proc_ptp_hw_access(ptp,
			DEV_CMD_PUT, DEV_PTP_CLK, output,
			&clk_opt, sizeof(clk_opt), NULL, &output,
			true);
		break;
	case PTP_ADJ_TIME:
		if (copy_from_user(&scaled_nsec, ifr->ifr_data, sizeof(s64)))
			return -EFAULT;
		convert_scaled_nsec(scaled_nsec, SCALED_NANOSEC_SHIFT,
			&ptp->adjust_sec, &ptp->adjust_offset);
		if (ptp->adjust_offset < 0 || ptp->adjust_sec < 0) {
			output = 1;
			clk_opt.sec = -ptp->adjust_sec;
			clk_opt.nsec = -ptp->adjust_offset;
		} else {
			output = 2;
			clk_opt.sec = ptp->adjust_sec;
			clk_opt.nsec = ptp->adjust_offset;
		}
		clk_opt.interval = 0;
		err = proc_ptp_hw_access(ptp,
			DEV_CMD_PUT, DEV_PTP_CLK, output,
			&clk_opt, sizeof(clk_opt), NULL, &output,
			true);
		break;
	case PTP_SET_ADJ:
		if (copy_from_user(&drift, ifr->ifr_data, sizeof(drift)))
			return -EFAULT;
		output = 1;
		clk_opt.sec = clk_opt.nsec = 0;
		clk_opt.drift = drift;
		clk_opt.interval = NANOSEC_IN_SEC;
		err = proc_ptp_hw_access(ptp,
			DEV_CMD_PUT, DEV_PTP_CLK, output,
			&clk_opt, sizeof(clk_opt), NULL, &output,
			true);
		break;
	case PTP_GET_ADJ:
		drift = ptp->drift_set;
		err = copy_to_user(ifr->ifr_data, &drift, sizeof(drift));
		break;
	case PTP_CLEANUP_TS:
		break;
	case PTP_SET_FIPER_ALARM:
		break;
	default:
		err = -EOPNOTSUPP;
	}
	return err;
}

static int ptp_dev_req(struct ptp_info *ptp, char *arg,
	struct ptp_dev_info *info)
{
	struct dev_info *hw_priv = container_of(ptp, struct dev_info, ptp_hw);
	struct ksz_request *req = (struct ksz_request *) arg;
	int len;
	int maincmd;
	int port;
	int req_size;
	int subcmd;
	int output;
	u8 data[PARAM_DATA_SIZE];
	struct ptp_dev_info *dev;
	int err = 0;
	int result = 0;

	/* Assume success. */
	result = DEV_IOC_OK;

	/* Check request size. */
	__get_user(req_size, &req->size);
	if (chk_ioctl_size(req_size, sizeof(struct ksz_request), 0, &req_size,
			&result, NULL, NULL))
		goto dev_ioctl_resp;

	err = 0;
	__get_user(maincmd, &req->cmd);
	__get_user(subcmd, &req->subcmd);
	__get_user(output, &req->output);
	len = req_size - sizeof(struct ksz_request);
	switch (maincmd) {
	case DEV_CMD_INFO:
		switch (subcmd) {
		case DEV_INFO_INIT:
			req_size = sizeof(struct ksz_request) + 4;
			if (len >= 4) {
				data[0] = 'M';
				data[1] = 'i';
				data[2] = 'c';
				data[3] = 'r';
				data[4] = ptp->version;
				data[5] = ptp->ports;
				if (!access_ok(VERIFY_WRITE, req->param.data,
						6) ||
						copy_to_user(req->param.data,
						data, 6)) {
					err = -EFAULT;
					goto dev_ioctl_done;
				}
				result = proc_ptp_hw_access(ptp,
					maincmd, subcmd, 0,
					data, 6, info, &output,
					true);
				__put_user(ptp->drift_set, &req->output);
			} else
				result = DEV_IOC_INVALID_LEN;
			break;
		case DEV_INFO_EXIT:
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, 0, info, &output,
				true);

		/* fall through */
		case DEV_INFO_QUIT:
			if (!info)
				break;
			data[0] = 0xF0;
			dev = find_minor_dev(info);
			if (dev)
				ptp_setup_udp_msg(dev, data, 4, NULL, NULL);
			ptp_setup_udp_msg(info, data, 4, NULL, NULL);
			break;
		case DEV_INFO_MSG:
			result = proc_ptp_eth_msg(hw_priv, len, &req_size,
				&req->param);
			break;
		case DEV_INFO_RESET:
			if (output < 3) {
				result = proc_ptp_hw_access(ptp,
					maincmd, subcmd, output,
					data, 0, info, &output,
					false);
			} else
				result = -EINVAL;
			break;
		default:
			result = DEV_IOC_INVALID_CMD;
			break;
		}
		break;
	case DEV_CMD_PUT:
		switch (subcmd) {
		case DEV_PTP_CFG:
			if (chk_ioctl_size(len, sizeof(struct ptp_cfg_options),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				false);
			break;
		case DEV_PTP_TEVT:
			if (chk_ioctl_size(len, sizeof(struct ptp_tsi_options),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			if (!info) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				true);
			__put_user(*data, &req->output);
			break;
		case DEV_PTP_TOUT:
			if (chk_ioctl_size(len, sizeof(struct ptp_tso_options),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			if (!info) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				true);
			__put_user(*data, &req->output);
			break;
		case DEV_PTP_CASCADE:
			if (chk_ioctl_size(len, sizeof(struct ptp_tso_options),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			__put_user(*data, &req->output);
			break;
		case DEV_PTP_CLK:
			if (chk_ioctl_size(len, sizeof(struct ptp_clk_options),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			break;
		case DEV_PTP_DELAY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_delay_values),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			__get_user(port, &req->output);
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				false);
			break;
		case DEV_PTP_REG:
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				false);
			break;
		case DEV_PTP_IDENTITY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_clock_identity),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			memcpy(&ptp->clockIdentity, data,
				sizeof(struct ptp_clock_identity));
			break;
		case DEV_PTP_PEER_DELAY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_delay_values),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			__get_user(port, &req->output);
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				false);
			break;
		case DEV_PTP_UTC_OFFSET:
			ptp->utc_offset = output;
			break;
		default:
			result = DEV_IOC_INVALID_CMD;
			break;
		}
		break;
	case DEV_CMD_GET:
		switch (subcmd) {
		case DEV_PTP_CFG:
			if (chk_ioctl_size(len, sizeof(struct ptp_cfg_options),
					sizeof(struct ksz_request),
					&req_size, &result, NULL, NULL))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				true);
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_cfg_options)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_cfg_options))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_TEVT:
			if (chk_ioctl_size(len, sizeof(struct ptp_tsi_info),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			if (!info) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			if (output)
				result = proc_ptp_hw_access(ptp,
					maincmd, subcmd, output,
					data, len, info, &output,
					false);
			else
				result = proc_dev_get_event(info, data);
			break;
		case DEV_PTP_TOUT:
			if (chk_ioctl_size(len, sizeof(struct ptp_tso_options),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_get_output(ptp, data);
			output = *((int *) data);
			__put_user(output, &req->output);
			break;
		case DEV_PTP_CLK:
			if (chk_ioctl_size(len, sizeof(struct ptp_clk_options),
					sizeof(struct ksz_request),
					&req_size, &result, NULL, NULL))
				goto dev_ioctl_resp;
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, 0,
				data, len, info, &output,
				true);
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_clk_options)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_clk_options))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_DELAY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_delay_values),
					sizeof(struct ksz_request),
					&req_size, &result, NULL, NULL))
				goto dev_ioctl_resp;
			__get_user(port, &req->output);
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_delay_values)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_delay_values))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_REG:
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			__put_user(output, &req->output);
			break;
		case DEV_PTP_IDENTITY:
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_clock_identity)) ||
					copy_to_user(req->param.data,
					&ptp->clockIdentity,
					sizeof(struct ptp_clock_identity))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_PEER_DELAY:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_delay_values),
					sizeof(struct ksz_request),
					&req_size, &result, NULL, NULL))
				goto dev_ioctl_resp;
			__get_user(port, &req->output);
			result = proc_ptp_hw_access(ptp,
				maincmd, subcmd, output,
				data, len, info, &output,
				true);
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_delay_values)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_delay_values))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		case DEV_PTP_UTC_OFFSET:
			__put_user(ptp->utc_offset, &req->output);
			break;
		case DEV_PTP_TIMESTAMP:
			if (chk_ioctl_size(len,
					sizeof(struct ptp_ts_options),
					sizeof(struct ksz_request),
					&req_size, &result, &req->param, data))
				goto dev_ioctl_resp;
			result = proc_ptp_get_timestamp(ptp, data, info);
			if (result)
				goto dev_ioctl_resp;
			if (!access_ok(VERIFY_WRITE, req->param.data,
					sizeof(struct ptp_ts_options)) ||
					copy_to_user(req->param.data, data,
					sizeof(struct ptp_ts_options))) {
				err = -EFAULT;
				goto dev_ioctl_done;
			}
			break;
		}
		break;
	default:
		result = DEV_IOC_INVALID_CMD;
		break;
	}

dev_ioctl_resp:
	__put_user(req_size, &req->size);
	__put_user(result, &req->result);

	/* Return ERESTARTSYS so that the system call is called again. */
	if (result < 0)
		err = result;

dev_ioctl_done:
	return err;
}  /* ptp_dev_req */

#ifdef HAVE_UNLOCKED_IOCTL
static long ptp_dev_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
#else
static int ptp_dev_ioctl(struct inode *inode, struct file *filp,
	unsigned int cmd, unsigned long arg)
#endif
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;
	struct ptp_info *ptp = info->ptp;
	int err = 0;

	if (_IOC_TYPE(cmd) != DEV_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > DEV_IOC_MAX)
		return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void *) arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void *) arg, _IOC_SIZE(cmd));
	if (err) {
		printk(KERN_ALERT "err fault\n");
		return -EFAULT;
	}
	if (down_interruptible(&info->sem))
		return -ERESTARTSYS;

	err = ptp_dev_req(ptp, (char *) arg, info);
	up(&info->sem);
	return err;
}  /* ptp_dev_ioctl */

static ssize_t ptp_dev_read(struct file *filp, char *buf, size_t count,
	loff_t *offp)
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;
	ssize_t result = 0;
	int rc;

	if (!info->read_len) {
		*offp = 0;
		rc = wait_event_interruptible(info->wait_udp,
			0 != info->read_len);

		/* Cannot continue if ERESTARTSYS. */
		if (rc < 0)
			return 0;
	}

	if (down_interruptible(&info->sem))
		return -ERESTARTSYS;

	mutex_lock(&info->lock);
	if (*offp >= info->read_len) {
		info->read_len = 0;
		count = 0;
		*offp = 0;
		goto dev_read_done;
	}

	if (*offp + count > info->read_len) {
		count = info->read_len - *offp;
		info->read_len = 0;
	}

	if (copy_to_user(buf, &info->read_buf[*offp], count)) {
		result = -EFAULT;
		goto dev_read_done;
	}
	if (info->read_len)
		*offp += count;
	else
		*offp = 0;
	result = count;

dev_read_done:
	mutex_unlock(&info->lock);
	up(&info->sem);
	return result;
}  /* ptp_dev_read */

static ssize_t ptp_dev_write(struct file *filp, const char *buf, size_t count,
	loff_t *offp)
{
	struct ptp_dev_info *info = (struct ptp_dev_info *)
		filp->private_data;
	ssize_t result = 0;
	size_t size;
	int rc;
	u8 cmd;

	if (!count)
		return result;

	if (down_interruptible(&info->sem))
		return -ERESTARTSYS;

	if (*offp >= info->write_len) {
		result = -ENOSPC;
		goto dev_write_done;
	}
	if (*offp + count > info->write_len)
		count = info->write_len - *offp;
	if (copy_from_user(info->write_buf, buf, count)) {
		result = -EFAULT;
		goto dev_write_done;
	}
	cmd = info->write_buf[0] & 0xf0;
	switch (cmd) {
	case TSM_CMD_GET_GPS_TS:
		size = sizeof(struct tsm_get_gps);
		break;
	case TSM_CMD_DB_GET_TIME:
		size = sizeof(struct tsm_get_time);
		break;
	case TSM_CMD_DB_GET:
		size = sizeof(struct tsm_db);
		break;
	case TSM_CMD_CNF_SET:
		size = sizeof(struct tsm_cfg);
		break;
	case TSM_CMD_CLOCK_SET:
		size = sizeof(struct tsm_clock_set);
		break;
	case TSM_CMD_CLOCK_CORRECT:
		size = sizeof(struct tsm_clock_correct);
		break;
	default:
		dbg_msg("tsm: %x\n", info->write_buf[0]);
		result = count;
		goto dev_write_done;
	}
	if (count < size) {
		result = -EFAULT;
		goto dev_write_done;
	}
	result = size;
	rc = parse_tsm_msg(info, count);
	if (rc)
		result = rc;

dev_write_done:
	up(&info->sem);
	return result;
}  /* ptp_dev_write */

static const struct file_operations ptp_dev_fops = {
	.read		= ptp_dev_read,
	.write		= ptp_dev_write,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl	= ptp_dev_ioctl,
#else
	.ioctl		= ptp_dev_ioctl,
#endif
	.open		= ptp_dev_open,
	.release	= ptp_dev_release,
};

static struct class *ptp_class;

static int init_ptp_device(int dev_major, char *dev_name, char *minor_name)
{
	int result;

	printk(KERN_INFO "  PTP driver %s %s\n", __DATE__, __TIME__);
	result = register_chrdev(dev_major, dev_name, &ptp_dev_fops);
	if (result < 0) {
		printk(KERN_WARNING "%s: can't get major %d\n", dev_name,
			dev_major);
		return result;
	}
	if (0 == dev_major)
		dev_major = result;
	ptp_class = class_create(THIS_MODULE, dev_name);
	if (IS_ERR(ptp_class)) {
		unregister_chrdev(dev_major, dev_name);
		return -ENODEV;
	}
	device_create(ptp_class, NULL, MKDEV(dev_major, 0), NULL, dev_name);
	device_create(ptp_class, NULL, MKDEV(dev_major, 1), NULL, minor_name);
	return dev_major;
}  /* init_ptp_device */

static void exit_ptp_device(int dev_major, char *dev_name)
{
	device_destroy(ptp_class, MKDEV(dev_major, 1));
	device_destroy(ptp_class, MKDEV(dev_major, 0));
	class_destroy(ptp_class);
	unregister_chrdev(dev_major, dev_name);
}  /* exit_ptp_device */

static void ptp_check(struct ptp_info *ptp)
{
	struct ptp_utime cur;
	struct ptp_utime now;

	ptp->features |= PTP_ADJ_HACK;
	mutex_lock(ptp->hwlock);
	get_ptp_time(ptp->ptpdev, &cur);
	adjust_ptp_time(ptp->ptpdev, true, 10, 0, true);
	get_ptp_time(ptp->ptpdev, &now);
	if (now.sec - cur.sec >= 10) {
		ptp->features |= PTP_ADJ_SEC;
		ptp->features &= ~PTP_ADJ_HACK;
		adjust_ptp_time(ptp->ptpdev, false, 10, 0, true);
		ptp->version = 1;
	}
	mutex_unlock(ptp->hwlock);
}  /* ptp_check */

static void ptp_init(struct ptp_info *ptp, u8 *mac_addr)
{
	int i;

#ifdef PTP_SPI
	ptp->access = create_singlethread_workqueue("ptp_access");
#endif
	init_ptp_work(ptp);
	mutex_init(&ptp->lock);
	init_waitqueue_head(&ptp->wait_ts);
	init_waitqueue_head(&ptp->wait_intr);
	INIT_WORK(&ptp->proc_ptp_irq, proc_ptp_intr);
#ifdef PTP_MONITOR
	INIT_WORK(&ptp->ts_rx, get_rx_ts);
#ifdef PTP_PROCESS
	INIT_WORK(&ptp->exit_state, reset_state);
#endif
#endif
	INIT_WORK(&ptp->adj_clk, adj_clock);
	INIT_WORK(&ptp->set_clk, set_clock);
	INIT_WORK(&ptp->sync_clk, synchronize);
	INIT_WORK(&ptp->synt_clk, syntonize);
	memcpy(&ptp->clockIdentity.addr[0], &mac_addr[0], 3);
	ptp->clockIdentity.addr[3] = 0xFF;
	ptp->clockIdentity.addr[4] = 0xFE;
	memcpy(&ptp->clockIdentity.addr[5], &mac_addr[3], 3);

	ptp->ports = MAX_PTP_PORT;
	ptp_check(ptp);

#if 0
	verify_calc_diff();
#endif
	ptp->mode = PTP_ENABLE |
		PTP_IPV4_UDP_ENABLE |
		PTP_1STEP;
	ptp->mode |= PTP_IPV6_UDP_ENABLE;
	ptp->mode |= PTP_ETH_ENABLE;
	ptp->cfg = 0;
	ptp->cfg |= PTP_DOMAIN_CHECK;
	ptp->cfg |= PTP_PDELAY_CHECK | PTP_DELAY_CHECK;
	ptp->cfg |= PTP_UNICAST_ENABLE;
#if 0
	ptp->cfg |= PTP_ALL_HIGH_PRIORITY;
#endif
	if (ptp->version >= 1) {
		ptp->cfg |= PTP_UDP_CHECKSUM;
		ptp->cfg |= PTP_SYNC_CHECK;
	}
#ifdef VERIFY_PTP_MSG
	ptp->cfg |= PTP_UDP_CHECKSUM;
#endif
	ptp->def_mode = ptp->mode;
	ptp->def_cfg = ptp->cfg;
	ptp->trig_intr = 0xfff;
	ptp->ts_intr =
		(TS_PORT2_INT_XDELAY |
		TS_PORT2_INT_SYNC |
		TS_PORT1_INT_XDELAY |
		TS_PORT1_INT_SYNC);

	ptp->gps_tsi = MAX_TIMESTAMP_UNIT;
	ptp->gps_gpi = DEFAULT_GPS_GPI;
	ptp->pps_gpo = DEFAULT_PPS_GPO;
	ptp->pps_tsi = DEFAULT_PPS_TSI;
#ifndef ENABLE_IRIG
	ptp->pps_tso = DEFAULT_PPS_TSO;
#else
	ptp->pps_tso = 10;
#endif
	ptp->mhz_gpo = DEFAULT_MHZ_GPO;
	ptp->mhz_tso = DEFAULT_MHZ_TSO;

	for (i = 0; i < MAX_TIMESTAMP_UNIT - 1; i++)
		ptp->events[i].max = 2;
	ptp->events[i].max = MAX_TIMESTAMP_EVENT_UNIT;

#ifdef PTP_MONITOR
	ptp->rx.head = ptp->rx.tail = 0;
	ptp->e2e.mask = MAX_DELAY_REQ_CNT - 1;
#endif

	ptp_priv = ptp;
	sprintf(ptp->dev_name[0], "ptp_dev");
	sprintf(ptp->dev_name[1], "ptp_event");
	ptp->dev_major = init_ptp_device(0, ptp->dev_name[0],
		ptp->dev_name[1]);
}  /* ptp_init */

static void ptp_exit(struct ptp_info *ptp)
{
	mutex_lock(ptp->hwlock);
	ptp_write(ptp->ptpdev, ADDR_16, TRIG_INT_ENABLE, 0);
	ptp_write(ptp->ptpdev, ADDR_16, TS_INT_ENABLE, 0);
	mutex_unlock(ptp->hwlock);

#ifdef PTP_SPI
	if (ptp->access) {
		destroy_workqueue(ptp->access);
		ptp->access = NULL;
	}
#endif
	if (ptp->dev_major >= 0)
		exit_ptp_device(ptp->dev_major, ptp->dev_name[0]);
}  /* ptp_exit */

enum {
	PROC_SET_PTP_FEATURES,
	PROC_SET_PTP_OVERRIDES,
	PROC_SET_PTP_VID,
};

struct ptp_attributes {
	int features;
	int overrides;
	int vid;
};

static ssize_t sysfs_ptp_read(struct ptp_info *ptp, int proc_num, ssize_t len,
	char *buf)
{
	switch (proc_num) {
	case PROC_SET_PTP_FEATURES:
		len += sprintf(buf + len, "%08x:\n", ptp->features);
		len += sprintf(buf + len, "\t%08x = adjust hack\n",
			PTP_ADJ_HACK);
		len += sprintf(buf + len, "\t%08x = adjust sec\n",
			PTP_ADJ_SEC);
#ifdef PTP_MONITOR
		len += sprintf(buf + len, "\t%08x = PTP syntonization\n",
			PTP_SYNT);
#endif
		len += sprintf(buf + len, "\t%08x = PTP monitoring\n",
			PTP_MON);
		break;
	case PROC_SET_PTP_OVERRIDES:
		len += sprintf(buf + len, "%08x:\n", ptp->overrides);
		break;
	case PROC_SET_PTP_VID:
		len += sprintf(buf + len, "0x%04x\n", ptp->vid);
		break;
	}
	return len;
}

static void sysfs_ptp_write(struct ptp_info *ptp, int proc_num, int num,
	const char *buf)
{
	int changes;

	switch (proc_num) {
	case PROC_SET_PTP_FEATURES:
		if ('0' != buf[0] || 'x' != buf[1])
			sscanf(buf, "%x", &num);
		changes = ptp->features ^ num;
		ptp->features = num;
#ifdef PTP_PROCESS
		if ((changes & (PTP_SYNT | PTP_SIM_2_STEP))) {
#ifdef PTP_2_STEP
			if (num & PTP_SIM_2_STEP) {
				ptp->sim_2_step = true;
				ptp->mode &= ~PTP_1STEP;
			} else {
				ptp->sim_2_step = false;
				ptp->mode |= PTP_1STEP;
			}
#endif
			if (num & (PTP_SYNT | PTP_SIM_2_STEP)) {
				ptp_init_state(ptp);
				if (num & PTP_SYNT) {
					ptp->sim = 1;
					ptp->I = 0;
					ptp->KP = 50;
					ptp->KI = 5;
				}
			} else {
				ptp_exit_state(ptp);
				dbg_msg("exit ptp\n");
			}
		}
#endif
		break;
	case PROC_SET_PTP_OVERRIDES:
		if ('0' != buf[0] || 'x' != buf[1])
			sscanf(buf, "%x", &num);
		changes = ptp->overrides ^ num;
		ptp->overrides = num;
		break;
	case PROC_SET_PTP_VID:
		ptp->vid = num;
		break;
	}
}

#ifdef PTP_MONITOR
enum {
	PROC_SET_CLOCK_ID,
};

struct static_clock_attributes {
	int id;
};

static ssize_t sysfs_clock_read(struct ptp_info *ptp, int num, int index,
	ssize_t len, char *buf)
{
	struct ptp_clock_identity *entry;

	entry = &ptp->clock_id[index];
	switch (num) {
	case PROC_SET_CLOCK_ID:
		len += sprintf(buf + len,
			"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
			entry->addr[0], entry->addr[1],
			entry->addr[2], entry->addr[3],
			entry->addr[4], entry->addr[5],
			entry->addr[6], entry->addr[7]);
		break;
	}
	return len;
}

static void sysfs_clock_write(struct ptp_info *ptp, int proc_num, int index,
	int num, const char *buf)
{
	struct ptp_clock_identity *entry;
	int i;
	int n[8];

	entry = &ptp->clock_id[index];
	switch (proc_num) {
	case PROC_SET_CLOCK_ID:
		i = sscanf(buf, "%x:%x:%x:%x:%x:%x:%x:%x",
			&n[0], &n[1], &n[2], &n[3], &n[4], &n[5], &n[6], &n[7]);
		if (8 == i) {
			for (i = 0; i < 8; i++)
				entry->addr[i] = (u8) n[i];
		}
		break;
	default:
		pr_alert("write_proc:%d\n", proc_num);
	}
}

static char *clock_name[CLOCK_ENTRIES] = {
	"clock0",
	"clock1",
};

#endif

