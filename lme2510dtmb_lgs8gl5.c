// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * lme2510dtmb_lgs8gl5.c — Fork of kernel lgs8gl5.c for LME2510 DTMB driver
 *
 * Based on Legend Silicon LGS-8GL5 DTMB demodulator driver
 * (Copyright (C) 2008 Sirius International).
 *
 * Key differences from kernel lgs8gl5.c:
 *   - init: reg7=0x1C, reg9..12=0, NO alt[0xC2]=0x28
 *   - set_frontend: unlock→tune→re-lock→resync→soft_reset→wait_lock
 *   - wait_lock: 3-mode retry (modes 0/2/4, carrier 20×30ms + lock 10×20ms)
 *   - post_lock_resync: demod[7E]&=0xFE, alt[C5]=(val&0xE6)|0x06, soft_reset
 *   - chip ID check (0x0E) in attach
 *   - start_demod replaced by set_frontend inline flow
 */

#include "lme2510dtmb.h"
#include "lme2510dtmb_lgs8gl5.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <media/dvb_frontend.h>
#include <media/dvb_demux.h>

/* ── Register map ─────────────────────────────────────────────── */
#define REG_CHIP_ID     0x00
#define REG_UNLOCK      0x01
#define REG_RESET       0x02
#define REG_03          0x03
#define REG_04          0x04
#define REG_07          0x07
#define REG_09          0x09
#define REG_0A          0x0a
#define REG_0B          0x0b
#define REG_0C          0x0c
#define REG_37          0x37
#define REG_STRENGTH    0x4b
#define REG_STRENGTH_MASK       0x7f
#define REG_STRENGTH_CARRIER    0x80
#define REG_INVERSION   0x7c
#define REG_INVERSION_ON        0x80
#define REG_7D          0x7d
#define REG_7E          0x7e
#define REG_A2          0xa2
#define REG_STATUS      0xa4
#define REG_STATUS_SYNC 0x04
#define REG_STATUS_LOCK 0x01
#define ALT_REG_C5      0xc5

struct lme2510dtmb_lgs8gl5_state {
	struct i2c_adapter *i2c;
	struct lme2510dtmb_lgs8gl5_config config;  /* copy, not pointer */
	struct dvb_frontend frontend;
};

static int debug_lgs8gl5;
#define dprintk(args...) \
	do { \
		if (debug_lgs8gl5) \
			printk(KERN_DEBUG "lme2510dtmb_lgs8gl5: " args); \
	} while (0)

/* ── I2C helpers ──────────────────────────────────────────────── */

static int lme2510dtmb_lgs8gl5_write_reg(struct lme2510dtmb_lgs8gl5_state *state,
				  u8 reg, u8 data)
{
	int ret;
	u8 buf[] = { reg, data };
	struct i2c_msg msg = {
		.addr  = state->config.demod_address,
		.flags = 0,
		.buf   = buf,
		.len   = 2
	};

	ret = i2c_transfer(state->i2c, &msg, 1);
	if (ret != 1)
		dprintk("%s: error (reg=0x%02x, val=0x%02x, ret=%i)\n",
			__func__, reg, data, ret);
	return (ret != 1) ? -1 : 0;
}

static int lme2510dtmb_lgs8gl5_read_reg(struct lme2510dtmb_lgs8gl5_state *state, u8 reg)
{
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[2] = {
		{ .addr = state->config.demod_address, .flags = 0, .buf = b0, .len = 1 },
		{ .addr = state->config.demod_address, .flags = I2C_M_RD, .buf = b1, .len = 1 },
	};

	ret = i2c_transfer(state->i2c, msg, 2);
	if (ret != 2)
		return -EIO;
	return b1[0];
}

static int __maybe_unused lme2510dtmb_lgs8gl5_update_reg(struct lme2510dtmb_lgs8gl5_state *state,
				   u8 reg, u8 data)
{
	lme2510dtmb_lgs8gl5_read_reg(state, reg);
	lme2510dtmb_lgs8gl5_write_reg(state, reg, data);
	return 0;
}

static int __maybe_unused lme2510dtmb_lgs8gl5_update_alt_reg(struct lme2510dtmb_lgs8gl5_state *state,
				       u8 reg, u8 data)
{
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	u8 b2[] = { reg, data };
	struct i2c_msg msg[3] = {
		{ .addr = state->config.alt_address, .flags = 0, .buf = b0, .len = 1 },
		{ .addr = state->config.alt_address, .flags = I2C_M_RD, .buf = b1, .len = 1 },
		{ .addr = state->config.alt_address, .flags = 0, .buf = b2, .len = 2 },
	};

	ret = i2c_transfer(state->i2c, msg, 3);
	return (ret != 3) ? -1 : 0;
}

static int lme2510dtmb_lgs8gl5_write_alt_reg(struct lme2510dtmb_lgs8gl5_state *state,
				      u8 reg, u8 data)
{
	int ret;
	u8 buf[] = { reg, data };
	struct i2c_msg msg = {
		.addr  = state->config.alt_address,
		.flags = 0,
		.buf   = buf,
		.len   = 2
	};

	ret = i2c_transfer(state->i2c, &msg, 1);
	return (ret != 1) ? -1 : 0;
}

static int lme2510dtmb_lgs8gl5_read_alt_reg(struct lme2510dtmb_lgs8gl5_state *state, u8 reg)
{
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[2] = {
		{ .addr = state->config.alt_address, .flags = 0, .buf = b0, .len = 1 },
		{ .addr = state->config.alt_address, .flags = I2C_M_RD, .buf = b1, .len = 1 },
	};

	ret = i2c_transfer(state->i2c, msg, 2);
	if (ret != 2)
		return -EIO;
	return b1[0];
}

/* ── Soft reset ───────────────────────────────────────────────── */

static void lme2510dtmb_lgs8gl5_soft_reset(struct lme2510dtmb_lgs8gl5_state *state)
{
	u8 val;

	if (!state)
		return;

	dprintk("%s\n", __func__);

	val = lme2510dtmb_lgs8gl5_read_reg(state, REG_RESET);
	if (val < 0) {
		dprintk("%s: read reset failed\n", __func__);
		return;
	}
	lme2510dtmb_lgs8gl5_write_reg(state, REG_RESET, val & ~0x01);
	lme2510dtmb_lgs8gl5_write_reg(state, REG_RESET, val | 0x01);
	msleep(5);
}

/* ── Init ──────────────────────────────────────────────────────── */

/*
 * lme2510dtmb_lgs8gl5_init — full demod init (lme2510dtmb variant).
 *
 * Diff from kernel lgs8gl5_init:
 *   - reg7 = 0x1C (|= 0x0C then &= 0x7C), not 0x10.
 *   - reg9/0A/0B/0C = 0x00, not 0x0E/0xE5/0x35/0x30.
 *   - NO alt[0xC2] = 0x28.  (Deleted.)
 */
static int lme2510dtmb_lgs8gl5_init(struct dvb_frontend *fe)
{
	struct lme2510dtmb_lgs8gl5_state *state = fe->demodulator_priv;
	u8 r7;

	if (!state) {
		pr_err("lme2510dtmb_lgs8gl5: init called with null state\n");
		return -EINVAL;
	}

	dprintk("%s\n", __func__);

	/* Init property cache with valid defaults so userspace
	 * (tvheadend etc.) doesn't try frequency 0 on first open. */
	fe->dtv_property_cache.frequency = 474000000;
	fe->dtv_property_cache.bandwidth_hz = 8000000;
	fe->dtv_property_cache.delivery_system = SYS_DVBT;
	fe->dtv_property_cache.modulation = QAM_64;
	fe->dtv_property_cache.inversion = INVERSION_AUTO;

	/* reg7: |= 0x0C then &= 0x7C  → net reg7 = 0x1C */
	r7 = lme2510dtmb_lgs8gl5_read_reg(state, REG_07);
	r7 |= 0x0C;
	lme2510dtmb_lgs8gl5_write_reg(state, REG_07, r7);
	r7 &= 0x7C;
	lme2510dtmb_lgs8gl5_write_reg(state, REG_07, r7);

	/* reg9..12 = 0 */
	lme2510dtmb_lgs8gl5_write_reg(state, REG_09, 0x00);
	lme2510dtmb_lgs8gl5_write_reg(state, REG_0A, 0x00);
	lme2510dtmb_lgs8gl5_write_reg(state, REG_0B, 0x00);
	lme2510dtmb_lgs8gl5_write_reg(state, REG_0C, 0x00);

	return 0;
}

/* ── Post-lock resync ──────────────────────────────────────────── */

/*
 * lme2510dtmb_lgs8gl5_post_lock_resync — FUN_000163E0 from UDE262D.sys.
 *
 * Steps:
 *   demod[0x7E] &= 0xFE
 *   alt[0xC5] = (val & 0xE6) | 0x06
 *   soft_reset
 */
static int lme2510dtmb_lgs8gl5_post_lock_resync(struct lme2510dtmb_lgs8gl5_state *state)
{
	u8 val;

	/* demod[0x7E] &= 0xFE */
	val = lme2510dtmb_lgs8gl5_read_reg(state, REG_7E);
	lme2510dtmb_lgs8gl5_write_reg(state, REG_7E, val & 0xFE);

	/* alt[0xC5] = (val & 0xE6) | 0x06 */
	val = lme2510dtmb_lgs8gl5_read_alt_reg(state, ALT_REG_C5);
	lme2510dtmb_lgs8gl5_write_alt_reg(state, ALT_REG_C5, (val & 0xE6) | 0x06);

	lme2510dtmb_lgs8gl5_soft_reset(state);
	return 0;
}

/* ── Demod resync ──────────────────────────────────────────────── */

/*
 * lme2510dtmb_lgs8gl5_demod_resync — pre-lock resync (FUN_00016500).
 *
 * Steps before wait_lock:
 *   demod[3]    &= 0xFE     (clear bit0)
 *   demod[0x7E] |= 0x01     (set bit0)
 *   alt[0xC5]   &= 0xE0     (clear bits 0..4)
 */
static int lme2510dtmb_lgs8gl5_demod_resync(struct lme2510dtmb_lgs8gl5_state *state)
{
	u8 val;

	val = lme2510dtmb_lgs8gl5_read_reg(state, REG_03);
	lme2510dtmb_lgs8gl5_write_reg(state, REG_03, val & 0xFE);

	val = lme2510dtmb_lgs8gl5_read_reg(state, REG_7E);
	lme2510dtmb_lgs8gl5_write_reg(state, REG_7E, val | 0x01);

	val = lme2510dtmb_lgs8gl5_read_alt_reg(state, ALT_REG_C5);
	lme2510dtmb_lgs8gl5_write_alt_reg(state, ALT_REG_C5, val & 0xE0);

	return 0;
}

/* ── Wait lock ─────────────────────────────────────────────────── */

/*
 * lme2510dtmb_lgs8gl5_wait_lock — FUN_00016860 from UDE262D.sys.
 *
 * 3 effective modes (0, 2, 4; skip 1 and 3), 3 retries per mode.
 *   Mode 0: reg4 bit1, reg37 bit7 clear
 *   Mode 2: reg37 bit7 clear only
 *   Mode 4: reg4 bit0, reg37 bit7 set
 *
 * Per mode:
 *   - write reg4/reg37 mask → soft_reset
 *   - Carrier poll: 20 × 30ms × read demod[0x4B] bit7
 *   - Lock poll:    10 × 20ms × read demod[0xA4] bit[1:0]==0x01
 *
 * Max time ≈ 3 × 3 × (600 + 200)ms ≈ 7.2 s
 */
static int lme2510dtmb_lgs8gl5_wait_lock(struct lme2510dtmb_lgs8gl5_state *state)
{
	int outer_idx;
	int retry = 0;

	dprintk("%s\n", __func__);

	for (outer_idx = 0; outer_idx < 5; outer_idx++) {
		int mod = outer_idx % 5;
		u8 r4, r37;
		int carrier_ok = 0;
		int ci, li;

		/* Skip modes 1 and 3 — only use 0, 2, 4 */
		if (mod == 1 || mod == 3)
			continue;

		/* Read and modify reg4, reg37 per mode */
		r4  = lme2510dtmb_lgs8gl5_read_reg(state, REG_04);
		r37 = lme2510dtmb_lgs8gl5_read_reg(state, REG_37);

		r4 &= 0xFC;
		switch (mod) {
		case 0: r37 &= 0x7F; r4 |= 0x02; break;  /* clear bit7, set bit1 */
		case 2: r37 &= 0x7F;           break;  /* clear bit7 only */
		case 4: r37 |= 0x80; r4 |= 0x01; break;  /* set bit7, set bit0 */
		}

		lme2510dtmb_lgs8gl5_write_reg(state, REG_04, r4);
		lme2510dtmb_lgs8gl5_write_reg(state, REG_37, r37);
		lme2510dtmb_lgs8gl5_soft_reset(state);

		/* Carrier poll: 20 × 30ms */
		for (ci = 0; ci < 20; ci++) {
			int v4b;
			msleep(30);
			v4b = lme2510dtmb_lgs8gl5_read_reg(state, REG_STRENGTH);
			if (v4b < 0)
				return v4b;
			if (v4b & REG_STRENGTH_CARRIER) {
				carrier_ok = 1;
				break;
			}
		}

		if (!carrier_ok) {
			/* Toggle reg 0x7C bit7 and retry */
			int v7c = lme2510dtmb_lgs8gl5_read_reg(state, REG_INVERSION);
			lme2510dtmb_lgs8gl5_write_reg(state, REG_INVERSION,
				(v7c & 0x80) ? (v7c & 0x7F) : (v7c | 0x80));
			if (++retry > 2)
				retry = 0;
			else
				outer_idx--;
			continue;
		}

		/* Lock poll: 10 × 20ms */
		for (li = 0; li < 10; li++) {
			int va4;
			msleep(20);
			va4 = lme2510dtmb_lgs8gl5_read_reg(state, REG_STATUS);
			if (va4 < 0)
				return va4;
			if ((va4 & 0x03) == 0x01) {
				/* Locked! b[1:0] = 01 */
				int a2 = lme2510dtmb_lgs8gl5_read_reg(state, REG_A2);
				if (a2 >= 0) {
					lme2510dtmb_lgs8gl5_post_lock_resync(state);
					lme2510dtmb_lgs8gl5_write_reg(state, REG_7D, (u8)a2);
				}
				dprintk("%s: LOCKED (mode=%d, carrier=%d, lock=%d)\n",
					__func__, mod, ci, li);
				return 0;
			}
		}

		/* No lock this mode — toggle reg7C, retry */
		{
			int v7c = lme2510dtmb_lgs8gl5_read_reg(state, REG_INVERSION);
			lme2510dtmb_lgs8gl5_write_reg(state, REG_INVERSION,
				(v7c & 0x80) ? (v7c & 0x7F) : (v7c | 0x80));
		}
		if (++retry > 2)
			retry = 0;
		else
			outer_idx--;
	}

	dprintk("%s: LOCK failed\n", __func__);
	return -ETIMEDOUT;
}

/* ── set_frontend ─────────────────────────────────────────────── */

/*
 * lme2510dtmb_lgs8gl5_set_frontend — tune to a new frequency.
 *
 * Diff from kernel:
 *   - Unlock demod[1]=0xE0 before tuner set_params
 *   - Re-lock demod[1]=0x60 after
 *   - Demod resync + soft_reset + wait_lock (3-mode retry)
 */
static int lme2510dtmb_lgs8gl5_set_frontend(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct lme2510dtmb_lgs8gl5_state *state = fe->demodulator_priv;
	struct dvb_usb_device *d = fe_to_d(fe);
	struct lme2510dtmb_state *st = d->priv;
	int ret;

	dprintk("%s\n", __func__);

	if (p->bandwidth_hz != 8000000)
		return -EINVAL;

	/* Allow I2C during demod re-init */
	st->stream_active = false;

	/* 1. Soft reset */
	lme2510dtmb_lgs8gl5_soft_reset(state);

	/* 2. Unlock demod[1] = 0xE0 */
	lme2510dtmb_lgs8gl5_write_reg(state, REG_UNLOCK, 0xE0);

	/* 3. Tune via tuner ops (MAX2165 set_params) */
	if (fe->ops.tuner_ops.set_params) {
		fe->ops.tuner_ops.set_params(fe);
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);
	}

	/* 4. Re-lock demod[1] = 0x60 */
	lme2510dtmb_lgs8gl5_write_reg(state, REG_UNLOCK, 0x60);

	/* 5. Demod resync */
	lme2510dtmb_lgs8gl5_demod_resync(state);

	/* 6. Soft reset again */
	lme2510dtmb_lgs8gl5_soft_reset(state);

	/* 7. Wait for lock */
	ret = lme2510dtmb_lgs8gl5_wait_lock(state);

	if (ret == 0) {
		struct dvb_usb_adapter *adap = fe_to_adap(fe);
		struct dvb_demux_feed *feed;
		int feed_count = 0;

		/* Re-start stream after demod re-init */
		if (st->streaming)
			lme2510dtmb_mcu_stream_restart(d);
		st->stream_active = true;

		list_for_each_entry(feed, &adap->demux.feed_list, list_head) {
			feed_count++;
			dev_dbg(&d->udev->dev,
				 "lme2510dtmb: post_start feed[%d] pid=0x%04x type=%d state=%d\n",
				 feed_count, feed->pid, feed->type,
				 feed->state);
		}
		dev_dbg(&d->udev->dev,
			 "lme2510dtmb: post_start feeds=%d streaming=%d\n",
			 feed_count, st->streaming);
	}

	return ret;
}

/* ── get_frontend ──────────────────────────────────────────────── */

static int lme2510dtmb_lgs8gl5_get_frontend(struct dvb_frontend *fe,
				     struct dtv_frontend_properties *p)
{
	struct dvb_usb_device *d = fe_to_d(fe);
	struct lme2510dtmb_state *st = d->priv;

	p->bandwidth_hz = 8000000;

	if (st->stream_active) {
		dev_dbg(&d->udev->dev, "lme2510dtmb: get_frontend: cached\n");
		return 0;
	}

	{
		struct lme2510dtmb_lgs8gl5_state *state = fe->demodulator_priv;
		int inv = lme2510dtmb_lgs8gl5_read_reg(state, REG_INVERSION);

		if (inv >= 0)
			p->inversion = (inv & REG_INVERSION_ON) ?
				       INVERSION_ON : INVERSION_OFF;
	}

	p->code_rate_HP = FEC_1_2;
	p->code_rate_LP = FEC_7_8;
	p->guard_interval = GUARD_INTERVAL_1_32;
	p->transmission_mode = TRANSMISSION_MODE_2K;
	p->modulation = QAM_64;
	p->hierarchy = HIERARCHY_NONE;

	return 0;
}

/* ── get_tune_settings ─────────────────────────────────────────── */

static int lme2510dtmb_lgs8gl5_get_tune_settings(struct dvb_frontend *fe,
					  struct dvb_frontend_tune_settings *fesettings)
{
	fesettings->min_delay_ms = 240;
	fesettings->step_size    = 0;
	fesettings->max_drift    = 0;
	return 0;
}

/* ── Read status / signal / snr (I2C fallback) ─────────────────── */

/*
 * These are kept as fallback.  The USB bridge layer (lme2510dtmb_status.c)
 * overrides these ops with EP8A interrupt-based implementations
 * during frontend_attach.
 */

static int lme2510dtmb_lgs8gl5_read_status(struct dvb_frontend *fe,
				    enum fe_status *status)
{
	struct lme2510dtmb_lgs8gl5_state *state = fe->demodulator_priv;
	struct dvb_usb_device *d = fe_to_d(fe);
	struct lme2510dtmb_state *st = d->priv;
	int level, flags;

	/* During TS streaming, avoid ALL EP1 I2C communication.
	 * MCU's USB-I2C bridge shares the bus with EP88 TS output;
	 * any I2C read during streaming causes MCU to stall TS. */
	if (st->stream_active) {
		dev_dbg(&d->udev->dev, "lme2510dtmb: read_status: cached (stream_active)\n");
		*status = FE_HAS_LOCK | FE_HAS_SIGNAL | FE_HAS_CARRIER |
			  FE_HAS_SYNC | FE_HAS_VITERBI;
		return 0;
	}

	level = lme2510dtmb_lgs8gl5_read_reg(state, REG_STRENGTH);
	flags = lme2510dtmb_lgs8gl5_read_reg(state, REG_STATUS);

	if (level < 0 || flags < 0)
		return -EIO;

	*status = 0;

	if ((level & REG_STRENGTH_MASK) > 0)
		*status |= FE_HAS_SIGNAL;
	if (level & REG_STRENGTH_CARRIER)
		*status |= FE_HAS_CARRIER;
	if (flags & REG_STATUS_SYNC)
		*status |= FE_HAS_SYNC;
	if (flags & REG_STATUS_LOCK)
		*status |= FE_HAS_LOCK;

	return 0;
}

static int lme2510dtmb_lgs8gl5_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	*ber = 0;
	return 0;
}

static int lme2510dtmb_lgs8gl5_read_signal_strength(struct dvb_frontend *fe,
					     u16 *signal_strength)
{
	struct dvb_usb_device *d = fe_to_d(fe);
	struct lme2510dtmb_state *st = d->priv;

	if (st->stream_active) {
		dev_dbg(&d->udev->dev, "lme2510dtmb: read_signal_strength: cached\n");
		*signal_strength = 0;
		return 0;
	}

	{
		struct lme2510dtmb_lgs8gl5_state *state = fe->demodulator_priv;
		int level = lme2510dtmb_lgs8gl5_read_reg(state, REG_STRENGTH);

		if (level < 0)
			return level;
		*signal_strength = (level & REG_STRENGTH_MASK) << 8;
	}
	return 0;
}

static int lme2510dtmb_lgs8gl5_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dvb_usb_device *d = fe_to_d(fe);
	struct lme2510dtmb_state *st = d->priv;

	if (st->stream_active) {
		dev_dbg(&d->udev->dev, "lme2510dtmb: read_snr: cached\n");
		*snr = 0;
		return 0;
	}

	{
		struct lme2510dtmb_lgs8gl5_state *state = fe->demodulator_priv;
		int level = lme2510dtmb_lgs8gl5_read_reg(state, REG_STRENGTH);

		if (level < 0)
			return level;
		*snr = (level & REG_STRENGTH_MASK) << 8;
	}
	return 0;
}

static int lme2510dtmb_lgs8gl5_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

/* ── Release ───────────────────────────────────────────────────── */

static void lme2510dtmb_lgs8gl5_release(struct dvb_frontend *fe)
{
	struct lme2510dtmb_lgs8gl5_state *state = fe->demodulator_priv;
	kfree(state);
}

/* ── dvb_frontend_ops ──────────────────────────────────────────── */

static const struct dvb_frontend_ops lme2510dtmb_lgs8gl5_ops;

/* ── attach ────────────────────────────────────────────────────── */

/*
 * lme2510dtmb_lgs8gl5_attach — probe and create LGS8GL5 frontend.
 *
 * Diff from kernel lgs8gl5_attach:
 *   - Reads chip ID from demod[0x00], expects 0x0E (LGS8GL5).
 *   - Returns NULL if chip ID doesn't match.
 */
struct dvb_frontend *lme2510dtmb_lgs8gl5_attach(
	const struct lme2510dtmb_lgs8gl5_config *config,
	struct i2c_adapter *i2c)
{
	struct lme2510dtmb_lgs8gl5_state *state = NULL;
	int chip_id;

	dprintk("%s\n", __func__);

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state == NULL)
		goto error;

	state->config = *config;
	state->i2c    = i2c;

	/* Check if demod is there and chip ID = 0x0E */
	chip_id = lme2510dtmb_lgs8gl5_read_reg(state, REG_CHIP_ID);
	if (chip_id < 0)
		goto error;

	dprintk("%s: chip_id = 0x%02X\n", __func__, chip_id);

	if (chip_id != 0x0E) {
		printk(KERN_ERR "lme2510dtmb_lgs8gl5: unexpected chip ID 0x%02X "
		       "(expected 0x0E)\n", chip_id);
		goto error;
	}

	/* Create dvb_frontend */
	memcpy(&state->frontend.ops, &lme2510dtmb_lgs8gl5_ops,
	       sizeof(struct dvb_frontend_ops));
	state->frontend.demodulator_priv = state;

	printk(KERN_INFO "lme2510dtmb_lgs8gl5: LGS-8GL5 DTMB demod attached\n");
	return &state->frontend;

error:
	kfree(state);
	return NULL;
}
EXPORT_SYMBOL(lme2510dtmb_lgs8gl5_attach);

/* ── dvb_frontend_ops instance ─────────────────────────────────── */

static const struct dvb_frontend_ops lme2510dtmb_lgs8gl5_ops = {
	.delsys = { SYS_DVBT },
	.info = {
		.name            = "LME2510DTMB LGS-8GL5 DTMB",
		.frequency_min_hz  = 474 * MHz,
		.frequency_max_hz  = 858 * MHz,
		.frequency_stepsize_hz = 10 * kHz,
		.caps = FE_CAN_FEC_AUTO |
			FE_CAN_QPSK | FE_CAN_QAM_16 | FE_CAN_QAM_32 |
			FE_CAN_QAM_64 | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_BANDWIDTH_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO |
			FE_CAN_RECOVER
	},

	.release = lme2510dtmb_lgs8gl5_release,

	.init = lme2510dtmb_lgs8gl5_init,

	.set_frontend = lme2510dtmb_lgs8gl5_set_frontend,
	.get_frontend = lme2510dtmb_lgs8gl5_get_frontend,
	.get_tune_settings = lme2510dtmb_lgs8gl5_get_tune_settings,

	.read_status = lme2510dtmb_lgs8gl5_read_status,
	.read_ber = lme2510dtmb_lgs8gl5_read_ber,
	.read_signal_strength = lme2510dtmb_lgs8gl5_read_signal_strength,
	.read_snr = lme2510dtmb_lgs8gl5_read_snr,
	.read_ucblocks = lme2510dtmb_lgs8gl5_read_ucblocks,
};

module_param(debug_lgs8gl5, int, 0644);
MODULE_PARM_DESC(debug_lgs8gl5, "Turn on/off frontend debugging (default:off).");

MODULE_DESCRIPTION("LME2510DTMB LGS-8GL5 DTMB Demodulator driver");
MODULE_AUTHOR("LME2510DTMB project");
MODULE_LICENSE("GPL");
