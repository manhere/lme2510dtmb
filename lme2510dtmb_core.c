// SPDX-License-Identifier: GPL-2.0-only
/*
 * lme2510dtmb.c �?LME2510 DTMB USB dongle Linux DVB driver (main module)
 *
 * Hardware: LME2511 + LGS8GL5 + MAX2165
 *
 * USB VID/PID: 0x3344/0x1120  (same as lmedm04 �?conflicts!)
 * Must blacklist dvb_usb_lmedm04.
 *
 * Architecture:
 *   - dvb-usb-v2 framework (dvb_usbv2_probe / dvb_usbv2_disconnect)
 *   - EP1 Bulk I2C bridge  (lme2510dtmb_i2c.c)
 *   - MCU commands          (lme2510dtmb_mcu.c)
 *   - Firmware upload       (lme2510dtmb_firmware.c)
 *   - LGS8GL5 demod fork    (lme2510dtmb_lgs8gl5.c)
 *   - MAX2165 tuner         (lme2510dtmb_max2165.c)
 *   - EP8A status polling   (lme2510dtmb_status.c)
 */

#include "lme2510dtmb.h"
#include "lme2510dtmb_lgs8gl5.h"
#include "lme2510dtmb_max2165.h"

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/delay.h>
#include <media/dvb_demux.h>
#include "dvb_usb.h"

/* ── Module params ─────────────────────────────────────────────── */

/*
 * EP8A interrupt status polling.  Default on (matches the proven
 * C++ user-space implementation).  Set ep8a_status=0 to keep the
 * I2C-based status fallbacks only.
 */
static bool ep8a_status = true;
module_param(ep8a_status, bool, 0444);
MODULE_PARM_DESC(ep8a_status,
		 "enable EP8A interrupt status polling (default: true)");

/* ── Power control ─────────────────────────────────────────────── */

/*
 * Clear a possibly-stalled bulk endpoint after firmware (re)enumeration
 * (ported from HiSilicon lme2510-dtmb.c prepare_runtime).  Some runtime
 * firmware states reject CLEAR_FEATURE while still accepting commands,
 * so tolerate -EPIPE.
 */
static int lme2510dtmb_clear_halt(struct dvb_usb_device *d, unsigned int pipe)
{
	int ret = usb_clear_halt(d->udev, pipe);

	return ret == -EPIPE ? 0 : ret;
}

int lme2510dtmb_power_ctrl(struct dvb_usb_device *d, int onoff)
{
	unsigned int pipe;
	int ret;

	if (!onoff)
		return 0;

	ret = usb_set_interface(d->udev,
				d->props->bInterfaceNumber, 1);
	if (ret)
		return ret;
	msleep(200);

	/* Command OUT / Command IN / TS IN — clear stale halt state on
	 * all three bulk pipes so the first transfer cannot fail with
	 * -EPIPE left over from the firmware upload stage. */
	pipe = usb_sndbulkpipe(d->udev, EP_BULK_OUT);
	ret = lme2510dtmb_clear_halt(d, pipe);
	if (ret)
		return ret;
	pipe = usb_rcvbulkpipe(d->udev, EP_BULK_IN);
	ret = lme2510dtmb_clear_halt(d, pipe);
	if (ret)
		return ret;
	pipe = usb_rcvbulkpipe(d->udev, EP_TS_IN);
	return lme2510dtmb_clear_halt(d, pipe);
}

/* ── Frontend attach ───────────────────────────────────────────── */

int lme2510dtmb_frontend_attach(struct dvb_usb_adapter *adap)
{
	struct dvb_usb_device *d = adap_to_d(adap);
	const struct lme2510dtmb_lgs8gl5_config demod_cfg = {
		.demod_address = I2C_ADDR_DEMOD_PRIMARY,
		.alt_address   = I2C_ADDR_DEMOD_ALT,
	};
	const struct lme2510dtmb_max2165_config tuner_cfg = {
		.i2c_address      = I2C_ADDR_TUNER,
		.osc_clk_khz      = 12000,
		.default_freq_khz = 474000,
	};
	u8 chip_type = 0x00;  /* MCU CMD 0x16 variant: 0 = LGS8GL5, 1 = LGS8G75 */
	int ret;

	dev_dbg(&d->udev->dev, "%s\n", __func__);
	dev_info(&d->udev->dev, "BUILD_ID " BUILD_ID "\n");

	/* ── Chip detect sequence (matching FUN_00015650 LGS8GL5 branch) ── */

	/* 1. Read chip ID from demod[0x00]: 0x0E = LGS8GL5, else LGS8G75 */

	adap->fe[0] = lme2510dtmb_lgs8gl5_attach(&demod_cfg,
				 &d->i2c_adap, &chip_type);
	if (!adap->fe[0]) {
		dev_err(&d->udev->dev, "DTMB demod not found\n");
		return -ENODEV;
	}

	/* 2. MCU demod variant select {16 01 <type>} */
	ret = lme2510dtmb_mcu_demod_variant(d, chip_type);
	if (ret < 0) {
		dev_err(&d->udev->dev, "MCU demod variant select failed\n");
		dvb_frontend_detach(adap->fe[0]);
		adap->fe[0] = NULL;
		return ret;
	}

	/* 3. Demod soft reset: read reg 0x02, clear bit0, set bit0 */
	{
		u8 val;
		u8 wr_buf[2], rd_buf[1];
		struct i2c_msg msgs[2] = {
			{ .addr = I2C_ADDR_DEMOD_PRIMARY,
			  .flags = 0, .buf = wr_buf, .len = 1 },
			{ .addr = I2C_ADDR_DEMOD_PRIMARY,
			  .flags = I2C_M_RD, .buf = rd_buf, .len = 1 },
		};

		/* Combined write-then-read: reg 0x02 → val */
		wr_buf[0] = 0x02;
		if (i2c_transfer(&d->i2c_adap, msgs, 2) != 2)
			goto demod_fail;
		val = rd_buf[0];

		/* Write: reg 0x02 = val & ~0x01 (clear bit0) */
		wr_buf[0] = 0x02; wr_buf[1] = val & 0xFE;
		msgs[0].len = 2;
		i2c_transfer(&d->i2c_adap, msgs, 1);

		/* Write: reg 0x02 = (val & ~0x01) | 0x01 (set bit0) */
		wr_buf[0] = 0x02; wr_buf[1] = (val & 0xFE) | 0x01;
		i2c_transfer(&d->i2c_adap, msgs, 1);
		msleep(5);
	}

	/* 4. Unlock demod[1] = 0xE0 */
	{
		u8 buf[] = { 0x01, 0xE0 };
		struct i2c_msg msg = { .addr = I2C_ADDR_DEMOD_PRIMARY,
				       .flags = 0, .buf = buf, .len = 2 };
		i2c_transfer(&d->i2c_adap, &msg, 1);
	}

	/* 5. Read MAX2165 ROM calibration */
	msleep(50);
	ret = lme2510dtmb_max2165_read_rom(d);
	if (ret < 0) {
		dev_err(&d->udev->dev, "MAX2165 ROM read failed\n");
		goto demod_fail;
	}

	/* 6. Tuner full init (16-byte block, 474 MHz) */
	ret = lme2510dtmb_max2165_init_tuner(d);
	if (ret < 0) {
		dev_err(&d->udev->dev, "MAX2165 init failed\n");
		goto demod_fail;
	}

	/* 7. Re-lock demod[1] = 0x60 */
	{
		u8 buf[] = { 0x01, 0x60 };
		struct i2c_msg msg = { .addr = I2C_ADDR_DEMOD_PRIMARY,
				       .flags = 0, .buf = buf, .len = 2 };
		i2c_transfer(&d->i2c_adap, &msg, 1);
	}

	/* 8. Attach tuner (registers set_params callback) */

	if (!lme2510dtmb_max2165_attach(adap->fe[0], &d->i2c_adap, &tuner_cfg)) {
		dev_err(&d->udev->dev, "MAX2165 attach failed\n");
		goto demod_fail;
	}

	/* 9. Register I2C gate control (no-op — EP1 mutex in dvb-usb-v2) */
	adap->fe[0]->ops.i2c_gate_ctrl = lme2510dtmb_i2c_gate_ctrl;

	/* 10. Demod init (fe->ops.init) is deferred to frontend open.
	 *     The framework's dvb_usb_fe_init wrapper calls power_ctrl(1)
	 *     then our lme2510dtmb_lgs8gl5_init — no need to call it here. */

	/* 11. EP8A interrupt status (module param ep8a_status, default on).
	 *     C++ user-space implementation proves EP8A coexists with EP88
	 *     TS streaming on this firmware; disable via ep8a_status=0 if
	 *     a given MCU batch turns out to conflict (dybd author's claim). */
	if (ep8a_status) {
		ret = lme2510dtmb_status_init(adap);
		if (ret < 0)
			dev_warn(&d->udev->dev,
				 "EP8A status init failed (%d), "
				 "keeping I2C fallback\n", ret);
	}

	dev_info(&d->udev->dev, "LME2510 DTMB frontend ready\n");
	return 0;

demod_fail:
	if (adap->fe[0]) {
		dvb_frontend_detach(adap->fe[0]);
		adap->fe[0] = NULL;
	}
	return -ENODEV;
}

/* ── I2C gate control (no-op) ──────────────────────────────────── */

int lme2510dtmb_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	/* All I2C operations share EP1 and are serialized by
	 * dvb-usb-v2's usb_mutex.  No additional gate needed. */
	return 0;
}

/* ── Streaming control ─────────────────────────────────────────── */

int lme2510dtmb_streaming_ctrl(struct dvb_frontend *fe, int onoff)
{
	struct dvb_usb_device *d = fe_to_d(fe);
	struct lme2510dtmb_state *st = d->priv;

	if (onoff) {
		struct dvb_usb_adapter *adap = fe_to_adap(fe);
		struct dvb_demux_feed *feed;
		int feed_count = 0;
		int n_pids = 0;

		st->streaming = true;

		/*
		 * EP1 window: the MCU TS engine is not yet running, this
		 * is the only safe place for EP1 traffic before streaming.
		 * With force_pid_filter_usage=1 the framework collects
		 * PIDs into st->pid_table via pid_filter() and we commit
		 * them here; otherwise all-pass (default, software demux
		 * does the filtering — zero mid-stream EP1 traffic).
		 */
		if (st->filter_enabled) {
			int k;
			for (k = 0; k < 16; k++)
				if (st->pid_table[k] != 0xffff)
					n_pids++;
		}
		if (st->filter_enabled && n_pids > 0) {
			u16 pids[16];
			int k, j = 0;
			for (k = 0; k < 16; k++)
				if (st->pid_table[k] != 0xffff)
					pids[j++] = st->pid_table[k];
			lme2510dtmb_mcu_commit_pid_filter(d, pids, j, 2);
		} else {
			lme2510dtmb_mcu_set_pid_filter(d);
		}
		lme2510dtmb_mcu_stream_restart(d);
		st->stream_active = true;

		list_for_each_entry(feed, &adap->demux.feed_list, list_head) {
			feed_count++;
			dev_dbg(&d->udev->dev,
				 "lme2510dtmb: feed[%d] pid=0x%04x type=%d state=%d\n",
				 feed_count, feed->pid, feed->type, feed->state);
		}
		dev_dbg(&d->udev->dev,
			 "lme2510dtmb: streaming_ctrl ON  feeds=%d streaming=%d\n",
			 feed_count, st->streaming);

		return 0;
	} else {
		st->streaming = false;
		st->stream_active = false;
		return 0;
	}
}

/* ── Stream config ─────────────────────────────────────────────── */

int lme2510dtmb_get_stream_config(struct dvb_frontend *fe, u8 *ts_type,
			   struct usb_data_stream_properties *stream)
{
	*ts_type = DVB_USB_FE_TS_TYPE_188;
	stream->endpoint = 0x88;
	return 0;
}

/* ── Exit ──────────────────────────────────────────────────────── */

void lme2510dtmb_exit(struct dvb_usb_device *d)
{
	struct lme2510dtmb_state *st = d->priv;

	if (!st)
		return;

	if (st->int_urb) {
		usb_kill_urb(st->int_urb);
		usb_free_urb(st->int_urb);
		st->int_urb = NULL;
		dev_dbg(&d->udev->dev, "EP8A interrupt URB stopped\n");
	}
}

/* ── dvb_usb_device_properties ─────────────────────────────────── */

static short adapter_nr[] = { 0, -1 };

/*
 * HW PID filter hooks — only invoked when the framework's
 * dvb_usb_force_pid_filter_usage=1 module option is set (or on
 * USB1.1 ports).  They only maintain an in-memory table; the actual
 * CMD03 commit happens in streaming_ctrl(ON), inside the pre-stream
 * EP1 window, to honor the MCU I2C/TS bus conflict constraint.
 */
static int lme2510dtmb_pid_filter_ctrl(struct dvb_usb_adapter *adap,
				       int onoff)
{
	struct lme2510dtmb_state *st = adap_to_d(adap)->priv;
	int k;

	if (onoff) {
		for (k = 0; k < 16; k++)
			st->pid_table[k] = 0xffff;
	}
	st->filter_enabled = onoff;
	return 0;
}

static int lme2510dtmb_pid_filter(struct dvb_usb_adapter *adap,
				  int index, u16 pid, int onoff)
{
	struct lme2510dtmb_state *st = adap_to_d(adap)->priv;
	int k;

	if (!onoff) {
		for (k = 0; k < 16; k++)
			if (st->pid_table[k] == pid)
				st->pid_table[k] = 0xffff;
		return 0;
	}

	for (k = 0; k < 16; k++)
		if (st->pid_table[k] == pid)
			return 0;

	for (k = 0; k < 16; k++) {
		if (st->pid_table[k] == 0xffff) {
			st->pid_table[k] = pid;
			return 0;
		}
	}

	dev_warn(&adap_to_d(adap)->udev->dev,
		 "%s: pid table full, falling back to all-pass\n",
		 __func__);
	st->filter_enabled = false;
	return 0;
}

static struct dvb_usb_device_properties lme2510dtmb_props = {
	.driver_name    = "lme2510dtmb",
	.owner          = THIS_MODULE,
	.adapter_nr     = adapter_nr,
	.bInterfaceNumber = 0,
	.size_of_priv   = sizeof(struct lme2510dtmb_state),

	.identify_state     = lme2510dtmb_identify_state,
	.download_firmware  = lme2510dtmb_download_firmware,

		.generic_bulk_ctrl_endpoint = 0x01,
		.generic_bulk_ctrl_endpoint_response = 0x01,

	.power_ctrl         = lme2510dtmb_power_ctrl,
	.i2c_algo           = &lme2510dtmb_i2c_algo,

	.frontend_attach    = lme2510dtmb_frontend_attach,
	.tuner_attach       = NULL,   /* done in frontend_attach */

	.streaming_ctrl     = lme2510dtmb_streaming_ctrl,
	.get_stream_config  = lme2510dtmb_get_stream_config,

	.exit               = lme2510dtmb_exit,

	.num_adapters = 1,
	.adapter = {
		{
			.caps = DVB_USB_ADAP_HAS_PID_FILTER |
				DVB_USB_ADAP_PID_FILTER_CAN_BE_TURNED_OFF,
			.pid_filter_count = 16,
			.pid_filter_ctrl  = lme2510dtmb_pid_filter_ctrl,
			.pid_filter       = lme2510dtmb_pid_filter,
			.stream =
			DVB_USB_STREAM_BULK(0x88, 10, 65536),
		},
	},
};

/* ── USB device ID table ───────────────────────────────────────── */

static const struct usb_device_id lme2510dtmb_id_table[] = {
	{
		DVB_USB_DEVICE(LME2510DTMB_VID, LME2510DTMB_PID_COLD,
			       &lme2510dtmb_props,
			       "LME2510 DTMB (cold)", RC_MAP_EMPTY)
	},
	{
		DVB_USB_DEVICE(LME2510DTMB_VID, LME2510DTMB_PID_RUNTIME,
			       &lme2510dtmb_props,
			       "LME2510 DTMB", RC_MAP_EMPTY)
	},
	{
		DVB_USB_DEVICE(LME2510DTMB_VID, LME2510DTMB_PID_RUNTIME_1122,
			       &lme2510dtmb_props,
			       "LME2510 DTMB (0x1122)", RC_MAP_EMPTY)
	},
	{
		DVB_USB_DEVICE(LME2510DTMB_VID, LME2510DTMB_PID_RUNTIME_22D0,
			       &lme2510dtmb_props,
			       "LME2510 DTMB (0x22d0)", RC_MAP_EMPTY)
	},
	{
		DVB_USB_DEVICE(LME2510DTMB_VID, LME2510DTMB_PID_RUNTIME_22D8,
			       &lme2510dtmb_props,
			       "LME2510 DTMB (0x22d8)", RC_MAP_EMPTY)
	},
	{
		DVB_USB_DEVICE(LME2510DTMB_VID, LME2510DTMB_PID_RUNTIME_E388,
			       &lme2510dtmb_props,
			       "LME2510 DTMB (0xe388)", RC_MAP_EMPTY)
	},
	{
		DVB_USB_DEVICE(LME2510DTMB_VID_ALT, LME2510DTMB_PID_RUNTIME_E388,
			       &lme2510dtmb_props,
			       "LME2510 DTMB (0x1b80:0xe388)", RC_MAP_EMPTY)
	},
	{ }  /* terminating entry */
};

MODULE_DEVICE_TABLE(usb, lme2510dtmb_id_table);

/* ── USB driver ────────────────────────────────────────────────── */

static struct usb_driver lme2510dtmb_driver = {
	.name       = "lme2510dtmb",
	.probe      = dvb_usbv2_probe,
	.disconnect = dvb_usbv2_disconnect,
	.id_table   = lme2510dtmb_id_table,
	.suspend      = dvb_usbv2_suspend,
	.resume       = dvb_usbv2_resume,
	.reset_resume = dvb_usbv2_reset_resume,
	.no_dynamic_id = 1,
};

module_usb_driver(lme2510dtmb_driver);

/* ── Module info ───────────────────────────────────────────────── */

MODULE_AUTHOR("LME2510DTMB project");
MODULE_DESCRIPTION("LME2510 DTMB USB dongle DVB driver");
MODULE_VERSION("0.2.0");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(LME2510DTMB_FW_FILE);
