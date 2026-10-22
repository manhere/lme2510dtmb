// SPDX-License-Identifier: GPL-2.0-only
/*
 * lme2510dtmb_status.c — EP8A interrupt status polling
 *
 * Merged driver: enabled by default (module param ep8a_status in
 * lme2510dtmb_core.c).  The C++ user-space implementation (CH1_DTMB_RE_cpp)
 * proves EP8A status transfers coexist with EP88 TS streaming on this
 * firmware — unlike EP1 I2C, which is forbidden while streaming.
 *
 * EP8A frames, 8-byte stride (HiSilicon lme2510-dtmb parse_status):
 *   [0] = 0xbb magic
 *   [1] = 0x05 (ignored)
 *   [2] = lock
 *   [3] = signal low
 *   [4] = snr
 *   [5] = signal high
 *   [6] = extra
 *
 * Strength/quality conversion formulas are the HiSilicon fork's
 * stream_strength()/stream_quality() branches (identical to the C++
 * implementation's EP8A formulas).
 */

#include "lme2510dtmb.h"

#include <linux/usb.h>
#include <media/dvb_frontend.h>

/* ── Interrupt URB callback ────────────────────────────────────── */

static void lme2510dtmb_int_callback(struct urb *urb)
{
	struct dvb_usb_adapter *adap = urb->context;
	struct dvb_usb_device *d = adap_to_d(adap);
	struct lme2510dtmb_state *st = d->priv;
	u8 *buf;
	int actual = urb->actual_length;
	int rc;

	switch (urb->status) {
	case 0:
	case -ETIMEDOUT:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	case -EOVERFLOW:
		/* Do NOT fall back to EP1 register reads here — EP1 is
		 * forbidden while the MCU TS engine runs.  Just log and
		 * keep polling. */
		dev_warn_ratelimited(&d->udev->dev,
				     "EP8A overflow, continuing\n");
		break;
	default:
		dev_dbg(&d->udev->dev, "%s: urb error %d\n",
			__func__, urb->status);
		break;
	}

	buf = (u8 *)urb->transfer_buffer;

	/* Parse 8-byte-strided frames (one transfer may carry several). */
	while (actual >= 8) {
		if (buf[0] == STATUS_MAGIC) {
			st->signal_lock        = buf[2];
			st->signal_strength_lo = buf[3];
			st->signal_snr         = buf[4];
			st->signal_strength_hi = buf[5];
			dev_dbg(&d->udev->dev,
				"EP8A: lock=%u lo=%u snr=%u hi=%u\n",
				buf[2], buf[3], buf[4], buf[5]);
		}
		buf += 8;
		actual -= 8;
	}

	rc = usb_submit_urb(urb, GFP_ATOMIC);
	if (rc && rc != -ENODEV && rc != -ESHUTDOWN)
		dev_warn_ratelimited(&d->udev->dev,
				     "EP8A resubmit failed: %d\n", rc);
}

/* ── Interrupt URB init ────────────────────────────────────────── */

static int lme2510dtmb_int_start(struct dvb_usb_adapter *adap)
{
	struct dvb_usb_device *d = adap_to_d(adap);
	struct lme2510dtmb_state *st = d->priv;
	struct usb_host_endpoint *ep;
	int ret;

	st->int_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!st->int_urb)
		return -ENOMEM;

	/* EP 0x8A may be reported as bulk on some controllers (EHCI quirk):
	 * fill as interrupt, override pipe if the descriptor says bulk. */
	usb_fill_int_urb(st->int_urb, d->udev,
			 usb_rcvintpipe(d->udev, 0x8a),
			 st->int_buffer, sizeof(st->int_buffer),
			 lme2510dtmb_int_callback, adap, 11);

	ep = usb_pipe_endpoint(d->udev, st->int_urb->pipe);
	if (ep && usb_endpoint_type(&ep->desc) == USB_ENDPOINT_XFER_BULK)
		st->int_urb->pipe = usb_rcvbulkpipe(d->udev, 0x8a);

	ret = usb_submit_urb(st->int_urb, GFP_KERNEL);
	if (ret) {
		usb_free_urb(st->int_urb);
		st->int_urb = NULL;
		return ret;
	}

	dev_info(&d->udev->dev, "EP8A status polling started\n");
	return 0;
}

/* ── Conversion formulas (HiSilicon fork, EP8A live path) ──────── */

static u16 lme2510dtmb_percent(u8 percent)
{
	return min_t(u8, percent, 100) * 0xffffU / 100;
}

static u8 lme2510dtmb_stream_strength(const struct lme2510dtmb_state *st,
				      bool locked)
{
	u16 raw = ((u16)st->signal_strength_hi << 8) | st->signal_strength_lo;
	u8 low = st->signal_strength_lo & 0x07;
	u8 strength;

	if (!locked)
		strength = low + 8;
	else if (raw >= 0x1f00 && raw <= 0x1fff)
		strength = low + 80;
	else if (raw >= 0x0058 && raw <= 0x00ff)
		strength = low + 70;
	else if (raw >= 0x0100 && raw <= 0x0180)
		strength = low + 60;
	else if (raw >= 0x0180 && raw <= 0x0200)
		strength = low + 50;
	else
		strength = 69;

	return min_t(u8, strength, 100);
}

static u8 lme2510dtmb_stream_quality(u8 raw, bool locked)
{
	u8 scaled = (u32)raw * 100 / 255;
	u8 remainder = raw % 15;

	if (!locked)
		return remainder + 5;
	if (!scaled)
		return remainder + 30;
	if (scaled <= 40)
		return 90 - scaled;
	if (scaled >= 90)
		return 30;
	return 100 - scaled * 3 / 4;
}

/* ── Frontend ops overrides ────────────────────────────────────── */

static int lme2510dtmb_ep8a_read_status(struct dvb_frontend *fe,
					enum fe_status *status)
{
	struct lme2510dtmb_state *st = fe_to_d(fe)->priv;

	*status = 0;

	if (st->signal_strength_hi || st->signal_strength_lo)
		*status |= FE_HAS_SIGNAL;

	if (st->signal_lock)
		*status |= (FE_HAS_LOCK | FE_HAS_CARRIER | FE_HAS_SYNC |
			    FE_HAS_VITERBI | FE_HAS_SIGNAL);

	return 0;
}

static int lme2510dtmb_ep8a_read_signal_strength(struct dvb_frontend *fe,
						 u16 *strength)
{
	struct lme2510dtmb_state *st = fe_to_d(fe)->priv;
	bool locked = st->signal_lock != 0;

	*strength = lme2510dtmb_percent(
		lme2510dtmb_stream_strength(st, locked));
	return 0;
}

static int lme2510dtmb_ep8a_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct lme2510dtmb_state *st = fe_to_d(fe)->priv;
	bool locked = st->signal_lock != 0;

	*snr = lme2510dtmb_percent(
		lme2510dtmb_stream_quality(st->signal_snr, locked));
	return 0;
}

/* ── Public entry point (called from frontend_attach) ──────────── */

int lme2510dtmb_status_init(struct dvb_usb_adapter *adap)
{
	int ret = lme2510dtmb_int_start(adap);
	if (ret < 0)
		return ret;

	if (adap->fe[0]) {
		adap->fe[0]->ops.read_status =
			lme2510dtmb_ep8a_read_status;
		adap->fe[0]->ops.read_signal_strength =
			lme2510dtmb_ep8a_read_signal_strength;
		adap->fe[0]->ops.read_snr = lme2510dtmb_ep8a_read_snr;
	}

	return 0;
}
