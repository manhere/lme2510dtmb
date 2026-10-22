// SPDX-License-Identifier: GPL-2.0-only
/*
 * lme2510dtmb_mcu.c -- MCU commands for LME2510 DTMB USB dongle
 */
#include "lme2510dtmb.h"
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/delay.h>

static int mcu_send(struct dvb_usb_device *d, const u8 *cmd, int cmdlen)
{
	u8 *buf;
	u8 *ack;
	int ret, actual;

	buf = kmalloc(cmdlen, GFP_KERNEL);
	ack = kmalloc(8, GFP_KERNEL);
	if (!buf || !ack) {
		ret = -ENOMEM;
		goto out;
	}

	memcpy(buf, cmd, cmdlen);

	mutex_lock(&d->usb_mutex);
	ret = usb_bulk_msg(d->udev, usb_sndbulkpipe(d->udev, EP_BULK_OUT),
			   buf, cmdlen, &actual, 200);
	if (ret == 0)
		usb_bulk_msg(d->udev, usb_rcvbulkpipe(d->udev, EP_BULK_IN),
			     ack, 8, &actual, 200);
	mutex_unlock(&d->usb_mutex);

	/* ACK visibility: firmware/MCU protocol has an ACK for every
	 * command.  We log but do not fail on absence — matching the
	 * tolerance of the HiSilicon fork, which works on this hardware. */
	if (ret == 0 && actual > 0)
		dev_dbg(&d->udev->dev, "%s: ack[0]=0x%02x\n",
			__func__, ack[0]);
	else
		dev_dbg(&d->udev->dev, "%s: no ack (ret=%d)\n",
			__func__, ret);

out:
	kfree(buf);
	kfree(ack);
	return ret;
}

int lme2510dtmb_mcu_cmd(struct dvb_usb_device *d, u8 *buf, u8 len)
{
	return mcu_send(d, buf, len);
}

int lme2510dtmb_mcu_demod_variant(struct dvb_usb_device *d, u8 variant)
{
	u8 cmd[] = { MCU_OP_DEMOD_VARIANT, 0x01, variant };
	return lme2510dtmb_mcu_cmd(d, cmd, sizeof(cmd));
}

int lme2510dtmb_mcu_stream_start(struct dvb_usb_device *d)
{
	u8 all_pids[] = { 0x03, 0x06, 0x00, 0xff, 0x01, 0x1f, 0x20, 0x81 };
	u8 start[]    = { MCU_OP_STREAM_CTL, 0x00 };
	int ret;

	ret = mcu_send(d, all_pids, sizeof(all_pids));
	if (ret < 0)
		return -EIO;
	ret = mcu_send(d, all_pids, sizeof(all_pids));
	if (ret < 0)
		return -EIO;

	ret = mcu_send(d, start, sizeof(start));
	if (ret < 0)
		return -EIO;

	msleep(500);

	return 0;
}

int lme2510dtmb_mcu_stream_stop(struct dvb_usb_device *d)
{
	u8 cmd = MCU_OP_STREAM_STOP;

	mcu_send(d, &cmd, 1);

	return 0;
}

int lme2510dtmb_mcu_stream_restart(struct dvb_usb_device *d)
{
	u8 cmd = MCU_OP_STREAM_CTL;

	return mcu_send(d, &cmd, 1);
}

int lme2510dtmb_mcu_set_pid_filter(struct dvb_usb_device *d)
{
	u8 all_pids[] = { 0x03, 0x06, 0x00, 0xff, 0x01, 0x1f, 0x20, 0x81 };
	int ret;

	ret = mcu_send(d, all_pids, sizeof(all_pids));
	if (ret < 0)
		return ret;
	return mcu_send(d, all_pids, sizeof(all_pids));
}

/*
 * Arbitrary 1..16 PID hardware filter (from CH1_DTMB_RE_cpp reverse
 * engineering of the same firmware):
 *
 *   CMD 0x03, len = 4*n + 2, then per PID k:
 *     [2k]   pid_lo
 *     [2k+1] pid_hi
 *   tail: 0x20, terminal = 0x81 (mode 2, filter on) / 0x80 (mode 1)
 *         + 2*(n-1)
 * Sent twice, then committed with CMD 0x06 0x00.
 *
 * Caller must ensure EP1 usage is safe (pre-stream window or
 * force_pid_filter_usage opt-in).
 */
int lme2510dtmb_mcu_commit_pid_filter(struct dvb_usb_device *d,
				      const u16 *pids, int count, int mode)
{
	u8 cmd[70];
	int len = 0, k, ret;

	if (count < 1 || count > 16)
		return -EINVAL;

	cmd[len++] = MCU_OP_PID_FILTER;
	cmd[len++] = (u8)(4 * count + 2);
	for (k = 0; k < count; k++) {
		cmd[len++] = (u8)(2 * k);
		cmd[len++] = pids[k] & 0xff;
		cmd[len++] = (u8)(2 * k + 1);
		cmd[len++] = pids[k] >> 8;
	}
	cmd[len++] = 0x20;
	cmd[len++] = (u8)((mode == 2 ? 0x81 : 0x80) + 2 * (count - 1));

	ret = mcu_send(d, cmd, len);
	if (ret < 0)
		return ret;
	ret = mcu_send(d, cmd, len);
	if (ret < 0)
		return ret;

	{
		u8 commit[] = { MCU_OP_STREAM_CTL, 0x00 };
		return mcu_send(d, commit, sizeof(commit));
	}
}
