// SPDX-License-Identifier: GPL-2.0-only
/*
 * lme2510dtmb_i2c.c -- I2C bridge via EP1 Bulk for LME2510 DTMB USB dongle
 *
 * Wire format matches lme2510dtmb-cli golden reference.
 * All DMA buffers are kmalloc'd to avoid stack-DMA warnings.
 */
#include "lme2510dtmb.h"
#include <linux/i2c.h>
#include <linux/usb.h>
#include <linux/delay.h>
#include <linux/slab.h>

static int ep1_i2c_write(struct dvb_usb_device *d, bool is_tuner,
			 u8 waddr, const u8 *buf, int len)
{
	struct lme2510dtmb_state *st = d->priv;
	u8 *obuf;
	u8 *ack;
	int wlen, ret, actual;

	obuf = kmalloc(64, GFP_KERNEL);
	ack = kmalloc(8, GFP_KERNEL);
	if (!obuf || !ack) {
		kfree(obuf);
		kfree(ack);
		return -ENOMEM;
	}

	if (is_tuner) {
		obuf[0] = I2C_OP_WR_MULTI;
		obuf[1] = (u8)(len + 1);
		obuf[2] = waddr;
		memcpy(&obuf[3], buf, len);
		wlen = 3 + len;
	} else {
		if (len < 2) { kfree(obuf); kfree(ack); return -EINVAL; }
		obuf[0] = I2C_OP_WR_SINGLE;
		obuf[1] = 0x04;
		obuf[2] = waddr;
		obuf[3] = buf[0];
		obuf[4] = buf[1];
		wlen = 5;
	}

	mutex_lock(&d->usb_mutex);
	ret = usb_bulk_msg(d->udev, usb_sndbulkpipe(d->udev, EP_BULK_OUT),
			   obuf, wlen, &actual, 400);
	if (ret == 0)
		usb_bulk_msg(d->udev, usb_rcvbulkpipe(d->udev, EP_BULK_IN),
			     ack, 8, &actual, 200);
	mutex_unlock(&d->usb_mutex);

	/* demod_read_hint bookkeeping: CMD 0x85 read requests echo the
	 * data value of the last demod write in byte[4]. */
	if (ret == 0 && !is_tuner && len >= 2)
		st->last_demod_write_val = buf[1];

	kfree(obuf);
	kfree(ack);
	return ret;
}

static int ep1_i2c_read(struct dvb_usb_device *d, bool is_tuner,
			u8 waddr, u8 reg, u8 *dest, int dlen)
{
	struct lme2510dtmb_state *st = d->priv;
	u8 *obuf;
	u8 *ibuf;
	int ret, actual;

	obuf = kmalloc(8, GFP_KERNEL);
	ibuf = kmalloc(64, GFP_KERNEL);
	if (!obuf || !ibuf) {
		kfree(obuf);
		kfree(ibuf);
		return -ENOMEM;
	}

	if (is_tuner) {
		obuf[0] = I2C_OP_RD_TUNER;
		obuf[1] = 0x03;
		obuf[2] = waddr;
		obuf[3] = reg;
		obuf[4] = 0x01;
	} else {
		obuf[0] = I2C_OP_RD_DEMOD;
		obuf[1] = 0x02;
		obuf[2] = waddr;
		obuf[3] = reg;
		obuf[4] = st->last_demod_write_val;
	}

	mutex_lock(&d->usb_mutex);

	ret = usb_bulk_msg(d->udev, usb_sndbulkpipe(d->udev, EP_BULK_OUT),
			   obuf, 5, &actual, 200);
	if (ret < 0) {
		mutex_unlock(&d->usb_mutex);
		kfree(obuf);
		kfree(ibuf);
		return ret;
	}

	/* 3-read retry (EHCI/xHCI ACK split, from lme2510dtmb-cli) */
	{
		int i;
		for (i = 0; i < 3; i++) {
			if (i > 0)
				msleep(100);
			ret = usb_bulk_msg(d->udev,
					   usb_rcvbulkpipe(d->udev, EP_BULK_IN),
					   ibuf, 64, &actual,
					   (i == 0) ? 200 : 2000);
			if (ret == 0 && actual > 1) {
				int copy = min_t(int, actual - 1, dlen);
				memcpy(dest, &ibuf[1], copy);
				mutex_unlock(&d->usb_mutex);
				kfree(obuf);
				kfree(ibuf);
				return 0;
			}
		}
	}

	mutex_unlock(&d->usb_mutex);
	kfree(obuf);
	kfree(ibuf);
	return -EAGAIN;
}

static int lme2510dtmb_i2c_master_xfer(struct i2c_adapter *adap,
				struct i2c_msg msg[], int num)
{
	struct dvb_usb_device *d = i2c_get_adapdata(adap);
	struct lme2510dtmb_state *st = d->priv;
	int i, ret;

	mutex_lock(&d->i2c_mutex);

	for (i = 0; i < num; i++) {
		bool is_read = msg[i].flags & I2C_M_RD;
		u8 addr7 = msg[i].addr;
		bool is_tuner = (addr7 >= 0x40);
		u8 waddr = addr7 << 1;

		if (is_read) {
			u8 reg = is_tuner ? 0 : st->cached_demod_reg_addr;
			ret = ep1_i2c_read(d, is_tuner, waddr, reg,
					   msg[i].buf, msg[i].len);
		} else if (i + 1 < num && (msg[i + 1].flags & I2C_M_RD)) {
			u8 reg = msg[i].buf[0];
			ret = ep1_i2c_read(d, is_tuner, waddr, reg,
					   msg[i + 1].buf, msg[i + 1].len);
			i++;
		} else {
			if (!is_tuner && msg[i].len == 1) {
				st->cached_demod_reg_addr = msg[i].buf[0];
				ret = 0;
			} else {
				ret = ep1_i2c_write(d, is_tuner, waddr,
						    msg[i].buf, msg[i].len);
			}
		}

		if (ret < 0) {
			mutex_unlock(&d->i2c_mutex);
			return -EAGAIN;
		}
	}

	mutex_unlock(&d->i2c_mutex);
	return i;
}

static u32 lme2510dtmb_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

const struct i2c_algorithm lme2510dtmb_i2c_algo = {
	.master_xfer   = lme2510dtmb_i2c_master_xfer,
	.functionality = lme2510dtmb_i2c_func,
};
