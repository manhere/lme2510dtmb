// SPDX-License-Identifier: GPL-2.0-only
/*
 * lme2510dtmb_firmware.c -- Cold/warm detection + firmware upload
 *
 * identify_state: reads cached alt-setting 1 descriptor → COLD/WARM.
 *   alt-setting activated in power_ctrl (matches lme2510dtmb-cli sequence).
 *
 * download_firmware:  ROM + APP in 50-byte chunks via dvb_usbv2_generic_rw_locked.
 */
#include "lme2510dtmb.h"
#include <linux/usb.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/delay.h>

static u8 fw_checksum(const u8 *data, int len)
{
	u8 s_even = 0, s_odd = 0;
	int i;
	for (i = 0; i < len; i++) {
		if (i & 1) s_odd  += data[i];
		else       s_even += data[i];
	}
	return s_even + s_odd;
}

static int send_chunk(struct dvb_usb_device *d, u8 hdr, u8 datalen,
		      const u8 *data)
{
	u8 *buf;
	u8 *ack_buf;
	int ret, total_len;

	buf = kmalloc(64, GFP_KERNEL);
	ack_buf = kmalloc(4, GFP_KERNEL);
	if (!buf || !ack_buf) {
		kfree(buf);
		kfree(ack_buf);
		return -ENOMEM;
	}

	buf[0] = hdr;
	buf[1] = (u8)(datalen - 1);
	memcpy(buf + 2, data, datalen);
	buf[2 + datalen] = fw_checksum(data, datalen);
	total_len = 2 + datalen + 1;

	mutex_lock(&d->usb_mutex);
	dev_dbg(&d->udev->dev, "fw: send_chunk hdr=0x%02x len=%d\n",
		hdr, datalen);
	ret = dvb_usbv2_generic_rw_locked(d, buf, total_len, ack_buf, 1);
	mutex_unlock(&d->usb_mutex);

	kfree(buf);

	if (ret < 0) {
		kfree(ack_buf);
		dev_err(&d->udev->dev, "%s: failed: %d\n", __func__, ret);
		return ret;
	}
	if (ack_buf[0] != 0x88 && ack_buf[0] != 0x77) {
		kfree(ack_buf);
		dev_err(&d->udev->dev, "%s: bad ACK 0x%02x\n",
			__func__, ack_buf[0]);
		return -EIO;
	}
	kfree(ack_buf);
	return 0;
}

static int upload_section(struct dvb_usb_device *d,
			  u8 hdr_mid, u8 hdr_last,
			  const u8 *data, int len, const char *label)
{
	int chunks = (len + FW_CHUNK_SIZE - 1) / FW_CHUNK_SIZE;
	int i;

	dev_info(&d->udev->dev, "fw: uploading %s (%d bytes)\n", label, len);
	for (i = 0; i < chunks; i++) {
		int off = i * FW_CHUNK_SIZE;
		int sz  = min_t(int, len - off, FW_CHUNK_SIZE);
		int ret = send_chunk(d, (i == chunks - 1) ? hdr_last : hdr_mid,
				     (u8)sz, data + off);
		if (ret < 0)
			return ret;
	}
	return 0;
}

/*
 * Firmware variant selection (mirrors the C++ user-space driver):
 * a stick enumerated at a PID whose low byte is 0xd0 (0x22d0 family)
 * runs the "pid0xd0" firmware build.  If such a stick shows up without
 * a ready MCU (no EP88 bulk endpoint), it needs a re-upload and must
 * get the same variant it was running — otherwise it would re-enumerate
 * as the main firmware's PID.  All other cold devices (incl. 0x1111
 * bootloader) use the main firmware, matching the C++ cold-boot path.
 */
static const char *lme2510dtmb_select_fw_file(struct dvb_usb_device *d)
{
	__le16 pid = d->udev->descriptor.idProduct;

	if ((le16_to_cpu(pid) & 0x00ff) == 0xd0)
		return LME2510DTMB_FW_PID0XD0_FILE;
	return LME2510DTMB_FW_FILE;
}

int lme2510dtmb_identify_state(struct dvb_usb_device *d, const char **name)
{
	struct usb_host_interface *alt;
	int i, ifnum = d->props->bInterfaceNumber;
	bool found_ep88 = false;

	/* When actconfig is not available (very early probe), assume COLD */
	if (!d->udev->actconfig || !d->udev->actconfig->interface) {
		dev_dbg(&d->udev->dev, "%s: no actconfig — assume COLD\n",
			__func__);
		*name = lme2510dtmb_select_fw_file(d);
		return COLD;
	}

	if (ifnum >= d->udev->actconfig->desc.bNumInterfaces)
		goto cold;

	/* Use alt-setting 1 descriptors; if not available, fallback to alt0 */
	alt = &d->udev->actconfig->interface[ifnum]->altsetting[0];
	if (d->udev->actconfig->interface[ifnum]->num_altsetting > 1)
		alt = &d->udev->actconfig->interface[ifnum]->altsetting[1];

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *epd = &alt->endpoint[i].desc;
		if ((epd->bEndpointAddress & 0x7f) == 0x08 &&
		    (epd->bEndpointAddress & USB_DIR_IN) &&
		    (epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK &&
		    usb_endpoint_maxp(epd) >= 512) {
			found_ep88 = true;
			break;
		}
	}

	if (found_ep88) {
		*name = NULL;
		return WARM;
	}
cold:
	*name = lme2510dtmb_select_fw_file(d);
	return COLD;
}

/* jiffies of last successful upload — guards against a warm re-probe
 * racing the reconnect (device may briefly reappear cold). */
static unsigned long last_fw_upload;

int lme2510dtmb_download_firmware(struct dvb_usb_device *d,
			   const struct firmware *fw)
{
	int ret;
	int app_size;

	/* Two variant blobs, both = ROM(512B) + APP:
	 *   main    5348B (APP 4836B)   — dvb-usb-lme2510dtmb.fw
	 *   pid0xd0 3655B (APP 3143B)   — dvb-usb-lme2510dtmb-pid0xd0.fw
	 * APP size is derived from the blob size (ROM is fixed 512B).
	 */
	if (fw->size == FW_TOTAL_SIZE)
		app_size = FW_APP_SIZE;
	else if (fw->size == FW_TOTAL_SIZE_PID0XD0)
		app_size = FW_APP_SIZE_PID0XD0;
	else
		return -EINVAL;

	if (last_fw_upload &&
	    time_before(jiffies, last_fw_upload + 60 * HZ)) {
		dev_info(&d->udev->dev,
			 "firmware already uploaded %lu s ago, waiting for warm state\n",
			 (jiffies - last_fw_upload) / HZ);
		return RECONNECTS_USB;
	}

	dev_info(&d->udev->dev, "Uploading firmware (%s, %d bytes)...\n",
		 fw->size == FW_TOTAL_SIZE ? "main" : "pid0xd0",
		 (int)fw->size);

	dev_info(&d->udev->dev,
		 "switching to alt-setting 1 (ifnum=%d)...\n",
		 d->props->bInterfaceNumber);

	ret = usb_set_interface(d->udev,
				d->props->bInterfaceNumber, 1);
	if (ret < 0) {
		dev_err(&d->udev->dev,
			"failed to set alt-setting 1: %d\n", ret);
		return ret;
	}

	ret = upload_section(d, 0x01, 0x81, fw->data, FW_ROM_SIZE, "ROM");
	if (ret < 0)
		return ret;

	ret = upload_section(d, 0x02, 0x82,
			     fw->data + FW_ROM_SIZE, app_size, "APP");
	if (ret < 0)
		return ret;

	dev_info(&d->udev->dev, "Firmware done, restarting MCU...\n");
	{
		u8 *cmd = kmalloc(2, GFP_KERNEL);
		u8 *dummy = kmalloc(2, GFP_KERNEL);
		if (cmd && dummy) {
			cmd[0] = 0x8A; cmd[1] = 0x00;
			msleep(100);
			mutex_lock(&d->usb_mutex);
			dvb_usbv2_generic_rw_locked(d, cmd, 2, dummy, 0);
			mutex_unlock(&d->usb_mutex);
			kfree(cmd);
			kfree(dummy);
		}
	}

	last_fw_upload = jiffies;
	return RECONNECTS_USB;
}
