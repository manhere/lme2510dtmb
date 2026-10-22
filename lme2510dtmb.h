/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * lme2510dtmb.h — LME2510 DTMB USB dongle Linux DVB driver
 *
 * Hardware: LME2511 + LGS8GL5 + MAX2165
 * Copyright (C) 2026
 */

#ifndef LME2510DTMB_H
#define LME2510DTMB_H

#define BUILD_ID "merged-1"

#include <linux/types.h>
#include <linux/usb.h>
#include <linux/i2c.h>
#include "dvb_usb.h"
#include <media/dvb_frontend.h>

/* ── USB ──────────────────────────────────────────────────────── */

#define LME2510DTMB_VID                0x3344
#define LME2510DTMB_VID_ALT            0x1b80  /* alternate vendor stick */
#define LME2510DTMB_PID_RUNTIME        0x1120
#define LME2510DTMB_PID_RUNTIME_1122   0x1122  /* warm, lmedm04 conflict (blacklist) */
#define LME2510DTMB_PID_RUNTIME_22D0   0x22d0  /* warm, C++ "pid0xd0" family */
#define LME2510DTMB_PID_RUNTIME_22D8   0x22d8  /* warm */
#define LME2510DTMB_PID_RUNTIME_E388   0xe388  /* warm, both VIDs */
#define LME2510DTMB_PID_COLD           0x1111  /* pre-firmware identity */

/* ── Endpoints (alt-setting 1) ────────────────────────────────── */

#define EP_BULK_OUT             0x01
#define EP_BULK_IN              0x81
#define EP_TS_IN                0x88
#define EP_INTR_IN              0x8A

/* ── I2C addresses ────────────────────────────────────────────── */
/*
 * Linux I2C framework uses 7-bit addresses.
 * The lme2510dtmb MCU expects raw 7-bit (demod) or 8-bit (tuner) on the wire.
 * master_xfer() converts internally.
 */
#define I2C_ADDR_DEMOD_PRIMARY  0x19    /* LGS8GL5 main, wire=0x32 */
#define I2C_ADDR_DEMOD_ALT      0x1b    /* LGS8GL5 alternate, wire=0x36 */
#define I2C_ADDR_TUNER          0x60    /* MAX2165, wire=0xC0 */

/* Wire addresses (used internally by lme2510dtmb_i2c.c) */
#define I2C_WIRE_DEMOD_PRIMARY  0x32
#define I2C_WIRE_DEMOD_ALT      0x36
#define I2C_WIRE_TUNER          0xc0

/* ── I2C opcodes (EP1 Bulk) ───────────────────────────────────── */

#define I2C_OP_WR_MULTI         0x04
#define I2C_OP_WR_SINGLE        0x05
#define I2C_OP_RD_TUNER         0x84
#define I2C_OP_RD_DEMOD         0x85

/* ── MCU opcodes ──────────────────────────────────────────────── */

#define MCU_OP_PID_FILTER       0x03
#define MCU_OP_STREAM_CTL       0x06
#define MCU_OP_STREAM_STOP      0x0a
#define MCU_OP_DEMOD_VARIANT    0x16

/* ── Firmware ─────────────────────────────────────────────────── */

#define FW_CHUNK_SIZE           50
#define FW_ROM_SIZE             512
#define FW_APP_SIZE             4836
#define FW_TOTAL_SIZE           (FW_ROM_SIZE + FW_APP_SIZE)  /* 5348 */
#define FW_APP_SIZE_PID0XD0     3143
#define FW_TOTAL_SIZE_PID0XD0   (FW_ROM_SIZE + FW_APP_SIZE_PID0XD0)  /* 3655 */
#define LME2510DTMB_FW_FILE            "dvb-usb-lme2510dtmb.fw"
#define LME2510DTMB_FW_PID0XD0_FILE    "dvb-usb-lme2510dtmb-pid0xd0.fw"

/* ── Status packet ────────────────────────────────────────────── */

#define STATUS_MAGIC            0xbb
#define STATUS_PKT_LEN          7

/* ── Driver private state ─────────────────────────────────────── */

struct lme2510dtmb_state {
	/* EP8A interrupt transfer */
	struct urb *int_urb;
	u8 int_buffer[8];

	/* Cached EP8A status values */
	u8 signal_lock;
	u8 signal_strength_lo;   /* agc */
	u8 signal_snr;
	u8 signal_strength_hi;

	/* MAX2165 ROM calibration table (read at init) */
	u8 rom_tf_ntch_hi;       /* rom[1] & 0x0F, freq >= 725 MHz */
	u8 rom_tf_ntch_low;      /* rom[1] >> 4,  freq <  725 MHz */
	u8 rom_tf_balun_low;     /* rom[2] & 0x0F, 470 MHz endpoint */
	u8 rom_tf_balun_hi;      /* rom[2] >> 4,  780 MHz endpoint */
	u8 rom_bb_8mhz;          /* rom[3] >> 4,  bb filter index */

	bool streaming;           /* feed active, URBs submitted */
	bool stream_active;        /* MCU TS streaming in progress —
				     * during this phase ALL EP1 (I2C/MCU)
				     * communication is forbidden to avoid
				     * USB bus contention with EP88 TS */

	/* HW PID filter (CMD03) table, committed at stream start window.
	 * Framework only calls the pid_filter hooks when
	 * dvb_usb_force_pid_filter_usage=1 is set (default: all-pass). */
	u16 pid_table[16];
	bool filter_enabled;

	/* Cached demod register address for split reg-write/read I2C calls.
	 * The lme2510dtmb MCU I2C bridge requires combined write-then-read on the wire;
	 * when the kernel I2C stack sends separate "write reg-addr" and "read"
	 * transactions, the reg is cached here and consumed by the next read.
	 */
	u8 cached_demod_reg_addr;

	/* demod_read_hint: CMD 0x85 read request byte[4] echoes the data
	 * value of the last demod write (protocol detail from the HiSilicon
	 * lme2510-dtmb fork).  Filling 0x00 also works, this is fidelity. */
	u8 last_demod_write_val;
};

/* ── Sub-module headers (struct defs + public APIs) ───────────── */

#include "lme2510dtmb_lgs8gl5.h"
#include "lme2510dtmb_max2165.h"

/* ── Public API ───────────────────────────────────────────────── */

/* I2C */
extern const struct i2c_algorithm lme2510dtmb_i2c_algo;

/* MCU */
int lme2510dtmb_mcu_cmd(struct dvb_usb_device *d, u8 *buf, u8 len);
int lme2510dtmb_mcu_demod_variant(struct dvb_usb_device *d, u8 variant);
int lme2510dtmb_mcu_stream_start(struct dvb_usb_device *d);
int lme2510dtmb_mcu_stream_stop(struct dvb_usb_device *d);
int lme2510dtmb_mcu_stream_restart(struct dvb_usb_device *d);
int lme2510dtmb_mcu_set_pid_filter(struct dvb_usb_device *d);
int lme2510dtmb_mcu_commit_pid_filter(struct dvb_usb_device *d,
				      const u16 *pids, int count, int mode);

/* Firmware */
int lme2510dtmb_identify_state(struct dvb_usb_device *d, const char **name);
int lme2510dtmb_download_firmware(struct dvb_usb_device *d,
			   const struct firmware *fw);

/* Power */
int lme2510dtmb_power_ctrl(struct dvb_usb_device *d, int onoff);

/* Frontend attach */
int lme2510dtmb_frontend_attach(struct dvb_usb_adapter *adap);

/* Streaming */
int lme2510dtmb_streaming_ctrl(struct dvb_frontend *fe, int onoff);

/* Status (EP8A interrupt URB + frontend ops override) */
int lme2510dtmb_status_init(struct dvb_usb_adapter *adap);

/* Stream config */
int lme2510dtmb_get_stream_config(struct dvb_frontend *fe, u8 *ts_type,
			   struct usb_data_stream_properties *stream);

/* Exit */
void lme2510dtmb_exit(struct dvb_usb_device *d);

/* I2C gate control (for DVB frontend) — no-op, EP1 mutex is in dvb-usb-v2 */
int lme2510dtmb_i2c_gate_ctrl(struct dvb_frontend *fe, int enable);

/* ── Demod ────────────────────────────────────────────────────── */

struct dvb_frontend *lme2510dtmb_lgs8gl5_attach(
	const struct lme2510dtmb_lgs8gl5_config *config,
	struct i2c_adapter *i2c, u8 *chip_type);

/* ── Tuner ────────────────────────────────────────────────────── */

struct dvb_frontend *lme2510dtmb_max2165_attach(
	struct dvb_frontend *fe,
	struct i2c_adapter *i2c,
	const struct lme2510dtmb_max2165_config *cfg);

/*
 * Read MAX2165 ROM calibration table and store in lme2510dtmb_state.
 * Must be called with demod unlocked (demod[1] == 0xE0).
 */
int lme2510dtmb_max2165_read_rom(struct dvb_usb_device *d);

/* Full tuner init (16-byte block, default frequency) */
int lme2510dtmb_max2165_init_tuner(struct dvb_usb_device *d);

#endif /* LME2510DTMB_H */
