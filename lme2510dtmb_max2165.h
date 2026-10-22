/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * lme2510dtmb_max2165.h — MAX2165 tuner config for LME2510 DTMB driver
 */

#ifndef LME2510DTMB_MAX2165_H
#define LME2510DTMB_MAX2165_H

#include <linux/types.h>

struct lme2510dtmb_max2165_config {
	/* Linux I2C 7-bit address (0x60 → wire 0xC0) */
	u8 i2c_address;
	u32 osc_clk_khz;         /* 12000 (12 MHz) */
	u32 default_freq_khz;    /* 474000 */
};

#endif /* LME2510DTMB_MAX2165_H */
