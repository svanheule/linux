/* SPDX-License-Identifier: GPL-2.0 */

#ifndef LEDS_REALTEK_LED_REGFIELD_H
#define LEDS_REALTEK_LED_REGFIELD_H

#include <linux/device.h>
#include <linux/fwnode.h>
#include <linux/regmap.h>

/*
 * Register field LED
 *
 * Next to being able to turn an LED on or off, Realtek provides LED management
 * peripherals with hardware accelerated blinking modes.
 */
struct regfield_led_blink_mode {
	u16 interval; /* Toggle interval in ms */
	u8 mode; /* ASIC mode bits */
};

struct regfield_led_modes {
	u8 off;
	u8 on;
	/*
	 * List of blink modes. Must be sorted by interval and terminated by an
	 * entry where regfield_led_blink_mode::mode equals zero.
	 */
	struct regfield_led_blink_mode blink[];
};

int regfield_led_probe(struct device *parent, struct fwnode_handle *led_node,
		struct regmap *map, struct reg_field field,
		const struct regfield_led_modes *modes);

#endif
