// SPDX-License-Identifier: GPL-2.0

#include <linux/leds.h>
#include <linux/property.h>

#include "led-regfield.h"

static int regfield_led_set_mode(const struct regfield_led *led, unsigned int mode)
{
	return regmap_field_write(led->field, mode);
}

static void regfield_led_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	struct regfield_led *led = to_regfield_led(led_cdev);
	bool turn_off = brightness == LED_OFF;

	if ((!led->active_low && turn_off) || (led->active_low && !turn_off))
		regfield_led_set_mode(led, led->modes->off);
	else
		regfield_led_set_mode(led, led->modes->on);
}

static enum led_brightness regfield_led_brightness_get(struct led_classdev *led_cdev)
{
	struct regfield_led *led = to_regfield_led(led_cdev);
	u32 val = 0;

	regmap_field_read(led->field, &val);

	if ((!led->active_low && val == led->modes->off) ||
		(led->active_low && val == led->modes->on))
		return 0;
	else
		return 1;
}

static int regfield_led_blink_set(struct led_classdev *led_cdev, unsigned long *delay_on,
				  unsigned long *delay_off)
{
	struct regfield_led *led = to_regfield_led(led_cdev);
	const struct regfield_led_blink_mode *blink = led->modes->blink;
	u32 cycle_ms = *delay_on + *delay_off;
	int err;

	if (!cycle_ms)
		cycle_ms = 500;

	while (blink->toggle_ms && (blink + 1)->toggle_ms) {
		/*
		 * Split at the arithmetic mean of intervals, which compares
		 * the half cycle interval (cycle_ms / 2) to the mean toggle
		 * interval ((blink->toggle_ms + (blink + 1)->toggle_ms) / 2).
		 * Since the (/ 2) is common on both sides, it can be dropped.
		 */
		if (cycle_ms > (blink->toggle_ms + (blink + 1)->toggle_ms))
			break;
		blink++;
	}

	err = regfield_led_set_mode(led, blink->mode);
	if (err)
		return err;

	*delay_on = blink->toggle_ms;
	*delay_off = blink->toggle_ms;

	return 0;
}

int regfield_led_probe(struct device *parent, struct fwnode_handle *led_node,
			struct regmap *map, struct reg_field field,
			const struct regfield_led_modes *modes)
{
	struct led_init_data init_data = {};
	struct regfield_led *led;

	if (!parent || !led_node)
		return -ENODEV;

	if (!map)
		return -ENXIO;

	if (!modes)
		return -EINVAL;

	led = devm_kzalloc(parent, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->field = devm_regmap_field_alloc(parent, map, field);
	if (IS_ERR(led->field))
		return PTR_ERR(led->field);

	init_data.fwnode = led_node;

	led->modes = modes;
	led->active_low = fwnode_property_read_bool(led_node, "active-low");

	led->cdev.max_brightness = 1;
	led->cdev.brightness_set = regfield_led_brightness_set;
	led->cdev.brightness_get = regfield_led_brightness_get;
	led->cdev.blink_set = regfield_led_blink_set;

	return devm_led_classdev_register_ext(parent, &led->cdev, &init_data);
}
