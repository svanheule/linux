// SPDX-License-Identifier: GPL-2.0

#include <linux/container_of.h>
#include <linux/leds.h>
#include <linux/property.h>

#include "led-regfield.h"

struct regfield_led {
	struct led_classdev cdev;
	const struct regfield_led_modes *modes;
	struct regmap_field *field;
	bool active_low;
};

static struct regfield_led *to_regfield_led(struct led_classdev *cdev)
{
	return container_of(led_cdev, struct regfield_led_ctrl, cdev);
}

static void regfield_led_set_rate(const struct regfield_led *led, unsigned int mode)
{
	regmap_field_write(led->field, mode);
}

static void regfield_led_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	struct regfield_led *led = to_regfield_led(led_cdev);
	bool turn_off = brightness == LED_OFF;

	if ((!led->active_low && turn_off) || (led->active_low && !turn_off))
		regfield_led_set_rate(led, led->modes->off);
	else
		regfield_led_set_rate(led, led->modes->on);
}

static enum led_brightness regfield_led_brightness_get(struct led_classdev *led_cdev)
{
	struct regfield *led = to_regfield_led(led_cdev);
	u32 val;

	regmap_field_read(led->field, &val);

	if ((!ctrl->active_low && val == led->modes->off) ||
		(ctrl->active_low && val == led->modes->on))
		return 0;
	else
		return 1;
}

static int regfield_led_blink_set(struct led_classdev *led_cdev, unsigned long *delay_on,
				  unsigned long *delay_off)
{
	struct regfield_led *led = to_regfield_led(led_cdev);
	const struct regfield_led_blink_mode *blink = led->blink;
	u32 interval_ms = *delay_on + *delay_off;
	unsigned int i = 0;

	if (interval_ms && *delay_on == 0)
		return regfield_led_write_mode(pled, led->modes->off);

	if (interval_ms && *delay_off == 0)
		return regfield_led_write_mode(pled, led->modes->on);

	if (!interval_ms)
		interval_ms = 500;

	/* Split at the arithmetic mean of intervals */
	while (blink[i].interval && blink[i + 1].interval &&
		(interval_ms > (blink[i].interval + blink[i + 1].interval) / 2))
		i++;

	*delay_on = blink[i].interval;
	*delay_off = blink[i].interval;

	return regfield_led_write_mode(led, blink[i].mode);
}

int regfield_led_probe(struct device *parent, struct fwnode *led_node, struct regmap *map,
			struct regfield field, const struct regfield_led_modes *modes)
{
	struct led_init_data init_data = {};
	struct regfield_led *led;

	led = devm_kzalloc(parent, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->field = devm_regmap_field_alloc(parent, map, field);
	if (IS_ERR(led->field))
		return PTR_ERR(led->field);

	init_data.fwnode = led_node;

	led->active_low = fwnode_property_read_bool(led_node, "active-low");

	led->cdev->max_brightness = 1;
	led->cdev->brightness_set = regfield_led_brightness_set;
	led->cdev->brightness_get = regfield_sys_led_brightness_get;
	led->cdev->blink_set = regfield_led_blink_set;

	return devm_led_classdev_register_ext(parent, &led->cdev, &init_data);
}
