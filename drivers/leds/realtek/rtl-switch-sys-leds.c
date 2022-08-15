// SPDX-License-Identifier: GPL-2.0

#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "led-regfield.h"

/*
 * Realtek hardware system LED
 *
 * The switch SoC supports one hardware managed direct LED output
 * to manage a system LED, with two supported blinking rates.
 */

static const struct regfield_led_modes rtl_sys_led_modes = {
	.off = 0,
	.on = 3,
	.blink = {
		{64, 1},
		{1024, 2},
		{ /* sentinel */ }
	},
};

static const struct reg_field rtl838x_sys_led_field = REG_FIELD(0xa000, 16, 17);
static const struct reg_field rtl839x_sys_led_field = REG_FIELD(0x00e4, 15, 16);
//static const struct reg_field rtl930x_sys_led_field = REG_FIELD();
//static const struct reg_field rtl931x_sys_led_field = REG_FIELD();

static const struct of_device_id of_rtl_sys_led_match[] = {
	{
		.compatible = "realtek,maple-sys-led",
		.data = &rtl838x_sys_led_field,
	},
	{
		.compatible = "realtek,cypress-sys-led",
		.data = &rtl839x_sys_led_field,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_rtl_sys_led_match);

static int rtl_sys_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct led_init_data init_data = {};
	const struct reg_field *field_data;
	struct regmap_field *field;
	struct regfield_led *led;
	struct regmap *map;
	int err;

	field_data = device_get_match_data(dev);
	if (!field_data)
		return dev_err_probe(dev, -EINVAL, "no reg_field data\n");

	if (!np)
		return dev_err_probe(dev, -ENODEV, "no DT node found\n");

	map = device_node_to_regmap(of_get_parent(np));
	if (!map)
		return dev_err_probe(dev, -ENXIO, "failed to get regmap\n");

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	field = devm_regmap_field_alloc(dev, map, *field_data);
	if (IS_ERR(field))
		return dev_err_probe(dev, PTR_ERR(field), "register field allocation failed\n");

	err = regfield_led_init(led, field, of_fwnode_handle(np), &rtl_sys_led_modes);
	if (err)
		dev_err_probe(dev, err, "regfield_led initialisation failed\n");

	init_data.fwnode = of_fwnode_handle(np);

	return devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
}

static struct platform_driver rtl_sys_led_driver = {
	.probe = rtl_sys_led_probe,
	.driver = {
		.name = "realtek-sys-led",
		.of_match_table = of_rtl_sys_led_match
	}
};
module_platform_driver(rtl_sys_led_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek switch SoC system LED driver");
MODULE_LICENSE("GPL v2");
