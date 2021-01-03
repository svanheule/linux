// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/leds.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define REALTEK_EIO_GLOBAL_CTRL				0x0

/*
 * Management of external RTL8231 GPIO expanders.
 * One RTL8231's GPIO registers can be shadowed to the internal GPIO_DIR
 * and GPIO_DAT registers.
 */
#define RTL8380_EIO_GPIO_INDIRECT_ACCESS	0x9C
#define RTL8380_EIO_GPIO_CTRL			0xE0
#define RTL8380_EIO_GPIO_DIR(pin)		(0xE4 + 4*((pin)/32))
#define RTL8380_EIO_GPIO_DAT(pin)		(0xEC + 4*((pin)/32))

struct realtek_eio_ctrl;
struct realtek_gpio_mdio_data;

struct realtek_eio_data {
	unsigned int sys_led_pos;
	unsigned int port_count;
};

struct realtek_eio_ctrl {
	struct device *dev;
	struct regmap *map;
	const struct realtek_eio_data *data;
	struct led_classdev sys_led;
};

/*
 * Realtek hardware system LED
 *
 * The switch SoC supports one hardware managed direct LED output
 * to manage a system LED, with two supported blinking rates.
 */
enum {
	REALTEK_SYS_LED_OFF = 0,
	REALTEK_SYS_LED_BLINK_64MS,
	REALTEK_SYS_LED_BLINK_1024MS,
	REALTEK_SYS_LED_ON
};

static void realtek_sys_led_set(const struct realtek_eio_ctrl *ctrl,
	unsigned int mode)
{
	regmap_update_bits(ctrl->map, REALTEK_EIO_GLOBAL_CTRL,
		(0x3 << ctrl->data->sys_led_pos),
		((mode & 0x3) << ctrl->data->sys_led_pos));
}

static void realtek_sys_led_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct realtek_eio_ctrl *ctrl =
		container_of(led_cdev, struct realtek_eio_ctrl, sys_led);

	if (brightness)
		realtek_sys_led_set(ctrl, REALTEK_SYS_LED_ON);
	else
		realtek_sys_led_set(ctrl, REALTEK_SYS_LED_OFF);
}

static enum led_brightness realtek_sys_led_brightness_get(
	struct led_classdev *led_cdev)
{
	struct realtek_eio_ctrl *ctrl =
		container_of(led_cdev, struct realtek_eio_ctrl, sys_led);
	u32 val;

	regmap_read(ctrl->map, REALTEK_EIO_GLOBAL_CTRL, &val);
	val = (val >> ctrl->data->sys_led_pos) & 0x3;

	if (val == REALTEK_SYS_LED_OFF)
		return LED_OFF;
	else
		return LED_ON;
}

static int realtek_sys_led_blink_set(struct led_classdev *led_cdev,
	unsigned long *delay_on, unsigned long *delay_off)
{
	struct realtek_eio_ctrl *ctrl =
		container_of(led_cdev, struct realtek_eio_ctrl, sys_led);
	u32 blink_interval = *delay_on + *delay_off;

	/* Split range at geometric mean of 64 and 1024 */
	if (blink_interval == 0 || blink_interval > 2*256) {
		*delay_on = 1024;
		*delay_off = 1024;
		realtek_sys_led_set(ctrl, REALTEK_SYS_LED_BLINK_1024MS);
	}
	else {
		*delay_on = 64;
		*delay_off = 64;
		realtek_sys_led_set(ctrl, REALTEK_SYS_LED_BLINK_64MS);
	}

	return 0;
}

static int realtek_sys_led_probe(struct realtek_eio_ctrl *ctrl,
	struct device *parent, struct device_node *np)
{
	struct led_classdev *sys_led = &ctrl->sys_led;
	struct led_init_data init_data = {};

	init_data.fwnode = of_fwnode_handle(np);

	sys_led->max_brightness = 1;
	sys_led->brightness_set = realtek_sys_led_brightness_set;
	sys_led->brightness_get = realtek_sys_led_brightness_get;
	sys_led->blink_set = realtek_sys_led_blink_set;

	return devm_led_classdev_register_ext(parent, sys_led, &init_data);
}

static const struct realtek_eio_data rtl8380_eio_data = {
	.sys_led_pos = 16,
	.port_count = 28,
};

//static struct realtek_eio_data rtl8390_eio_data = {
//	.sys_led_pos = 15,
//};
//
//static struct realtek_eio_data rtl9300_eio_data = {
//	.sys_led_pos = 13,
//};
//
//static struct realtek_eio_data rtl9310_eio_data = {
//	.sys_led_pos = 12,
//};

static const struct of_device_id of_realtek_eio_match[] = {
	{
		.compatible = "realtek,rtl8380-eio",
		.data = &rtl8380_eio_data,
	},
};

MODULE_DEVICE_TABLE(of, of_realtek_eio_match);

static const struct mfd_cell mfd_port_led_devices[] = {
	OF_MFD_CELL("realtek-eio-port-led",
		NULL, NULL, 0, 0, "realtek,rtl8380-eio-port-led"),
};

static int realtek_eio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *np_sys_led;
	const struct of_device_id *match;
	struct realtek_eio_ctrl *ctrl;
	int err, val;
	unsigned r;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	match = of_match_device(of_realtek_eio_match, &pdev->dev);
	if (match)
		ctrl->data = (struct realtek_eio_data *) match->data;
	else {
		dev_err(dev, "no device match\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, ctrl);

	ctrl->dev = dev;

	if (!np) {
		dev_err(dev, "no DT node found\n");
		return -EINVAL;
	}

	ctrl->map = device_node_to_regmap(np);
	if (!ctrl->map) {
		dev_err(dev, "failed to get regmap\n");
		return -EINVAL;
	}

	/* Parse optional sys-led child */
	np_sys_led = of_get_child_by_name(np, "sys-led");
	if (IS_ERR(np_sys_led))
		return PTR_ERR(np_sys_led);

	if (np_sys_led) {
		err = realtek_sys_led_probe(ctrl, dev, np_sys_led);
		if (err)
		    return err;
	}

	/* Find sub-devices */
	mfd_add_devices(dev, 0, mfd_port_led_devices,
		ARRAY_SIZE(mfd_port_led_devices), NULL, 0, NULL);

	/* Parse optional mdio-bus child */
	// TODO

	/* Dump register values */
	for (r = 0; r <= regmap_get_max_register(ctrl->map); r += 4) {
		regmap_read(ctrl->map, r, &val);
		dev_info(dev, "%02x %08x\n", r, val);
	}

	return 0;
}

static struct platform_driver realtek_eio_driver = {
	.probe = realtek_eio_probe,
	.driver = {
		.name = "realtek-ext-io",
		.of_match_table = of_realtek_eio_match
	}
};

module_platform_driver(realtek_eio_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek switch SoC external LED/GPIO driver");
MODULE_LICENSE("GPL v2");
