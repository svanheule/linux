// SPDX-License-Identifier: GPL-2.0-only

#include <linux/gpio/driver.h>
#include <linux/mfd/rtl8231.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define RTL8231_GPIO_DIR_IN	1
#define RTL8231_GPIO_DIR_OUT	0

#define RTL8231_MAX_GPIOS	37

enum rtl8231_gpio_regfield {
	RTL8231_FIELD_PIN_MODE0,
	RTL8231_FIELD_PIN_MODE1,
	RTL8231_FIELD_PIN_MODE2,
	RTL8231_FIELD_GPIO_DIR0,
	RTL8231_FIELD_GPIO_DIR1,
	RTL8231_FIELD_GPIO_DIR2,
	RTL8231_FIELD_GPIO_DATA0,
	RTL8231_FIELD_GPIO_DATA1,
	RTL8231_FIELD_GPIO_DATA2,
	RTL8231_FIELD_GPIO_MAX
};

static const struct reg_field rtl8231_fields[RTL8231_FIELD_GPIO_MAX] = {
	[RTL8231_FIELD_PIN_MODE0] = REG_FIELD(RTL8231_REG_PIN_MODE0, 0, 15),
	[RTL8231_FIELD_PIN_MODE1] = REG_FIELD(RTL8231_REG_PIN_MODE1, 0, 15),
	[RTL8231_FIELD_PIN_MODE2] = REG_FIELD(RTL8231_REG_PIN_HI_CFG, 0, 4),
	[RTL8231_FIELD_GPIO_DIR0] = REG_FIELD(RTL8231_REG_GPIO_DIR0, 0, 15),
	[RTL8231_FIELD_GPIO_DIR1] = REG_FIELD(RTL8231_REG_GPIO_DIR1, 0, 15),
	[RTL8231_FIELD_GPIO_DIR2] = REG_FIELD(RTL8231_REG_PIN_HI_CFG, 5, 9),
	[RTL8231_FIELD_GPIO_DATA0] = REG_FIELD(RTL8231_REG_GPIO_DATA0, 0, 15),
	[RTL8231_FIELD_GPIO_DATA1] = REG_FIELD(RTL8231_REG_GPIO_DATA1, 0, 15),
	[RTL8231_FIELD_GPIO_DATA2] = REG_FIELD(RTL8231_REG_GPIO_DATA2, 0, 4),
};

struct rtl8231_gpio_ctrl {
	struct gpio_chip gc;
	struct regmap_field *fields[RTL8231_FIELD_GPIO_MAX];
};

static int rtl8231_pin_read(struct rtl8231_gpio_ctrl *ctrl, int base, int offset)
{
	int field = base + offset / 16;
	int bit = offset % 16;
	unsigned int v;
	int err;

	err = regmap_field_read(ctrl->fields[field], &v);

	if (err)
		return err;

	return !!(v & BIT(bit));
}

static int rtl8231_pin_write(struct rtl8231_gpio_ctrl *ctrl, int base, int offset, int val)
{
	int field = base + offset / 16;
	int bit = offset % 16;

	return regmap_field_update_bits(ctrl->fields[field], BIT(bit), val << bit);
}

static int rtl8231_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	return rtl8231_pin_write(ctrl, RTL8231_FIELD_GPIO_DIR0, offset, RTL8231_GPIO_DIR_IN);
}

static int rtl8231_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	int err;

	err = rtl8231_pin_write(ctrl, RTL8231_FIELD_GPIO_DIR0, offset, RTL8231_GPIO_DIR_OUT);
	if (err)
		return err;

	return rtl8231_pin_write(ctrl, RTL8231_FIELD_GPIO_DATA0, offset, value);
}

static int rtl8231_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	return rtl8231_pin_read(ctrl, RTL8231_FIELD_GPIO_DIR0, offset);
}

static int rtl8231_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	return rtl8231_pin_read(ctrl, RTL8231_FIELD_GPIO_DATA0, offset);
}

static void rtl8231_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	rtl8231_pin_write(ctrl, RTL8231_FIELD_GPIO_DATA0, offset, value);
}

static int rtl8231_gpio_get_multiple(struct gpio_chip *gc,
	unsigned long *mask, unsigned long *bits)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	int read, field;
	int offset, shift;
	int sub_mask;
	int value, err;

	err = 0;
	read = 0;
	field = 0;

	while (read < gc->ngpio) {
		shift = read % (8 * sizeof(*bits));
		offset = read / (8 * sizeof(*bits));
		sub_mask = (mask[offset] >> shift) & 0xffff;
		if (sub_mask) {
			err = regmap_field_read(ctrl->fields[RTL8231_FIELD_GPIO_DATA0 + field],
				&value);
			if (err)
				return err;
			value = (sub_mask & value) << shift;
			sub_mask <<= shift;
			bits[offset] = (bits[offset] & ~sub_mask) | value;
		}

		field += 1;
		read += 16;
	}

	return err;
}

static void rtl8231_gpio_set_multiple(struct gpio_chip *gc,
	unsigned long *mask, unsigned long *bits)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	int read, field;
	int offset, shift;
	int sub_mask;
	int value;

	read = 0;
	field = 0;

	while (read < gc->ngpio) {
		shift = read % (8 * sizeof(*bits));
		offset = read / (8 * sizeof(*bits));
		sub_mask = (mask[offset] >> shift) & 0xffff;
		if (sub_mask) {
			value = bits[offset] >> shift;
			regmap_field_update_bits(ctrl->fields[RTL8231_FIELD_GPIO_DATA0 + field],
				sub_mask, value);
		}

		field += 1;
		read += 16;
	}
}

static int rtl8231_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtl8231_gpio_ctrl *ctrl;
	struct regmap *map;
	int field;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	map = dev_get_regmap(dev->parent, NULL);

	for (field = 0; field < RTL8231_FIELD_GPIO_MAX; field++) {
		ctrl->fields[field] = devm_regmap_field_alloc(dev, map, rtl8231_fields[field]);
		if (IS_ERR(ctrl->fields[field])) {
			dev_err(dev, "unable to allocate regmap field\n");
			return PTR_ERR(ctrl->fields[field]);
		}
	}

	/* Select GPIO functionality for all pins and set to input */
	regmap_field_write(ctrl->fields[RTL8231_FIELD_PIN_MODE0], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_GPIO_DIR0], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_PIN_MODE1], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_GPIO_DIR1], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_PIN_MODE2], 0x1f);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_GPIO_DIR2], 0x1f);

	ctrl->gc.base = -1;
	ctrl->gc.ngpio = RTL8231_MAX_GPIOS;
	ctrl->gc.label = "rtl8231-gpio";
	/* Either use parent device, or set gc.of_node explicitly */
	ctrl->gc.parent = dev;
	ctrl->gc.of_node = dev->parent->of_node;
	ctrl->gc.owner = THIS_MODULE;
	ctrl->gc.can_sleep = true;

	ctrl->gc.set = rtl8231_gpio_set;
	ctrl->gc.set_multiple = rtl8231_gpio_set_multiple;
	ctrl->gc.get = rtl8231_gpio_get;
	ctrl->gc.get_multiple = rtl8231_gpio_get_multiple;
	ctrl->gc.direction_input = rtl8231_direction_input;
	ctrl->gc.direction_output = rtl8231_direction_output;
	ctrl->gc.get_direction = rtl8231_get_direction;

	return devm_gpiochip_add_data(dev, &ctrl->gc, ctrl);
}

static struct platform_driver rtl8231_pinctrl_driver = {
	.driver = {
		.name = "rtl8231-pinctrl",
	},
	.probe = rtl8231_pinctrl_probe,
};
module_platform_driver(rtl8231_pinctrl_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek RTL8231 pin control and GPIO support");
MODULE_LICENSE("GPL v2");
