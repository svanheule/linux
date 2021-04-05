// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

/* RTL8231 registers for LED control */
#define RTL8231_FUNC0			0x00
#define RTL8231_FUNC1			0x01
#define RTL8231_PIN_MODE0		0x02
#define RTL8231_PIN_MODE1		0x03
#define RTL8231_PIN_HI_CFG		0x04
#define RTL8231_GPIO_DIR0		0x05
#define RTL8231_GPIO_DIR1		0x06
#define RTL8231_GPIO_INVERT0		0x07
#define RTL8231_GPIO_INVERT1		0x08
#define RTL8231_GPIO_DATA0		0x1c
#define RTL8231_GPIO_DATA1		0x1d
#define RTL8231_GPIO_DATA2		0x1e

#define RTL8231_READY_CODE_VALUE	0x37
#define RTL8231_GPIO_DIR_IN		1
#define RTL8231_GPIO_DIR_OUT		0

#define RTL8231_MAX_GPIOS		37

enum rtl8231_regfield {
	RTL8231_FIELD_LED_START,
	RTL8231_FIELD_READY_CODE,
	RTL8231_FIELD_SOFT_RESET,
	RTL8231_FIELD_PIN_MODE0,
	RTL8231_FIELD_PIN_MODE1,
	RTL8231_FIELD_PIN_MODE2,
	RTL8231_FIELD_GPIO_DIR0,
	RTL8231_FIELD_GPIO_DIR1,
	RTL8231_FIELD_GPIO_DIR2,
	RTL8231_FIELD_GPIO_DATA0,
	RTL8231_FIELD_GPIO_DATA1,
	RTL8231_FIELD_GPIO_DATA2,
	RTL8231_FIELD_MAX
};

static const struct reg_field rtl8231_fields[RTL8231_FIELD_MAX] = {
	[RTL8231_FIELD_LED_START]   = REG_FIELD(RTL8231_FUNC0, 1, 1),
	[RTL8231_FIELD_READY_CODE]  = REG_FIELD(RTL8231_FUNC1, 4, 9),
	[RTL8231_FIELD_SOFT_RESET]  = REG_FIELD(RTL8231_PIN_HI_CFG, 15, 15),
	[RTL8231_FIELD_PIN_MODE0]   = REG_FIELD(RTL8231_PIN_MODE0, 0, 15),
	[RTL8231_FIELD_PIN_MODE1]   = REG_FIELD(RTL8231_PIN_MODE1, 0, 15),
	[RTL8231_FIELD_PIN_MODE2]   = REG_FIELD(RTL8231_PIN_HI_CFG, 0, 4),
	[RTL8231_FIELD_GPIO_DIR0]   = REG_FIELD(RTL8231_GPIO_DIR0, 0, 15),
	[RTL8231_FIELD_GPIO_DIR1]   = REG_FIELD(RTL8231_GPIO_DIR1, 0, 15),
	[RTL8231_FIELD_GPIO_DIR2]   = REG_FIELD(RTL8231_PIN_HI_CFG, 5, 9),
	[RTL8231_FIELD_GPIO_DATA0]  = REG_FIELD(RTL8231_GPIO_DATA0, 0, 15),
	[RTL8231_FIELD_GPIO_DATA1]  = REG_FIELD(RTL8231_GPIO_DATA1, 0, 15),
	[RTL8231_FIELD_GPIO_DATA2]  = REG_FIELD(RTL8231_GPIO_DATA2, 0, 4),
};

/**
 * struct rtl8231_gpio_ctrl - Control data for an RTL8231 chip
 *
 * @gc: Associated gpio_chip instance
 * @dev
 * @fields
 */
struct rtl8231_gpio_ctrl {
	struct gpio_chip gc;
	struct device *dev;
	struct regmap_field *fields[RTL8231_FIELD_MAX];
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

static int rtl8231_probe_gpio(struct rtl8231_gpio_ctrl *ctrl)
{
	u32 ngpios = RTL8231_MAX_GPIOS;

	device_property_read_u32(ctrl->dev, "ngpios", &ngpios);
	if (ngpios > RTL8231_MAX_GPIOS) {
		dev_err(ctrl->dev, "ngpios can be at most %d\n", RTL8231_MAX_GPIOS);
		return -EINVAL;
	}

	ctrl->gc.base = -1;
	ctrl->gc.ngpio = ngpios;
	ctrl->gc.label = "rtl8231-gpio";
	ctrl->gc.parent = ctrl->dev;
	ctrl->gc.owner = THIS_MODULE;
	ctrl->gc.can_sleep = true;

	ctrl->gc.set = rtl8231_gpio_set;
	ctrl->gc.set_multiple = rtl8231_gpio_set_multiple;
	ctrl->gc.get = rtl8231_gpio_get;
	ctrl->gc.get_multiple = rtl8231_gpio_get_multiple;
	ctrl->gc.direction_input = rtl8231_direction_input;
	ctrl->gc.direction_output = rtl8231_direction_output;
	ctrl->gc.get_direction = rtl8231_get_direction;

	return devm_gpiochip_add_data(ctrl->dev, &ctrl->gc, ctrl);
}

static int rtl8231_init(struct device *dev, struct regmap *map)
{
	struct rtl8231_gpio_ctrl *ctrl;
	unsigned int v;
	int field, err;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->dev = dev;

	for (field = 0; field < RTL8231_FIELD_MAX; field++) {
		ctrl->fields[field] = devm_regmap_field_alloc(dev, map, rtl8231_fields[field]);
		if (IS_ERR(ctrl->fields[field])) {
			dev_err(dev, "unable to allocate regmap field\n");
			return PTR_ERR(ctrl->fields[field]);
		}
	}

	err = regmap_field_read(ctrl->fields[RTL8231_FIELD_READY_CODE], &v);
	if (err) {
		dev_err(dev, "failed to read READY_CODE\n");
		return err;
	} else if (v != RTL8231_READY_CODE_VALUE) {
		dev_err(dev, "RTL8231 not present or ready 0x%x != 0x%x\n",
			v, RTL8231_READY_CODE_VALUE);
		return -ENODEV;
	}

	dev_info(dev, "RTL8231 found\n");

	/* If the device was already configured, just leave it alone */
	err = regmap_field_read(ctrl->fields[RTL8231_FIELD_LED_START], &v);
	if (err)
		return err;
	else if (v) {
		dev_info(dev, "already initialised\n");
		return 0;
	}

	regmap_field_write(ctrl->fields[RTL8231_FIELD_SOFT_RESET], 1);
	mdelay(1);

	/* Select GPIO functionality for all pins and set to input */
	regmap_field_write(ctrl->fields[RTL8231_FIELD_PIN_MODE0], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_GPIO_DIR0], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_PIN_MODE1], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_GPIO_DIR1], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_PIN_MODE2], 0x1f);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_GPIO_DIR2], 0x1f);

	regmap_field_write(ctrl->fields[RTL8231_FIELD_LED_START], 1);

	return rtl8231_probe_gpio(ctrl);
}

static void rtl8231_regmap_cfg_init(struct regmap_config *cfg, int reg_bits)
{
	cfg->val_bits = 16;
	cfg->max_register = 0x1e;
	cfg->cache_type = REGCACHE_NONE;
	cfg->num_ranges = 0;
	cfg->use_single_read = true;
	cfg->use_single_write = true;
	cfg->reg_format_endian = REGMAP_ENDIAN_BIG;
	cfg->val_format_endian = REGMAP_ENDIAN_BIG;
	cfg->reg_bits = reg_bits;
}

/* MDIO mode */
static int rtl8231_mdio_probe(struct mdio_device *mdiodev)
{
	struct device *dev = &mdiodev->dev;
	struct regmap *map;
	struct regmap_config regmap_cfg = {};

	rtl8231_regmap_cfg_init(&regmap_cfg, 5);
	map = devm_regmap_init_mdio(mdiodev, &regmap_cfg);

	if (IS_ERR(map)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(map);
	}

	return rtl8231_init(dev, map);
}

static const struct of_device_id rtl8231_mdio_of_match[] = {
	{ .compatible = "realtek,rtl8231-mdio" },
	{},
};
MODULE_DEVICE_TABLE(of, rtl8231_mdio_of_match);

static struct mdio_driver rtl8231_mdio_driver = {
	.mdiodrv.driver = {
		.name = "rtl8231-expander",
		.of_match_table	= rtl8231_mdio_of_match,
	},
	.probe = rtl8231_mdio_probe,
};
mdio_module_driver(rtl8231_mdio_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek RTL8231 GPIO and LED expander support");
MODULE_LICENSE("GPL v2");
