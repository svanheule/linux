// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
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
	[RTL8231_FIELD_GPIO_DATA1]  = REG_FIELD(RTL8231_GPIO_DATA1, 0 ,15),
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

static int regmap_mii_reg_read(void *context, unsigned int reg,
	unsigned int *val)
{
	struct mdio_device *mdiodev = context;
	int ret;

	ret = mdiobus_read(mdiodev->bus, mdiodev->addr, reg);
	*val = ret & 0xffff;
	dev_dbg(&mdiodev->dev, "MDIO regmap read [%x] A%02x R%02x : %04x\n",
		ret, mdiodev->addr, reg, ret & 0xffff);

	return ret < 0 ? ret : 0;
}

static int regmap_mii_reg_write(void *context, unsigned int reg,
	unsigned int val)
{
	struct mdio_device *mdiodev = context;
	int err;

	err = mdiobus_write(mdiodev->bus, mdiodev->addr, reg, val);
	dev_dbg(&mdiodev->dev, "MDIO regmap write [%x] A%02x R%02x : %04x\n",
		err, mdiodev->addr, reg, err & 0xffff);

	return err;
}

static int regmap_mii_reg_update_bits(void *context, unsigned int reg,
	unsigned int mask, unsigned int val)
{
	struct mdio_device *mdiodev = context;
	int err;

	err = mdiobus_modify(mdiodev->bus, mdiodev->addr, reg, mask, val);
	dev_dbg(&mdiodev->dev, "MDIO regmap update [%x] A%02x R%02x : %04x\n",
		err, mdiodev->addr, reg, err & 0xffff);

	return err;
}

static const struct regmap_bus regmap_mdio_bus = {
	.fast_io = true,
	.reg_write = regmap_mii_reg_write,
	.reg_read = regmap_mii_reg_read,
	.reg_update_bits = regmap_mii_reg_update_bits,
};

static int rtl8231_pin_read(struct rtl8231_gpio_ctrl *ctrl, int base,
	int offset)
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

static int rtl8231_pin_write(struct rtl8231_gpio_ctrl *ctrl,
	int base, int offset, int val)
{
	int field = base + offset / 16;
	int bit = offset % 16;

	return regmap_field_update_bits(ctrl->fields[field],
		BIT(bit), val << bit);
}

static int rtl8231_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	return rtl8231_pin_write(ctrl, RTL8231_FIELD_GPIO_DIR0, offset,
		RTL8231_GPIO_DIR_IN);
}

static int rtl8231_direction_output(struct gpio_chip *gc,
	unsigned int offset, int value)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	int err;

	err = rtl8231_pin_write(ctrl, RTL8231_FIELD_GPIO_DIR0, offset,
		RTL8231_GPIO_DIR_OUT);
	if (err)
		return err;

	return rtl8231_pin_write(ctrl, RTL8231_FIELD_GPIO_DATA0, offset,
		value);
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

static void rtl8231_gpio_set(struct gpio_chip *gc,
	unsigned int offset, int value)
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
			err = regmap_field_read(ctrl->fields[RTL8231_FIELD_GPIO_DATA0 + field], &value);
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
	int value, err;

	err = 0;
	read = 0;
	field = 0;

	while (read < gc->ngpio) {
		shift = read % (8 * sizeof(*bits));
		offset = read / (8 * sizeof(*bits));
		sub_mask = (mask[offset] >> shift) & 0xffff;
		if (sub_mask) {
			value = bits[offset] >> shift;
			regmap_field_update_bits(ctrl->fields[RTL8231_FIELD_GPIO_DATA0 + field], sub_mask, value);
		}

		field += 1;
		read += 16;
	}
}

static int rtl8231_init(struct rtl8231_gpio_ctrl *ctrl)
{
	unsigned int v;
	int err;

	err = regmap_field_read(ctrl->fields[RTL8231_FIELD_READY_CODE], &v);
	if (err) {
		dev_err(ctrl->dev, "failed to read READY_CODE\n");
		return -ENODEV;
	}
	else if (v != RTL8231_READY_CODE_VALUE) {
		dev_err(ctrl->dev, "RTL8231 not present or ready %d != %d\n",
			v, RTL8231_READY_CODE_VALUE);
		return -ENODEV;
	}

	dev_info(ctrl->dev, "RTL8231 found\n");

	/* If the device was already configured, just leave it alone */
	err = regmap_field_read(ctrl->fields[RTL8231_FIELD_LED_START], &v);
	if (err)
		return err;
	else if (v)
		return 0;

	regmap_field_write(ctrl->fields[RTL8231_FIELD_SOFT_RESET], 1);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_LED_START], 1);

	/* Select GPIO functionality for all pins and set to input */
	regmap_field_write(ctrl->fields[RTL8231_FIELD_PIN_MODE0], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_GPIO_DIR0], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_PIN_MODE1], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_GPIO_DIR1], 0xffff);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_PIN_MODE2], 0x1f);
	regmap_field_write(ctrl->fields[RTL8231_FIELD_GPIO_DIR2], 0x1f);

	return 0;
}

#define OF_COMPATIBLE_RTL8231_MDIO	"realtek,rtl8231-mdio"
#define OF_COMPATIBLE_RTL8231_I2C	"realtek,rtl8231-i2c"

static const struct of_device_id rtl8231_gpio_of_match[] = {
	{ .compatible = OF_COMPATIBLE_RTL8231_MDIO },
	{ .compatible = OF_COMPATIBLE_RTL8231_I2C },
	{},
};

MODULE_DEVICE_TABLE(of, rtl8231_gpio_of_match);

static int rtl8231_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *expander_np = NULL;
	struct regmap *map;
	struct regmap_config regmap_cfg = {};
	struct mdio_device *mdiodev;
	struct i2c_client *i2cdev;
	struct rtl8231_gpio_ctrl *ctrl;
	int field, err;
	u32 ngpios, reg_width;

	if (!np) {
		dev_err(dev, "no DT node found\n");
		return -EINVAL;
	}

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ngpios = 37;
	of_property_read_u32(np, "ngpios", &ngpios);
	if (ngpios > 37) {
		dev_err(dev, "ngpios must be smaller than or equal to 37\n");
		return -EINVAL;
	}

	regmap_cfg.val_bits = 16;
	regmap_cfg.max_register = 30;
	regmap_cfg.cache_type = REGCACHE_NONE;
	regmap_cfg.num_ranges = 0;
	regmap_cfg.use_single_read = true;
	regmap_cfg.use_single_write = true;
	regmap_cfg.reg_format_endian = REGMAP_ENDIAN_BIG;
	regmap_cfg.val_format_endian = REGMAP_ENDIAN_BIG;

	if (of_device_is_compatible(np, OF_COMPATIBLE_RTL8231_MDIO)) {
		expander_np = of_parse_phandle(np, "dev-handle", 0);
		if (!expander_np) {
			dev_err(dev, "missing dev-handle node\n");
			return -EINVAL;
		}

		mdiodev = of_mdio_find_device(expander_np);
		if (!mdiodev) {
			dev_err(dev, "failed to find MDIO device\n");
			err = -EPROBE_DEFER;
			goto err_expander_invalid;
		}

		regmap_cfg.reg_bits = 5;
		map = devm_regmap_init(dev, &regmap_mdio_bus, mdiodev,
			&regmap_cfg);
		if (IS_ERR(map)) {
			dev_err(dev, "failed to init regmap\n");
			goto err_expander_invalid;
		}
	}
	else if (of_device_is_compatible(np, OF_COMPATIBLE_RTL8231_I2C)) {
		// TODO untested
		i2cdev = of_find_i2c_device_by_node(np);
		if (IS_ERR(i2cdev)) {
			dev_err(dev, "failed to find I2C device\n");
			err = -ENODEV;
			put_device(&i2cdev->dev);
			goto err_expander_invalid;
		}

		/* Complete 7-bit I2C address is [1 0 1 0 A2 A1 A0] */
		if ((i2cdev->addr & ~(0x7)) != 0x50) {
			dev_err(dev, "invalid address\n");
			err = -EINVAL;
			put_device(&i2cdev->dev);
			goto err_expander_invalid;
		}

		if (of_property_read_u32(np, "realtek,regnum-width",
			    &reg_width) || reg_width != 1 || reg_width != 2) {
			dev_err(dev, "invalid realtek,regnum-width\n");
			err = -EINVAL;
			put_device(&i2cdev->dev);
			goto err_expander_invalid;
		}

		regmap_cfg.reg_bits = 8*reg_width;

		map = devm_regmap_init_i2c(i2cdev, &regmap_cfg);
		if (IS_ERR(map)) {
			put_device(&i2cdev->dev);
			dev_err(dev, "failed to init regmap\n");
			goto err_expander_invalid;
		}
	}
	else {
		dev_err(dev, "invalid bus type\n");
		return -ENOTSUPP;
	}

	for (field = 0; field < RTL8231_FIELD_MAX; field++) {
		ctrl->fields[field] = devm_regmap_field_alloc(dev, map,
			rtl8231_fields[field]);
		if (IS_ERR(ctrl->fields[field])) {
			dev_err(dev, "unable to allocate regmap field\n");
			err = PTR_ERR(ctrl->fields[field]);
			goto err_expander_invalid;
		}
	}

	ctrl->dev = dev;
	err = rtl8231_init(ctrl);
	if (err < 0)
		goto err_expander_invalid;

	ctrl->gc.base = -1;
	ctrl->gc.ngpio = ngpios;
	ctrl->gc.label = "rtl8231-gpio";
	ctrl->gc.parent = dev;
	ctrl->gc.owner = THIS_MODULE;
	ctrl->gc.can_sleep = true;

	ctrl->gc.set = rtl8231_gpio_set;
	ctrl->gc.set_multiple = rtl8231_gpio_set_multiple;
	ctrl->gc.get = rtl8231_gpio_get;
	ctrl->gc.get_multiple = rtl8231_gpio_get_multiple;
	ctrl->gc.direction_input = rtl8231_direction_input;
	ctrl->gc.direction_output = rtl8231_direction_output;
	ctrl->gc.get_direction = rtl8231_get_direction;

	err = devm_gpiochip_add_data(dev, &ctrl->gc, ctrl);

err_expander_invalid:
	of_node_put(expander_np);

	return err;
}

static struct platform_driver rtl8231_gpio_driver = {
	.driver = {
		.name = "rtl8231-gpio",
		.of_match_table	= rtl8231_gpio_of_match,
	},
	.probe = rtl8231_gpio_probe,
};

module_platform_driver(rtl8231_gpio_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek RTL8231 GPIO expander chip support");
MODULE_LICENSE("GPL v2");
