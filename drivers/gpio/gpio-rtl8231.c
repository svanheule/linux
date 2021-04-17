// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>
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

#define SMI_CMD_READ(addr)		(((addr) << 1) | 0x1)
#define SMI_CMD_WRITE(addr)		((addr) << 1)
#define REALTEK_SMI_ACK_RETRY_COUNT	5

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
/* SMI options */
struct rtl8231_smi_ctrl {
	struct device		*dev;
	struct gpio_desc	*mdc;
	struct gpio_desc	*mdio;
	unsigned int		clk_delay;
	spinlock_t		lock;
	unsigned short		addr;
	bool			regnum_is_16b;
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

/* Driver for SMI mode */
static inline void realtek_smi_clk_delay(struct rtl8231_smi_ctrl *ctrl)
{
	ndelay(ctrl->clk_delay);
}

static void realtek_smi_start(struct rtl8231_smi_ctrl *ctrl)
{
	/* Set GPIO pins to output mode, with initial state:
	 * SCK = 0, SDA = 1
	 */
	gpiod_direction_output(ctrl->mdc, 0);
	gpiod_direction_output(ctrl->mdio, 1);
	realtek_smi_clk_delay(ctrl);

	/* CLK 1: 0 -> 1, 1 -> 0 */
	gpiod_set_value(ctrl->mdc, 1);
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdc, 0);
	realtek_smi_clk_delay(ctrl);

	/* CLK 2: transfer start condition
	 * falling edge of MDIO on MDC high
	 */
	gpiod_set_value(ctrl->mdc, 1);
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdio, 0);
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdc, 0);
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdio, 1);
}

static void realtek_smi_stop(struct rtl8231_smi_ctrl *ctrl)
{
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdio, 0);
	gpiod_set_value(ctrl->mdc, 1);
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdio, 1);
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdc, 1);
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdc, 0);
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdc, 1);

	/* Add a click */
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdc, 0);
	realtek_smi_clk_delay(ctrl);
	gpiod_set_value(ctrl->mdc, 1);

	/* Set GPIO pins to input mode */
	gpiod_direction_input(ctrl->mdio);
	gpiod_direction_input(ctrl->mdc);
}

static void realtek_smi_write_bits(struct rtl8231_smi_ctrl *ctrl, u32 data, u32 len)
{
	for (; len > 0; len--) {
		realtek_smi_clk_delay(ctrl);

		/* Prepare data */
		gpiod_set_value(ctrl->mdio, !!(data & (1 << (len - 1))));
		realtek_smi_clk_delay(ctrl);

		/* Clocking */
		gpiod_set_value(ctrl->mdc, 1);
		realtek_smi_clk_delay(ctrl);
		gpiod_set_value(ctrl->mdc, 0);
	}
}

static void realtek_smi_read_bits(struct rtl8231_smi_ctrl *ctrl, u32 len, u32 *data)
{
	gpiod_direction_input(ctrl->mdio);

	for (*data = 0; len > 0; len--) {
		u32 u;

		realtek_smi_clk_delay(ctrl);

		/* Clocking */
		gpiod_set_value(ctrl->mdc, 1);
		realtek_smi_clk_delay(ctrl);
		u = !!gpiod_get_value(ctrl->mdio);
		gpiod_set_value(ctrl->mdc, 0);

		*data |= (u << (len - 1));
	}

	gpiod_direction_output(ctrl->mdio, 0);
}

static int realtek_smi_wait_for_ack(struct rtl8231_smi_ctrl *ctrl)
{
	int retry_cnt;

	retry_cnt = 0;
	do {
		u32 ack;

		realtek_smi_read_bits(ctrl, 1, &ack);
		if (ack == 0)
			break;

		if (++retry_cnt > REALTEK_SMI_ACK_RETRY_COUNT) {
			dev_err(ctrl->dev, "ACK timeout\n");
			return -ETIMEDOUT;
		}
	} while (1);

	return 0;
}

static int realtek_smi_write_byte(struct rtl8231_smi_ctrl *ctrl, u8 data)
{
	realtek_smi_write_bits(ctrl, data, 8);
	return realtek_smi_wait_for_ack(ctrl);
}

static int realtek_smi_write_byte_noack(struct rtl8231_smi_ctrl *ctrl, u8 data)
{
	realtek_smi_write_bits(ctrl, data, 8);
	return 0;
}

static int realtek_smi_read_byte(struct rtl8231_smi_ctrl *ctrl, u8 *data, bool read_more)
{
	u32 t;

	/* Read data */
	realtek_smi_read_bits(ctrl, 8, &t);
	*data = (t & 0xff);

	/* Send an ACK (0), or NACK (1) after last read */
	if (read_more)
		realtek_smi_write_bits(ctrl, 0x00, 1);
	else
		realtek_smi_write_bits(ctrl, 0x01, 1);


	return 0;
}

static int realtek_smi_write_regnum(struct rtl8231_smi_ctrl *ctrl, u32 regnum)
{
	int ret;

	/* Set REGNUM[7:0] */
	ret = realtek_smi_write_byte(ctrl, regnum & 0xff);
	if (ret)
		return ret;

	if (ctrl->regnum_is_16b) {
		/* Set REGNUM[15:8] */
		ret = realtek_smi_write_byte(ctrl, regnum >> 8);
		if (ret)
			return ret;
	}

	return 0;
}

static int realtek_smi_read_reg(struct rtl8231_smi_ctrl *ctrl, u32 addr, u32 *data)
{
	unsigned long flags;
	u8 lo = 0;
	u8 hi = 0;
	int step = 0;
	int ret;

	spin_lock_irqsave(&ctrl->lock, flags);

	realtek_smi_start(ctrl);

	/* Send READ command */
	ret = realtek_smi_write_byte(ctrl, SMI_CMD_READ(ctrl->addr));
	if (ret)
		goto out;
	step++;

	ret = realtek_smi_write_regnum(ctrl, addr);
	step++;

	/* Read DATA[7:0] */
	realtek_smi_read_byte(ctrl, &lo, true);
	/* Read DATA[15:8] */
	realtek_smi_read_byte(ctrl, &hi, false);

	*data = ((u32)lo) | (((u32)hi) << 8);

	ret = 0;

 out:
	realtek_smi_stop(ctrl);
	spin_unlock_irqrestore(&ctrl->lock, flags);
	if (ret)
		dev_err(ctrl->dev, "read aborted at %d: %d\n", step, ret);

	return ret;
}

static int realtek_smi_write_reg(struct rtl8231_smi_ctrl *ctrl, u32 addr, u32 data, bool ack)
{
	unsigned long flags;
	int ret;
	int step = 0;

	spin_lock_irqsave(&ctrl->lock, flags);

	realtek_smi_start(ctrl);

	/* Send WRITE command */
	ret = realtek_smi_write_byte(ctrl, SMI_CMD_WRITE(ctrl->addr));
	if (ret)
		goto out;
	step++;

	ret = realtek_smi_write_regnum(ctrl, addr);
	if (ret)
		goto out;

	/* Write DATA[7:0] */
	ret = realtek_smi_write_byte(ctrl, data & 0xff);
	if (ret)
		goto out;
	step++;

	/* Write DATA[15:8] */
	if (ack)
		ret = realtek_smi_write_byte(ctrl, data >> 8);
	else
		ret = realtek_smi_write_byte_noack(ctrl, data >> 8);
	if (ret)
		goto out;
	step++;

	ret = 0;

 out:
	realtek_smi_stop(ctrl);
	spin_unlock_irqrestore(&ctrl->lock, flags);
	if (ret)
		dev_err(ctrl->dev, "write aborted at %d: %d\n", step, ret);

	return ret;
}

/* Regmap accessors */

static int realtek_smi_write(void *ctx, u32 reg, u32 val)
{
	struct rtl8231_smi_ctrl *ctrl = ctx;

	return realtek_smi_write_reg(ctrl, reg, val, true);
}

static int realtek_smi_read(void *ctx, u32 reg, u32 *val)
{
	struct rtl8231_smi_ctrl *ctrl = ctx;

	return realtek_smi_read_reg(ctrl, reg, val);
}

static int rtl8231_smi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtl8231_smi_ctrl *ctrl;
	struct regmap_config regmap_cfg = {};
	struct regmap *map;
	u32 addr, regnum_width;
	int err;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	err = device_property_read_u32(dev, "realtek,smi-addr", &(addr));
	if (err) {
		dev_err(dev, "missing or invalid realtek,smi-addr\n");
		return err;
	} else if (addr >= 0x80) {
		dev_err(dev, "realtek,smi-addr must be smaller than %d\n", 0x80);
		return -EINVAL;
	}

	err = device_property_read_u32(dev, "realtek,smi-regnum-width", &regnum_width);
	if (err) {
		dev_err(dev, "missing or invalid realtek,smi-regnum-width\n");
		return err;
	} else if (regnum_width != 1 && regnum_width != 2) {
		dev_err(dev, "realtek,smi-regnum-width must be 1 or 2\n");
		return -EINVAL;
	}

	ctrl->addr = addr;
	ctrl->regnum_is_16b = regnum_width == 2;
	ctrl->dev = dev;
	ctrl->clk_delay = 800;
	spin_lock_init(&ctrl->lock);

	ctrl->mdc = devm_gpiod_get_optional(dev, "mdc", GPIOD_OUT_LOW);
	if (IS_ERR(ctrl->mdc))
		return PTR_ERR(ctrl->mdc);
	ctrl->mdio = devm_gpiod_get_optional(dev, "mdio", GPIOD_OUT_LOW);
	if (IS_ERR(ctrl->mdio))
		return PTR_ERR(ctrl->mdio);

	/* Set GPIO pins to input mode */
	gpiod_direction_input(ctrl->mdio);
	gpiod_direction_input(ctrl->mdc);

	rtl8231_regmap_cfg_init(&regmap_cfg, 16);
	regmap_cfg.reg_read = realtek_smi_read;
	regmap_cfg.reg_write = realtek_smi_write;
	map = devm_regmap_init(dev, NULL, ctrl, &regmap_cfg);

	if (IS_ERR(map)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(map);
	}


	return rtl8231_init(dev, map);
}

static const struct of_device_id rtl8231_smi_of_match[] = {
	{ .compatible = "realtek,rtl8231-smi" },
	{},
};
MODULE_DEVICE_TABLE(of, rtl8231_smi_of_match);

static struct platform_driver rtl8231_smi_driver = {
	.driver = {
		.name = "rtl8231-expander",
		.of_match_table	= rtl8231_smi_of_match,
	},
	.probe = rtl8231_smi_probe,
};
module_platform_driver(rtl8231_smi_driver);

/* I2C support */
static int rtl8231_i2c_probe(struct i2c_client *i2cdev)
{
	struct device *dev = &i2cdev->dev;
	struct regmap_config regmap_cfg = {};
	struct regmap *map;
	u32 reg_width;
	int err;

	err = device_property_read_u32(dev, "realtek,regnum-width", &reg_width);
	if (err) {
		dev_err(dev, "invalid or missing realtek,regnum-width\n");
		return err;
	} else if (reg_width != 1 && reg_width != 2) {
		dev_err(dev, "realtek,regnum-width must be 1 or 2\n");
		return -EINVAL;
	}

	rtl8231_regmap_cfg_init(&regmap_cfg, 8*reg_width);
	map = devm_regmap_init_i2c(i2cdev, &regmap_cfg);

	if (IS_ERR(map)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(map);
	}


	return rtl8231_init(dev, map);
}

static const struct of_device_id rtl8231_i2c_of_match[] = {
	{ .compatible = "realtek,rtl8231-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, rtl8231_i2c_of_match);

/* Upper three address bits are fixed */
static const unsigned short normal_i2c[] = {
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, I2C_CLIENT_END};

static struct i2c_driver rtl8231_i2c_driver = {
	.driver = {
		.name = "rtl8231-expander",
		.of_match_table	= rtl8231_i2c_of_match,
	},
	.probe_new = rtl8231_i2c_probe,
	.address_list = normal_i2c,
	// TODO .detect ?
};
module_i2c_driver(rtl8231_i2c_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek RTL8231 GPIO and LED expander support");
MODULE_LICENSE("GPL v2");
