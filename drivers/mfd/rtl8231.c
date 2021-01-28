// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/errno.h>
#include <linux/mfd/rtl8231.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define OF_COMPATIBLE_RTL8231_MDIO	"realtek,rtl8231-mdio"
#define OF_COMPATIBLE_RTL8231_I2C_8B	"realtek,rtl8231-i2c-8b"
#define OF_COMPATIBLE_RTL8231_I2C_16B	"realtek,rtl8231-i2c-16b"

static const struct of_device_id rtl8231_of_match[] = {
	{ .compatible = OF_COMPATIBLE_RTL8231_MDIO },
	{ .compatible = OF_COMPATIBLE_RTL8231_I2C_8B },
	{ .compatible = OF_COMPATIBLE_RTL8231_I2C_16B },
	{},
};
MODULE_DEVICE_TABLE(of, rtl8231_of_match);

static const struct reg_field rtl8231_ctrl_field_led_start =
	REG_FIELD(RTL8231_REG_FUNC0, 1, 1);
static const struct reg_field rtl8231_ctrl_field_ready_code =
	REG_FIELD(RTL8231_REG_FUNC1, 4, 9);
static const struct reg_field rtl8231_ctrl_field_soft_reset =
	REG_FIELD(RTL8231_REG_PIN_HI_CFG, 15, 15);

/*
 * TODO MFD sub-devices
 *  - pin controller
 *  - gpio controller
 *  - led controller (not yet)
 */

static int regmap_mdio_reg_read(void *context, unsigned int reg,
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

static int regmap_mdio_reg_write(void *context, unsigned int reg,
	unsigned int val)
{
	struct mdio_device *mdiodev = context;
	int err;

	err = mdiobus_write(mdiodev->bus, mdiodev->addr, reg, val);
	dev_dbg(&mdiodev->dev, "MDIO regmap write [%x] A%02x R%02x : %04x\n",
		err, mdiodev->addr, reg, err & 0xffff);

	return err;
}

static int regmap_mdio_reg_update_bits(void *context, unsigned int reg,
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
	.reg_write = regmap_mdio_reg_write,
	.reg_read = regmap_mdio_reg_read,
	.reg_update_bits = regmap_mdio_reg_update_bits,
};

static int rtl8231_init(struct device *dev, struct regmap *regmap)
{
	struct regmap_field *ready_code = NULL;
	struct regmap_field *led_start = NULL;
	struct regmap_field *soft_reset = NULL;
	unsigned int v;
	int err;

	ready_code = regmap_field_alloc(regmap, rtl8231_ctrl_field_ready_code);
	if (!ready_code) {
		err = -ENOMEM;
		goto exit_free_fields;
	}

	led_start = regmap_field_alloc(regmap, rtl8231_ctrl_field_led_start);
	if (!led_start) {
		err = -ENOMEM;
		goto exit_free_fields;
	}

	soft_reset = regmap_field_alloc(regmap, rtl8231_ctrl_field_soft_reset);
	if (!soft_reset) {
		err = -ENOMEM;
		goto exit_free_fields;
	}

	err = regmap_field_read(ready_code, &v);
	if (err) {
		dev_err(dev, "failed to read READY_CODE\n");
		goto exit_free_fields;
	}
	else if (v != RTL8231_FUNC1_READY_CODE_VALUE) {
		dev_err(dev, "RTL8231 not present or ready %d != %d\n",
			v, RTL8231_FUNC1_READY_CODE_VALUE);
		err = -ENODEV;
		goto exit_free_fields;
	}

	dev_info(dev, "RTL8231 found\n");

	/*
	 * If the device was already configured, just leave it alone.
	 * This prevents loosing configurations loaded by the bootloader.
	 */
	err = regmap_field_read(led_start, &v);
	if (err || v)
		goto exit_free_fields;

	err = regmap_field_write(soft_reset, 1);
	if (err)
		goto exit_free_fields;

	err = regmap_field_write(led_start, 1);

exit_free_fields:
	regmap_field_free(ready_code);
	regmap_field_free(led_start);
	regmap_field_free(soft_reset);

	return err;
}

static const struct pinctrl_pin_desc rtl8231_pins[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
	PINCTRL_PIN(8, "gpio8"),
	PINCTRL_PIN(9, "gpio9"),
	PINCTRL_PIN(10, "gpio10"),
	PINCTRL_PIN(11, "gpio11"),
	PINCTRL_PIN(12, "gpio12"),
	PINCTRL_PIN(13, "gpio13"),
	PINCTRL_PIN(14, "gpio14"),
	PINCTRL_PIN(15, "gpio15"),
	PINCTRL_PIN(16, "gpio16"),
	PINCTRL_PIN(17, "gpio17"),
	PINCTRL_PIN(18, "gpio18"),
	PINCTRL_PIN(19, "gpio19"),
	PINCTRL_PIN(20, "gpio20"),
	PINCTRL_PIN(21, "gpio21"),
	PINCTRL_PIN(22, "gpio22"),
	PINCTRL_PIN(23, "gpio23"),
	PINCTRL_PIN(24, "gpio24"),
	PINCTRL_PIN(25, "gpio25"),
	PINCTRL_PIN(26, "gpio26"),
	PINCTRL_PIN(27, "gpio27"),
	PINCTRL_PIN(28, "gpio28"),
	PINCTRL_PIN(29, "gpio29"),
	PINCTRL_PIN(30, "gpio30"),
	PINCTRL_PIN(31, "gpio31"),
	PINCTRL_PIN(32, "gpio32"),
	PINCTRL_PIN(33, "gpio33"),
	PINCTRL_PIN(34, "gpio34"),
	PINCTRL_PIN(35, "gpio35"),
	PINCTRL_PIN(36, "gpio36"),
};

/*
 * Single LED scan matrix, group A (ports 0-11)
 * Row and column pins of scan matrix for LED0, LED1, LED2
 * Columns control LED0/1/2 for ports [n] and [n+6]
 *          L0[n]    L1[n]    L2[n]    L0[n+6]  L1[n+6]  L2[n+6]
 *           |        |        |        |        |        |
 *  P0/P6  --X--------X--------X--------X--------X--------X (3)
 *           |        |        |        |        |        |
 *  P1/P7  --X--------X--------X--------X--------X--------X (4)
 *           |        |        |        |        |        |
 *  P2/P8  --X--------X--------X--------X--------X--------X (5)
 *           |        |        |        |        |        |
 *  P3/P9  --X--------X--------X--------X--------X--------X (6)
 *           |        |        |        |        |        |
 *  P4/P10 --X--------X--------X--------X--------X--------X (7)
 *           |        |        |        |        |        |
 *  P5/P11 --X--------X--------X--------X--------X--------X (8)
 *          (0)      (1)      (2)      (9)     (10)     (11)
 */
static u8 single_color_rows_p0_p11[6] = {3, 4, 5, 6, 7, 8};
static u8 single_color_cols_p0_p11[2][3] = {{0, 1, 2}, {9, 10, 11}};

/*
 * Single LED scan matrix, group B (ports 12-23)
 * Row and column pins of scan matrix for LED0, LED1, LED2
 * Columns control LED0/1/2 for ports (N) and (N+6)
 *           L0[n]    L1[n]    L2[n]    L0[n+6]  L1[n+6]  L2[n+6]
 *            |        |        |        |        |        |
 *  P12/P18 --X--------X--------X--------X--------X--------X (15)
 *            |        |        |        |        |        |
 *  P13/P19 --X--------X--------X--------X--------X--------X (16)
 *            |        |        |        |        |        |
 *  P14/P20 --X--------X--------X--------X--------X--------X (17)
 *            |        |        |        |        |        |
 *  P15/P21 --X--------X--------X--------X--------X--------X (18)
 *            |        |        |        |        |        |
 *  P16/P22 --X--------X--------X--------X--------X--------X (19)
 *            |        |        |        |        |        |
 *  P17/P23 --X--------X--------X--------X--------X--------X (20)
 *          (12)     (13)     (14)    (21)      (22)     (23)
 */
static u8 single_color_rows_p12_p23[6] = {15, 16, 17, 18, 19, 20};
static u8 single_color_cols_p12_p23[2][3] = {{12, 13, 14}, {21, 22, 23}};

/*
 * Single color mode group C (ports 24-31)
 * Group C is actually a bi-color group, so has pins to select
 * the port and LED polarity (color). Only two colors per port.
 *           P24     P25  ...  P30     P31
 *            |       |         |       |
 *  LED POL --X-------X---/\/---X-------X (15)
 *          (24)    (25)  ... (31)    (32)
 */
static u8 single_color_port_p24_p32[8] = {24, 25, 26, 27, 29, 30, 31, 32};
static u8 single_color_polarity_p24_p32 = 28;

static int single_color_matrix_pins(unsigned int port, unsigned int led,
	unsigned int *row, unsigned int *col)
{
	if (led > 2)
		return -EINVAL;

	if (port < 12) {
		*row = single_color_rows_p0_p11[port % 6];
		*col = single_color_cols_p0_p11[port / 6][led];
	}
	else if (port < 24) {
		port -= 12;
		*row = single_color_rows_p12_p23[port % 6];
		*col = single_color_cols_p12_p23[port / 6][led];
	}
	else if (port < 32 && led < 2) {
		port -= 24;
		*row = single_color_polarity_p24_p32;
		*col = single_color_port_p24_p32[port];
	}
	else {
		return -EINVAL;
	}

	return 0;
}

/*
 * Bi-color mode group A
 * Scan matrix for ports 0-11
 */
static u8 bi_color_port_p0_p11[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static u8 bi_color_polarity_p0_p11 = 12;

/*
 * Bi-color mode group B
 * Scan matrix for ports 12-17
 */
static u8 bi_color_port_p12_p17[6] = {23, 24, 25, 26, 27, 28};
static u8 bi_color_polarity_p12_p17 = 21;

/*
 * Bi-color mode group C
 * Scan matrix for ports 18-23
 */
static u8 bi_color_port_p18_p23[6] = {29, 30, 31, 32, 33, 34};
static u8 bi_color_polarity_p18_p23 = 22;

/*
 * Bi-color mode single color group
 * Independent LED to provide the third color for ports 0-24.
 *     P[n]     P[n+6]   P[n+12]  P[n+18]
 *       |        |        |        |
 *  +0 --X--------X--------X--------X (15)
 *       |        |        |        |
 *  +1 --X--------X--------X--------X (16)
 *       |        |        |        |
 *  +2 --X--------X--------X--------X (17)
 *       |        |        |        |
 *  +3 --X--------X--------X--------X (18)
 *       |        |        |        |
 *  +4 --X--------X--------X--------X (19)
 *       |        |        |        |
 *  +6 --X--------X--------X--------X (20)
 *     (13)     (14)     (21)     (22)
 */
static u8 bi_color_rows_p0_p23[6] = {15, 16, 17, 18, 19, 20};
static u8 bi_color_cols_p0_p23[4] = {13, 14, 21, 22};

static int bi_color_matrix_pins(unsigned int port, unsigned int led,
	unsigned int *row, unsigned int *col)
{
	if (port > 23)
		return -EINVAL;

	if (led < 2) {
		if (port < 12) {
			*row = bi_color_polarity_p0_p11;
			*col = bi_color_port_p0_p11[port];
		}
		else if (port < 18) {
			port -= 12;
			*row = bi_color_polarity_p12_p17;
			*col = bi_color_port_p12_p17[port];
		}
		else {
			port -= 18;
			*row = bi_color_polarity_p18_p23;
			*col = bi_color_port_p18_p23[port];
		}
	}
	else if (led == 2) {
		*row = bi_color_rows_p0_p23[port % 6];
		*col = bi_color_cols_p0_p23[port / 6];
	}
	else {
		return -EINVAL;
	}

	return 0;
}

static struct pinctrl_desc rtl8231_pinctrl_desc = {
	.name = "rtl8231",
	.pins = rtl8231_pins,
	.npins = ARRAY_SIZE(rtl8231_pins),
	.owner = THIS_MODULE,
};

static int rtl8231_pinctrl_probe(struct device *dev, struct regmap *regmap)
{
	int err;
	struct pinctrl_dev *pctl;

	err = devm_pinctrl_register_and_init(dev, &rtl8231_pinctrl_desc,
		NULL, &pctl);

	if (err)
		return err;

	return pinctrl_enable(pctl);
}

static int rtl8231_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *expander_np = NULL;
	struct regmap *regmap;
	struct regmap_config regmap_cfg = {};
	struct mdio_device *mdiodev;
	int err;

	dev_info(dev, "probing for RTL8231\n");

	// Use fwnode instead of device_node?
	if (!np) {
		dev_err(dev, "no DT node found\n");
		return -EINVAL;
	}

	regmap_cfg.val_bits = 16;
	regmap_cfg.max_register = RTL8231_REG_COUNT;
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

		regmap = devm_regmap_init(dev, &regmap_mdio_bus, mdiodev,
			&regmap_cfg);
		if (IS_ERR(regmap)) {
			dev_err(dev, "failed to init regmap\n");
			goto err_expander_invalid;
		}
	}
	else {
		dev_err(dev, "invalid bus type\n");
		return -ENOTSUPP;
	}

	err = rtl8231_init(dev, regmap);

err_expander_invalid:
	of_node_put(expander_np);

	return err;
}

static struct platform_driver rtl8231_gpio_driver = {
	.driver = {
		.name = "rtl8231",
		.of_match_table	= rtl8231_of_match,
	},
	.probe = rtl8231_probe,
};

module_platform_driver(rtl8231_gpio_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek RTL8231 expander chip support");
MODULE_LICENSE("GPL v2");
