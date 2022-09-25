// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "core.h"
#include "pinmux.h"

/**
 * struct rtl_swcore_mux_desc - switchcore pin group information
 *
 * Pins are frequently muxed between alternative functions, but the control
 * bits for the muxes are scattered throught the switchcore's register space.
 * Provide a regmap-based interface to flexibly manage these mux fields, which
 * may vary in size and do not always provide a GPIO function.
 *
 * @name: name to identify the pin group
 * @field_desc: description of the register field with mux control bits
 * @functions: NULL terminated array of function names
 * @pins: array of numbers of the pins in this group
 * @npins: number of pins in this group
 */
struct rtl_swcore_mux_desc {
	const char *name;
	struct reg_field field;
	const unsigned int *pins;
	unsigned int npins;
};

#define SWITCHCORE_MUX(_name, _field, _pins)		{	\
		.name = (_name),				\
		.field = _field,				\
		.pins = (_pins),				\
		.npins = ARRAY_SIZE(_pins),			\
	}

/**
 * struct rtl_swcore_mux_setting - stored mux setting
 *
 * @mux: pointer to the mux descriptor
 * @value: value to write in the mux's register field to apply this setting
 */
struct rtl_swcore_mux_setting {
	const struct rtl_swcore_mux_desc *mux;
	unsigned int value;
};

/**
 * struct rtl_swcore_function_desc - switchcore function information
 *
 * @name: name of this function
 * @settings: list of mux settings that enable this function on said mux
 * @nsettings: length of the @settings list
 */
struct rtl_swcore_function_desc {
	const char *name;
	const struct rtl_swcore_mux_setting *settings;
	unsigned int nsettings;
};

#define SWITCHCORE_FUNCTION(_name, _settings)		{	\
		.name = (_name),				\
		.settings = (_settings),			\
		.nsettings = ARRAY_SIZE(_settings),		\
	}

struct rtl_swcore_config {
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
	const struct rtl_swcore_function_desc *functions;
	unsigned int nfunctions;
	const struct rtl_swcore_mux_desc **groups;
	unsigned int ngroups;
};

struct rtl_swcore_pinctrl {
	struct pinctrl_desc pdesc;
	struct device *dev;
	const struct rtl_swcore_config *config;
	struct regmap_field **mux_fields;
};

/*
 * RTL838x chips come in LQFP packages with 216 pins. Pins are indexed
 * counter-clockwise, starting with pin 1 at the bottom left.
 */
static const struct pinctrl_pin_desc rtl838x_swcore_pins[] = {
	/* JTAG pins */
	PINCTRL_PIN(28, "tck"),
	PINCTRL_PIN(29, "tms"),
	PINCTRL_PIN(30, "tdo"),
	PINCTRL_PIN(31, "tdi"),
	PINCTRL_PIN(32, "ntrst"),
	/* aux MDIO bus pins */
	PINCTRL_PIN(110, "aux-mdio"),
	PINCTRL_PIN(111, "aux-mdc"),
	/* system LED pin */
	PINCTRL_PIN(113, "sys-led"),
	/* UART1/SPI slave pins */
	PINCTRL_PIN(116, "uart1-rx"),
	PINCTRL_PIN(117, "uart1-tx"),
};

static const unsigned int rtl838x_jtag_pins[] = {28, 29, 30, 31, 32};
static const unsigned int rtl838x_aux_mdio_pins[] = {110, 111};
static const unsigned int rtl838x_sys_led_pins[] = {113};
static const unsigned int rtl838x_uart1_pins[] = {116, 117};

static const struct rtl_swcore_mux_desc rtl838x_mux_jtag =
	SWITCHCORE_MUX("jtag", REG_FIELD(0x1000, 2, 3), rtl838x_jtag_pins);

static const struct rtl_swcore_mux_desc rtl838x_mux_aux_mdio =
	SWITCHCORE_MUX("aux-mdio", REG_FIELD(0xa0e0, 0, 0), rtl838x_aux_mdio_pins);

static const struct rtl_swcore_mux_desc rtl838x_mux_sys_led =
	SWITCHCORE_MUX("sys-led", REG_FIELD(0xa000, 15, 15), rtl838x_sys_led_pins);

static const struct rtl_swcore_mux_desc rtl838x_mux_uart1 =
	SWITCHCORE_MUX("uart1", REG_FIELD(0x1000, 4, 4), rtl838x_uart1_pins);

static const struct rtl_swcore_mux_desc *rtl838x_groups[] = {
	&rtl838x_mux_jtag,
	&rtl838x_mux_aux_mdio,
	&rtl838x_mux_sys_led,
	&rtl838x_mux_uart1,
};

static const struct rtl_swcore_mux_setting rtl838x_gpio_settings[] = {
	{&rtl838x_mux_jtag, 2},
	{&rtl838x_mux_aux_mdio, 0},
	{&rtl838x_mux_sys_led, 0},
};
static const struct rtl_swcore_mux_setting rtl838x_aux_mdio_settings[] = {
	{&rtl838x_mux_aux_mdio, 1},
};
static const struct rtl_swcore_mux_setting rtl838x_sys_led_settings[] = {
	{&rtl838x_mux_sys_led, 1},
};
static const struct rtl_swcore_mux_setting rtl838x_uart1_settings[] = {
	{&rtl838x_mux_uart1, 1},
};
static const struct rtl_swcore_mux_setting rtl838x_spi_slave_settings[] = {
	{&rtl838x_mux_uart1, 0},
};

static const struct rtl_swcore_function_desc rtl838x_functions[] = {
	SWITCHCORE_FUNCTION("gpio", rtl838x_gpio_settings),
	SWITCHCORE_FUNCTION("aux-mdio", rtl838x_aux_mdio_settings),
	SWITCHCORE_FUNCTION("sys-led", rtl838x_sys_led_settings),
	SWITCHCORE_FUNCTION("uart1", rtl838x_uart1_settings),
	SWITCHCORE_FUNCTION("spi-slave", rtl838x_spi_slave_settings),
};

static const struct rtl_swcore_config rtl838x_config = {
	.pins = rtl838x_swcore_pins,
	.npins = ARRAY_SIZE(rtl838x_swcore_pins),
	.functions = rtl838x_functions,
	.nfunctions = ARRAY_SIZE(rtl838x_functions),
	.groups = rtl838x_groups,
	.ngroups = ARRAY_SIZE(rtl838x_groups),
};

/*
 * RTL839x chips are in BGA packages with 26×26 positions. Board designs number
 * these as 1..26 for the rows, and A..AF for the columns, with position A1 in
 * the bottom left corner. Letters I, O, Q, S, X, and Z are skipped; presumably
 * to avoid ambiguities.
 * This gives a total of 676 positions. Note that not all positions will
 * actually have a pad, and many pads will be used for power.
 *
 * Index pins using (ROW + 26×COL), where ROW and COL mapped as:
 *   - ROW: {1..26} -> {0..25}
 *   - COL: {A..AF} -> {0..25}
 *
 *     ROW |  1  2  3  4  5  6  7  8  9 10 11 12 13
 *     COL |  A  B  C  D  E  F  G  H  J  K  L  M  N
 *   ------|---------------------------------------
 *   INDEX |  0  1  2  3  4  5  6  7  8  9 10 11 12
 *
 *     ROW | 14 15 16 17 18 19 20 21 22 23 24 25 26
 *     COL |  P  R  T  U  V  W  Y AA AB AC AD AE AF
 *   ------|---------------------------------------
 *   INDEX | 13 14 15 16 17 18 19 20 21 22 23 24 25
 *
 * Since there are no datasheets available, use a virtual pin range starting at
 * 676 for pins with unknown positions. When actual pin positions are found
 * (if ever), these can the be mapped to their real values.
 */
#define RTL839X_VPIN(num)		(26 * 26 + (num))

static const struct pinctrl_pin_desc rtl839x_swcore_pins[] = {
	/* sys-led, or gpio0 */
	PINCTRL_PIN(RTL839X_VPIN(0), "sys-led"),
	/* aux mdio clock, or gpio2 */
	PINCTRL_PIN(RTL839X_VPIN(2), "aux-mdc"),
	/* aux mdio data, or gpio3 */
	PINCTRL_PIN(RTL839X_VPIN(3), "aux-mdio"),
	/* JTAG tck, UART1 cts, or gpio4 */
	PINCTRL_PIN(RTL839X_VPIN(4), "tck"),
	/* JTAG tdi, UART1 rx (?), or gpio6 */
	PINCTRL_PIN(RTL839X_VPIN(6), "tdi"),
	/* JTAG tdo, UART1 tx (?), or gpio7 */
	PINCTRL_PIN(RTL839X_VPIN(7), "tdo"),
};

static const unsigned int rtl839x_jtag_pins[] = {
	RTL839X_VPIN(4), RTL839X_VPIN(5), RTL839X_VPIN(6), RTL839X_VPIN(7)
};
static const unsigned int rtl839x_aux_mdio_pins[] = {
	RTL839X_VPIN(2), RTL839X_VPIN(3)
};
static const unsigned int rtl839x_sys_led_pins[] = {RTL839X_VPIN(0)};

static const struct rtl_swcore_mux_desc rtl839x_mux_jtag =
	SWITCHCORE_MUX("jtag", REG_FIELD(0x000a, 0, 1), rtl839x_jtag_pins);

static const struct rtl_swcore_mux_desc rtl839x_mux_aux_mdio =
	SWITCHCORE_MUX("aux-mdio", REG_FIELD(0x00e4, 18, 20), rtl839x_aux_mdio_pins);

static const struct rtl_swcore_mux_desc rtl839x_mux_sys_led =
	SWITCHCORE_MUX("sys-led", REG_FIELD(0x00e4, 14, 14), rtl839x_sys_led_pins);

static const struct rtl_swcore_mux_desc *rtl839x_groups[] = {
	&rtl839x_mux_jtag,
	&rtl839x_mux_aux_mdio,
	&rtl839x_mux_sys_led,
};

static const struct rtl_swcore_mux_setting rtl839x_gpio_settings[] = {
	{&rtl839x_mux_jtag, 2},
	{&rtl839x_mux_aux_mdio, 0},
	{&rtl839x_mux_sys_led, 0},
};
static const struct rtl_swcore_mux_setting rtl839x_aux_mdio_settings[] = {
	{&rtl839x_mux_aux_mdio, 4},
};
static const struct rtl_swcore_mux_setting rtl839x_sys_led_settings[] = {
	{&rtl839x_mux_sys_led, 1},
};
static const struct rtl_swcore_mux_setting rtl839x_uart1_settings[] = {
	{&rtl839x_mux_jtag, 1},
};

static const struct rtl_swcore_function_desc rtl839x_functions[] = {
	SWITCHCORE_FUNCTION("gpio", rtl839x_gpio_settings),
	SWITCHCORE_FUNCTION("aux-mdio", rtl839x_aux_mdio_settings),
	SWITCHCORE_FUNCTION("sys-led", rtl839x_sys_led_settings),
	SWITCHCORE_FUNCTION("uart1", rtl839x_uart1_settings),
};

static const struct rtl_swcore_config rtl839x_config = {
	.pins = rtl839x_swcore_pins,
	.npins = ARRAY_SIZE(rtl839x_swcore_pins),
	.functions = rtl839x_functions,
	.nfunctions = ARRAY_SIZE(rtl839x_functions),
	.groups = rtl839x_groups,
	.ngroups = ARRAY_SIZE(rtl839x_groups),
};

/* TODO RTL9300 */

/* TODO RTL9310 */

static int rtl_swcore_group_count(struct pinctrl_dev *pctldev)
{
	struct rtl_swcore_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->config->ngroups;
}

static const char * rtl_swcore_group_name(struct pinctrl_dev *pctldev,
	unsigned int selector)
{
	struct rtl_swcore_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	return priv->config->groups[selector]->name;
};

static int rtl_swcore_group_pins(struct pinctrl_dev *pctldev,
	unsigned int selector, const unsigned int **pins,
	unsigned int *num_pins)
{
	struct rtl_swcore_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	*pins = priv->config->groups[selector]->pins;
	*num_pins = priv->config->groups[selector]->npins;

	return 0;
}

static int rtl_swcore_set_mux(struct pinctrl_dev *pctldev,
	unsigned int selector, unsigned int group)
{
	struct rtl_swcore_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct rtl_swcore_function_desc *function;
	const struct rtl_swcore_mux_setting *setting;
	const struct rtl_swcore_mux_desc *mux;
	unsigned int i;

	dev_info(priv->dev, "requesting selector %d, group %d\n", selector, group);

	function = &priv->config->functions[selector];
	mux = priv->config->groups[group];

	for (i = 0; i < function->nsettings; i++) {
		setting = &function->settings[i];
		if (setting->mux == mux) {
			dev_info(priv->dev, "set mux %s to function %s (%d)\n",
				mux->name, function->name, setting->value);
			return regmap_field_write(priv->mux_fields[group], setting->value);
		}
	}

	/* Should never hit this, unless something was misconfigured */
	return -ENODEV;
}

static const struct pinctrl_ops rtl_swcore_pinctrl_ops = {
	.get_groups_count = rtl_swcore_group_count,
	.get_group_name = rtl_swcore_group_name,
	.get_group_pins = rtl_swcore_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinmux_ops rtl_swcore_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = rtl_swcore_set_mux,
	.strict = true,
};

static int rtl_swcore_functions_init(struct pinctrl_dev *pctl, struct rtl_swcore_pinctrl *priv)
{
	const struct rtl_swcore_function_desc *function;
	unsigned int ngroups;
	const char **groups;
	unsigned int f_idx;
	unsigned int g_idx;

	for (f_idx = 0; f_idx < priv->config->nfunctions; f_idx++) {
		function = &priv->config->functions[f_idx];
		ngroups = function->nsettings;

		dev_info(priv->dev, "found %d groups for function %s\n", ngroups, function->name);

		groups = devm_kcalloc(priv->dev, ngroups, sizeof(*groups), GFP_KERNEL);
		if (!groups)
			return -ENOMEM;

		for (g_idx = 0; g_idx < ngroups; g_idx++)
			groups[g_idx] = function->settings[g_idx].mux->name;

		pinmux_generic_add_function(pctl, function->name, groups, ngroups, NULL);
	}

	return 0;
}

static const struct of_device_id of_rtl_swcore_pinctrl_match[] = {
	{
		.compatible = "realtek,maple-pinctrl",
		.data = &rtl838x_config,
	},
	{
		.compatible = "realtek,cypress-pinctrl",
		.data = &rtl839x_config,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_rtl_swcore_pinctrl_match);

static int rtl_swcore_pinctrl_probe(struct platform_device *pdev)
{
	const struct rtl_swcore_config *config;
	struct rtl_swcore_pinctrl *priv;
	struct device *dev = &pdev->dev;
	struct pinctrl_dev *pctldev;
	struct regmap_field *field;
	struct regmap *regmap;
	int mux;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	config = of_device_get_match_data(dev);
	if (!config)
		return dev_err_probe(dev, -EINVAL, "no config\n");

	regmap = device_node_to_regmap(of_get_parent(dev_of_node(dev)));
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "failed to find parent regmap\n");

	priv->dev = dev;
	priv->config = config;
	priv->pdesc.name = "realtek-switchcore-pinctrl";
	priv->pdesc.owner = THIS_MODULE;
	priv->pdesc.pctlops = &rtl_swcore_pinctrl_ops;
	priv->pdesc.pmxops = &rtl_swcore_pinmux_ops;
	priv->pdesc.pins = config->pins;
	priv->pdesc.npins = config->npins;

	priv->mux_fields = devm_kcalloc(dev, config->ngroups, sizeof(*priv->mux_fields),
		GFP_KERNEL);
	if (!priv->mux_fields)
		return -ENOMEM;

	for (mux = 0; mux < config->ngroups; mux++) {
		field = devm_regmap_field_alloc(dev, regmap, config->groups[mux]->field);
		if (IS_ERR(field))
			return PTR_ERR(field);

		priv->mux_fields[mux] = field;
	}

	err = devm_pinctrl_register_and_init(dev, &priv->pdesc, priv, &pctldev);
	if (err)
		return dev_err_probe(dev, err, "failed to register\n");

	err = rtl_swcore_functions_init(pctldev, priv);
	if (err)
		return dev_err_probe(dev, err, "failed to generate function list\n");

	err = pinctrl_enable(pctldev);
	if (err)
		return dev_err_probe(dev, err, "failed to enable\n");

	return 0;
};

static struct platform_driver rtl_swcore_pinctrl_driver = {
	.driver = {
		.name = "realtek-switchcore-pinctrl",
		.of_match_table = of_rtl_swcore_pinctrl_match
	},
	.probe = rtl_swcore_pinctrl_probe,
};
module_platform_driver(rtl_swcore_pinctrl_driver);
