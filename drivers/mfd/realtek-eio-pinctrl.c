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

#define REALTEK_EIO_GLOBAL_CTRL			0x0

/*
 * Management of external RTL8231 GPIO expanders.
 * One RTL8231's GPIO registers can be shadowed to the internal GPIO_DIR
 * and GPIO_DAT registers.
 */
#define RTL8380_EIO_GPIO_CTRL			0xE0

/*
 * Realtek EIO pin control
 *
 * RTL8380, RTL8390 and RTL9300 contain bits that control
 * the pin muxing of:
 * - GPIO 0 (A0/sys-led)
 * - GPIO 2, 3 (A2/MDCX, A3/MDIOX)
 */
struct realtek_pingroup_desc {
	const char *name;
	const unsigned int *pins;
	unsigned int num_pins;
	const struct reg_field field_desc;
	int value_gpio;
};

struct realtek_function_desc {
	const char *name;
	unsigned int field_value;
	const char *group_name;
	const struct realtek_pingroup_desc *group;
};

struct realtek_gpio_desc {
	unsigned int offset;
	unsigned int field_value;
	const struct realtek_pingroup_desc *group;
};

struct realtek_eio_pinctrl {
	struct device *dev;
	struct regmap *regmap;
	const struct realtek_function_desc *functions;
	unsigned int function_count;
	const struct realtek_pingroup_desc *groups;
	unsigned int group_count;
};

static struct pinctrl_pin_desc realtek_eio_pins[] = {
	PINCTRL_PIN(0, "A0"), /* system LED */
	PINCTRL_PIN(2, "A2"), /* RTL8231 MDC */
	PINCTRL_PIN(3, "A3"), /* RTL8231 MDIO */
};

static const unsigned int sys_led_pins[] = {0};
static const unsigned int ext_gpio_mii_pins[] = {2, 3};

static const struct realtek_pingroup_desc rtl_eio_groups[] = {
	{
		.name = "sys-led",
		.pins = sys_led_pins,
		.num_pins = ARRAY_SIZE(sys_led_pins),
		.field_desc = {
			.reg = REALTEK_EIO_GLOBAL_CTRL,
			.lsb = 15,
			.msb = 15,
		},
		.value_gpio = 0,
	},
	{
		.name = "ext-gpio-mii",
		.pins = ext_gpio_mii_pins,
		.num_pins = ARRAY_SIZE(ext_gpio_mii_pins),
		.field_desc = {
			.reg = RTL8380_EIO_GPIO_CTRL,
			.lsb = 0,
			.msb = 0,
		},
		.value_gpio = 0,
	},
};

static const struct realtek_function_desc rtl_eio_functions[] = {
	{
		.name = "sys-led",
		.field_value = 1,
		.group_name = "sys-led",
		.group = &rtl_eio_groups[0],
	},
	{
		.name = "ext-gpio-mii",
		.field_value = 1,
		.group_name = "ext-gpio-mii",
		.group = &rtl_eio_groups[1],
	},
};

static int realtek_eio_group_count(struct pinctrl_dev *pctldev)
{
	struct realtek_eio_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_info(priv->dev, "get group count\n");
	return priv->group_count;
}

static const char * realtek_eio_group_name(struct pinctrl_dev *pctldev,
	unsigned int selector)
{
	struct realtek_eio_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_info(priv->dev, "get group name %d\n", selector);
	return priv->groups[selector].name;
};

static int realtek_eio_group_pins(struct pinctrl_dev *pctldev,
	unsigned int selector, const unsigned int **pins,
	unsigned int *num_pins)
{
	struct realtek_eio_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_info(priv->dev, "get groups pins %d\n", selector);
	*pins = priv->groups[selector].pins;
	*num_pins = priv->groups[selector].num_pins;

	return 0;
}

static int realtek_eio_function_count(struct pinctrl_dev *pctldev)
{
	struct realtek_eio_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_info(priv->dev, "get function count\n");
	return priv->function_count;
}

static const char * realtek_eio_function_name(struct pinctrl_dev *pctldev,
	unsigned int selector)
{
	struct realtek_eio_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_info(priv->dev, "get function name %d\n", selector);
	return priv->functions[selector].name;
};

static int realtek_eio_function_group(struct pinctrl_dev *pctldev,
	unsigned int selector, const char * const **groups,
	unsigned int * const num_groups)
{
	struct realtek_eio_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_info(priv->dev, "get function groups %d\n", selector);
	*groups = &(priv->functions[selector].group_name);
	*num_groups = 1;

	return 0;
}

static int realtek_mux_set_field(struct regmap *regmap,
	const struct reg_field *field_desc, unsigned int value)
{
	struct regmap_field *field;
	int err;

	field = regmap_field_alloc(regmap, *field_desc);
	if (!field)
		return -ENOMEM;

	err = regmap_field_write(field, value);

	regmap_field_free(field);
	return err;

}

static int realtek_eio_set_mux(struct pinctrl_dev *pctldev,
	unsigned int selector, unsigned int group)
{
	struct realtek_eio_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	dev_info(priv->dev, "set mux %d\n", selector);
	return realtek_mux_set_field(priv->regmap,
		&priv->functions[selector].group->field_desc,
		priv->functions[selector].field_value);
}

static int realtek_eio_request_enable(struct pinctrl_dev *pctldev,
	struct pinctrl_gpio_range *range, unsigned int offset)
{
	struct realtek_eio_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	int group, pin;

	dev_info(priv->dev, "request gpio %d\n", offset);

	for (group = 0; group < priv->group_count; group++)
		for (pin = 0; pin < priv->groups[group].num_pins; pin++)
			if (pin == offset && priv->groups[group].value_gpio >= 0)
				return realtek_mux_set_field(priv->regmap,
					&priv->groups[group].field_desc,
					priv->groups[group].value_gpio);

	return -EINVAL;
}

static const struct pinctrl_ops realtek_eio_pinctrl_ops = {
	.get_groups_count = realtek_eio_group_count,
	.get_group_name = realtek_eio_group_name,
	.get_group_pins = realtek_eio_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static const struct pinmux_ops realtek_eio_pinmux_ops = {
	.get_functions_count = realtek_eio_function_count,
	.get_function_name = realtek_eio_function_name,
	.get_function_groups = realtek_eio_function_group,
	.set_mux = realtek_eio_set_mux,
	.gpio_request_enable = realtek_eio_request_enable,
	.strict = true,
};

static struct pinctrl_desc realtek_eio_pinctrl_desc = {
	.name = "realtek-eio",
	.pins = realtek_eio_pins,
	.npins = ARRAY_SIZE(realtek_eio_pins),
	.owner = THIS_MODULE,
	.pctlops = &realtek_eio_pinctrl_ops,
	.pmxops = &realtek_eio_pinmux_ops,
};

static int realtek_eio_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pinctrl_dev *pctldev;
	struct realtek_eio_pinctrl *priv;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->functions = rtl_eio_functions;
	priv->function_count = ARRAY_SIZE(rtl_eio_functions);
	priv->groups = rtl_eio_groups;
	priv->group_count = ARRAY_SIZE(rtl_eio_groups);

	priv->regmap = syscon_node_to_regmap(dev->of_node->parent);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	dev_info(dev, "registering pin controller\n");
	err = devm_pinctrl_register_and_init(dev, &realtek_eio_pinctrl_desc,
		priv, &pctldev);
	if (err)
		return err;
	dev_info(dev, "registered pin controller\n");

	err = pinctrl_enable(pctldev);
	dev_info(dev, "enabled pin controller\n");

	return err;
};

static const struct of_device_id of_realtek_eio_pinctrl_match[] = {
	{ .compatible = "realtek,rtl8380-eio-pinctrl" },
	{ /* sentinel */ }
};

static struct platform_driver realtek_eio_pinctrl_driver = {
	.driver = {
		.name = "realtek-eio-pinctrl",
		.of_match_table = of_realtek_eio_pinctrl_match
	},
	.probe = realtek_eio_pinctrl_probe,
};

module_platform_driver(realtek_eio_pinctrl_driver);
