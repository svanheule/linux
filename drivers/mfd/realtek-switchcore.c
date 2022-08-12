// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/bitfield.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct realtek_switchcore_ctrl;

struct realtek_switchcore_data {
	const struct mfd_cell *mfd_devices;
	unsigned int mfd_device_count;
	void (*probe_model_name)(const struct realtek_switchcore_ctrl *ctrl);
};

struct realtek_switchcore_ctrl {
	struct device *dev;
	struct regmap *map;
	const struct realtek_switchcore_data *data;
};

#define MODEL_NAME_CHAR_XLATE(val)		((val) ? 'A' + (val) - 1 : '\0')
#define MODEL_NAME_CHAR(reg, msb, lsb)		(MODEL_NAME_CHAR_XLATE(FIELD_GET(GENMASK((msb), (lsb)), (val))))

/*
 * Model name probe
 *
 * Reads the family-specific MODEL_NAME_INFO register
 * to identify the SoC model and revision
 */
#define RTL8380_REG_MODEL_NAME_INFO		0x00d4
#define RTL8380_REG_CHIP_INFO			0x00d8

#define RTL8380_REG_INT_RW_CTRL			0x0058

static void rtl8380_probe_model_name(const struct realtek_switchcore_ctrl *ctrl)
{
	char model_char[4] = {0, 0, 0, 0};
	u32 model_id, rl_id;
	char chip_rev;
	u32 val = 0;

	regmap_read(ctrl->map, RTL8380_REG_MODEL_NAME_INFO, &val);
	model_id = FIELD_GET(GENMASK(31, 16), val);
	model_char[0] = MODEL_NAME_CHAR(val, 15, 11);
	model_char[1] = MODEL_NAME_CHAR(val, 10, 6);
	model_char[2] = MODEL_NAME_CHAR(val, 5, 1);

	/* CHIP_INFO register must be unlocked by writing 0xa to the top bits */
	regmap_write(ctrl->map, RTL8380_REG_CHIP_INFO, FIELD_PREP(GENMASK(31, 28), 0xa));
	regmap_read(ctrl->map, RTL8380_REG_CHIP_INFO, &val);
	chip_rev = MODEL_NAME_CHAR(val, 20, 16) ?: '0';
	rl_id = FIELD_GET(GENMASK(15, 0), val);

	dev_info(ctrl->dev, "found RTL%04x%s rev. %c, CPU RTL%04x\n",
		model_id, model_char, chip_rev, rl_id);
}

#define RTL8390_REG_MODEL_NAME_INFO	0x0ff0
#define RTL8390_REG_CHIP_INFO		0x0ff4

static void rtl8390_probe_model_name(const struct realtek_switchcore_ctrl *ctrl)
{
	char model_char[3] = {0, 0, 0};
	u32 model_id, rl_id;
	char chip_rev;
	u32 val = 0;

	regmap_read(ctrl->map, RTL8390_REG_MODEL_NAME_INFO, &val);
	model_id = FIELD_GET(GENMASK(31, 16), val);
	model_char[0] = MODEL_NAME_CHAR(val, 15, 11);
	model_char[1] = MODEL_NAME_CHAR(val, 10, 6);

	/* CHIP_INFO register must be unlocked by writing 0xa to the top bits */
	regmap_write(ctrl->map, RTL8390_REG_CHIP_INFO, FIELD_PREP(GENMASK(31, 28), 0xa));
	regmap_read(ctrl->map, RTL8390_REG_CHIP_INFO, &val);
	chip_rev = MODEL_NAME_CHAR(val, 20, 16) ?: '0';
	rl_id = FIELD_GET(GENMASK(15, 0), val);

	dev_info(ctrl->dev, "found RTL%04x%s rev. %c, CPU RTL%04x\n",
		model_id, model_char, chip_rev, rl_id);
}

/*
 * Realtek hardware system LED
 *
 * The switch SoC supports one hardware managed direct LED output
 * to manage a system LED, with two supported blinking rates.
 *
 * TODO Move back here from split driver
 */

static const struct mfd_cell rtl8380_mfd_devices[] = {
	MFD_CELL_OF("realtek-switchcore-sys-led", NULL, NULL, 0, 0, "realtek,maple-sys-led"),
	MFD_CELL_OF("realtek-switchcore-port-leds",
		NULL, NULL, 0, 0, "realtek,rtl8380-port-led"),
	MFD_CELL_OF("realtek-switchcore-aux-mdio",
		NULL, NULL, 0, 0, "realtek,rtl8380-aux-mdio"),
	MFD_CELL_OF("realtek-switchcore-pinctrl",
		NULL, NULL, 0, 0, "realtek,rtl8380-pinctrl"),
};

static const struct realtek_switchcore_data rtl8380_switchcore_data = {
	.mfd_devices = rtl8380_mfd_devices,
	.mfd_device_count = ARRAY_SIZE(rtl8380_mfd_devices),
	.probe_model_name = rtl8380_probe_model_name,
};

static const struct mfd_cell rtl8390_mfd_devices[] = {
	MFD_CELL_OF("realtek-switchcore-sys-led", NULL, NULL, 0, 0, "realtek,cypress-sys-led"),
	MFD_CELL_OF("realtek-switchcore-port-leds",
		NULL, NULL, 0, 0, "realtek,rtl8390-port-led"),
	MFD_CELL_OF("realtek-switchcore-aux-mdio",
		NULL, NULL, 0, 0, "realtek,rtl8390-aux-mdio"),
	MFD_CELL_OF("realtek-switchcore-pinctrl",
		NULL, NULL, 0, 0, "realtek,rtl8390-pinctrl"),
};

static const struct realtek_switchcore_data rtl8390_switchcore_data = {
	.mfd_devices = rtl8390_mfd_devices,
	.mfd_device_count = ARRAY_SIZE(rtl8390_mfd_devices),
	.probe_model_name = rtl8390_probe_model_name,
};

static const struct of_device_id of_realtek_switchcore_match[] = {
	{
		.compatible = "realtek,rtl8380-switchcore",
		.data = &rtl8380_switchcore_data,
	},
	{
		.compatible = "realtek,rtl8390-switchcore",
		.data = &rtl8390_switchcore_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_realtek_switchcore_match);

static int realtek_switchcore_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;
	struct realtek_switchcore_ctrl *ctrl;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	match = of_match_device(of_realtek_switchcore_match, dev);
	if (match)
		ctrl->data = (struct realtek_switchcore_data *) match->data;
	else
		return dev_err_probe(dev, -EINVAL, "no device match\n");

	ctrl->dev = dev;

	if (!np)
		return dev_err_probe(dev, -ENODEV, "no DT node found\n");

	ctrl->map = device_node_to_regmap(np);
	if (!ctrl->map)
		return dev_err_probe(dev, -ENXIO, "failed to get regmap\n");

	if (ctrl->data->probe_model_name)
		ctrl->data->probe_model_name(ctrl);

	/* Find sub-devices */
	if (ctrl->data->mfd_devices)
		mfd_add_devices(dev, 0, ctrl->data->mfd_devices,
			ctrl->data->mfd_device_count, NULL, 0, NULL);

	return 0;
}

static struct platform_driver realtek_switchcore_driver = {
	.probe = realtek_switchcore_probe,
	.driver = {
		.name = "realtek-switchcore",
		.of_match_table = of_realtek_switchcore_match
	}
};
module_platform_driver(realtek_switchcore_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek SoC switch core driver");
MODULE_LICENSE("GPL v2");
