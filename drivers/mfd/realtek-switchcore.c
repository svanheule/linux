// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/bits.h>
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

/*
 * Model name probe
 *
 * Reads the family-specific MODEL_NAME_INFO register
 * to identify the SoC model and revision
 */
#define MODEL_NAME_CHAR_XLATE(val)	((val) ? 'A' + (val) - 1 : '\0')
#define MODEL_NAME_CHAR(reg, mask)	MODEL_NAME_CHAR_XLATE(FIELD_GET((mask), (val)))

#define RTL_MODEL_ID_FIELD		GENMASK(31, 16)

#define RTL_CHIP_INFO_UNLOCK		GENMASK(31, 28)
#define RTL_CHIP_INFO_UNLOCK_CODE	0xa
#define RTL_CHIP_INFO_CHIP_REV		GENMASK(20, 16)
#define RTL_CHIP_INFO_RLID		GENMASK(15, 0)

/* Maple registers */
#define RTL838X_REG_MODEL_NAME_INFO	0x00d4
#define RTL838X_REG_CHIP_INFO		0x00d8
#define RTL838X_REG_MODE_DEFINE_CTL	0x1024

#define RTL838X_REG_INT_RW_CTRL		0x0058

/* Cypress registers */
#define RTL839X_REG_MODEL_NAME_INFO	0x0ff0
#define RTL839X_REG_CHIP_INFO		0x0ff4

static void rtl83xx_read_chip_info(struct regmap *map, unsigned int reg,
				      char *chip_rev, unsigned int *rl_id)
{
	u32 val = 0;

	regmap_write(map, reg, FIELD_PREP(RTL_CHIP_INFO_UNLOCK, RTL_CHIP_INFO_UNLOCK_CODE));
	regmap_read(map, reg, &val);
	*chip_rev = MODEL_NAME_CHAR(val, RTL_CHIP_INFO_CHIP_REV) ?: '0';
	*rl_id = FIELD_GET(RTL_CHIP_INFO_RLID, val);
}

static void rtl_swcore_chip_print(struct device *dev, unsigned int model_id,
				  const char *model_suffix, unsigned int chip_rev,
				  unsigned int rl_id)
{
	dev_info(dev, "found RTL%04x%s rev. %c, RL:%04x\n",
		 model_id, model_suffix, chip_rev, rl_id);
}

static void rtl838x_probe_model_name(const struct realtek_switchcore_ctrl *ctrl)
{
	char model_char[4] = {};
	u32 model_id, rl_id;
	char chip_rev;
	u32 val = 0;

	regmap_read(ctrl->map, RTL838X_REG_MODEL_NAME_INFO, &val);
	model_id = FIELD_GET(RTL_MODEL_ID_FIELD, val);
	model_char[0] = MODEL_NAME_CHAR(val, GENMASK(15, 11));
	model_char[1] = MODEL_NAME_CHAR(val, GENMASK(10, 6));
	model_char[2] = MODEL_NAME_CHAR(val, GENMASK(5, 1));

	if (model_id == 0x8380) {
		regmap_read(ctrl->map, RTL838X_REG_MODE_DEFINE_CTL, &val);
		/*
		 * Undocumented bit which is only set on RTL8380M.
		 * Possibly related to the presence of QSGMII ports for external phy.
		 */
		if (!(val & BIT(23)))
			model_id = 0x8381;
	}

	rtl83xx_read_chip_info(ctrl->map, RTL838X_REG_CHIP_INFO, &chip_rev, &rl_id);
	rtl_swcore_chip_print(ctrl->dev, model_id, model_char, chip_rev, rl_id);
}

static void rtl839x_probe_model_name(const struct realtek_switchcore_ctrl *ctrl)
{
	char model_char[3] = {};
	u32 model_id, rl_id;
	char chip_rev;
	u32 val = 0;

	regmap_read(ctrl->map, RTL839X_REG_MODEL_NAME_INFO, &val);
	model_id = FIELD_GET(GENMASK(31, 16), val);
	model_char[0] = MODEL_NAME_CHAR(val, GENMASK(15, 11));
	model_char[1] = MODEL_NAME_CHAR(val, GENMASK(10, 6));

	rtl83xx_read_chip_info(ctrl->map, RTL839X_REG_CHIP_INFO, &chip_rev, &rl_id);
	rtl_swcore_chip_print(ctrl->dev, model_id, model_char, chip_rev, rl_id);
}


static const struct mfd_cell rtl838x_mfd_devices[] = {
	MFD_CELL_OF("realtek-switchcore-sys-led", NULL, NULL, 0, 0, "realtek,maple-sys-led"),
	MFD_CELL_OF("realtek-switchcore-port-leds",
		NULL, NULL, 0, 0, "realtek,maple-port-led"),
	MFD_CELL_OF("realtek-switchcore-aux-mdio",
		NULL, NULL, 0, 0, "realtek,maple-aux-mdio"),
	MFD_CELL_OF("realtek-switchcore-pinctrl",
		NULL, NULL, 0, 0, "realtek,maple-pinctrl"),
};

static const struct realtek_switchcore_data rtl838x_switchcore_data = {
	.mfd_devices = rtl838x_mfd_devices,
	.mfd_device_count = ARRAY_SIZE(rtl838x_mfd_devices),
	.probe_model_name = rtl838x_probe_model_name,
};

static const struct mfd_cell rtl839x_mfd_devices[] = {
	MFD_CELL_OF("realtek-switchcore-sys-led", NULL, NULL, 0, 0, "realtek,cypress-sys-led"),
	MFD_CELL_OF("realtek-switchcore-port-leds",
		NULL, NULL, 0, 0, "realtek,cypress-port-led"),
	MFD_CELL_OF("realtek-switchcore-aux-mdio",
		NULL, NULL, 0, 0, "realtek,cypress-aux-mdio"),
	MFD_CELL_OF("realtek-switchcore-pinctrl",
		NULL, NULL, 0, 0, "realtek,cypress-pinctrl"),
};

static const struct realtek_switchcore_data rtl839x_switchcore_data = {
	.mfd_devices = rtl839x_mfd_devices,
	.mfd_device_count = ARRAY_SIZE(rtl839x_mfd_devices),
	.probe_model_name = rtl839x_probe_model_name,
};

static const struct of_device_id of_realtek_switchcore_match[] = {
	{
		.compatible = "realtek,maple-switchcore",
		.data = &rtl838x_switchcore_data,
	},
	{
		.compatible = "realtek,cypress-switchcore",
		.data = &rtl839x_switchcore_data,
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
