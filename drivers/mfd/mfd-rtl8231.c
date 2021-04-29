// SPDX-License-Identifier: GPL-2.0-only

#include <linux/mfd/core.h>
#include <linux/mfd/rtl8231.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

static const struct reg_field RTL8231_FIELD_READY_CODE = REG_FIELD(RTL8231_REG_FUNC1, 4, 9);

#define RTL8231_READY_CODE_VALUE	0x37
#define RTL8231_LED_START_MASK		BIT(1)

// TODO MFD cells for GPIO, pinctrl, LED driver
static const struct mfd_cell rtl8231_cells[] = {
	{
		.name = "rtl8231-gpio",
		.of_compatible = "realtek,rtl8231-gpio",
	},
	{
		.name = "rtl8231-led",
		.of_compatible = "realtek,rtl8231-led",
	},
};

static int rtl8231_init(struct device *dev, struct regmap *map)
{
	struct regmap_field *field_ready_code;
	unsigned int v;
	int err = 0;

	field_ready_code = regmap_field_alloc(map, RTL8231_FIELD_READY_CODE);
	if (IS_ERR(field_ready_code))
		return PTR_ERR(field_ready_code);

	err = regmap_field_read(field_ready_code, &v);

	if (err) {
		dev_err(dev, "failed to read READY_CODE\n");
	} else if (v != RTL8231_READY_CODE_VALUE) {
		dev_err(dev, "RTL8231 not present or ready 0x%x != 0x%x\n",
			v, RTL8231_READY_CODE_VALUE);
		err = -ENODEV;
	}

	regmap_field_free(field_ready_code);

	if (err)
		return err;

	regmap_update_bits(map, RTL8231_REG_FUNC0, RTL8231_LED_START_MASK, RTL8231_LED_START_MASK);


	return 0;
}

static int rtl8231_mdio_reg_read(void *ctx, unsigned int reg, unsigned int *val)
{
	struct mdio_device *mdiodev = ctx;
	int ret;

	ret = mdiobus_read(mdiodev->bus, mdiodev->addr, reg);
	if (ret < 0)
		return ret;

	*val = ret & 0xffff;
	return 0;
}

static int rtl8231_mdio_reg_write(void *ctx, unsigned int reg, unsigned int val)
{
	struct mdio_device *mdiodev = ctx;

	return mdiobus_write(mdiodev->bus, mdiodev->addr, reg, val);
}

static const struct regmap_config rtl8231_regmap_config = {
	.val_bits = 16,
	.reg_bits = 5,
	.max_register = RTL8231_REG_COUNT - 1,
	.use_single_read = true,
	.use_single_write = true,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.reg_read = rtl8231_mdio_reg_read,
	.reg_write = rtl8231_mdio_reg_write,
};

static int rtl8231_mdio_probe(struct mdio_device *mdiodev)
{
	struct device *dev = &mdiodev->dev;
	struct regmap *map;
	int err;

	map = devm_regmap_init(dev, NULL, mdiodev, &rtl8231_regmap_config);

	if (IS_ERR(map)) {
		dev_err(dev, "failed to init regmap\n");
		return PTR_ERR(map);
	}

	err = rtl8231_init(dev, map);
	if (err)
		return err;

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
				    rtl8231_cells, ARRAY_SIZE(rtl8231_cells),
				    NULL, 0, NULL);
}

static void rtl8231_mdio_remove(struct mdio_device *mdiodev)
{
	struct regmap *map = dev_get_regmap(&mdiodev->dev, NULL);

	if (!map)
		return;

	regmap_update_bits(map, RTL8231_REG_FUNC0, RTL8231_LED_START_MASK, 0);
}

static const struct of_device_id rtl8231_of_match[] = {
	{ .compatible = "realtek,rtl8231" },
	{},
};
MODULE_DEVICE_TABLE(of, rtl8231_of_match);

static struct mdio_driver rtl8231_mdio_driver = {
	.mdiodrv.driver = {
		.name = "rtl8231-expander-mfd",
		.of_match_table	= rtl8231_of_match,
	},
	.probe = rtl8231_mdio_probe,
	.remove = rtl8231_mdio_remove,
};
mdio_module_driver(rtl8231_mdio_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek RTL8231 GPIO and LED expander");
MODULE_LICENSE("GPL v2");
