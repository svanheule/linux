// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/*
 * MDIO bus for RTL8231 access
 */
#define REALTEK_EIO_GLOBAL_CTRL			0x0

#define REALTEK_GPIO_MDIO_READ			0
#define REALTEK_GPIO_MDIO_WRITE			1
#define REALTEK_GPIO_MDIO_CMD			BIT(0)

#define RTL8380_EIO_GPIO_INDIRECT_ACCESS	0x09C
#define RTL8380_EIO_GPIO_CTRL			0x0E0
#define RTL8390_EIO_GPIO_INDIRECT_ACCESS	0x140

struct realtek_eio_mdio_ctrl;

struct realtek_eio_mdio_data {
	unsigned int mdio_access_reg;
	int (*eio_mdio_cmd)(struct realtek_eio_mdio_ctrl *ctrl, int addr,
		int regnum, int rw, u16 *data);
	int (*eio_mdio_enable)(struct realtek_eio_mdio_ctrl *ctrl,
		bool enable);
	int (*eio_mdio_set_clock)(struct realtek_eio_mdio_ctrl *ctrl,
	        int frequency);
	int (*eio_mdio_set_preamble)(struct realtek_eio_mdio_ctrl *ctrl,
		int preamble);
};

struct realtek_eio_mdio_ctrl {
	struct device *dev;
	struct regmap *map;
	const struct realtek_eio_mdio_data *data;
};

#define mii_bus_to_ctrl(bus) \
	((struct realtek_eio_mdio_ctrl *) bus->priv)

static int rtl8380_eio_mdio_cmd(struct realtek_eio_mdio_ctrl *ctrl,
	int addr, int regnum, int rw, u16 *data)
{
	u32 cmd;
	unsigned i = 0;

	cmd = ((regnum & 0x1f) << 7) | ((addr & 0x1f) << 2);
	if (rw == REALTEK_GPIO_MDIO_WRITE)
		cmd |= ((u32) *data) << 16;

	cmd |= (rw & 0x1) << 1 | REALTEK_GPIO_MDIO_CMD;
	regmap_write(ctrl->map, ctrl->data->mdio_access_reg, cmd);
	do {
		udelay(3);
		i++;
		regmap_read(ctrl->map, ctrl->data->mdio_access_reg, &cmd);
	} while (cmd & REALTEK_GPIO_MDIO_CMD);
	dev_info(ctrl->dev, "ADDR(%02x) REG(%02x) RW(%d) %08x (%d checks) \n",  addr, regnum, rw, cmd, i);

	*data = cmd >> 16;

	return 0;
}

static int rtl8380_eio_mdio_enable(struct realtek_eio_mdio_ctrl *ctrl,
	bool enable)
{
	if (enable)
		return regmap_set_bits(ctrl->map,
			RTL8380_EIO_GPIO_CTRL, BIT(0));
	else
		return regmap_clear_bits(ctrl->map,
			RTL8380_EIO_GPIO_CTRL, BIT(0));
}
	

static int realtek_eio_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct realtek_eio_mdio_ctrl *ctrl = mii_bus_to_ctrl(bus);
	u16 data;
	int err;

	err = ctrl->data->eio_mdio_cmd(ctrl, addr, regnum,
		REALTEK_GPIO_MDIO_READ, &data);

	if (err)
		return err;
	else
		return data;
}

static int realtek_eio_mdio_write(struct mii_bus *bus, int addr, int regnum,
	u16 val)
{
	struct realtek_eio_mdio_ctrl *ctrl = mii_bus_to_ctrl(bus);
	int err;

	err = ctrl->data->eio_mdio_cmd(ctrl, addr, regnum,
		REALTEK_GPIO_MDIO_WRITE, &val);

	return err;
}

static int realtek_eio_mdc_setting(int frequency)
{
	/*
	 * Valid settings and corresponding frequencies:
	 *   - 0: 2.6MHz
	 *   - 1: 5.2MHz
	 *   - 2: 10.4MHz
	 *   - 3: 20.8MHz
	 * Round down to select a clock speed that is likely to also work
	 */
	if (frequency < 5200000) {
		return 0;
	}
	else if (frequency < 10400000) {
		return 1;
	}
	else if (frequency < 20800000) {
		return 2;
	} else {
		return 3;
	}
}

static int rtl8380_eio_mdio_set_clock(struct realtek_eio_mdio_ctrl *ctrl,
        int frequency)
{
	int mdc_setting = realtek_eio_mdc_setting(frequency);

	return regmap_update_bits(ctrl->map, RTL8380_EIO_GPIO_CTRL,
		0x3 << 8, mdc_setting << 8);
}
static const struct realtek_eio_mdio_data rtl8380_eio_mdio_data = {
	.mdio_access_reg = RTL8380_EIO_GPIO_INDIRECT_ACCESS,
	.eio_mdio_cmd = rtl8380_eio_mdio_cmd,
	.eio_mdio_enable = rtl8380_eio_mdio_enable,
	.eio_mdio_set_clock = rtl8380_eio_mdio_set_clock
};

static const struct of_device_id of_realtek_eio_mdio_match[] = {
	{
		.compatible = "realtek,rtl8380-eio-mdio",
		.data = (void *) &rtl8380_eio_mdio_data
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, of_realtek_eio_mdio_match);

static int realtek_eio_mdio_probe(struct platform_device *pdev)
{
	struct realtek_eio_mdio_ctrl *ctrl;
	const struct of_device_id *match;
	struct device_node *np;
	struct mii_bus *bus;
	unsigned i;
	int err;
	u32 mdc_frequency;

	np = pdev->dev.of_node;

	if (!pdev->mfd_cell) {
		dev_err(&pdev->dev, "no mfd_cell specified\n");
		return -ENODEV;
	}

	bus = devm_mdiobus_alloc_size(&pdev->dev, sizeof(*ctrl));
	if (!bus)
		return -ENOMEM;
	ctrl = bus->priv;

	ctrl->dev = &pdev->dev;
	ctrl->map = syscon_node_to_regmap(np->parent);
	if (!ctrl->map) {
		dev_err(&pdev->dev, "failed to get regmap\n");
		return -EINVAL;
	}

	match = of_match_device(of_realtek_eio_mdio_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "could not find matching device\n");
		return -ENODEV;
	}
	ctrl->data = (const struct realtek_eio_mdio_data *) match->data;

	bus->name = "Realtek external GPIO SMI bus";
	snprintf(bus->id, MII_BUS_ID_SIZE, "rtl-eio-0") ;
	bus->parent = &pdev->dev;
	bus->read = realtek_eio_mdio_read;
	bus->write = realtek_eio_mdio_write;
	/* Don't have interrupts */
	for (i = 0; i < PHY_MAX_ADDR; i++)
		bus->irq[i] = PHY_POLL;

	err = devm_of_mdiobus_register(&pdev->dev, bus, np);
	if (err) {
		dev_err(&pdev->dev, "failed to register mdio bus\n");
		return err;
	}

	/* Must disable before changing settings */
	ctrl->data->eio_mdio_enable(ctrl, false);

	if (ctrl->data->eio_mdio_set_clock) {
		/*
		 * If property is not present, mdc_frequency will remain at
		 * the default value (0), which will select the* slowest
		 * clock speed.
		 */
		mdc_frequency = 0;
		of_property_read_u32(np, "clock-frequency", &mdc_frequency);
		ctrl->data->eio_mdio_set_clock(ctrl, mdc_frequency);
	}

	// TODO optional suppress-preamble (default: 8-bit, not 32-bit)
	ctrl->data->eio_mdio_enable(ctrl, true);

	return 0;
}

static struct platform_driver realtek_eio_mdio_driver = {
	.driver = {
		.name = "realtek-eio-mdio",
		.of_match_table = of_realtek_eio_mdio_match
	},
	.probe = realtek_eio_mdio_probe,
	// TODO .remove
};

module_platform_driver(realtek_eio_mdio_driver);

