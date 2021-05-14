// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtual multiplexer driver
 *
 * Author: Sander Vanheule <sander@svanheule.net>
 */

#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/platform_device.h>
#include <linux/property.h>

static int mux_virtual_set(struct mux_control *mux, int state)
{
	return 0;
}

static const struct mux_control_ops mux_virtual_ops = {
	.set = mux_virtual_set,
};

static int mux_virtual_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mux_chip *mux_chip;
	u32 num_states;
	s32 idle_state;
	int ret;

	mux_chip = devm_mux_chip_alloc(dev, 1, 0);
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	mux_chip->ops = &mux_virtual_ops;

	ret = device_property_read_u32(dev, "num-states", &num_states);
	if (ret >= 0 || num_states == 0) {
		dev_err(dev, "invalid num-states %u\n", num_states);
		return -EINVAL;
	}
	mux_chip->mux->states = num_states;

	ret = device_property_read_u32(dev, "idle-state", (u32 *)&idle_state);
	if (ret >= 0 && idle_state != MUX_IDLE_AS_IS) {
		if (idle_state < 0 || idle_state >= mux_chip->mux->states) {
			dev_err(dev, "invalid idle-state %u\n", idle_state);
			return -EINVAL;
		}

		mux_chip->mux->idle_state = idle_state;
	}

	ret = devm_mux_chip_register(dev, mux_chip);
	if (ret < 0)
		return ret;

	dev_info(dev, "%u-way mux-controller registered\n", mux_chip->mux->states);

	return 0;
}

static const struct of_device_id mux_virtual_match[] = {
	{ .compatible = "virtual-mux", },
	{}
};
MODULE_DEVICE_TABLE(of, mux_virtual_match);

static struct platform_driver mux_virtual_driver = {
	.driver = {
		.name = "virtual-mux",
		.of_match_table	= mux_virtual_match,
	},
	.probe = mux_virtual_probe,
};
module_platform_driver(mux_virtual_driver);

MODULE_DESCRIPTION("Virtual multiplexer driver");
MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_LICENSE("GPL v2");
