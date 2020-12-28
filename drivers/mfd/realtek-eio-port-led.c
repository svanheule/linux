// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/leds.h>
#include <linux/led-class-multicolor.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

// TODO move to header
#define REALTEK_EIO_GLOBAL_CTRL				0x0

/*
 * Realtek hardware port LED
 *
 * The switch ASIC can control up to three LEDs per phy, based on a number of
 * matching conditions. Alternatively, each individual LED output can also be
 * configured for manual control.
 *
 * Register offsets are defined relative the offset of LED_GLB_CTL.
 */
#define RTL8380_EIO_PORT_LED_MODE			0x4
#define RTL8380_EIO_PORT_LED_EN				0x8
#define RTL8380_EIO_PORT_LED_SOFT_EN(led)		(0x10 + 4*(led))
#define RTL8380_EIO_PORT_LED_SOFT_MODE(port)		(0x1C + 4*(port))
#define RTL8380_EIO_PORT_LED_STATUS_LOW(port, led)	\
	(0xAC + 4*(port/10) + 0xC*(led))
#define RTL8380_EIO_PORT_LED_STATUS_HIGH(port, led)	(0xD0 + 4*(led))

#define RTL8390_EIO_PORT_LED_SOFT_EN

struct led_port_blink_mode {
	u16 interval; /* Toggle interval in ms */
	u8 mode; /* ASIC mode bits */
};

#define REALTEK_PORT_LED_BLINK_COUNT	6
struct led_port_modes {
	unsigned int off;
	unsigned int on;
	struct led_port_blink_mode blink[REALTEK_PORT_LED_BLINK_COUNT];
};

struct realtek_port_led_info {
	unsigned int reg;
	unsigned int index;
};

struct realtek_port_led {
	struct led_classdev led;
	struct realtek_port_led_info info;
	struct regmap *map;
	const struct led_port_modes *modes;
};

struct realtek_eio_port_led_ctrl;

struct realtek_eio_port_led_data {
	unsigned int port_count;
	const struct led_port_modes *port_modes;
	unsigned int port_mode_base;
	void (*port_led_init)(struct realtek_eio_port_led_ctrl *ctrl,
		unsigned int led_count);
	int (*port_led_set_hw_modes)(struct realtek_eio_port_led_ctrl *ctrl);
};

struct realtek_eio_port_led_ctrl {
	struct device *dev;
	struct regmap *map;
	struct device_node *np;
	const struct realtek_eio_port_led_data *data;
};

static const struct led_port_modes rtl8380_port_led_modes = {
	.off = 0,
	.on = 5,
	.blink  = {{  32, 1},
		   {  64, 2},
		   { 128, 3},
		   { 256, 6},
		   { 512, 4},
		   {1024, 7}}
};

static const struct led_port_modes rtl8390_port_led_modes = {
	.off = 0,
	.on = 7,
	.blink = {{  32, 1},
		  {  64, 2},
		  { 128, 3},
		  { 256, 4},
		  { 512, 5},
		  {1024, 6}}
};

static inline int realtek_port_led_set_mode(struct regmap *map,
	const struct realtek_port_led_info *info, unsigned mode)
{
	int offset = 3*info->index;

	return regmap_update_bits(map, info->reg,
		0x7 << offset, mode << offset);
}

static void realtek_port_led_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct realtek_port_led *pled =
		container_of(led_cdev, struct realtek_port_led, led);
	int mode = brightness ? pled->modes->on : pled->modes->off;

	realtek_port_led_set_mode(pled->map, &pled->info, mode);
}

static enum led_brightness realtek_port_led_brightness_get(
	struct led_classdev *led_cdev)
{
	struct realtek_port_led *pled =
		container_of(led_cdev, struct realtek_port_led, led);
	unsigned int current_mode;
	u32 val;

	regmap_read(pled->map, pled->info.reg, &val);
	current_mode = (val >> 3*pled->info.index) & 0x7;

	if (current_mode == pled->modes->off)
		return LED_OFF;
	else
		return LED_ON;
}

static int realtek_port_led_blink_set(struct led_classdev *led_cdev,
	unsigned long *delay_on, unsigned long *delay_off)
{
	struct realtek_port_led *pled =
		container_of(led_cdev, struct realtek_port_led, led);
	const struct led_port_blink_mode *modes = &pled->modes->blink[0];
	unsigned long blink_interval = *delay_on + *delay_off;
	unsigned int i = 0;

	if (blink_interval == 0) {
		blink_interval = 512;
	}
	else if (*delay_on == 0) {
		realtek_port_led_set_mode(pled->map, &pled->info,
			pled->modes->off);
		return 0;
	}
	else if (*delay_off == 0) {
		realtek_port_led_set_mode(pled->map, &pled->info,
			pled->modes->on);
		return 0;
	}

	/*
	 * Since the modes always double in interval, the geometric mean of
	 * modes (i) and (i+1) is equal to sqrt(2)*mode.interval
	 */
	while (i < (REALTEK_PORT_LED_BLINK_COUNT-1) &&
		blink_interval > (2*141*modes[i].interval)/100)
		i++;

	*delay_on = modes[i].interval;
	*delay_off = modes[i].interval;
	realtek_port_led_set_mode(pled->map, &pled->info,
		modes[i].mode);

	return 0;
}

static void realtek_port_led_read_address(struct device_node *np,
	int *port_index, int *led_index)
{
	const __be32 *addr;

	addr = of_get_address(np, 0, NULL, NULL);
	if (addr) {
		*port_index = of_read_number(addr, 1);
		*led_index = of_read_number(addr+1, 1);
	}
}

static int realtek_port_led_probe_single(struct realtek_eio_port_led_ctrl *ctrl,
	struct device_node *np)
{
	struct realtek_port_led *port_led;
	unsigned int port_index, led_index;
	struct led_init_data init_data = {};
	u32 port_mask;
	int err;

	realtek_port_led_read_address(np, &port_index, &led_index);
	port_mask = BIT(port_index);

	if (of_property_read_bool(np, "realtek,hardware-managed"))
		goto port_led_simple_enable;

	port_led = devm_kzalloc(ctrl->dev, sizeof(*port_led), GFP_KERNEL);
	if (!port_led) {
		dev_err(ctrl->dev, "failed to allocate port led\n");
		return -ENOMEM;
	}

	init_data.fwnode = of_fwnode_handle(np);

	port_led->map = ctrl->map;
	port_led->modes = ctrl->data->port_modes;
	port_led->info.reg = ctrl->data->port_mode_base + 4*port_index;
	port_led->info.index = led_index;

	port_led->led.max_brightness = 1;
	port_led->led.brightness_set = realtek_port_led_brightness_set;
	port_led->led.brightness_get = realtek_port_led_brightness_get;
	port_led->led.blink_set = realtek_port_led_blink_set;

	dev_dbg(ctrl->dev, "registering port led %d.%d: reg=%08x, mask=%08x\n",
		port_index, led_index, port_led->info.reg,
		0x7 << 3*port_led->info.index);

	// TODO private triggers?
	err = devm_led_classdev_register_ext(ctrl->dev, &port_led->led,
		&init_data);
	if (err)
		return err;

	// TODO Use generic register offsets
	regmap_set_bits(ctrl->map, RTL8380_EIO_PORT_LED_SOFT_EN(led_index),
		port_mask);

port_led_simple_enable:
	regmap_set_bits(ctrl->map, RTL8380_EIO_PORT_LED_EN, port_mask);

	return 0;
}

struct realtek_port_multi_led {
	struct led_classdev_mc mc_cdev;
	struct regmap *map;
	const struct led_port_modes *modes;
	struct realtek_port_led_info *leds;
};

static void bicolor_led_brightness_set(struct led_classdev *cdev,
	enum led_brightness brightness)
{
	struct led_classdev_mc *mc_cdev = lcdev_to_mccdev(cdev);
	struct realtek_port_multi_led *ml = container_of(mc_cdev,
		struct realtek_port_multi_led, mc_cdev);
	unsigned l, mode;

	led_mc_calc_color_components(mc_cdev, brightness);

	for (l = 0; l < mc_cdev->num_colors; l++) {
		mode = mc_cdev->subled_info[l].brightness ?
			ml->modes->on : ml->modes->off;
		realtek_port_led_set_mode(ml->map, &ml->leds[l], mode);
	}
}

static int realtek_port_led_probe_multi(struct realtek_eio_port_led_ctrl *ctrl,
	struct device_node *np, int max_sub_led)
{
	struct led_init_data init_data = {};
	struct realtek_port_multi_led *mled;
	struct led_classdev *led_cdev;
	struct mc_subled *subled_info;
	struct realtek_port_led_info *subled;
	struct device_node *sub_np;
	unsigned channel;
	int subled_count;
	int port, port_led;
	int err;

	if (of_property_read_bool(np, "realtek,hardware-managed")) {
		dev_warn(ctrl->dev,
			"hardware managed multi-led not supported\n");
		return -ENODEV;
	}

	subled_count = of_get_child_count(np);
	dev_info(ctrl->dev, "found %d sub led nodes\n", subled_count);

	if (!subled_count) {
		dev_warn(ctrl->dev, "no LEDs defined\n");
		return -ENODEV;
	}
	else if (subled_count > max_sub_led) {
		dev_warn(ctrl->dev, "too many LEDs defined\n");
		return -EINVAL;
	}

	mled = devm_kzalloc(ctrl->dev, sizeof(*mled), GFP_KERNEL);
	if (!mled)
		return -ENOMEM;

	subled = devm_kcalloc(ctrl->dev, subled_count,
		sizeof(*subled), GFP_KERNEL);
	subled_info = devm_kcalloc(ctrl->dev, subled_count,
		sizeof(*subled_info), GFP_KERNEL);

	if (!subled_info || !subled)
		return -ENOMEM;

	mled->map = ctrl->map;
	mled->modes = ctrl->data->port_modes;
	mled->leds = subled;
	mled->mc_cdev.subled_info = subled_info;
	mled->mc_cdev.num_colors = subled_count;

	init_data.fwnode = of_fwnode_handle(np);

	channel = 0;

	for_each_child_of_node(np, sub_np) {
		subled_info->channel = channel++;
		of_property_read_u32(sub_np, "color",
			&subled_info->color_index);

		// TODO check port and port_led values
		realtek_port_led_read_address(sub_np, &port, &port_led);
		subled->reg = ctrl->data->port_mode_base + port;
		subled->index = port_led;

		subled_info++;
		subled++;
	}

	led_cdev = &mled->mc_cdev.led_cdev;
	led_cdev->max_brightness = 1;
	led_cdev->brightness_set = bicolor_led_brightness_set;

	err = devm_led_classdev_multicolor_register_ext(ctrl->dev,
		&mled->mc_cdev, &init_data);

	if (err)
	    dev_err(ctrl->dev, "failed to register multi-led\n");
	return err;
}

static void rtl8380_port_led_init(struct realtek_eio_port_led_ctrl *ctrl,
	unsigned int led_count)
{
	u32 led_mask = BIT(led_count)-1;
	u32 port_mask = BIT(ctrl->data->port_count)-1;

	regmap_update_bits(ctrl->map, REALTEK_EIO_GLOBAL_CTRL,
		0x3f, (led_mask << 3) | led_mask);

	regmap_clear_bits(ctrl->map, RTL8380_EIO_PORT_LED_EN, port_mask);
	regmap_clear_bits(ctrl->map, RTL8380_EIO_PORT_LED_SOFT_EN(0),
		port_mask);
	regmap_clear_bits(ctrl->map, RTL8380_EIO_PORT_LED_SOFT_EN(1),
		port_mask);
	regmap_clear_bits(ctrl->map, RTL8380_EIO_PORT_LED_SOFT_EN(2),
		port_mask);
}

//static void rtl8390_port_led_set_count(struct realtek_eio_ctrl *ctrl,
//	unsigned int count)
//{
//	regmap_update_bits(ctrl->map, REALTEK_EIO_GLOBAL_CTRL,
//		0x3 << 2, count << 2);
//}
//
//static void rtl9300_port_led_set_count(struct realtek_eio_ctrl *ctrl,
//	unsigned int count)
//{
//	regmap_write(ctrl->map, RTL9300_EIO_LED_NUM, count);
//}

static const struct realtek_eio_port_led_data rtl8380_port_led_data = {
	.port_count = 28,
	.port_modes = &rtl8380_port_led_modes,
	.port_mode_base = RTL8380_EIO_PORT_LED_SOFT_MODE(0),
	.port_led_init = rtl8380_port_led_init
};

static const struct of_device_id of_realtek_eio_port_led_match[] = {
	{
		.compatible = "realtek,rtl8380-eio-port-led",
		.data = &rtl8380_port_led_data
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, of_realtek_eio_port_led_match);

static int realtek_port_led_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct device_node *np, *child;
	const struct of_device_id *match;
	struct realtek_eio_port_led_ctrl ctrl;
	int child_count, match_condition_count;
	u32 leds_per_port, led_cfg;
	int err;
	unsigned led_set, led_set_index, led_cfg_shift;

	dev = &pdev->dev;
	np = dev->of_node;

	dev_info(dev, "probing port leds\n");

	if (!pdev->mfd_cell) {
		dev_err(dev, "must be instantiated as MFD child\n");
		return -ENODEV;
	}

	ctrl.dev = dev;
	ctrl.np = np;
	ctrl.map = syscon_node_to_regmap(of_get_parent(np));
	if (!ctrl.map) {
		dev_err(dev, "failed to find parent regmap\n");
		return -EINVAL;
	}

	match = of_match_device(of_realtek_eio_port_led_match, dev);
	if (!match) {
		dev_err(dev, "failed to find matching data\n");
		return -EINVAL;
	}
	ctrl.data = (const struct realtek_eio_port_led_data *) match->data;

	child_count = of_get_child_count(np);
	dev_dbg(dev, "%d child nodes\n", child_count);

	if (!child_count)
		return 0;

	err = of_property_read_u32(np, "realtek,num-led", &leds_per_port);
	if (err)
		return -EINVAL;

	if (leds_per_port > 3) {
		dev_err(dev, "number of leds per port too large (%d)\n",
			leds_per_port);
		return -EINVAL;
	}

	// TODO Make the number of led sets variable
	match_condition_count = of_property_count_u32_elems(np, "realtek,hardware-match-condition");
	if (match_condition_count != leds_per_port*2) {
		dev_err(dev, "hardware-match-condition must contain %d elements", leds_per_port*2);
		return -EINVAL;
	}

	ctrl.data->port_led_init(&ctrl, leds_per_port);

	for (led_set = 0; led_set < 2; led_set++) {
		for (led_set_index = 0; led_set_index < leds_per_port; led_set_index++) {
			led_cfg_shift = (3*led_set + led_set_index)*5;
			of_property_read_u32_index(np, "realtek,hardware-match-condition", led_set_index+leds_per_port*led_set, &led_cfg);
			// TODO generic implementation
			regmap_update_bits(ctrl.map, RTL8380_EIO_PORT_LED_MODE,
				0x1f << led_cfg_shift,
				led_cfg << led_cfg_shift);
		}
	}

	for_each_child_of_node(np, child) {
		if (of_n_addr_cells(child) != 2 || of_n_size_cells(child) != 0) {
			dev_err(dev, "#address-cells (%d) is not 2 or #size-cells (%d) is not 0\n",
				(u32) of_n_addr_cells(child), (u32) of_n_size_cells(child));
			of_node_put(child);
			return -EINVAL;
		}

		if (of_node_name_prefix(child, "led")) {
			err = realtek_port_led_probe_single(&ctrl, child);
			if (err)
				dev_warn(dev, "failed to register node\n");
			continue;
		}
		else if (of_node_name_prefix(child, "multi-led")) {
			dev_info(dev, "found multi-led node\n");

			err = realtek_port_led_probe_multi(&ctrl, child,
				leds_per_port);
			if (err)
				dev_warn(dev, "failed to register node\n");
			continue;
		}

		dev_dbg(dev, "skipping unsupported node %s\n",
			of_node_full_name(child));
	}

	return 0;
}

static struct platform_driver realtek_eio_port_led_driver = {
	.probe = realtek_port_led_probe,
	.driver = {
		.name = "realtek-eio-port-led",
		.of_match_table = of_realtek_eio_port_led_match
	}
};

module_platform_driver(realtek_eio_port_led_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek switch SoC port LED driver");
MODULE_LICENSE("GPL v2");

