// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/leds.h>
#include <linux/led-class-multicolor.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define REALTEK_EIO_GLOBAL_CTRL				0x0

#define RTL8380_EIO_PORT_LED_MODE			0x4
#define RTL8380_EIO_PORT_LED_EN				0x8
#define RTL8380_EIO_PORT_LED_SOFT_EN(led)		(0x10 + 4*(led))
#define RTL8380_EIO_PORT_LED_SOFT_MODE(port)		(0x1C + 4*(port))
#define RTL8380_EIO_PORT_LED_STATUS_LOW(port, led)	\
 	(0xAC + 4*(port/10) + 0xC*(led))
#define RTL8380_EIO_PORT_LED_STATUS_HIGH(port, led)	(0xD0 + 4*(led))

#define RTL8390_EIO_PORT_LED_SOFT_EN

struct led_port_blink_mode {
	/* Toggle interval (delay_on, delay_off), 0 for always on */
	u16 interval;
	u8 mode; /* ASIC mode bits */
};

#define REALTEK_PORT_LED_BLINK_COUNT 6
struct led_port_modes {
	unsigned int off;
	unsigned int on;
	struct led_port_blink_mode blink[REALTEK_PORT_LED_BLINK_COUNT];
};

/*
 * Management of external RTL8231 GPIO expanders.
 * One RTL8231's GPIO registers can be shadowed to the internal GPIO_DIR
 * and GPIO_DAT registers.
 */
#define RTL8380_EIO_GPIO_INDIRECT_ACCESS	0x9C
#define RTL8380_EIO_GPIO_CTRL			0xE0
#define RTL8380_EIO_GPIO_DIR(pin)		(0xE4 + 4*((pin)/32))
#define RTL8380_EIO_GPIO_DAT(pin)		(0xEC + 4*((pin)/32))

struct realtek_eio_ctrl;

struct realtek_eio_data {
	unsigned int sys_led_pos;
	unsigned int port_count;
	const struct led_port_modes *port_modes;
	unsigned int port_mode_base;
	void (*set_port_led_count)(struct realtek_eio_ctrl *ctrl,
		unsigned int count);
};

struct realtek_eio_ctrl {
	struct device *dev;
	struct regmap *map;
	const struct realtek_eio_data *data;
	struct led_classdev sys_led;
};

/*
 * Realtek hardware system LED
 *
 * The switch SoC supports one hardware managed direct LED output
 * to manage a system LED, with two supported blinking rates.
 */
enum {
	REALTEK_SYS_LED_OFF = 0,
	REALTEK_SYS_LED_BLINK_64MS,
	REALTEK_SYS_LED_BLINK_1024MS,
	REALTEK_SYS_LED_ON
};

static void realtek_sys_led_set(const struct realtek_eio_ctrl *ctrl,
	unsigned int mode)
{
	regmap_update_bits(ctrl->map, REALTEK_EIO_GLOBAL_CTRL,
		(0x3 << ctrl->data->sys_led_pos),
		((mode & 0x3) << ctrl->data->sys_led_pos));
}

static void realtek_sys_led_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct realtek_eio_ctrl *ctrl =
		container_of(led_cdev, struct realtek_eio_ctrl, sys_led);

	if (brightness)
		realtek_sys_led_set(ctrl, REALTEK_SYS_LED_ON);
	else
		realtek_sys_led_set(ctrl, REALTEK_SYS_LED_OFF);
}

static enum led_brightness realtek_sys_led_brightness_get(
	struct led_classdev *led_cdev)
{
	struct realtek_eio_ctrl *ctrl =
		container_of(led_cdev, struct realtek_eio_ctrl, sys_led);
	u32 val;

	regmap_read(ctrl->map, REALTEK_EIO_GLOBAL_CTRL, &val);
	val = (val >> ctrl->data->sys_led_pos) & 0x3;

	if (val == REALTEK_SYS_LED_OFF)
		return LED_OFF;
	else
		return LED_ON;
}

static int realtek_sys_led_blink_set(struct led_classdev *led_cdev,
	unsigned long *delay_on, unsigned long *delay_off)
{
	struct realtek_eio_ctrl *ctrl =
		container_of(led_cdev, struct realtek_eio_ctrl, sys_led);
	u32 blink_interval = *delay_on + *delay_off;

	/* Split range at geometric mean of 64 and 1024 */
	if (blink_interval == 0 || blink_interval > 2*256) {
		*delay_on = 1024;
		*delay_off = 1024;
		realtek_sys_led_set(ctrl, REALTEK_SYS_LED_BLINK_1024MS);
	}
	else {
		*delay_on = 64;
		*delay_off = 64;
		realtek_sys_led_set(ctrl, REALTEK_SYS_LED_BLINK_64MS);
	}

	return 0;
}

static int realtek_sys_led_probe(struct realtek_eio_ctrl *ctrl,
	struct device *parent, struct fwnode_handle *fwnode)
{
	struct led_classdev *sys_led = &ctrl->sys_led;
	struct led_init_data init_data = {};

	init_data.fwnode = fwnode;

	sys_led->max_brightness = 1;
	sys_led->brightness_set = realtek_sys_led_brightness_set;
	sys_led->brightness_get = realtek_sys_led_brightness_get;
	sys_led->blink_set = realtek_sys_led_blink_set;

	devm_led_classdev_register_ext(parent, sys_led, &init_data);

	return 0;
}

/*
 * Realtek hardware port LED
 *
 * The switch ASIC can control up to three LEDs per phy, based on a number of
 * matching conditions. Alternatively, each individual LED output can also be
 * configured for manual control.
 */
struct realtek_port_led_info {
	unsigned int reg;
	unsigned int index;
};

struct realtek_port_led {
	struct led_classdev led;
	const struct led_port_modes *modes;
	struct regmap *map;
	struct realtek_port_led_info info;
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

static int realtek_port_led_probe_single(struct realtek_eio_ctrl *ctrl,
	struct device_node *np)
{
	struct realtek_port_led *port_led;
	unsigned int port_index, led_index;
	struct led_init_data init_data = {};
	u32 port_mask;

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

	/* Enable LED management and mark as software managed */
	dev_dbg(ctrl->dev, "registering port led %d.%d: reg=%08x, mask=%08x\n",
		port_index, led_index, port_led->info.reg,
		0x7 << 3*port_led->info.index);
	// TODO Use generic register offsets
	regmap_update_bits(ctrl->map, RTL8380_EIO_PORT_LED_SOFT_EN(led_index),
		port_mask, port_mask);

	// TODO private triggers?

	devm_led_classdev_register_ext(ctrl->dev, &port_led->led, &init_data);

port_led_simple_enable:
	// TODO Use generic register offsets
	regmap_update_bits(ctrl->map, RTL8380_EIO_PORT_LED_EN,
		port_mask, port_mask);

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

static int realtek_port_led_probe_multi(struct realtek_eio_ctrl *ctrl,
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

	subled = devm_kzalloc(ctrl->dev,
		sizeof(*subled)*subled_count, GFP_KERNEL);
	subled_info = devm_kzalloc(ctrl->dev,
		sizeof(*subled_info)*subled_count, GFP_KERNEL);

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

	led_cdev = &(mled->mc_cdev.led_cdev);
	led_cdev->max_brightness = 1;
	led_cdev->brightness_set = bicolor_led_brightness_set;

	err = devm_led_classdev_multicolor_register_ext(ctrl->dev,
		&mled->mc_cdev, &init_data);

	if (err)
	    dev_err(ctrl->dev, "failed to register multi-led\n");
	return err;
}

static int realtek_port_led_probe(struct realtek_eio_ctrl *ctrl,
	struct device_node *np)
{
	struct device_node *child;
	int child_count, match_condition_count;
	u32 leds_per_port, port_index, led_index, led_cfg;
	int err;
	unsigned led_set, led_set_index, led_cfg_shift;
	bool is_led, is_multi_led;

	child_count = of_get_child_count(np);
	dev_dbg(ctrl->dev, "port-led %d children\n", child_count);

	if (!child_count)
		return 0;

	err = of_property_read_u32(np, "realtek,num-led", &leds_per_port);
	if (err)
		return -EINVAL;

	if (leds_per_port > 3) {
		dev_err(ctrl->dev, "number of leds per port too large (%d)\n",
			leds_per_port);
		return -EINVAL;
	}

	// TODO Make the number of led sets variable
	match_condition_count = of_property_count_u32_elems(np, "realtek,hardware-match-condition");
	if (match_condition_count != leds_per_port*2) {
		dev_err(ctrl->dev, "hardware-match-condition must contain %d elements", leds_per_port*2);
		return -EINVAL;
	}

	ctrl->data->set_port_led_count(ctrl, leds_per_port);

	for (led_set = 0; led_set < 2; led_set++) {
		for (led_set_index = 0; led_set_index < leds_per_port; led_set_index++) {
			led_cfg_shift = (3*led_set + led_set_index)*5;
			of_property_read_u32_index(np, "realtek,hardware-match-condition", led_set_index+leds_per_port*led_set, &led_cfg);
			regmap_update_bits(ctrl->map, RTL8380_EIO_PORT_LED_MODE,
				0x1f << led_cfg_shift,
				(led_cfg & 0x1f) << led_cfg_shift);
		}
	}

	for_each_child_of_node(np, child) {
		if (of_n_addr_cells(child) != 2 || of_n_size_cells(child) != 0) {
			dev_err(ctrl->dev, "#address-cells (%d) is not 2 or #size-cells (%d) is not 0\n",
				(u32) of_n_addr_cells(child), (u32) of_n_size_cells(child));
			return -EINVAL;
		}

		is_led = of_node_name_prefix(child, "led");
		is_multi_led = of_node_name_prefix(child, "multi-led");

		realtek_port_led_read_address(child, &port_index, &led_index);

		if (is_led) {
			err = realtek_port_led_probe_single(ctrl, child);
			if (err)
				return err;
			continue;
		}
		else if (is_multi_led) {
			dev_info(ctrl->dev, "found multi-led node\n");

			err = realtek_port_led_probe_multi(ctrl, child,
				leds_per_port);
			if (err)
				return err;

			continue;
		}

		dev_dbg(ctrl->dev, "skipping unsupported led-port node %s\n",
			of_node_full_name(child));
	}

	return 0;
}

static void rtl8380_port_led_set_count(struct realtek_eio_ctrl *ctrl,
	unsigned int count)
{
	u32 led_mask = BIT(count)-1;

	regmap_update_bits(ctrl->map, REALTEK_EIO_GLOBAL_CTRL,
		0x3f, (led_mask << 3) | led_mask);
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

static const struct realtek_eio_data rtl8380_eio_data = {
	.sys_led_pos = 16,
	.port_count = 28,
	.port_modes = &rtl8380_port_led_modes,
	.port_mode_base = RTL8380_EIO_PORT_LED_SOFT_MODE(0),
	.set_port_led_count = rtl8380_port_led_set_count
};

//static struct realtek_eio_data rtl8390_eio_data = {
//	.sys_led_pos = 15,
//	.port_modes = &rtl8390_port_led_modes
//};
//
//static struct realtek_eio_data rtl9300_eio_data = {
//	.sys_led_pos = 13,
//	.port_modes = &rtl8390_port_led_modes
//};
//
//static struct realtek_eio_data rtl9310_eio_data = {
//	.sys_led_pos = 12,
//	.port_modes = &rtl8390_port_led_modes
//};

static const struct of_device_id of_realtek_eio_match[] = {
	{
		.compatible = "realtek,rtl8380-ext-io",
		.data = &rtl8380_eio_data
	}
};

MODULE_DEVICE_TABLE(of, of_realtek_eio_match);

static int realtek_eio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *np_sys_led, *np_port_led;
	const struct of_device_id *match;
	struct realtek_eio_ctrl *ctrl;
	int err, val;
	unsigned r;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	match = of_match_device(of_realtek_eio_match, &pdev->dev);
	if (match)
		ctrl->data = (struct realtek_eio_data *) match->data;
	else {
		dev_err(dev, "no device match\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, ctrl);

	ctrl->dev = dev;

	if (!np) {
		dev_err(dev, "no DT node found\n");
		return -EINVAL;
	}

	ctrl->map = device_node_to_regmap(np);
	if (!ctrl->map) {
		dev_err(dev, "failed to get regmap\n");
		return -EINVAL;
	}
	/* Parse optional sys-led child */
	// TODO

	/* Parse optional port-led child */
	np_port_led = of_get_child_by_name(np, "port-led");
	if (IS_ERR(np_port_led))
		return PTR_ERR(np_port_led);

	if (np_port_led) {
		dev_info(dev, "probe port-led child\n");
		err = realtek_port_led_probe(ctrl, np_port_led);
		if (err)
			return err;
	}
	else
		dev_info(dev, "no port-led child found\n");

	/* Parse optional mdio-bus child */
	// TODO

	/* Dump register values */
	for (r = 0; r <= regmap_get_max_register(ctrl->map); r += 4) {
		regmap_read(ctrl->map, r, &val);
		dev_info(dev, "%02x %08x\n", r, val);
	}

	return 0;
}

static struct platform_driver realtek_eio_driver = {
	.probe = realtek_eio_probe,
	.driver = {
		.name = "realtek-ext-io",
		.of_match_table = of_realtek_eio_match
	}
};

module_platform_driver(realtek_eio_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek switch SoC external LED/GPIO driver");
MODULE_LICENSE("GPL v2");
