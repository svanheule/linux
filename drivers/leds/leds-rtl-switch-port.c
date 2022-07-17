// SPDX-License-Identifier: GPL-2.0

#include <linux/leds.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/*
 * Realtek switch port LED
 *
 * The switch ASIC can control up to three LEDs per phy, based on a number of
 * matching conditions. Alternatively, each individual LED output can also be
 * configured for manual control.
 */
enum rtl_led_output_mode {
	RTL_LED_OUTPUT_SERIAL = 0,
	RTL_LED_OUTPUT_SCAN_SINGLE = 1,
	RTL_LED_OUTPUT_SCAN_BICOLOR = 2,
	RTL_LED_OUTPUT_DISABLED = 3,
};

#define PTRG_NONE		0
#define PTRG_ACT_RX		BIT(0)
#define PTRG_ACT_TX		BIT(1)
#define PTRG_ACT		PTRG_ACT_RX | PTRG_ACT_TX
#define PTRG_LINK_10		BIT(2)
#define PTRG_LINK_100		BIT(3)
#define PTRG_LINK_1000		BIT(4)
#define PTRG_LINK_2500		BIT(5)
#define PTRG_LINK_5000		BIT(6)
#define PTRG_LINK_10000		BIT(7)

struct led_port_blink_mode {
	u16 interval; /* Toggle interval in ms */
	u8 mode; /* ASIC mode bits */
};

#define REALTEK_PORT_LED_BLINK_COUNT	6
struct led_port_modes {
	u8 off;
	u8 on;
	struct led_port_blink_mode blink[REALTEK_PORT_LED_BLINK_COUNT];
};

struct led_port_group {
	unsigned int index;
	struct regmap_field *setting;
	unsigned int size;
	/* bitmap to keep track of associated ports */
	unsigned long *ports;
};

#define GROUP_LIST_INDEX(cfg, grp, idx)		((cfg)->port_led_count * (grp) + idx)

struct switch_port_led_config;

struct switch_port_led_ctrl {
	struct device *dev;
	struct regmap *map;
	const struct switch_port_led_config *cfg;
	struct mutex lock;
	struct led_port_group *groups;
};

struct switch_port_led {
	struct led_classdev led;
	struct switch_port_led_ctrl *ctrl;
	unsigned int port;
	unsigned int index;
	u32 trigger_flags;
	struct led_port_group *current_group;
};

struct switch_port_led_config {
	unsigned int port_count;
	unsigned int port_led_count;
	unsigned int group_count;
	const struct led_port_modes *modes;
	/* Set the number of LEDs per port */
	int (*port_led_init)(struct switch_port_led_ctrl *ctrl,
		unsigned int led_count, enum rtl_led_output_mode mode);
	/* Enable or disable all LEDs for a certain port */
	int (*set_port_enabled)(struct switch_port_led *led, bool enabled);
	int (*set_hw_managed)(struct switch_port_led *led, bool hw_managed);
	int (*write_mode)(struct switch_port_led *led, unsigned int mode);
	int (*read_mode)(struct switch_port_led *led);
	int (*trigger_xlate)(u32 trigger);
	/*
	 * Find the group the LED with this trigger setting can be assigned to.
	 * Can be either an existing group with identical settings, or an empty
	 * group. Return group on success, or < 0 on failure.
	 */
	struct led_port_group *(*map_group)(struct switch_port_led *led, u32 trigger);
	int (*assign_group)(struct switch_port_led *led, u32 group);
	struct reg_field group_settings[];
};

static struct led_port_group *switch_port_led_get_group(
	struct switch_port_led *pled, unsigned int group)
{
	unsigned int i = GROUP_LIST_INDEX(pled->ctrl->cfg, group, pled->index);

	return &pled->ctrl->groups[i];
}

static struct led_port_group *rtl_generic_port_led_map_group(struct switch_port_led *led, u32 trigger)
{
	struct switch_port_led_ctrl *ctrl = led->ctrl;
	int rtl_trg = ctrl->cfg->trigger_xlate(trigger);
	struct led_port_group *group;
	u32 current_trg;
	unsigned int i;
	int err;

	if (rtl_trg < 0)
	       return ERR_PTR(rtl_trg);

	for (i = 0; i < led->ctrl->cfg->group_count; i++) {
		group = switch_port_led_get_group(led, i);
		err = regmap_field_read(group->setting, &current_trg);
		if (err)
			return ERR_PTR(err);

		if (current_trg == rtl_trg || bitmap_empty(group->ports, group->size))
			return group;
	}

	dev_warn(ctrl->dev, "no available group for (%d,%d): 0x%02x\n",
		led->port, led->index, rtl_trg);
	return ERR_PTR(-ENOSPC);
}

/*
 * SoC specific implementation for RTL8380 series (Maple)
 */
#define RTL8380_REG_LED_MODE_SEL		0x1004
#define RTL8380_REG_LED_GLB_CTRL		0xa000
#define RTL8380_REG_LED_MODE_CTRL		0xa004
#define RTL8380_REG_LED_P_EN_CTRL		0xa008
#define RTL8380_REG_LED_SW_P_EN_CTRL(led)	(0xa010 + 4 * (led)->index)
#define RTL8380_REG_LED_SW_CTRL(led)		(0xa01c + 4 * (led)->port)

#define RTL8380_PORT_LED_COUNT			3
#define RTL8380_GROUP_SETTING_WIDTH		5
#define RTL8380_GROUP_SETTING_SHIFT(grp, idx)	\
	(RTL8380_GROUP_SETTING_WIDTH * ((idx) + RTL8380_PORT_LED_COUNT * (grp)))
#define RTL8380_GROUP_SETTING(grp, idx)	{				\
		.reg = RTL8380_REG_LED_MODE_CTRL,			\
		.lsb = RTL8380_GROUP_SETTING_SHIFT(grp, idx),		\
		.msb = RTL8380_GROUP_SETTING_SHIFT(grp, (idx) + 1) - 1,	\
	}

enum rtl83xx_port_trigger {
	RTL83XX_TRIG_LINK_ACT = 0,
	RTL83XX_TRIG_LINK = 1,
	RTL83XX_TRIG_ACT = 2,
	RTL83XX_TRIG_ACT_RX = 3,
	RTL83XX_TRIG_ACT_TX = 4,
	RTL83XX_TRIG_DUPLEX_MODE = 6,
	RTL83XX_TRIG_LINK_1G = 7,
	RTL83XX_TRIG_LINK_100M = 8,
	RTL83XX_TRIG_LINK_10M = 9,
	RTL83XX_TRIG_LINK_ACT_1G = 10,
	RTL83XX_TRIG_LINK_ACT_100M = 11,
	RTL83XX_TRIG_LINK_ACT_10M = 12,
	RTL83XX_TRIG_LINK_ACT_1G_100M = 13,
	RTL83XX_TRIG_LINK_ACT_1G_10M = 14,
	RTL83XX_TRIG_LINK_ACT_100M_10M = 15,
	RTL83XX_TRIG_LINK_ACT_10G = 21,
	RTL83XX_TRIG_DISABLED = 31,
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

static int rtl8380_port_trigger_xlate(u32 port_led_trigger)
{
	switch (port_led_trigger) {
	case PTRG_NONE:
		return RTL83XX_TRIG_DISABLED;
	case PTRG_ACT_RX:
		return RTL83XX_TRIG_ACT_RX;
	case PTRG_ACT_TX:
		return RTL83XX_TRIG_ACT_TX;
	case PTRG_ACT:
		return RTL83XX_TRIG_ACT;
	case PTRG_LINK_10 | PTRG_LINK_100 | PTRG_LINK_1000:
		return RTL83XX_TRIG_LINK;
	case PTRG_LINK_10:
		return RTL83XX_TRIG_LINK_10M;
	case PTRG_LINK_100:
		return RTL83XX_TRIG_LINK_100M;
	case PTRG_LINK_1000:
		return RTL83XX_TRIG_LINK_1G;
	case PTRG_ACT | PTRG_LINK_10 | PTRG_LINK_100 | PTRG_LINK_1000:
		return RTL83XX_TRIG_LINK_ACT;
	case PTRG_ACT | PTRG_LINK_10:
		return RTL83XX_TRIG_LINK_ACT_10M;
	case PTRG_ACT | PTRG_LINK_100:
		return RTL83XX_TRIG_LINK_ACT_100M;
	case PTRG_ACT | PTRG_LINK_1000:
		return RTL83XX_TRIG_LINK_ACT_1G;
	case PTRG_ACT | PTRG_LINK_10 | PTRG_LINK_100:
		return RTL83XX_TRIG_LINK_ACT_100M_10M;
	case PTRG_ACT | PTRG_LINK_10 | PTRG_LINK_1000:
		return RTL83XX_TRIG_LINK_ACT_1G_10M;
	case PTRG_ACT | PTRG_LINK_100 | PTRG_LINK_1000:
		return RTL83XX_TRIG_LINK_ACT_1G_100M;
	default:
		return -EINVAL;
	}
}

/*
 * RTL8380 has two static groups:
 *   - group 0: ports 0-23
 *   - group 1: ports 24-27
 *
 * When both groups need the same setting, the generic implementation would
 * always return the first group. However, high ports can only be controlled
 * via the second group, so we need an override of the generic implementation.
 */
static struct led_port_group *rtl8380_port_led_map_group(struct switch_port_led *led, u32 trigger)
{
	int rtl_trigger = rtl8380_port_trigger_xlate(trigger);
	struct switch_port_led_ctrl *ctrl = led->ctrl;
	struct led_port_group *group;
	u32 current_trigger;
	int err;

	if (rtl_trigger < 0)
	       return ERR_PTR(rtl_trigger);

	if (led->port > 23)
		group = switch_port_led_get_group(led, 1);
	else
		group = switch_port_led_get_group(led, 0);

	err = regmap_field_read(group->setting, &current_trigger);
	if (err)
		return ERR_PTR(err);

	if (current_trigger != rtl_trigger && !bitmap_empty(group->ports, group->size)) {
		dev_warn(ctrl->dev, "cannot map (%d,%d) to group %d: 0x%02x != 0x%02x\n",
			led->port, led->index, group->index, current_trigger, rtl_trigger);
		return ERR_PTR(-ENOSPC);
	}

	return group;
}

int rtl8380_port_led_assign_group(struct switch_port_led *led, unsigned int group)
{
	if (led->port > 23)
		return group == 1 ? 0 : -EINVAL;

	return group == 0 ? 0 : -EINVAL;
}

static int rtl8380_port_led_set_port_enabled(struct switch_port_led *led, bool enabled)
{
	unsigned int reg = RTL8380_REG_LED_P_EN_CTRL;
	u32 val = enabled ? BIT(led->port) : 0;

	return regmap_update_bits(led->ctrl->map, reg, BIT(led->port), val);
}

static int rtl8380_port_led_set_hw_managed(struct switch_port_led *led, bool hw_managed)
{
	unsigned int reg = RTL8380_REG_LED_SW_P_EN_CTRL(led);
	u32 val = hw_managed ? 0 : BIT(led->port);

	return regmap_update_bits(led->ctrl->map, reg, BIT(led->port), val);
}

static int rtl8380_port_led_write_mode(struct switch_port_led *led, unsigned int mode)
{
	unsigned int reg = RTL8380_REG_LED_SW_CTRL(led);
	unsigned int offset = 3 * led->index;
	u32 mask = GENMASK(2, 0) << offset;
	u32 value = mode << offset;

	return regmap_update_bits(led->ctrl->map, reg, mask, value);
}

static int rtl8380_port_led_read_mode(struct switch_port_led *led)
{
	u32 val;
	int ret;

	ret = regmap_read(led->ctrl->map, RTL8380_REG_LED_SW_CTRL(led), &val);
	if (ret)
		return ret;

	return (val >> (3 * led->index)) & GENMASK(2, 0);
}

static int rtl8380_port_led_init(struct switch_port_led_ctrl *ctrl,
	unsigned int led_count, enum rtl_led_output_mode mode)
{
	u32 led_count_mask, led_mask;
	int err;

	err = regmap_update_bits(ctrl->map, RTL8380_REG_LED_MODE_SEL, GENMASK(1, 0), mode);
	if (err)
		return err;

	led_count_mask = GENMASK(led_count - 1, 0);
	led_mask = GENMASK(5, 0);
	return regmap_update_bits(ctrl->map, RTL8380_REG_LED_GLB_CTRL, led_mask,
			(led_count_mask << 3) | led_count_mask);
}

static const struct switch_port_led_config rtl8380_port_led_config = {
	.port_count = 28,
	.port_led_count = 3,
	.group_count = 2,
	.modes = &rtl8380_port_led_modes,
	.port_led_init = rtl8380_port_led_init,
	.set_port_enabled = rtl8380_port_led_set_port_enabled,
	.set_hw_managed = rtl8380_port_led_set_hw_managed,
	.write_mode = rtl8380_port_led_write_mode,
	.read_mode = rtl8380_port_led_read_mode,
	.trigger_xlate = rtl8380_port_trigger_xlate,
	.map_group = rtl8380_port_led_map_group,
	.assign_group = rtl8380_port_led_assign_group,
	.group_settings = {
		RTL8380_GROUP_SETTING(0, 0),
		RTL8380_GROUP_SETTING(0, 1),
		RTL8380_GROUP_SETTING(0, 2),
		RTL8380_GROUP_SETTING(1, 0),
		RTL8380_GROUP_SETTING(1, 1),
		RTL8380_GROUP_SETTING(1, 2),
	},
};

/*
 * SoC specific implementation for RTL8390 series (Cypress)
 */
#define RTL8390_REG_LED_GLB_CTRL		0x00e4
#define RTL8390_REG_LED_COPR_SET_SEL_CTRL(led)	(0x00f0 + 4 * ((led)->port / 16))
#define RTL8390_REG_LED_COPR_PMASK_CTRL(led)	(0x0110 + 4 * ((led)->port / 32))
#define RTL8390_REG_LED_SW_CTRL			0x0128
#define RTL8390_REG_LED_SW_P_EN_CTRL(led)	(0x012c + 4 * ((led)->port / 10))
#define RTL8390_REG_LED_SW_P_CTRL(led)		(0x0144 + 4 * (led)->port)

#define RTL8390_PORT_LED_COUNT			3
#define RTL8390_GROUP_SETTING_WIDTH		5
#define RTL8390_GROUP_SETTING_REG(_grp)		(0x00ec - 4 * (_grp / 2))
#define RTL8390_GROUP_SETTING_SHIFT(_grp, _idx)	\
	(RTL8390_GROUP_SETTING_WIDTH * ((_idx) + RTL8390_PORT_LED_COUNT * (_grp % 2)))
#define RTL8390_GROUP_SETTING(_grp, _idx)	{				\
		.reg = RTL8390_GROUP_SETTING_REG(_grp) ,			\
		.lsb = RTL8390_GROUP_SETTING_SHIFT(_grp, _idx),			\
		.msb = RTL8390_GROUP_SETTING_SHIFT(_grp, (_idx) + 1) - 1,	\
	}

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

static void rtl8390_port_commit(struct switch_port_led_ctrl *ctrl)
{
	/*
	 * Could trigger the latching with delayed work,
	 * but that's probably not worth the overhead
	 */
	regmap_write(ctrl->map, RTL8390_REG_LED_SW_CTRL, 1);
}

static int rtl8390_port_trigger_xlate(u32 port_led_trigger)
{
	switch (port_led_trigger) {
	case PTRG_ACT | PTRG_LINK_10000:
		return RTL83XX_TRIG_LINK_ACT_10G;
	default:
		return rtl8380_port_trigger_xlate(port_led_trigger);
	}
}

int rtl8390_port_led_assign_group(struct switch_port_led *led, unsigned int group)
{
	unsigned int reg = RTL8390_REG_LED_COPR_SET_SEL_CTRL(led);
	unsigned int shift = 2 * (led->port % 16);
	u32 mask = GENMASK(1, 0) << shift;
	u32 val = group << shift;

	return regmap_update_bits(led->ctrl->map, reg, mask, val);
}

static int rtl8390_port_led_set_port_enabled(struct switch_port_led *led, bool enabled)
{
	/* Always enable a port as copper-only */
	int reg = RTL8390_REG_LED_COPR_PMASK_CTRL(led);
	u32 field_mask = BIT(led->port % 32);
	u32 val = enabled ? field_mask : 0;

	return regmap_update_bits(led->ctrl->map, reg, field_mask, val);
}

static int rtl8390_port_led_set_hw_managed(struct switch_port_led *led, bool hw_managed)
{
	int reg = RTL8390_REG_LED_SW_P_EN_CTRL(led);
	u32 field_mask = BIT(3 * (led->port % 10) + led->index);
	u32 val = hw_managed ? 0 : field_mask;

	/* TODO requires commiting settings? */
	return regmap_update_bits(led->ctrl->map, reg, field_mask, val);
}

static int rtl8390_port_led_write_mode(struct switch_port_led *led, unsigned int mode)
{
	unsigned int reg = RTL8390_REG_LED_SW_P_CTRL(led);
	unsigned int shift = led->index * 3;
	u32 mask = GENMASK(2, 0) << shift;
	u32 value = mode << shift;
	int err;

	err = regmap_update_bits(led->ctrl->map, reg, mask, value);
	if (!err)
		rtl8390_port_commit(led->ctrl);

	return err;
}

static int rtl8390_port_led_read_mode(struct switch_port_led *led)
{
	unsigned int reg = RTL8390_REG_LED_SW_P_CTRL(led);
	unsigned int shift = led->index * 3;
	u32 val;
	int err;

	err = regmap_read(led->ctrl->map, reg, &val);
	if (err)
		return err;

	return (val >> shift) & GENMASK(2, 0);
}

static int rtl8390_port_led_init(struct switch_port_led_ctrl *ctrl,
	unsigned int led_count, enum rtl_led_output_mode mode)
{
	u32 count_mask = GENMASK(3, 2);
	u32 mode_mask = GENMASK(1, 0);
	u32 enable = BIT(5);
	u32 count_val = FIELD_PREP(count_mask, led_count);
	u32 mode_val = FIELD_PREP(mode_mask, mode);

	return regmap_update_bits(ctrl->map, RTL8390_REG_LED_GLB_CTRL,
		count_mask | enable | mode_mask, count_val | enable | mode_val);
}

static const struct switch_port_led_config rtl8390_port_led_config = {
	.port_count = 52,
	.port_led_count = 3,
	.group_count = 4,
	.modes = &rtl8390_port_led_modes,
	.port_led_init = rtl8390_port_led_init,
	.set_port_enabled = rtl8390_port_led_set_port_enabled,
	.set_hw_managed = rtl8390_port_led_set_hw_managed,
	.write_mode = rtl8390_port_led_write_mode,
	.read_mode = rtl8390_port_led_read_mode,
	.trigger_xlate = rtl8390_port_trigger_xlate,
	.map_group = rtl_generic_port_led_map_group,
	.assign_group = rtl8390_port_led_assign_group,
	.group_settings = {
		RTL8390_GROUP_SETTING(0, 0),
		RTL8390_GROUP_SETTING(0, 1),
		RTL8390_GROUP_SETTING(0, 2),
		RTL8390_GROUP_SETTING(1, 0),
		RTL8390_GROUP_SETTING(1, 1),
		RTL8390_GROUP_SETTING(1, 2),
		RTL8390_GROUP_SETTING(2, 0),
		RTL8390_GROUP_SETTING(2, 1),
		RTL8390_GROUP_SETTING(2, 2),
		RTL8390_GROUP_SETTING(3, 0),
		RTL8390_GROUP_SETTING(3, 1),
		RTL8390_GROUP_SETTING(3, 2),
	},
};

/*
 * Led subsystem interface
 */
static void switch_port_led_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct switch_port_led *pled = container_of(led_cdev, struct switch_port_led, led);
	struct switch_port_led_ctrl *ctrl = pled->ctrl;

	ctrl->cfg->write_mode(pled, brightness ? ctrl->cfg->modes->on : ctrl->cfg->modes->off);
}

static enum led_brightness switch_port_led_brightness_get(struct led_classdev *led_cdev)
{
	struct switch_port_led *pled = container_of(led_cdev, struct switch_port_led, led);
	struct switch_port_led_ctrl *ctrl = pled->ctrl;

	return ctrl->cfg->read_mode(pled) != ctrl->cfg->modes->off;
}

static int switch_port_led_blink_set(struct led_classdev *led_cdev,
	unsigned long *delay_on, unsigned long *delay_off)
{
	struct switch_port_led *pled = container_of(led_cdev, struct switch_port_led, led);
	struct switch_port_led_ctrl *ctrl = pled->ctrl;

	const struct led_port_blink_mode *blink = &ctrl->cfg->modes->blink[0];
	unsigned long interval_ms = *delay_on + *delay_off;
	unsigned int i = 0;

	if (interval_ms && *delay_on == 0)
		return ctrl->cfg->write_mode(pled, ctrl->cfg->modes->off);

	if (interval_ms && *delay_off == 0)
		return ctrl->cfg->write_mode(pled, ctrl->cfg->modes->on);

	if (!interval_ms)
		interval_ms = 500;

	/*
	 * Since the modes always double in interval, the geometric mean of
	 * intervals [i] and [i + 1] is equal to sqrt(2) * interval[i]
	 */
	while (i < (REALTEK_PORT_LED_BLINK_COUNT - 1) &&
		interval_ms > (2 * 141 * blink[i].interval) / 100)
		i++;

	*delay_on = blink[i].interval;
	*delay_off = blink[i].interval;

	return ctrl->cfg->write_mode(pled, blink[i].mode);
}

static struct led_hw_trigger_type switch_port_rtl_hw_trigger_type;

static ssize_t rtl_hw_trigger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct switch_port_led *pled = dev_get_drvdata(dev);

	return sprintf(buf, "%x\n", pled->trigger_flags);
}

static ssize_t rtl_hw_trigger_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct switch_port_led *pled = dev_get_drvdata(dev);
	struct led_port_group *group, *new_group;
	unsigned int member_count;
	int trigger;
	int nchars;
	int value;
	int err;

	if (sscanf(buf, "%x%n", &value, &nchars) != 1 || nchars + 1 < count)
		return -EINVAL;

	mutex_lock(&pled->ctrl->lock);

	/*
	 * If the private trigger is already active:
	 *   - modify the current group if we are the only member,
	 *   - or join a new group if one is available
	 */
	if (pled->current_group) {
		group = pled->current_group;

		trigger = pled->ctrl->cfg->trigger_xlate(value);
		if (trigger < 0) {
			err = trigger;
			goto err_out;
		}

		member_count = bitmap_weight(group->ports, group->size);

		if (member_count == 1) {
			err = regmap_field_write(group->setting, trigger);
			if (err)
				goto err_out;

			goto out;
		}

		new_group = pled->ctrl->cfg->map_group(pled, value);
		if (IS_ERR(new_group)) {
			err = PTR_ERR(new_group);
			goto err_out;
		}

		if (member_count == 0) {
			err = regmap_field_write(new_group->setting, trigger);
			if (err)
				goto err_out;
		}

		err = pled->ctrl->cfg->assign_group(pled, new_group->index);
		if (err)
			goto err_out;

		bitmap_clear(group->ports, pled->port, 1);
		bitmap_set(new_group->ports, pled->port, 1);
		pled->current_group = new_group;
	}

out:
	pled->trigger_flags = value;

err_out:
	mutex_unlock(&pled->ctrl->lock);

	if (err)
		return err;

	return count;
}
static DEVICE_ATTR_RW(rtl_hw_trigger);

static struct attribute *rtl_hw_trigger_attrs[] = {
	&dev_attr_rtl_hw_trigger.attr,
	NULL,
};
ATTRIBUTE_GROUPS(rtl_hw_trigger);

static int switch_port_led_trigger_activate(struct led_classdev *led_cdev)
{
	struct switch_port_led *pled = container_of(led_cdev, struct switch_port_led, led);
	struct led_port_group *group;
	int rtl_trigger;
	int err = 0;

	mutex_lock(&pled->ctrl->lock);

	rtl_trigger = pled->ctrl->cfg->trigger_xlate(pled->trigger_flags);
	if (rtl_trigger < 0) {
		err = rtl_trigger;
		goto out;
	}

	group = pled->ctrl->cfg->map_group(pled, pled->trigger_flags);
	if (IS_ERR(group)) {
		err = PTR_ERR(group);
		goto out;
	}

	if (bitmap_empty(group->ports, group->size)) {
		err = regmap_field_write(group->setting, rtl_trigger);
		if (err)
			goto out;
	}

	bitmap_set(group->ports, pled->port, 1);
	pled->current_group = group;

	err = pled->ctrl->cfg->set_hw_managed(pled, true);

out:
	mutex_unlock(&pled->ctrl->lock);

	return err;
}

static void switch_port_led_trigger_deactivate(struct led_classdev *led_cdev)
{
	struct switch_port_led *pled = container_of(led_cdev, struct switch_port_led, led);
	struct led_port_group *group;

	mutex_lock(&pled->ctrl->lock);

	pled->ctrl->cfg->set_hw_managed(pled, false);

	group = pled->current_group;
	pled->current_group = NULL;
	bitmap_clear(group->ports, pled->port, 1);

	mutex_unlock(&pled->ctrl->lock);
}

static struct led_trigger switch_port_rtl_hw_trigger = {
	.name = "realtek-switchport",
	.activate = switch_port_led_trigger_activate,
	.deactivate = switch_port_led_trigger_deactivate,
	.trigger_type = &switch_port_rtl_hw_trigger_type,
};

static void realtek_port_led_read_address(struct device_node *np,
	int *port_index, int *led_index)
{
	const __be32 *addr;

	addr = of_get_address(np, 0, NULL, NULL);
	if (addr) {
		*port_index = of_read_number(addr, 1);
		*led_index = of_read_number(addr + 1, 1);
	}
}

static struct switch_port_led *switch_port_led_probe_single(
	struct switch_port_led_ctrl *ctrl, struct device_node *np)
{
	struct led_init_data init_data = {};
	struct switch_port_led *pled;
	unsigned int port_index;
	unsigned int led_index;
	int err;

	realtek_port_led_read_address(np, &port_index, &led_index);

	if (port_index >= ctrl->cfg->port_count || led_index >= ctrl->cfg->port_led_count)
		return ERR_PTR(-ENODEV);

	pled = devm_kzalloc(ctrl->dev, sizeof(*pled), GFP_KERNEL);
	if (!pled)
		return ERR_PTR(-ENOMEM);

	init_data.fwnode = of_fwnode_handle(np);

	pled->ctrl = ctrl;
	pled->port = port_index;
	pled->index = led_index;

	pled->led.max_brightness = 1;
	pled->led.brightness_set = switch_port_led_brightness_set;
	pled->led.brightness_get = switch_port_led_brightness_get;
	pled->led.blink_set = switch_port_led_blink_set;
	pled->led.trigger_type = &switch_port_rtl_hw_trigger_type;

	ctrl->cfg->set_hw_managed(pled, false);
	ctrl->cfg->set_port_enabled(pled, true);

	err = devm_led_classdev_register_ext(ctrl->dev, &pled->led, &init_data);
	if (err)
		return ERR_PTR(err);

	err = devm_device_add_groups(pled->led.dev, rtl_hw_trigger_groups);
	if (err)
		return ERR_PTR(err);

	dev_set_drvdata(pled->led.dev, pled);

	return pled;
}

static int realtek_port_led_probe(struct platform_device *pdev)
{
	struct switch_port_led_ctrl *ctrl;
	struct device *dev = &pdev->dev;
	struct device_node *np, *child;
	struct reg_field group_setting;
	unsigned int member_map_count;
	enum rtl_led_output_mode mode;
	struct led_port_group *group;
	struct switch_port_led *pled;
	const char *mode_name;
	u32 leds_per_port = 0;
	int i, i_grp, i_led;
	int child_count;
	int err;

	np = dev->of_node;

	dev_info(dev, "probing port leds\n");

//	if (!pdev->mfd_cell)
//		return dev_err_probe(dev, -ENODEV, "must be instantiated as MFD child\n");

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	mutex_init(&ctrl->lock);

	ctrl->dev = dev;
	ctrl->cfg = of_device_get_match_data(dev);
	if (!ctrl->cfg)
		return dev_err_probe(dev, -ENODEV, "failed to find matching data\n");

	ctrl->map = device_node_to_regmap(of_get_parent(np));
	if (IS_ERR_OR_NULL(ctrl->map))
		return dev_err_probe(dev, PTR_ERR(ctrl->map), "failed to find parent regmap\n");

	member_map_count = ctrl->cfg->port_led_count * ctrl->cfg->group_count;
	ctrl->groups = devm_kcalloc(dev, member_map_count,
		sizeof(*ctrl->groups), GFP_KERNEL);
	if (!ctrl->groups)
		return -ENOMEM;

	for (i_grp = 0; i_grp < ctrl->cfg->group_count; i_grp++) {
		for (i_led = 0; i_led < ctrl->cfg->port_led_count; i_led++) {
			i = GROUP_LIST_INDEX(ctrl->cfg, i_grp, i_led);

			group = &ctrl->groups[i];
			group_setting = ctrl->cfg->group_settings[i];

			group->index = i_grp;
			group->size = ctrl->cfg->port_count;
			group->setting = devm_regmap_field_alloc(dev, ctrl->map, group_setting);
			if (!group->setting)
				return -ENOMEM;

			group->ports = devm_bitmap_zalloc(dev, ctrl->cfg->port_count, GFP_KERNEL);
			if (!group->ports)
				return -ENOMEM;
		}
	}

	err = devm_led_trigger_register(dev, &switch_port_rtl_hw_trigger);
	if (err)
		return dev_err_probe(dev, err, "failed to register private trigger");

	child_count = of_get_child_count(np);
	dev_info(dev, "%d child nodes\n", child_count);

	if (!child_count)
		return 0;

	err = fwnode_property_read_string(dev_fwnode(dev), "realtek,output-mode", &mode_name);
	if (err)
		return dev_err_probe(dev, err, "failed to read realtek,output-mode\n");

	if (strcmp(mode_name, "serial") == 0)
		mode = RTL_LED_OUTPUT_SERIAL;
	else if (strcmp(mode_name, "single-color-scan") == 0)
		mode = RTL_LED_OUTPUT_SCAN_SINGLE;
	else if (strcmp(mode_name, "bi-color-scan") == 0)
		mode = RTL_LED_OUTPUT_SCAN_BICOLOR;
	else
		return dev_err_probe(dev, -EINVAL, "realtek,output-mode invalid\n");

	for_each_available_child_of_node(np, child) {
		if (of_n_addr_cells(child) != 2 || of_n_size_cells(child) != 0) {
			dev_err(dev, "#address-cells (%d) is not 2 or #size-cells (%d) is not 0\n",
				(u32) of_n_addr_cells(child), (u32) of_n_size_cells(child));
			of_node_put(child);
			return -EINVAL;
		}

		if (!of_node_name_prefix(child, "led")) {
			dev_dbg(dev, "skipping unsupported node %s\n", of_node_full_name(child));
			continue;
		}

		pled = switch_port_led_probe_single(ctrl, child);
		if (IS_ERR(pled)) {
			dev_warn(dev, "failed to register led: %ld\n", PTR_ERR(pled));
			continue;
		}

		if (pled->index + 1 > leds_per_port)
			leds_per_port = pled->index + 1;

		if (leds_per_port > ctrl->cfg->port_led_count)
			return dev_err_probe(dev, -EINVAL,
				"too many LEDs per port: %d > %d\n",
				leds_per_port, ctrl->cfg->port_led_count);
	}

	return ctrl->cfg->port_led_init(ctrl, leds_per_port, mode);
}

static const struct of_device_id of_switch_port_led_match[] = {
	{
		.compatible = "realtek,rtl8380-port-led",
		.data = &rtl8380_port_led_config
	},
	{
		.compatible = "realtek,rtl8390-port-led",
		.data = &rtl8390_port_led_config
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_switch_port_led_match);

static struct platform_driver realtek_switch_port_led_driver = {
	.probe = realtek_port_led_probe,
	.driver = {
		.name = "realtek-switch-port-led",
		.of_match_table = of_switch_port_led_match
	}
};
module_platform_driver(realtek_switch_port_led_driver);

MODULE_AUTHOR("Sander Vanheule <sander@svanheule.net>");
MODULE_DESCRIPTION("Realtek SoC switch port LED driver");
MODULE_LICENSE("GPL v2");
