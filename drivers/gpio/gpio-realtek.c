// SPDX-License-Identifier: GPL-2.0-only

#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/swab.h>

/*
 * Total register block size is 0x1C for ports four ports.
 * On the RTL8380/RLT8390 platforms port A, B, and C are implemented.
 * Only the RTL8389 and RTL8328 implement the second bank with ports E, F, G.
 *
 * Port information is stored with the first port at offset 0, followed by the
 * second, etc. Most registers store one bit per GPIO and should be read out in
 * reversed endian order. The two interrupt mask registers store two bits per
 * GPIO, and should be manipulated with swahw32.
 */

// Pin mux? 0: "normal", 1: "dedicate peripheral"
#define REALTEK_GPIO_REG_CNR		0x00
/* Clear bit (0) for input, set bit (1) for output */
#define REALTEK_GPIO_REG_DIR		0x08
#define REALTEK_GPIO_REG_DATA		0x0C
/* Read bit for IRQ status, write 1 to clear IRQ */
#define REALTEK_GPIO_REG_ISR		0x10
/* Two bits per GPIO */
#define REALTEK_GPIO_REG_IMR(pin)	(0x14 + sizeof(u32)*((pin)/16))
#define REALTEK_GPIO_IRQ_EDGE_FALLING	1
#define REALTEK_GPIO_IRQ_EDGE_RISING	2

#define REALTEK_GPIO_MAX		32

/*
 * Realtek GPIO driver data
 * Because the interrupt mask register (IMR) combines the function of
 * IRQ type selection and masking, two extra values are store.
 * intr_mask is used to mask/unmask the interrupts for certain GPIO,
 * and intr_type is used to store the selected interrupt types. The
 * logical AND of these values is written to IMR on changes.
 *
 * @dev Parent device
 * @gc Associated gpio_chip instance
 * @base Base address of the register block
 * @lock Lock for accessing the IRQ registers and values
 * @intr_mask Mask for GPIO interrupts
 * @intr_type GPIO interrupt type selection
 */
struct realtek_gpio_ctrl {
	struct device *dev;
	struct gpio_chip gc;
	void __iomem *base;
	raw_spinlock_t lock;
	u32 intr_mask[2];
	u32 intr_type[2];
};

static struct realtek_gpio_ctrl *irq_data_to_ctrl(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);

	return container_of(gc, struct realtek_gpio_ctrl, gc);
}

static inline void realtek_gpio_isr_clear(struct realtek_gpio_ctrl *ctrl,
	unsigned int pin)
{
	iowrite32(swab32(BIT(pin)), ctrl->base + REALTEK_GPIO_REG_ISR);
}

static void realtek_gpio_irq_ack(struct irq_data *data)
{
	struct realtek_gpio_ctrl *ctrl = irq_data_to_ctrl(data);
	u32 pin = irqd_to_hwirq(data);

	realtek_gpio_isr_clear(ctrl, pin);
}

static inline u32 realtek_gpio_imr_bits(unsigned int pin, u32 value)
{
	return (value << 2*(pin % 16));
}

static inline void realtek_update_imr(struct realtek_gpio_ctrl *ctrl,
	unsigned int pin, u32 type, u32 mask)
{
	iowrite32(swahw32(type & mask),
		ctrl->base + REALTEK_GPIO_REG_IMR(pin));
}

static void realtek_gpio_irq_unmask(struct irq_data *data)
{
	struct realtek_gpio_ctrl *ctrl = irq_data_to_ctrl(data);
	unsigned int pin = irqd_to_hwirq(data);
	unsigned int offset = pin/16;
	unsigned long flags;
	u32 m;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	m = ctrl->intr_mask[offset];
	m |= realtek_gpio_imr_bits(pin, 3);
	ctrl->intr_mask[offset] = m;
	realtek_update_imr(ctrl, pin, ctrl->intr_type[offset], m);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

static void realtek_gpio_irq_mask(struct irq_data *data)
{
	struct realtek_gpio_ctrl *ctrl = irq_data_to_ctrl(data);
	unsigned int pin = irqd_to_hwirq(data);
	unsigned int offset = pin/16;
	unsigned long flags;
	u32 m;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	m = ctrl->intr_mask[offset];
	m &= ~realtek_gpio_imr_bits(pin, 3);
	ctrl->intr_mask[offset] = m;
	realtek_update_imr(ctrl, pin, ctrl->intr_type[offset], m);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

static int realtek_gpio_irq_set_type(struct irq_data *data,
	unsigned int flow_type)
{
	struct realtek_gpio_ctrl *ctrl = irq_data_to_ctrl(data);
	irq_flow_handler_t handler;
	unsigned int pin = irqd_to_hwirq(data);
	unsigned int offset = pin/16;
	unsigned long flags;
	u32 type, t;

	switch (flow_type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_NONE:
		type = 0;
		handler = handle_bad_irq;
		break;
	case IRQ_TYPE_EDGE_RISING:
		type = REALTEK_GPIO_IRQ_EDGE_RISING;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		type = REALTEK_GPIO_IRQ_EDGE_RISING;
		fallthrough;
	case IRQ_TYPE_EDGE_FALLING:
		type |= REALTEK_GPIO_IRQ_EDGE_FALLING;
		handler = handle_edge_irq;
		break;
	default:
		return -EINVAL;
	}

	irq_set_handler_locked(data, handler);

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	t = ctrl->intr_type[offset];
	t &= ~realtek_gpio_imr_bits(pin, 3);
	t |= realtek_gpio_imr_bits(pin, type);
	ctrl->intr_type[offset] = t;
	realtek_update_imr(ctrl, pin, t, ctrl->intr_mask[offset]);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);

	return 0;
}

static void realtek_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct realtek_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	int offset;
	unsigned long status;

	chained_irq_enter(irq_chip, desc);

	status = swab32(ioread32(ctrl->base + REALTEK_GPIO_REG_ISR));
	if (status)
		for_each_set_bit(offset, &status, gc->ngpio) {
			dev_dbg(ctrl->dev, "gpio irq %d\n", offset);
			generic_handle_irq(irq_find_mapping(gc->irq.domain,
							offset));
			realtek_gpio_isr_clear(ctrl, offset);
		}

	chained_irq_exit(irq_chip, desc);
}

static struct irq_chip realtek_gpio_irq_chip = {
	.name = "realtek-gpio",
	.irq_ack = realtek_gpio_irq_ack,
	.irq_mask = realtek_gpio_irq_mask,
	.irq_unmask = realtek_gpio_irq_unmask,
	.irq_set_type = realtek_gpio_irq_set_type
};

static int realtek_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct gpio_irq_chip *girq;
	struct realtek_gpio_ctrl *ctrl;
	u32 ngpios;
	int err, irq;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	platform_set_drvdata(pdev, ctrl);

	ctrl->dev = dev;

	if (!np) {
		dev_err(&pdev->dev, "no DT node found\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "ngpios", &ngpios) != 0)
		ngpios = REALTEK_GPIO_MAX;

	if (ngpios > REALTEK_GPIO_MAX) {
		dev_err(&pdev->dev, "invalid ngpios (max. %d)\n",
		       REALTEK_GPIO_MAX);
		return -EINVAL;
	}

	irq = of_irq_get(np, 0);
	if (irq < 0) {
	    dev_err(dev, "failed to find irq number");
	    return irq;
	}

	ctrl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl->base))
		return PTR_ERR(ctrl->base);

	raw_spin_lock_init(&ctrl->lock);

	err = bgpio_init(&ctrl->gc, dev, 4,
		ctrl->base + REALTEK_GPIO_REG_DATA, NULL, NULL,
		ctrl->base + REALTEK_GPIO_REG_DIR, NULL,
		BGPIOF_BIG_ENDIAN_BYTE_ORDER);
	if (err) {
	    dev_err(dev, "unable to init generic GPIO");
	    return err;
	}

	ctrl->gc.ngpio = ngpios;
	ctrl->gc.owner = THIS_MODULE;
	ctrl->gc.request = gpiochip_generic_request;
	ctrl->gc.free = gpiochip_generic_free;

	girq = &ctrl->gc.irq;
	girq->chip = &realtek_gpio_irq_chip;
	girq->parent_handler = realtek_gpio_irq_handler;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
				GFP_KERNEL);
	if (!girq->parents)
	    return -ENOMEM;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;
	girq->parents[0] = irq;

	/* Disable and clear all interrupts */
	iowrite32(0, ctrl->base + REALTEK_GPIO_REG_IMR(0));
	iowrite32(0, ctrl->base + REALTEK_GPIO_REG_IMR(16));
	iowrite32(swab32(BIT(ngpios)-1), ctrl->base + REALTEK_GPIO_REG_ISR);

	err = gpiochip_add_data(&ctrl->gc, ctrl);
	return err;
}

static const struct of_device_id realtek_gpio_of_match[] = {
	{ .compatible = "realtek,realtek-gpio" },
	{},
};

MODULE_DEVICE_TABLE(of, realtek_gpio_of_match);

static struct platform_driver realtek_gpio_driver = {
	.driver = {
		.name = "realtek-gpio",
		.of_match_table	= realtek_gpio_of_match,
	},
	.probe = realtek_gpio_probe,
};

module_platform_driver(realtek_gpio_driver);

MODULE_DESCRIPTION("Realtek GPIO support");
MODULE_LICENSE("GPL v2");
