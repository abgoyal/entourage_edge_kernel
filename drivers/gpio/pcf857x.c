/*
 * pcf857x - driver for pcf857x, pca857x, and pca967x I2C GPIO expanders
 *
 * Copyright (C) 2007 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c/pcf857x.h>

#include <asm/gpio.h>


static const struct i2c_device_id pcf857x_id[] = {
	{ "pcf8574", 8 },
	{ "pca8574", 8 },
	{ "pca9670", 8 },
	{ "pca9672", 8 },
	{ "pca9674", 8 },
	{ "pcf8575", 16 },
	{ "pca8575", 16 },
	{ "pca9671", 16 },
	{ "pca9673", 16 },
	{ "pca9675", 16 },
	{ "max7328", 8 },
	{ "max7329", 8 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcf857x_id);

#undef GPIO_DEBUG

#ifdef CONFIG_GPIO_PCF857X_GENERIC_IRQ
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#endif

/*
 * The pcf857x, pca857x, and pca967x chips only expose one read and one
 * write register.  Writing a "one" bit (to match the reset state) lets
 * that pin be used as an input; it's not an open-drain model, but acts
 * a bit like one.  This is described as "quasi-bidirectional"; read the
 * chip documentation for details.
 *
 * Many other I2C GPIO expander chips (like the pca953x models) have
 * more complex register models and more conventional circuitry using
 * push/pull drivers.  They often use the same 0x20..0x27 addresses as
 * pcf857x parts, making the "legacy" I2C driver model problematic.
 */
struct pcf857x {
	struct gpio_chip	chip;
	struct i2c_client	*client;
	struct mutex		lock;		/* protect 'out' */
	unsigned		out;		/* software latch */
#ifdef CONFIG_GPIO_PCF857X_GENERIC_IRQ
	uint16_t last_input;
	/*
	 * Note: Generic IRQ is not accessible within module code, the IRQ
	 * support will thus _only_ be available if the driver is built-in
	 */
	int irq;	/* IRQ for the chip itself */
	int irq_start;	/* starting IRQ for the on-chip GPIO lines */

	uint16_t irq_mask;
	uint16_t irq_falling_edge;
	uint16_t irq_rising_edge;

	struct irq_chip irq_chip;
	struct work_struct irq_work;
#endif
};

/*-------------------------------------------------------------------------*/

/* Talk to 8-bit I/O expander */

static int pcf857x_input8(struct gpio_chip *chip, unsigned offset)
{
	struct pcf857x	*gpio = container_of(chip, struct pcf857x, chip);
	int		status;

	mutex_lock(&gpio->lock);
	gpio->out |= (1 << offset);
	status = i2c_smbus_write_byte(gpio->client, gpio->out);
	mutex_unlock(&gpio->lock);

	return status;
}

static int pcf857x_get8(struct gpio_chip *chip, unsigned offset)
{
	struct pcf857x	*gpio = container_of(chip, struct pcf857x, chip);
	s32		value;

	value = i2c_smbus_read_byte(gpio->client);
	return (value < 0) ? 0 : (value & (1 << offset));
}

static int pcf857x_output8(struct gpio_chip *chip, unsigned offset, int value)
{
	struct pcf857x	*gpio = container_of(chip, struct pcf857x, chip);
	unsigned	bit = 1 << offset;
	int		status;

	mutex_lock(&gpio->lock);
	if (value)
		gpio->out |= bit;
	else
		gpio->out &= ~bit;
	status = i2c_smbus_write_byte(gpio->client, gpio->out);
	mutex_unlock(&gpio->lock);

	return status;
}

static void pcf857x_set8(struct gpio_chip *chip, unsigned offset, int value)
{
	pcf857x_output8(chip, offset, value);
}

/*-------------------------------------------------------------------------*/

/* Talk to 16-bit I/O expander */

static int i2c_write_le16(struct i2c_client *client, u16 word)
{
	u8 buf[2] = { word & 0xff, word >> 8, };
	int status;

	status = i2c_master_send(client, buf, 2);
	return (status < 0) ? status : 0;
}

static int i2c_read_le16(struct i2c_client *client)
{
	u8 buf[2];
	int status;

	status = i2c_master_recv(client, buf, 2);
	if (status < 0)
		return status;
	return (buf[1] << 8) | buf[0];
}

static int pcf857x_input16(struct gpio_chip *chip, unsigned offset)
{
	struct pcf857x	*gpio = container_of(chip, struct pcf857x, chip);
	int		status;

	mutex_lock(&gpio->lock);
	gpio->out |= (1 << offset);
	status = i2c_write_le16(gpio->client, gpio->out);
	mutex_unlock(&gpio->lock);

	return status;
}

static int pcf857x_get16(struct gpio_chip *chip, unsigned offset)
{
	struct pcf857x	*gpio = container_of(chip, struct pcf857x, chip);
	int		value;

	value = i2c_read_le16(gpio->client);
#ifdef GPIO_DEBUG
	printk(KERN_CRIT "%s: entry (offset = %d, value = 0x%08x)\n",
				   	__FUNCTION__, offset, value);
#endif

	return (value < 0) ? 0 : (value & (1 << offset));
}

static int pcf857x_output16(struct gpio_chip *chip, unsigned offset, int value)
{
	struct pcf857x	*gpio = container_of(chip, struct pcf857x, chip);
	unsigned	bit = 1 << offset;
	int		status;

#ifdef GPIO_DEBUG
	printk(KERN_CRIT "%s: config gpio%d as output\n", __FUNCTION__, offset);
#endif

	mutex_lock(&gpio->lock);
	if (value)
		gpio->out |= bit;
	else
		gpio->out &= ~bit;
	status = i2c_write_le16(gpio->client, gpio->out);
	mutex_unlock(&gpio->lock);

	return status;
}

static void pcf857x_set16(struct gpio_chip *chip, unsigned offset, int value)
{
	pcf857x_output16(chip, offset, value);
}

#ifdef CONFIG_GPIO_PCF857X_GENERIC_IRQ
/* FIXME: change to schedule_delayed_work() here if reading out of
 * registers does not reflect the actual pin levels
 */

static void pcf857x_irq_work(struct work_struct *work)
{
	struct pcf857x *chip;
	uint16_t input, mask, rising, falling;
	int ret, i;

	chip = container_of(work, struct pcf857x, irq_work);

	ret = i2c_read_le16(chip->client);
	if (ret < 0)
		return;

	input = ret & 0xffff;
	mask = (input ^ chip->last_input) & chip->irq_mask;
	rising = (input & mask) & chip->irq_rising_edge;
	falling = (~input & mask) & chip->irq_falling_edge;

	irq_enter();

	for (i = 0; i < chip->chip.ngpio; i++) {
		if ((rising | falling) & (1u << i)) {
			int irq = chip->irq_start + i;
			struct irq_desc *desc;

			desc = irq_desc + irq;
			desc_handle_irq(irq, desc);
		}
	}

	irq_exit();

	chip->last_input = input;
}

static void
pcf857x_irq_demux(unsigned int irq, struct irq_desc *desc)
{
	struct pcf857x *chip = desc->handler_data;

	desc->chip->mask(chip->irq);
	desc->chip->ack(chip->irq);
	schedule_work(&chip->irq_work);
	desc->chip->unmask(chip->irq);
}

static void pcf857x_irq_mask(unsigned int irq)
{
	struct irq_desc *desc = irq_desc + irq;
	struct pcf857x *chip = desc->chip_data;

	chip->irq_mask &= ~(1u << (irq - chip->irq_start));
}

static void pcf857x_irq_unmask(unsigned int irq)
{
	struct irq_desc *desc = irq_desc + irq;
	struct pcf857x *chip = desc->chip_data;

	chip->irq_mask |= 1u << (irq - chip->irq_start);
}

static void pcf857x_irq_ack(unsigned int irq)
{
	/* unfortunately, we have to provide an empty irq_chip.ack even
	 * if we do nothing here, Generic IRQ will complain otherwise
	 */
}

static int pcf857x_irq_set_type(unsigned int irq, unsigned int type)
{
	struct irq_desc *desc = irq_desc + irq;
	struct pcf857x *chip = desc->chip_data;
	uint16_t mask = 1u << (irq - chip->irq_start);
	int gpio = irq_to_gpio(irq);

#ifdef GPIO_DEBUG
	printk(KERN_CRIT "%s: entry, irq = %d (gpio = %d)\n",
				   	__FUNCTION__, irq, gpio);
#endif
	if (type == IRQ_TYPE_PROBE) {
		if ((mask & chip->irq_rising_edge) ||
		    (mask & chip->irq_falling_edge))
			return 0;

		type = IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING;
	}

#if 0
	/* disable, because gpio already 'owned' by gpio-keys when calling here */
	if (gpio_request(gpio, "gpio expander pin")) {
		printk(KERN_ERR "Request GPIO failed, gpio: 0x%x\n", gpio);
		return -1;
	}
	gpio_direction_input(gpio);	
	gpio_free(gpio);	
#endif

	if (type & IRQ_TYPE_EDGE_RISING)
		chip->irq_rising_edge |= mask;
	else
		chip->irq_rising_edge &= ~mask;

	if (type & IRQ_TYPE_EDGE_FALLING)
		chip->irq_falling_edge |= mask;
	else
		chip->irq_falling_edge &= ~mask;

	return 0;
}

static int pcf857x_init_irq(struct pcf857x *chip)
{
	struct irq_chip *ic = &chip->irq_chip;
	int irq, irq_start;

	chip->irq = chip->client->irq;
	chip->irq_start = irq_start = gpio_to_irq(chip->chip.base);

#ifdef GPIO_DEBUG
	printk(KERN_CRIT "%s: irq = %d, irq_start = %d, chip.base = %d\n",
			__FUNCTION__, chip->irq, chip->irq_start, chip->chip.base);
#endif

	/* do not install GPIO interrupts for the chip if
	 * 1. the PCF857X interrupt line is not used
	 * 2. or the GPIO interrupt number exceeds NR_IRQS
	 */
	if (chip->irq <= 0 || irq_start + chip->chip.ngpio >= NR_IRQS)
		return -EINVAL;

	chip->irq_mask	= 0;
	chip->irq_rising_edge  = 0;
	chip->irq_falling_edge = 0;

	ic->ack = pcf857x_irq_ack;
	ic->mask = pcf857x_irq_mask;
	ic->unmask = pcf857x_irq_unmask;
	ic->set_type = pcf857x_irq_set_type;

	for (irq = irq_start; irq < irq_start + chip->chip.ngpio; irq++) {
#ifdef GPIO_DEBUG
		printk(KERN_CRIT "%s: setting irq chip for irq %d\n",
			__FUNCTION__, irq);
#endif

		set_irq_chip(irq, ic);
		set_irq_chip_data(irq, chip);
		set_irq_handler(irq, handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	set_irq_type(chip->irq, IRQ_TYPE_EDGE_FALLING);
	set_irq_data(chip->irq, chip);
	set_irq_chained_handler(chip->irq, pcf857x_irq_demux);

	INIT_WORK(&chip->irq_work, pcf857x_irq_work);
	return 0;
}
#else
static inline int pcf857x_init_irq(struct pcf857x *chip)
{
	return 0;
}
#endif /* CONFIG_GPIO_PCF857X_GENERIC_IRQ */

/*-------------------------------------------------------------------------*/

static int pcf857x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pcf857x_platform_data	*pdata;
	struct pcf857x			*gpio;
	int				status;

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_dbg(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	/* Allocate, initialize, and register this gpio_chip. */
	gpio = kzalloc(sizeof *gpio, GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	mutex_init(&gpio->lock);

	gpio->chip.base = pdata->gpio_base;
	gpio->chip.can_sleep = 1;
	gpio->chip.dev = &client->dev;
	gpio->chip.owner = THIS_MODULE;

	/* NOTE:  the OnSemi jlc1562b is also largely compatible with
	 * these parts, notably for output.  It has a low-resolution
	 * DAC instead of pin change IRQs; and its inputs can be the
	 * result of comparators.
	 */

	/* 8574 addresses are 0x20..0x27; 8574a uses 0x38..0x3f;
	 * 9670, 9672, 9764, and 9764a use quite a variety.
	 *
	 * NOTE: we don't distinguish here between *4 and *4a parts.
	 */
	gpio->chip.ngpio = id->driver_data;
	if (gpio->chip.ngpio == 8) {
		gpio->chip.direction_input = pcf857x_input8;
		gpio->chip.get = pcf857x_get8;
		gpio->chip.direction_output = pcf857x_output8;
		gpio->chip.set = pcf857x_set8;

		if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE))
			status = -EIO;

		/* fail if there's no chip present */
		else
			status = i2c_smbus_read_byte(client);

	/* '75/'75c addresses are 0x20..0x27, just like the '74;
	 * the '75c doesn't have a current source pulling high.
	 * 9671, 9673, and 9765 use quite a variety of addresses.
	 *
	 * NOTE: we don't distinguish here between '75 and '75c parts.
	 */
	} else if (gpio->chip.ngpio == 16) {
		gpio->chip.direction_input = pcf857x_input16;
		gpio->chip.get = pcf857x_get16;
		gpio->chip.direction_output = pcf857x_output16;
		gpio->chip.set = pcf857x_set16;

		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
			status = -EIO;

		/* fail if there's no chip present */
		else
			status = i2c_read_le16(client);

	} else {
		dev_dbg(&client->dev, "unsupported number of gpios\n");
		status = -EINVAL;
	}

	if (status < 0)
		goto fail;

	gpio->chip.label = client->name;

	gpio->client = client;
	i2c_set_clientdata(client, gpio);

	/* NOTE:  these chips have strange "quasi-bidirectional" I/O pins.
	 * We can't actually know whether a pin is configured (a) as output
	 * and driving the signal low, or (b) as input and reporting a low
	 * value ... without knowing the last value written since the chip
	 * came out of reset (if any).  We can't read the latched output.
	 *
	 * In short, the only reliable solution for setting up pin direction
	 * is to do it explicitly.  The setup() method can do that, but it
	 * may cause transient glitching since it can't know the last value
	 * written (some pins may need to be driven low).
	 *
	 * Using pdata->n_latch avoids that trouble.  When left initialized
	 * to zero, our software copy of the "latch" then matches the chip's
	 * all-ones reset state.  Otherwise it flags pins to be driven low.
	 */
	gpio->out = ~pdata->n_latch;

	status = gpiochip_add(&gpio->chip);
	if (status < 0)
		goto fail;

	/* NOTE: these chips can issue "some pin-changed" IRQs, which we
	 * don't yet even try to use.  Among other issues, the relevant
	 * genirq state isn't available to modular drivers; and most irq
	 * methods can't be called from sleeping contexts.
	 */

	dev_info(&client->dev, "gpios %d..%d on a %s (irq %d)\n",
			gpio->chip.base,
			gpio->chip.base + gpio->chip.ngpio - 1,
			client->name,
			client->irq);

#if 1
	/*
	 * Required (maybe temprarily) to allow the GPIO expander interrupt
	 * to be pulled up properly, and therefore work properly.
	 */
	{
		int	value, i;

		/* force camera power on */
		pcf857x_set16(&(gpio->chip), 12, 1);

		for (i=0; i < 4; i++) {
			value = i2c_read_le16(gpio->client);
			printk(KERN_CRIT "%s: val(%d) = 0x%04x\n", __FUNCTION__, i, value);
		}
	}
#endif

	/* Let platform code set up the GPIOs and their users.
	 * Now is the first time anyone could use them.
	 */
	if (pdata->setup) {
		status = pdata->setup(client,
				gpio->chip.base, gpio->chip.ngpio,
				pdata->context);
		if (status < 0)
			dev_warn(&client->dev, "setup --> %d\n", status);
	}

#ifdef CONFIG_GPIO_PCF857X_GENERIC_IRQ
	if(client->irq > 0){
		/* clear the interrupt */
		status = i2c_read_le16(client);
		if (status < 0)
			goto fail;

		gpio->last_input = status & 0xffff;
		status = pcf857x_init_irq(gpio);
		if (status) {
			status = gpiochip_remove(&gpio->chip);
			goto fail;
		}

	}		
#endif

	return 0;

fail:
	dev_dbg(&client->dev, "probe error %d for '%s'\n",
			status, client->name);
	kfree(gpio);
	return status;
}

#ifdef CONFIG_GPIO_PCF857X_GENERIC_IRQ
static int pcf857x_remove(struct i2c_client *client)
{
	printk(KERN_ERR "failed to unload the driver with IRQ support\n");
	return -EINVAL;
}
#else
static int pcf857x_remove(struct i2c_client *client)
{
	struct pcf857x_platform_data	*pdata = client->dev.platform_data;
	struct pcf857x			*gpio = i2c_get_clientdata(client);
	int				status = 0;

	if (pdata->teardown) {
		status = pdata->teardown(client,
				gpio->chip.base, gpio->chip.ngpio,
				pdata->context);
		if (status < 0) {
			dev_err(&client->dev, "%s --> %d\n",
					"teardown", status);
			return status;
		}
	}

	status = gpiochip_remove(&gpio->chip);
	if (status == 0)
		kfree(gpio);
	else
		dev_err(&client->dev, "%s --> %d\n", "remove", status);
	return status;
}
#endif

#ifdef CONFIG_PM
#define LAST_GPIO_INPUT 10
static
int pcf857x_suspend(struct i2c_client *client, pm_message_t mesg) {
	register int i;
	struct pcf857x *gpio = i2c_get_clientdata(client);

/*
 * We need to change all inputs that we don't want to wake-up the system
 * to outputs at logic-level zero.
 */
	for(i = 4; i <= LAST_GPIO_INPUT; i++)
		pcf857x_output16(&(gpio->chip), i, 0);

	return 0;
}

static
int pcf857x_resume(struct i2c_client *client) {

	register int i;

	struct pcf857x *gpio = i2c_get_clientdata(client);

/*
 * We need to change all outputs that we didn't want to wake-up the system
 * back to inputs.
 */
	for(i = 4; i <= LAST_GPIO_INPUT; i++) {
		pcf857x_output16(&(gpio->chip), i, 1);		// set to high
		pcf857x_input16(&(gpio->chip), i);			// switch directions
		pcf857x_get16(&(gpio->chip), i);			// clear interrupt
	}

	pcf857x_output16(&(gpio->chip), 11, 0);	// restore autoup_stop state

	return 0;
}
#else
#define pcf857x_suspend NULL
#define pcf857x_resume NULL
#endif

static struct i2c_driver pcf857x_driver = {
	.driver = {
		.name	= "pcf857x",
		.owner	= THIS_MODULE,
	},
	.probe	= pcf857x_probe,
	.remove	= pcf857x_remove,
	.id_table = pcf857x_id,
	.suspend = pcf857x_suspend,
	.resume = pcf857x_resume,
};

static int __init pcf857x_init(void)
{
	return i2c_add_driver(&pcf857x_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(pcf857x_init);

static void __exit pcf857x_exit(void)
{
	i2c_del_driver(&pcf857x_driver);
}
module_exit(pcf857x_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Brownell");
