// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core driver for QinHeng Electronics CH347 USB-GPIO adapter
 *
 * Copyright (c) 2023 Alexey Starikovskiy <aystarik@gmail.com>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>

#include "ch347.h"

#define CH347_GPIO_NUM_PINS	8

struct ch347_gpio {
	struct platform_device *pdev;
	struct gpio_chip gpio;
	u8 ibuf[3 + 8];
	u8 obuf[3 + 8];

};

static void ch347_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	unsigned i;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	for (i = 0; i < 8; ++i) {
		u8 pin = ch347->ibuf[3 + i];
		seq_printf(s, "gpio-%-1d-%s %s\n", i, pin & 0x80 ? "out":"in ", pin & 0x40 ? "hi" : "lo");
	}
}

static int gpio_transfer(struct ch347_gpio *dev)
{
	return ch347_xfer(dev->pdev, dev->obuf, 11, dev->ibuf, 11);
}

static int ch347_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	int rc;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
//dev_err(ch347->gpio.parent, "gpio_get[%d] start", offset);
	if (offset > 7) return 0;
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	rc = gpio_transfer(ch347);
	if (rc < 0)
		return rc;
//dev_err(ch347->gpio.parent, "gpio_get[%d]=%d", offset, ch347->ibuf[3 + offset] & 0x40);
	return (ch347->ibuf[3 + offset] & 0x40) ? 1 : 0;
}

static int ch347_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	int rc;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
//dev_err(ch347->gpio.parent, "gpio_get_direction[%d] start", offset);
	if (offset > 7) return 0;
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	rc = gpio_transfer(ch347);
	if (rc < 0)
		return rc;
//dev_err(ch347->gpio.parent, "gpio_get_direction[%d]=%d", offset, ch347->ibuf[3 + offset] & 0x80);
	return (ch347->ibuf[3 + offset] & 0x80) ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int ch347_gpio_get_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
	int rc;
	unsigned i;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	rc = gpio_transfer(ch347);

	if (rc < 0)
		return rc;
	*bits = 0;
	for (i = 0; i < CH347_GPIO_NUM_PINS; ++i) {
		if (*mask & BIT(i) && (ch347->ibuf[3 + i] & 0x40)) {
			*bits |= BIT(i);
		}
	}
//dev_info(ch347->gpio.parent, "get_multi[%lx]=%lx", *mask, *bits);
	return 0;
}

static void ch347_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	if (offset > 7) return;
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	ch347->obuf[3 + offset] |= 0xc0; // enable pin change
	if (ch347->ibuf[3 + offset] & 0x80) { // copy direction
		ch347->obuf[3 + offset] |= 0x30;
	}
	if (value) {
		ch347->obuf[3 + offset] |= 0x08;
	}
	gpio_transfer(ch347);
}

static void ch347_gpio_set_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
	unsigned i;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	for (i = 0; i < 8; ++i) {
		if (*mask & BIT(i) && (ch347->ibuf[3 + i] & 0x80)) {
			ch347->obuf[3 + i] |= 0xf0; // enable pin change & copy direction
			if (*bits & BIT(i)) {
				ch347->obuf[3 + i] |= 0x08;
			}
		}
	}
	gpio_transfer(ch347);
}

static int ch347_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	if (offset > 7) return 0;
	memset(3 + ch347->obuf, 0, 8); // clear all pins
	ch347->obuf[3 + offset] |= 0xc0; // enable pin change
	if (ch347->ibuf[3 + offset] & 0x40) { // copy value
		ch347->obuf[3 + offset] |= 0x08;
	}
	gpio_transfer(ch347);

	return 0;
}

static int ch347_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	if (offset > 7) return 0;
	memset(3 + ch347->obuf, 0, 8); // clear all pins
	ch347->obuf[3 + offset] |= 0xf0; // enable pin change & output
	if (ch347->ibuf[3 + offset] & 0x40) { // copy value
		ch347->obuf[3 + offset] |= 0x08;
	}
	gpio_transfer(ch347);

	return 0;
}

static int ch347_gpio_probe(struct platform_device *pdev)
{
	struct ch347_gpio *ch347;
	struct device *dev = &pdev->dev;
	int ret;

	ch347 = devm_kzalloc(dev, sizeof(struct ch347_gpio), GFP_KERNEL);
	if (!ch347)
		return -ENOMEM;
	ch347->pdev = pdev;
	ch347->gpio.label = "ch347";
	ch347->gpio.parent = dev;
	ch347->gpio.owner = THIS_MODULE;
	ch347->gpio.base = -1;
	ch347->gpio.ngpio = CH347_GPIO_NUM_PINS;
	ch347->gpio.can_sleep = true;

	ch347->gpio.dbg_show = ch347_gpio_dbg_show;

	ch347->gpio.set = ch347_gpio_set;
	ch347->gpio.set_multiple = ch347_gpio_set_multiple;

	ch347->gpio.get = ch347_gpio_get;
	ch347->gpio.get_multiple = ch347_gpio_get_multiple;

	ch347->gpio.get_direction = ch347_gpio_get_direction;
	ch347->gpio.direction_input = ch347_gpio_direction_input;
	ch347->gpio.direction_output = ch347_gpio_direction_output;

	platform_set_drvdata(pdev, ch347);

	memset(ch347->obuf, 0, 11);
	ch347->obuf[0] = 0xcc; // these fields do not ever change
	ch347->obuf[1] = 8; // these fields do not ever change
	ch347->obuf[2] = 0; // these fields do not ever change

	dev_info(dev, "ch347 gpio driver");

	gpio_transfer(ch347);

	ret = devm_gpiochip_add_data(dev, &ch347->gpio, ch347);
	if (ret < 0) {
		dev_err(dev, "failed to add gpio chip: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ch347_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ch347_gpio_driver = {
	.driver.name	= "ch347-gpio",
	.probe		= ch347_gpio_probe,
	.remove		= ch347_gpio_remove,
};

module_platform_driver(ch347_gpio_driver);

MODULE_DESCRIPTION("Driver for the QinHeng Electronics CH347 GPIO interface");
MODULE_AUTHOR("Alexey Starikovskiy <aystarik@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ch347-gpio");
