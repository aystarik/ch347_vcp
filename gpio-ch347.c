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
#include <linux/mutex.h>
#include <linux/gpio/driver.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/version.h>

#include "ch347.h"

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 5, 0))
#define GPIO_LINE_DIRECTION_IN	1
#define GPIO_LINE_DIRECTION_OUT	0
#endif

/*
 * gpio_chip.set()/set_multiple() returned void before 6.16 and return an int
 * from 6.16 on ("gpiolib: indicate errors in value setters"), so the setters
 * below need a version-dependent signature. CH347_GPIO_SET_FN() supplies the
 * return type and CH347_GPIO_SET_DONE() performs the matching return.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 16, 0))
#define CH347_GPIO_SET_FN(fn)		int fn
#define CH347_GPIO_SET_DONE(rc)		return (rc)
#else
#define CH347_GPIO_SET_FN(fn)		void fn
#define CH347_GPIO_SET_DONE(rc)		do { (void)(rc); return; } while (0)
#endif

#define CH347_GPIO_NUM_PINS	8

struct ch347_gpio {
	struct platform_device *pdev;
	struct gpio_chip gpio;
	struct mutex lock;	/* serialize accesses to obuf/ibuf */
	u8 ibuf[3 + 8];
	u8 obuf[3 + 8];
};

static void ch347_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	unsigned i;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	u8 pins[8];

	/*
	 * The cached state is refreshed by the get/set/direction paths. This
	 * callback has no context guarantee, so only read the cache when the
	 * lock is free (mutex_trylock() never sleeps) and never start a
	 * transfer from here.
	 */
	if (!mutex_trylock(&ch347->lock)) {
		seq_puts(s, "gpio state busy\n");
		return;
	}
	memcpy(pins, ch347->ibuf + 3, sizeof(pins));
	mutex_unlock(&ch347->lock);

	for (i = 0; i < 8; ++i) {
		u8 pin = pins[i];
		seq_printf(s, "gpio-%-1d-%s %s\n", i, pin & 0x80 ? "out":"in ", pin & 0x40 ? "hi" : "lo");
	}
}

static int gpio_transfer(struct ch347_gpio *dev)
{
	return ch347_xfer(dev->pdev, dev->obuf, 11, dev->ibuf, 11);
}

/* Refresh the cached read state from hardware, must be called with lock held */
static void gpio_refresh(struct ch347_gpio *ch347)
{
	memset(ch347->obuf + 3, 0, 8);
	gpio_transfer(ch347);
}

static int ch347_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	int rc;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	if (offset > 7) return 0;
	mutex_lock(&ch347->lock);
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	rc = gpio_transfer(ch347);
	if (rc < 0) {
		mutex_unlock(&ch347->lock);
		return rc;
	}
	rc = (ch347->ibuf[3 + offset] & 0x40) ? 1 : 0;
	mutex_unlock(&ch347->lock);
	return rc;
}

static int ch347_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	int rc;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	if (offset > 7) return 0;
	mutex_lock(&ch347->lock);
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	rc = gpio_transfer(ch347);
	if (rc < 0) {
		mutex_unlock(&ch347->lock);
		return rc;
	}
	rc = (ch347->ibuf[3 + offset] & 0x80) ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
	mutex_unlock(&ch347->lock);
	return rc;
}

static int ch347_gpio_get_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
	int rc;
	unsigned i;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	mutex_lock(&ch347->lock);
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	rc = gpio_transfer(ch347);

	if (rc < 0) {
		mutex_unlock(&ch347->lock);
		return rc;
	}
	*bits = 0;
	for (i = 0; i < CH347_GPIO_NUM_PINS; ++i) {
		if (*mask & BIT(i) && (ch347->ibuf[3 + i] & 0x40)) {
			*bits |= BIT(i);
		}
	}
	mutex_unlock(&ch347->lock);
	return 0;
}

static CH347_GPIO_SET_FN(ch347_gpio_set)(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	int rc;

	if (offset > 7)
		CH347_GPIO_SET_DONE(0);

	mutex_lock(&ch347->lock);
	gpio_refresh(ch347); // make sure cached direction is current
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	ch347->obuf[3 + offset] |= 0xc0; // enable pin change
	if (ch347->ibuf[3 + offset] & 0x80) { // copy direction
		ch347->obuf[3 + offset] |= 0x30;
	}
	if (value) {
		ch347->obuf[3 + offset] |= 0x08;
	}
	rc = gpio_transfer(ch347);
	mutex_unlock(&ch347->lock);

	CH347_GPIO_SET_DONE(rc < 0 ? -EIO : 0);
}

static CH347_GPIO_SET_FN(ch347_gpio_set_multiple)(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{
	unsigned i;
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	int rc;

	mutex_lock(&ch347->lock);
	gpio_refresh(ch347); // make sure cached directions are current
	memset(ch347->obuf + 3, 0, 8); // clear all pins
	for (i = 0; i < 8; ++i) {
		if (*mask & BIT(i) && (ch347->ibuf[3 + i] & 0x80)) {
			ch347->obuf[3 + i] |= 0xf0; // enable pin change & copy direction
			if (*bits & BIT(i)) {
				ch347->obuf[3 + i] |= 0x08;
			}
		}
	}
	rc = gpio_transfer(ch347);
	mutex_unlock(&ch347->lock);

	CH347_GPIO_SET_DONE(rc < 0 ? -EIO : 0);
}

static int ch347_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	int rc;
	if (offset > 7) return 0;
	mutex_lock(&ch347->lock);
	gpio_refresh(ch347); // make sure cached value is current
	memset(3 + ch347->obuf, 0, 8); // clear all pins
	ch347->obuf[3 + offset] |= 0xc0; // enable pin change
	if (ch347->ibuf[3 + offset] & 0x40) { // copy value
		ch347->obuf[3 + offset] |= 0x08;
	}

	rc = (gpio_transfer(ch347) < 0) ? -EIO : 0;
	mutex_unlock(&ch347->lock);

	return rc;
}

static int ch347_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ch347_gpio *ch347 = gpiochip_get_data(chip);
	int rc;
	if (offset > 7) return 0;
	mutex_lock(&ch347->lock);
	gpio_refresh(ch347); // make sure cached value is current
	memset(3 + ch347->obuf, 0, 8); // clear all pins
	ch347->obuf[3 + offset] |= 0xf0; // enable pin change & output
	if (value) { // drive the requested level, not the cached one
		ch347->obuf[3 + offset] |= 0x08;
	}

	rc = (gpio_transfer(ch347) < 0) ? -EIO : 0;
	mutex_unlock(&ch347->lock);

	return rc;
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
	mutex_init(&ch347->lock);
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

	ret = gpio_transfer(ch347);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to configure GPIO: %d", __func__, ret);
		return ret;
	}

	ret = devm_gpiochip_add_data(dev, &ch347->gpio, ch347);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to add gpio chip: %d", __func__, ret);
		return ret;
	}

	return 0;
}

static void ch347_gpio_remove(struct platform_device *pdev) {}

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
