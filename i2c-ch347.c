// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core driver for QinHeng Electronics CH347 USB-I2C adapter
 *
 * Copyright (c) 2023 Alexey Starikovskiy <aystarik@gmail.com>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>

#include "ch347.h"

enum ch347_speed_t {
	CH347_I2C_LOW_SPEED = 0,	// low speed - 20kHz
	CH347_I2C_STANDARD_SPEED = 1,	// standard speed - 100kHz
	CH347_I2C_FAST_SPEED = 2,	// fast speed - 400kHz
	CH347_I2C_HIGH_SPEED = 3,	// high speed - 750kHz
};

#define CH347_CMD_I2C_STREAM 0xAA
#define CH347_CMD_I2C_STM_END 0x00

#define CH347_CMD_I2C_STM_STA 0x74
#define CH347_CMD_I2C_STM_STO 0x75
#define CH347_CMD_I2C_STM_OUT 0x80
#define CH347_CMD_I2C_STM_IN  0xC0
#define CH347_CMD_I2C_STM_SET 0x60

#define CH347_MAX_I2C_XFER 0x3f

#define CH347_I2C_BUF_SIZE 0x50 // make it bigger than XFER

struct ch347_i2c {
	struct platform_device *pdev;
	struct i2c_adapter adapter;
	struct mutex io_mutex;

	uint8_t ibuf[CH347_I2C_BUF_SIZE];
	uint8_t obuf[CH347_I2C_BUF_SIZE];
};

static int ch347_i2c_read(struct ch347_i2c *ch347, struct i2c_msg *msg)
{
	int byteoffset = 0, bytestoread;
	int ret = 0;
	uint8_t *ptr;

	while (msg->len - byteoffset > 0) {
		bytestoread = msg->len - byteoffset;
		if (bytestoread > CH347_MAX_I2C_XFER)
			bytestoread = CH347_MAX_I2C_XFER;
		ptr = ch347->obuf;
		*ptr++ = CH347_CMD_I2C_STREAM;
		*ptr++ = CH347_CMD_I2C_STM_STA;
		*ptr++ = CH347_CMD_I2C_STM_OUT | 1;
		*ptr++ = (msg->addr << 1) | 1;
		if (bytestoread > 1) {
			*ptr++ = CH347_CMD_I2C_STM_IN | (bytestoread - 1);
		}
		*ptr++ = CH347_CMD_I2C_STM_IN;
		*ptr++ = CH347_CMD_I2C_STM_STO;
		*ptr++ = CH347_CMD_I2C_STM_END;

		ret = ch347_xfer(ch347->pdev, ch347->obuf, ptr - ch347->obuf,
			ch347->ibuf, bytestoread + 1);
		if (ret > 0) {
			if (ret != bytestoread + 1)
				ret = -1;
			if (ch347->ibuf[0] != 1)
				ret = -ETIMEDOUT;
			if (ret > -1) {
				memcpy(&msg->buf[byteoffset], &ch347->ibuf[1], bytestoread);
				byteoffset += bytestoread;
			}
		}

		if (ret < 0)
			break;
	}
	return ret;
}

static int ch347_i2c_write(struct ch347_i2c *ch347, struct i2c_msg *msg) {
	unsigned left = msg->len, wlen;
	uint8_t *ptr = msg->buf, *outptr;
	int ret = 0, i;
	bool first = true;

	do {
		outptr = ch347->obuf;
		*outptr++ = CH347_CMD_I2C_STREAM;
		wlen = left;
		if (wlen > CH347_MAX_I2C_XFER - 1)
			wlen = CH347_MAX_I2C_XFER - 1;
		if (first) { // Start packet
			*outptr++ = CH347_CMD_I2C_STM_STA;
			*outptr++ = CH347_CMD_I2C_STM_OUT | (wlen + 1);
			*outptr++ = msg->addr << 1;
		} else {
			*outptr++ = CH347_CMD_I2C_STM_OUT | wlen;
		}
		memcpy(outptr, ptr, wlen);
		outptr += wlen;
		ptr += wlen;
		left -= wlen;
		if (left == 0) {  // Stop packet
			*outptr++ = CH347_CMD_I2C_STM_STO;
		}
		*outptr++ = CH347_CMD_I2C_STM_END;
		ret = ch347_xfer(ch347->pdev, ch347->obuf, outptr - ch347->obuf,
			ch347->ibuf, wlen + (first ? 1 : 0));
		first = false;
		if (ret < 0)
			return ret;
		for (i = 0; i < ret; ++i) {
			if (ch347->ibuf[i] != 1)
				return -ETIMEDOUT;
		}
	} while (left);

	return ret;
}

// ----- configurable during runtime ---------------------------
static int speed = CH347_I2C_STANDARD_SPEED; // module parameter speed, default standard speed

static int ch347_i2c_set_speed(struct ch347_i2c *dev) {
	int rv;
	static int speed_last	= -1;
	static char* i2c_speed_desc[] = { "20 kbps", "100 kbps", "400 kbps", "750 kbps" };
	if (speed == speed_last)
		return 0;
	if (speed < CH347_I2C_LOW_SPEED || speed > CH347_I2C_HIGH_SPEED)
	{
		dev_err(&dev->pdev->dev, "%s: Parameter speed can only have values from 0 to 3", __func__);
		speed = speed_last;
		return -EINVAL;
	}
	dev_info(&dev->pdev->dev, "Change I2C bus speed to %s", i2c_speed_desc[speed]);
	dev->obuf[0] = CH347_CMD_I2C_STREAM;
	dev->obuf[1] = CH347_CMD_I2C_STM_SET | speed;
	dev->obuf[2] = CH347_CMD_I2C_STM_END;

	rv = ch347_xfer(dev->pdev, dev->obuf, 3, NULL, 0);
	if (rv < 0)
		return rv;

	speed_last = speed;
	return 0;
}

static int ch347_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct ch347_i2c *ch347 = i2c_get_adapdata(adapter);
	int ret;
	int i;

	mutex_lock(&ch347->io_mutex);

	ret = ch347_i2c_set_speed(ch347);
	if (ret < 0)
		goto exit;

	for (i = 0; i < num; ++i) {
		if (msgs[i].flags & I2C_M_RD) {
			ret = ch347_i2c_read(ch347, &msgs[i]); // checks for NACK of msg->addr
			if (ret < 0) {
				dev_dbg(&ch347->pdev->dev,
					"%s: Read I2C message %d/%d failed: %d", __func__, i + 1, num, ret);
				goto exit;
			}
			msgs[i].len = ret;
		} else {
			ret = ch347_i2c_write(ch347, &msgs[i]);
			if (ret < 0) {
				dev_dbg(&ch347->pdev->dev,
					"%s: Write I2C message %d/%d failed: %d", __func__, i + 1, num, ret);
				goto exit;
			}
		}
	}

	ret = num;

exit:
	mutex_unlock(&ch347->io_mutex);
	return ret;
}

static u32 ch347_i2c_func(struct i2c_adapter *a)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ch347_i2c_usb_algorithm = {
	.master_xfer = ch347_i2c_xfer,
	.functionality = ch347_i2c_func,
};

static int ch347_i2c_probe(struct platform_device *pdev)
{
	struct ch347_i2c *ch347;
	struct device *dev = &pdev->dev;
	int rv;

	ch347 = devm_kzalloc(dev, sizeof(*ch347), GFP_KERNEL);
	if (!ch347)
		return -ENOMEM;
	ch347->pdev = pdev;
	ch347->adapter.owner = THIS_MODULE;
	ch347->adapter.class = I2C_CLASS_HWMON;
	ch347->adapter.algo = &ch347_i2c_usb_algorithm;
	ch347->adapter.dev.parent = dev;
	i2c_set_adapdata(&ch347->adapter, ch347);
	if (ch347_mode(pdev) == CH347_MODE_3) {
		// force GPIO #3 (SCL) to output high
		ch347->obuf[0] = 0xcc;
		ch347->obuf[1] = 8;
		ch347->obuf[3 + 3] = 0xf8;
		ch347_xfer(pdev, ch347->obuf, 11, NULL, 0);
	}
	snprintf(ch347->adapter.name, sizeof(ch347->adapter.name), "%s-%s", "ch347-i2c", dev_name(pdev->dev.parent));
	platform_set_drvdata(pdev, ch347);

	mutex_init(&ch347->io_mutex);

	rv = ch347_i2c_set_speed(ch347);
	if (rv < 0) {
		dev_err(&pdev->dev, "%s: Failed to set I2C speed: %d", __func__, rv);
		return rv;
	}

	rv = i2c_add_adapter(&ch347->adapter);
	return rv;
}

static int ch347_i2c_remove(struct platform_device *pdev)
{
	struct ch347_i2c *ch347 = platform_get_drvdata(pdev);

	i2c_del_adapter(&ch347->adapter);
	return 0;
}

static struct platform_driver ch347_i2c_driver = {
	.driver.name = "ch347-i2c",
	.probe = ch347_i2c_probe,
	.remove	= ch347_i2c_remove,
};

module_platform_driver(ch347_i2c_driver);

MODULE_DESCRIPTION("Driver for the QinHeng Electronics CH347 I2C master interface");
MODULE_AUTHOR("Alexey Starikovskiy <aystarik@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ch347-i2c");

module_param(speed, int, 0644);
MODULE_PARM_DESC(speed, "I2C bus speed: 0 (20 kbps), 1 (100 kbps), 2 (400 kbps), 3 (750 kbps)");
