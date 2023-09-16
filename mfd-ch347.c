// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core driver for QinHeng Electronics CH347 USB adapter
 *
 * Copyright (c) 2023 Alexey Starikovskiy <aystarik@gmail.com>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/rculist.h>

#include "ch347.h"

#define CH347_USB_VENDOR 0x1a86
#define CH347_USB_DEVICE_M1 0x55db
#define CH347_USB_DEVICE_M3 0x55dd

#define DEFAULT_TIMEOUT 300

struct ch347_dev {
	struct usb_device *usb_dev;
	struct usb_interface *interface;
	struct mutex lck;
	u8 ep_in;
	u8 ep_out;
	bool mode3;
};

static const struct mfd_cell ch347_devs[] = {
	{ .name = "ch347-gpio", },
	{ .name = "ch347-i2c", },
	{ .name = "ch347-spi", }, // must be last -- optional
};

static void ch347_free(struct ch347_dev *ch347)
{
	usb_put_dev(ch347->usb_dev);
	kfree(ch347);
}

int ch347_xfer(struct platform_device *pdev,
	       const uint8_t *obuf, unsigned obuf_len,
	       uint8_t *ibuf, unsigned ibuf_len)
{
	int ret = 0;
	int actual = 0;
	struct ch347_dev *ch347 = dev_get_drvdata(pdev->dev.parent);
	//dev_info(&pdev->dev, "ch347_xfer: %p, %d, %p, %d", obuf, obuf_len, ibuf, ibuf_len);
	mutex_lock(&ch347->lck);
	if (obuf_len) {
		ret = usb_bulk_msg(ch347->usb_dev, usb_sndbulkpipe(ch347->usb_dev, ch347->ep_out),
				   (void *)obuf, obuf_len, &actual, DEFAULT_TIMEOUT);
		if (ret < 0) {
			dev_err(&pdev->dev, "send failed");
			goto unlock;
		}
	}
	if (ibuf_len) {
		memset(ibuf, 0, ibuf_len);
		ret = usb_bulk_msg(ch347->usb_dev, usb_rcvbulkpipe(ch347->usb_dev, ch347->ep_in),
				   ibuf, ibuf_len, &actual, DEFAULT_TIMEOUT);

		if (ret < 0) {
			dev_err(&pdev->dev, "recv failed");
			goto unlock;
		}
	}
unlock:
	mutex_unlock(&ch347->lck);
	if (ret)
		return ret;
	return actual;

}
EXPORT_SYMBOL(ch347_xfer);

bool ch347_mode3(struct platform_device *pdev)
{
	struct ch347_dev *ch347 = dev_get_drvdata(pdev->dev.parent);
	return ch347->mode3;
}
EXPORT_SYMBOL(ch347_mode3);

static void ch347_disconnect(struct usb_interface *interface)
{
	struct ch347_dev *ch347 = usb_get_intfdata(interface);

	mfd_remove_devices(&interface->dev);

	ch347_free(ch347);
}


static int ch347_probe(struct usb_interface *interface, const struct usb_device_id *usb_id)
{
	struct usb_host_interface *hostif = interface->cur_altsetting;
	struct usb_endpoint_descriptor *epin;
	struct usb_endpoint_descriptor *epout;
	struct device *dev = &interface->dev;
	struct ch347_dev *ch347;
	int ret;

	if (hostif->desc.bInterfaceNumber != 2 ||
	    hostif->desc.bNumEndpoints < 2)
		return -ENODEV;

	epout = &hostif->endpoint[0].desc;
	if (!usb_endpoint_is_bulk_out(epout))
		return -ENODEV;
	epin = &hostif->endpoint[1].desc;
	if (!usb_endpoint_is_bulk_in(epin))
		return -ENODEV;

	ch347 = kzalloc(sizeof(*ch347), GFP_KERNEL);
	if (!ch347)
		return -ENOMEM;

	ch347->ep_out = epout->bEndpointAddress;
	ch347->ep_in = epin->bEndpointAddress;
	ch347->usb_dev = usb_get_dev(interface_to_usbdev(interface));
	ch347->interface = interface;
	mutex_init(&ch347->lck);
	usb_set_intfdata(interface, ch347);
	ch347->mode3 = usb_id->idProduct == CH347_USB_DEVICE_M3;
	ret = mfd_add_hotplug_devices(dev, ch347_devs, ch347->mode3 ? 2 : 3);
	if (ret != 0) {
		dev_err(dev, "failed to add mfd devices to core\n");
		goto out_free;
	}
	return 0;

out_free:
	ch347_free(ch347);
	return ret;
}

static const struct usb_device_id ch347_table[] = {
	{ USB_DEVICE(CH347_USB_VENDOR, CH347_USB_DEVICE_M1) }, /* mode #1 (UART + SPI + I2C + GPIO) */
	{ USB_DEVICE(CH347_USB_VENDOR, CH347_USB_DEVICE_M3) }, /* mode #3 (UART + JTAG + I2C + GPIO) */
	{ }
};

MODULE_DEVICE_TABLE(usb, ch347_table);

static struct usb_driver ch347_driver = {
	.name = "ch347",
	.disconnect = ch347_disconnect,
	.probe = ch347_probe,
	.id_table = ch347_table,
};

module_usb_driver(ch347_driver);

MODULE_DESCRIPTION("Core driver for QinHeng Electronics CH347 USB adapter");
MODULE_AUTHOR("Alexey Starikovskiy <aystarik@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ch347-mfd");
