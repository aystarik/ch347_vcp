// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core driver for QinHeng Electronics CH347 USB adapter
 *
 * Copyright (c) 2023 Alexey Starikovskiy <aystarik@gmail.com>
 *
 */

//#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/rculist.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/jiffies.h>

#include "ch347.h"

#define CH347_PACKET_LENGTH   510

#define CH347_RX_BUFFERS      4
#define CH347_RX_BUFFER_SIZE  CH347_PACKET_LENGTH

#define CH347_TX_BUFFERS      8
#define CH347_TX_BUFFER_SIZE  4096

#define CH347_USB_VENDOR 0x1a86
#define CH347_USB_DEVICE_M1 0x55db
#define CH347_USB_DEVICE_M3 0x55dd

#define CH347_DEFAULT_TIMEOUT 1000

struct ch347_urb {
	uint8_t *buf_dma;
	struct urb *urb;
	unsigned size;
};

#define RXB_FLAG_NO_RESUBMIT  0x01

struct ch347_rx_buffer {
	int index;
	struct ch347_urb *urb;
	struct ch347_dev *ch347;

	unsigned flags;
	unsigned length;
	unsigned actual_length;
	int status;
	struct completion complete;
};

struct ch347_tx_buffer {
	int index;
	struct ch347_urb *urb;
	struct ch347_dev *ch347;
};

struct ch347_dev {
	struct usb_device *usb_dev;
	struct usb_interface *interface;

	struct mutex io_mutex;
	struct usb_anchor submitted;

	DECLARE_BITMAP(rxb_bitmap, CH347_RX_BUFFERS);
	struct ch347_rx_buffer rxb[CH347_RX_BUFFERS];
	spinlock_t rxb_lock;
	struct semaphore rx_limit_sem;

	DECLARE_BITMAP(txb_bitmap, CH347_TX_BUFFERS);
	struct ch347_tx_buffer txb[CH347_TX_BUFFERS];
	spinlock_t txb_lock;
	struct semaphore tx_limit_sem;

	int errors;
	spinlock_t err_lock;

	u8 ep_in;
	u8 ep_out;
	ch347_mode_t mode;
};

static const struct mfd_cell ch347_devs[] = {
	{ .name = "ch347-gpio", },
	{ .name = "ch347-i2c", },
	{ .name = "ch347-spi", }, // must be last -- optional
};

static struct ch347_urb *ch347_urb_alloc(struct ch347_dev *ch347, unsigned size)
{
	struct ch347_urb *urb;

	urb = kzalloc(sizeof(*urb), GFP_KERNEL);
	if (!urb)
		return NULL;

	urb->size = size;
	urb->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb->urb) {
		kfree(urb);
		return NULL;
	}

	urb->buf_dma = usb_alloc_coherent(ch347->usb_dev,
		urb->size, GFP_KERNEL, &urb->urb->transfer_dma);
	if (!urb->buf_dma) {
		usb_free_urb(urb->urb);
		kfree(urb);
		return NULL;
	}

	return urb;
}

static void ch347_urb_free(struct ch347_dev *ch347, struct ch347_urb *urb)
{
	if (!urb)
		return;

	if (urb->urb)
		usb_kill_urb(urb->urb);

	if (urb->buf_dma && urb->urb)
		usb_free_coherent(ch347->usb_dev, urb->size,
			urb->buf_dma, urb->urb->transfer_dma);

	if (urb->urb)
		usb_free_urb(urb->urb);

	kfree(urb);
}

static void ch347_free_buffers(struct ch347_dev *ch347)
{
	int i;

	for (i = 0; i < CH347_RX_BUFFERS; ++i) {
		if (ch347->rxb[i].urb) {
			ch347_urb_free(ch347, ch347->rxb[i].urb);
		}
	}

	for (i = 0; i < CH347_TX_BUFFERS; ++i) {
		if (ch347->txb[i].urb) {
			ch347_urb_free(ch347, ch347->txb[i].urb);
		}
	}
}

static int ch347_init_buffers(struct ch347_dev *ch347)
{
	int i;
	int retval = 0;

	sema_init(&ch347->rx_limit_sem, CH347_RX_BUFFERS);
	sema_init(&ch347->tx_limit_sem, CH347_RX_BUFFERS);
	spin_lock_init(&ch347->rxb_lock);
	spin_lock_init(&ch347->txb_lock);

	for (i = 0; i < CH347_RX_BUFFERS; ++i) {
		ch347->rxb[i].ch347 = ch347;
		ch347->rxb[i].urb = ch347_urb_alloc(ch347, CH347_RX_BUFFER_SIZE);
		init_completion(&ch347->rxb[i].complete);

		if (!ch347->rxb[i].urb) {
			retval = -ENOMEM;
			goto out_free;
		}

		ch347->rxb[i].index = i;
	}

	for (i = 0; i < CH347_TX_BUFFERS; ++i) {
		ch347->txb[i].ch347 = ch347;
		ch347->txb[i].urb = ch347_urb_alloc(ch347, CH347_TX_BUFFER_SIZE);
		if (!ch347->txb[i].urb) {
			retval = -ENOMEM;
			goto out_free;
		}

		ch347->txb[i].index = i;
	}

	return 0;

out_free:
	ch347_free_buffers(ch347);
	return retval;
}

static int __get_free_buf_index(
	struct semaphore *limit_sem, spinlock_t *lock,
	unsigned long *bitmap, unsigned count
)
{
	unsigned long flags;
	int index;

	if (down_interruptible(limit_sem))
		return count;

	spin_lock_irqsave(lock, flags);

	index = find_first_zero_bit(bitmap, count);
	if (index < count)
		set_bit(index, bitmap);

	spin_unlock_irqrestore(lock, flags);

	return index;
}

static struct ch347_rx_buffer *ch347_get_rx_buffer(struct ch347_dev *ch347)
{
	int index = __get_free_buf_index(
		&ch347->rx_limit_sem, &ch347->rxb_lock,
		ch347->rxb_bitmap, CH347_RX_BUFFERS);

	if (index < CH347_RX_BUFFERS)
		return &ch347->rxb[index];

	return NULL;
}

static void ch347_put_rx_buffer(struct ch347_dev *ch347, struct ch347_rx_buffer *rxb)
{
	unsigned long flags;

	spin_lock_irqsave(&ch347->rxb_lock, flags);
	clear_bit(rxb->index, ch347->rxb_bitmap);
	reinit_completion(&rxb->complete);
	spin_unlock_irqrestore(&ch347->rxb_lock, flags);

	up(&ch347->rx_limit_sem);
}

static struct ch347_tx_buffer *ch347_get_tx_buffer(struct ch347_dev *ch347)
{
	int index = __get_free_buf_index(
		&ch347->tx_limit_sem, &ch347->txb_lock,
		ch347->txb_bitmap, CH347_TX_BUFFERS);

	if (index < CH347_TX_BUFFERS)
		return &ch347->txb[index];

	return NULL;
}

static void ch347_put_tx_buffer(struct ch347_dev *ch347, struct ch347_tx_buffer *txb)
{
	unsigned long flags;

	spin_lock_irqsave(&ch347->txb_lock, flags);
	clear_bit(txb->index, ch347->txb_bitmap);
	spin_unlock_irqrestore(&ch347->txb_lock, flags);

	up(&ch347->tx_limit_sem);
}

static void ch347_draw_down(struct ch347_dev *ch347);

static void ch347_free(struct ch347_dev *ch347)
{
	mutex_lock(&ch347->io_mutex);
	ch347->interface = NULL;
	mutex_unlock(&ch347->io_mutex);

	ch347_draw_down(ch347);
	ch347_free_buffers(ch347);

	usb_put_dev(ch347->usb_dev);
	kfree(ch347);
}

static void ch347_write_bulk_callback(struct urb *urb)
{
	struct ch347_tx_buffer *txb = urb->context;
	struct ch347_dev *ch347 = txb->ch347;

	dev_dbg(&ch347->interface->dev,
		"%s: Write URB callback (len = %u, actual_len = %u)",
		__func__, urb->transfer_buffer_length, urb->actual_length);

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN)) {
			dev_err(&ch347->interface->dev,
				"%s: Nonzero write bulk status received: %d",
				__func__, urb->status);
		}

		spin_lock(&ch347->err_lock);
		ch347->errors = urb->status;
		spin_unlock(&ch347->err_lock);
	}

	/* free up our allocated buffer */
	ch347_put_tx_buffer(ch347, txb);
}

static void ch347_read_bulk_callback(struct urb *urb)
{
	struct ch347_rx_buffer *rxb = urb->context;
	struct ch347_dev *ch347 = rxb->ch347;

	dev_dbg(&ch347->interface->dev,
		"%s: Received read URB (status = %d, len = %u, actual_len = %u, context = 0x%p)",
		__func__, urb->status, rxb->length, urb->actual_length, rxb);

	rxb->actual_length = urb->actual_length;
	rxb->status = urb->status;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN)) {
			dev_err(&ch347->interface->dev,
				"%s: Nonzero read bulk status received: %d",
				__func__, urb->status);
		}

		complete(&rxb->complete);
		return;
	}

	if (!(rxb->flags & RXB_FLAG_NO_RESUBMIT)) {
		if (urb->actual_length != rxb->length) {
			int err;

			err = usb_submit_urb(urb, GFP_ATOMIC);
			if (err < 0) {
				dev_err(&ch347->interface->dev,
					"Failed to resubmit read URB: %d", err);
				return;
			}

			dev_dbg(&ch347->interface->dev,
				"%s: Resubmitted read URB (context = 0x%p)", __func__, rxb);
			return;
		}
	}

	complete(&rxb->complete);
}

static int ch347_data_xfer(
	struct ch347_dev *ch347,
	const uint8_t *obuf, unsigned obuf_len,
	uint8_t *ibuf, unsigned ibuf_len,
	unsigned flags,
	unsigned timeout)
{
	int bytes_read = 0;
	struct ch347_rx_buffer *rxb = NULL;
	struct ch347_tx_buffer *txb = NULL;
	int retval = 0;

	if (ibuf && ((ibuf_len > CH347_RX_BUFFER_SIZE) || (ibuf_len == 0))) {
		retval = -EINVAL;
		goto exit;
	}

	if (obuf && ((obuf_len > CH347_TX_BUFFER_SIZE) || (obuf_len == 0))) {
		retval = -EINVAL;
		goto exit;
	}

	spin_lock_irq(&ch347->err_lock);
	retval = ch347->errors;
	if (retval < 0) {
		ch347->errors = 0;
		retval = (retval == -EPIPE) ? retval : -EIO;
	}
	spin_unlock_irq(&ch347->err_lock);
	if (retval < 0)
		goto error;

	if (obuf) {
		/* Prepare TX buffer */
		txb = ch347_get_tx_buffer(ch347);
		if (!txb) {
			retval = -ENOMEM;
			goto error;
		}

		memcpy(txb->urb->buf_dma, obuf, obuf_len);

		usb_fill_bulk_urb(
			txb->urb->urb, ch347->usb_dev,
			usb_sndbulkpipe(ch347->usb_dev, ch347->ep_out),
			txb->urb->buf_dma, obuf_len,
			ch347_write_bulk_callback, txb);
		txb->urb->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	}

	if (ibuf) {
		/* Prepare RX buffer */
		rxb = ch347_get_rx_buffer(ch347);
		if (!rxb) {
			retval = -ENOMEM;
			goto error;
		}

		rxb->flags  = flags;
		rxb->length = ibuf_len;
		rxb->status = -ETIMEDOUT;

		usb_fill_bulk_urb(
			rxb->urb->urb, ch347->usb_dev,
			usb_rcvbulkpipe(ch347->usb_dev, ch347->ep_in),
			rxb->urb->buf_dma, ibuf_len,
			ch347_read_bulk_callback, rxb);
		rxb->urb->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	}

	mutex_lock(&ch347->io_mutex);
	{
		if (!ch347->interface) {
			mutex_unlock(&ch347->io_mutex);
			retval = -ENODEV;
			goto error;
		}

		if (ibuf) {
			/* Submit RX URB */
			retval = usb_submit_urb(rxb->urb->urb, GFP_KERNEL);
			if (retval) {
				mutex_unlock(&ch347->io_mutex);
				dev_err(&ch347->interface->dev,
					"%s: Failed submitting read URB: %d", __func__, retval);
				goto error;
			}

			dev_dbg(&ch347->interface->dev,
				"%s: Submitted read URB (len = %u, context = 0x%p): %d",
				__func__, ibuf_len, rxb, retval);
		}

		if (obuf) {
			/* Submit TX URB */
			usb_anchor_urb(txb->urb->urb, &ch347->submitted);

			/* send the data out the bulk port */
			retval = usb_submit_urb(txb->urb->urb, GFP_KERNEL);
			if (retval) {
				mutex_unlock(&ch347->io_mutex);
				dev_err(&ch347->interface->dev,
					"%s: Failed submitting write URB: %d", __func__, retval);
				goto error_unanchor;
			}

			dev_dbg(&ch347->interface->dev,
				"%s: Submitted write URB (len = %u): %d", __func__, obuf_len, retval);

			retval = obuf_len;
		} /* obuf */
	}
	mutex_unlock(&ch347->io_mutex);

	if (rxb) {
		retval = -ETIMEDOUT;
		if (wait_for_completion_timeout(&rxb->complete, msecs_to_jiffies(timeout)))
			retval = rxb->status;

		bytes_read = rxb->actual_length;
		if (bytes_read > 0)
			memcpy(ibuf, rxb->urb->buf_dma, bytes_read);

		if (retval) {
			dev_dbg(&ch347->interface->dev,
				"%s: Failed to receive data (len = %u): %d",
				__func__, ibuf_len, retval);
			usb_kill_urb(rxb->urb->urb);
			ch347_put_rx_buffer(ch347, rxb);
			goto exit;
		}

		ch347_put_rx_buffer(ch347, rxb);

		retval = bytes_read;
	} /* ibuf */

	return retval;

error_unanchor:
	if (obuf && txb)
		usb_unanchor_urb(txb->urb->urb);
error:
	if (obuf && txb) {
		usb_kill_urb(txb->urb->urb);
		ch347_put_tx_buffer(ch347, txb);
	}
	if (ibuf && rxb) {
		usb_kill_urb(rxb->urb->urb);
		ch347_put_rx_buffer(ch347, rxb);
	}
exit:
	return retval;
}

int ch347_xfer(struct platform_device *pdev,
	       const uint8_t *obuf, unsigned obuf_len,
	       uint8_t *ibuf, unsigned ibuf_len)
{
	int retval;
	struct ch347_dev *ch347 = dev_get_drvdata(pdev->dev.parent);

	dev_dbg(&ch347->interface->dev,
		"%s: obuf = %p, obuf_len = %u, ibuf = %p, ibuf_len = %u",
		__func__, obuf, obuf_len, ibuf, ibuf_len);
	retval = ch347_data_xfer(ch347, obuf, obuf_len,
		ibuf, ibuf_len, 0, CH347_DEFAULT_TIMEOUT);

	return retval;
}
EXPORT_SYMBOL(ch347_xfer);

ch347_mode_t ch347_mode(struct platform_device *pdev)
{
	struct ch347_dev *ch347 = dev_get_drvdata(pdev->dev.parent);
	return ch347->mode;
}
EXPORT_SYMBOL(ch347_mode);

/*
 * Sometimes, for some completely unknown and magical reasons,
 * when the driver is loaded for the first time after system
 * power on or reboot, the first read from CH347 returns
 * completely wrong data with wrong size. So this function
 * is called at the first step and it tries to read as much
 * data as possible after performing a dummy GPIO configuration
 * write. In a normal state, the size of the returned data should
 * be 11 bytes. But, experiments have shown that sometimes this
 * is not true and either 2 bytes or, even 64 bytes are returned.
 * Once all available unexpected data is readed, further
 * communication with the CH347 device is fine.
 */
static int ch347_fixup_startup(struct ch347_dev *ch347) {
	int ret;
	uint8_t *ibuf;

	u8 obuf[3 + 8];
	memset(obuf, 0, 11);
	obuf[0] = 0xcc;
	obuf[1] = 8;
	obuf[2] = 0;

	/* Try to read maximum allowed buffer */
	ibuf = kzalloc(CH347_RX_BUFFER_SIZE, GFP_KERNEL);
	if (!ibuf)
		return -ENOMEM;

	ret = ch347_data_xfer(ch347, obuf, 11,
		ibuf, CH347_RX_BUFFER_SIZE,
		RXB_FLAG_NO_RESUBMIT, 250);

	if (ret == 11) {
		/* HW is OK. Readed as much as expected. */
		ret = 0;
	}
	else if (ret >= 0) {
		dev_warn(&ch347->interface->dev, "Unexpectedly readed %u bytes", ret);
		ret = 0;
	}
	else {
		ch347_draw_down(ch347);

		/* Clear errors */
		spin_lock_irq(&ch347->err_lock);
		ch347->errors = 0;
		spin_unlock_irq(&ch347->err_lock);

		/* Try again */
		ret = ch347_data_xfer(ch347, obuf, 11,
			ibuf, 11, 0, CH347_DEFAULT_TIMEOUT);
	}

	kfree(ibuf);
	return ret;
}

static void ch347_draw_down(struct ch347_dev *ch347)
{
	int time;
	int i;

	time = usb_wait_anchor_empty_timeout(&ch347->submitted, CH347_DEFAULT_TIMEOUT);
	if (!time)
		usb_kill_anchored_urbs(&ch347->submitted);

	for (i = 0; i < CH347_RX_BUFFERS; ++i)
		if (ch347->rxb[i].urb)
			usb_kill_urb(ch347->rxb[i].urb->urb);
}

static void ch347_disconnect(struct usb_interface *interface)
{
	struct ch347_dev *ch347  = usb_get_intfdata(interface);

	mutex_lock(&ch347->io_mutex);
	ch347->interface = NULL;
	mutex_unlock(&ch347->io_mutex);

	ch347_draw_down(ch347);
	mfd_remove_devices(&interface->dev);
	ch347_free(ch347);

	dev_info(&interface->dev, "CH347 disconnected");
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

	mutex_init(&ch347->io_mutex);
	sema_init(&ch347->tx_limit_sem, CH347_TX_BUFFERS);
	init_usb_anchor(&ch347->submitted);
	spin_lock_init(&ch347->err_lock);

	usb_set_intfdata(interface, ch347);
	if (usb_id->idProduct == CH347_USB_DEVICE_M3)
		ch347->mode = CH347_MODE_3;
	else
		ch347->mode = CH347_MODE_1;

	ret = ch347_init_buffers(ch347);
	if (ret)
		goto out_free;

	ret = ch347_fixup_startup(ch347);
	if (ret < 0) {
		dev_err(dev, "%s: Can't start driver. Seems like hardware problem: %d",
			__func__, ret);
		goto out_free;
	}

	ret = mfd_add_hotplug_devices(dev, ch347_devs,
		(ch347->mode == CH347_MODE_3) ? 2 : 3);
	if (ret != 0) {
		dev_err(dev, "%s: Failed to add MFD devices to core: %d", __func__, ret);
		goto out_free;
	}

	dev_info(&interface->dev, "CH347 attached");

	return 0;

out_free:
	ch347_free(ch347);
	return ret;
}

static int ch347_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct ch347_dev *ch347 = usb_get_intfdata(intf);

	if (!ch347)
		return 0;
	ch347_draw_down(ch347);
	return 0;
}

static int ch347_resume(struct usb_interface *intf)
{
	return 0;
}

static int ch347_pre_reset(struct usb_interface *intf)
{
	struct ch347_dev *ch347 = usb_get_intfdata(intf);

	mutex_lock(&ch347->io_mutex);
	ch347_draw_down(ch347);

	return 0;
}

static int ch347_post_reset(struct usb_interface *intf)
{
	struct ch347_dev *ch347 = usb_get_intfdata(intf);

	ch347->errors = -EPIPE;
	mutex_unlock(&ch347->io_mutex);
	return 0;
}

static const struct usb_device_id ch347_table[] = {
	{ USB_DEVICE(CH347_USB_VENDOR, CH347_USB_DEVICE_M1) }, /* mode #1 (UART + SPI + I2C + GPIO) */
	{ USB_DEVICE(CH347_USB_VENDOR, CH347_USB_DEVICE_M3) }, /* mode #3 (UART + JTAG + I2C + GPIO) */
	{ }
};

MODULE_DEVICE_TABLE(usb, ch347_table);

static struct usb_driver ch347_driver = {
	.name = "ch347",
	.probe = ch347_probe,
	.disconnect = ch347_disconnect,
	.suspend = ch347_suspend,
	.resume = ch347_resume,
	.pre_reset = ch347_pre_reset,
	.post_reset = ch347_post_reset,
	.id_table = ch347_table,
	.supports_autosuspend = 1,
};

module_usb_driver(ch347_driver);

MODULE_DESCRIPTION("Core driver for QinHeng Electronics CH347 USB adapter");
MODULE_AUTHOR("Alexey Starikovskiy <aystarik@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ch347-mfd");
