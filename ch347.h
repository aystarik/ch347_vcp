/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_USB_CH347_H
#define __LINUX_USB_CH347_H

typedef enum {
	CH347_MODE_1 = 1,
	CH347_MODE_3 = 3
} ch347_mode_t;

int ch347_xfer(struct platform_device *pdev, const uint8_t *obuf, unsigned obuf_len,
               uint8_t *ibuf, unsigned ibuf_len);

ch347_mode_t ch347_mode(struct platform_device *pdev);

#endif
