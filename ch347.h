/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_USB_CH347_H
#define __LINUX_USB_CH347_H

int ch347_xfer(struct platform_device *pdev, const uint8_t *obuf, unsigned obuf_len,
               uint8_t *ibuf, unsigned ibuf_len);

bool ch347_mode3(struct platform_device *pdev);

#endif
