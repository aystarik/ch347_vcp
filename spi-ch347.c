// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core driver for QinHeng Electronics CH347 USB-SPI adapter
 *
 * Copyright (c) 2023 Alexey Starikovskiy <aystarik@gmail.com>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>

#include "ch347.h"

const unsigned MAX_SPI_SPEED = 60*1000*1000; //HZ
const unsigned MIN_SPI_SPEED = MAX_SPI_SPEED >> 7;

enum CH347_SPI_DIR {
	SPI_DIR_2LINES_FULLDUPLEX = 0, // default
	SPI_DIR_2LINES_RX = 0x400,
	SPI_DIR_1LINE_RX = 0x8000,
	SPI_DIR_1LINE_TX = 0xc000
};

enum CH347_SPI_MODE {
	SPI_MODE_MASTER = 0x104, // default
	SPI_MODE_SLAVE = 0,
};

enum CH347_SPI_NSS {
	SPI_NSS_SOFTWARE = 0x200, // default
	SPI_NSS_HARDWARE = 0,
};

/* this structure is going to device, so don't change anything */
struct ch347_hw_config {
	uint16_t direction;
	uint16_t mode;
	uint16_t bpw;
	uint16_t polarity;
	uint16_t phase;
	uint16_t nss; /* hardware or software managed CS */
	uint16_t baud_prescaler; /* prescaler = x * 8. x: 0=60MHz, 1=30MHz, 2=15MHz, 3=7.5MHz, 4=3.75MHz, 5=1.875MHz, 6=937.5KHz，7=468.75KHz */
	uint16_t first_bit; /* MSB or LSB first */
	uint16_t crc_polynomial; /* polynomial used for the CRC calculation. */
	uint16_t write_read_interval; /* uS */
	uint8_t out_default_data;     /* Data to output on MOSI during SPI reading */
/*
 * Bit 7: CS0 polarity
 * Bit 6: CS1 polarity
 */
	uint8_t cs_config;
	uint8_t reserved[4];
};


struct ch347_spi {
	struct platform_device *pdev;
	struct spi_master *master;

	struct spi_device *slaves[2];

	struct ch347_hw_config cfg;

	u8 ibuf[512];
	u8 obuf[512];

	u32 speed;
	u8 cs;
	u8 mode;
};

static int get_hw_speed(uint16_t prescaler) {
	return MAX_SPI_SPEED >> prescaler;
}

static void set_hw_speed(struct ch347_spi *ch347, unsigned speed) {
	unsigned freq = MAX_SPI_SPEED;
	unsigned i;
	for (i = 0; i < 7; ++i) {
		if (freq <= speed) break;
		freq >>= 1;
	}
	ch347->cfg.baud_prescaler = i << 3;
	ch347->speed = freq;
}

static int ch347_get_hw_config(struct ch347_spi *ch347) {
	int rv;
	ch347->obuf[0] = 0xca;
	ch347->obuf[1] = 1;
	ch347->obuf[2] = 0;
	ch347->obuf[3] = 1;
	rv = ch347_xfer(ch347->pdev, ch347->obuf, 4, ch347->ibuf, sizeof(struct ch347_hw_config) + 3);
	if (rv < 0)
		return rv;
	memcpy(&ch347->cfg, ch347->ibuf + 3, sizeof(struct ch347_hw_config));
	ch347->speed = get_hw_speed(ch347->cfg.baud_prescaler >> 3);
	ch347->mode = ch347->cfg.phase | ch347->cfg.polarity;
	ch347->mode |= (ch347->cfg.first_bit & 0x80) ? SPI_LSB_FIRST : 0;
	return 0;
}

static int ch347_set_cs(struct ch347_spi *ch347, int cs, bool val) {
	u8 *ptr = ch347->obuf;
	memset(ptr, 0, 16);
	*ptr++ = 0xc1;
	*ptr++ = 10;
	*ptr++ = 0;
	ptr[cs ? 5 : 0] = (val) ? 0x80 : 0xc0;
	return ch347_xfer(ch347->pdev, ch347->obuf, 13, NULL, 0);
}

static int ch347_prepare_message(struct spi_master *master,
				 struct spi_message *message) {
	int rv;
	struct ch347_spi *ch347 = spi_master_get_devdata(master);
	struct spi_device *spi = message->spi;
	if (ch347->cs != spi->chip_select) {
		rv = ch347_set_cs(ch347, spi->chip_select, (spi->mode & SPI_CS_HIGH) ? false : true);
		if (rv < 0)
			return rv;
		ch347->cs = spi->chip_select;
	}
	return 0;
}

static int ch347_transfer_setup(struct ch347_spi *ch347, u32 speed, u8 mode) {
	u8 *ptr = ch347->obuf;
	int rv;

	set_hw_speed(ch347, speed);
	ch347->mode = mode;
	ch347->cfg.phase = mode & SPI_CPHA;
	ch347->cfg.polarity = mode & SPI_CPOL;
	ch347->cfg.first_bit = (mode & SPI_LSB_FIRST) ? 0x80 : 0;

	*ptr++ = 0xc0;
	*ptr++ = sizeof(ch347->cfg);
	*ptr++ = 0;
	memcpy(ptr, &ch347->cfg, sizeof(ch347->cfg));
	rv = ch347_xfer(ch347->pdev, ch347->obuf, 3 + sizeof(ch347->cfg), ch347->ibuf, 4);
	if (rv < 0) {
		dev_err(&ch347->pdev->dev, "ch347_xfer failed: %d", rv);
		return rv;
	}
	if (ch347->ibuf[0] != 0xc0 || ch347->ibuf[3]) {
		dev_err(&ch347->pdev->dev, "ibuf[0]=%x, ibuf[3] = %d", ch347->ibuf[0], ch347->ibuf[3]);
		return -1;
	}
	return 0;
}

static int ch347_spi_write(struct ch347_spi *ch347, const u8 *tx_data, u16 data_len)
{
	int rv;
	unsigned len, remaining = data_len, offset;
	ch347->obuf[0] = 0xc4;
	do {
		len = (remaining > sizeof(ch347->obuf) - 3) ? sizeof(ch347->obuf) - 3 : remaining;
		offset = data_len - remaining;
		ch347->obuf[1] = len;
		ch347->obuf[2] = len >> 8;
		memcpy(ch347->obuf + 3, tx_data + offset, len);
		rv = ch347_xfer(ch347->pdev, ch347->obuf, len + 3, NULL, 0);
		if (rv < 0)
			return rv;
		rv = ch347_xfer(ch347->pdev, NULL, 0, ch347->ibuf, 4);
		if (rv < 0)
			return rv;
		remaining -= len;
	} while (remaining);
	return 0;

}

static int ch347_spi_read(struct ch347_spi *ch347, u8 *rx_data, u32 data_len)
{
	int rv;
	unsigned len, remaining = data_len, offset;
	ch347->obuf[0] = 0xc3;
	ch347->obuf[1] = 4;
	ch347->obuf[2] = 0;
	memcpy(ch347->obuf + 3, &data_len, 4);
	rv = ch347_xfer(ch347->pdev, ch347->obuf, 7, NULL, 0);
	if (rv < 0)
		return rv;
	do {
		len = (remaining > sizeof(ch347->ibuf) - 3) ? sizeof(ch347->ibuf) - 3 : remaining;
		offset = data_len - remaining;
		rv = ch347_xfer(ch347->pdev, NULL, 0, ch347->ibuf, len + 3);
		if (rv < 0)
			return rv;
		if (rv != len + 3)
			return -EINVAL;
		memcpy(rx_data + offset, ch347->ibuf + 3, len);
		remaining -= len;
	} while (remaining);
	return 0;
}

static int ch347_rdwr(struct ch347_spi *ch347, const u8 *tx_data, u8 *rx_data, u32 data_len)
{
	unsigned remaining = data_len;
	unsigned offset, len;
	int rv;
	if (!tx_data) {
		//dev_info(&ch347->pdev->dev, "ch347_rdwr !tx_data");
		return ch347_spi_read(ch347, rx_data, data_len);
	}
	if (!rx_data) {
		//dev_info(&ch347->pdev->dev, "ch347_rdwr !rx_data");
		return ch347_spi_write(ch347, tx_data, data_len);
	}
	do {
		len = (remaining > sizeof(ch347->obuf) - 3) ? sizeof(ch347->obuf) - 3 : remaining;
		offset = data_len - remaining;
		ch347->obuf[0] = 0xc2;
		ch347->obuf[1] = len;
		ch347->obuf[2] = len >> 8;
		memcpy(ch347->obuf + 3, tx_data + offset, len);
		rv = ch347_xfer(ch347->pdev, ch347->obuf, len + 3, ch347->ibuf, len + 3);
		if (rv < 0)
			return rv;
		if (rv != len + 3)
			return -EINVAL;
		memcpy(rx_data + offset, ch347->ibuf + 3, len);
		remaining -= len;
	} while (remaining);
	return 0;
}

static int ch347_transfer_one(struct spi_master *master,
			      struct spi_device *spi,
			      struct spi_transfer *xfer)
{
	int rv;
	struct ch347_spi *ch347 = spi_master_get_devdata(master);
	//dev_info(&ch347->pdev->dev, "ch347_transfer_one start");
	if (xfer->speed_hz != ch347->speed || (spi->mode & (SPI_CPHA|SPI_CPOL|SPI_LSB_FIRST)) != ch347->mode) {
		rv = ch347_transfer_setup(ch347, xfer->speed_hz, spi->mode);
		if (rv < 0) {
			dev_err(&ch347->pdev->dev, "can not setup transfer");
			return rv;
		}
		// settings changed, need to reassert chip_select
		ch347_set_cs(ch347, spi->chip_select, (spi->mode & SPI_CS_HIGH) ? false : true);
	}
	rv = ch347_rdwr(ch347, xfer->tx_buf, xfer->rx_buf, xfer->len);
	if (rv < 0) {
		dev_err(&ch347->pdev->dev, "write/read failed");
		return rv;
	}
	if (xfer->cs_change || spi_transfer_is_last(master, xfer)) {
		ch347_set_cs(ch347, spi->chip_select, (spi->mode & SPI_CS_HIGH) ? true : false);
		ch347->cs = 0xff;
	}
	//dev_info(&ch347->pdev->dev, "ch347_transfer_one end");
	return 0;
}

static int ch347_transfer_one_message(struct spi_master *master, struct spi_message *m)
{
	struct ch347_spi *ch347 = spi_master_get_devdata(master);
	struct spi_device *spi = m->spi;
	struct spi_transfer *xfer = list_first_entry(&m->transfers, struct spi_transfer, transfer_list);
	int rv;

	if (xfer->speed_hz != ch347->speed || (spi->mode & (SPI_CPHA|SPI_CPOL|SPI_LSB_FIRST)) != ch347->mode) {
		rv = ch347_transfer_setup(ch347, xfer->speed_hz, spi->mode);
		if (rv < 0) {
			dev_err(&ch347->pdev->dev, "can not setup transfer");
			return rv;
		}
	}
	if ((spi->mode & SPI_NO_CS) == 0) {
		ch347_set_cs(ch347, spi->chip_select, (spi->mode & SPI_CS_HIGH) ? false : true);
	}

	m->status = 0;
	m->actual_length = 0;

	list_for_each_entry(xfer, &m->transfers, transfer_list) {
		rv = ch347_rdwr(ch347, xfer->tx_buf, xfer->rx_buf, xfer->len);
		if (rv < 0) {
			dev_err(&ch347->pdev->dev, "write/read failed");
			m->status = rv;
			return rv;
		}
		m->actual_length += xfer->len;
	}

	if ((spi->mode & SPI_NO_CS) == 0) {
		ch347_set_cs(ch347, spi->chip_select, (spi->mode & SPI_CS_HIGH) ? true : false);
	}

	spi_finalize_current_message(master);

	return 0;
}

static int add_slave(struct ch347_spi *ch347, struct spi_board_info *board_info)
{
	unsigned int cs = board_info->chip_select;

	/* Sanity check */
	if (cs >= ch347->master->num_chipselect)
		return -EINVAL;

	board_info->bus_num = ch347->master->bus_num;

	if (ch347->slaves[cs] != NULL) {
		return -EADDRINUSE;
	}

	ch347->slaves[cs] = spi_new_device(ch347->master, board_info);
	if (!ch347->slaves[cs]) {
		return -ENOMEM;
	}
	return 0;
}

static int remove_slave(struct ch347_spi *ch347, unsigned int cs)
{
	if (cs >= ch347->master->num_chipselect)
		return -EINVAL;

	if (ch347->slaves[cs] == NULL)
		return -ENODEV;
	spi_unregister_device(ch347->slaves[cs]);
	ch347->slaves[cs] = NULL;
	return 0;
}

static ssize_t new_device_store(struct device *mdev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned speed = MAX_SPI_SPEED / 1000; // measure speed in kHz 
	struct ch347_spi *ch347 = dev_get_drvdata(mdev);
	struct spi_board_info board_info = {
	    .mode = SPI_MODE_0,
	    .max_speed_hz = MAX_SPI_SPEED,
	};
	char *req_org = NULL, *req = NULL, *str = NULL;
	int rc;

	req_org = kstrdup(buf, GFP_KERNEL);
	if (!req_org)
		return -ENOMEM;

	req = req_org;

	str = strsep(&req, " ");
	if (str == NULL) {
		rc = -EINVAL;
		goto free_req;
	}

	rc = strscpy(board_info.modalias, str, sizeof(board_info.modalias));
	if (rc < 0)
		goto free_req;

	str = strsep(&req, " ");
	if (str == NULL) {
		rc = -EINVAL;
		goto free_req;
	}
	rc = kstrtou16(str, 0, &board_info.chip_select);
	if (rc)
		goto free_req;

	str = strsep(&req, " ");
	if (str != NULL) {
		if (kstrtou32(str, 0, &speed))
			speed = MAX_SPI_SPEED / 1000;
	}

	board_info.max_speed_hz = speed * 1000;

	rc = add_slave(ch347, &board_info);
	if (rc)
		goto free_req;

	kfree(req_org);

	return count;

free_req:
	kfree(req_org);

	return rc;
}

static DEVICE_ATTR_WO(new_device);

static ssize_t delete_device_store(struct device *mdev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct ch347_spi *ch347 = dev_get_drvdata(mdev);
	unsigned cs;
	int rc;

	rc = kstrtouint(buf, 0, &cs);
	if (rc)
		return rc;

	rc = remove_slave(ch347, cs);
	if (rc)
		return rc;

	return count;
}

static DEVICE_ATTR_WO(delete_device);

static int ch347_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct ch347_spi *ch347;
	struct device *dev = &pdev->dev;
	int rv;

	master = spi_alloc_master(dev, sizeof(struct ch347_spi));
	if (!master)
		return -ENOMEM;
	platform_set_drvdata(pdev, master);
	ch347 = spi_master_get_devdata(master);

	ch347->master = master;
	ch347->master->dev.of_node = dev->of_node;
	ch347->pdev = pdev;

	ch347->cs = 0xff;
	ch347->mode = 0xff;
	ch347->speed = 0;

	rv = ch347_get_hw_config(ch347);
	if (rv < 0)
		return rv;

	master->num_chipselect = 2;
	master->min_speed_hz = MIN_SPI_SPEED;
	master->max_speed_hz = MAX_SPI_SPEED;  
	master->bits_per_word_mask = 0;

	master->bus_num = -1;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST; // SPI_CS_HIGH -- possible

	master->prepare_message = ch347_prepare_message;
	master->transfer_one = ch347_transfer_one;
	master->transfer_one_message = ch347_transfer_one_message;

	rv = devm_spi_register_master(dev, master);
	if (rv < 0)
		return rv;
	rv = device_create_file(&master->dev, &dev_attr_new_device);
	if (rv) {
		dev_err(dev, "can not create new_device file");
		return rv;
	}
	rv = device_create_file(&master->dev, &dev_attr_delete_device);
	if (rv) {
		dev_err(dev, "can not create delete_device file");
		return rv;
	}

	return 0;
}

static int ch347_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	device_remove_file(&master->dev, &dev_attr_new_device);
	device_remove_file(&master->dev, &dev_attr_delete_device);
	return 0;
}

static struct platform_driver ch347_spi_driver = {
	.driver.name	= "ch347-spi",
	.probe		= ch347_spi_probe,
	.remove	= ch347_spi_remove,
};

module_platform_driver(ch347_spi_driver);

MODULE_DESCRIPTION("Driver for the QinHeng Electronics CH347 SPI master interface");
MODULE_AUTHOR("Alexey Starikovskiy <aystarik@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ch347-spi");
