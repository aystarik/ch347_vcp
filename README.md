# ch347_vcp
Linux kernel drivers for vendor class protocols of CH347

Set has four drivers -- base is MFD (mfd-ch347.ko), and on top of it are I2C, GPIO and SPI drivers (Modes 1 and 3).

CH347 can also do UART(s), which are handled independently (in parallel to _this_ driver) by either standard USB CDC driver or
by common W-CH driver for all their UARTs.

JTAG support in handled in userspace via openFPGALoader (open source) or binary driver for OpenOCD released by W-CH.

## I2C (i2c-ch347.ko)
Driver behaves as an ordinary I2C master controller, e.g. _i2c-tools_ work with it and it
is possible to add slave devices via usual methods.

## GPIO (gpio-ch347.ko)
Driver controls 8 pins, there is no check if these pins are used by any other protocol

GPIO | PIN | UART0/1 | SPI | JTAG | I2C
-|-|-|-|-|-
0 | 6 | CTS0 | SCK | TCK
1  | 7 | RTS0|MISO | TDO
2 | 5 | DSR0 | SCS0 | TMS
3 | 11 |  RI0 |||SCL
4 | 15 | DCD0/ACT |||
5 | 9 | TNOW0/DTR0|SCS1|TRST
6 |2| CTS1
7 | 13 |RTS1

## SPI (spi-ch347.ko)
Driver adds new SPI master controller with speeds up to 60MHz and 2 slaves. If your setup does not support such frequencies (think "dupont" wires),
decrease frequency when adding slave devices.

To add a slave device you should send string containing device driver name, chip select number and optionally frequency into "new_device" file in sysfs directory of the driver.
```
cat "spi-nor 0 15000" > /sys/class/.../spi2/new_device
```
