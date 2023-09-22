PWD         := $(shell pwd)
KVERSION    := $(shell uname -r)
KERNEL_SRC  ?= /lib/modules/$(KVERSION)/build

obj-m += mfd-ch347.o
obj-m += i2c-ch347.o
obj-m += gpio-ch347.o
obj-m += spi-ch347.o

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install
