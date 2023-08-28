PWD         := $(shell pwd)
KVERSION    := $(shell uname -r)
KDIR        ?= /lib/modules/$(KVERSION)/build

obj-m += mfd-ch347.o
obj-m += i2c-ch347.o
obj-m += gpio-ch347.o
obj-m += spi-ch347.o

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean
