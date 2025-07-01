obj-m += imx678.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	make -C $(KDIR) M=$(shell pwd) modules

clean:
	make -C $(KDIR)  M=$(shell pwd) clean

%.dtbo: %.dts
	dtc -@ -I dts -O dtb -o $@ $<

