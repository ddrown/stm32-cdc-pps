ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m  := cdc-acm.o

else
# normal makefile
KDIR = /lib/modules/`uname -r`/build

.PHONY: default clean

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

endif
