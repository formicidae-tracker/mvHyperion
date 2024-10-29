MODULE_VERSION ?= 2.47.2
MODULE_VERSION_MAJOR ?= 2
MODULE_VERSION_MINOR ?= 47


ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m  := hyperion_generic.o
hyperion_generic-y := hyperion_generic_main.o

ccflags-y	:= -DMAJOR_VERSION=$(MODULE_VERSION_MAJOR) \
	-DMINOR_VERSION=$(MODULE_VERSION_MINOR) \
	-DDEBUG=1 \
	-I$(src)/../common \
	-DUSE_NEW_REMAP_PAGE_RANGE=1 \
	-DUSE_ACCESS_OK \
	-DHAVE_MSECS_TO_JIFFIES_EXTRA=1 \
	-DNIOS_SUPPORTED_TRANSFER=1

else
# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install
	depmod -A


clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif
