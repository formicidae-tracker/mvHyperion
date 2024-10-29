MODULE_VERSION ?= 2.47.2
MODULE_VERSION_MAJOR ?= 2
MODULE_VERSION_MINOR ?= 47

COMMON_PATH=$(src)/../common
COMMON_SYSTEM_PATH=$(src)/../../common

ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m  := hyperion.o
hyperion-y := hyperionmain.o \
	$(COMMON_PATH)/matrix_tools.o \
	read_write.o \
	device_func.o \
	device_queue.o \
	uart_read_write.o \
	$(COMMON_SYSTEM_PATH)/DMACtrl.o \
	$(COMMON_SYSTEM_PATH)/i2c_access.o \
	$(COMMON_SYSTEM_PATH)/Hyperion/DigitalIO.o\
	$(COMMON_SYSTEM_PATH)/Hyperion/PoCLControl.o


ccflags-y	:= -DMAJOR_VERSION=$(MODULE_VERSION_MAJOR) \
	-DMINOR_VERSION=$(MODULE_VERSION_MINOR) \
	-DDEBUG=1 \
	-I$(src)/../common \
	-include $(src)/../common/mv_kconfig.h \
	-DUSE_NEW_REMAP_PAGE_RANGE=1 \
	-DUSE_ACCESS_OK \
	-DHAVE_MSECS_TO_JIFFIES_EXTRA=1 \
	-DNIOS_SUPPORTED_TRANSFER=0 \
	-DREMOVE_REQUEST_BUFFER_MAPPING

else
# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif
