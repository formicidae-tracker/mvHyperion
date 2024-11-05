MODULE_VERSION ?= 2.47.2
MODULE_VERSION_MAJOR ?= 2
MODULE_VERSION_MINOR ?= 47

COMMON_PATH=../common
COMMON_SYSTEM_PATH=../../common

KBUILD_EXTRA_SYMBOLS=$(M)/../HyperionGeneric/Module.symvers


ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m  := hyperion2.o
hyperion2-y := drivermain.o \
	$(COMMON_PATH)/matrix_tools.o \
	$(COMMON_PATH)/device_queue.o \
	$(COMMON_SYSTEM_PATH)/i2c_access.o \
	$(COMMON_SYSTEM_PATH)/ksemaphore.o \
	$(COMMON_SYSTEM_PATH)/spi_access.o \
	$(COMMON_SYSTEM_PATH)/Hyperion/PoCLControl.o \
	$(COMMON_SYSTEM_PATH)/Hyperion/external_boot.o \
	$(COMMON_SYSTEM_PATH)/Hyperion/uart_read_write.o \
	iocontrol.o \
	user_buffer.o \
	read_write.o \
	hyperion_base.o \
	dma_sg_list_buffer.o \
	clf_func.o \
	utils.o


ccflags-y	:= -DMAJOR_VERSION=$(MODULE_VERSION_MAJOR) \
	-DMINOR_VERSION=$(MODULE_VERSION_MINOR) \
	-DDEBUG=1 \
	-I$(src)/../common \
	-I$(src)/../../common \
	-I$(src)/../../common/Hyperion \
	-I$(src) \
	-I$(src)/../Hyperion \
	-DUSE_NEW_REMAP_PAGE_RANGE=1 \
	-DUSE_ACCESS_OK \
	-DHAVE_MSECS_TO_JIFFIES_EXTRA=1 \
	-DNIOS_SUPPORTED_TRANSFER=0 \
	-DREMOVE_REQUEST_BUFFER_MAPPING \
	-DHAVE_UNLOCKED_IOCTL=1

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
