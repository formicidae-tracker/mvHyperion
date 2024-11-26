MODULE_VERSION ?= 2.44.0
MODULE_VERSION_MAJOR ?= 2
MODULE_VERSION_MINOR ?= 44


KBUILD_EXTRA_SYMBOLS=$(M)/../HyperionGeneric/Module.symvers

ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
hyperion2-y  := drivermain.o \
	../common/matrix_tools.o \
	../common/device_queue.o \
	../../common/i2c_access.o \
	../../common/ksemaphore.o \
	../../common/spi_access.o \
	../../common/Hyperion/PoCLControl.o \
	../../common/Hyperion/external_boot.o \
	../../common/Hyperion/uart_read_write.o \
	iocontrol.o \
	user_buffer.o \
	read_write.o \
	hyperion_base.o \
	dma_sg_list_buffer.o \
	clf_func.o

obj-m := hyperion2.o

ccflags-y	:= -DMAJOR_VERSION=$(MODULE_VERSION_MAJOR) \
	-DMINOR_VERSION=$(MODULE_VERSION_MINOR) \
	-DDEBUG=1 \
	-I$(M)/../common \
	-I$(M)/../../common \
	-I$(M)/../../common/Hyperion \
	-I$(M)/../Hyperion \
	-I$(M) \
	-I$(M)/../../../ \
	-DUSE_NEW_REMAP_PAGE_RANGE=1 \
	-DUSE_ACCESS_OK \
	-DHAVE_MSECS_TO_JIFFIES_EXTRA=1 \
	-DNIOS_SUPPORTED_TRANSFER=1 \
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
