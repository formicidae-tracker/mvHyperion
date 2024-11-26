/*
 * MATRIX VISION mvHYPERION driver
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: drivermain.h,v 1.10 2010-06-23 14:52:23 ug Exp $
 *
 */
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef DriverMainH
#define DriverMainH DriverMainH

#include <linux/version.h>

#ifndef KERNEL_VERSION
#   define KERNEL_VERSION(a,b,c) (((a) << 16) | ((b) << 8) | (c))
#endif

// Use versioning if needed
#if defined(CONFIG_MODVERSIONS) && ! defined(MODVERSIONS)
#   define MODVERSIONS
#endif

#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/highmem.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 )
//#   include <linux/dma-mapping.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
#   include <linux/dma-attrs.h>
#endif

#include <asm/uaccess.h>
#include <asm/dma.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION( 3, 4, 0 )
#   include <asm/system.h>
#endif
#include <asm/cacheflush.h>
#ifdef CONFIG_PPC_OF
#   include <asm/prom.h>
#endif

#include "matrix_types.h"
#include "read_write.h"
#include "HyperionRegister.h"
#include "HyperionEeprom.h"
#include "HyperionConst.h"

//-------------------------------------------------------------------------------------------
#define kHz (1000)
#define MHz (kHz * kHz)
//-------------------------------------------------------------------------------------------
#ifdef USE_FIXED_DEVICE
#   define HYPERION_MAJOR  60
#else // dynamic allocation of device number
#   define HYPERION_MAJOR  0
#endif

#define MODULE_NAME "hyperion2_"

//-------------------------------------------------------------------------------------------
#define MAXHYPERIONS        16
#define MAX_CONT_DEVICES    256

//------------------------------------------------------------------------------------------
#define HYPERION_FLAG_NOT_INITIALIZED   0
#define HYPERION_FLAG_INITIALIZED   0x1
#define HYPERION_FLAG_IRQ       0x2
#define HYPERION_FLAG_MSI       0x4
#define HYPERION_FLAG_MSIX      0x8


//========== memory spaces ===========================================================
typedef struct
{
    unsigned long           addr;
    unsigned long           size;
    TKernelVirtualAddress   kptr;
    TUserVirtualAddress     descr;    // used only in titan user space
} TSpace;

#define _Ctrl(c)    (1<<c##_CTRL_BIT)
#define POCL_CTRL_BIT   0

//-------------------------------------------------------------------------------------------
struct hyperion
//-------------------------------------------------------------------------------------------
{
    int number;
    unsigned flags;
    unsigned bus;
    unsigned devfunc;
    u_char irqlin;
    unsigned irqcnt;
    spinlock_t s_lock;
    struct cdev cdev;       ///< Char device structure
    struct cdev cdev_serial[MAX_PARALLEL_TRANSFER]; ///< Char device structure
    struct pci_dev* pdev;
    //struct hyperion_device_extension device_extension;
    struct memory_space memory_base[6];
    struct msix_entry msi_x_entry[HYPERION_MSI_X_MAX_VECTORS];
    struct tasklet_struct hyperion_tasklet;
    //this struct will be initialized during hyperion_open and removed with hyperion_close
    struct _pci_cfg_reg
    {
        u32 command;
        u32 latency;
        u32 baseaddr;
        u32 interrupt;
    } pci_cfg_reg;
    struct _DeviceInfo
    {
        unsigned short vendorId, deviceId;
    } vd_id;
    void* device;
    struct list_head head_uaddr;
    void* owner;
    int users;
    struct _HYPERION_EEPROM_CONTENT eeprom_content;
    int firmware_version;
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 )
    //nothing to do
#elif LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    struct dma_attrs attrs;
#endif
    void* pdma_attrs;
};

struct hyperion* get_hyperion( unsigned int index );

#endif //#define DriverMainH
