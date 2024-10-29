/*
 * MATRIX VISION mvHYPERION driver
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: hyperion.h,v 1.51 2009-12-01 15:47:38 ug Exp $
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

#ifndef hyperionH
#define hyperionH hyperionH


#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/completion.h>
#include <linux/sched.h>


#include "matrix_types.h"
#include "HyperionRegister.h"
#include "hyperion_dma_nios.h"
#include "HyperionProp.h"
#include "device_queue.h"
#include "pipe.h"
#include "DMACtrl.h"
#include "HyperionEeprom.h"
#include "uart_read_write.h"
#include "PoCLControl.h"

#define MODULE_NAME "hyperion"
#define HYPERION_MAJ_VERSION    MAJOR_VERSION
#define HYPERION_MIN_VERSION    MINOR_VERSION
#define HYPERION_NUM_VERSION    (HYPERION_MAJ_VERSION*100+HYPERION_MIN_VERSION)

//-------------------------------------------------------------------------------------------
#define kHz (1000)
#define MHz (kHz * kHz)
#define TRANSLATION_TABLE_ELEMENTS 512
#define MAXSG TRANSLATION_TABLE_ELEMENTS
#define MAX_LENGTH (MAXSG*PAGE_SIZE)
#define ONBOARD_MEMORY_LEN 640
#define DMA_RESULT_QUEUE_LEN 2048

//-------------------------------------------------------------------------------------------
#ifdef USE_FIXED_DEVICE
#define HYPERION_MAJOR    60
#else // dynamic allocation of device number
#define HYPERION_MAJOR  0
#endif

//-------------------------------------------------------------------------------------------
#define MAXHYPERIONS        16
#define MAX_CONT_DEVICES    256
#define DMA_RESULT_QUEUE_LEN    2048

//------------------------------------------------------------------------------------------
#define HYPERION_FLAG_NOT_INITIALIZED   0
#define HYPERION_FLAG_INITIALIZED   0x1
#define HYPERION_FLAG_IRQ       0x2
#define HYPERION_FLAG_MSI       0x4
#define HYPERION_FLAG_MSIX      0x8

#ifndef DMA_BIT_MASK
#define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))
#endif

struct mux_controller_sequence
{
    u_char changed;
    u32 size;
    u32 muxdata[MUX_CONTROLLER_SEQUENCE_SIZE_BYTES];
};

//========== memory spaces ===========================================================
typedef struct
{
    unsigned long           addr;
    unsigned long           size;
    TKernelVirtualAddress   kptr;
    TUserVirtualAddress     descr;    // used only in titan user space
} TSpace;

//-------------------------------------------------------------------------------------------
struct dma_transfer_object
//-------------------------------------------------------------------------------------------
{
    unsigned int init_done;
    DMA_CONTROLLER* dma_controller_pci;     ///< the pci dma controller using a translationtable --> AvalonToPCITranslationTable
    DMA_CONTROLLER* dma_controller_videoin[cfdCommandFifoMaxList];      ///< this dma controller transfers data from video in to internal ram

    //DMA Transfer hyperion to pci
    long frame_counter;                     ///< framecounter will be reset only at startdevice
    void* presult_proplist;                 ///< pointer to the result property list = image_footer
    long requestID;                         ///< actual requestID, an important result property
    struct device_queue* rw_queue;          ///< pointer to the read/write queue defined in the device extension

    wait_queue_head_t* presult_queue;       ///< pointer to our device result queue for handling parallel transfer results
    u_long objid;
    struct memory_space avalon_to_pci_table;
    unsigned char muxdata_reset_required;
    struct mux_controller_sequence mux_seq;
    volatile struct io_object* active_ioobj;
    unsigned char rewrite_transfer_parameter;
    volatile unsigned char abort_all_transfer;
    volatile unsigned char abort_this_request;
    volatile unsigned char timeout_occurred;
    volatile unsigned char restart_current_ioobj;
    struct timer_list abort_timer;
};

//-------------------------------------------------------------------------------------------
struct user_buffer_list
//-------------------------------------------------------------------------------------------
{
    char __user* buf;
    size_t count;
    struct scatterlist* sg;
    int nr_pages;
    struct list_head list;
    void* private;
};

//-------------------------------------------------------------------------------------------
struct hyperion_device
//-------------------------------------------------------------------------------------------
{
    int                 index;
    unsigned            flags;
    unsigned            bus;
    unsigned            devfunc;
    u_char              irqlin;
    unsigned            irqcnt;
    spinlock_t          s_lock;
    struct cdev         cdev;       ///< Char device structure
    struct cdev         cdev_serial[MAX_PARALLEL_TRANSFER]; ///< Char device structure
    struct pci_dev*          pdev;
    //struct hyperion_device_extension device_extension;
    struct memory_space memory_base[6];
    struct tasklet_struct hyperion_tasklet;


    //this struct will be initialized during hyperion_open and removed with hyperion_close
    //dma_ctrl's
    //queues etc..
    //access to uart's etc
    struct device_queue*         pqueues[MAX_PARALLEL_TRANSFER]; ///< pointer to the queue objects
    struct device_queue         dq_read_write[MAX_PARALLEL_TRANSFER]; ///< queues for reads and writes for the DMA_Channels
    struct device_queue*         pqueue_result; ///< pointer to the result_queue
    struct device_queue         dq_result; ///< queue including result ioobj
    struct dma_transfer_object*      pdma_object[MAX_PARALLEL_TRANSFER]; ///< pointer to the dma objects
    struct dma_transfer_object      dma_transfer_object[MAX_PARALLEL_TRANSFER]; ///< the dmaobjects contains all structures and variables for transfering
    struct memory_space         hyperion_base;
    HYPERION_BASE_REGISTER_DEF*  reg_def;
    TPipe*                       dma_interrupt_result; ///< resultqueue to save interruptstatus for handling in our DPC
    TPipe*                       cleanup_request_pipe;
    spinlock_t                  lock_ires;
    wait_queue_head_t           result_queue;       ///< result queue for capturing ResultPacket
    struct _pci_cfg_reg
    {
        u32 command;
        u32 latency;
        u32 baseaddr;
        u32 interrupt;
    } pci_cfg_reg;
    struct mux_controller_sequence mux_seq;
    uart_object_t               uart_port[UART_NUM];
    struct completion compl_abort;
    u32 hrtc_version;
    POCL_OBJECT pocl[MAX_PARALLEL_TRANSFER];
    void ( *set_mux_data )( struct hyperion_device* device, struct mux_controller_sequence* mux_seq, u_long inputchannel );
    void ( *set_hrtc_ram )( struct hyperion_device* device, struct transfer_parameter* tp );
    struct _DeviceInfo
    {
        unsigned short vendorId, deviceId;
    } vd_id;
    struct msix_entry msi_x_entry[HYPERION_MSI_X_MAX_VECTORS];
    struct list_head head_uaddr;
    int pci_hyperion_page_size;
    int address_space_encoding;
    int eeprom_write_access;
};

struct hyperion_device* get_hyperion_device( unsigned int index );
void write_results_to_rq( volatile struct io_object* ioobj, u64 j64, unsigned long status );
void complete_request( volatile struct io_object* ioobj, unsigned char status );

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
void jit_timer_cancel_request( struct timer_list* arg );
#else
void jit_timer_cancel_request( unsigned long arg );
#endif
void add_to_cleanup_pipe( struct hyperion_device* device, struct page* page, void* vmem );
#define _GET_ACTIVE_IOOBJ(dto) dto->active_ioobj ? dto->active_ioobj : get_current_iocb(dto->rw_queue)
void free_ubuf_obj( struct hyperion_device* device, char __user* buffer, size_t count );
void free_all_ubuf( struct hyperion_device* device );

#define IO_READ_8(MEMBASE,REGDEF,REGID,OFFSET) ioread8( (void*)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define IO_WRITE_8(MEMBASE,REGDEF,REGID,OFFSET,DATA) iowrite8(DATA, (void*)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define IO_READ_32(MEMBASE,REGDEF,REGID,OFFSET) ioread32( (void __iomem *)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define IO_READ_32_PRINT(MEMBASE,REGDEF,REGID,OFFSET)\
    printk("ioread32(base %p, regid %d, off %x) regdef.off %x regdef.off_mul %d *%p = %x\n", (void*)MEMBASE.base, REGID, OFFSET, REGDEF[REGID].offset, REGDEF[REGID].off_mul, (void __iomem *)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)), ioread32((void __iomem *)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET))));
#define IO_WRITE_32(MEMBASE,REGDEF,REGID,OFFSET,DATA) iowrite32(DATA, (void __iomem *)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define REG_POINTER(MEMBASE,REGDEF,REGID,OFFSET) (MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET))

#endif //#define hyperionH hyperionH
