/*
 * read_write.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: read_write.c,v 1.115 2010-12-14 16:08:44 ug Exp $
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

#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/list.h>
#include <linux/aio.h>
#include <linux/pagemap.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/version.h>
#ifndef KERNEL_VERSION
#   define KERNEL_VERSION(a,b,c) (((a) << 16) | ((b) << 8) | (c))
#endif
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/dma.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION( 3, 4, 0 )
#   include <asm/system.h>
#endif
#include <asm/cacheflush.h>
#include "matrix_types.h"
#include "matrix_tools.h"
#include "hyperion.h"
#include "pipe.h"
#include "property.h"
#include "device_func.h"
#include "hyperion_defs.h"
#include "read_write.h"

static int sgl_unmap_user_pages( struct scatterlist* sgl, const unsigned int nr_pages, int dirtied );
void remove_all_requests( struct hyperion_device* device, struct dma_transfer_object* dto, TItem dma_res );
struct user_buffer_list* get_user_buffer_obj( struct hyperion_device* device, char __user* buf, size_t count );
int check_transfer_state( struct hyperion_device* device, struct dma_transfer_object* dto, volatile struct io_object* ioobj );
#define ENABLE_POCL 0
#define ENABLE_OVERLAPPED_TRANSFER 1

#define USE_KMAP_FOR_RESULT 1
#define SET_BIT(preg,bit,enable)\
    {\
        u32 reg_value = ioread32( (void __iomem *)preg );\
        if( enable )\
            reg_value |= bit;\
        else\
            reg_value &= ~bit;\
        iowrite32( reg_value, (void __iomem *)preg );\
    }

//-------------------------------------------------------------------------------------------
typedef struct _TRANSLATION_TABLE_DEF
//-------------------------------------------------------------------------------------------
{
    int address;
    int size;
} TRANSLATION_TABLE_DEF_T;

//-------------------------------------------------------------------------------------------
static TRANSLATION_TABLE_DEF_T translation_table_defs[MAX_PARALLEL_TRANSFER][4] =
//-------------------------------------------------------------------------------------------
{
//channel 0
    {{AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, ( TRANSLATION_TABLE_SIZE / MAX_PARALLEL_TRANSFER )}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE}, {0, 0}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE}},
//channel1
    {{( AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE + ( TRANSLATION_TABLE_SIZE / MAX_PARALLEL_TRANSFER ) ), ( TRANSLATION_TABLE_SIZE / MAX_PARALLEL_TRANSFER )}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE}, {0, 0}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE}}
};

//-------------------------------------------------------------------------------------------
static TRANSLATION_TABLE_DEF_T translation_table_defs_trigger[MAX_PARALLEL_TRANSFER][4] =
//-------------------------------------------------------------------------------------------
{

//channel 0
    {{AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE / ( MAX_PARALLEL_TRANSFER * cfdCommandFifoMaxList )}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE / cfdCommandFifoMaxList},  {0, 0}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE / cfdCommandFifoMaxList}},
//channel 1
    {{AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE + TRANSLATION_TABLE_SIZE / MAX_PARALLEL_TRANSFER, TRANSLATION_TABLE_SIZE / ( MAX_PARALLEL_TRANSFER * cfdCommandFifoMaxList )}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE / cfdCommandFifoMaxList}, { 0, 0 }, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE / cfdCommandFifoMaxList}}
};

//-------------------------------------------------------------------------------------------
static TRANSLATION_TABLE_DEF_T translation_table_defs_64K[MAX_PARALLEL_TRANSFER][4] =
//-------------------------------------------------------------------------------------------
{
//channel 0
    {{AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, ( TRANSLATION_TABLE_SIZE_64K / MAX_PARALLEL_TRANSFER )}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE_64K}, {0, 0}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE_64K}},
//channel1
    {{( AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE + ( TRANSLATION_TABLE_SIZE_64K / MAX_PARALLEL_TRANSFER ) ), ( TRANSLATION_TABLE_SIZE_64K / MAX_PARALLEL_TRANSFER )}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE_64K}, {0, 0}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE_64K}}
};

//-------------------------------------------------------------------------------------------
static TRANSLATION_TABLE_DEF_T translation_table_defs_trigger_64K[MAX_PARALLEL_TRANSFER][4] =
//-------------------------------------------------------------------------------------------
{

//channel 0
    {{AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE_64K / ( MAX_PARALLEL_TRANSFER * cfdCommandFifoMaxList )}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE_64K / cfdCommandFifoMaxList},  {0, 0}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE_64K / cfdCommandFifoMaxList}},
//channel 1
    {{AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE + TRANSLATION_TABLE_SIZE_64K / MAX_PARALLEL_TRANSFER, TRANSLATION_TABLE_SIZE_64K / ( MAX_PARALLEL_TRANSFER * cfdCommandFifoMaxList )}, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE_64K / cfdCommandFifoMaxList}, { 0, 0 }, {AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE, TRANSLATION_TABLE_SIZE_64K / cfdCommandFifoMaxList}}
};

//-------------------------------------------------------------------------------------------
void
complete_request( volatile struct io_object *ioobj, unsigned char status )
//-------------------------------------------------------------------------------------------
{
    if( ioobj )
    {
        PRINTKM( DMA, ( PKTD "%s ioobj %p\n", ioobj->device_index,
                        __FUNCTION__, ioobj ) );
        ///< todo answer with correct result
#ifdef REMOVE_REQUEST_BUFFER_MAPPING
        {
            struct hyperion_device *device
                = (struct hyperion_device *)ioobj->iocb->ki_filp->private_data;
            run_iocb( device->pqueue_result, (struct io_object *)ioobj );
        }
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION( 4, 0, 0 )
        aio_complete( ioobj->iocb, ioobj->count, 0 );
#elif LINUX_VERSION_CODE < KERNEL_VERSION( 5, 16, 0 )
        ioobj->iocb->ki_complete( ioobj->iocb, ioobj->count, 0 );
#else
        ioobj->iocb->ki_complete( ioobj->iocb, ioobj->count );
#endif
    }
}

//-------------------------------------------------------------------------------------------
void
free_ubuf( struct hyperion_device *device, struct user_buffer_list *ubuf_obj )
//-------------------------------------------------------------------------------------------
{
    int i;
    struct scatterlist *sgl;
    if( ubuf_obj != NULL )
    {
#ifndef REMOVE_REQUEST_BUFFER_MAPPING
        list_del( &ubuf_obj->list );
#endif
        if( ubuf_obj->sg )
        {
            sgl = ubuf_obj->sg;
            for( i = 0; i < ubuf_obj->nr_pages; i++ )
            {
                // printk(" %s pci_unmap_page( addr 0x%llx) index %d\n",
                // __FUNCTION__,  sgl[i].dma_address, i );
                dma_unmap_page( &device->pdev->dev, sgl[i].dma_address,
                                PAGE_SIZE, DMA_FROM_DEVICE );
            }
            sgl_unmap_user_pages( ubuf_obj->sg, ubuf_obj->nr_pages, 0 );
            vfree( ubuf_obj->sg );
        }
        vfree( ubuf_obj );
    }
}

//-------------------------------------------------------------------------------------------
void free_ubuf_obj( struct hyperion_device* device, char __user* buffer, size_t count )
//-------------------------------------------------------------------------------------------
{
    struct user_buffer_list* ubuf_obj;
    struct io_object* ioobj;
    ubuf_obj = get_user_buffer_obj( device, buffer, count );
    ioobj = ( struct io_object* )ubuf_obj->private;
    PRINTKM( DMA, ( PKTD "%s ubuf p%p obj p%p\n", device->index, __FUNCTION__, buffer, ubuf_obj ) );
    free_ubuf( device, ubuf_obj );
    kfree( ( void* )ioobj );

}

//-------------------------------------------------------------------------------------------
void free_all_ubuf( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    struct list_head* lh = device->head_uaddr.next;
    struct user_buffer_list* ubuf_obj = NULL;

    while( lh != &device->head_uaddr )
    {
        ubuf_obj = list_entry( lh, struct user_buffer_list, list );
        lh = lh->next;
        free_ubuf( device, ubuf_obj );
    }
}

/*
    volatile INTERRUPT_STATUS_REG   *InterruptStatus;
    volatile DMA_CONTROLLER         *DMAControllerPCI[MAX_PARALLEL_TRANSFER];
    volatile DMA_CONTROLLER         *DMAControllerVideoIn[MAX_PARALLEL_TRANSFER];
    volatile UART_REG               *UARTReg[UART_NUM];
    volatile MUX_CTRL_REG           *MuxCtrl[MAX_PARALLEL_TRANSFER];
    volatile UCHAR                  *MuxRAM[MAX_PARALLEL_TRANSFER];
    volatile AOI_XSTART_REG         *AoiXStart[MAX_PARALLEL_TRANSFER];
    volatile AOI_XSTOP_REG          *AoiXStop[MAX_PARALLEL_TRANSFER];
    volatile AOI_YSTART_REG         *AoiYStart[MAX_PARALLEL_TRANSFER];
    volatile AOI_YSTOP_REG          *AoiYStop[MAX_PARALLEL_TRANSFER];
    volatile IIC_READ_REG           *IICRead;
    volatile SYSTEM_CONTROL_REG     *SystemCtrl;
    volatile unsigned long          *SystemVersion;
    volatile SCAN_PIX_REG           *ScanPixLine0[MAX_PARALLEL_TRANSFER];
    volatile SCAN_PIX_REG           *ScanPixLine1[MAX_PARALLEL_TRANSFER];
    volatile SCAN_PIX_REG           *ScanLines[MAX_PARALLEL_TRANSFER];
    volatile ULONG                  *FrameLineCounter[MAX_PARALLEL_TRANSFER];
*/

//-------------------------------------------------------------------------------------------
void release_mapping( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    if( get_current_iocb( device->pqueue_result ) )
    {
        struct io_object* ioobj = get_current_iocb( device->pqueue_result );
        free_ubuf( device, ioobj->ubuf_obj );
        run_next_iocb( device->pqueue_result );
        kfree( ( void* )ioobj );
    }
}


//-------------------------------------------------------------------------------------------
void register_dump( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_INTERRUPT_STATUS ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_INTERRUPT_STATUS ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_VERSION ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_VERSION ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_SCAN_PIX_LINE0 ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_SCAN_PIX_LINE0 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_SCAN_PIX_LINE1 ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_SCAN_PIX_LINE1 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_SCAN_LINES ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_SCAN_LINES ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_XSTART ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_XSTART ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_XSTOP ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_XSTOP ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_YSTART ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_YSTART ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_YSTOP ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_YSTOP ) );

    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_MUX_CONTROLLER ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_MUX_CONTROLLER ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_SCAN_PIX_LINE0 ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_SCAN_PIX_LINE0 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_SCAN_PIX_LINE1 ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_SCAN_PIX_LINE1 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_SCAN_LINES ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_SCAN_LINES ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_XSTART ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_XSTART ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_XSTOP ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_XSTOP ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_YSTART ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_YSTART ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_YSTOP ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_YSTOP ) );

    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CONTROL ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CONTROL ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_VERSION ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_VERSION ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_STAT ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_STAT ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_INTR_ENABLE ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_INTR_ENABLE ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_INTR_STAT_CLEAR ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_INTR_STAT_CLEAR ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_CMD_STAT ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_CMD_STAT ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_CMD_STAT2 ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_CMD_STAT2 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_XFER_REMAIN2 ), IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_XFER_REMAIN2 ) );
    {
        u32 i, *hrtcram = ( u32* )( REG_POINTER( device->hyperion_base, device->reg_def, ebrhTriggerHrtController0, OFF_HRT_CONTROLLER_RAM ) );
        for( i = 0; i < 32; i++ )
        {
            printk( "%p : 0x%x\n", hrtcram, ioread32( ( void __iomem* )hrtcram ) );
            ++hrtcram;
            //++hrtcram;
        }
    }
}

//-------------------------------------------------------------------------------------------
unsigned char rw_has_transfer_parameter_changed( void* context )
//-------------------------------------------------------------------------------------------
{
    struct io_object* ioobj = ( struct io_object* )context;
    return ioobj->transfer_param.has_properties_changed;
}

//-------------------------------------------------------------------------------------------
int prepare_scatter_gather_list( void* context, u32 nbytes_next_xfer, u32 i_scatter_gather, unsigned int* offset )
//-------------------------------------------------------------------------------------------
{
    struct io_object* ioobj = ( struct io_object* )context;
    struct hyperion_device* device = ( struct hyperion_device* )ioobj->iocb->ki_filp->private_data;
    struct dma_transfer_object* dto = ioobj->dma_transfer_object;
    u32 itt ;
    u64* translation_table = ( u64* )dto->avalon_to_pci_table.base;
    volatile dma_addr_t address;
    int xfer, page_length, hyperion_page_size = device->pci_hyperion_page_size;

    xfer = ( int )nbytes_next_xfer;
    itt = i_scatter_gather;

    //PRINTKM(DMA,(PKTD "prep_sg_list() dto p%p xfer 0x%x isg %u max_e %lu\n", ioobj->device_index, dto, xfer, itt ));
    *offset = ioobj->sg[ioobj->isg].offset;
    xfer -= *offset;
    //PRINTKM(DMA,(PKTD "xfer 0x%x offset 0x%x\n", ioobj->device_index, xfer, *offset ));

    if( hyperion_page_size != PAGE_SIZE )
    {
        while( xfer > 0 && itt < TRANSLATION_TABLE_ELEMENTS && ioobj->isg < ioobj->do_dio )
        {
            //PRINTKM(DMA,(PKTD " ScatterGatherList next element %d xfer 0x%x\n", ioobj->device_index, ioobj->isg, xfer ));
            address = sg_dma_address( &ioobj->sg[ioobj->isg] );
            address = ( address & ~3 ) | device->address_space_encoding;
            page_length = ( int )( ioobj->sg[ioobj->isg].length );
            while( page_length > 0 )
            {
                translation_table[itt] = cpu_to_le64( ( u64 )address );
                //PRINTKM(DMA,(PKTD " %s: xfer 0x%x itranslationtable %d translationtable %p addr 0x%x len 0x%x\n", ioobj->device_index, __FUNCTION__, xfer, itt, &translation_table[itt], address, page_length ));
                page_length -= hyperion_page_size;
                address += hyperion_page_size;
                if( itt++ >= TRANSLATION_TABLE_ELEMENTS )
                {
                    //PRINTKM(DMA,(PKTD " %s itranslationtable %d return dcecTranslationTableElements\n", ioobj->device_index, __FUNCTION__, itt ));
                    return dcecTranslationTableElements;
                }
            }
            xfer -= ( int )( ioobj->sg[ioobj->isg].length );
            ++ioobj->isg;
        }
    }
    else
    {
        while( xfer > 0 && itt < TRANSLATION_TABLE_ELEMENTS && ioobj->isg < ioobj->do_dio )
        {
            //PRINTKM(DMA,(PKTD " ScatterGatherList next element %d xfer 0x%x\n", ioobj->device_index, ioobj->isg, xfer ));
            address = sg_dma_address( &ioobj->sg[ioobj->isg] );
            address = ( address & ~3 ) | device->address_space_encoding;
            translation_table[itt] = cpu_to_le64( ( u64 )address );
            //PRINTKM(DMA,(PKTD " %s: xfer 0x%x itranslationtable %d translationtable %p addr 0x%x len 0x%x\n", ioobj->device_index, __FUNCTION__, xfer, itt, &translation_table[itt], address, page_length ));
            xfer -= ( int )( ioobj->sg[ioobj->isg].length );
            ++itt;
            ++ioobj->isg;
        }
    }
    wmb();

    //PRINTKM(DMA,(PKTD " prepare_scatter_gather_list() page_off 0x%lx ioobj->isg %ld\n", ioobj->device_index, PAGE_SIZE + xfer, ioobj->isg ));
    return dcecNoError;
}

//-------------------------------------------------------------------------------------------
static unsigned char dma_controller_interrupt( struct hyperion_device* device, struct dma_transfer_object* dto, DMA_CONTROLLER* dma_controller, unsigned int int_src )
//-------------------------------------------------------------------------------------------
{
    TItem it;
    BOOLEAN writeOK;
    u32 int_stat, int_en, dma_error = cerrSuccess, ie;
    volatile struct io_object* ioobj = _GET_ACTIVE_IOOBJ( dto );

    int_stat = ioread32( ( void __iomem* )( dma_controller->dma_reg.interrupt_status ) );
    iowrite32( int_stat, ( void __iomem* )( dma_controller->dma_reg.interrupt_status ) );
    int_en = ioread32( ( void __iomem* )( dma_controller->dma_reg.interrupt_enable ) );

    PRINTKM( DMA, ( PKTD " %s %lu int_src %d int_en 0x%x int_stat 0x%x, frameid %ld, dto p%p dma_master 0x%x\n", device->index, __FUNCTION__, jiffies, int_src, int_en, int_stat, dto->requestID, dto, ioread32( ( void __iomem* )( dma_controller->dma_reg.transfer_ctrl_status ) ) ) );

    if( ( int_stat & int_en ) == 0 )
    {
        return FALSE;
    }

    if( ( int_stat & int_en ) & COMMAND_ABORT_INTR )
    {
        if( ioobj != NULL )
        {
            u32 transfer_block_length, actual_transfer_length;
            transfer_block_length = ioread32( ( void __iomem* )( ioobj->controller_videoin->cmd_xfer_total ) );
            actual_transfer_length = ioread32( ( void __iomem* )( ioobj->controller_videoin->cmd_xfer_remain ) );
            ioobj->numxfer += ( transfer_block_length - actual_transfer_length ) << 2;
            PRINTKM( DMA, ( PKTD " %s, %lu, xfer_tot 0x%x xfer_remain 0x%x numxfer 0x%lx", device->index, __FUNCTION__, jiffies, transfer_block_length, actual_transfer_length, ioobj->numxfer ) );
        }
    }
    if( ( int_stat & int_en ) & PSEUDO_INTR )
    {
        ie = int_en;
        ie &= ~PSEUDO_INTR;
        iowrite32( ie, ( void __iomem* )( dma_controller->dma_reg.interrupt_enable ) );
        dma_error = cerrTimeout;
    }
    if( ( int_stat & int_en ) & DATA_OVERFLOW_INTR )
    {
        u32 dma_stat = ioread32( ( void __iomem* )( dma_controller->dma_reg.dma_status ) );
        PRINTKM( DMA, ( PKTD " %s %lu dma_overflow detected isrc %d ien 0x%x ist 0x%x dst 0x%x nbytes %d\n", device->index, __FUNCTION__, jiffies, int_src, int_en, int_stat, dma_stat, dma_controller->nbytes ) );
        dto->abort_this_request = TRUE;
    }
    int_en |= OPCODE_STATUS_MSK;
    memset( &it, 0, sizeof( TItem ) );
    it.dto = dto;
    it.int_stat = int_stat & int_en;
    it.int_src = int_src;
    it.controller = ioobj != NULL ? ioobj->controller_videoin : dma_controller;
    it.dma_error = dma_error;
    _PIPE_WRITE( device->dma_interrupt_result, it, writeOK );
    return TRUE;
}

//-------------------------------------------------------------------------------------------
static unsigned char uart_interrupt( struct hyperion_device* device, uart_object_t* pua )
//-------------------------------------------------------------------------------------------
{
    u32 int_stat, int_en;
    int_stat = IO_READ_32( device->hyperion_base, device->reg_def, pua->register_index, OFF_UART_INTR_CLEAR_STAT );
    IO_WRITE_32( device->hyperion_base, device->reg_def, pua->register_index, OFF_UART_INTR_CLEAR_STAT, int_stat );
    int_en = IO_READ_32( device->hyperion_base, device->reg_def, pua->register_index, OFF_UART_INTR_ENABLE );

    //PRINTKM(INTR,(PKTD " uart ctrl 0x%x int_stat 0x%x int_en 0x%x\n", device->index, _READ_REG( pua->uart_reg.UARTRead, Ctrl ), int_stat, int_en ));
    if( int_stat & int_en )
    {
        TItem it;
        BOOLEAN writeOK;
        memset( &it, 0, sizeof( TItem ) );
        it.pua = pua;
        it.int_stat = int_stat & int_en;
        //PRINTKM(INTR,(PKTD " uart ctrl 0x%x int_stat 0x%x int_en 0x%x", device->index, _READ_REG( pua->uart_reg.UARTRead, Ctrl ), int_stat, int_en ));
        _PIPE_WRITE( device->dma_interrupt_result, it, writeOK );
        /// \todo add result handling for pipewrite e.g. if queue is full
        return TRUE;
    }
    return FALSE;
}

#if ENABLE_POCL
//-------------------------------------------------------------------------------------------
static unsigned char pocl_interrupt( struct hyperion_device* device, POCL_OBJECT* ppo )
//-------------------------------------------------------------------------------------------
{
    u32 int_stat, int_en;
    int_stat = IO_READ_32( device->hyperion_base, device->reg_def, ppo->register_index, OFF_POCL_INTR_CLEAR_STAT );
    IO_WRITE_32( device->hyperion_base, device->reg_def, ppo->register_index, OFF_POCL_INTR_CLEAR_STAT, int_stat );
    int_en = IO_READ_32( device->hyperion_base, device->reg_def, ppo->register_index, OFF_POCL_INTR_ENABLE );
    if( int_stat & int_en )
    {
        TItem it;
        BOOLEAN writeOK;
        memset( &it, 0, sizeof( TItem ) );
        it.ppo = ppo;
        //DoTraceMessage( TRACELEVELINTERRUPT, " %s, %I64u, uart ctrl 0x%x int_stat 0x%x int_en 0x%x", __FUNCTION__,  _READ_REG( pua->UartReg.UARTRead, Ctrl ), int_stat, int_en );
        _PIPE_WRITE( device->dma_interrupt_result, it, writeOK );
        ///// \todo add result handling for pipewrite e.g. if queue is full
        return TRUE;
    }
    return FALSE;
}
#endif
/////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------------------------------
// our top half interrupt handler, only search about the interrupt source
#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,19)
irqreturn_t hyperion_interrupt( int irq, void* dev_id, struct pt_regs* fake )
#else
// The prototype for interrupt handler functions has changed in 2.6.19.
// In short, the regs argument has been removed, since almost nobody used it.
// Any interrupt handler which needs the pre-interrupt register state can use get_irq_regs() to obtain it.
irqreturn_t hyperion_interrupt( int irq, void* dev_id )
#endif
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device = ( struct hyperion_device* )dev_id;
    irqreturn_t ret = IRQ_NONE;
    unsigned char interrupting = FALSE;
    unsigned int int_stat = IO_READ_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_INTERRUPT_STATUS );
    int interrupt;

    //PRINTKM(DMA,(PKTD " %s %lu int_stat 0x%x\n", device->index, __FUNCTION__, jiffies, int_stat ));
    for( interrupt = 0; interrupt < eIntMax; interrupt++ )
    {
        if( int_stat & ( 1 << interrupt ) )
        {
            switch( interrupt )
            {
            case eIntVideoInDma0:
                {
                    interrupting |= dma_controller_interrupt( device, device->pdma_object[0],   device->pdma_object[0]->dma_controller_videoin[cfdCommandFifo0], eIntVideoInDma0 );
                    break;
                }
            case eIntVideoInDma1:
                {
                    interrupting |= dma_controller_interrupt( device, device->pdma_object[1], device->pdma_object[1]->dma_controller_videoin[cfdCommandFifo0], eIntVideoInDma1 );
                    break;
                }
            case eIntDdrToPciDma0:
                {
                    interrupting |= dma_controller_interrupt( device, device->pdma_object[0], device->pdma_object[0]->dma_controller_pci, eIntDdrToPciDma0 );
                    break;
                }
            case eIntDdrToPciDma1:
                {
                    interrupting |= dma_controller_interrupt( device, device->pdma_object[1],
                                    device->pdma_object[1]->dma_controller_pci, eIntDdrToPciDma1 );
                    break;
                }
            case eIntUart0:
                {
                    interrupting |= uart_interrupt( device, &device->uart_port[0] );
                    break;
                }
            case eIntUart1:
                {
                    interrupting |= uart_interrupt( device, &device->uart_port[1] );
                    break;
                }
#if ENABLE_POCL
            case eIntPoCL0:
                {
                    interrupting |= pocl_interrupt( device, &device->pocl[0] );
                    break;
                }
            case eIntPoCL1:
                {
                    interrupting |= pocl_interrupt( device, &device->pocl[1] );
                    break;
                }
#endif
            }
        }
    }

    if( interrupting )
    {
        tasklet_schedule( &device->hyperion_tasklet );
        ret = IRQ_HANDLED;
    }

    return ret;
}

//-------------------------------------------------------------------------------------------
void complete_transfer( TItem dma_res, volatile struct io_object* ioobj, u32 status )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device;
    struct dma_transfer_object* dto = dma_res.dto;
    //PRINTKM(DMA,(PKTD " %s transfer ready compl_ioobj p%p\n", device->index, __FUNCTION__, dto->active_ioobj ));
    if( ioobj == NULL )
    {
        return;
    }
    //if (ioobj->Cancel)
    //  status = STATUS_CANCELLED;
    //else
    //  status = AreRequestsBeingAborted(dto->RWQueue);

    //if(NT_SUCCESS(status))
    {
        //flush buffers ???
        write_results_to_rq( ioobj, get_jiffies_64(), status );
    }
    //else
    //write_results_to_rq( ioobj, get_jiffies_64(), cerrDMACancelled );
    if( get_current_iocb( dto->rw_queue ) == NULL )
    {
        //printk( " %s queue empty detected\n", __FUNCTION__);
        device = ( struct hyperion_device* )ioobj->iocb->ki_filp->private_data;
        IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhTriggerHrtController0 + ioobj->transfer_param.inputchannel, OFF_HRT_CONTROLLER_CTRL, 0 );
        StopTransfer( ioobj->controller_videoin );
    }
    del_timer( ( struct timer_list* )&ioobj->timer_iocb );
    //PRINTKM(DMA,(PKTD " %s %lu request completed buffer %p reqid %d channel %d status %dcompleted %s\n", ioobj->device_index, __FUNCTION__, jiffies, ioobj->buffer, ioobj->transfer_param.reqid, ioobj->transfer_param.inputchannel, status, current->comm ));
    complete_request( ioobj, 0 );
}

//-------------------------------------------------------------------------------------------
unsigned char complete_active_ioobj( TItem dma_res, u32 status )
//-------------------------------------------------------------------------------------------
{
    struct dma_transfer_object* dto = dma_res.dto;
    volatile struct io_object* ioobj = dto->active_ioobj;
    dto->active_ioobj = NULL;
#if ENABLE_OVERLAPPED_TRANSFER
    if( ioobj )
    {
        complete_transfer( dma_res, ioobj, status );
        return TRUE;
    }
    else
    {
        return FALSE;
    }
#else
    ioobj = get_current_iocb( dto->rw_queue );
    if( !ioobj )
    {
        PRINTKM( DMA, ( PKTD " %s wrong condition found active_ioobj NULL\n", device->index, __FUNCTION__ ) );
        return FALSE;
    }
    //if(NT_SUCCESS(status))
    {
        //flush buffers ???
        write_results_to_rq( ioobj, get_jiffies_64(), status );
    }
    //else
    //write_results_to_rq( ioobj, get_jiffies_64(), cerrDMACancelled );
    StopTransfer( dto->dma_controller_videoin[cfdCommandFifo0] );
    run_next_iocb( dto->rw_queue );
    del_timer( &ioobj->timer_iocb );
    //PRINTKM(DMA,(PKTD " %s %lu request completed buffer %p reqid %d channel %d status %d completed %s\n", ioobj->device_index, __FUNCTION__, jiffies, ioobj->buffer, ioobj->transfer_param.reqid, ioobj->transfer_param.inputchannel, status, current->comm ));
    complete_request( ioobj, 0 );
    return TRUE;
#endif
}

//-------------------------------------------------------------------------------------------
void restart_current_transfer( volatile struct io_object* ioobj, struct dma_transfer_object* dto )
//-------------------------------------------------------------------------------------------
{
    if( ioobj == NULL )
    {
        return;
    }
    StopTransfer( ioobj->controller_videoin );
    ClearCommandFifo( ioobj->controller_videoin );
    ResetDMACtrlBuffer( ioobj->controller_videoin );
    restart_current_iocb( dto->rw_queue );
}

//-------------------------------------------------------------------------------------------
void execute_commad_intr_req( struct hyperion_device* device, TItem dma_res )
//-------------------------------------------------------------------------------------------
{
    int i;
    unsigned long opc_stat = ( dma_res.int_stat & OPCODE_STATUS_MSK ) >> 28;
    struct dma_transfer_object* dto = dma_res.dto;
    volatile struct io_object* ioobj;

    if( dto->dma_controller_pci )
    {
        for( i = 0; i < NUMOFOPCS; i++ )
        {
            switch( opc_stat & ( 1 << i ) )
            {
            case opcsfUnlock:
                {
                    //if( dto->active_ioobj )
                    //  ioobj = dto->active_ioobj;
                    //else
                    ioobj = get_current_iocb( dto->rw_queue );
                    if( !ioobj )
                    {
                        break;
                    }

                    if( dma_res.controller == dto->dma_controller_videoin[cfdCommandFifo0] )
                    {
                        HandleDMAEventDirectTransfer( dto->dma_controller_videoin[cfdCommandFifo0], IncrementReadData, dma_res.int_stat );
                        if( PrepareTransferRAMToPCI( dto->dma_controller_pci, ( void* )ioobj ) != dcecNoError )
                        {
                            break;
                        }
                        StartTransfer( dto->dma_controller_pci, FALSE );
                    }
                    else if( dma_res.controller == dto->dma_controller_pci )
                    {
                        HandleDMAEventDirectTransfer( dto->dma_controller_videoin[cfdCommandFifo0], IncrementWriteQuota, dma_res.int_stat );
                        HandleDMAEventDirectTransfer( dto->dma_controller_pci, IncrementWriteQuota, dma_res.int_stat );
                    }
                    else
                        ;
                    break;
                }
            case opcsfStartNext:
                {
                    if( dma_res.controller == dto->dma_controller_videoin[cfdCommandFifo0] )
                    {
                        //flush buffers ???
                        //dto->active_ioobj = run_next_iocb( dto->rw_queue );
                    }
                    else if( dma_res.controller == dto->dma_controller_pci )
                        ;// nothing to do
                    else
                        ;// ?
                    break;
                }
            case opcsfTransferReady:
                {
                    if( dma_res.controller == dto->dma_controller_pci )
                    {
                        //ioobj = dto->active_ioobj;
                        //dto->active_ioobj = NULL;
                        ioobj = get_current_iocb( dto->rw_queue );
                        if( !ioobj )
                        {
                            break;
                        }
                        //if (ioobj->Cancel)
                        //  status = STATUS_CANCELLED;
                        //else
                        //  status = AreRequestsBeingAborted(dto->RWQueue);
                        //if(NT_SUCCESS(status))
                        {
                            //flush buffers ???
                            write_results_to_rq( ioobj, get_jiffies_64(), cerrSuccess );
                        }
                        //else
                        //write_results_to_rq( ioobj, get_jiffies_64(), cerrDMACancelled );
                        del_timer( ( struct timer_list* )&ioobj->timer_iocb );
                        run_next_iocb( dto->rw_queue );
                        PRINTKM( DMA, ( PKTD " %s %lu msec request completed buffer %p reqid %d channel %d completed %s\n", ioobj->device_index, __FUNCTION__, jiffies, ioobj->buffer, ioobj->transfer_param.reqid, ioobj->transfer_param.inputchannel, current->comm ) );
                        complete_request( ioobj, 0 );
                        break;
                    }
                    break;
                }
            default:
                break;
            }//switch( opc_stat & (1 << i) )
        }//for( i = 0; i < NUMOFOPCS; i++ )
    }
    else // direct transfer
    {
        for( i = 0; i < NUMOFOPCS; i++ )
        {
            switch( opc_stat & ( 1 << i ) )
            {
            case opcsfUnlock:
                {
                    ioobj = _GET_ACTIVE_IOOBJ( dto );
                    if( ioobj == NULL )
                    {
                        break;
                    }
                    if( dma_res.int_stat & COMMAND_ABORT_INTR )
                    {
                        ResetDMACtrlBuffer( ioobj->controller_videoin );
                    }
                    else
                    {
                        ioobj->numxfer += ioobj->controller_videoin->write_buffer.block_size;
                        if( HandleDMAEventDirectTransfer( ioobj->controller_videoin, IncrementWriteQuota, dma_res.int_stat ) != dcecNoError )
                        {
                            StopTransfer( ioobj->controller_videoin );
                            //remove iopbj with cerrDMAScatterGatherList
                        }
                    }
                    break;
                }
            case opcsfStartNext:
                {
#if ENABLE_OVERLAPPED_TRANSFER
                    //flush buffers ???
                    ioobj = get_current_iocb( dto->rw_queue );
                    if( ioobj == NULL )
                    {
                        break;
                    }
                    dto->active_ioobj = ioobj;
                    run_next_iocb( dto->rw_queue );
#endif
                    //PRINTKM(DMA,(PKTD " %s startnext compl_ioobj p%p\n", device->index, __FUNCTION__, dto->active_ioobj ));
                    break;
                }
            case opcsfTransferReady:
                {
                    if( complete_active_ioobj( dma_res, cerrSuccess ) == FALSE )
                    {
                        break;
                    }
                    if( get_current_iocb( dto->rw_queue ) )
                    {
                        ioobj = get_current_iocb( dto->rw_queue );
                        if( dto->restart_current_ioobj )
                        {
                            dto->restart_current_ioobj = FALSE;
                            restart_current_transfer( ioobj, ioobj->dma_transfer_object );
                        }
                        else if( ioobj->transfer_param.trigger_mode == etsStartSignal )
                        {
                            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhTriggerHrtController0 + ioobj->transfer_param.inputchannel, OFF_HRT_CONTROLLER_CTRL, 0 );
                            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhTriggerHrtController0 + ioobj->transfer_param.inputchannel, OFF_HRT_CONTROLLER_CTRL, HRTC_ENABLE | HRTC_INTERRUPT_ENABLE );
                            StartTransfer( ioobj->controller_videoin, FALSE );
                        }
                        else if( ( ioobj->transfer_param.trigger_mode & ( etsStartSignalOverlap | etsStopSignal ) ) && ( ( dma_res.int_stat & COMMAND_ABORT_INTR ) == 0 ) )
                        {
                            restart_current_transfer( ioobj, ioobj->dma_transfer_object );
                        }
                        else
                            ;
                    }
                    break;
                }

            default:
                break;
            }//switch( opc_stat & (1 << i) )
        }//for( i = 0; i < NUMOFOPCS; i++ )
    }
}

//-------------------------------------------------------------------------------------------
void remove_all_requests( struct hyperion_device* device, struct dma_transfer_object* dto, TItem dma_res )
//-------------------------------------------------------------------------------------------
{
    volatile struct io_object* ioobj;
    PRINTKM( DMA, ( PKTD " +%s %lu\n", device->index, __FUNCTION__, jiffies ) );

    if( dto->active_ioobj )
    {
        ioobj = dto->active_ioobj;
        dto->active_ioobj = NULL;
        complete_transfer( dma_res, ioobj, cerrDMAAborted );
    }
    ioobj = get_current_iocb( dto->rw_queue );
    if( ioobj )
    {
        complete_transfer( dma_res, ioobj, cerrDMAAborted );
    }

    remove_and_notify_iocbs( dto->rw_queue );
    complete_all( &device->compl_abort );
    PRINTKM( DMA, ( PKTD " -%s %lu\n", device->index, __FUNCTION__, jiffies ) );
}


//-------------------------------------------------------------------------------------------
int check_transfer_state( struct hyperion_device* device, struct dma_transfer_object* dto, volatile struct io_object* ioobj )
//-------------------------------------------------------------------------------------------
{
    u32 dma_status = ioread32( ( void __iomem* )( dto->dma_controller_videoin[cfdCommandFifo0]->dma_reg.dma_status ) );
    PRINTKM( DMA, ( PKTD " %s %lu m_stat %x\n", device->index, __FUNCTION__, jiffies, dma_status ) );
    if( ( dma_status & 0xffff ) != 0 && dto->timeout_occurred == FALSE )
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
//-------------------------------------------------------------------------------------------
void abort_process_timeout( struct timer_list* t )
//-------------------------------------------------------------------------------------------
{
    struct dma_transfer_object* dto = from_timer( dto, t, abort_timer );
#else
//-------------------------------------------------------------------------------------------
void abort_process_timeout( unsigned long arg )
//-------------------------------------------------------------------------------------------
{
    struct dma_transfer_object* dto = ( struct dma_transfer_object* )arg;
#endif
    iowrite32( PSEUDO_INTR, ( void __iomem* )( dto->dma_controller_videoin[cfdCommandFifo0]->dma_reg.interrupt_enable ) );
}

//-------------------------------------------------------------------------------------------
void execute_driver_generated_intr_req( struct hyperion_device* device, TItem dma_res )
//-------------------------------------------------------------------------------------------
{
    struct dma_transfer_object* dto = dma_res.dto;
    volatile struct io_object* ioobj = NULL;

    PRINTKM( DMA, ( PKTD " %s %lu dma_transfer error %d abort_transfer % d\n", device->index, __FUNCTION__, jiffies, dma_res.dma_error, dto->abort_all_transfer ) );
    if( dto->abort_all_transfer || dto->abort_this_request )
    {
        DisableInterrupts( dto->dma_controller_videoin[cfdCommandFifo0] );
        ClearCommandFifo( dto->dma_controller_videoin[cfdCommandFifo0] );
        ResetDMACtrlBuffer( dma_res.controller );
        _PIPE_RESET( device->dma_interrupt_result );
        if( check_transfer_state( device, dto, ioobj ) )
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
            timer_setup( &dto->abort_timer, abort_process_timeout, 0 );
#else
            setup_timer( &dto->abort_timer, abort_process_timeout, ( unsigned long )dto );
#endif
            mod_timer( &dto->abort_timer, jiffies + 20 );
            return;
        }
        dto->rewrite_transfer_parameter = TRUE;
        dto->mux_seq.changed = TRUE;
        StopTransfer( dto->dma_controller_videoin[cfdCommandFifo0] );
        ResetDMAController( dto->dma_controller_videoin[cfdCommandFifo0] );
        if( dto->abort_all_transfer )
        {
            dto->abort_all_transfer = FALSE;
            remove_all_requests( device, dto, dma_res );
        }
        else if( dto->abort_this_request )
        {
            if( dto->active_ioobj )
            {
                ioobj = dto->active_ioobj;
                dto->active_ioobj = NULL;
                complete_transfer( dma_res, ioobj, cerrDMADataOverflow );
                restart_current_iocb( dto->rw_queue );
            }
            else
            {
                ioobj = run_next_iocb( dto->rw_queue );
                if( ioobj )
                {
                    complete_transfer( dma_res, ioobj, cerrDMADataOverflow );
                }
            }
            dto->abort_this_request = FALSE;
        }
    }
    else
    {
        ResetDMAController( dto->dma_controller_videoin[cfdCommandFifo0] );
        if( dto->active_ioobj )
        {
            ioobj = dto->active_ioobj;
            dto->active_ioobj = NULL;
            write_results_to_rq( ioobj, get_jiffies_64(), cerrTimeout );
            complete_request( ioobj, 0 );
            restart_current_iocb( dto->rw_queue );
        }
        else
        {
            ioobj = run_next_iocb( dto->rw_queue );
            if( ioobj )
            {
                write_results_to_rq( ioobj, get_jiffies_64(), cerrTimeout );
                complete_request( ioobj, 0 );
            }
        }
        dto->timeout_occurred = FALSE;
    }//if( device->abort_all_transfer )
}

//-------------------------------------------------------------------------------------------
void execute_command_abort_intr_req( struct hyperion_device* device, TItem dma_res )
//-------------------------------------------------------------------------------------------
{
    struct dma_transfer_object* dto = dma_res.dto;

    if( complete_active_ioobj( dma_res, cerrSuccess ) == FALSE )
    {
        return;
    }
    ResetDMACtrlBuffer( dma_res.controller );
    if( get_current_iocb( dto->rw_queue ) )
    {
        struct io_object* ioobj = get_current_iocb( dto->rw_queue );
        if( dto->restart_current_ioobj )
        {
            dto->restart_current_ioobj = FALSE;
            restart_current_transfer( ioobj, ioobj->dma_transfer_object );
        }
    }
}

//-------------------------------------------------------------------------------------------
void hyperion_do_tasklet( unsigned long index )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device = get_hyperion_device( index );
    struct dma_transfer_object* dto;
    TItem dma_res;
    unsigned char read_ok;
    int intr;

    //PRINTKM(DMA,(PKTD " %s( device_index %d )\n", (int)index, __FUNCTION__, (int)index));
    do
    {
        _PIPE_READ_LOCK( device->dma_interrupt_result, &dma_res, read_ok );
        if( read_ok == 0 )
        {
            break;
        }

        if( dma_res.dto != 0  && dma_res.controller != 0 )
        {
            dto = dma_res.dto;
            if( dto->abort_all_transfer || dto->abort_this_request )
            {
                execute_driver_generated_intr_req( device, dma_res );
            }
            else
            {
                for( intr = 0; intr < IRQ_MAX; intr++ )
                {
                    if( ( dma_res.int_stat & ( 1 << intr ) ) == COMMAND_READY_INTR )
                    {
                        execute_commad_intr_req( device, dma_res );
                    }
                    else if( ( dma_res.int_stat & ( 1 << intr ) ) == COMMAND_ABORT_INTR )
                    {
                        execute_command_abort_intr_req( device, dma_res );
                    }
                    else if( ( dma_res.int_stat & ( 1 << intr ) ) == PSEUDO_INTR )
                    {
                        execute_driver_generated_intr_req( device, dma_res );
                    }
                }
            }
        }
        else if( dma_res.pua != 0 )
        {
            read_serial( dma_res.pua );
        }
        else if( dma_res.ppo != 0 )
        {
            HandlePoCLEvent( dma_res.ppo );
        }
    }
    while( 1 );
}

//-------------------------------------------------------------------------------------------
int cpy_to_footer( volatile struct io_object* ioobj, char* result_list, int size )
//-------------------------------------------------------------------------------------------
{
    int page_idx, page_off, page_len;
    char* footer;
    struct page* page;

    page_idx = ioobj->transfer_param.footeroffset / PAGE_SIZE;
    page_off = ioobj->transfer_param.footeroffset & ( PAGE_SIZE - 1 );
    page_len = ioobj->sg[page_idx].length - page_off;
    while( size > 0 && page_idx < ioobj->do_dio )
    {
        page_len = ioobj->sg[page_idx].length - page_off;
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 24 )
        page = sg_page( &ioobj->sg[page_idx] );
#else
        page = ioobj->sg[page_idx].page;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 37 )
        footer = ( char* )kmap_atomic( page ) + page_off;
#else
        footer = ( char* )kmap_atomic( page, KM_SOFTIRQ0 ) + page_off;
#endif
        if( footer != NULL )
        {
            memcpy( footer, result_list, page_len );
        }
        else
        {
            return -1;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 37 )
        kunmap_atomic( footer );
#else
        kunmap_atomic( footer, KM_SOFTIRQ0 );
#endif
        ++page_idx;
        page_off = 0;
        size -= page_len;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------
/// \brief write all results, error, timestamp to requestlist
void write_results_to_rq( volatile struct io_object* ioobj, u64 j64, unsigned long status )
//-------------------------------------------------------------------------------------------
{
    struct dma_transfer_object* dto = ( struct dma_transfer_object* )ioobj->dma_transfer_object;
    u64 nsec;
    volatile TPropertyElement* list1;
    char* list;

    ioobj->result_packet = NULL; //will be freed in ioctl-after appl reads results
    nsec = j64 * TICK_NSEC;
    do_div( nsec, NSEC_PER_USEC );

    list1 = ioobj->result_property;
    list = ( char* )list1;
    _SET_PROP_ELEM_I( list1, prSResPropertyCount, prSnapRequestResultMax, TRUE );
    _SET_PROP_ELEM_I( list1, prSResStatus, status, TRUE );
    _SET_PROP_ELEM_I( list1, prSResRequestID, ioobj->transfer_param.reqid, TRUE );
    _SET_PROP_ELEM_I( list1, prSResFrameNr, dto->frame_counter, TRUE );
    _SET_PROP_ELEM_I( list1, prSResTimeStampLowPart, ( nsec & 0xffffffff ), TRUE );
    _SET_PROP_ELEM_I( list1, prSResTimeStampHighPart, ( ( nsec >> 32 ) & 0xffffffff ), TRUE );
    _SET_PROP_ELEM_I( list1, prSResScanPixLine0, ioobj->scan_pixel_line0, TRUE );
    _SET_PROP_ELEM_I( list1, prSResScanPixLine1, ioobj->scan_pixel_line1, TRUE );
    _SET_PROP_ELEM_I( list1, prSResScanLines, ioobj->scan_lines, TRUE );
    if( ioobj->numxfer > ioobj->transfer_param.transferlength )
    {
        ioobj->numxfer = ioobj->transfer_param.transferlength;
    }
    _SET_PROP_ELEM_I( list1, prSresBytesTransferredSoFar, ioobj->numxfer, TRUE );

    cpy_to_footer( ioobj, list, ioobj->transfer_param.footersize );
    PRINTKM( DMA, ( PKTD " %s status %lx\n", ioobj->device_index, __FUNCTION__, status ) );
}

/*
//-------------------------------------------------------------------------------------------
static int cancel_async_read_write( struct kiocb *iocb, struct io_event *event )
//-------------------------------------------------------------------------------------------
{
    struct io_object *ioobj = (struct io_object *)iocb->private;
    struct dma_transfer_object *dto = (struct dma_transfer_object *)ioobj->dma_transfer_object;
    long result = 0, user_result = 0;

    //if( iocb->privatedata->transfering )
    //  stop tranfer
    cancel_iocb( dto->rw_queue, iocb );
    aio_complete( iocb, result, user_result );
    return 0;
}
*/

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
//-------------------------------------------------------------------------------------------
void jit_timer_cancel_request( struct timer_list* t )
{
    struct io_object* iocb = from_timer( iocb, t, timer_iocb );
    struct dma_transfer_object* dto = ( struct dma_transfer_object* ) ( iocb->dma_transfer_object );
//-------------------------------------------------------------------------------------------
#else
//-------------------------------------------------------------------------------------------
void jit_timer_cancel_request( unsigned long arg )
//-------------------------------------------------------------------------------------------
{
    struct dma_transfer_object* dto = ( struct dma_transfer_object* )arg;
#endif
    dto->timeout_occurred = TRUE;
    iowrite32( PSEUDO_INTR, ( void __iomem* )( dto->dma_controller_videoin[cfdCommandFifo0]->dma_reg.interrupt_enable ) );
}

//-------------------------------------------------------------------------------------------
void test_interrupt( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    //test interrupt
    PRINTKM( INTR, ( PKTD " %s pdma_object p%p\n", device->index, __FUNCTION__, device->pdma_object[0] ) );
    //_WRITE_REG( device->pdma_object[0]->dma_controller_videoin->pWriteRegister, InterruptEnable, 0x10 );
    //_WRITE_BIT( device->pdma_object[0]->dma_controller_videoin->pWriteRegister, InterruptEnable, PIE, 1 );
}

//-------------------------------------------------------------------------------------------
void set_hrtc_ram( struct hyperion_device* device, struct transfer_parameter* tp )
//-------------------------------------------------------------------------------------------
{
    u32 i;
    for( i = 0; i < tp->trigger_hrtc_command_count && i < emvdHRTCTriggerCommand; i++ )
    {
        IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhTriggerHrtController0 + tp->inputchannel, ( OFF_HRT_CONTROLLER_RAM + ( i * sizeof( unsigned int ) ) ), tp->trigger_hrtc_ram[i] );
    }
}

//-------------------------------------------------------------------------------------------
void set_hrtc_ram_64( struct hyperion_device* device, struct transfer_parameter* tp )
//-------------------------------------------------------------------------------------------
{
    u32 i, *hrtc_ram = ( u32* )( REG_POINTER( device->hyperion_base, device->reg_def, ebrhTriggerHrtController0 + tp->inputchannel, OFF_HRT_CONTROLLER_RAM ) );
    for( i = 0; i < tp->trigger_hrtc_command_count && i < emvdHRTCTriggerCommand; i++ )
    {
        iowrite32( tp->trigger_hrtc_ram[i], ( void __iomem* )hrtc_ram );
        hrtc_ram++;
        hrtc_ram++;
    }
}

//-------------------------------------------------------------------------------------------
void start_transfer_dummy( struct io_object* ioobj, int index )
//-------------------------------------------------------------------------------------------
{
    PRINTKM( MOD, ( PKTD "%s\n", ioobj->device_index, __FUNCTION__ ) );
}

//-------------------------------------------------------------------------------------------
void start_transfer( struct io_object* ioobj, int index )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device = ( struct hyperion_device* )ioobj->iocb->ki_filp->private_data;
    struct transfer_parameter* tp = &ioobj->transfer_param;
    struct dma_transfer_object* dto = ioobj->dma_transfer_object;
    unsigned long timeout_jiffies;
    unsigned char synchronize_medium_channel = FALSE, restart_trigger_hrtc = FALSE;
    unsigned int mux_ctrl = 0;

    timeout_jiffies = msecs_to_jiffies( tp->timeoutmsec );
    ioobj->timer_iocb.expires = jiffies;
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
    // the timer function 'jit_timer_cancel_request' is set in the call to 'timer_setup' in the function 'async_read_write'
    PRINTKM( DMA, ( PKTD " %s init timer jiffies %lu timeout_jiffies %lu ioobj %p\n", ioobj->device_index, __FUNCTION__, jiffies, timeout_jiffies, ioobj ) );
#else
    ioobj->timer_iocb.function = jit_timer_cancel_request;
    ioobj->timer_iocb.data = ( unsigned long )dto;
    PRINTKM( DMA, ( PKTD " %s init timer jiffies %lu timeout_jiffies %lu data 0x%lx ioobj %p\n", ioobj->device_index, __FUNCTION__, jiffies, timeout_jiffies, ioobj->timer_iocb.data, ioobj ) );
#endif
    if( tp->timeoutmsec > 0 )
    {
        mod_timer( &ioobj->timer_iocb, ( jiffies + timeout_jiffies ) );
    }
    if( dto->active_ioobj != NULL && ( tp->has_properties_changed || dto->rewrite_transfer_parameter || tp->trigger_hrtc_command_count > 0 ) )
    {
        dto->restart_current_ioobj = TRUE;
        PRINTKM( MOD, ( PKTD " %s, jiffies %lu, reqid %d restart current ioobj", ioobj->device_index, __FUNCTION__, jiffies, tp->reqid ) );
        return;
    }
    if( tp->has_properties_changed || dto->rewrite_transfer_parameter )
    {
        PRINTKM( MOD, ( PKTD "%s rewrite transferparameter\n", ioobj->device_index, __FUNCTION__ ) );
        dto->rewrite_transfer_parameter = FALSE;
        mux_ctrl |= ( tp->datavalidenable > 0 )     ? DATA_VALID_ENABLE : 0;
        mux_ctrl |= ( ( !tp->scanmode ) > 0 )       ? LINE_SCAN_ENABLE : 0;
        mux_ctrl |= ( ( tp->medium_mode_ctrl << MEDIUM_MODE_BIT ) & MEDIUM_MODE_ENABLE );
        mux_ctrl |= ( tp->enable_aoimode > 0 )  ? AOI_ENABLE : 0;
        mux_ctrl |= ( ( tp->expand_lval & EXPAND_LVAL_MSK ) << EXPAND_LVAL );
        PRINTKM( MOD, ( PKTD "%s mux_ctrl %x\n", ioobj->device_index, __FUNCTION__, mux_ctrl ) );
        if( tp->medium_mode_ctrl )
        {
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_XSTART, tp->xstart );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_XSTOP, tp->xstop );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_YSTART, tp->ystart );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_AOI_YSTOP, tp->ystop );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_XSTART, tp->xstart );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_XSTOP, tp->xstop );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_YSTART, tp->ystart );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_AOI_YSTOP, tp->ystop );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER, mux_ctrl );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_MUX_CONTROLLER, mux_ctrl );
        }
        else
        {
            IO_WRITE_32( device->hyperion_base, device->reg_def, tp->inputchannel + ebrhCLController0, OFF_AOI_XSTART, tp->xstart );
            IO_WRITE_32( device->hyperion_base, device->reg_def, tp->inputchannel + ebrhCLController0, OFF_AOI_XSTOP, tp->xstop );
            IO_WRITE_32( device->hyperion_base, device->reg_def, tp->inputchannel + ebrhCLController0, OFF_AOI_YSTART, tp->ystart );
            IO_WRITE_32( device->hyperion_base, device->reg_def, tp->inputchannel + ebrhCLController0, OFF_AOI_YSTOP, tp->ystop );
            IO_WRITE_32( device->hyperion_base, device->reg_def, tp->inputchannel + ebrhCLController0, OFF_MUX_CONTROLLER, mux_ctrl );
        }
        //PRINTKM(DMA,(PKTD " mxctrl[0] 0x%x mxctrl[1] 0x%x",  device->index, *device->register_base.MuxCtrl[0], *device->register_base.MuxCtrl[1]  ));
        SET_BIT( REG_POINTER( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL ), ENABE_BASE_MODE_CH1, ( tp->medium_mode_ctrl == 0 ) );
        SetWaitOfFrameEdge( ioobj->controller_videoin, !tp->scanmode );
    }
    if( tp->trigger_hrtc_command_count > 0 )
    {
        restart_trigger_hrtc = TRUE;
        ( *device->set_hrtc_ram )( device, tp );
    }
    if( restart_trigger_hrtc || ( ( tp->trigger_mode > 0 ) && ( dto->active_ioobj == NULL ) ) )
    {
        IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhTriggerHrtController0 + tp->inputchannel, OFF_HRT_CONTROLLER_CTRL, 0 );
        IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhTriggerHrtController0 + tp->inputchannel, OFF_HRT_CONTROLLER_CTRL, HRTC_ENABLE | HRTC_INTERRUPT_ENABLE );
    }
    ioobj->isg = 0;
    //PRINTKM(DMA,(PKTD "%s ioobj %p reqId %d, timeout %d\n", ioobj->device_index, __FUNCTION__, ioobj, tp->reqid, tp->timeoutmsec ));
    SetXferTotal( ioobj->controller_videoin, ioobj->transfer_param.transferlength );
#if ENABLE_POCL
    if( tp->medium_mode_ctrl )
    {
        synchronize_medium_channel = ( GetCLClockStatus( &device->pocl[0] ) || GetCLClockStatus( &device->pocl[1] ) );
    }
#else
    synchronize_medium_channel = ( device->mux_seq.changed &&
                                   ( tp->medium_mode_ctrl > 0 ) &&
                                   ( DMAControllerVersion( ioobj->controller_videoin ) < RESET_MEDIUM_VERSION2 ) &&
                                   ( check_transfer_state( device, dto, ioobj ) == FALSE ) );
#endif
    ( *device->set_mux_data )( device, &device->mux_seq, tp->inputchannel );
    if( synchronize_medium_channel )
    {
        SynchronizeMediumChannel( ioobj->controller_videoin );
    }
    PrepareDirectTransfer( ioobj->controller_videoin, !dto->dma_controller_pci ? ( void* )ioobj : NULL, tp->scanmode, tp->has_properties_changed, tp->trigger_mode );
    ioobj->scan_pixel_line0 = IO_READ_32( device->hyperion_base, device->reg_def, tp->inputchannel + ebrhCLController0, OFF_SCAN_PIX_LINE0 );
    ioobj->scan_pixel_line1 = IO_READ_32( device->hyperion_base, device->reg_def, tp->inputchannel + ebrhCLController0, OFF_SCAN_PIX_LINE1 );
    ioobj->scan_lines = IO_READ_32( device->hyperion_base, device->reg_def, tp->inputchannel + ebrhCLController0, OFF_SCAN_LINES );
    if( dto->active_ioobj == NULL )
    {
        StartTransfer( ioobj->controller_videoin, synchronize_medium_channel );
    }
    //register_dump( device );

    /*else
    {
        //ResetDMAController( dto->dma_controller_pci );
        write_results_to_rq( ioobj, get_jiffies_64(), cerrResultPacketAbort );
        run_next_iocb( dto->rw_queue );
        PRINTKM(DMA,(PKTD "request action unknown action code signal this packet jiffies %lu\n",ioobj->device_index, jiffies));
        complete_request( ioobj, 0 );
    }*/
}

//-------------------------------------------------------------------------------------------
void read_hrtcram_parameter( TPropertyElement* list, u32 property_count, u32 hrtc_command_count, u32* hrtc_ram, u32 buffer_size )
//-------------------------------------------------------------------------------------------
{
    TPropertyElement* prop_hrtc_ram;
    int i;

    memset( ( void* )hrtc_ram, 0, buffer_size );
    _SCAN_PROPERTY_LIST( list, prop_hrtc_ram, property_count, prHRTCRAMData );
    if( prop_hrtc_ram )
    {
        for( i = 0; i < hrtc_command_count && i < emvdHRTCTriggerCommand; i++ )
        {
            PRINTKM( MOD, ( PKTD " %s hrtcRAM[%d] %x\n", -1, __FUNCTION__, i, prop_hrtc_ram->u.intElement ) );
            hrtc_ram[i] = prop_hrtc_ram->u.intElement;
            ++prop_hrtc_ram;
        }
    }
}


//-------------------------------------------------------------------------------------------
void get_transfer_param( TPropertyElement* list, struct io_object* ioobj )
//-------------------------------------------------------------------------------------------
{
    unsigned long property_count = 0;
    unsigned char changed = FALSE;
    struct transfer_parameter* tp = &ioobj->transfer_param;
    unsigned long mux_ctrl_count = 0;
    TPropertyElement* pr_mux_ram;

    memset( tp, 0, sizeof( struct transfer_parameter ) );
    _READ_PROPERTYI( list, 1, prInfoPropertyCount, property_count, changed );
    _READ_PROPERTYI( list, property_count, prInfoRequestAction, tp->reqaction, changed );
    _READ_PROPERTYI( list, property_count, prDMATransferLength, tp->transferlength, changed );
    _READ_PROPERTYI( list, property_count, prInfoFooterOffset, tp->footeroffset, changed );
    _READ_PROPERTYI( list, property_count, prInfoFooterSize, tp->footersize, changed );
    _READ_PROPERTYI( list, property_count, prInfoRequestID, tp->reqid, changed );
    _READ_PROPERTYI( list, property_count, prInfoTimeout, tp->timeoutmsec, changed );
    _READ_PROPERTYI( list, property_count, prCnInput, tp->inputchannel, changed );
    _READ_PROPERTYI( list, property_count, prClXPos, tp->xstart, changed );
    _READ_PROPERTYI( list, property_count, prClWidth, tp->xstop, changed );
    _READ_PROPERTYI( list, property_count, prClYPos, tp->ystart, changed );
    _READ_PROPERTYI( list, property_count, prClHeight, tp->ystop, changed );
    _READ_PROPERTYI( list, property_count, prClScanMode, tp->scanmode, changed );
    _READ_PROPERTYI( list, property_count, prClDataValidEnable, tp->datavalidenable, changed );
    _READ_PROPERTYI( list, property_count, prDMAVideoInDirectTransfer, tp->videoin_direct_transfer, changed );
    _READ_PROPERTYI( list, property_count, prDMAVideoInWaitOfFrameEdge, tp->videoin_wait_of_frameedge, changed );
    _READ_PROPERTYI( list, property_count, prClMediumSource, tp->medium_mode_ctrl, changed );
    _READ_PROPERTYI( list, property_count, prCLEnableAOIMode, tp->enable_aoimode, changed );
    _READ_PROPERTYI( list, property_count, prCLExpandLineValid, tp->expand_lval, changed );
    _READ_PROPERTYI( list, property_count, prIsFrameStartMode, tp->frame_start.trigger_mode, changed );
    _READ_PROPERTYI( list, property_count, prIsFrameStartOverlap, tp->frame_start.trigger_overlap, changed );
    _READ_PROPERTYI( list, property_count, prIsFrameStopMode, tp->frame_stop.trigger_mode, changed );
    _READ_PROPERTYI( list, property_count, prInfoPropertiesChanged, tp->has_properties_changed, changed );
    if( tp->frame_start.trigger_mode )
    {
        tp->trigger_mode |= tp->frame_start.trigger_mode ? etsStartSignal : 0;
        tp->trigger_mode |= tp->frame_start.trigger_overlap ? etsStartSignalOverlap : 0;
        tp->trigger_mode |= tp->frame_stop.trigger_mode ? etsStopSignal : 0;
        //  tp->trigger_mode |= tp->FrameStop.TriggerMode ? etsStopSignalOverlap : 0;
    }
    _READ_PROPERTYI( list, property_count, prHRTCCommandCount, tp->trigger_hrtc_command_count, changed );
    read_hrtcram_parameter( list, property_count, tp->trigger_hrtc_command_count, tp->trigger_hrtc_ram, sizeof( u32 ) * emvdHRTCTriggerCommand );
    _READ_PROPERTYI( list, property_count, prCnMuxCtrlCount, mux_ctrl_count, changed );
    _SCAN_PROPERTY_LIST( list, pr_mux_ram, property_count, prCnMuxCtrlRamData );
    if( pr_mux_ram )
    {
        struct hyperion_device* device = ( struct hyperion_device* )ioobj->device;
        struct mux_controller_sequence mux_seq;
        int i;
        const int max_count_changes = 4;
        u32* mux_ram = mux_seq.muxdata;
        mux_seq.size = mux_ctrl_count * sizeof( u32 );
        for( i = 0; i < mux_ctrl_count; i++ )
        {
            mux_ram[i] = pr_mux_ram->u.intElement;
            ++pr_mux_ram;
        }
        for( i = 0; i < max_count_changes; i++ )
        {
            if( device->mux_seq.muxdata[i] != mux_seq.muxdata[i] )
            {
                memcpy( ( void* )device->mux_seq.muxdata, ( void* )mux_seq.muxdata, MUX_CONTROLLER_SEQUENCE_SIZE_BYTES * sizeof( u32 ) );
                device->mux_seq.size = mux_seq.size;
                device->mux_seq.changed = 1;
                break;
            }
        }
    }
    PRINTKM( MOD, ( PKTD "%s changed %d tf_len 0x%x, footeroff 0x%x, id 0x%x timeout 0x%x w %d h %d sm %d medium_mode_ctrl %d hrtc_cmd_cnt %d\n", ioobj->device_index, __FUNCTION__, tp->has_properties_changed, tp->transferlength, tp->footeroffset, tp->reqid, tp->timeoutmsec, tp->xstop - tp->xstart, tp->ystop - tp->ystart, tp->scanmode, tp->medium_mode_ctrl, tp->trigger_hrtc_command_count ) );
}

//-------------------------------------------------------------------------------------------
void set_device_specific( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    PRINTKM( MOD, ( PKTD " %s vendor 0x%x device 0x%x\n", device->index, __FUNCTION__, ( int )device->vd_id.vendorId, ( int )device->vd_id.deviceId ) );
    switch( device->vd_id.deviceId )
    {
    case PCI_DEVICE_ID_HYPERION_CL4E:
        device->set_mux_data = set_mux_data_64;
        device->set_hrtc_ram = set_hrtc_ram_64;
        break;
    default:
    case PCI_DEVICE_ID_HYPERION_CLE:
        device->set_mux_data = set_mux_data;
        device->set_hrtc_ram = set_hrtc_ram;
        break;
    }
}

//-------------------------------------------------------------------------------------------
int setup_read_write( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    int i, result = 0;
    u32 ramtopci_dmactrl_ver, vin_dmactrl_ver, system_ctrl_reg;
//  struct SBuffer *buffer = NULL;
//  unsigned long translation_table_size;

    PRINTKM( MOD, ( PKTD " setup_read_write( )\n", device->index ) );
    init_waitqueue_head( &device->result_queue );
    set_device_specific( device );

    ramtopci_dmactrl_ver = IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlPCI0, OFF_VIDEO_IN_CTRL_VERSION );
    vin_dmactrl_ver = IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_VERSION );

    PRINTKM( MOD, ( PKTD " found pcidmactrl version 0x%x videoin version 0x%x\n",  device->index, ramtopci_dmactrl_ver, vin_dmactrl_ver ) );
    device->hrtc_version = IO_READ_32( device->hyperion_base, device->reg_def, ebrhHrtController0, OFF_HRT_CONTROLLER_VER );
    system_ctrl_reg = IO_READ_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL );
    device->pci_hyperion_page_size = system_ctrl_reg & HYPERION_PCI_PAGE_SIZE_64K ? PAGE_SIZE_64K : PAGE_SIZE_4K;
    PRINTKM( MOD, ( PKTD " hyperion page_size %d kB sys_ctrl 0x%x\n",  device->index, device->pci_hyperion_page_size / KB, system_ctrl_reg ) );

    // initialize DMA_TRANSFER_OBJECT's
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        SET_BIT( REG_POINTER( device->hyperion_base, device->reg_def, i + ebrhCLController0, OFF_MUX_CONTROLLER ), AOI_ENABLE, TRUE );
        device->pdma_object[i]->avalon_to_pci_table.size = ( TRANSLATION_TABLE_ELEMENTS * device->pci_hyperion_page_size ) / MAX_PARALLEL_TRANSFER;
        device->pdma_object[i]->avalon_to_pci_table.base = ( u_char* )REG_POINTER( device->hyperion_base, device->reg_def, ebrhPCICore, OFF_AVALON_TRANSLATION_TABLE );     //PRINTKM(MOD,(PKTD " initialize dma_transfer_obj pcitranstable base p%p size 0x%x",  device->index, device->pdma_object[i]->avalon_to_pci_table.base, device->pdma_object[i]->avalon_to_pci_table.size ));
        device->pdma_object[i]->frame_counter = 0;
        set_queue_start_iocb( device->pqueues[i], start_transfer );
        device->pdma_object[i]->rw_queue = device->pqueues[i];
        device->pdma_object[i]->active_ioobj = NULL;
        device->pdma_object[i]->rewrite_transfer_parameter = TRUE;
        device->pdma_object[i]->init_done = FALSE;
        device->pdma_object[i]->abort_this_request = FALSE;
    }

    if( vin_dmactrl_ver != -1 && ramtopci_dmactrl_ver != -1 )
    {
#ifdef MEM_DMA_CONTROLLER
        // initialize DMA_TRANSFER_OBJECT's
        for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
        {
            device->pdma_object[i]->objid = ( UCHAR )i;
            device->pdma_object[i]->abort_all_transfer = FALSE;
            device->pdma_object[i]->timeout_occurred = FALSE;
            device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0] = ( DMA_CONTROLLER* )DMAController( NULL, ( PVOID )( PVOID )device->hyperion_base.base, i + ebrhDMACtrlVideoIn0, device->reg_def, NULL, cfdCommandFifo0 );
            device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo1] = ( DMA_CONTROLLER* )DMAController( NULL, ( PVOID )( PVOID )device->hyperion_base.base, i + ebrhDMACtrlVideoIn0, device->reg_def, NULL, cfdCommandFifo1 );
            device->pdma_object[i]->dma_controller_pci = ( DMA_CONTROLLER* )DMAController( NULL, ( PVOID )( PVOID )device->hyperion_base.base, i + ebrhDMACtrlPCI0, device->reg_def, ( PPREPARE_SCATTER_GATHER_LIST )prepare_scatter_gather_list, cfdCommandFifo0 );
            if( !device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0] || !device->pdma_object[i]->dma_controller_pci )
            {
                PRINTKM( MOD, ( PKTD " unable to allocate dmacontroller struct index %d", device->index, i ) );
                return -ENOMEM;
            }
            SetPCIPageSize( device->pdma_object[i]->dma_controller_pci, device->pci_hyperion_page_size );
            device->pdma_object[i]->init_done = TRUE;
        }
#endif
    }
    else if( vin_dmactrl_ver != -1 )
    {
        // initialize DMA_TRANSFER_OBJECT's for direct transfer
        for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
        {
            device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0] = ( DMA_CONTROLLER* )DMAController( NULL, ( PVOID )device->hyperion_base.base, i + ebrhDMACtrlVideoIn0, device->reg_def, ( PPREPARE_SCATTER_GATHER_LIST ) prepare_scatter_gather_list, cfdCommandFifo0 );
            device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo1] = ( DMA_CONTROLLER* )DMAController( NULL, ( PVOID )device->hyperion_base.base, i + ebrhDMACtrlVideoIn0, device->reg_def, ( PPREPARE_SCATTER_GATHER_LIST ) prepare_scatter_gather_list, cfdCommandFifo1 );
            device->pdma_object[i]->dma_controller_pci = NULL;
            if( !device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0] )
            {
                PRINTKM( MOD, ( PKTD " unable to allocate dmacontroller struct index %d", device->index, i ) );
                return -ENOMEM;
            }
            SetPCIPageSize( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0], device->pci_hyperion_page_size );
            SetPCIPageSize( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo1], device->pci_hyperion_page_size );
            {
                TRANSLATION_TABLE_DEF_T* ttdefs = device->pci_hyperion_page_size == PAGE_SIZE_4K ? &translation_table_defs[0][0] : &translation_table_defs_64K[0][0];
                SetDMAControllerBuffer( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0], ttdefs->address, ttdefs->size, FALSE );
                SetDMAControllerBuffer( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo1], ttdefs->address, ttdefs->size, FALSE );
            }
            device->pdma_object[i]->init_done = TRUE;
        }
    }
    _PIPE_INITIALIZE( device->dma_interrupt_result, DMA_RESULT_QUEUE_LEN );
    _PIPE_INITIALIZE( device->cleanup_request_pipe, DMA_RESULT_QUEUE_LEN );

    for( i = 0; i < UART_NUM; i++ )
    {
        create_serial( &device->hyperion_base, device->reg_def, &device->uart_port[i], i + ebrhUart0, device->index );
#if ENABLE_POCL
        CreatePoCLObject( &device->pocl[i], device->hyperion_base.base, device->reg_def, i + ebrhPoCLCtrl0 );
#endif
    }
    for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES; i++ )

    {

        device->mux_seq.muxdata[i] = 0;

    }

    device->mux_seq.changed = 0;

    device->mux_seq.size = 0;
    INIT_LIST_HEAD( &device->head_uaddr );
    PRINTKM( MOD, ( PKTD "setup_read_write( ) --> read_write module initialized\n", device->index ) );
    return result;
}

//-------------------------------------------------------------------------------------------
void release_read_write( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    u_long i;

    for( i = 0; i < UART_NUM; i++ )
    {
#if ENABLE_POCL
        ClosePoCL( &device->pocl[i] );
#endif
        close_serial( &device->uart_port[i] );
    }

    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        if( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0] )
        {
            DMAControllerDestruct( &device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0] );
        }
        if( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo1] )
        {
            DMAControllerDestruct( &device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo1] );
        }
        if( device->pdma_object[i]->dma_controller_pci )
        {
            DMAControllerDestruct( &device->pdma_object[i]->dma_controller_pci );
        }
    }
    _PIPE_DESTROY( device->dma_interrupt_result );
    _PIPE_DESTROY( device->cleanup_request_pipe );
}

//-------------------------------------------------------------------------------------------
// The following functions may be useful for a larger audience.
// copied from streaming tape device driver st.c
static int
sgl_map_user_pages( struct scatterlist *sgl, const unsigned int max_pages,
                    unsigned long uaddr, size_t count, int rw )
//-------------------------------------------------------------------------------------------
{
    unsigned long end = ( uaddr + count + PAGE_SIZE - 1 ) >> PAGE_SHIFT;
    unsigned long start = uaddr >> PAGE_SHIFT;
    const int nr_pages = end - start;
    int res, i, j;
    struct page **pages;

    PRINTKM( MEM,
             ( PKTD "sgl_map_user_pages() sgl p%p max_pages 0x%u uaddr 0x%lx "
                    "count 0x%zx end 0x%lx start 0x%lx nr_pages 0x%x\n",
               -1, sgl, max_pages, uaddr, count, end, start, nr_pages ) );
    /* User attempted Overflow! */
    if( ( uaddr + count ) < uaddr )
    {
        PRINTKM(
            FILE,
            ( PKTD "sgl_map_user_pages() (uaddr + count) < uaddr \n", -1 ) );
        return -EINVAL;
    }

    /* Too big */
    if( nr_pages > max_pages )
    {
        PRINTKM( FILE,
                 ( PKTD "sgl_map_user_pages() nr_pages > max_pages\n", -1 ) );
        return -ENOMEM;
    }

    /* Hmm? */
    if( count == 0 )
    {
        PRINTKM( FILE, ( PKTD "sgl_map_user_pages() count == 0\n", -1 ) );
        return 0;
    }

    if( ( pages = vmalloc( max_pages * sizeof( *pages ) ) ) == NULL )
    {
        PRINTKM( FILE,
                 ( PKTD "pages == NULL vmalloc returns NULL ( size 0x%zx )\n",
                   -1, ( max_pages * sizeof( *pages ) ) ) );
        return -ENOMEM;
    }

/* Try to fault in all of the necessary pages */
// PRINTKM(MEM,(PKTD "sgl_map_user_pages()
// down_read(&current->mm->mmap_sem)\n", -1 ));
#ifdef MMAP_LOCK_INITIALIZER
    mmap_read_lock( current->mm );
#else
    down_read( &current->mm->mmap_sem );
#endif
    /* rw==READ means read from drive, write into memory area */
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 6, 5, 0 )
    {
        int locked = 1;
        res = get_user_pages_remote( current->mm, uaddr, nr_pages,
                                     rw == READ ? FOLL_WRITE : 0, pages,
                                     &locked );
        if( locked )
        {
            mmap_read_unlock( current->mm );
        }
    }
#elif LINUX_VERSION_CODE >= KERNEL_VERSION( 5, 18, 0 )
    {
        int locked = 1;
        res = get_user_pages_remote( uaddr, nr_pages,
                                     rw == READ ? FOLL_WRITE : 0, pages, NULL,
                                     &locked );
        if( locked )
        {
            mmap_read_unlock( current->mm );
        }
    }

#elif LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 9, 0 )
    {
        int locked = 1;
        res = get_user_pages_locked(
            uaddr, nr_pages, rw == READ ? FOLL_WRITE : 0, pages, &locked );
        if( locked )
        {
#ifdef MMAP_LOCK_INITIALIZER
            mmap_read_unlock( current->mm );
#else
            up_read( &current->mm->mmap_sem );
#endif
        }
    }
#else
    res = get_user_pages( current, current->mm, uaddr, nr_pages, rw == READ, 0,
                          pages, NULL );
    up_read( &current->mm->mmap_sem );
#endif
    // PRINTKM(MEM,(PKTD "get_user_pages[_locked]() returns %d nr_pages %d \n",
    // -1, res, nr_pages));

    /* Errors and no page mapped should return here */
    if( res < nr_pages )
    {
        goto out_unmap;
    }

    for( i = 0; i < nr_pages; i++ )
    {
        // FIXME: flush superflous for rw==READ, probably wrong function for
        // rw==WRITE
        flush_dcache_page( pages[i] );
    }

    /* Populate the scatter/gather list */
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 24 )
    sg_init_table( sgl, nr_pages );
    sg_set_page( &sgl[0], pages[0], 0, ( uaddr & ~PAGE_MASK ) );
    sgl[0].length = PAGE_SIZE - sgl[0].offset;
#else
    sgl[0].page = pages[0];
    sgl[0].offset = uaddr & ~PAGE_MASK;
    sgl[0].dma_address = 0;
    sgl[0].length = PAGE_SIZE - sgl[0].offset;
#endif
    /*#if !defined(CONFIG_HIGHMEM64G) && (BITS_PER_LONG == 32)
        PRINTKM(MEM,(PKTD "sgl_map_user_pages() page %p __pa 0x%x off 0x%x len
    0x%x\n", -1, sgl[0].page, sgl[0].dma_address, sgl[0].offset,
    sgl[0].length)); #else PRINTKM(MEM,(PKTD "sgl_map_user_pages() page %p __pa
    0x%llx off 0x%x len 0x%x\n", -1, sgl[0].page, sgl[0].dma_address,
    sgl[0].offset, sgl[0].length)); #endif*/
    if( nr_pages > 1 )
    {
        sgl[0].length = PAGE_SIZE - sgl[0].offset;
        count -= sgl[0].length;
        for( i = 1; i < nr_pages; i++ )
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 24 )
            sg_set_page( &sgl[i], pages[i],
                         ( count < PAGE_SIZE ? count : PAGE_SIZE ), 0 );
#else
            sgl[i].offset = 0;
            sgl[i].page = pages[i];
            sgl[i].length = count < PAGE_SIZE ? count : PAGE_SIZE;
            sgl[i].dma_address = 0;
#endif
            count -= PAGE_SIZE;
            /*#if !defined(CONFIG_HIGHMEM64G) && (BITS_PER_LONG == 32)
                PRINTKM(MEM,(PKTD "sgl_map_user_pages() page[%d] %p __pa 0x%x
            off 0x%x len 0x%x\n", -1, i, sgl[i].page, sgl[i].dma_address,
            sgl[i].offset, sgl[i].length )); #else PRINTKM(MEM,(PKTD
            "sgl_map_user_pages() page[%d] %p __pa 0x%llx off 0x%x len 0x%x\n",
            -1, i, sgl[i].page, sgl[i].dma_address, sgl[i].offset,
            sgl[i].length )); #endif*/
        }
    }
    else
    {
        sgl[0].length = count;
    }

    vfree( pages );
    return nr_pages;

out_unmap:
    PRINTKM( FILE,
             ( PKTD
               "error: get_user_pages[_locked]() returns %d nr_pages %d\n",
               -1, res, nr_pages ) );
    if( res > 0 )
    {
        for( j = 0; j < res; j++ )
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 6, 0 )
            put_page( pages[j] );
#else
            page_cache_release( pages[j] );
#endif
        }
        res = 0;
    }
    vfree( pages );
    return res;
}

//-------------------------------------------------------------------------------------------
// And unmap them...
// copied from streaming tape device driver st.c
// TODO remove function calls from tasklet routine because of kunmap and call them from IOCTL_READRESULTPACKET
static int sgl_unmap_user_pages( struct scatterlist* sgl, const unsigned int nr_pages, int dirtied )
//-------------------------------------------------------------------------------------------
{
    int i;
    struct page* page;

    for( i = 0; i < nr_pages; i++ )
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 24 )
        page = sg_page( &sgl[i] );
#else
        page = sgl[i].page;
#endif
        //PRINTKM(MEM,(PKTD "sgl_unmap_user_pages() page[%d] %p, setdirty %d\n", -1, i, page, !PageReserved(page)));
        if( ! PageReserved( page ) )
        {
            SetPageDirty( page );
        }
        /* FIXME: cache flush missing for rw==READ
         * FIXME: call the correct reference counting function
         */
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 6, 0 )
        put_page( page );
#else
        page_cache_release( page );
#endif
    }
    return 0;
}

//-------------------------------------------------------------------------------------------
struct user_buffer_list* get_user_buffer_obj( struct hyperion_device* device, char __user* buf, size_t count )
//-------------------------------------------------------------------------------------------
{
    struct list_head* lh;
    struct user_buffer_list* ubuf_obj = NULL;

    for( lh = device->head_uaddr.next; ; lh = lh->next )
    {
        if( lh == &device->head_uaddr )
        {
            PRINTKM( MEM, ( PKTD " %s buf %p not found in ubuf_obj list add new\n", device->index, __FUNCTION__, buf ) );
            break;
        }
        if( list_entry( lh, struct user_buffer_list, list )->buf == buf )
        {
            ubuf_obj = list_entry( lh, struct user_buffer_list, list );
            PRINTKM( MEM, ( PKTD " %s sg_list found entry ubuf_obj %p buf %p\n", device->index, __FUNCTION__, ubuf_obj, buf ) );
            break;
        }
    }
    return ubuf_obj;
}


//-------------------------------------------------------------------------------------------
static int
setup_buffering( struct kiocb *iocb, char __user *buf, size_t count,
                 int is_read, struct user_buffer_list **ubuf_obj_result )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device *device
        = (struct hyperion_device *)iocb->ki_filp->private_data;
    int nr_pages;
    unsigned short use_sg;
    struct user_buffer_list *ubuf_obj = NULL;

    *ubuf_obj_result = 0;
    use_sg = count / PAGE_SIZE + 4;
    ubuf_obj = get_user_buffer_obj( device, buf, count );

    if( ubuf_obj == NULL )
    {
        ubuf_obj = vmalloc( sizeof( struct user_buffer_list ) );
        if( ubuf_obj == NULL )
        {
            return -ENOMEM;
        }
        memset( ubuf_obj, 0, sizeof( struct user_buffer_list ) );
        ubuf_obj->sg = vmalloc( use_sg * sizeof( struct scatterlist ) );
        if( ubuf_obj->sg == NULL )
        {
            PRINTKM(
                MEM,
                ( PKTD
                  " %s vmalloc for sglist failed use_sg %d sizeof sg %lx\n",
                  device->index, __FUNCTION__, use_sg,
                  (long unsigned int)sizeof( struct scatterlist ) ) );
            return -ENOMEM;
        }
        ubuf_obj->buf = buf;
        ubuf_obj->count = count;
        nr_pages
            = sgl_map_user_pages( ubuf_obj->sg, use_sg, (unsigned long)buf,
                                  count, ( is_read ? READ : WRITE ) );
        PRINTKM( MEM,
                 ( PKTD " %s sgl_map_user_pages > result %d ioobj->sg p%p\n",
                   device->index, __FUNCTION__, nr_pages, ubuf_obj->sg ) );
        if( nr_pages > 0 )
        {
            int i, dma_map_result = nr_pages, map_result;
            dma_addr_t addr;
            struct scatterlist *sgl = ubuf_obj->sg;
            for( i = 0; i < nr_pages; i++ )
            {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 24 )
                addr = dma_map_page( &device->pdev->dev, sg_page( &sgl[i] ), 0,
                                     PAGE_SIZE, DMA_FROM_DEVICE );
#else
                addr = pci_map_page( device->pdev, sgl[i].page, 0, PAGE_SIZE,
                                     PCI_DMA_FROMDEVICE );
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 27 )
                map_result = dma_mapping_error( &device->pdev->dev, addr );
#else
                map_result = pci_dma_mapping_error( addr );
#endif
                if( map_result != 0 )
                {
                    printk( " %s %d pci_map_page() failed res %d\n",
                            __FUNCTION__, __LINE__, map_result );
                    dma_map_result = -ENOMEM;
                    break;
                }
                sgl[i].dma_address = addr;
            }
            // if (STbp->do_dio)
            //{
            //   STp->nbr_dio++;
            //   STp->nbr_pages += STbp->do_dio;
            //   for (i=1; i < STbp->do_dio; i++)
            //       if (page_to_pfn(STbp->sg[i].page) ==
            //       page_to_pfn(STbp->sg[i-1].page) + 1)
            //           STp->nbr_combinable++;
            // }
            if( dma_map_result <= 0 )
            {
                printk( " %s %d dma_map_sg() failed result %d index[%d]\n",
                        __FUNCTION__, __LINE__, dma_map_result, i );
                vfree( ubuf_obj->sg );
                vfree( ubuf_obj );
                return 0;
            }
            ubuf_obj->nr_pages = nr_pages;
#ifndef REMOVE_REQUEST_BUFFER_MAPPING
            list_add_tail( &ubuf_obj->list, &device->head_uaddr );
#endif
            *ubuf_obj_result = ubuf_obj;
            return nr_pages;
        }
        vfree( ubuf_obj );
        return 0;
    }
    *ubuf_obj_result = ubuf_obj;
    return ubuf_obj->nr_pages;
}

//-------------------------------------------------------------------------------------------
static void prepare_controller_buffer( struct hyperion_device* device, struct io_object* ioobj )
{
    TRANSLATION_TABLE_DEF_T* ttdefs, *ttdefs_trig;
    struct dma_transfer_object* dto = ( struct dma_transfer_object* )ioobj->dma_transfer_object;

    if( device->pci_hyperion_page_size == PAGE_SIZE_64K )
    {
        ttdefs = &translation_table_defs_64K[ioobj->transfer_param.inputchannel][ioobj->transfer_param.medium_mode_ctrl];
        ttdefs_trig = &translation_table_defs_trigger_64K[ioobj->transfer_param.inputchannel][ioobj->transfer_param.medium_mode_ctrl];
    }
    else
    {
        ttdefs = &translation_table_defs[ioobj->transfer_param.inputchannel][ioobj->transfer_param.medium_mode_ctrl];
        ttdefs_trig = &translation_table_defs_trigger[ioobj->transfer_param.inputchannel][ioobj->transfer_param.medium_mode_ctrl];
    }
    if( ioobj->transfer_param.trigger_mode )
    {
        SetDMAControllerBuffer( dto->dma_controller_videoin[cfdCommandFifo0], ttdefs_trig->address, ttdefs_trig->size, FALSE );
        SetDMAControllerBuffer( dto->dma_controller_videoin[cfdCommandFifo1], ttdefs_trig->address + ttdefs_trig->size, ttdefs_trig->size, FALSE );
    }
    else
    {
        SetDMAControllerBuffer( dto->dma_controller_videoin[cfdCommandFifo0], ttdefs->address, ttdefs->size, FALSE );
        SetDMAControllerBuffer( dto->dma_controller_videoin[cfdCommandFifo1], ttdefs->address, ttdefs->size, FALSE );
    }
}

//-------------------------------------------------------------------------------------------
static void check_medium_channels( struct hyperion_device* device, struct io_object* ioobj )
//-------------------------------------------------------------------------------------------
{
    struct transfer_parameter* tp = &ioobj->transfer_param;
    u32 mux_ctrl, i = 0;
    if( tp->medium_mode_ctrl )
    {
        int delay_j = msecs_to_jiffies( 10 );
        do
        {
            mux_ctrl = IO_READ_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER );
            if( mux_ctrl & PHASE_ERROR_DETECT )
            {
                mux_ctrl |= RESET_MEDIUM_CHANNELS;
                IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER, mux_ctrl );
                mux_ctrl &= ~RESET_MEDIUM_CHANNELS;
                IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER, mux_ctrl );
            }
            else
            {
                break;
            }
            ++i;
            wait_jiffies( delay_j );
        }
        while( i < 10 );
    }
}

//-------------------------------------------------------------------------------------------
static ssize_t async_read_write( struct kiocb* iocb, char __user* buffer, size_t count, loff_t pos, int is_read )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device = ( struct hyperion_device* )iocb->ki_filp->private_data;
    struct io_object* ioobj = NULL;
    struct user_buffer_list* ubuf_obj = NULL;
    int result = -EINVAL, prop_changed = FALSE;
    unsigned long property_count = 0, inputchannel = 0, request_id = 0, request_action = 0;
    TPropertyElement prop_count, *list = NULL;

    //PRINTKM(DMA,(PKTD " %s %lu ubuffer %p count %d is_read %d is_synch_io %d\n", device->index, __FUNCTION__, jiffies, buffer, (int)count, is_read, is_sync_kiocb( iocb ) ) );

    result = setup_buffering( iocb, buffer, count, is_read, &ubuf_obj );
    if( result <= 0 )
    {
        PRINTKM( FILE, ( PKTD "setup_buffering() failed result %d\n", device->index, result ) );
        result = -EFAULT;
        goto err_out;
    }
    if( ubuf_obj->private == NULL )
    {
        ioobj = kmalloc( sizeof( struct io_object ), GFP_KERNEL );
        if( ioobj == NULL )
        {
            printk( " %s error kmalloc try calloc for io_object\n", __FUNCTION__ );
            return -ENOMEM;
        }
        memset( ioobj, 0, sizeof( struct io_object ) );
        ubuf_obj->private = ( void* )ioobj;
    }
    else
    {
        ioobj = ( struct io_object* )ubuf_obj->private;
    }
    iocb->private = ( void* )ioobj;
    ioobj->device = ( void* )device;
    ioobj->device_index = device->index;
    ioobj->iocb = iocb;
    ioobj->buffer = buffer;
    ioobj->count = count;
    ioobj->pos = pos;
    ioobj->ubuf_obj = ( void* )ubuf_obj;
    ioobj->sg = ubuf_obj->sg;
    ioobj->do_dio = ubuf_obj->nr_pages;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
    iocb->ki_cancel = NULL;
#else
    kiocb_set_cancel_fn( iocb, NULL );
#endif

    if( copy_from_user( ( void* )&prop_count, buffer, sizeof( TPropertyElement ) ) )
    {
        PRINTKM( FILE, ( PKTD "copy_from_user() failed\n", device->index ) );
        result = -ENOMEM;
        goto err_out;
    }
    property_count = prop_count.u.intElement;
    list = ( TPropertyElement* )vmalloc( property_count * sizeof( TPropertyElement ) );
    if( list == NULL )
    {
        PRINTKM( FILE, ( PKTD "vmalloc() failed propertycount %d, memsize req 0x%zx, propid %d\n", device->index, prop_count.u.intElement, prop_count.u.intElement * sizeof( TPropertyElement ), prop_count.PropertyID ) );
        result = -ENOMEM;
        goto err_out;
    }
    if( copy_from_user( ( void* )list, buffer, property_count * sizeof( TPropertyElement ) ) )
    {
        PRINTKM( FILE, ( PKTD "copy_from_user() failed\n", device->index ) );
        goto err_out;
    }

    _READ_PROPERTYI( list, property_count, prCnInput, inputchannel, prop_changed );
    _READ_PROPERTYI( list, property_count, prInfoRequestID, request_id, prop_changed );
    _READ_PROPERTYI( list, property_count, prInfoRequestAction, request_action, prop_changed );
    //PRINTKM(IO,(PKTD "aio_read_write() dma_ch %lu found PROP_START_CODE propCount %lu reqid %lu\n", device->index, inputchannel, property_count, request_id ));
    if( inputchannel  >= MAX_PARALLEL_TRANSFER )
    {
        inputchannel = 0;
    }

    if( request_action == raiSnapRequest && device->pdma_object[inputchannel]->init_done )
    {
        struct dma_transfer_object* dto = device->pdma_object[inputchannel];
        volatile struct io_object* ioobj_ch;
        //iocb->ki_cancel = cancel_async_read_write;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
        iocb->ki_cancel = NULL;
#else
        kiocb_set_cancel_fn( iocb, NULL );
#endif
        ioobj->dma_transfer_object = ( void* )dto;
        get_transfer_param( list, ioobj );
        vfree( list );
        list = NULL;
        ioobj->start_jiffies = jiffies;
        if( ioobj->transfer_param.trigger_mode > 0 )
        {
            ioobj->controller_videoin = dto->dma_controller_videoin[ioobj->transfer_param.reqid & cfdCommandFifo1];
        }
        else
        {
            ioobj->controller_videoin = dto->dma_controller_videoin[cfdCommandFifo0];
        }
        ioobj_ch = _GET_ACTIVE_IOOBJ( dto );
        if( ioobj_ch == NULL )
        {
            //printk( " %s %lu queue empty no active ioobj\n", __FUNCTION__, jiffies );
            prepare_controller_buffer( device, ioobj );
        }
        check_medium_channels( device, ioobj );

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
        timer_setup( &ioobj->timer_iocb, jit_timer_cancel_request, 0 );
#else
        init_timer( &ioobj->timer_iocb );
#endif
        run_iocb( device->pqueues[inputchannel ], ioobj ); //run_iocb start with correct start_iocb routine
    }
    else
    {
        result = -EFAULT;
        goto err_out;
    }
    //PRINTKM(DMA,(PKTD " %s %lu msec aio_read_write() --> -EIOCBQUEUED ubuffer %p\n", device->index, __FUNCTION__, jiffies, buffer) );
    return -EIOCBQUEUED;
err_out:
    PRINTKM( FILE, ( PKTD "%s() failed result %d\n", device->index, __FUNCTION__, result ) );
    if( ioobj )
    {
        kfree( ioobj );
    }
    if( list )
    {
        vfree( list );
    }
    return result;

}

#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,19)
//-------------------------------------------------------------------------------------------
ssize_t async_io_read( struct kiocb* iocb, char __user* buffer,  size_t count, loff_t pos )
//-------------------------------------------------------------------------------------------
{
    //printk( " %s \n", __FUNCTION__ );
    return async_read_write( iocb, buffer, count, pos, TRUE );
}

//-------------------------------------------------------------------------------------------
ssize_t async_io_write( struct kiocb* iocb, const char __user* buffer, size_t count, loff_t pos )
//-------------------------------------------------------------------------------------------
{
//  return async_read_write( iocb, buffer, count, pos, FALSE );
    return 0;
}
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
//-------------------------------------------------------------------------------------------
ssize_t async_io_read( struct kiocb* iocb, const struct iovec* iocv, unsigned long count, loff_t pos )
//-------------------------------------------------------------------------------------------
{
    //  printk( " %s iov_base p%p iov_len %d count %d pos %d\n", __FUNCTION__, iocv->iov_base, (int)iocv->iov_len, (int)count, pos );
    return async_read_write( iocb, ( char __user* )iocv->iov_base, iocv->iov_len, pos, TRUE );
}

//-------------------------------------------------------------------------------------------
ssize_t async_io_write( struct kiocb* iocb, const struct iovec* iocv, unsigned long count, loff_t pos )
//-------------------------------------------------------------------------------------------
{
    //  return async_read_write( iocb, (char __user *)iocv->iov_base, count, pos, FALSE );
    return 0;
}
#else
//-------------------------------------------------------------------------------------------
ssize_t
async_io_read( struct kiocb *iocb, struct iov_iter *iocv_iter )
//-------------------------------------------------------------------------------------------
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 6, 4, 0 )
    const struct iovec *iov = iter_iov( iocv_iter );
#else
    const struct iovec *iov = iocv_iter->iov;
#endif

    //  printk( " %s iov_base p%p iov_len %d count %d pos %d\n", __FUNCTION__,
    //  iocv->iov_base, (int)iocv->iov_len, (int)count, pos );
    return async_read_write( iocb, iov->iov_base, iov->iov_len,
                             iocv_iter->iov_offset, TRUE );
}

//-------------------------------------------------------------------------------------------
ssize_t async_io_write( struct kiocb* iocb, struct iov_iter* iocv_iter )
//-------------------------------------------------------------------------------------------
{
    //  return async_read_write( iocb, (char __user *)iocv->iov_base, count, pos, FALSE );
    return 0;
}
#endif

//-------------------------------------------------------------------------------------------
unsigned int hyperion_poll( struct file* filp, struct poll_table_struct* wait )
//----------------/usr/include/X11/Xtrans/Xtrans.h:---------------------------------------------------------------------------
{
    struct hyperion_device* device = filp->private_data;
    unsigned int mask = 0;

    //  int left = (dev->rp + dev->buffersize - dev->wp) % dev->buffersize;

    poll_wait( filp, &device->result_queue, wait );
    //if (dev->rp != dev->wp) mask |= POLLIN | POLLRDNORM; /* lesbar */
    //if (left != 1)     mask |= POLLOUT | POLLWRNORM; /* schreibbar */

    mask = !is_result_queue_empty( &device->result_queue );
    return mask;
}


//-------------------------------------------------------------------------------------------
void abort_transfer( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    DECLARE_WAIT_QUEUE_HEAD( wq );
    u32 ie;
    int i = 0;
    struct dma_transfer_object* dto;
    const int timeout_msec = 5000;

    //PRINTKM(DMA,(PKTD " +%s %lu\n", device->index, __FUNCTION__, jiffies));
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        dto = device->pdma_object[i];
        if( dto->init_done && get_current_iocb( dto->rw_queue ) )
        {
            dto->abort_all_transfer = TRUE;
            init_completion( &device->compl_abort );
            ie = ioread32( ( void __iomem* )( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0]->dma_reg.interrupt_enable ) );
            ie |= PSEUDO_INTR;
            iowrite32( ie, ( void __iomem* )( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0]->dma_reg.interrupt_enable ) );
            wait_for_completion_timeout( &device->compl_abort, msecs_to_jiffies( timeout_msec ) );
        }
    }
    //PRINTKM(DMA,(PKTD " -%s %lu\n", device->index, __FUNCTION__, jiffies));
}//abort_transfer
