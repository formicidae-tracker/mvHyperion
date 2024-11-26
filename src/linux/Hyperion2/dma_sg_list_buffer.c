/*
 * dma_sg_list_buffer.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion
 series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: dma_sg_list_buffer.c,v 1.10 2010-06-23 14:52:23 ug Exp $
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

#include "clf_func.h"
#include "drivermain.h"
#include "linux/gfp.h"

//----------------------------------------------------------------------------------------------
int
alloc_contiguous_buffer( struct pci_dev *pdev, int size, void **buffer,
                         dma_addr_t *phys_addr, int direction )
//----------------------------------------------------------------------------------------------
{
    *buffer = dma_alloc_coherent( &pdev->dev, size, phys_addr, GFP_KERNEL );
    if( *buffer == NULL )
    {
        printk( KERN_ERR
                "hyperion2 %s couldn't allocate sg_buffer_pool size %d\n",
                __FUNCTION__, size );
        return -ENOMEM;
    }
    memset( *buffer, 0, size );
    return size;
}

//----------------------------------------------------------------------------------------------
void free_contiguous_buffer( struct pci_dev* pdev, int size, void* buffer, dma_addr_t phys_addr, int direction )
//----------------------------------------------------------------------------------------------
{
    dma_free_coherent( &pdev->dev, size, buffer, phys_addr );
}

//----------------------------------------------------------------------------------------------
int
init_sg_buffer_list( struct hyperion *phyperion )
//----------------------------------------------------------------------------------------------
{
    struct hyperion_device *phyp_dev
        = (struct hyperion_device *)phyperion->device;
    int i, result = -ENOMEM;

    INIT_LIST_HEAD( &phyp_dev->dma_sg_list.head );
    spin_lock_init( &phyp_dev->dma_sg_list.s_lock );

    for( i = 0; i < MAX_COUNT_REQUEST_OBJECT; i++ )
    {
        phyp_dev->dma_sg_list_pool[i].index = i;
        phyp_dev->dma_sg_list_pool[i].buf_size = ( 512 * 1024 );
        list_add_tail( &phyp_dev->dma_sg_list_pool[i].list_entry,
                       &phyp_dev->dma_sg_list.head );
        // alloc buffer for sg_dma_address
        result = alloc_contiguous_buffer(
            phyperion->pdev, phyp_dev->dma_sg_list_pool[i].buf_size,
            &phyp_dev->dma_sg_list_pool[i].buf,
            &phyp_dev->dma_sg_list_pool[i].phy, DMA_TO_DEVICE );
        if( result < 0 )
        {
            return result;
        }
        else
        {
            phyp_dev->dma_sg_list_pool[i].size = result;
        }
        // alloc buffer for communication host - hyperion processor
        result = alloc_contiguous_buffer(
            phyperion->pdev, PAGE_SIZE,
            &phyp_dev->dma_sg_list_pool[i].nios_msg,
            &phyp_dev->dma_sg_list_pool[i].nios_msg_phy, DMA_TO_DEVICE );
        if( result < 0 )
        {
            return result;
        }
        else
        {
            phyp_dev->dma_sg_list_pool[i].nios_msg_size = result;
        }

        // alloc buffer for communication hyperion processor - host
        result = alloc_contiguous_buffer(
            phyperion->pdev, PAGE_SIZE,
            &phyp_dev->dma_sg_list_pool[i].host_msg,
            &phyp_dev->dma_sg_list_pool[i].host_msg_phy, DMA_FROM_DEVICE );
        if( result < 0 )
        {
            return result;
        }
        else
        {
            phyp_dev->dma_sg_list_pool[i].host_msg_size = result;
        }
        // alloc buffer for request_extension
        result = alloc_contiguous_buffer(
            phyperion->pdev, PAGE_SIZE,
            &phyp_dev->dma_sg_list_pool[i].request_extension,
            &phyp_dev->dma_sg_list_pool[i].req_ext_phy, DMA_BIDIRECTIONAL );
        if( result < 0 )
        {
            return result;
        }

        if( !dma_set_mask( &phyperion->pdev->dev, DMA_BIT_MASK( 64 ) ) )
        {
            const u64 highPart = DMA_BIT_MASK( 32 ) << 32;
            phyp_dev->dma_sg_list_pool[i].phy
                |= ( phyp_dev->dma_sg_list_pool[i].phy & highPart )
                       ? phyp_dev->address_space_encoding
                       : 0;
            phyp_dev->dma_sg_list_pool[i].nios_msg_phy
                |= ( phyp_dev->dma_sg_list_pool[i].nios_msg_phy & highPart )
                       ? phyp_dev->address_space_encoding
                       : 0;
            phyp_dev->dma_sg_list_pool[i].host_msg_phy
                |= ( phyp_dev->dma_sg_list_pool[i].host_msg_phy & highPart )
                       ? phyp_dev->address_space_encoding
                       : 0;
            phyp_dev->dma_sg_list_pool[i].req_ext_phy
                |= ( phyp_dev->dma_sg_list_pool[i].req_ext_phy & highPart )
                       ? phyp_dev->address_space_encoding
                       : 0;
        }

        PRINTKM(
            DMA,
            ( PKTD ": - %s - "
                   "[%d] nios_msg %p phy %p host_msg %p phy %p "
                   "sg_list_dma_addr %p phy %p\n",
              phyperion->number, __FUNCTION__, i,
              phyp_dev->dma_sg_list_pool[i].nios_msg,
              (void *)( (UINT_PTR)phyp_dev->dma_sg_list_pool[i].nios_msg_phy ),
              phyp_dev->dma_sg_list_pool[i].host_msg,
              (void *)( (UINT_PTR)phyp_dev->dma_sg_list_pool[i].host_msg_phy ),
              phyp_dev->dma_sg_list_pool[i].buf,
              (void *)( (UINT_PTR)phyp_dev->dma_sg_list_pool[i].phy ) ) );
    }
    return 0;
}

//----------------------------------------------------------------------------------------------
void
release_sg_buffer_list( struct hyperion *phyperion )
//----------------------------------------------------------------------------------------------
{
    struct hyperion_device *phyp_dev
        = (struct hyperion_device *)phyperion->device;
    int i;
    for( i = 0; i < MAX_COUNT_REQUEST_OBJECT; i++ )
    {
        list_del( &phyp_dev->dma_sg_list_pool[i].list_entry );
        free_contiguous_buffer(
            phyperion->pdev, phyp_dev->dma_sg_list_pool[i].buf_size,
            phyp_dev->dma_sg_list_pool[i].buf,
            phyp_dev->dma_sg_list_pool[i].phy, DMA_FROM_DEVICE );

        free_contiguous_buffer(
            phyperion->pdev, PAGE_SIZE, phyp_dev->dma_sg_list_pool[i].nios_msg,
            phyp_dev->dma_sg_list_pool[i].nios_msg_phy, DMA_FROM_DEVICE );

        free_contiguous_buffer(
            phyperion->pdev, PAGE_SIZE, phyp_dev->dma_sg_list_pool[i].host_msg,
            phyp_dev->dma_sg_list_pool[i].host_msg_phy, DMA_FROM_DEVICE );

        free_contiguous_buffer(
            phyperion->pdev, PAGE_SIZE,
            phyp_dev->dma_sg_list_pool[i].request_extension,
            phyp_dev->dma_sg_list_pool[i].req_ext_phy, DMA_FROM_DEVICE );
    }
}

//----------------------------------------------------------------------------------------------
struct dma_sg_list_entry* get_sg_list_buffer( struct hyperion_device* phyp_dev )
//----------------------------------------------------------------------------------------------
{
    unsigned long irqflags;
    struct list_head* next;
    struct dma_sg_list_entry* dma_sg_list = NULL;

    spin_lock_irqsave( &phyp_dev->dma_sg_list.s_lock, irqflags );
    if( !list_empty( &phyp_dev->dma_sg_list.head ) )
    {
        next = remove_head_list( &phyp_dev->dma_sg_list.head );
        dma_sg_list = container_of( next, struct dma_sg_list_entry, list_entry );
    }
    else
    {
        dma_sg_list = NULL;
    }
    spin_unlock_irqrestore( &phyp_dev->dma_sg_list.s_lock, irqflags );
    return dma_sg_list;
}

//----------------------------------------------------------------------------------------------
void free_sg_list_buffer( struct hyperion_device* phyp_dev, struct dma_sg_list_entry* dma_sg_list )
//----------------------------------------------------------------------------------------------
{
    unsigned long irqflags;

    if( dma_sg_list != NULL )
    {
        dma_sg_list->size = 0;
        spin_lock_irqsave( &phyp_dev->dma_sg_list.s_lock, irqflags );
        list_add_tail( &dma_sg_list->list_entry, &phyp_dev->dma_sg_list.head );
        spin_unlock_irqrestore( &phyp_dev->dma_sg_list.s_lock, irqflags );
    }
}

//----------------------------------------------------------------------------------------------
void dump_sg_list_buffer( struct hyperion_device* phyp_dev )
//----------------------------------------------------------------------------------------------
{
    int i;
    unsigned int* pDump = 0;

    PRINTKM( DMA, ( PKTD ": - %s - \n", phyp_dev->number, __FUNCTION__ ) );
    for( i = 0; i < MAX_COUNT_REQUEST_OBJECT; i++ )
    {
        pDump = phyp_dev->dma_sg_list_pool[i].host_msg;
        PRINTKM( DMA, ( PKTD "    host_msg buffer %p: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", phyp_dev->number, pDump, pDump[0], pDump[1], pDump[4], pDump[5], pDump[8], pDump[9], pDump[12], pDump[13], pDump[16], pDump[17], pDump[20], pDump[21], pDump[24], pDump[25] ) );
    }
}
