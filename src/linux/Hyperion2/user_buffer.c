/*
 * clf_func.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: user_buffer.c,v 1.9 2009-08-20 06:59:15 ug Exp $
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

#include "drivermain.h"
#include "hyperion_base.h"

//-------------------------------------------------------------------------------------------
void free_user_buffer( struct hyperion* phyperion, struct user_buffer_descriptor* puser_buffer_descr )
//-------------------------------------------------------------------------------------------
{
    if( puser_buffer_descr != NULL )
    {
        remove_user_buffer_descriptor_from_list( phyperion, puser_buffer_descr );
        if( puser_buffer_descr->sg != NULL )
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 )
            dma_unmap_sg_attrs( &phyperion->pdev->dev, puser_buffer_descr->sg, puser_buffer_descr->nr_pages, DMA_FROM_DEVICE, ( long unsigned int )phyperion->pdma_attrs );
#elif LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
            dma_unmap_sg_attrs( &phyperion->pdev->dev, puser_buffer_descr->sg, puser_buffer_descr->nr_pages, DMA_FROM_DEVICE, ( struct dma_attrs* )phyperion->pdma_attrs );
#else
            dma_unmap_sg( &phyperion->pdev->dev, puser_buffer_descr->sg, puser_buffer_descr->nr_pages, DMA_FROM_DEVICE );
#endif
            sgl_unmap_user_pages( puser_buffer_descr->sg, puser_buffer_descr->nr_pages, 0 );
            vfree( puser_buffer_descr->sg );
        }
    }
}

//-------------------------------------------------------------------------------------------
void free_user_buffer_descriptor( struct hyperion* phyperion, char __user* puser_buffer, size_t count )
//-------------------------------------------------------------------------------------------
{
    struct user_buffer_descriptor* puser_buffer_descr;
    puser_buffer_descr = get_user_buffer_descriptor( phyperion, puser_buffer, count );
    PRINTKM( DMA, ( PKTD "%s ubuf p%p obj p%p\n", phyperion->number, __FUNCTION__, puser_buffer, puser_buffer_descr ) );
    free_user_buffer( phyperion, puser_buffer_descr );
}

//-------------------------------------------------------------------------------------------
void free_all_user_buffer_descriptors( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    struct list_head* lh = phyperion->head_uaddr.next;
    struct user_buffer_descriptor* puser_buffer_descr = NULL;
    unsigned long irqflags;

    while( lh != &phyperion->head_uaddr )
    {
        spin_lock_irqsave( &phyperion->s_lock, irqflags );
        puser_buffer_descr = list_entry( lh, struct user_buffer_descriptor, list );
        lh = lh->next;
        spin_unlock_irqrestore( &phyperion->s_lock, irqflags );
        free_user_buffer( phyperion, puser_buffer_descr );
    }
    INIT_LIST_HEAD( &phyperion->head_uaddr );
}

//-------------------------------------------------------------------------------------------
struct user_buffer_descriptor* get_user_buffer_descriptor( struct hyperion* phyperion, char __user* puser_buffer, size_t count )
//-------------------------------------------------------------------------------------------
{
    struct list_head* lh;
    struct user_buffer_descriptor* puser_buffer_descr = NULL;
    unsigned long irqflags;

    spin_lock_irqsave( &phyperion->s_lock, irqflags );
    for( lh = phyperion->head_uaddr.next;; lh = lh->next )
    {
        if( lh == &phyperion->head_uaddr )
        {
            PRINTKM( MEM, ( PKTD " %s buf %p not found in puser_buffer_descr list add new\n", phyperion->number, __FUNCTION__, puser_buffer ) );
            break;
        }
        if( list_entry( lh, struct user_buffer_descriptor, list )->puser_buffer == puser_buffer )
        {
            puser_buffer_descr = list_entry( lh, struct user_buffer_descriptor, list );
            PRINTKM( MEM, ( PKTD " %s sg_list found entry puser_buffer_descr %p user_buffer %p\n", phyperion->number, __FUNCTION__, puser_buffer_descr, puser_buffer ) );
            break;
        }
    }
    spin_unlock_irqrestore( &phyperion->s_lock, irqflags );
    return puser_buffer_descr;
}

//-------------------------------------------------------------------------------------------
void add_user_buffer_descriptor_to_tail( struct hyperion* phyperion, struct user_buffer_descriptor* puser_buffer_descr )
//-------------------------------------------------------------------------------------------
{
    unsigned long irqflags = 0;
    spin_lock_irqsave( &phyperion->s_lock, irqflags );
    list_add_tail( &puser_buffer_descr->list, &phyperion->head_uaddr );
    spin_unlock_irqrestore( &phyperion->s_lock, irqflags );
}

//-------------------------------------------------------------------------------------------
void remove_user_buffer_descriptor_from_list( struct hyperion* phyperion, struct user_buffer_descriptor* puser_buffer_descr )
//-------------------------------------------------------------------------------------------
{
    unsigned long irqflags = 0;
    spin_lock_irqsave( &phyperion->s_lock, irqflags );
    list_del_init( &( puser_buffer_descr->list ) );
    spin_unlock_irqrestore( &phyperion->s_lock, irqflags );
}
