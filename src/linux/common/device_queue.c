/*
 * device_queue.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: device_queue.c,v 1.1 2009-02-17 14:25:43 ug Exp $
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

#include "device_queue.h"

//-------------------------------------------------------------------------------------------
void initialize_queue( pdevice_queue_t pqueue, prequest_start_iocb start_iocb, int index )
//-------------------------------------------------------------------------------------------
{
    pqueue->index = index;
    INIT_LIST_HEAD( &pqueue->head );
    spin_lock_init( &pqueue->s_lock );
    pqueue->start_iocb = start_iocb;
    pqueue->current_ioobj = NULL;
}

//-------------------------------------------------------------------------------------------
int is_queue_empty( pdevice_queue_t pqueue )
//-------------------------------------------------------------------------------------------
{
    return list_empty( &pqueue->head );
}

//-------------------------------------------------------------------------------------------
void set_queue_start_iocb( pdevice_queue_t pqueue, prequest_start_iocb start_iocb )
//-------------------------------------------------------------------------------------------
{
    pqueue->start_iocb = start_iocb;
}


//-------------------------------------------------------------------------------------------
void cancel_iocb( pdevice_queue_t pqueue, struct kiocb* iocb )
//-------------------------------------------------------------------------------------------
{
    //unsigned long irq_flags;
    struct hyperion_request_packet* phyperion_request_packet;
    unsigned long irqflags;

    spin_lock_irqsave( &pqueue->s_lock, irqflags );
    phyperion_request_packet = ( struct hyperion_request_packet* )iocb->private;
    iocb->private = NULL;
    remove_entry( &phyperion_request_packet->list_entry );
    ///< todo sgl_unmap_user_pages
    kfree( phyperion_request_packet );
    /// \todo set status canceled ?
    spin_unlock_irqrestore( &pqueue->s_lock, irqflags );
}


//-------------------------------------------------------------------------------------------
void remove_and_notify_iocbs( pdevice_queue_t pqueue )
//-------------------------------------------------------------------------------------------
{
    struct list_head cancel_list;
    struct list_head* current_list_entry, *first, *next;
    struct hyperion_request_packet* phyperion_request_packet;
    unsigned long irqflags;

    PRINTKM( IO, ( PKTD "+%s \n", -1, __FUNCTION__ ) );
    INIT_LIST_HEAD( &cancel_list );
    spin_lock_irqsave( &pqueue->s_lock, irqflags );
    first = &pqueue->head;
    for( next = first->next; next != first; )
    {
        //ioobj = container_of( next, struct hyperion_request_packet, list_entry);
        current_list_entry = next;
        next = next->next;
        //set cancel routine to null
        list_del( current_list_entry );
        list_add_tail( current_list_entry, &cancel_list );
    }
    pqueue->current_ioobj = NULL;
    spin_unlock_irqrestore( &pqueue->s_lock, irqflags );
    while( !list_empty( &cancel_list ) )
    {
        next = remove_head_list( &cancel_list );
        phyperion_request_packet = container_of( next, struct hyperion_request_packet, list_entry );
        complete_request( phyperion_request_packet, 0 );
    }
}


//-------------------------------------------------------------------------------------------
struct hyperion_request_packet* run_next_iocb( pdevice_queue_t pqueue )
//-------------------------------------------------------------------------------------------
{
    //unsigned long irq_flags;
    struct list_head* next;
    volatile struct hyperion_request_packet* phyperion_request_packet, *current_ioobj;
    unsigned long irqflags;

    spin_lock_irqsave( &pqueue->s_lock, irqflags );
    //FIXME nullify current_iocb with an atomic operation
    current_ioobj = pqueue->current_ioobj;
    pqueue->current_ioobj = NULL;

    while( /*handle queue cancel || */ !list_empty( &pqueue->head ) )
    {
        next = remove_head_list( &pqueue->head );
        phyperion_request_packet = container_of( next, struct hyperion_request_packet, list_entry );

        /*if( cancel queue )
        {
        ...
        continue;
        }
        else
        */
        pqueue->current_ioobj = phyperion_request_packet;
        spin_unlock_irqrestore( &pqueue->s_lock, irqflags );
        pqueue->start_iocb( ( struct hyperion_request_packet* )phyperion_request_packet, pqueue->index );
        return ( struct hyperion_request_packet* )current_ioobj;

    }
    spin_unlock_irqrestore( &pqueue->s_lock, irqflags );
    return ( struct hyperion_request_packet* )current_ioobj;
}


//-------------------------------------------------------------------------------------------
void restart_current_iocb( pdevice_queue_t pqueue )
//-------------------------------------------------------------------------------------------
{
    unsigned long irqflags;
    spin_lock_irqsave( &pqueue->s_lock, irqflags );
    if( pqueue->current_ioobj )
    {
        pqueue->start_iocb( ( struct hyperion_request_packet* )pqueue->current_ioobj, pqueue->index );
    }
    spin_unlock_irqrestore( &pqueue->s_lock, irqflags );
}


//-------------------------------------------------------------------------------------------
int run_iocb( pdevice_queue_t pqueue, struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    unsigned long irqflags;
    //unsigned long irq_flags;
    spin_lock_irqsave( &pqueue->s_lock, irqflags );

    PRINTKM( IO, ( PKTD " %s try to start next listempty %s current_ioobj p%p\n", -1, __FUNCTION__, list_empty( &pqueue->head ) ? "true" : "false", pqueue->current_ioobj ) );

    if( pqueue->current_ioobj != NULL )
    {
        // iocb to queue
        list_add_tail( &phyperion_request_packet->list_entry, &pqueue->head );
        spin_unlock_irqrestore( &pqueue->s_lock, irqflags );
    }
    else
    {
        // run iocb
        pqueue->current_ioobj = phyperion_request_packet;
        spin_unlock_irqrestore( &pqueue->s_lock, irqflags );
        pqueue->start_iocb( ( struct hyperion_request_packet* )pqueue->current_ioobj, pqueue->index );
    }
    return 0;
}

//-------------------------------------------------------------------------------------------
void insert_iocb_tail( pdevice_queue_t pqueue, struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    unsigned long irqflags;
    //unsigned long irq_flags;
    spin_lock_irqsave( &pqueue->s_lock, irqflags );

    list_add_tail( &phyperion_request_packet->list_entry, &pqueue->head );
    spin_unlock_irqrestore( &pqueue->s_lock, irqflags );
}

//-------------------------------------------------------------------------------------------
void remove_entries( pdevice_queue_t pqueue )
//-------------------------------------------------------------------------------------------
{
}

//-------------------------------------------------------------------------------------------
struct hyperion_request_packet* get_current_iocb( pdevice_queue_t pqueue )
//-------------------------------------------------------------------------------------------
{
    return ( struct hyperion_request_packet* )pqueue->current_ioobj;
}


/////////////////////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------------------
result_packet_entry_t* allocate_result_packet_entry( void )
//--------------------------------------------------------------------------------------
{
    result_packet_entry_t* prp = ( result_packet_entry_t* )kmalloc( sizeof( result_packet_entry_t ), GFP_KERNEL );
    prp->result_packet = NULL;
    return prp;
}

//--------------------------------------------------------------------------------------
void free_result_packet_entry( result_packet_entry_t* prp )
//--------------------------------------------------------------------------------------
{
    kfree( prp );
}

//--------------------------------------------------------------------------------------
void push_result_packet_entry( wait_queue_head_t* head, result_packet_entry_t* entry )
//--------------------------------------------------------------------------------------
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 13, 0 )
    list_add_tail( &entry->list_entry, &head->head );
#else
    list_add_tail( &entry->list_entry, &head->task_list );
#endif
}

//--------------------------------------------------------------------------------------
result_packet_entry_t* pop_result_packet_entry( wait_queue_head_t* head )
//--------------------------------------------------------------------------------------
{
    struct list_head* entry;

    if( !is_result_queue_empty( head ) )
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 13, 0 )
        entry = remove_head_list( &head->head );
#else
        entry = remove_head_list( &head->task_list );
#endif
        return container_of( entry, struct result_packet_entry, list_entry );
    }
    else
    {
        return NULL;
    }
}

//--------------------------------------------------------------------------------------
int is_result_queue_empty( wait_queue_head_t* head )
//--------------------------------------------------------------------------------------
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 13, 0 )
    return list_empty( &head->head );
#else
    return list_empty( &head->task_list );
#endif
}

//--------------------------------------------------------------------------------------
void empty_result_queue( wait_queue_head_t* head )
//--------------------------------------------------------------------------------------
{
    result_packet_entry_t* entry;
    while( !is_result_queue_empty( head ) )
    {
        entry = pop_result_packet_entry( head );
        free_result_packet_entry( entry );
    }
}
