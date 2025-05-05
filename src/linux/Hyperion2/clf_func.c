/*
 * clf_func.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: clf_func.c,v 1.28 2011-02-25 13:11:35 ug Exp $
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
#include "dma_sg_list_buffer.h"
#include "drivermain.h"
#include "hyperion_base.h"
#include <utils.h>

static HYPERION_BASE_REGISTER_DEF CLeRegisterBaseA32[ebrhMax] = {
#include "hyperion_register_a32.h"
};

static HYPERION_BASE_REGISTER_DEF CL4eRegisterBase[ebrhMax] =
{
#include "hyperion_register_cl4e.h"
};
static HYPERION_BASE_REGISTER_DEF HDSDI4eRegisterBase[ebrhMax] =
{
#include "hyperion_register_hdsdi_4e.h"
};

#define SET_BIT(preg,bit,enable)\
    {\
        u32 reg_value = ioread32( (void __iomem *)preg );\
        if( enable )\
            reg_value |= bit;\
        else\
            reg_value &= ~bit;\
        iowrite32( reg_value, (void __iomem *)preg );\
    }

#define _READ_MAILBOX(mailbox) IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhPCICore, mailbox )
//-------------------------------------------------------------------------------------------
/// \brief item definition
struct SItem
//-------------------------------------------------------------------------------------------
{
    unsigned long status; ///< current interruptstatus
};

//-------------------------------------------------------------------------------------------
void restart_trigger_hrtc( struct hyperion_device* phyp_dev, int restart, int hrtc_index )
//-------------------------------------------------------------------------------------------
{
    if( restart )
    {
        IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhTriggerHrtController0 + hrtc_index, OFF_HRT_CONTROLLER_CTRL, 0 );
        IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhTriggerHrtController0 + hrtc_index, OFF_HRT_CONTROLLER_CTRL, HRTC_ENABLE | HRTC_INTERRUPT_ENABLE );
    }
}

//-------------------------------------------------------------------------------------------
int
prepare_scatter_gather_list(
    struct hyperion_device *phyp_dev,
    struct hyperion_request_packet *phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    volatile dma_addr_t address;
    int xfer, page_length;
    struct dma_sg_list_entry *list_entry
        = (struct dma_sg_list_entry *)phyperion_request_packet->dma_list_entry;
    u64 *psg_list_hyperion_dma_buffer = (u64 *)( list_entry->buf );
    unsigned long index_sg_list_entry = 0;
    struct user_buffer_descriptor *puser_buffer_descr
        = &phyperion_request_packet->user_buffer_descr;

    xfer = (int)phyperion_request_packet->parameters.transferlength;
    // PRINTKM(DMA,(PKTD "prep_sg_list() dto p%p xfer 0x%x isg %u max_e %lu\n",
    // phyp_dev->number, dto, xfer, itt ));
    xfer -= puser_buffer_descr->sg[0].offset;
    // PRINTKM(DMA,(PKTD "xfer 0x%x offset 0x%x\n", phyp_dev->number, xfer,
    // *offset ));

    if( phyp_dev->pci_hyperion_page_size != PAGE_SIZE )
    {
        while( xfer > 0 && index_sg_list_entry < puser_buffer_descr->nr_pages )
        {
            // PRINTKM(DMA,(PKTD " ScatterGatherList next element %d xfer
            // 0x%x\n", phyp_dev->number, index_sg_list_entry, xfer ));
            address = sg_dma_address(
                          &puser_buffer_descr->sg[index_sg_list_entry] )
                      & ~3;
            if( !dma_set_mask( &phyp_dev->pdev->dev, DMA_BIT_MASK( 64 ) ) )
            {
                const u64 highPart = DMA_BIT_MASK( 32 ) << 32;
                address |= ( address & highPart )
                               ? phyp_dev->address_space_encoding
                               : 0;
            }

            page_length
                = (int)( puser_buffer_descr->sg[index_sg_list_entry].length );
            while( page_length > 0 )
            {
                *psg_list_hyperion_dma_buffer = cpu_to_le64( (u64)address );
                // PRINTKM(DMA,(PKTD " %s: xfer 0x%x itranslationtable %d
                // translationtable %p addr 0x%llx len 0x%x\n",
                // phyp_dev->number, __FUNCTION__, xfer, itt,
                // &translation_table[itt], address, page_length ));
                // PRINTKM(DMA,(PKTD " %s: xfer 0x%x sg_list[%d] addr 0x%llx
                // 0x%llx len 0x%x\n", phyp_dev->number, __FUNCTION__, xfer,
                // index_sg_list_entry, *psg_list_hyperion_dma_buffer, address,
                // sg_dma_len(&phyperion_request_packet->sg[index_sg_list_entry])
                // ));
                page_length -= phyp_dev->pci_hyperion_page_size;
                address += phyp_dev->pci_hyperion_page_size;
                ++psg_list_hyperion_dma_buffer;
            }
            xfer
                -= (int)( puser_buffer_descr->sg[index_sg_list_entry].length );
            ++index_sg_list_entry;
        }
    }
    else
    {
        while( xfer > 0 && index_sg_list_entry < puser_buffer_descr->nr_pages )
        {
            // PRINTKM(DMA,(PKTD " ScatterGatherList next element %d xfer
            // 0x%x\n", phyp_dev->number, index_sg_list_entry, xfer ));
            address = sg_dma_address(
                          &puser_buffer_descr->sg[index_sg_list_entry] )
                      & ~3;
            if( !dma_set_mask( &phyp_dev->pdev->dev, DMA_BIT_MASK( 64 ) ) )
            {
                const u64 highPart = DMA_BIT_MASK( 32 ) << 32;
                address |= ( address & highPart )
                               ? phyp_dev->address_space_encoding
                               : 0;
            }
            *psg_list_hyperion_dma_buffer = cpu_to_le64( (u64)address );
            // PRINTKM(DMA,(PKTD " %s: [%d] xfer 0x%x addr 0x%x len 0x%x\n",
            // phyp_dev->number, __FUNCTION__, index_sg_list_entry, xfer,
            // *psg_list_hyperion_dma_buffer,
            // phyperion_request_packet->sg[index_sg_list_entry].length ));
            xfer
                -= (int)( puser_buffer_descr->sg[index_sg_list_entry].length );
            ++index_sg_list_entry;
            ++psg_list_hyperion_dma_buffer;
        }
    }
    // PRINTKM(DMA,(PKTD " prepare_scatter_gather_list() page_off 0x%lx
    // index_sg_list_entry %ld\n", phyp_dev->number, PAGE_SIZE + xfer,
    // index_sg_list_entry ));
    return dcecNoError;
}

//-------------------------------------------------------------------------------------------
void translate_property32_to_property( TPropertyElement* list, TPropertyElement32* list32, unsigned int prop_count )
//-------------------------------------------------------------------------------------------
{
    int i;
    for( i = 0; i < prop_count; i++ )
    {
        list->PropertyID = le32_to_cpu( list32->PropertyID );
        list->u.intElement =  le32_to_cpu( list32->u.intElement );
        list->Changed =  le32_to_cpu( list32->Changed );
        list->Type =  le32_to_cpu( list32->Type );
        ++list32;
        ++list;
    }
}

//-------------------------------------------------------------------------------------------
void translate_property_to_property32( TPropertyElement* list, TPropertyElement32* list32, unsigned int prop_count )
//-------------------------------------------------------------------------------------------
{
    int i;
    for( i = 0; i < prop_count; i++ )
    {
        list32->PropertyID = cpu_to_le32( list->PropertyID );
        list32->u.intElement =  cpu_to_le32( list->u.intElement );
        list32->Changed =  cpu_to_le32( list->Changed );
        list32->Type =  cpu_to_le32( list->Type );
        ++list32;
        ++list;
    }
}

//-------------------------------------------------------------------------------------------
/// \brief write all results, error, timestamp to requestlist
void write_results_to_rq( struct hyperion_device* phyp_dev, struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    struct transfer_parameter* tp;
    TPropertyElement32* list_proc_result;
    TPropertyElement* list_request_result;
    u64 proc_timestamp, sec, usec, nsec;
    u32 timestamp_low = 0, timestamp_high = 0;
    unsigned long property_count = 0;
    unsigned int bytes_to_copy = 0;
    u32 changed = 0, /*property_count = 0,*/ request_status = 0, request_id = 0, bytes_transferred_so_far = 0, trigger_frame_start_cnt = 0,
        acquisition_start_cnt = 0, frame_nr = 0;

    tp = &phyperion_request_packet->parameters;
    list_proc_result = ( ( struct dma_sg_list_entry* )( phyperion_request_packet->dma_list_entry ) )->host_msg;
    list_request_result = phyperion_request_packet->result_property;
    translate_property32_to_property( list_request_result, list_proc_result, prSnapRequestResultMax );
    _READ_PROPERTYI( list_request_result, 1, prSResPropertyCount, property_count, changed );
    _READ_PROPERTYI( list_request_result, property_count, prSResTimeStampLowPart, timestamp_low, changed );
    _READ_PROPERTYI( list_request_result, property_count, prSResTimeStampHighPart, timestamp_high, changed );
    _READ_PROPERTYI( list_request_result, property_count, prSResRequestID, request_id, changed );
    _READ_PROPERTYI( list_request_result, property_count, prSResStatus, request_status, changed );
    _READ_PROPERTYI( list_request_result, property_count, prSresBytesTransferredSoFar, bytes_transferred_so_far, changed );
    _READ_PROPERTYI( list_request_result, property_count, prSresTriggerFrameStartCounter, trigger_frame_start_cnt, changed );
    _READ_PROPERTYI( list_request_result, property_count, prSresTriggerAcquisitionStartCounter, acquisition_start_cnt, changed );
    _READ_PROPERTYI( list_request_result, property_count, prSResFrameNr, frame_nr, changed );

    PRINTKM( DMA, ( PKTD " %s nios result host_msg %p prop_cnt %lu hi 0x%x lo 0x%x regid %d(==%x) status %u bytestransferred %u\n", phyp_dev->number, __FUNCTION__, ( ( struct dma_sg_list_entry* )( phyperion_request_packet->dma_list_entry ) )->host_msg, property_count, timestamp_high, timestamp_low, request_id, request_id, request_status, bytes_transferred_so_far ) );
    if( property_count == 0 )
    {
        dump_sg_list_buffer( phyp_dev );
    }
    proc_timestamp = timestamp_high;
    proc_timestamp = ( proc_timestamp << 32 ) | timestamp_low;
    sec = proc_timestamp;
    do_div( sec, phyp_dev->processor_info.cpu_clk_hz );
    usec = sec * 1000000;
    nsec = ( ( proc_timestamp - sec * phyp_dev->processor_info.cpu_clk_hz ) * 1000000 );
    do_div( nsec, phyp_dev->processor_info.cpu_clk_hz );
    usec += nsec;
    proc_timestamp = usec;

    list_request_result = phyperion_request_packet->result_property;
    _SET_PROP_ELEM_I( list_request_result, prSResPropertyCount, prSnapRequestResultMax, TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSResStatus, request_status, TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSResRequestID, request_id, TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSResTimeStampLowPart, ( proc_timestamp & 0xffffffff ), TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSResTimeStampHighPart, ( ( proc_timestamp >> 32 ) & 0xffffffff ), TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSResScanPixLine0, phyperion_request_packet->scan_pixel_line0, TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSResScanPixLine1, phyperion_request_packet->scan_pixel_line1, TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSResScanLines, phyperion_request_packet->scan_lines, TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSresBytesTransferredSoFar, bytes_transferred_so_far, TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSresTriggerFrameStartCounter, trigger_frame_start_cnt, TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSresTriggerAcquisitionStartCounter, acquisition_start_cnt, TRUE );
    _SET_PROP_ELEM_I( list_request_result, prSResFrameNr, frame_nr, TRUE );
    //PRINTKM(DMA,(PKTD " %s, tstamp 0x%llx, status 0x%x\n", phyp_dev->number, __FUNCTION__, proc_timestamp, request_status ));
    bytes_to_copy = sizeof( phyperion_request_packet->result_property ) < phyperion_request_packet->request_buffer.trailerBufferSize ? sizeof( phyperion_request_packet->result_property ) : phyperion_request_packet->request_buffer.trailerBufferSize;
    hyperion_copy_to_trailer( &phyperion_request_packet->trailer_buffer_descr, ( char* )phyperion_request_packet->result_property, bytes_to_copy );
}

//-------------------------------------------------------------------------------------------
static unsigned char uart_interrupt( struct hyperion* phyperion, uart_object_t* pua )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    u32 int_stat, int_en;
    int_stat = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, pua->register_index, OFF_UART_INTR_CLEAR_STAT );
    IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, pua->register_index, OFF_UART_INTR_CLEAR_STAT, int_stat );
    int_en = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, pua->register_index, OFF_UART_INTR_ENABLE );
    return( ( int_stat & int_en ) != 0 );
}


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
    struct hyperion* phyperion = ( struct hyperion* )dev_id;
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    u32 pcie_intr_stat = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhPCICore, OFF_PCI_EXPRESS_INTERRUPT_STATUS );
    u32 pcie_intr_enable = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhPCICore, OFF_PCI_EXPRESS_INTERRUPT_ENABLE );
    irqreturn_t ret = IRQ_NONE;
    int interrupting = 0;
    TItem it;
    unsigned char write_ok;

    if( pcie_intr_stat & pcie_intr_enable )
    {
        IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhPCICore, OFF_PCI_EXPRESS_INTERRUPT_STATUS, ( pcie_intr_stat & pcie_intr_enable ) );
        interrupting = TRUE;
    }

    if( pcie_intr_stat & pcie_intr_enable & AV_IRQ_ASSERTED )
    {
        switch( pcie_intr_stat & AVL_IRQ_INPUT_VECTOR )
        {
        case UART0_IRQ_VEC:
            {
                uart_interrupt( phyperion, &phyp_dev->uart_port[0] );
                break;
            }
        case UART1_IRQ_VEC:
            {
                uart_interrupt( phyperion, &phyp_dev->uart_port[1] );
                break;
            }
        }
    }

    if( interrupting )
    {
        it.status = ( pcie_intr_stat & pcie_intr_enable );
        it.status |= ( pcie_intr_stat & AVL_IRQ_INPUT_VECTOR );
        _PIPE_WRITE( phyp_dev->interrupt_result_pipe, it, write_ok );
        tasklet_schedule( &phyperion->hyperion_tasklet );
        ret = IRQ_HANDLED;
    }

    return ret;
}

//-------------------------------------------------------------------------------------------
void
hyperion_do_tasklet( unsigned long index )
//-------------------------------------------------------------------------------------------
{
    struct hyperion        *phyperion = get_hyperion( index );
    struct hyperion_device *phyp_dev
        = (struct hyperion_device *)phyperion->device;
    struct hyperion_request_packet *phyperion_request_packet = NULL;
    TItem                           isr_result;
    message_item                    msg_it;
    unsigned char                   read_ok;
    u32                             dto_index;
    unsigned long                   irqflags;
    msg_it.message = 0;

    do
    {
        _PIPE_READ_LOCK( phyp_dev->interrupt_result_pipe, &isr_result,
                         read_ok );
        if( read_ok == 0 )
        {
            break;
        }
        // printk( " %s isr_result.status 0x%x\n", __FUNCTION__,
        // isr_result.status );
        if( isr_result.status & A2P_MAILBOX_INT0 )
        {
            complete( &phyp_dev->message_received );
        }

        if( isr_result.status & A2P_MAILBOX_INT1 )
        {
            do
            {
                if( _ITEMS_IN_MSG_PIPE_LE32(
                        phyp_dev->request_result_message ) )
                {
                    spin_lock_irqsave( &phyp_dev->ioctl_lock.s_tasklet,
                                       irqflags );
                    _READ_MSG_PIPE_LE32( phyp_dev->request_result_message,
                                         msg_it );
                    spin_unlock_irqrestore( &phyp_dev->ioctl_lock.s_tasklet,
                                            irqflags );
                }
                else
                {
                    break;
                }
                // printk(" %s isr_result.status 0x%x message 0x%x\n",
                // __FUNCTION__, isr_result.status, msg_it.message );
                dto_index = _GET_MESSAGE_QUEUE_ID( msg_it.message );
                phyperion_request_packet = get_current_iocb(
                    phyp_dev->pdma_object[dto_index]->rw_queue );
                if( phyperion_request_packet != NULL )
                {
                    // read message from nios
                    write_results_to_rq( phyp_dev, phyperion_request_packet );
                    complete_request( phyperion_request_packet,
                                      0 ); //! phyperion_request_packet will be
                                           //! freed in complete_request()
                    phyperion_request_packet = NULL;
                    run_next_iocb(
                        phyp_dev->pdma_object[dto_index]->rw_queue );
                }
            } while( 1 );
        }

        if( isr_result.status & A2P_MAILBOX_INT2 )
        {
            unsigned int mb2 = _READ_MAILBOX( OFF_A2P_MAILBOX2 );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def,
                         ebrhPCICore, OFF_P2A_MAILBOX2, mb2 );
            PRINTKM( INTR, ( PKTD " msg: 0x%08x\n", phyperion->number, mb2 ) );
        }

        if( isr_result.status & AV_IRQ_ASSERTED )
        {
            switch( isr_result.status & AVL_IRQ_INPUT_VECTOR )
            {
            case UART0_IRQ_VEC:
            {
                read_serial( &phyp_dev->uart_port[0] );
                break;
            }
            case UART1_IRQ_VEC:
            {
                read_serial( &phyp_dev->uart_port[1] );
                break;
            }
            }
        }
    } while( 1 );
}

//-------------------------------------------------------------------------------------------
int commit_request_to_mailbox( struct hyperion_device* phyp_dev, struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    struct dma_sg_list_entry* list_entry = ( struct dma_sg_list_entry* )phyperion_request_packet->dma_list_entry;
    struct transfer_parameter* tp = &phyperion_request_packet->parameters;
    TPropertyElement* list0 = 0;
    TPropertyElement* list1 = 0;
    unsigned long property_count = 0;
    message_item msg_it;
    u64 dma_address;

    PRINTKM( DMA, ( PKTD " %s host_msg %p reqid %d (==%x) \n", phyp_dev->number, __FUNCTION__, list_entry->host_msg, tp->reqid, tp->reqid ) );
    //preset result_buffer
    memset( list_entry->host_msg, 0, PAGE_SIZE );
    list1 = ( TPropertyElement* )list_entry->host_msg;
    list0 = list1;
    ++list1;
    _WRITE_PROPERTYI( list1, prSresBytesTransferredSoFar, 0, TRUE );
    ++property_count;
    _WRITE_PROPERTYI( list1, prSResStatus, cerrDMAAborted, TRUE );
    ++property_count;
    ++property_count;
    _WRITE_PROPERTYI( list0, prSResPropertyCount, property_count, TRUE );
    //request parameter

    _SCAN_PROPERTY_LIST( phyperion_request_packet->request_parameter, list0, tp->property_count, prClScanMode );
    _WRITE_PROPERTYI( list0, prClScanMode, tp->scanmode ? ( SYNC_STATUS_FRAME | SYNC_STATUS_LINE ) : SYNC_STATUS_LINE, TRUE );
    dma_address = cpu_to_le64( list_entry->phy );
    if( dma_address ==  list_entry->phy )
    {
        _SCAN_PROPERTY_LIST( phyperion_request_packet->request_parameter, list0, tp->property_count, prDMASGListPhysLowAddress );
        _WRITE_PROPERTYI( list0, prDMASGListPhysLowAddress, le32_to_cpu( dma_address & 0xffffffff ), TRUE );
        _SCAN_PROPERTY_LIST( phyperion_request_packet->request_parameter, list0, tp->property_count, prDMASGListPhysHighAddress );
        _WRITE_PROPERTYI( list0, prDMASGListPhysHighAddress, le32_to_cpu( ( dma_address >> 32 ) & 0xffffffff ), TRUE );
    }
    else
    {
        _SCAN_PROPERTY_LIST( phyperion_request_packet->request_parameter, list0, tp->property_count, prDMASGListPhysHighAddress );
        _WRITE_PROPERTYI( list0, prDMASGListPhysHighAddress, le32_to_cpu( dma_address & 0xffffffff ), TRUE );
        _SCAN_PROPERTY_LIST( phyperion_request_packet->request_parameter, list0, tp->property_count, prDMASGListPhysLowAddress );
        _WRITE_PROPERTYI( list0, prDMASGListPhysLowAddress, le32_to_cpu( ( dma_address >> 32 ) & 0xffffffff ), TRUE );
    }
    //write also to nios_msg_buffer
    translate_property_to_property32( phyperion_request_packet->request_parameter, ( TPropertyElement32* )list_entry->nios_msg, tp->property_count );
    msg_it.message = MESSAGE_TYPE_REQUEST;
    msg_it.message |= ( ( ( tp->property_count * sizeof( TPropertyElement32 ) ) & MESSAGE_SIZE_MSK ) << MB0_MESSAGE_SIZE ) | ( list_entry->index << MB0_MESSAGE_INDEX ) | _SET_MESSAGE_QUEUE_ID( tp->inputchannel );
    if( _ITEMS_IN_MSG_PIPE_LE32( phyp_dev->request_message ) < le32_to_cpu( phyp_dev->request_message.shared_def->items_max ) )
    {
        _WRITE_MSG_PIPE_LE32( phyp_dev->request_message, msg_it );
        return 0;
    }
    else
    {
        return -ECOMM;
    }
}

//-------------------------------------------------------------------------------------------
void start_transfer_dummy( struct hyperion_request_packet* phyperion_request_packet, int index )
//-------------------------------------------------------------------------------------------
{
}

//-------------------------------------------------------------------------------------------
int synchronize_cameralink_input_channels( struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    struct hyperion* phyperion = ( struct hyperion* )phyperion_request_packet->private;
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    struct transfer_parameter* tp = &phyperion_request_packet->parameters;
    unsigned int i = 0, mux_ctrl = 0;
    int result = 0;

    if( phyp_dev->fpga_info.bits.automatic_sync_cl_channels == 0 && tp->medium_mode_ctrl )
    {
        int b_phase_error_detected = 0;

        for( i = 0; i < MAX_PHASE_ERROR_RETRY_COUNT; i++ )
        {
            mux_ctrl = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER );
            if( mux_ctrl & PHASE_ERROR_DETECT )
            {
                mux_ctrl |= RESET_MEDIUM_CHANNELS;
                IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER, mux_ctrl );
                mux_ctrl &= ~RESET_MEDIUM_CHANNELS;
                IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER, mux_ctrl );
                b_phase_error_detected = 1;
            }
            else
            {
                break;
            }
            sleep_msec( 1 );
        }
        mux_ctrl = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER );
        if( mux_ctrl & PHASE_ERROR_DETECT )
        {
            PRINTKM( MOD, ( PKTD "%s: dma_channels could not synchronize.\n", phyperion->number, __FUNCTION__ ) );
        }
        PRINTKM( MOD, ( PKTD " %s, status 0x%x, manually cameralink_channel synchronisation has finished, bPhaseErrorDetected %d, retryCount %d\n", phyperion->number, __FUNCTION__, result, b_phase_error_detected, i ) );
    }
    else
    {
        PRINTKM( MOD, ( PKTD " %s, automatically cameralink_channel synchronisation detected.\n", phyperion->number, __FUNCTION__ ) );
        result = 0;
    }
    return result;
}

//-------------------------------------------------------------------------------------------
int start_transfer( struct hyperion_request_packet* phyperion_request_packet, int index )
//-------------------------------------------------------------------------------------------
{
    struct hyperion* phyperion = ( struct hyperion* )phyperion_request_packet->private;
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    struct dma_transfer_object* dto = phyp_dev->pdma_object[phyperion_request_packet->parameters.inputchannel];
    struct transfer_parameter* tp = &phyperion_request_packet->parameters;
    unsigned int mux_ctrl = 0;
    int result = 0;

    ( *phyp_dev->set_mux_data )( phyperion, &dto->mux_seq, tp->inputchannel );
    if( tp->has_properties_changed )
    {
        PRINTKM( MOD, ( PKTD "%s rewrite transferparameter\n", phyperion->number, __FUNCTION__ ) );
        //dto->rewrite_transfer_parameter = FALSE;
        mux_ctrl |= ( tp->datavalidenable > 0 )     ? DATA_VALID_ENABLE : 0;
        mux_ctrl |= ( ( !tp->scanmode ) > 0 )       ? LINE_SCAN_ENABLE : 0;
        mux_ctrl |= ( ( tp->medium_mode_ctrl << MEDIUM_MODE_BIT ) & MEDIUM_MODE_ENABLE );
        mux_ctrl |= ( tp->enable_aoimode > 0 )  ? AOI_ENABLE : 0;
        mux_ctrl |= ( ( tp->expand_lval & EXPAND_LVAL_MSK ) << EXPAND_LVAL );
        PRINTKM( MOD, ( PKTD "%s mux_ctrl %x\n", phyperion->number, __FUNCTION__, mux_ctrl ) );
        if( tp->medium_mode_ctrl )
        {
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_XSTART, tp->xstart );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_XSTOP, tp->xstop );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_YSTART, tp->ystart );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_YSTOP, tp->ystop );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_XSTART, tp->xstart );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_XSTOP, tp->xstop );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_YSTART, tp->ystart );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_YSTOP, tp->ystop );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER, mux_ctrl );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_MUX_CONTROLLER, mux_ctrl );
        }
        else
        {
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, tp->inputchannel + ebrhCLController0, OFF_AOI_XSTART, tp->xstart );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, tp->inputchannel + ebrhCLController0, OFF_AOI_XSTOP, tp->xstop );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, tp->inputchannel + ebrhCLController0, OFF_AOI_YSTART, tp->ystart );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, tp->inputchannel + ebrhCLController0, OFF_AOI_YSTOP, tp->ystop );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, tp->inputchannel + ebrhCLController0, OFF_MUX_CONTROLLER, mux_ctrl );
        }
        //PRINTKM(DMA,(PKTD " mxctrl[0] 0x%x mxctrl[1] 0x%x",  phyperion->number, *phyperion->register_base.MuxCtrl[0], *phyperion->register_base.MuxCtrl[1]  ));
        SET_BIT( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL ), ENABE_BASE_MODE_CH1, ( tp->medium_mode_ctrl == 0 ) );
        IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_EXTENDED_MODE, tp->line_scan_startcondition );
        IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_EXTENDED_MODE, tp->line_scan_startcondition );
    }
    PRINTKM( DMA, ( PKTD "%s phyperion_request_packet %p reqId %d, timeout %d\n", phyperion->number, __FUNCTION__, phyperion_request_packet, tp->reqid, tp->timeoutmsec ) );
    result = synchronize_cameralink_input_channels( phyperion_request_packet );
    if( result >= 0 )
    {
        phyperion_request_packet->scan_pixel_line0 = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, tp->inputchannel + ebrhCLController0, OFF_SCAN_PIX_LINE0 );
        phyperion_request_packet->scan_pixel_line1 = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, tp->inputchannel + ebrhCLController0, OFF_SCAN_PIX_LINE1 );
        phyperion_request_packet->scan_lines = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, tp->inputchannel + ebrhCLController0, OFF_SCAN_LINES );

        result = commit_request_to_mailbox( phyp_dev, phyperion_request_packet );
    }
    if( result < 0 )
    {
        PRINTKM( IO, ( PKTD " commit_request_to_mailbox( ) result %d", phyperion->number, result ) );
    }
    return result;
}

//-------------------------------------------------------------------------------------------
void
hyperion_func_abort_transfer( struct hyperion *phyperion )
//-------------------------------------------------------------------------------------------
{
    DECLARE_WAIT_QUEUE_HEAD( wq );
    struct hyperion_device *phyp_dev
        = (struct hyperion_device *)phyperion->device;
    struct hyperion_request_packet *phyperion_request_packet;
    struct dma_transfer_object *dto;
    TPropertyElement *list_request_result;
    int i;
    const int timeout_msec = 100;
    unsigned int message = MESSAGE_TYPE_ABORT_REQUESTS;
    unsigned int bytes_to_copy;

    PRINTKM( DMA,
             ( PKTD " >%s %lu\n", phyperion->number, __FUNCTION__, jiffies ) );
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        dto = phyp_dev->pdma_object[i];
        if( get_current_iocb( dto->rw_queue ) )
        {
            transmit_message( phyp_dev, message, timeout_msec, NULL, 0, NULL );
            while(
                ( phyperion_request_packet = run_next_iocb( dto->rw_queue ) )
                != NULL )
            {
                list_request_result
                    = phyperion_request_packet->result_property;
                PRINTKM( DMA,
                         ( PKTD " %s abort and release reqid %d (==%x) \n",
                           phyp_dev->number, __FUNCTION__,
                           phyperion_request_packet->parameters.reqid,
                           phyperion_request_packet->parameters.reqid ) );
                _SET_PROP_ELEM_I( list_request_result, prSResPropertyCount,
                                  prSnapRequestResultMax, TRUE );
                _SET_PROP_ELEM_I( list_request_result, prSResStatus,
                                  cerrDMAAborted, TRUE );
                _SET_PROP_ELEM_I( list_request_result, prSResRequestID,
                                  phyperion_request_packet->parameters.reqid,
                                  TRUE );
                _SET_PROP_ELEM_I( list_request_result, prSResTimeStampLowPart,
                                  0, TRUE );
                _SET_PROP_ELEM_I( list_request_result, prSResTimeStampHighPart,
                                  0, TRUE );
                _SET_PROP_ELEM_I( list_request_result, prSResScanPixLine0,
                                  phyperion_request_packet->scan_pixel_line0,
                                  TRUE );
                _SET_PROP_ELEM_I( list_request_result, prSResScanPixLine1,
                                  phyperion_request_packet->scan_pixel_line1,
                                  TRUE );
                _SET_PROP_ELEM_I( list_request_result, prSResScanLines,
                                  phyperion_request_packet->scan_lines, TRUE );
                _SET_PROP_ELEM_I( list_request_result,
                                  prSresBytesTransferredSoFar, 0, TRUE );
                bytes_to_copy
                    = sizeof( phyperion_request_packet->result_property )
                              < phyperion_request_packet->request_buffer
                                    .trailerBufferSize
                          ? sizeof( phyperion_request_packet->result_property )
                          : phyperion_request_packet->request_buffer
                                .trailerBufferSize;
                hyperion_copy_to_trailer(
                    &phyperion_request_packet->trailer_buffer_descr,
                    (char *)phyperion_request_packet->result_property,
                    bytes_to_copy );
                complete_request( phyperion_request_packet, 0 );
            }
        }
    }
    PRINTKM( DMA,
             ( PKTD " <%s %lu\n", phyperion->number, __FUNCTION__, jiffies ) );
} // abort_transfer

//-------------------------------------------------------------------------------------------
int hyperion_func_async_read( struct hyperion* phyperion, struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    int result = 0;
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;

    if( phyp_dev->user_flash_enabled == FALSE )
    {
        printk( MODULE_NAME " %s userflash with correct firmware file not programmed, read access cancelled\n", __FUNCTION__ );
        return -1010;
    }

    if( phyperion_request_packet->parameters.reqaction == raiSnapRequest )
    {
        struct dma_transfer_object* dto = phyp_dev->pdma_object[phyperion_request_packet->parameters.inputchannel];
        //iocb->ki_cancel = cancel_async_read_write;
        if( phyp_dev->mux_seq.changed )
        {
            phyp_dev->mux_seq.changed = FALSE;
            dto->mux_seq.changed = TRUE;
            dto->mux_seq.size = phyp_dev->mux_seq.size;
            memcpy( ( void* )dto->mux_seq.muxdata, ( void* )phyp_dev->mux_seq.muxdata, MUX_CONTROLLER_SEQUENCE_SIZE_BYTES );
        }
        if( phyperion_request_packet->dma_list_entry == NULL )
        {
            phyperion_request_packet->dma_list_entry = ( void* )get_sg_list_buffer( phyp_dev );
            if( phyperion_request_packet->dma_list_entry == NULL )
            {
                return -ENOMEM;
            }
            if( prepare_scatter_gather_list( phyp_dev, phyperion_request_packet ) < 0 )
            {
                return -ENOMEM;
            }
        }
        result = start_transfer( phyperion_request_packet, 0 );
        if( result >= 0 )
        {
            run_iocb( dto->rw_queue, phyperion_request_packet );
        }
        else
        {
            goto err_out;
        }
        return result;
    }
    else
    {
        return -EFAULT;
    }

err_out:
    hyperion_func_release_dma( phyperion_request_packet );
    return result;

}

//-------------------------------------------------------------------------------------------
void prepare_communication_buffer( struct hyperion_device* phyp_dev )
//-------------------------------------------------------------------------------------------
{
    u64* comm_buffer = ( u64* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhOnChipMemData, 0 );
    int i;

    PRINTKM( IO, ( PKTD " %s hyperion base %p\n", phyp_dev->number, __FUNCTION__, phyp_dev->hyperion_base.base ) );
    for( i = 0; i < MAX_COUNT_REQUEST_OBJECT; i++ )
    {
        *comm_buffer = cpu_to_le64( phyp_dev->dma_sg_list_pool[i].nios_msg_phy );
        PRINTKM( MOD, ( PKTD " %s %p %llx\n", phyp_dev->number, __FUNCTION__, comm_buffer, ( long long unsigned int )( *comm_buffer ) ) );
        ++comm_buffer;
        *comm_buffer = cpu_to_le64( phyp_dev->dma_sg_list_pool[i].host_msg_phy );
        PRINTKM( MOD, ( PKTD " %s %p %llx\n", phyp_dev->number, __FUNCTION__, comm_buffer, ( long long unsigned int )( *comm_buffer ) ) );
        ++comm_buffer;
    }
}


//-------------------------------------------------------------------------------------------
void initialize_message_pipe( struct hyperion_device* phyp_dev, message_pipe* msg_pipe, unsigned long off_shared, unsigned long off_pipe, unsigned long size_pipe )
//-------------------------------------------------------------------------------------------
{
    msg_pipe->shared_def = ( message_pipe_shared_def* )( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhOnChipMemData, off_shared ) );
    msg_pipe->data = ( message_item* )( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhOnChipMemData, off_pipe ) );
    msg_pipe->head = msg_pipe->data;
    msg_pipe->tail = msg_pipe->data;
    msg_pipe->shared_def->items_r = 0;
    msg_pipe->shared_def->items_w = 0;
    msg_pipe->shared_def->items_max = cpu_to_le32( size_pipe / sizeof( message_item ) ); // will be overwritte from fpga processor
    msg_pipe->shared_def->host_initialized = 1;
}

//-------------------------------------------------------------------------------------------
void enable_enhanced_hardware_debugging( struct hyperion_device* phyp_dev )
//-------------------------------------------------------------------------------------------
{
    const unsigned int system_ctrl_enable_hardware_debugging = 0x80000;
#if DEBUG
    unsigned int system_ctrl = 0;
    unsigned int output_signal_selector = 0;
    if( _flgset( ENABLE_HARDWARE_SIGNAL_DEBUGGING_OUT_ON_J6 ) )
    {
        system_ctrl = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL );
        PRINTKM( IO, ( PKTD "hyperion system control register: 0x%x\n", phyp_dev->number, system_ctrl ) );
        system_ctrl |= system_ctrl_enable_hardware_debugging;
        IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL, system_ctrl );
        system_ctrl = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL );
        PRINTKM( IO, ( PKTD "hyperion system control register debug on J6 enabled: 0x%x\n", phyp_dev->number, system_ctrl ) );
        output_signal_selector = ( debug & HARDWARE_SIGNAL_SELECTOR_MSK ) >> 8 ;
        PRINTKM( IO, ( PKTD "hyperion enhanced hardware debugging, output signal selector = 0x%x\n", phyp_dev->number, output_signal_selector ) );
        IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DEBUG_SIGNALS, output_signal_selector );
    }
    else
#endif
    {
        system_ctrl = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL );
        system_ctrl &= ~system_ctrl_enable_hardware_debugging;
        IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL, system_ctrl );
    }
}
//-------------------------------------------------------------------------------------------
int
hyperion_func_init( struct hyperion *phyperion, unsigned long control )
//-------------------------------------------------------------------------------------------
{
    int                     i, result = 0;
    u32                     system_ctrl_reg;
    struct hyperion_device *phyp_dev
        = kmalloc( sizeof( struct hyperion_device ), GFP_KERNEL );

    PRINTKM( MOD, ( PKTD " >%s\n", phyperion->number, __FUNCTION__ ) );
    if( phyp_dev == NULL )
    {
        return -ENOMEM;
    }
    memset( phyp_dev, 0, sizeof( struct hyperion_device ) );
    phyperion->device = (void *)phyp_dev;
    phyp_dev->number = phyperion->number;
    mutex_init( &phyp_dev->ioctl_lock.s_message );
    spin_lock_init( &phyp_dev->ioctl_lock.s_tasklet );
    mutex_init( &phyp_dev->ioctl_lock.s_generic );
    mutex_init( &phyp_dev->ioctl_lock.s_request );
    mutex_init( &phyp_dev->ioctl_lock.s_digital_io );
    init_completion( &phyp_dev->message_received );
    phyp_dev->pdev = phyperion->pdev;
    phyp_dev->address_space_encoding = DMA_ADDRESS_SPACE_ENCODING_32BIT;
    if( !dma_set_mask( &phyperion->pdev->dev, DMA_BIT_MASK( 64 ) ) )
    {
        phyp_dev->address_space_encoding = DMA_ADDRESS_SPACE_ENCODING_64BIT;
        dma_set_coherent_mask( &phyperion->pdev->dev, DMA_BIT_MASK( 64 ) );
    }
    else if( !dma_set_mask( &phyperion->pdev->dev, DMA_BIT_MASK( 32 ) ) )
    {
        dma_set_coherent_mask( &phyperion->pdev->dev, DMA_BIT_MASK( 32 ) );
    }
    else
    {
        printk( KERN_WARNING " %s: No suitable DMA available.\n",
                __FUNCTION__ );
        return -1000;
    }
    result = init_sg_buffer_list( phyperion );
    if( result < 0 )
    {
        return result;
    }
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        phyp_dev->pqueues[i] = &phyp_dev->dq_read_write[i];
        phyp_dev->pdma_object[i] = &phyp_dev->dma_transfer_object[i];
        initialize_queue( phyp_dev->pqueues[i], start_transfer_dummy, i );
    }
    phyp_dev->pqueue_result = &phyp_dev->dq_result;
    initialize_queue( phyp_dev->pqueue_result, start_transfer_dummy, 0 );
    switch( phyperion->vd_id.deviceId )
    {
    case PCI_DEVICE_ID_HYPERION_CL4E:
        PRINTKM( MOD, ( PKTD " initialize PCI_DEVICE_ID_HYPERION_CL4E\n",
                        phyperion->number ) );
        phyp_dev->hyperion_base.base = phyperion->memory_base[2].base;
        phyp_dev->hyperion_base.size = phyperion->memory_base[2].size;
        phyp_dev->reg_def = CL4eRegisterBase;
        break;
    case PCI_DEVICE_ID_HYPERION_HDSDI_4X:
        PRINTKM( MOD, ( PKTD " initialize PCI_DEVICE_ID_HYPERION_HDSDI_4X\n",
                        phyperion->number ) );
        phyp_dev->hyperion_base.base = phyperion->memory_base[2].base;
        phyp_dev->hyperion_base.size = phyperion->memory_base[2].size;
        phyp_dev->reg_def = HDSDI4eRegisterBase;
        break;
    default:
    case PCI_DEVICE_ID_HYPERION_CLE:
        PRINTKM( MOD, ( PKTD " initialize PCI_DEVICE_ID_HYPERION_CLE\n",
                        phyperion->number ) );
        phyp_dev->hyperion_base.base = phyperion->memory_base[0].base;
        phyp_dev->hyperion_base.size = phyperion->memory_base[0].size;
        phyp_dev->reg_def = CLeRegisterBaseA32;
        break;
    }

    system_ctrl_reg = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def,
                                  ebrhSystemRegister, OFF_SYSTEM_CONTROL );
    phyp_dev->pci_hyperion_page_size
        = system_ctrl_reg & HYPERION_PCI_PAGE_SIZE_64K ? PAGE_SIZE_64K
                                                       : PAGE_SIZE_4K;
    PRINTKM( MOD, ( PKTD " hyperion page_size %d kB sys_ctrl 0x%x\n",
                    phyp_dev->number, phyp_dev->pci_hyperion_page_size / KB,
                    system_ctrl_reg ) );

    if( IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def,
                    ebrhSystemRegister, OFF_SYSTEM_VERSION )
        == 0 )
    {
        printk( "hyperion2 module"
                " %s default flash found, grabber functionality reduced to "
                "flashupdate\n",
                __FUNCTION__ );
        phyp_dev->user_flash_enabled = FALSE;
        return 0;
    }
    else
    {
        phyp_dev->user_flash_enabled = TRUE;
    }

    // initialize DMA_TRANSFER_OBJECT's
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        phyp_dev->pdma_object[i]->index = i;
        SET_BIT( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def,
                              i + ebrhCLController0, OFF_MUX_CONTROLLER ),
                 AOI_ENABLE, TRUE );
        phyp_dev->pdma_object[i]->rw_queue = phyp_dev->pqueues[i];
        phyp_dev->pdma_object[i]->init_done = FALSE;
        CreatePoCLObject( &phyp_dev->pocl[i], phyp_dev->hyperion_base.base,
                          phyp_dev->reg_def, i + ebrhPoCLCtrl0 );
        if( control & _Ctrl( POCL ) )
        {
            PoCLChanged( &phyp_dev->pocl[i], TRUE );
        }
    }

    phyp_dev->reg_def[ebrhOnChipMemData].offset = ONCHIP_MEM_DATA_BASE;
    phyp_dev->reg_def[ebrhOnChipMemData].offset = ONCHIP_MEM_DATA_BASE;
    prepare_communication_buffer( phyp_dev );
    _PIPE_INITIALIZE( phyp_dev->interrupt_result_pipe, DMA_RESULT_QUEUE_LEN );

    // this are our communication path to the fpga processor. The pipes
    // initialized here are located in a shared buffer from fpga.
    initialize_message_pipe( phyp_dev, &phyp_dev->request_message,
                             OFF_ONCHIP_MEM_DATA_NIOS_MSG_HEADER,
                             OFF_ONCHIP_MEM_DATA_NIOS_MSG_PIPE,
                             SIZE_ONCHIP_MEM_DATA_NIOS_MSG_PIPE );
    initialize_message_pipe( phyp_dev, &phyp_dev->request_result_message,
                             OFF_ONCHIP_MEM_DATA_HOST_MSG_HEADER,
                             OFF_ONCHIP_MEM_DATA_HOST_MSG_PIPE,
                             SIZE_ONCHIP_MEM_DATA_HOST_MSG_PIPE );
    hold_nios_in_reset( (unsigned char *)phyp_dev->hyperion_base.base );
    phyp_dev->processor_info.cpu_clk_hz = 100 * 1000000;
    phyp_dev->processor_status = cnerrNiosStopped;
    phyp_dev->processor_app_size = 0;
    phyp_dev->eeprom_write_access = -1;
    enable_enhanced_hardware_debugging( phyp_dev );
    // PRINTKM(MOD,(PKTB " <%s --> irqlin %d\n", phyp_dev->number,
    // __FUNCTION__, phyperion->irqlin));
    PRINTKM( MOD, ( PKTD " <%s\n", phyperion->number, __FUNCTION__ ) );
    return result;
}

//-------------------------------------------------------------------------------------------
int hyperion_func_close( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    int i;
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;

    IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhPCICore, OFF_PCI_EXPRESS_INTERRUPT_ENABLE, 0 );
    for( i = 0; i < UART_NUM; i++ )
    {
        ClosePoCL( &phyp_dev->pocl[i] );
    }
    _PIPE_DESTROY( phyp_dev->interrupt_result_pipe );
    release_sg_buffer_list( phyperion );
    hyperion_halt_processor( phyperion );
    kfree( phyp_dev );
    return 0;
}

//-------------------------------------------------------------------------------------------
int hyperion_func_enable_interrupt( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhPCICore, OFF_PCI_EXPRESS_INTERRUPT_ENABLE, AVL_IRQ | A2P_MAILBOX_INT0 | A2P_MAILBOX_INT1 | A2P_MAILBOX_INT2 );
    return 0;
}

//-------------------------------------------------------------------------------------------
int
query_processor_system_infos( struct hyperion_device *phyp_dev )
//-------------------------------------------------------------------------------------------
{
    printk( KERN_INFO "query processor in atomic %d\n", in_atomic() );
    int result = 0;
    unsigned long cpu_clk_hz, message = MESSAGE_TYPE_SYSTEM_INFO;
    message |= QUERY_CPU_CLK_HZ;
    result = transmit_message( phyp_dev, message, 500, NULL, 0, &cpu_clk_hz );
    if( result < 0 || cpu_clk_hz == 0 )
    {
        cpu_clk_hz = 100 * 1000000;
        printk( KERN_ERR "could not query system infos: %d %s\n", result,
                strkerr( result ) );
    }
    phyp_dev->processor_info.cpu_clk_hz = cpu_clk_hz;
    printk( " %s  fpga processor cpuclk_hz %d status %x", __FUNCTION__,
            phyp_dev->processor_info.cpu_clk_hz, result );
    return result;
}

//-------------------------------------------------------------------------------------------
void
query_fpga_capabilities( struct hyperion_device *phyp_dev )
//-------------------------------------------------------------------------------------------
{
    int result = 0;
    unsigned long fpga_cap = 0;
    unsigned long message = MESSAGE_TYPE_SYSTEM_INFO;
    message |= QUERY_FPGA_CAPABILITIES;
    phyp_dev->fpga_info.capabilities = 0;
    result = transmit_message( phyp_dev, message, 500, NULL, 0, &fpga_cap );
    if( result >= 0 )
    {
        phyp_dev->fpga_info.capabilities = fpga_cap;
    }
    else
    {
        printk( KERN_ERR "could not query fpga capabilities: %d %s\n", result,
                strkerr( result ) );
        phyp_dev->fpga_info.capabilities = 0;
    }
    PRINTKM( IO, ( PKTD " %s  fpga info 0x%x", phyp_dev->number, __FUNCTION__,
                   phyp_dev->fpga_info.capabilities ) );
}

//-------------------------------------------------------------------------------------------
void hyperion_halt_processor( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    hold_nios_in_reset( ( unsigned char* )phyp_dev->hyperion_base.base );
}

//-------------------------------------------------------------------------------------------
int
hyperion_boot_processor( struct hyperion_device *phyp_dev, void *apps,
                         unsigned int size )
//-------------------------------------------------------------------------------------------
{
    int i, result = -ENODEV;
    const int retries = 10;
    for( i = 0; i < retries; i++ )
    {
        if( start_nios_cpu( (unsigned char *)phyp_dev->hyperion_base.base,
                            apps, size ) )
        {
            wait_jiffies( msecs_to_jiffies( 500 ) );
            result = query_processor_system_infos( phyp_dev );
            printk( " %s, hyperion board fpga processor started successfully "
                    "status 0x%x\n",
                    __FUNCTION__, result );
            query_fpga_capabilities( phyp_dev );
        }
        else
        {
            result = -ENXIO;
            printk(
                " %s, hyperion board fpga processor not started status 0x%x\n",
                __FUNCTION__, result );
        }
        if( result == 0 )
        {
            break;
        }
    }
    return result;
}

//-------------------------------------------------------------------------------------------
int
transmit_message( struct hyperion_device *phyp_dev, unsigned int message,
                  unsigned int timeout_msec, void *buffer, unsigned int size,
                  unsigned long *processor_result )
//-------------------------------------------------------------------------------------------
{
    int result = -ENODATA;
    mutex_lock( &phyp_dev->ioctl_lock.s_message );
    if( buffer != NULL && size > 0
        && size <= SIZE_ONCHIP_MEM_DATA_SHARED_BUFFER_NIOS )
    {
        memcpy( (void *)( (char *)phyp_dev->hyperion_base.base
                          + ONCHIP_MEM_DATA_BASE
                          + OFF_ONCHIP_MEM_DATA_SHARED_BUFFER_NIOS ),
                buffer, size );
    }
    reinit_completion( &phyp_dev->message_received );
    IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhPCICore,
                 OFF_P2A_MAILBOX0, message );

    result = wait_for_completion_io_timeout(
        &phyp_dev->message_received, msecs_to_jiffies( timeout_msec ) );

    if( result > 0 ) // no timeout
    {
        if( processor_result != NULL )
        {
            *processor_result = _READ_MAILBOX( OFF_A2P_MAILBOX0 );
            PRINTKM( IO, ( PKTD " %s result %x "
                                " mailbox0 0x%lx"
                                "\n",
                           phyp_dev->number, __FUNCTION__, result,
                           *processor_result ) );
            result = 0;
        }
    }
    else // have a timeout or an error
    {
        result = -ETIME;
    }

    mutex_unlock( &phyp_dev->ioctl_lock.s_message );
    return result;
}

//-------------------------------------------------------------------------------------------
void hyperion_func_release_dma( struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    struct hyperion* phyperion = ( struct hyperion* )phyperion_request_packet->private;
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    free_sg_list_buffer( phyp_dev, ( struct dma_sg_list_entry* )phyperion_request_packet->dma_list_entry );
    phyperion_request_packet->dma_list_entry = NULL;
}

//-------------------------------------------------------------------------------------------
void hyperion_func_release_hw( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    hyperion_func_abort_transfer( phyperion );
    hyperion_halt_processor( phyperion );
    free_all_user_buffer_descriptors( phyperion );
}

//-------------------------------------------------------------------------------------------
void hyperion_func_camera_power( struct hyperion* phyperion, int state )
//-------------------------------------------------------------------------------------------
{
    int i;
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;

    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        PoCLChanged( &phyp_dev->pocl[i], state );
    }

}
