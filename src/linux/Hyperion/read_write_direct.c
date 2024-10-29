/*
 * read_write.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id:
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

#include <asm/uaccess.h>
#include <asm/dma.h>
#include <asm/system.h>
#include <asm/cacheflush.h>

#ifndef KERNEL_VERSION
#   define KERNEL_VERSION(a,b,c) (((a) << 16) | ((b) << 8) | (c))
#endif

#include "matrix_tools.h"
#include "hyperion.h"
#include "device_func.h"
#include "hyperion_defs.h"
#include "HyperionIoCtl.h"

//-------------------------------------------------------------------------------------------
void prepare_scatter_gather_list_direct( void* pioobj, u32 nbytes_next_xfer, u32* pnext_xfer, u32 i_scatter_gather )
//-------------------------------------------------------------------------------------------
{
    struct io_object* ioobj = ( struct io_object* )pioobj;
    struct dma_transfer_object* dto = ioobj->dma_transfer_object;
    u32 max_entries, isg ;
    int elen, xfer = ( int )( nbytes_next_xfer > ioobj->nbytes ? ioobj->nbytes : nbytes_next_xfer );
    u64* translation_table = ( u64* )dto->avalon_to_pci_table.base;
    dma_addr_t address;

    if( pnext_xfer != NULL )
    {
        *pnext_xfer = xfer;
    }

    ioobj->num_xfer += xfer;
    ioobj->nbytes -= xfer;
    isg = i_scatter_gather;
    max_entries = dto->avalon_to_pci_table.size / PAGE_SIZE;

    //get scatter/gather list
    //result_dma_map = dma_map_sg( struct device *dev, ioobj->sg, i, DMA_FROM_DEVICE );
    //or call int dma_map_sg(struct device *dev, struct scatterlist *sg, int nents, enum dma_data_direction direction)
    //dma_addr_t sg_dma_address(struct scatterlist *sg);
    //Returns the bus (DMA) address from this scatterlist entry.

    //unsigned int sg_dma_len(struct scatterlist *sg);
    //Returns the length of this buffer.

    PRINTKM( DMA, ( PKTD "prep_sg_list() dto p%p xfer 0x%x ioobj->nbytes %lu isg %lu max_entr %u\n", ioobj->device_index, dto, xfer, ioobj->nbytes, ioobj->isg, max_entries ) );
    while( xfer > 0 )
    {
        //PRINTKM(DMA,(PKTD " ScatterGatherList next element %d xfer 0x%x\n", ioobj->device_index, isg, xfer ));

        elen = ( int )( ioobj->sg[ioobj->isg].length );
        do
        {
            address = sg_dma_address( &ioobj->sg[ioobj->isg] );
            address &= ~3;
            translation_table[isg % max_entries] = ( u64 )address;
            //PRINTKM(DMA,(PKTD " ScatterGatherList: elem %d addr 0x%x len 0x%x\n", ioobj->device_index, isg, address, elen ));
            elen -= PAGE_SIZE;
            xfer -= PAGE_SIZE;
            ++isg;
        }
        while( elen > 0 && isg < max_entries );
        ++ioobj->isg;
    }
}

//-------------------------------------------------------------------------------------------
int setup_dmatransfer_object_direct( struct hyperion_device* device, u_long i )
//-------------------------------------------------------------------------------------------
{
    struct dma_transfer_object* dto = device->pdma_object[i];
    void* dmactrl_base = ( void* )device->register_base.DMAControllerVideoIn[i];
    long result = 0;

    dto->dma_controller_videoin = ( DMA_CONTROLLER* )DMAController( NULL, dmactrl_base, TRUE );
    dto->dma_controller_pci = NULL;
    dto->rw_queue = device->pqueues[i];
    if( !dto->dma_controller_videoin )
    {
        PRINTKM( MOD, ( PKTD " unable to allocate dmacontroller struct index %lu", device->index, i ) );
        return -ENOMEM;
    }
    dto->avalon_to_pci_table.size = MAX_LENGTH / MAX_PARALLEL_TRANSFER;
    dto->avalon_to_pci_table.base = device->memory_base[0].base + PCI_MEM_AVALON_TO_PCI_TRANSLATION_TABLE + ( i * ( TRANSLATION_TABLE_ELEMENTS / MAX_PARALLEL_TRANSFER ) * sizeof( u64 ) );
    dto->frame_counter = 0;
    dto->presult_queue = &device->result_queue;
    dto->objid = i;
    result = SetDMADestinationParameter( dto->dma_controller_videoin,
                                         AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE + ( dto->objid * ( dto->avalon_to_pci_table.size ) ),
                                         dto->avalon_to_pci_table.size,
                                         2 );
    if( !result )
    {
        DMAControllerDestruct( &dto->dma_controller_videoin );
        //PRINTKM(MOD,(PKTD "unable to allocate DMAControllerVideoIn destination descriptor", ioobj->device_index );
        result = -ENOMEM;
    }

    ResetDMAController( dto->dma_controller_videoin );
    return 0;
}

//-------------------------------------------------------------------------------------------

#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,19)
irqreturn_t hyperion_interrupt_direct( int irq, void* dev_id, struct pt_regs* fake )
#else
// The prototype for interrupt handler functions has changed in 2.6.19.
// In short, the regs argument has been removed, since almost nobody used it.
// Any interrupt handler which needs the pre-interrupt register state can use get_irq_regs() to obtain it.
irqreturn_t hyperion_interrupt_direct( int irq, void* dev_id )
#endif
//-------------------------------------------------------------------------------------------
{
    return IRQ_NONE;
}

//-------------------------------------------------------------------------------------------
void hyperion_do_tasklet_direct( unsigned long index )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device = get_hyperion_device( index );
    struct dma_transfer_object* dto;
    TItem dma_res;
    unsigned char read_ok;
    struct io_object* ioobj;
    unsigned long opc_stat;

    PRINTKM( INTR, ( PKTD " hyperion_do_tasklet_direct( device_index %d )\n", ( int )index, ( int )index ) );
    do
    {
        _PIPE_READ( device->dma_interrupt_result, &dma_res, read_ok );
        if( read_ok == 0 )
        {
            break;
        }
        dto = dma_res.dto;
        PRINTKM( INTR, ( PKTD "hyp_tasklet() get intr result item ok %d intstat 0x%lx dto p%p\n", device->index, read_ok, dma_res.int_stat, dto ) );
        ioobj = get_current_iocb( dto->rw_queue );
        if( ioobj != NULL )
        {
            // test if ioobj is cancelled
            //if( ioobj->canceled )
            //{
            DMAInterruptDirectTransfer( dto->dma_controller_videoin, dma_res.int_stat );
            opc_stat = ( dma_res.int_stat & OPCODE_STATUS_MSK ) >> 28;
            if( opc_stat & opcSStartNext )
            {
//              PIRP nextIrp = GetNextIrp( dto->RWQueue );
//              DoTraceMessage( TRACELEVELDMATRANSFER, "Next Irp p%p", nextIrp );
//
//#if USE_START_NEXT_TRANSFER_BEFORE_CURRENT_COMPLETE
//                  if( nextIrp != NULL )
//                  {
//                      (*pie->AdapterObject->DmaOperations->FreeMapRegisters)
//                          (pie->AdapterObject, pie->regbase, pie->nMapRegistersAllocated);
//                      (*pie->AdapterObject->DmaOperations->PutDmaAdapter)(pie->AdapterObject);
//
//                      KIRQL oldirql;
//                      KeRaiseIrql( DISPATCH_LEVEL, &oldirql );
//                      StartIo( fdo, nextIrp );
//                      KeLowerIrql( oldirql );
//
//                  }
//#endif
                continue;
            }
            else if( opc_stat & opcSTransferReady )
            {
                PRINTKM( INTR, ( PKTD "hyp_tasklet() found opc_stat transfer ready write to resultq\n", device->index ) );
                write_results_to_rq( ioobj, get_jiffies_64(), cerrSuccess );
            }
            else
            {
                continue;
            }

            if( dma_res.int_stat & ( 1 << IRQ_PSEUDO ) )
            {
                //write_results_to_rq( ioobj, get_jiffies_64(), cerrDMAAborted );
                //dto->transfer_aborted = TRUE;
                continue;
            }
            /*}
            else
                //dma canceled
                write_results_to_rq( ioobj, get_jiffies_64(), cerrDMACancelled );*/

            del_timer_sync( &ioobj->timer_iocb );
            run_next_iocb( dto->rw_queue );
            complete_request( ioobj, 0 );
            PRINTKM( IO, ( PKTD "%lu msec request completed buffer %p reqid %d channel %d completed %s\n", ioobj->device_index, ( ( jiffies * 1000 ) / HZ ), ioobj->buffer, ioobj->transfer_param.reqid, ioobj->transfer_param.inputchannel, current->comm ) );

        }
        else
        {
            PRINTKM( INTR, ( PKTD "hyp_tasklet() ioobj == NULL\n", device->index ) );
            break;
        }
        //else if( dma_res.pua != 0 )
        //{
        //  ReadDataFromSerial( dma_res.pua );
        //}

    }
    while( 1 );
}

//-------------------------------------------------------------------------------------------
void jit_timer_cancel_direct( unsigned long arg )
//-------------------------------------------------------------------------------------------
{
    struct io_object* ioobj = ( struct io_object* )arg;
    struct dma_transfer_object* dto = ( struct dma_transfer_object* )ioobj->dma_transfer_object;
    PRINTKM( IO, ( PKTD "timer_cancel( arg 0x%lx ) %lu pid %6i proc %i %s\n", ioobj->device_index, arg, jiffies, current->pid, smp_processor_id(), current->comm ) );
    StopTransfer( dto->dma_controller_videoin, dsDone );
    ResetDMAController( dto->dma_controller_videoin );
    write_results_to_rq( ioobj, get_jiffies_64(), cerrTimeout );
    run_next_iocb( dto->rw_queue );
    complete_request( ioobj, 0 );
}

//-------------------------------------------------------------------------------------------
void setup_transfer_and_start_direct( struct io_object* ioobj, int index )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device = ( struct hyperion_device* )ioobj->iocb->ki_filp->private_data;
    struct transfer_parameter* tp = &ioobj->transfer_param;
    struct dma_transfer_object* dto = device->pdma_object[index];
    unsigned long timeout_jiffies;

    if( tp->reqaction == raiSnapRequest )
    {
        device->register_base.AoiXStart[tp->inputchannel]->xstart = tp->xstart;
        device->register_base.AoiXStop[tp->inputchannel]->xstop = tp->xstop;
        device->register_base.AoiYStart[tp->inputchannel]->ystart = tp->ystart;
        device->register_base.AoiYStop[tp->inputchannel]->ystop = tp->ystop;
        device->register_base.MuxCtrl[tp->inputchannel]->line_scan = !tp->scanmode;
        device->register_base.MuxCtrl[tp->inputchannel]->dval = tp->datavalidenable;

        ioobj->num_xfer = 0;
        ioobj->nbytes = ioobj->transfer_param.transferlength;
        ioobj->isg = 0;
        dto->xfer = 0;
        PRINTKM( DMA, ( PKTD "setup_transfer_and_start_direct() reqId %d, timeout %d transfer_aborted %d\n", ioobj->device_index, tp->reqid, tp->timeoutmsec, dto->transfer_aborted ) );
        SetXferTotal( dto->dma_controller_videoin, ioobj->nbytes );
        set_mux_data( device, &dto->mux_seq, tp->inputchannel );
        if( dto->transfer_aborted == FALSE )
        {
            PRINTKM( DMA, ( PKTD "transferready for prepare %d transfer in proc %d\n", ioobj->device_index, CanTransferPrepared( dto->dma_controller_videoin ), ioobj->transfer_in_process ) );
            //if( CanTransferPrepared( dto->dma_controller_videoin) && ioobj->transfer_in_process == FALSE )
            {
                ioobj->transfer_in_process = TRUE;
                PrepareDirectTransfer( dto->dma_controller_videoin, ( void* )ioobj, tp->scanmode );
                timeout_jiffies = msecs_to_jiffies( tp->timeoutmsec );
                init_timer( &ioobj->timer_iocb );
                ioobj->timer_iocb.function = jit_timer_cancel_direct;
                ioobj->timer_iocb.expires = ( jiffies + timeout_jiffies );
                ioobj->timer_iocb.data = ( unsigned long )ioobj;
                PRINTKM( DMA, ( PKTD "init timer jiffies %lu timeout_jiffies %lu data 0x%lx ioobj %p\n", ioobj->device_index, jiffies, timeout_jiffies, ioobj->timer_iocb.data, ioobj ) );
                ioobj->camera_status.ScanPixLine0 = device->register_base.ScanPixLine0[tp->inputchannel]->value;
                ioobj->camera_status.ScanPixLine1 = device->register_base.ScanPixLine1[tp->inputchannel]->value;
                ioobj->camera_status.ScanLines = device->register_base.ScanLines[tp->inputchannel]->value;
                add_timer( &ioobj->timer_iocb );
                StartTransfer( dto->dma_controller_videoin );
            }
        }
        else
        {
            write_results_to_rq( ioobj, get_jiffies_64(), cerrDMAAborted );
            run_next_iocb( dto->rw_queue );
            complete_request( ioobj, 0 );
            PRINTKM( DMA, ( PKTD "request aborted empty queue jiffies %lu\n", ioobj->device_index, jiffies ) );
        }
    }
    else if( tp->reqaction == raiSnapAbort )
    {
        ResetDMAController( dto->dma_controller_pci );
        write_results_to_rq( ioobj, get_jiffies_64(), cerrResultPacketAbort );
        dto->transfer_aborted = 0;
        run_next_iocb( dto->rw_queue );
        complete_request( ioobj, 0 );
        PRINTKM( DMA, ( PKTD "request action abort found, signal this packet jiffies %lu\n", ioobj->device_index, jiffies ) );
    }
    else
        ;///< todo this request has no action defined, remove it
}


//-------------------------------------------------------------------------------------------
void abort_transfer_direct( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    struct pci_dev* pdev;
    struct list_head* head;
    DECLARE_WAIT_QUEUE_HEAD( wq );
    int flag = 0;
    u32 videoin_dma_config[2];

    //PRINTKM(MOD,(PKTD "abort_transfer_direct()\n", device->index));
    pdev = pci_find_slot( device->bus, device->devfunc );
    if( pdev == NULL )
    {
        PRINTKM( MOD, ( PKTD "abort_transfer_direct() no pci_device found\n", device->index ) );
        return;
    }

    _WRITE_BIT( device->pdma_object[0]->dma_controller_videoin->pWriteRegister, InterruptEnable, PIE, 1 );
    head = &device->pqueues[0]->head;
    while( !list_empty( head ) )
    {
        wait_event_interruptible_timeout( wq, list_empty( head ), msecs_to_jiffies( 10 ) );
    }
    _WRITE_BIT( device->pdma_object[0]->dma_controller_videoin->pWriteRegister, InterruptEnable, PIE, 1 );
    head = &device->pqueues[1]->head;
    while( !list_empty( head ) )
    {
        wait_event_interruptible_timeout( wq, list_empty( head ), msecs_to_jiffies( 10 ) );
    }

    wait_event_interruptible_timeout( wq, ( flag > 0 ), msecs_to_jiffies( 10 ) );
    videoin_dma_config[0] = _READ_REG( device->pdma_object[0]->dma_controller_videoin->pWriteRegister, Control );
    videoin_dma_config[1] = _READ_REG( device->pdma_object[1]->dma_controller_videoin->pWriteRegister, Control );
    device->register_base.SystemCtrl->local_reset = RESET_HYPERION_DATA;
    if( pci_write_config_dword( pdev, PCI_BASE_ADDRESS_0, device->pci_cfg_reg.baseaddr ) )
    {
        PRINTKM( MOD, ( PKTD "abort_transfer_direct() can't write PCI_BASE_ADDRESS_0 register\n", device->index ) );
        return;
    }
    if( pci_write_config_dword( pdev, PCI_COMMAND, device->pci_cfg_reg.command ) )
    {
        PRINTKM( MOD, ( PKTD "abort_transfer_direct() can't write PCI_COMMAND register\n", device->index ) );
        return;
    }
    if( pci_write_config_dword( pdev, PCI_INTERRUPT_LINE, device->pci_cfg_reg.interrupt ) )
    {
        PRINTKM( MOD, ( PKTD "abort_transfer_direct() can't write PCI_INTERRUPT_LINE register\n", device->index ) );
        return;
    }
    if( pci_write_config_dword( pdev, PCI_CACHE_LINE_SIZE, device->pci_cfg_reg.latency ) )
    {
        PRINTKM( MOD, ( PKTD "abort_transfer_direct() can't write PCI_CACHE_LINE_SIZE register\n", device->index ) );
        return;
    }
    ResetDMAController( device->pdma_object[0]->dma_controller_videoin );
    ResetDMAController( device->pdma_object[1]->dma_controller_videoin );
    device->register_base.SystemCtrl->base_ch1 = 1;
    device->register_base.MuxCtrl[0]->aoi_en = 1;
    device->register_base.MuxCtrl[1]->aoi_en = 1;
    device->pdma_object[0]->mux_seq.changed = TRUE;
    device->pdma_object[1]->mux_seq.changed = TRUE;
    _WRITE_REG( device->pdma_object[0]->dma_controller_videoin->pWriteRegister, Control, videoin_dma_config[0] );
    _WRITE_REG( device->pdma_object[1]->dma_controller_videoin->pWriteRegister, Control, videoin_dma_config[1] );

}//abort_transfer_direct
