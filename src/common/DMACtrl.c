//DMACtrl.cpp includes all needed functions for controlling dma-transfer
#ifdef DRIVER
#   include "stddcls.h"
#   include "driver.h"
#   include "DMACtrl.h"
#   include "DMACtrl.tmh"
#else //linux
#   include <linux/fs.h>
#   include <linux/types.h>
#   include <linux/version.h>
#   ifndef KERNEL_VERSION
#       define KERNEL_VERSION(a,b,c) (((a) << 16) | ((b) << 8) | (c))
#   endif
#   include "matrix_types.h"
#   include "hyperion.h"
#   include "DMACtrl.h"
#   include "read_write.h"
#endif

//----------------------------------------------------------------------------------------------
#if defined(linux) || defined(__linux) || defined(__linux__)
#   define WRITE_REG32(reg,val) iowrite32(val,(void __iomem *)(reg))
#   define READ_REG32(reg) ioread32((void __iomem *)(reg))
#else
#   define WRITE_REG32(reg,val) WRITE_REGISTER_ULONG((PULONG)reg,val)
#   define READ_REG32(reg) READ_REGISTER_ULONG((PULONG)reg)
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

#ifdef DRIVER
#   define _WMB()
#else
#   define _WMB() wmb()
#endif
static char* Actionstring[] =
{
    "AddSourceAddress",
    "AddAddress",
    "NextDescriptor",
    "AddCmdOffsetLength",
    "InterruptIncrementReadData",
    "InterruptIncrementWriteQuota",
    "CompleteThisTransfer",
    "SetupTransfer",
    "res0",
    "res1",
    "res2",
    "res3",
    "res4",
    "res5",
    "res6",
    "res7",
    "res8",
};

//----------------------------------------------------------------------------------------------
#define _ADD_TO(oc) WRITE_REG32(pdma->cmd_fifo, opc)

//----------------------------------------------------------------------------------------------
/// \brief constructor for DMA-Controller access
/// \param context the device_context can be any struct for building the scatter/gather pciaddr-list if dmacontroller should send data to pcibus
/// \param membase the pointer you get from MmMapIoSpace()
void* DMAController( void* context, void* membase, int reg_index, HYPERION_BASE_REGISTER_DEF* reg_def, PPREPARE_SCATTER_GATHER_LIST prepare_sg_list, enum COMMAND_FIFO_DEFINITION cmd_list )
//----------------------------------------------------------------------------------------------
{
#ifdef DRIVER
    DMA_CONTROLLER* pdma = ( DMA_CONTROLLER* )ExAllocatePoolWithTag( NonPagedPool, sizeof( DMA_CONTROLLER ), 'dctl' );
    PUCHAR hyperion_base;
    DoTraceMessage( TRACELEVELDMACONTROLLER, " %s allocate dmacontroller struct p%p size 0x%x membase p%p", __FUNCTION__, pdma, sizeof( DMA_CONTROLLER ), membase );
#else
    DMA_CONTROLLER* pdma = ( DMA_CONTROLLER* )kmalloc( sizeof( DMA_CONTROLLER ), GFP_KERNEL );
    struct memory_space hyperion_base;
#endif
    if( pdma != 0 )
    {
#ifdef DRIVER
        RtlZeroMemory( pdma, sizeof( DMA_CONTROLLER ) );
        KeInitializeSpinLock( &pdma->lock );
        hyperion_base = ( unsigned char* )membase;
#else
        memset( pdma, 0, sizeof( DMA_CONTROLLER ) );
        spin_lock_init( &pdma->lock );
        hyperion_base.base = ( unsigned char* )membase;
#endif
        pdma->device_context = ( void* )context;
        pdma->mem_base = membase;
        pdma->register_index = reg_index;
        pdma->prepare_sg_list = prepare_sg_list;
        pdma->dma_reg.gen_control = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_CONTROL );
        pdma->dma_reg.version = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_CTRL_VERSION );
        pdma->dma_reg.transfer_ctrl_status = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_CTRL_STAT );
        pdma->dma_reg.interrupt_enable = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_INTR_ENABLE );
        pdma->dma_reg.interrupt_status = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_INTR_STAT_CLEAR );
        pdma->dma_reg.dma_status = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_DMA_CMD_STAT );
        pdma->dma_reg.dma_xfer_remain = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_DMA_XFER_REMAIN );
        pdma->dma_reg.dma_total_xfer = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_DMA_TOTAL_XFER );
        pdma->dma_reg.dma_status2 = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_DMA_CMD_STAT2 );
        pdma->dma_reg.dma_xfer_remain2 = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_DMA_XFER_REMAIN2 );
        pdma->dma_reg.dma_total_xfer2 = ( unsigned int* )REG_POINTER( hyperion_base, reg_def, reg_index, OFF_VIDEO_IN_DMA_TOTAL_XFER2 );
        pdma->version = READ_REG32( pdma->dma_reg.version );
        pdma->page_size = PAGE_SIZE;
        if( ( pdma->version & VERSIONS_NUMBER_MSK ) >= TRIGGER_MODE_AVAILABLE_VERSION )
        {
            if( cmd_list == cfdCommandFifo1 )
            {
                pdma->cmd_fifo = pdma->dma_reg.dma_status2;
                pdma->clear_cmd_fifo_msk = CLEAR_COMMAND_FIFO_1;
                pdma->cmd_xfer_remain = pdma->dma_reg.dma_xfer_remain2;
                pdma->cmd_xfer_total = pdma->dma_reg.dma_total_xfer2;
                pdma->cmd_fifo_sel = CMD_FIFO_SELECT;
            }
            else
            {
                pdma->cmd_fifo = pdma->dma_reg.dma_status ;
                pdma->clear_cmd_fifo_msk = CLEAR_COMMAND_FIFO;
                pdma->cmd_xfer_remain = pdma->dma_reg.dma_xfer_remain;
                pdma->cmd_xfer_total = pdma->dma_reg.dma_total_xfer;
                pdma->cmd_fifo_sel = 0;
            }
        }
        else
        {
            pdma->cmd_fifo = pdma->dma_reg.dma_status ;
            pdma->clear_cmd_fifo_msk = CLEAR_COMMAND_FIFO;
            pdma->cmd_xfer_remain = pdma->dma_reg.dma_xfer_remain;
            pdma->cmd_xfer_total = pdma->dma_reg.dma_total_xfer;
            pdma->cmd_fifo_sel = 0;
        }
        ResetDMAController( pdma );
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, MASTER_BURST_LEN );
        DisableInterrupts( pdma );
#ifdef DRIVER
        DoTraceMessage( TRACELEVELDMACONTROLLER, " %s controller initialize successful version %x pdma %p", __FUNCTION__, pdma->version, pdma );
#else
        PRINTKM( MOD, ( PKTD " %s() controller initialize successful version %x pdma %p\n", -1, __FUNCTION__, pdma->version, pdma ) );
#endif
    }
    else
    {
#ifdef DRIVER
        DoTraceMessage( TRACELEVELDMACONTROLLER, " %s controller not initialized pdma p%p", __FUNCTION__, pdma );
#else
        PRINTKM( MOD, ( PKTD " %s controller not initialized pdma %p\n", -1, __FUNCTION__, pdma ) );
#endif
    }
    return ( void* )pdma;
}

//----------------------------------------------------------------------------------------------
/// \brief destructor for dma-controller access, free all allocated resources in the dma_controller struct
void DMAControllerDestruct( DMA_CONTROLLER** ppdma )
//----------------------------------------------------------------------------------------------
{
    DMA_CONTROLLER* pdma = 0;
    if( *ppdma != 0 )
    {
        pdma = *ppdma;
        StopTransfer( pdma );
        ResetDMAController( pdma );
#ifdef DRIVER
        ExFreePool( *ppdma );
#else
        kfree( *ppdma );
#endif
        ppdma = 0;
    }
#ifdef DRIVER
#else
    PRINTKM( MOD, ( PKTD " DMAControllerDestruct()--> controller removed successfully\n", pdma->device_index ) );
#endif
}

//----------------------------------------------------------------------------------------------
void SetPCIPageSize( DMA_CONTROLLER* pdma, unsigned int page_size )
//----------------------------------------------------------------------------------------------
{
    if( pdma != 0 && page_size > 0 )
    {
        pdma->page_size = page_size;
    }
}

//----------------------------------------------------------------------------------------------
int GetDMAControllerVersion( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
    if( pdma != 0 )
    {
        return pdma->version;
    }
    else
    {
        return -1;
    }
}

//----------------------------------------------------------------------------------------------
void WriteOPCTransfer( DMA_CONTROLLER* pdma, unsigned int xferlen, unsigned char interrupt_enable, char* msg )
//----------------------------------------------------------------------------------------------
{
    unsigned int opc = ( opcExecuteDMATransfer << dcotOpcode ) | ( ( interrupt_enable & 1 ) << dcotInterruptEnable ) | ( xferlen >> 2 );
    _ADD_TO( opc );
#ifdef DRIVER
    DoTraceMessage( TRACELEVELDMAOPCODE, " %s, %I64u, opc 0x%x %s %p", __FUNCTION__, GetTimeMicroSec(), opc, msg, ( void* )pdma->cmd_fifo );
#else
    PRINTKM( DMA, ( PKTD " opc 0x%x %s\n", pdma->device_index, opc, msg ) );
#endif
}


//----------------------------------------------------------------------------------------------
void WriteOPCLongTransfer( DMA_CONTROLLER* pdma, unsigned int xferlen, unsigned char interrupt_enable, unsigned char opc_status, unsigned char opc_status_reset, char* msg )
//----------------------------------------------------------------------------------------------
{
    unsigned int opc, xfer = ( xferlen >> 2 ), max_transfer = ( ( 1 << BIT_COUNT_LONG_TRANSFER_LENGTH ) - 1 ) & ~TRANSFER_CORRECTION_MSK;

    while( xfer > max_transfer )
    {
        opc = ( opcExecuteLongDMATransfer << dcotOpcode ) | max_transfer;
        xfer -= max_transfer;
        _ADD_TO( opc );
#ifdef DRIVER
        DoTraceMessage( TRACELEVELDMAOPCODE, " %s, %I64u, opc 0x%x %s %p", __FUNCTION__, GetTimeMicroSec(), opc, msg, ( void* )pdma->cmd_fifo );
#else
        PRINTKM( DMA, ( PKTD " opc 0x%x %s\n", pdma->device_index, ( unsigned int )opc, msg ) );
#endif
    }
    opc = ( opcExecuteLongDMATransfer << dcotOpcode ) | ( ( interrupt_enable & 1 ) << dcotInterruptEnable ) |
          ( opc_status << dcotStatusSet ) | ( opc_status_reset << dcotStatusReset ) | ( xfer );
    _ADD_TO( opc );
    _WMB();
#ifdef DRIVER
    DoTraceMessage( TRACELEVELDMAOPCODE, " %s, %I64u, opc 0x%x %s %p ", __FUNCTION__, GetTimeMicroSec(), opc, msg, ( void* )pdma->cmd_fifo );
#else
    PRINTKM( DMA, ( PKTD " opc 0x%x %s\n", pdma->device_index, ( unsigned int )opc, msg ) );
#endif
}

//----------------------------------------------------------------------------------------------
void WriteOPCAddr( DMA_CONTROLLER* pdma, unsigned int addr, enum FIFO_OPERATION_CODE opcode, char* msg )
//----------------------------------------------------------------------------------------------
{
    unsigned int opc = ( opcode << dcodOpcode ) | ( addr );
    _ADD_TO( opc );
    _WMB();
#ifdef DRIVER
    DoTraceMessage( TRACELEVELDMAOPCODE, " %s, %I64u, opc 0x%x %s %p ", __FUNCTION__, GetTimeMicroSec(), opc, msg, ( void* )pdma->cmd_fifo );
#else
    PRINTKM( DMA, ( PKTD " opc 0x%x %s\n", pdma->device_index, opc, msg ) );
#endif
}

//----------------------------------------------------------------------------------------------
void WriteOPCWaitFrameEnd( DMA_CONTROLLER* pdma, unsigned char waitFrame, unsigned char waitLine, unsigned char clearDataFifo, char* msg )
//----------------------------------------------------------------------------------------------
{
    unsigned int opc = ( opcWaitFrameEnd << dcowOpcode ) | ( ( clearDataFifo & 1 ) << dcowClearDataFifo ) | ( ( waitLine & 1 ) << dcowWaitLine ) | ( waitFrame & 1 );
    _ADD_TO( opc );
    _WMB();
#ifdef DRIVER
    DoTraceMessage( TRACELEVELDMAOPCODE, " %s, %I64u, opc 0x%x %s %p", __FUNCTION__, GetTimeMicroSec(), opc, msg, ( void* )pdma->cmd_fifo );
#else
    PRINTKM( DMA, ( PKTD " opc 0x%x %s\n", pdma->device_index, opc, msg ) );
#endif
}


//----------------------------------------------------------------------------------------------
int PrepareScatterGatherList( DMA_CONTROLLER* pdma, unsigned long nbytes, unsigned long isg, unsigned int* offset )
//----------------------------------------------------------------------------------------------
{
    if( pdma->device_context != NULL && pdma->prepare_sg_list != NULL )
    {
        return ( *pdma->prepare_sg_list )( pdma->device_context, nbytes, isg, offset );
    }
    else
    {
        return dcecNoError;
    }
}

//----------------------------------------------------------------------------------------------
enum DMA_COMMAND_ACTION DoSetupTransfer( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
    if( pdma->opcode_control & opccClearDataFifo )
    {
        WriteOPCWaitFrameEnd( pdma, pdma->wait_frame, pdma->wait_line, TRUE, " dt" );
    }
    pdma->wait_frame = 0;
    pdma->wait_line = 0;
    pdma->write_buffer.write &= ~( PAGE_SIZE - 1 );
    return NextDescriptor;
}

//----------------------------------------------------------------------------------------------
int HandleDMAEventDirectTransfer( DMA_CONTROLLER* pdma, enum DMA_COMMAND_ACTION event, unsigned int int_stat )
//----------------------------------------------------------------------------------------------
{
    int result = dcecNoError;
    unsigned char is_setup_action = FALSE;
    enum DMA_COMMAND_ACTION action = event;
#ifdef DRIVER
    KIRQL oldirql;
    KeAcquireSpinLock( &pdma->lock, &oldirql );
#else
    unsigned long irqflags;
    spin_lock_irqsave( &pdma->lock, irqflags );
#endif

    while( 1 )
    {
#ifdef DRIVER
        DoTraceMessage( TRACELEVELDMACONTROLLER, " %s action %s int_stat 0x%x", __FUNCTION__, Actionstring[action], int_stat );
#else
        PRINTKM( MOD, ( PKTD " action[%d] %s int_stat 0x%x\n", pdma->device_index, action, Actionstring[action], ( unsigned int )int_stat ) );
#endif
        switch( action )
        {
        case AddSrcAddress:
            break;
        case SetupTransfer:
            {
                is_setup_action = TRUE;
                action = DoSetupTransfer( pdma );
            }
            continue;
        case NextDescriptor:
            {
#ifdef DRIVER
                DoTraceMessage( TRACELEVELDMACONTROLLER, " %s write->WriteQuotaAvailable %d total %d ", __FUNCTION__, pdma->write_buffer.write_quota_available, pdma->write_buffer.total_size );
                DoTraceMessage( TRACELEVELDMACONTROLLER, " %s read->read_data_available %d total %d ", __FUNCTION__, pdma->read_buffer.read_data_available, pdma->read_buffer.total_size );

#else
                PRINTKM( MOD, ( PKTD " %s write->write_quota_available %d total %d\n", pdma->device_index, __FUNCTION__, pdma->write_buffer.write_quota_available, pdma->write_buffer.total_size ) );
#endif
                if( pdma->write_buffer.write_quota_available /*&& (pdma->read_buffer ? (pdma->read_buffer->read_data_available > 0) : 1)*/ )
                {
                    action = AddAddress;
                    continue;
                }
                else
                {
                    break;
                }
            }
        case AddAddress:
            {
                /*if( pdma->read_buffer )
                    WriteOPCAddr( pdma, (pdma->read_buffer->Read >> 3), opcSourceAddress, " " );*/
                WriteOPCAddr( pdma, ( pdma->write_buffer.write >> 3 ), opcDestinationAddress, " " );
                action = AddCmdOffsetLength;
                continue;
            }
        case AddCmdOffsetLength:
            {
                unsigned long xfer, xfer_write, split_transfer_size = 0;
                unsigned char interrupt_enable = 1, opc_stat_reset, opc_stat = opcsfUnlock;
                unsigned int offset = 0;

                if( READ_REG32( pdma->dma_reg.dma_status ) & COMMAND_FIFO_FULL )
                    //dcecCommandFifoFull
                {
                    break;
                }
                xfer_write = pdma->write_buffer.block_size;
                if( PrepareScatterGatherList( pdma, pdma->write_buffer.block_size, ( pdma->write_buffer.write / pdma->page_size ), &offset ) != dcecNoError )
                {
                    result = dcecTerminateTransfer;
                    break;
                }
                if( offset != 0 )
                {
                    WriteOPCAddr( pdma, ( pdma->write_buffer.write + offset ) >> 3, opcDestinationAddress, " " );
                    xfer_write -= offset;
                }
                xfer = pdma->nbytes < xfer_write ? pdma->nbytes : xfer_write;

                pdma->read_buffer.read_data_available -= pdma->read_buffer.block_size;
                pdma->read_buffer.read += pdma->read_buffer.block_size;
                if( pdma->read_buffer.read >= ( pdma->read_buffer.address + pdma->read_buffer.total_size ) )
                {
                    pdma->read_buffer.read = pdma->read_buffer.address;
                }

                pdma->write_buffer.write_quota_available -= pdma->write_buffer.block_size;
                pdma->write_buffer.write += pdma->write_buffer.block_size;
                if( pdma->write_buffer.write >= ( pdma->write_buffer.address + pdma->write_buffer.total_size ) )
                {
                    pdma->write_buffer.write = pdma->write_buffer.address;
                }

                if( xfer <= pdma->nbytes )
                {
                    pdma->nbytes -= xfer;
                }
                else
                {
#ifdef DRIVER
                    DoTraceMessage( TRACELEVELERROR, " %s, %I64u, detected nbytes 0x%x < xfer 0x%x\n", __FUNCTION__, GetTimeMicroSec(), pdma->nbytes, xfer );
#endif
                    pdma->nbytes = 0;
                }
                if( pdma->opcode_control & opccPrepareNextWithinFirstTransfer )
                {
                    if( is_setup_action )
                    {
                        opc_stat |= opcsfStartNext;
                        split_transfer_size = xfer >> 1;
                        //split_transfer_size = 0x1000;
                    }
                }
                else if( pdma->opcode_control & opccPrepareNextWithinLastTransfer )
                {
                    if( ( is_setup_action && pdma->transfer_block_count == 1 ) )
                    {
                        //one block transfer
                        split_transfer_size = xfer >> 1;
                    }
                    else if( pdma->transfer_block_count == 2 )
                    {
                        //if penultimate block is reached
                        split_transfer_size = xfer >> 1;
                    }
                    else
                        ;
                }
                else
                {
                    ;
                }
                if( split_transfer_size )
                {
                    opc_stat = opcsfStartNext;
                    opc_stat_reset = opcsfReset & ~opc_stat;
                    WriteOPCLongTransfer( pdma, split_transfer_size, interrupt_enable, opc_stat, opc_stat_reset, " " );
                    xfer -= split_transfer_size;
                    opc_stat = opcsfUnlock;
                    split_transfer_size = 0;
                }
                opc_stat |= ( pdma->nbytes == 0 ) ? opcsfTransferReady : 0;
                opc_stat_reset = opcsfReset & ~opc_stat;
                WriteOPCLongTransfer( pdma, xfer, interrupt_enable, opc_stat, opc_stat_reset, " " );
                --pdma->transfer_block_count;
                is_setup_action = FALSE;
                action = CompleteThisTransfer;
                continue;
            }
        case CompleteThisTransfer:
            {
#ifdef DRIVER
                DoTraceMessage( TRACELEVELDMACONTROLLER, " %s nbytes 0x%x ", __FUNCTION__, pdma->nbytes );
#else
                PRINTKM( MOD, ( PKTD " nbytes 0x%x context p%p\n", pdma->device_index, pdma->nbytes, pdma->device_context ) );
#endif
                if( pdma->nbytes > 0 )
                {
                    action = NextDescriptor;
                    continue;
                }
                else
                {
                    break;
                }
            }
        case IncrementReadData:
            {
                pdma->write_buffer.read_data_available += pdma->write_buffer.block_size;
#ifdef DRIVER
                DoTraceMessage( TRACELEVELDMACONTROLLER, " %s read_data_available %d total %d ", __FUNCTION__, pdma->write_buffer.read_data_available, pdma->write_buffer.total_size );

#else
#endif
                break;
            }
        case IncrementWriteQuota:
            {
                pdma->write_buffer.write_quota_available += pdma->write_buffer.block_size;
                if( pdma->write_buffer.write_quota_available > pdma->write_buffer.total_size )
                {
                    pdma->write_buffer.write_quota_available = pdma->write_buffer.total_size;
                }
                action = CompleteThisTransfer;
                continue;
            }
        }
        break;
    }
#ifdef DRIVER
    DoTraceMessage( TRACELEVELDMACONTROLLER, " - %s ", __FUNCTION__ );
    KeReleaseSpinLock( &pdma->lock, oldirql );
#else
    spin_unlock_irqrestore( &pdma->lock, irqflags );
#endif
    return result;
}


//----------------------------------------------------------------------------------------------
void SetDMAControllerBuffer( DMA_CONTROLLER* pdma, unsigned int address, unsigned int total_size, unsigned char read )
//----------------------------------------------------------------------------------------------
{
    struct SBuffer* buffer;
    if( pdma != 0 )
    {
        buffer = read ? &pdma->read_buffer : &pdma->write_buffer;
        buffer->address = address;
        buffer->total_size = total_size;
        buffer->block_size = total_size / TRANSFERBLOCKS_PER_CHANNEL;
        ResetDMACtrlBuffer( pdma );
    }
}

//----------------------------------------------------------------------------------------------
/// \brief set the bytes to transfer in this stage
void SetXferTotal( DMA_CONTROLLER* pdma, unsigned int xfer_total )
//----------------------------------------------------------------------------------------------
{
    if( pdma != 0 )
    {
        pdma->nbytes = xfer_total;
    }
}

//----------------------------------------------------------------------------------------------
void ResetDMACtrlBuffer( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
    if( pdma != 0 )
    {
        pdma->read_buffer.write_quota_available = pdma->read_buffer.total_size;
        pdma->read_buffer.read_data_available = 0;
        pdma->read_buffer.read = pdma->read_buffer.address;
        pdma->read_buffer.write = pdma->read_buffer.address;
#ifdef DRIVER
#else
        PRINTKM( DMA, ( PKTD " %s read_buffer totalsize %dkB\n", pdma->device_index, __FUNCTION__, pdma->read_buffer.total_size / 1024 ) );
#endif
        pdma->write_buffer.write_quota_available = pdma->write_buffer.total_size;
        pdma->write_buffer.read_data_available = 0;
        pdma->write_buffer.read = pdma->write_buffer.address;
        pdma->write_buffer.write = pdma->write_buffer.address;
#ifdef DRIVER
#else
        PRINTKM( DMA, ( PKTD " %s write_buffer totalsize %dkB\n", pdma->device_index, __FUNCTION__, pdma->write_buffer.total_size / 1024 ) );
#endif
    }
}

//----------------------------------------------------------------------------------------------
void ResetDMAController( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
    if( pdma != 0 )
    {
#ifdef DRIVER
        DoTraceMessage( TRACELEVELDMACONTROLLER, " %s ResetDMAController(pdma %p)\n", __FUNCTION__, pdma );
#else
        PRINTKM( DMA, ( PKTD " ResetDMAController(pdma %p)\n", pdma->device_index, pdma ) );
#endif
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, ( MASTER_BURST_LEN | CLEAR_COMMAND_FIFO_1 | CLEAR_COMMAND_FIFO | CLEAR_DATA_FIFO | RESET_DMA ) );
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, MASTER_BURST_LEN );
        pdma->device_context = NULL;
        pdma->nbytes = 0;
        ResetDMACtrlBuffer( pdma );
    }
}

//----------------------------------------------------------------------------------------------
void ClearCommandFifo( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
#ifdef DRIVER
    ULONG m_ctrl;
#else
    u32 m_ctrl;
#endif
    if( pdma != 0 )
    {
        m_ctrl = READ_REG32( pdma->dma_reg.transfer_ctrl_status );
        m_ctrl |= pdma->clear_cmd_fifo_msk;
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, m_ctrl );
        m_ctrl &= ~pdma->clear_cmd_fifo_msk;
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, m_ctrl );
    }
}

//----------------------------------------------------------------------------------------------
void SynchronizeMediumChannel( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
    if( pdma != 0 )
    {
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, ( RESET_MEDIUM_CHANNEL | MASTER_BURST_LEN ) );
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, ( MASTER_BURST_LEN ) );
    }
}

//----------------------------------------------------------------------------------------------
void StartTransfer( DMA_CONTROLLER* pdma, unsigned int synchronize_medium_channel )
//----------------------------------------------------------------------------------------------
{
    unsigned int cmd_fifo_auto_switch;
#ifdef DRIVER
    DoTraceMessage( TRACELEVELDMACONTROLLER, " %s StartTransfer() ", __FUNCTION__ );
#else

#endif
    if( pdma != 0 )
    {
        cmd_fifo_auto_switch = ( pdma->opcode_control & opccCommandFifoAutoSwitchEnable ) ? ENABLE_CMD_LIST_AUTO_SWITCH | AUTO_CLEAR_CMD_FIFO : 0;
        WRITE_REG32( pdma->dma_reg.interrupt_enable, DATA_OVERFLOW_INTR | COMMAND_READY_INTR | COMMAND_ABORT_INTR );
        //if( synchronize_medium_channel )
        //{
        //  WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, (RESET_MEDIUM_CHANNEL | MASTER_BURST_LEN) );
        //  //delay;
        //}
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, ( cmd_fifo_auto_switch | MASTER_BURST_LEN | pdma->cmd_fifo_sel ) );
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, ( cmd_fifo_auto_switch | MASTER_BURST_LEN | DMA_MASTER_EN | pdma->cmd_fifo_sel ) );
    }
}

//----------------------------------------------------------------------------------------------
void SetWaitOfFrameEdge( DMA_CONTROLLER* pdma, unsigned int edge )
//----------------------------------------------------------------------------------------------
{
    unsigned int control_reg;
    if( pdma != 0 )
    {
        control_reg = READ_REG32( pdma->dma_reg.gen_control );
        control_reg &= ~DISABLE_BYTE_WRITE_ACCESS;
        if( edge )
        {
            control_reg |= START_WAIT_OF_FRAME_EDGE;
        }
        else
        {
            control_reg &= ~START_WAIT_OF_FRAME_EDGE;
        }
        WRITE_REG32( pdma->dma_reg.gen_control, control_reg );
    }
}

//----------------------------------------------------------------------------------------------
void StopTransfer( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
#ifdef DRIVER
    DoTraceMessage( TRACELEVELDMACONTROLLER, " %s StopTransfer() ", __FUNCTION__ );
#else
#endif
    if( pdma != 0 )
    {
        WRITE_REG32( pdma->dma_reg.transfer_ctrl_status, MASTER_BURST_LEN );
        WRITE_REG32( pdma->dma_reg.interrupt_enable, 0 );
    }
}

//----------------------------------------------------------------------------------------------
/// \brief enables the dma_controller interrupt
void EnableInterrupts( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
    if( pdma != 0 )
    {
        WRITE_REG32( pdma->dma_reg.interrupt_enable, COMMAND_READY_INTR );
    }
}

//----------------------------------------------------------------------------------------------
/// \brief disables the dma_controller interrupt
void DisableInterrupts( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
    unsigned int intr_stat;
    if( pdma != 0 )
    {
        WRITE_REG32( pdma->dma_reg.interrupt_enable, 0 );
        intr_stat = READ_REG32( pdma->dma_reg.interrupt_status );
        WRITE_REG32( pdma->dma_reg.interrupt_status, intr_stat );
    }
}

//----------------------------------------------------------------------------------------------
/// \brief returns the actual dma masterstatus register
unsigned int GetDMAMasterStatus( DMA_CONTROLLER* pdma )
//----------------------------------------------------------------------------------------------
{
    if( pdma != 0 )
    {
        return READ_REG32( pdma->dma_reg.dma_status );
    }
    else
    {
        return ( unsigned int ) - 1;
    }
}

//----------------------------------------------------------------------------------------------
int PrepareDirectTransfer( DMA_CONTROLLER* pdma, PVOID context, unsigned int scan_mode, unsigned int has_videoin_prop_changed, unsigned int trigger_control )
//----------------------------------------------------------------------------------------------
{
#ifdef DRIVER
    DoTraceMessage( TRACELEVELDMACONTROLLER, " %s PrepareDirectTransfer( %p %p %d)", __FUNCTION__, pdma, context, scan_mode );
#else
#endif
    if( pdma != 0 )
    {
        unsigned char dma_enabled = ( ( unsigned char )READ_REG32( pdma->dma_reg.transfer_ctrl_status ) & DMA_MASTER_EN );
        pdma->device_context = context;
        pdma->transfer_block_count = pdma->nbytes / pdma->write_buffer.block_size + 1;
        if( scan_mode == 1 )
        {
            pdma->wait_frame = 1;
            pdma->wait_line = 1;
        }
        else
        {
            pdma->wait_frame = 0;
            pdma->wait_line = 1;
        }
        pdma->opcode_control = 0;
        if( has_videoin_prop_changed || ( dma_enabled == FALSE ) || pdma->wait_frame || trigger_control == etsStartSignal )
        {
            pdma->opcode_control |= opccClearDataFifo;
        }

        if( trigger_control & ( etsStartSignalOverlap | etsStopSignal ) )
        {
            pdma->opcode_control |= ( opccPrepareNextWithinFirstTransfer | opccCommandFifoAutoSwitchEnable );
        }
        else
        {
            pdma->opcode_control |= opccPrepareNextWithinLastTransfer;
        }

        return HandleDMAEventDirectTransfer( pdma, SetupTransfer, 0 );
    }
    return FALSE;
}

//----------------------------------------------------------------------------------------------
int PrepareTransferRAMToPCI( DMA_CONTROLLER* pdma, PVOID context )
//----------------------------------------------------------------------------------------------
{
    if( pdma != 0 )
    {
        pdma->device_context = context;
        return HandleDMAEventDirectTransfer( pdma, NextDescriptor, 0 );
    }
    return FALSE;
}
