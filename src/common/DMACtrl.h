//----------------------------------------------------------------------------------------------
#ifndef DMA_CTRL_H
#define DMA_CTRL_H DMA_CTRL_H
//----------------------------------------------------------------------------------------------
#include "pipe.h"
#if defined(linux) || defined(__linux) || defined(__linux__)
#   include <linux/fs.h>
#endif
#include "HyperionRegister.h"
#include "HyperionProp.h"

//----------------------------------------------------------------------------------------------
//register definitions for the DMA controller Altera NII51006
#define _fillbits(n)    volatile unsigned int : (n)/3; volatile unsigned int : (n)-(n)/3
//----------------------------------------------------------------------------------------------
#define BIT_COUNT_TRANSFER_LENGTH 14
#define BIT_COUNT_OFFSET 14
#define BIT_COUNT_LONG_TRANSFER_LENGTH 20
#define VERSIONS_NUMBER 12
#define VERSIONS_NUMBER_MSK ((1<<VERSIONS_NUMBER)-1)
#define TRANSFER_CORRECTION_MSK 0xffff //reducing of max transfer size necessary to avoid any side effects

#define OPCODE_STATUS_MSK 0xf0000000
#define TRIGGER_MODE_AVAILABLE_VERSION 0x1B
#define RESET_MEDIUM_VERSION2 0x1f


//----------------------------------------------------------------------------------------------
// Operation code transfer definition
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
enum FIFO_OPERATION_CODE
//----------------------------------------------------------------------------------------------
{
    opcSourceAddress = 0,
    opcDestinationAddress = 1,
    opcExecuteDMATransfer = 2,
    opcExecuteLongDMATransfer = 3,
    opcWaitFrameEnd = 4,
    NUMOFOPC,
};

//----------------------------------------------------------------------------------------------
enum DMA_CONTROLLER_OPCODE_DESTINATION
//----------------------------------------------------------------------------------------------
{
    dcodAddress = 0,
    dcodOpcode = 29
};

//----------------------------------------------------------------------------------------------
enum DMA_CONTROLLER_OPCODE_WAIT
//----------------------------------------------------------------------------------------------
{
    dcowWaitFrame = 0,
    dcowWaitLine = 1,
    dcowClearDataFifo = 28,
    dcowOpcode = 29
};

//----------------------------------------------------------------------------------------------
enum DMA_CONTROLLER_OPCODE_TRANSFER
//----------------------------------------------------------------------------------------------
{
    dcotTransferLength = 0,
    dcotStatusReset = 20,
    dcotStatusSet = 24,
    dcotInterruptEnable = 28,
    dcotOpcode = 29
};


//----------------------------------------------------------------------------------------------
// misc. definition
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
enum DMA_CONTROLLER_ERROR_CODE
//----------------------------------------------------------------------------------------------
{
    dcecNoError = 0,
    dcecPhysAddrError,
    dcecNoReadDataAvailable,
    dcecTerminateTransfer,
    dcecTranslationTableElements,
    dcecCommandFifoFull
};

//----------------------------------------------------------------------------------------------
enum COMMAND_FIFO_DEFINITION
//----------------------------------------------------------------------------------------------
{
    cfdCommandFifo0 = 0,
    cfdCommandFifo1,
    cfdCommandFifoMaxList
};

//----------------------------------------------------------------------------------------------
enum OPERATION_CODE_STATUS_FLAG
//----------------------------------------------------------------------------------------------
{
    opcsfDefault = 0,
    opcsfUnlock = 1,
    opcsfStartNext = 2,
    opcsfTransferReady = 4,
    opcsfReset = 0xf,
    NUMOFOPCS,
};

//----------------------------------------------------------------------------------------------
enum OPERATION_CODE_CONTROL
//----------------------------------------------------------------------------------------------
{
    opccClearDataFifo = 1,
    opccPrepareNextWithinFirstTransfer = 2,
    opccPrepareNextWithinLastTransfer = 4,
    opccPrepareNextAfterLastTransferComplete = 8,
    opccCommandFifoAutoSwitchEnable = 0x10
};

//----------------------------------------------------------------------------------------------
enum DMA_COMMAND_ACTION
//----------------------------------------------------------------------------------------------
{
    AddSrcAddress = 0,
    AddAddress,
    NextDescriptor,
    AddCmdOffsetLength,
    IncrementReadData,
    IncrementWriteQuota,
    CompleteThisTransfer,
    SetupTransfer,
};

//----------------------------------------------------------------------------------------------
//version
//----------------------------------------------------------------------------------------------
#define CONTROLLER_VERSION          (0xFFF<<0)
#define MEDIUM_SUPPORT              (1<<12)
#define FULL_SUPPORT                (1<<13)
//----------------------------------------------------------------------------------------------
//transfer_ctrl_status
//----------------------------------------------------------------------------------------------
#define DISABLE_BYTE_WRITE_ACCESS   (1<<0)
#define START_WAIT_OF_FRAME_EDGE    (1<<1)
/*#define SYNCMODE_CC1              (3<<4)
#define SYNCSEL_CC1                 (7<<6)
#define SYNCMODE_CC2                (3<<9)
#define SYNCSEL_CC2                 (7<<11)
#define SYNCMODE_CC3                (3<<14)
#define SYNCSEL_CC3                 (7<<16)
#define SYNCMODE_CC4                (3<<19)
#define SYNCSEL_CC4                 (7<<21)
#define SYNCMODE_FLASHOUT           (3<<24)
#define SYNCSEL_FLASHOUT            (7<<26)*/

//----------------------------------------------------------------------------------------------
//transfer_ctrl_status
//----------------------------------------------------------------------------------------------
#define CMD_FIFO_SELECT_BIT     11
#define COMMAND_N_BIT           28
#define DMA_MASTER_EN           (1<<0)
#define RESET_DMA               (1<<1)
#define CLEAR_DATA_FIFO         (1<<2)
#define CLEAR_COMMAND_FIFO      (1<<3)
#define MASTER_BURST_LEN        (3<<4)
#define CLEAR_SINGLE_COMMAND    (1<<6)
#define CLEAR_COMMAND_FIFO_1    (1<<7)
#define RESET_MEDIUM_CHANNEL    (1<<8)
#define ENABLE_CMD_LIST_AUTO_SWITCH (1<<9)
#define ACTIVE_CMD_FIFO_LIST    (1<<10)
#define CMD_FIFO_SELECT         (1<<CMD_FIFO_SELECT_BIT)
#define AUTO_CLEAR_CMD_FIFO     (1<<12)
#define DATA_FIFO_N             (0xf<<24)
#define COMMAND_N               (0xF<<COMMAND_N_BIT)

//----------------------------------------------------------------------------------------------
//interrupt_status and enable
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
enum INTERRUPT_REPORT_BIT
//----------------------------------------------------------------------------------------------
{
    IRQ_COMMAND = 0,
    IRQ_FRAME_END,
    IRQ_COMMAND_ABORT,
    IRQ_DATA_OVERFLOW,
    IRQ_PSEUDO,
    IRQ_MAX,
};
#define COMMAND_READY_INTR      (1<<IRQ_COMMAND)
#define END_OF_FRAME_INTR       (1<<IRQ_FRAME_END)
#define COMMAND_ABORT_INTR      (1<<IRQ_COMMAND_ABORT)
#define DATA_OVERFLOW_INTR      (1<<IRQ_DATA_OVERFLOW)
#define PSEUDO_INTR             (1<<IRQ_PSEUDO)
#define CMD_FIFO_ACTIVE_STATUS  (1<<27)
#define COMMAND_FLAGS_STATUS    (0xF<<COMMAND_N_BIT)

//----------------------------------------------------------------------------------------------
//dma status
//----------------------------------------------------------------------------------------------
#define ACTUAL_LENGTH           0xFFFF
#define COMMAND_DIFF            (0xFF<<16)
#define COMMAND_FIFO_FULL       (1<<24)
#define COMMAND_FIFO_OVERFLOW   (1<<25)
#define DATA_FIFO_OVERFLOW      (1<<26)
#define FRAME_DONE              (1<<27)

//----------------------------------------------------------------------------------------------
typedef struct _DMARegister
{
    volatile unsigned int* gen_control;
    volatile unsigned int* version;
    volatile unsigned int* transfer_ctrl_status; //master
    volatile unsigned int* interrupt_enable;
    volatile unsigned int* interrupt_status;
    volatile unsigned int* dma_status;
    volatile unsigned int* dma_xfer_remain;
    volatile unsigned int* dma_total_xfer;
    volatile unsigned int* dma_status2;
    volatile unsigned int* dma_xfer_remain2;
    volatile unsigned int* dma_total_xfer2;
} DMA_REGISTER;


//----------------------------------------------------------------------------------------------
///< brief our ringbuffer definition
struct SBuffer
{
    unsigned int address; ///< start address of these buffer
    unsigned int read; ///< write index, wrap around
    unsigned int write; ///< read pointer
    unsigned int total_size; ///< the total buffer size in bytes
    unsigned int block_size; ///< the maximum sizeof one transfer in bytes, we are working with buffer snippets( or blocks )
    unsigned int write_quota_available; ///< tells how much space is available for writing
    unsigned int read_data_available; ///< how many bytes are availalbe for reading
};

typedef int( *PPREPARE_SCATTER_GATHER_LIST )(
    void* context,
    unsigned long nbytes,
    unsigned long isg,
    unsigned int* offset );

typedef struct _DMA_CONTROLLER_INTERRUPT_COUNTER
{
    INTERRRUPT_COUNTER_BASE command_ready;
    INTERRRUPT_COUNTER_BASE command_abort;
    INTERRRUPT_COUNTER_BASE data_overflow;
    INTERRRUPT_COUNTER_BASE pseudo;
} DMA_CONTROLLER_INTERRUPT_COUNTER;

//----------------------------------------------------------------------------------------------
typedef struct _DMA_CONTROLLER
{
    void* device_context;   ///< pointer to a device specific structure including e.g. parameters to calculate scatter/gather list
    int register_index;
    unsigned int version;
    volatile unsigned char* mem_base; ///< pointer to the DMA controller base
    volatile DMA_REGISTER dma_reg; ///< pointer to the DMA controller base
    volatile unsigned int* cmd_fifo;
    volatile unsigned int* cmd_xfer_remain;
    volatile unsigned int* cmd_xfer_total;
    unsigned int cmd_fifo_sel;
    unsigned int clear_cmd_fifo_msk;
    unsigned int offset;
    unsigned int nbytes; ///< # bytes remaining to transfer
    unsigned char wait_frame, wait_line; ///< control flags for defining the start mode
    int device_index; ///< index of these controller
    struct SBuffer read_buffer, write_buffer; ///< read, write buffer descriptor
    unsigned int transfer_block_count;
    PPREPARE_SCATTER_GATHER_LIST prepare_sg_list;
#ifdef DRIVER
    KSPIN_LOCK lock;
#else
    spinlock_t lock;
#endif
    unsigned int opcode_control;
    unsigned int page_size;
} DMA_CONTROLLER;

void* DMAController( void* context, void* membase, int reg_index, HYPERION_BASE_REGISTER_DEF* reg_def, PPREPARE_SCATTER_GATHER_LIST prepare_sg_list, enum COMMAND_FIFO_DEFINITION cmd_list );
void DMAControllerDestruct( DMA_CONTROLLER** ppdma );
void SetDMAControllerBuffer( DMA_CONTROLLER* pdma, unsigned int address, unsigned int total_size, unsigned char read );
void ResetDMACtrlBuffer( DMA_CONTROLLER* pdma );
void ResetDMAController( DMA_CONTROLLER* pdma );
void StartTransfer( DMA_CONTROLLER* pdma, unsigned int synchronize_medium_channel );
void StopTransfer( DMA_CONTROLLER* pdma );
int PrepareTransferRAMToPCI( DMA_CONTROLLER* pdma, void* context );
int PrepareDirectTransfer( DMA_CONTROLLER* pdma, void* context, unsigned int scan_mode, unsigned int has_videoin_prop_changed, unsigned int trigger_control );
void SetXferTotal( DMA_CONTROLLER* pdma, unsigned int xfer_total );
void DisableInterrupts( DMA_CONTROLLER* pdma );
int HandleDMAEventDirectTransfer( DMA_CONTROLLER* pdma, enum DMA_COMMAND_ACTION event, unsigned int int_stat );
void SetWaitOfFrameEdge( DMA_CONTROLLER* pdma, unsigned int edge );
void EnableInterrupts( DMA_CONTROLLER* pdma );
void DisableInterrupts( DMA_CONTROLLER* pdma );
void ClearCommandFifo( DMA_CONTROLLER* pdma );
void SynchronizeMediumChannel( DMA_CONTROLLER* pdma );
int GetDMAControllerVersion( DMA_CONTROLLER* pdma );
void SetPCIPageSize( DMA_CONTROLLER* pdma, unsigned int page_size );

#define DMAControllerVersion(pdma) (pdma->version & VERSIONS_NUMBER_MSK)

#endif // DMA_CTRL_H
