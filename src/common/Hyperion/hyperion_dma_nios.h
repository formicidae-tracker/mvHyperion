#ifndef HYPERION_DMA_NIOS_
#define HYPERION_DMA_NIOS_
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
// additional system definitions
//----------------------------------------------------------------------------------------------
#define SGDMA_MAX_TRANSFER_SIZE 0x200
#define HOST_PAGE_SIZE 0x1000
#define HOST_PAGE_MSK ~(HOST_PAGE_SIZE-1)
#define ONCHIP_MEM_NAME "/dev/onchip_mem"
#define ONCHIP_MEM_TYPE "altera_avalon_onchip_memory2"
#define ONCHIP_MEM_BASE 0x02210000
#define ONCHIP_MEM_SPAN 32768
/*
#define PCI_EXPRESS_COMPILER_NAME "/dev/pci_express_compiler"
#define PCI_EXPRESS_COMPILER_BASE 0x4000
#define PCI_EXPRESS_COMPILER_SPAN 16384*/
#define TRANSLATION_TABLE_NAME "/dev/translation_table"
#define TRANSLATION_TABLE_BASE 0x201000
#define TRANSLATION_TABLE_SPAN 4096
#define ONCHIP_MEM_DATA_NAME "/dev/onchip_mem_data"
#define ONCHIP_MEM_DATA_BASE 0x02220000
#define ONCHIP_MEM_DATA_SPAN 16384
#define SYSTEM_NAME "/dev/system"
#define SYSTEM_BASE 0x0
#define SYSTEM_SPAN 2097152
//----------------------------------------------------------------------------------------------
typedef struct _message_pipe_shared_def
//----------------------------------------------------------------------------------------------
{
    unsigned int items_r, items_w, items_max;
    unsigned char host_initialized;
    unsigned char client_initialized;
} message_pipe_shared_def;
//----------------------------------------------------------------------------------------------
typedef struct _message_item
//----------------------------------------------------------------------------------------------
{
    unsigned int message;
} message_item;
//----------------------------------------------------------------------------------------------
typedef struct _message_pipe
//----------------------------------------------------------------------------------------------
{
    message_pipe_shared_def* shared_def;
    message_item* data, *head, *tail;
} message_pipe;
//-------------------------------------------------------------------------------------
#define _ITEMS_IN_MSG_PIPE(pipe) (pipe.shared_def->items_w - pipe.shared_def->items_r)
#define _ITEMS_IN_MSG_PIPE_LE32(pipe) (le32_to_cpu(pipe.shared_def->items_w) - le32_to_cpu(pipe.shared_def->items_r))
//-------------------------------------------------------------------------------------
#define _RESET_MSG_PIPE(pipe) pipe.shared_def->items_r = pipe.shared_def->items_w = 0;
//-------------------------------------------------------------------------------------
#define _WRITE_MSG_PIPE(pipe,item)\
    {\
        unsigned int diff = pipe.shared_def->items_w - pipe.shared_def->items_r;\
        unsigned int item_id = pipe.shared_def->items_w % pipe.shared_def->items_max;\
        if( diff < pipe.shared_def->items_max )\
        {\
            pipe.head[item_id] = item;\
            pipe.shared_def->items_w++;\
        }\
    }
//-------------------------------------------------------------------------------------
#define _WRITE_MSG_PIPE_LE32(pipe,item)\
    {\
        unsigned int it_w = le32_to_cpu(pipe.shared_def->items_w), it_r = le32_to_cpu(pipe.shared_def->items_r), it_max = le32_to_cpu(pipe.shared_def->items_max);\
        unsigned int diff = it_w - it_r;\
        unsigned int item_id = it_w % it_max;\
        if( diff < it_max )\
        {\
            pipe.head[item_id].message = cpu_to_le32( item.message );\
            it_w++;\
            pipe.shared_def->items_w = cpu_to_le32( it_w );\
        }\
    }
//-------------------------------------------------------------------------------------
#define _READ_MSG_PIPE(pipe,item)\
    {\
        unsigned int diff = pipe.shared_def->items_w - pipe.shared_def->items_r;\
        unsigned int item_id = pipe.shared_def->items_r % pipe.shared_def->items_max;\
        if( diff != 0 )\
        {\
            item = pipe.tail[item_id];\
            pipe.shared_def->items_r++;\
        }\
    }
//-------------------------------------------------------------------------------------
#define _READ_MSG_PIPE_LE32(pipe,item)\
    {\
        unsigned int it_w = le32_to_cpu(pipe.shared_def->items_w), it_r = le32_to_cpu(pipe.shared_def->items_r), it_max = le32_to_cpu( pipe.shared_def->items_max );\
        unsigned int diff = it_w - it_r;\
        unsigned int item_id = it_r % it_max;\
        if( diff != 0 )\
        {\
            message_item msg_item;\
            msg_item = pipe.tail[item_id];\
            item.message = le32_to_cpu( msg_item.message );\
            it_r++;\
            pipe.shared_def->items_r = cpu_to_le32( it_r );\
        }\
    }
//-------------------------------------------------------------------------------------
#define _MSG_PIPE_CURRENT(pipe,item)\
    {\
        unsigned int diff = pipe.shared_def->items_w - pipe.shared_def->items_r;\
        unsigned int item_id = pipe.shared_def->items_r % pipe.shared_def->items_max;\
        if( diff != 0 )\
        {\
            item = pipe.tail[item_id];\
        }\
    }
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
// onchip_mem_data segment size
//----------------------------------------------------------------------------------------------
/*
 * onchip_mem_data partitioning
 * offset
 * [00000000 - 000001FF] message_list physical addresses, this range will be filled out from host
 * [00000200 - 00000FFF] sgdma descriptors, right place for this ? better placed in heap memory
 * [00001000 - 00001FFF] request host addr, this partition will be used as temporary space for physical host addr
 * [00002000 - 00002FFF] request io buffer, this partition will be used as input outbuffer for request and result parameter
 * [00003000 - 000030xx] shared memory host -> nios message header sizeof(message_pipe_shared_def)
 * [000030xx - 00003xxx] shared memory host -> nios message pipe 512 bytes
 * [00003xxx - 000033FF] shared memory host -> nios512 - sizeof(message_pipe_shared_def) bytes
 * [00003400 - 000034xx] shared memory nios -> host message header sizeof(message_pipe_shared_def)
 * [000034xx - 00003xxx] shared memory nios -> host message pipe 512 bytes
 * [00003xxx - 000034FF] shared memory nios -> host 512 - sizeof(message_pipe_shared_def) bytes
*/
//----------------------------------------------------------------------------------------------
// onchip_mem_data definitions
//----------------------------------------------------------------------------------------------
#define OFF_ONCHIP_MEM_DATA_MESSAGE_LIST            0
#define SIZE_ONCHIP_MEM_DATA_MESSAGE_LIST           (MAX_COUNT_REQUEST_OBJECT*sizeof(message_addr_list))
#define OFF_ONCHIP_MEM_DATA_SGDMA_DESCR             (OFF_ONCHIP_MEM_DATA_MESSAGE_LIST + SIZE_ONCHIP_MEM_DATA_MESSAGE_LIST)
#define SIZE_ONCHIP_MEM_DATA_SGDMA_DESCR            (0x1000 - SIZE_ONCHIP_MEM_DATA_MESSAGE_LIST)
#define OFF_ONCHIP_MEM_DATA_REQUEST_HOST_ADDR       (OFF_ONCHIP_MEM_DATA_SGDMA_DESCR + SIZE_ONCHIP_MEM_DATA_SGDMA_DESCR)
#define SIZE_ONCHIP_MEM_DATA_REQUEST_DATA           0x1000
#define OFF_ONCHIP_MEM_DATA_REQUEST_IO_BUFFER       (OFF_ONCHIP_MEM_DATA_REQUEST_HOST_ADDR + SIZE_ONCHIP_MEM_DATA_REQUEST_DATA)
#define SIZE_ONCHIP_MEM_DATA_REQUEST_DATA           0x1000
#define OFF_ONCHIP_MEM_DATA_NIOS_MSG_HEADER         (OFF_ONCHIP_MEM_DATA_REQUEST_IO_BUFFER + SIZE_ONCHIP_MEM_DATA_REQUEST_DATA)
#define SIZE_ONCHIP_MEM_DATA_NIOS_MSG_HEADER        (sizeof(message_pipe_shared_def))
#define OFF_ONCHIP_MEM_DATA_NIOS_MSG_PIPE           (OFF_ONCHIP_MEM_DATA_NIOS_MSG_HEADER + SIZE_ONCHIP_MEM_DATA_NIOS_MSG_HEADER)
#define SIZE_ONCHIP_MEM_DATA_NIOS_MSG_PIPE          0x200
#define OFF_ONCHIP_MEM_DATA_SHARED_BUFFER_NIOS      (OFF_ONCHIP_MEM_DATA_NIOS_MSG_PIPE + SIZE_ONCHIP_MEM_DATA_NIOS_MSG_PIPE)
#define SIZE_ONCHIP_MEM_DATA_SHARED_BUFFER_NIOS     (0x200-sizeof(message_pipe_shared_def))
#define OFF_ONCHIP_MEM_DATA_HOST_MSG_HEADER         (OFF_ONCHIP_MEM_DATA_SHARED_BUFFER_NIOS + SIZE_ONCHIP_MEM_DATA_SHARED_BUFFER_NIOS)
#define SIZE_ONCHIP_MEM_DATA_HOST_MSG_HEADER        sizeof(message_pipe_shared_def)
#define OFF_ONCHIP_MEM_DATA_HOST_MSG_PIPE           (OFF_ONCHIP_MEM_DATA_HOST_MSG_HEADER + SIZE_ONCHIP_MEM_DATA_HOST_MSG_HEADER)
#define SIZE_ONCHIP_MEM_DATA_HOST_MSG_PIPE          0x200
#define OFF_ONCHIP_MEM_DATA_SHARED_BUFFER_HOST      (OFF_ONCHIP_MEM_DATA_NIOS_MSG_PIPE + SIZE_ONCHIP_MEM_DATA_NIOS_MSG_PIPE)
#define SIZE_ONCHIP_MEM_DATA_SHARED_BUFFER_HOST     (0x200-sizeof(message_pipe_shared_def))
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
// mailbox 0 message format definition
//----------------------------------------------------------------------------------------------
//general message definition [31 .. message_type .. 28, 27 ... message specific ... 0]
#define MESSAGE_TYPE_REQUEST        (0<<MB0_MESSAGE_TYPE)
#define MESSAGE_TYPE_SYSTEM_INFO    (1<<MB0_MESSAGE_TYPE)
#define MESSAGE_TYPE_ABORT_REQUESTS (2<<MB0_MESSAGE_TYPE)
#define MESSAGE_TYPE_METHOD         (3<<MB0_MESSAGE_TYPE)
//----------------------------------------------------------------------------------------------
// query for request
//----------------------------------------------------------------------------------------------
//message request [31 .. message_type .. 28, queue_index ( == inputchannel) 27, 21 .. message size  .. 8, 7 .. message index .. 0]
#define MB0_MESSAGE_INDEX        0
#define MB0_MESSAGE_SIZE        8
#define MBO_MESSAGE_QUEUE_ID    27
#define MB0_MESSAGE_TYPE        28
#define NBIT_MESSAGE_INDEX        5
#define NBIT_MESSAGE_SIZE        13
#define NBIT_MESSAGE_QUEUE_ID    1
#define NBIT_MESSAGE_CODE        4
#define MESSAGE_INDEX_MSK        ((1<<NBIT_MESSAGE_INDEX)-1)
#define MESSAGE_SIZE_MSK        ((1<<NBIT_MESSAGE_SIZE)-1)
#define MESSAGE_TYPE_MSK        ((1<<NBIT_MESSAGE_SIZE)-1)
#define MESSAGE_QUEUE_ID_MSK    ((1<<NBIT_MESSAGE_QUEUE_ID)-1)
#define _SET_MESSAGE_QUEUE_ID(id) ((id & NBIT_MESSAGE_QUEUE_ID) << MBO_MESSAGE_QUEUE_ID)
#define _GET_MESSAGE_QUEUE_ID(msg) ((msg >> MBO_MESSAGE_QUEUE_ID) & NBIT_MESSAGE_QUEUE_ID)
//----------------------------------------------------------------------------------------------
// query nios system info
//----------------------------------------------------------------------------------------------
//message query system info [31 .. message_type .. 28, 27 .. information index (info type) .. 0]
#define MB0_SYSTEM_INFO                0
#define NBIT_SYSTEM_INFO            28
#define MESSAGE_SIZE_SYS_INFO_MSK    ((1<<NBIT_SYSTEM_INFO)-1)
// query info_types
#define QUERY_CPU_CLK_HZ            0
#define QUERY_FPGA_CAPABILITIES     1

//----------------------------------------------------------------------------------------------
// method call host->nios
//----------------------------------------------------------------------------------------------
//message method [31 .. message_type .. 28, 27 .. parameter size  .. 8, 7 .. method index .. 0]
#define ID_METHOD_IS_DIGITAL_INPUT_AVAILABLE    0
#define ID_METHOD_IS_DIGITAL_OUTPUT_AVAILABLE   1
#define ID_METHOD_CONFIGURE_DIGITAL_OUTPUT      2
#define ID_METHOD_WRITE_DIGITAL_OUTPUT          3
#define ID_METHOD_READ_DIGITAL_INPUT            4
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
// dma_main definitions
//----------------------------------------------------------------------------------------------
#define MAX_COUNT_REQUEST_OBJECT (1<<NBIT_MESSAGE_INDEX)
//----------------------------------------------------------------------------------------------
typedef struct _message_addr_list
{
    unsigned long long nios_msg_phy;
    unsigned long long host_msg_phy;
} message_addr_list;
//----------------------------------------------------------------------------------------------
// request parameter definitons
//----------------------------------------------------------------------------------------------
#define SYNC_STATUS_FRAME            0x1
#define SYNC_STATUS_LINE            0x2

//----------------------------------------------------------------------------------------------
// address space encoding
//----------------------------------------------------------------------------------------------
#define DMA_ADDRESS_SPACE_ENCODING_32BIT 0
#define DMA_ADDRESS_SPACE_ENCODING_64BIT 1


//----------------------------------------------------------------------------------------------
// debug message codes
//----------------------------------------------------------------------------------------------
#define NIOS_DEBUG_MSG_PREPARE_REQUEST  0x10000000
#define NIOS_DEBUG_MSG_MSG_FOUND        0x11000000
#define NIOS_DEBUG_MSG_START_VIDEOIN    0x20000000
#define NIOS_DEBUG_MSG_RELEASE_REQUEST  0x30000000
#define NIOS_DEBUG_MSG_CANCEL_REQUEST   0x40000000
#define NIOS_DEBUG_MSG_ABORT_REQUEST    0x50000000
#define NIOS_DEBUG_MSG_INTR             0x60000000

#endif /*HYPERION_DMA_NIOS_*/
