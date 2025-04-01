#ifndef ClfFuncH
#define ClfFuncH ClfFuncH

#include "device_queue.h"
#include "hyperion_base.h"
#include "hyperion_dma_nios.h"
#include "linux/mutex.h"

#define MAX_SGLIST_BUFFER_SIZE ( 512 * 1024 )
#define MSG_BUFFER_SIZE ( 4 * 1024 )
#define NIOS_MEMORY_APP_SIZE ( 32 * 1024 )

//-------------------------------------------------------------------------------------------
struct dma_transfer_object
//-------------------------------------------------------------------------------------------
{
    unsigned int         index;
    unsigned int         init_done;
    struct device_queue *rw_queue; ///< pointer to the read/write queue defined
                                   ///< in the device extension
    struct mux_controller_sequence mux_seq;
};

//-------------------------------------------------------------------------------------------
struct dma_sg_list_entry
//-------------------------------------------------------------------------------------------
{
    struct list_head list_entry;
    int              index;
    int              buf_size;
    void            *buf;
    dma_addr_t       phy;
    int              size;
    void            *nios_msg;
    dma_addr_t       nios_msg_phy;
    int              nios_msg_size;
    void            *host_msg;
    dma_addr_t       host_msg_phy;
    int              host_msg_size;
    void            *request_extension;
    dma_addr_t       req_ext_phy;
};

//-------------------------------------------------------------------------------------------
struct dma_sg_list
//-------------------------------------------------------------------------------------------
{
    struct list_head head;
    spinlock_t       s_lock;
};

//-------------------------------------------------------------------------------------------
typedef struct ioctl_lock
//-------------------------------------------------------------------------------------------
{
    struct mutex s_generic;
    struct mutex s_request;
    struct mutex s_digital_io;
    struct mutex s_message;
    spinlock_t   s_tasklet;
} t_ioctl_lock;

//-------------------------------------------------------------------------------------------
struct hyperion_device
//-------------------------------------------------------------------------------------------
{
    int                         number;
    struct pci_dev             *pdev;
    struct memory_space         hyperion_base;
    HYPERION_BASE_REGISTER_DEF *reg_def;
    struct device_queue
        *pqueues[MAX_PARALLEL_TRANSFER]; ///< pointer to the queue objects
    struct device_queue
        dq_read_write[MAX_PARALLEL_TRANSFER]; ///< queues for reads and writes
                                              ///< for the DMA_Channels
    struct device_queue *pqueue_result;       ///< pointer to the result_queue
    struct device_queue  dq_result;           ///< queue including result ioobj
    struct dma_transfer_object
        *pdma_object[MAX_PARALLEL_TRANSFER]; ///< pointer to the dma objects
    struct dma_transfer_object dma_transfer_object
        [MAX_PARALLEL_TRANSFER]; ///< the dmaobjects contains all structures
                                 ///< and variables for transfering

    TPipe *interrupt_result_pipe; ///< resultqueue to save interruptstatus for
                                  ///< handling in our DPC
    struct mux_controller_sequence mux_seq;
    uart_object_t                  uart_port[UART_NUM];
    POCL_OBJECT                    pocl[MAX_PARALLEL_TRANSFER];
    void ( *set_mux_data )( struct hyperion                *phyperion,
                            struct mux_controller_sequence *mux_seq,
                            u_long                          inputchannel );
    void ( *set_hrtc_ram )( struct hyperion           *phyperion,
                            struct transfer_parameter *tp );

    int          pci_hyperion_page_size;
    int          processor_status;
    unsigned int processor_app_size;
    struct processor_info_object
    {
        int cpu_clk_hz;
    } processor_info;
    union _fpga_capabilities
    {
        unsigned int capabilities;
        struct
        {
            unsigned int automatic_sync_cl_channels : 1;
            unsigned int reserved : 31;
        } bits;
    } fpga_info;

    struct completion        message_received;
    message_pipe             request_message;
    message_pipe             request_result_message;
    struct dma_sg_list_entry dma_sg_list_pool[MAX_COUNT_REQUEST_OBJECT];
    struct dma_sg_list       dma_sg_list;
    int                      address_space_encoding;
    unsigned char            user_flash_enabled;
    int                      eeprom_write_access;
    t_ioctl_lock             ioctl_lock;
};

int hyperion_func_init( struct hyperion *phyperion, unsigned long control );
int hyperion_func_close( struct hyperion *phyperion );
int hyperion_func_async_read(
    struct hyperion                *phyperion,
    struct hyperion_request_packet *phyperion_request_packet );
void hyperion_func_abort_transfer( struct hyperion *phyperion );
int  hyperion_func_enable_interrupt( struct hyperion *phyperion );
int  hyperion_boot_processor( struct hyperion_device *phyp_dev, void *apps,
                              unsigned int size );
void hyperion_func_release_dma(
    struct hyperion_request_packet *phyperion_request_packet );
int  transmit_message( struct hyperion_device *phyp_dev, unsigned int message,
                       unsigned int timeout_msec, void *buffer,
                       unsigned int size, unsigned long *processor_result );
void translate_property_to_property32( TPropertyElement   *list,
                                       TPropertyElement32 *list32,
                                       unsigned int        prop_count );
void translate_property32_to_property( TPropertyElement   *list,
                                       TPropertyElement32 *list32,
                                       unsigned int        prop_count );
void hyperion_halt_processor( struct hyperion *phyperion );
void hyperion_func_release_hw( struct hyperion *phyperion );
void hyperion_func_camera_power( struct hyperion *phyperion, int state );
#endif // ClfFuncH
