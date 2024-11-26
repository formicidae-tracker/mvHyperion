#ifndef HyperionH
#define HyperionH HyperionH

#include "drivermain.h"

#include <linux/aio.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/pagemap.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
#include <asm/scatterlist.h>
#else
#include <linux/scatterlist.h>
#endif

#include "pipe.h"
#include "DMACtrl.h"
#include "HyperionEeprom.h"
#include "uart_read_write.h"
#include "PoCLControl.h"

#include "matrix_tools.h"       // <== this should be put in a common directory for all drivers!
#include "user_buffer.h"

#include "HyperionIoCtl.h"
#include "i2c_access.h"
#include "DigitalIO.h"
#include "HyperionRegister.h"
#include "HyperionProp.h"
#include "property.h"
#include "external_boot.h"

//-------------------------------------------------------------------------------------------
#define remove_entry(entry) {\
        struct list_head *prev, *next;\
        next = (entry)->next;\
        prev = (entry)->prev;\
        prev->next = next;\
        next->prev = prev;\
    }

//-------------------------------------------------------------------------------------------
#define remove_head_list(head) \
    (head)->next;\
    {remove_entry((head)->next)}

#ifndef DMA_BIT_MASK
#   define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))
#endif

//-------------------------------------------------------------------------------------------
struct mux_controller_sequence
//-------------------------------------------------------------------------------------------
{
    u_char changed;
    u32 size;
    u32 muxdata[MUX_CONTROLLER_SEQUENCE_SIZE_BYTES];
};

//-------------------------------------------------------------------------------------------
typedef struct trigger_control
//-------------------------------------------------------------------------------------------
{
    u32 trigger_mode;
    u32 trigger_overlap;
} trigger_control_t;

//-------------------------------------------------------------------------------------------
///\brief transfer parameter structure, this struct includes all parameters for a request
struct transfer_parameter
//-------------------------------------------------------------------------------------------
{
    u32 property_count;
    u32 has_properties_changed;
    u32 reqaction;
    u32 reqid;
    u32 transferlength;
    u32 footeroffset;
    u32 footersize;
    u32 timeoutmsec;
    u32 inputchannel;
    u32 xstart;
    u32 xstop;
    u32 ystart;
    u32 ystop;
    void* resultpacket;
    u32 scanmode;
    u32 datavalidenable;
    u32 videoin_direct_transfer;
    u32 videoin_wait_of_frameedge;
    u32 medium_mode_ctrl;
    u32 enable_aoimode;
    trigger_control_t frame_start;
    trigger_control_t frame_stop;
    u32 trigger_mode;
    u32 trigger_hrtc_command_count;
    u32 trigger_hrtc_ram[emvdHRTCTriggerCommand];
    u32 expand_lval;
    u32 line_scan_startcondition;
};

//-------------------------------------------------------------------------------------------
struct hyperion_request_packet
//-------------------------------------------------------------------------------------------
{
    struct list_head list_entry;
    void* private;
    struct kiocb* iocb;
    char __user* user_buffer; ///< pointer to the user buffer
    size_t count; ///< sizeof user buffer
    struct user_buffer_descriptor user_buffer_descr;
    loff_t pos; ///< offset read data from
    struct transfer_parameter parameters; ///< this structs hold the parameter for next transfer
    unsigned long start_jiffies; ///< start timestamp
    TPropertyElement* request_parameter;
    unsigned int size_request_parameter;
    TPropertyElement result_property[prSnapRequestResultMax];
    unsigned int scan_pixel_line0;
    unsigned int scan_pixel_line1;
    unsigned int scan_lines;
    void* dma_list_entry;
    HyperionRequestBuffer request_buffer; ///< this request buffer includes the user mode buffer pointers to leader_buffer, payload_buffer and trailer_buffer. All actions with these buffers must be done with copy_from_user, copy_to_user kernel functions or special mapping algorithm.
    struct user_buffer_descriptor trailer_buffer_descr;
};


#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,19)
irqreturn_t hyperion_interrupt( int irq, void* dev_id, struct pt_regs* fake );
#else
irqreturn_t hyperion_interrupt( int irq, void* dev_id );
#endif
void hyperion_do_tasklet( unsigned long index );

int hyperion_board_init( struct hyperion* phyperion, unsigned long control );
int hyperion_board_close( struct hyperion* phyperion );
int hyperion_async_read( struct hyperion* phyperion, struct hyperion_request_packet* phyperion_request_packet );
void hyperion_abort_transfer( struct hyperion* phyperion );
void hyperion_add_to_cleanup_pipe( struct hyperion* phyperion, struct page* page, void* vmem );
void hyperion_remove_objects_from_cleanup_pipe( struct hyperion* phyperion );
int hyperion_i2c_receive_data( struct hyperion* phyperion, int addr, int sub_addr, int length, unsigned char* pdata );
int hyperion_get_firmware_version( struct hyperion* phyperion );
int hyperion_enable_interrupt( struct hyperion* phyperion );
void hyperion_release_dma( struct hyperion_request_packet* phyperion_request_packet );
void hyperion_release_hw( struct hyperion* phyperion );
void hyperion_camera_power( struct hyperion* phyperion, int state );
int hyperion_copy_to_trailer( struct user_buffer_descriptor* puser_buffer_descr, char* data, unsigned int size );

#define IO_READ_8(MEMBASE,REGDEF,REGID,OFFSET) ioread8( (void*)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define IO_WRITE_8(MEMBASE,REGDEF,REGID,OFFSET,DATA) iowrite8(DATA, (void*)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define IO_READ_32(MEMBASE,REGDEF,REGID,OFFSET) ioread32( (void __iomem *)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define IO_READ_32_PRINT(MEMBASE,REGDEF,REGID,OFFSET)\
    printk("ioread32(base %p, regid %d, off %x) regdef.off %x regdef.off_mul %d *%p = %x\n", (void*)MEMBASE.base, REGID, OFFSET, REGDEF[REGID].offset, REGDEF[REGID].off_mul, (void __iomem *)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)), ioread32((void __iomem *)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET))));
#define IO_WRITE_32(MEMBASE,REGDEF,REGID,OFFSET,DATA) iowrite32(DATA, (void __iomem *)(MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define REG_POINTER(MEMBASE,REGDEF,REGID,OFFSET) (MEMBASE.base+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET))

//-------------------------------------------------------------------------------------------
// enhanced debug flags
//-------------------------------------------------------------------------------------------
#define ENABLE_HARDWARE_SIGNAL_DEBUGGING_OUT_ON_J6_DBG_BIT 10
#define HARDWARE_SIGNAL_SELECTOR_MSK 0x300


#endif //HyperionH
