#ifndef devicequeueH
#define devicequeueH devicequeueH

#include <linux/version.h>
#include <linux/aio.h>
#include <linux/timer.h>
#include "hyperion.h"
#include "DMACtrl.h"
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
#   include <asm/scatterlist.h>
#else
#   include <linux/scatterlist.h>
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)

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




/////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------
typedef struct result_packet_entry
//--------------------------------------------------------------------------------------
{
    void* result_packet;
    u32 status;
    u32 request_id;
    u32 timestamp_lowpart_us;
    u32 timestamp_highpart_us;
    struct list_head list_entry;
} result_packet_entry_t;


result_packet_entry_t* allocate_result_packet_entry( void );
void free_result_packet_entry( result_packet_entry_t* prp );
void push_result_packet_entry( wait_queue_head_t* head, result_packet_entry_t* entry );
result_packet_entry_t* pop_result_packet_entry( wait_queue_head_t* head );
int is_result_queue_empty( wait_queue_head_t* head );
void empty_result_queue( wait_queue_head_t* head );

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
};

//-------------------------------------------------------------------------------------------
struct io_object
//-------------------------------------------------------------------------------------------
{
    int device_index;
    struct list_head list_entry;
    struct kiocb* iocb;
    struct scatterlist* sg;
    unsigned int do_dio; ///< direct i/o set up
    char __user* buffer; ///< pointer to the user buffer
    size_t count; ///< sizeof user buffer
    loff_t pos; ///< offset read data from
    unsigned long isg; ///< counter for updating the avalonpcitranslationtable
    struct transfer_parameter transfer_param; ///< this structs hold the parameter for next transfer
    void* dma_transfer_object; ///< pointer to the dma-transferobject working with this request
    unsigned long start_jiffies; ///< start timestamp
    struct timer_list timer_iocb; ///< the timer object for cancelling an iocb action if timeout expire.
    result_packet_entry_t* result_packet;
    TPropertyElement result_property[prSnapRequestResultMax];
    unsigned int scan_pixel_line0;
    unsigned int scan_pixel_line1;
    unsigned int scan_lines;
    DMA_CONTROLLER* controller_videoin;
    unsigned long numxfer; ///< bytes transferred so far
    void* ubuf_obj;
    void* device;
};


//-------------------------------------------------------------------------------------------
typedef void ( *prequest_start_iocb )( struct io_object* ioobj, int index );


//-------------------------------------------------------------------------------------------
struct device_queue
//-------------------------------------------------------------------------------------------
{
    struct list_head head;
    spinlock_t s_lock;
    volatile struct io_object* current_ioobj;
    prequest_start_iocb start_iocb;
    int index;
};

typedef struct device_queue device_queue_t, *pdevice_queue_t;
typedef struct device_queue_entry device_queue_entry_t, *pdevice_queue_entry_t;


void initialize_queue( pdevice_queue_t pqueue, prequest_start_iocb start_iocb, int index );
struct io_object* run_next_iocb( pdevice_queue_t pqueue );
int run_iocb( pdevice_queue_t pqueue, struct io_object* iocb );
void remove_entries( pdevice_queue_t pqueue );
struct io_object* get_current_iocb( pdevice_queue_t pqueue );
void cancel_iocb( pdevice_queue_t pqueue, struct kiocb* iocb );
void insert_iocb_tail( pdevice_queue_t pqueue, struct io_object* ioobj );
void set_queue_start_iocb( pdevice_queue_t pqueue, prequest_start_iocb start_iocb );
void remove_and_notify_iocbs( pdevice_queue_t pqueue );
void restart_current_iocb( pdevice_queue_t pqueue );
int is_queue_empty( pdevice_queue_t pqueue );



//A driver which implements cancellation needs to implement a function for that purpose:
//
//    int my_aio_cancel(struct kiocb *iocb, struct io_event *event);
//
//A pointer to this function can be stored into any IOCB which can be cancelled:
//
//    iocb->ki_cancel = my_aio_cancel;

////--------------------------------------------------------------------------------------
//typedef struct _DEVQUEUE {
////--------------------------------------------------------------------------------------
//  LIST_ENTRY head;
//  KSPIN_LOCK lock;
//  PDRIVER_STARTIO StartIo;
//  LONG stallcount;
//  PIRP CurrentIrp;
//  KEVENT evStop;
//  PQNOTIFYFUNC notify;
//  PVOID notifycontext;
//  NTSTATUS abortstatus;
//  PVOID debugport;
//  } DEVQUEUE, *PDEVQUEUE;
//
//VOID NTAPI AbortAllRequests(PDEVQUEUE* q, ULONG nq, NTSTATUS status);
//VOID NTAPI AbortRequests(PDEVQUEUE pdq, NTSTATUS status);
//VOID NTAPI AllowAllRequests(PDEVQUEUE* q, ULONG nq);
//VOID NTAPI AllowRequests(PDEVQUEUE pdq);
//NTSTATUS NTAPI AreRequestsBeingAborted(PDEVQUEUE pdq);
//VOID NTAPI CancelRequest(PDEVQUEUE pdq, PIRP Irp);
//BOOLEAN   NTAPI CheckAnyBusyAndStall(PDEVQUEUE* q, ULONG nq, PDEVICE_OBJECT fdo);
//BOOLEAN NTAPI CheckBusyAndStall(PDEVQUEUE pdq);
//VOID NTAPI CleanupAllRequests(PDEVQUEUE* q, ULONG nq, PFILE_OBJECT fop, NTSTATUS status);
//VOID NTAPI CleanupRequests(PDEVQUEUE pdq, PFILE_OBJECT fop, NTSTATUS status);
//PIRP NTAPI GetCurrentIrp(PDEVQUEUE pdq);
//VOID NTAPI InitializeQueue(PDEVQUEUE pdq, PDRIVER_STARTIO StartIo);
//VOID NTAPI RestartAllRequests(PDEVQUEUE* q, ULONG nq, PDEVICE_OBJECT fdo);
//VOID NTAPI RestartRequests(PDEVQUEUE pdq, PDEVICE_OBJECT fdo);
//VOID NTAPI StallAllRequests(PDEVQUEUE* q, ULONG nq);
//VOID NTAPI StallRequests(PDEVQUEUE pdq);
//NTSTATUS NTAPI StallAllRequestsAndNotify(PDEVQUEUE* q, ULONG nq, PQNOTIFYFUNC notify, PVOID context);
//NTSTATUS NTAPI StallRequestsAndNotify(PDEVQUEUE pdq, PQNOTIFYFUNC notify, PVOID context);
//PIRP NTAPI StartNextPacket(PDEVQUEUE pdq, PDEVICE_OBJECT fdo);
//VOID NTAPI StartPacket(PDEVQUEUE pdq, PDEVICE_OBJECT fdo, PIRP Irp, PDRIVER_CANCEL cancel);
//VOID NTAPI WaitForCurrentIrp(PDEVQUEUE pdq);
//VOID NTAPI WaitForCurrentIrps(PDEVQUEUE* q, ULONG nq);

#endif //devicequeueH
