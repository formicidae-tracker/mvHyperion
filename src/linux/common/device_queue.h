#ifndef devicequeueH
#define devicequeueH devicequeueH

#include "hyperion_base.h"

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
typedef void ( *prequest_start_iocb )( struct hyperion_request_packet* phyperion_request_packet, int index );


//-------------------------------------------------------------------------------------------
struct device_queue
//-------------------------------------------------------------------------------------------
{
    struct list_head head;
    spinlock_t s_lock;
    volatile struct hyperion_request_packet* current_ioobj;
    prequest_start_iocb start_iocb;
    int index;
};

typedef struct device_queue device_queue_t, *pdevice_queue_t;
typedef struct device_queue_entry device_queue_entry_t, *pdevice_queue_entry_t;


void initialize_queue( pdevice_queue_t pqueue, prequest_start_iocb start_iocb, int index );
struct hyperion_request_packet* run_next_iocb( pdevice_queue_t pqueue );
int run_iocb( pdevice_queue_t pqueue, struct hyperion_request_packet* phyperion_request_packet );
void remove_entries( pdevice_queue_t pqueue );
struct hyperion_request_packet* get_current_iocb( pdevice_queue_t pqueue );
void cancel_iocb( pdevice_queue_t pqueue, struct kiocb* iocb );
void insert_iocb_tail( pdevice_queue_t pqueue, struct hyperion_request_packet* phyperion_request_packet );
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
