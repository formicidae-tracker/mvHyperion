#ifdef WINNT
#   include <wdm.h>
#elif defined(WIN95)
#   define WANTVXDWRAPS
#   undef  WIN31COMPAT
#   define WIN40SERVICES
#   include <miniport.h>
#   include <basedef.h>
#   include <vmm.h>
#   include <vtd.h>

//ug: function must be defined before including vxdwraps.h
HTIMEOUT VXDINLINE
KSemaSet_Async_Time_Out( void ( *pfnTimeout )(), CMS cms, ULONG ulRefData )
{
    HTIMEOUT htimeout;
    _asm mov eax, [cms]
    _asm mov edx, [ulRefData]
    _asm mov esi, [pfnTimeout]
    VMMCall( Set_Async_Time_Out )
    _asm mov [htimeout], esi
    return( htimeout );
}

void VXDINLINE
KSemaCancel_Time_Out( HTIMEOUT htimeout )
{
    _asm mov esi, htimeout
    VMMCall( Cancel_Time_Out )
}

#   include <vxdwraps.h>
#else /* linux */
#   include <linux/version.h>
#   ifndef KERNEL_VERSION
#       define KERNEL_VERSION(a,b,c) (((a) << 16) | ((b) << 8) | (c))
#   endif
#   if LINUX_VERSION_CODE >= KERNEL_VERSION (2,4,12)
#       include <linux/slab.h>
#   else
#       include <linux/malloc.h>
#   endif
#   if LINUX_VERSION_CODE >= KERNEL_VERSION (2,6,26)
#       include <linux/sched.h>
#       include <linux/semaphore.h>
#   else
#       include <asm/semaphore.h>
#   endif
#   include <linux/interrupt.h>
#   include <kcompat.h>
#   include <matrix_tools.h>

static char sem_magic[] = "SEMA" ;
#   define SEM_MAGIC   (*(unsigned long *)sem_magic)

#endif /* linux */

struct SKSemaphore
{
    unsigned long Magic ;
    unsigned long inicount;
    unsigned long limit;
    unsigned long sema_count;
#ifdef WINNT
    PRKSEMAPHORE  KSem ;
    LONG ReleaseError;
#elifdef WIN95
    int Status;
    ULONG KSem ;
#else /* linux */
    struct semaphore KSem ;
#if LINUX_VERSION_CODE < KERNEL_VERSION (2,3,0)
    struct wait_queue*   queue;
#else
    wait_queue_head_t   queue;
#endif

#endif
};



#include "ksemaphore.h"

#ifdef WIN95
TKSemaphore* SemaphoreHandles = NULL;
LONG InitializedSemaphores = 0;
LONG MaxCountOfSemaHandles = 0x1000 / sizeof( TKSemaphore );
#endif

#define INFINITE     0xFFFFFFFF


/* ----------------------------------------------------------------------- */
TKSemaphore* create_sema( int inicount, int limit )
{
    TKSemaphore* sem = NULL;
#ifdef WINNT
    sem = ( TKSemaphore* )ExAllocatePool( NonPagedPool, sizeof( TKSemaphore ) );

    if( sem )
    {
        sem->KSem = ( PRKSEMAPHORE )ExAllocatePool( NonPagedPool, sizeof( KSEMAPHORE ) );
        if( sem->KSem )
        {
            KeInitializeSemaphore( sem->KSem, inicount, limit );
            sem->inicount = inicount;
            sem->limit = limit;
            sem->ReleaseError = STATUS_SUCCESS;
            sem->sema_count = inicount;
            //_MV_Print (("create semaphore p%p ok initcount %d\n", sem, inicount));
            return sem;
        }
        else
        {
            ExFreePool( sem );
            //_MV_Print (("create semaphore failed \n"));
            return NULL;
        }
    }
    else
    {
        //_MV_Print (("create semaphore failed \n"));
        return NULL;
    }
#elifdef WIN95

    ULONG pagecount = 1;
    ULONG physaddr;
    if( SemaphoreHandles == NULL )
    {
        SemaphoreHandles = ( TKSemaphore* )_PageAllocate( pagecount, PG_SYS, 0, 0, 0, 0xffffffff, ( PVOID* ) & ( physaddr ), PAGEFIXED );
    }

    if( SemaphoreHandles )
    {
        if( InitializedSemaphores < MaxCountOfSemaHandles )
        {
            sem = &SemaphoreHandles[InitializedSemaphores];
            InitializedSemaphores++;
        }
        else
        {
            return NULL;
        }

        sem->KSem = Create_Semaphore( inicount );
        sem->inicount = inicount;
        sem->limit = limit;
        sem->sema_count = inicount;
        return sem;
    }
    else
    {
        return NULL;
    }
#else /* linux */
    if( ( sem = kmalloc( sizeof( TKSemaphore ), GFP_KERNEL ) ) != NULL )
    {
        memset( sem, 0, sizeof( TKSemaphore ) ) ;
        sem->Magic = SEM_MAGIC ;
        sema_init( &sem->KSem, inicount ) ;
        sem->inicount = inicount;
        sem->sema_count = inicount;
        sem->limit = limit;
        INIT_WAIT_QUEUE_HEAD( sem->queue ) ;
        return sem;
    }
    else
    {
        return sem;
    }
#endif /* linux */
}



/* ----------------------------------------------------------------------- */
int destroy_sema( TKSemaphore* sem )
{
    if( sem )
    {
#ifdef WINNT
        //_MV_Print (("destroy semaphore\n"));
        ExFreePool( sem->KSem );
        ExFreePool( sem );
        return 0 ;
#elifdef WIN95
        Destroy_Semaphore( sem->KSem );
        if( InitializedSemaphores > 0 )
        {
            InitializedSemaphores--;
        }

        if( InitializedSemaphores == 0 )
        {
            _PageFree( ( PVOID )SemaphoreHandles, PG_SYS );
            SemaphoreHandles = NULL;
        }
        return 0;
#else
        if( sem && sem->Magic == SEM_MAGIC )
        {
            memset( sem, 0xff, sizeof( TKSemaphore ) ) ;  // debugging aid
            sem->Magic = 0xAABBCCDD ;
            kfree( sem ) ;
            return 0 ;
        }
        else
        {
            return -1 ;
        }
#endif
    }
    else
    {
        return -1 ;
    }
}


/* ----------------------------------------------------------------------- */
void reset_sema( TKSemaphore* sem )
{
    if( sem )
    {
        sem->sema_count = sem->inicount;
#ifdef WINNT
        //_MV_Print (("reset_sema %x \n", sem->KSem));
        KeInitializeSemaphore( sem->KSem, sem->inicount, sem->limit );
        sem->ReleaseError = STATUS_SUCCESS;
#elifdef WIN95
        Destroy_Semaphore( sem->KSem );
        sem->KSem = Create_Semaphore( sem->KSem );
#else /* linux */
        if( sem->Magic == SEM_MAGIC )
        {
            sema_init( &sem->KSem, sem->inicount ) ;
        }
#endif /* linux */
    }
}


#ifdef WIN95

//-----------------------------------------------------------------------------
void handle_timeout( void )
{
    TKSemaphore* sem;

    // Get pointer to IntTable
    __asm mov [sem], edx

    if( sem )
    {
        sem->Status = ObjTimeout;
        Signal_Semaphore( sem->KSem );
        //_MV_Print (("semaphore %x timeout \n", sem->KSem));
    }
}

#endif


/* ----------------------------------------------------------------------- */
int
wait_sema( TKSemaphore *sem, int timeoutmsec )
{
    if( sem )
    {
#ifdef WINNT
        NTSTATUS status;
        LARGE_INTEGER timeout;

        timeout.QuadPart = -10000;
        timeout.QuadPart *= timeoutmsec;

        //_MV_Print (("+wait semaphore, timeoutmsec %d\n", timeoutmsec));
        // status = KeWaitForSingleObject (sem->KSem, Executive, KernelMode,
        // TRUE, (timeoutmsec == INFINITE ? NULL : &timeout) );
        status = KeWaitForSingleObject(
            sem->KSem, UserRequest, UserMode, TRUE,
            ( timeoutmsec == INFINITE ? NULL : &timeout ) );
        sem->ReleaseError = STATUS_SUCCESS;
        if( sem->sema_count > 0 )
        {
            sem->sema_count--;
        }
        //_MV_Print (("-wait semaphore p%p status 0x%x\n", sem->KSem, status));
        switch( status )
        {
        case STATUS_SUCCESS:
            return ObjSignaled;
        case STATUS_TIMEOUT:
            return ObjTimeout;
        case STATUS_ALERTED:
        default:
            return ObjError;
        }
#elifdef WIN95
        HTIMEOUT htimeout = NULL;

        sem->Status = ObjSignaled;
        if( timeoutmsec > 0 )
        {
            if( timeoutmsec != INFINITE )
            {
                htimeout = KSemaSet_Async_Time_Out( handle_timeout,
                                                    timeoutmsec, (ULONG)sem );
            }

            Wait_Semaphore( sem->KSem, BLOCK_SVC_INTS | BLOCK_THREAD_IDLE );
            KSemaCancel_Time_Out( htimeout );
            if( sem->sema_count > 0 )
            {
                sem->sema_count--;
            }
        }
        else
        {
            if( sem->sema_count > 0 )
            {
                Wait_Semaphore( sem->KSem,
                                BLOCK_SVC_INTS | BLOCK_THREAD_IDLE );
                sem->sema_count--;
            }
            else
            {
                sem->Status = ObjTimeout;
            }
        }
        //_MV_Print (("SYS: wait_sema %d st %d\n", sem->sema_count,
        // sem->Status));
        return sem->Status;

#else /* linux */
        if( sem && sem->Magic == SEM_MAGIC )
        {
            unsigned long waitjiffies;
            waitjiffies = msecs_to_jiffies( timeoutmsec );
            if( sem->sema_count > 0 )
            {
                sem->sema_count--;
            }

            // down_interruptible ( &sem->KSem ) ;
            if( timeoutmsec > 0 )
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
                wait_event_interruptible_timeout(
                    sem->queue, ( &sem->KSem.count > 0 ), waitjiffies );
#else
                wait_event_interruptible_timeout(
                    sem->queue, ( atomic_read( &sem->KSem.count ) > 0 ),
                    waitjiffies );
#endif
            if( down_trylock( &sem->KSem ) == 0 )
            {
                return ObjSignaled;
            }
            else
            {
                return ObjTimeout;
            }
        }
        else
        {
            return ObjError;
        }
#endif /* linux */
    }
    else
    {
        //_MV_Print (("wait semaphore () sem p%p\n", sem));
        return -1;
    }
}

/* ----------------------------------------------------------------------- */
int signal_sema( TKSemaphore* sem, int inc )
{
    if( sem )
    {
        sem->sema_count++;
#ifdef WINNT
        if( sem->sema_count < sem->limit && sem->ReleaseError != STATUS_SEMAPHORE_LIMIT_EXCEEDED )
        {
            sem->ReleaseError = KeReleaseSemaphore( sem->KSem, 0, inc, FALSE );

        }
        return 0 ;
#elifdef WIN95
        Signal_Semaphore( sem->KSem );
        //_MV_Print (("SYS: signal_sema %d\n", sem->sema_count));
        return 0;
#else /* linux */
        if( sem && sem->Magic == SEM_MAGIC )
        {
            up( &sem->KSem ) ;
            wake_up_interruptible( &sem->queue );
        }

        return 0;
#endif /* linux */
    }
    else
    {
        return -1 ;
    }
}
