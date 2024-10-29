#if defined(linux) || defined(__linux) || defined(__linux__)
#   if NIOS_SUPPORTED_TRANSFER
#       include "hyperion_base.h"
#   else
#       include "hyperion.h"
#   endif
#   include "PoCLControl.h"
#else
#   include "stddcls.h"
#   include "PoCLControl.h"
#   include "Driver.h"
//#    include "PoCLControl.tmh"
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

#define POCL_DBG 0
#define ENABLE_POCL 1
#define ENABLE_INTERRUPT 0
#if defined(linux) || defined(__linux) || defined(__linux__)
#   if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
void jit_timer_pocl( struct timer_list* t );
#   else
void jit_timer_pocl( unsigned long arg );
#   endif

#   define CANCEL_TIMER() del_timer( &ppo->timer )
#   define ADD_TIMER(timer_expires_msec)\
    {\
        unsigned long timeout_jiffies = msecs_to_jiffies( timer_expires_msec );\
        mod_timer( &ppo->timer, (jiffies + timeout_jiffies) );\
    }

#   if POCL_DBG
#       define dbg_print(msg) printk msg
#   else
#       define dbg_print(msg)
#   endif

#   define DELAY_EXECUTION_US(dl) udelay(dl)

#else

VOID DpcForPoCL( IN PKDPC Dpc, IN PVOID DeferredContext, IN PVOID SystemArgument1, IN PVOID SystemArgument2 );

#   define CANCEL_TIMER() KeCancelTimer( &ppo->Timer )

#   define ADD_TIMER(timer_expires_msec)\
    {\
        LARGE_INTEGER lTimeout;\
        lTimeout.QuadPart = -10000;\
        lTimeout.QuadPart *= timer_expires_msec;\
        KeSetTimer( &ppo->Timer, lTimeout, &ppo->PoCLDpc );\
    }

#   if POCL_DBG
#       define dbg_print(msg) DbgPrint msg
#   else
#       define dbg_print(msg)
#   endif
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

#if POCL_DBG
static char* PoCLActionstring[] =
{
    "epcsConnectionSense",
    "epcsCameraCableDetect",
    "epcsClockDetect",
    "res0",
    "res1",
};
#endif

//----------------------------------------------------------------------------------
int CreatePoCLObject( POCL_OBJECT* ppo, void* base, HYPERION_BASE_REGISTER_DEF* regDef, int regIndex )
//----------------------------------------------------------------------------------
{
    int result = FALSE;

    dbg_print( ( " %s, poclobj %p, base %p, reg_index %d\n", __FUNCTION__, ppo, base, regIndex ) );
    if( ppo != NULL && base != NULL )
    {
#if defined(linux) || defined(__linux) || defined(__linux__)
        ppo->RegisterBase.base = ( unsigned char* )base;
#else
        ppo->RegisterBase = ( unsigned char* )base;
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
        ppo->Index = regIndex == ebrhCLController0 ? 0 : 1;
        ppo->RegDef = regDef;
        ppo->RegisterIndex = regIndex;
        if( IO_READ_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_VERSION ) == -1 )
        {
            return FALSE;
        }
#if defined(linux) || defined(__linux) || defined(__linux__)

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
        timer_setup( &ppo->timer, jit_timer_pocl, 0 );
#else
        init_timer( &ppo->timer );
        ppo->timer.function = jit_timer_pocl;
        ppo->timer.data = ( unsigned long )ppo;
#endif

        ppo->timer.expires = 0;
#else
        KeInitializeDpc( &ppo->PoCLDpc, DpcForPoCL, ( PVOID )ppo );
        KeInitializeTimer( &ppo->Timer );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
        ppo->State = FALSE;
        ppo->InitDone = TRUE;
        ppo->CLClockLost = TRUE;
    }
    return result;
}

//----------------------------------------------------------------------------------
void ClosePoCL( POCL_OBJECT* ppo )
//----------------------------------------------------------------------------------
{
    if( ppo->InitDone )
    {
        ppo->InitDone = FALSE;
        CANCEL_TIMER();
        IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_INTR_ENABLE, 0 );
        IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_CONTROL, 0 );
    }
}

//----------------------------------------------------------------------------------
void HandlePoCLEvent( POCL_OBJECT* ppo )
//----------------------------------------------------------------------------------
{
    unsigned int pocl_state;
    while( 1 )
    {
        //dbg_print(( " %s, id %d, %s\n", __FUNCTION__, ppo->Id, PoCLActionstring[ppo->Action] ));
        switch( ppo->Action )
        {
        case epcsConnectionSense:
            {
                IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_CONTROL, CL_CLK_OBSERVE );
                ppo->Action = epcsCameraCableDetect;
                ADD_TIMER( 500 ); //this timeout is not specified but necessary to avoid to give the connection sense to much system resource
                break;
            }
        case epcsCameraCableDetect:
            {
                ppo->Action = epcsClockDetect;
                //IO_READ_32_PRINT(ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_STATUS);
                pocl_state = IO_READ_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_STATUS );
                dbg_print( ( " %s, id %d, status 0x%x\n", __FUNCTION__, ppo->RegisterIndex, pocl_state ) );
                ppo->DetectedCamera = pocl_state & STAT_THRESHOLD;
                switch( ppo->DetectedCamera )
                {
                case eptNonPoCLCamera:
                    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_CONTROL, ( CL_CLK_OBSERVE | NON_POCL_CAMERA_ENABLE ) );
                    ADD_TIMER( 3000 );
                    break;
                case eptPoCLCamera:
                    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_CONTROL, ( CL_CLK_OBSERVE | POCL_CAMERA_ENABLE ) );
                    ADD_TIMER( 3000 );
                    break;
                default:
                    //for revision < 2
                    if( ( pocl_state & STAT_CL_CLK_OFF_TIMER ) < CRITICAL_CL_CLK_OFF_TIMER )
                    {
                        ADD_TIMER( 3000 );
                        break;
                    }
                    ppo->Action = epcsConnectionSense;
                    continue;
                }
                break;
            }
        case epcsClockDetect:
            {
                pocl_state = IO_READ_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_STATUS );
                if( ( pocl_state & STAT_CL_CLK_OFF_TIMER ) >= CRITICAL_CL_CLK_OFF_TIMER )
                {
                    dbg_print( ( " %s, id %d, RegisterBase %p status 0x%x camera absent\n", __FUNCTION__, ppo->RegisterIndex, ppo->RegisterBase, pocl_state ) );
                    ppo->CLClockLost = TRUE;
                    CANCEL_TIMER();
                    ppo->Action = epcsConnectionSense;
                    continue;
                }
                else
                {
                    if( ppo->CLClockLost )
                    {
                        ppo->CLClockLost = FALSE;
                        ChannelLinkReset( ppo );
                    }
                }
#if !ENABLE_INTERRUPT
                ADD_TIMER( 3000 );
#endif
                break;
            }
        }
        break;
    }
}

#if defined(linux) || defined(__linux) || defined(__linux__)
#  if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
//----------------------------------------------------------------------------------
void jit_timer_pocl( struct timer_list* t )
//----------------------------------------------------------------------------------
{
    POCL_OBJECT* ppo = from_timer( ppo, t, timer );
    HandlePoCLEvent( ppo );
}
#  else
//----------------------------------------------------------------------------------
void jit_timer_pocl( unsigned long arg )
//----------------------------------------------------------------------------------
{
    HandlePoCLEvent( ( POCL_OBJECT* )arg );
}
#  endif
#else
//----------------------------------------------------------------------------------
VOID DpcForPoCL( IN PKDPC Dpc, IN PVOID DeferredContext, IN PVOID SystemArgument1, IN PVOID SystemArgument2 )
//----------------------------------------------------------------------------------
{
    HandlePoCLEvent( ( POCL_OBJECT* )DeferredContext );
}
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

//----------------------------------------------------------------------------------
unsigned char GetCLClockStatus( POCL_OBJECT* ppo )
//----------------------------------------------------------------------------------
{
    unsigned char clClockLost = ppo->CLClockLost;
    ppo->CLClockLost = FALSE;
    return clClockLost;
}

//----------------------------------------------------------------------------------
void ChannelLinkReset( POCL_OBJECT* ppo )
//----------------------------------------------------------------------------------
{
#if NIOS_SUPPORTED_TRANSFER
    unsigned int muxControl = IO_READ_32( ppo->RegisterBase, ppo->RegDef, ebrhCLController0, OFF_MUX_CONTROLLER ), mux;
    mux = muxControl;
    muxControl |= MEDIUM_MODE_ENABLE;
    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ebrhCLController0, OFF_MUX_CONTROLLER, muxControl );
    DELAY_EXECUTION_US( 50 );
    muxControl |= RESET_MEDIUM_CHANNELS;
    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ebrhCLController0, OFF_MUX_CONTROLLER, muxControl );
    DELAY_EXECUTION_US( 50 );
    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ebrhCLController0, OFF_MUX_CONTROLLER, mux );
#endif //NIOS_SUPPORTED_TRANSFER
    dbg_print( ( " %s\n", __FUNCTION__ ) );
}

//----------------------------------------------------------------------------------
void PoCLChanged( POCL_OBJECT* ppo, int state )
//----------------------------------------------------------------------------------
{
    ULONG reg = 0;
    dbg_print( ( DRIVERNAME " : %s "" state %d", __FUNCTION__, state ) );
    if( ppo->InitDone && state != ppo->State )
    {
        reg = IO_READ_32( ppo->RegisterBase, ppo->RegDef, ebrhSystemRegister, OFF_SYSTEM_CONTROL );
        switch( state )
        {
        default:
        case 0:
            {
                IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ebrhSystemRegister, OFF_SYSTEM_CONTROL, reg & ~( ENABLE_VOLTAGE_12V ) );
                if( ppo->InitDone == TRUE )
                {
                    CANCEL_TIMER();
                    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_INTR_ENABLE, 0 );
                    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_CONTROL, 0 );
                }
            }
            break;
        case 1:
            {
                IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ebrhSystemRegister, OFF_SYSTEM_CONTROL, reg | ENABLE_VOLTAGE_12V );
                if( ppo->InitDone == TRUE )
                {
#if ENABLE_POCL
                    ppo->Action = epcsConnectionSense;
                    ppo->DetectedCamera = eptNoCameraDetected;
                    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_CONTROL, CL_CLK_OBSERVE );
#if ENABLE_INTERRUPT
                    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_INTR_ENABLE, ( OVERLOAD_INTR | CL_CLK_HALT_INTR ) );
#else //ENABLE_INTERRUPT
                    IO_WRITE_32( ppo->RegisterBase, ppo->RegDef, ppo->RegisterIndex, OFF_POCL_INTR_ENABLE, 0 );
#endif //ENABLE_INTERRUPT
                    HandlePoCLEvent( ppo );
                    ChannelLinkReset( ppo );
#endif //ENABLE_POCL
                }
            }
            break;
        }
        ppo->State = state;
    }
}
