//----------------------------------------------------------------------------------------------
#ifndef PoCLControlH
#define PoCLControlH PoCLControlH
//----------------------------------------------------------------------------------------------

#if defined(linux) || defined(__linux) || defined(__linux__)
#   include "matrix_types.h"
#   include <linux/timer.h>
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

#include "HyperionRegister.h"

//----------------------------------------------------------------------------------------------
enum POCL_CONTROLLER_STATE
//----------------------------------------------------------------------------------------------
{
    epcsConnectionSense = 0,
    epcsCameraCableDetect,
    epcsClockDetect
};

//----------------------------------------------------------------------------------------------
enum POCL_THRESHOLD
//----------------------------------------------------------------------------------------------
{
    eptNonPoCLCamera = 0,
    eptPoCLCamera,
    eptInternalError,
    eptNoCameraDetected
};

//----------------------------------------------------------------------------------------------
typedef struct _POCL_OBJECT
//----------------------------------------------------------------------------------------------
{
    int Index;
    unsigned int InitDone;
#ifdef linux
    struct memory_space RegisterBase;
    struct timer_list timer;
#else
    unsigned char* RegisterBase;
    KDPC PoCLDpc;
    KTIMER Timer;
#endif
    enum POCL_CONTROLLER_STATE Action;
    enum POCL_THRESHOLD DetectedCamera;
    unsigned char CLClockLost;
    HYPERION_BASE_REGISTER_DEF* RegDef;
    int RegisterIndex;
    int State;
} POCL_OBJECT;

void HandlePoCLEvent( POCL_OBJECT* ppo );
int CreatePoCLObject( POCL_OBJECT* ppo, void* base, HYPERION_BASE_REGISTER_DEF* regDef, int regIndex );
void ClosePoCL( POCL_OBJECT* ppo );
unsigned char GetCLClockStatus( POCL_OBJECT* ppo );
void ChannelLinkReset( POCL_OBJECT* ppo );
void PoCLChanged( POCL_OBJECT* ppo, int state );

#endif // PoCLControlH
