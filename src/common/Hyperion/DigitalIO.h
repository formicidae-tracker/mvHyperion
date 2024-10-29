#ifndef DigitalIOH
#define DigitalIOH DigitalIOH

#include "HyperionRegister.h"

#define INVERTED 1
#define NOT_INVERTED 0

//----------------------------------------------------------------------------------------------
typedef struct _DIGITAL_IN_OUT
//----------------------------------------------------------------------------------------------
{
    int pin;
    int ctrl_index;
    int register_offset;
    int nbit_sync_mode;
    int nbit_sync_select;
    int nbit_input;
    int inverted;
    int firmware_version;
} DIGITAL_IN_OUT;

//----------------------------------------------------------------------------------------------
enum DIGITAL_IO_SYNC_MODE
//----------------------------------------------------------------------------------------------
{
    dsmOff = 0,
    dsmOn,
    dsmSyncSel,
    dsmSyncSelInv
};

/*enum DIGITAL_IO_INPUT_PASSTHROUGH
{
    dipTriggerInput_0 = 0,
    dipSyncInput_0,
    dipTriggerInput_1,
    dipSyncInput_1,
    dipDigIn_0,
    dipDigIn_1,
    dipReserved,
    dipHRTCCh0,
    dipHRTCCh1,
};*/

#define SYNC_MODE   0x3
#define SYNC_SEL    0x7

void ConfigureDigitalOutput( unsigned char* base, HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int output, unsigned int passthrough_inv, unsigned int input_to_passthrough );
void WriteDigitalOutput( unsigned char* base,  HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int output, unsigned int state );
unsigned int ReadDigitalInput( unsigned char* base,  HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int input );
int IsDigitalInputAvailable( unsigned char* base, HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int input );
int IsDigitalOutputAvailable( unsigned char* base, HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int output );

/*
//----------------------------------------------------------------------------------------------
typedef union _ControlRegister
//----------------------------------------------------------------------------------------------
{
    struct
    {
        volatile unsigned int DisableByteWriteAccess : 1;
        volatile unsigned int StartWaitOfFrameEdge : 1;
        volatile unsigned int Reserved0 : 2;
        volatile unsigned int SyncmodeCC1 : 2;
        volatile unsigned int SyncselCC1 : 3;
        volatile unsigned int SyncmodeCC2 : 2;
        volatile unsigned int SyncselCC2 : 3;
        volatile unsigned int SyncmodeCC3 : 2;
        volatile unsigned int SyncselCC3 : 3;
        volatile unsigned int SyncmodeCC4 : 2;
        volatile unsigned int SyncselCC4 : 3;
        volatile unsigned int SyncmodeFlashout : 2;
        volatile unsigned int SyncselFlashout : 3;
        volatile unsigned int Reserved1 : 3;
    }b;
    volatile unsigned int i;
}GENERIC_CONTROL;
*/


/*
//----------------------------------------------------------------------------------------------
typedef union _DigitalIOControlRegister
//----------------------------------------------------------------------------------------------
{
    struct
    {
        volatile unsigned int SyncmodeDigOut0 : 2;
        volatile unsigned int SyncselDigOut0 : 3;
        volatile unsigned int SyncmodeDigOut1 : 2;
        volatile unsigned int SyncselDigOut1 : 3;
        volatile unsigned int Reserved : 18;
        volatile unsigned int DigitalIn0 : 1;
        volatile unsigned int DigitalIn1 : 1;
        volatile unsigned int SyncIn : 1;
        volatile unsigned int TriggerIn : 1;
    }b;
    volatile unsigned int i;
}DIGITALIO_CONTROL;
*/

/*
//----------------------------------------------------------------------------------------------
typedef struct _CameraControl
{
    union
    {
        struct
        {
            volatile unsigned int syncmode_cc1  : 3;
            volatile unsigned int reserved1     : 1;
            volatile unsigned int syncsel_cc1   : 4;
            volatile unsigned int syncmode_cc2  : 3;
            volatile unsigned int reserved2     : 1;
            volatile unsigned int syncsel_cc2   : 4;
            volatile unsigned int syncmode_cc3  : 3;
            volatile unsigned int reserved3     : 1;
            volatile unsigned int syncsel_cc3   : 4;
            volatile unsigned int syncmode_cc4  : 3;
            volatile unsigned int reserved4     : 1;
            volatile unsigned int syncsel_cc4   : 4;
        }b;
        volatile unsigned int i;
    }control;
    union
    {
        struct
        {
            volatile unsigned int syncmode_flash    : 3;
            volatile unsigned int reserved1         : 1;
            volatile unsigned int syncsel_flash     : 4;
            volatile unsigned int syncmode_digout0  : 3;
            volatile unsigned int reserved2         : 1;
            volatile unsigned int syncsel_digout0   : 4;
            volatile unsigned int syncmode_digout1  : 3;
            volatile unsigned int reserved3         : 1;
            volatile unsigned int syncsel_digout1   : 4;
            _fillbits(8);
        }b;
        volatile unsigned int i;
    }extended;
}CAMERA_CONTROL;
*/

#endif //DigitalIOH
