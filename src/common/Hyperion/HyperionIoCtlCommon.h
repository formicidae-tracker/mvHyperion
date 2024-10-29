//-----------------------------------------------------------------------------
#ifndef HyperionIoCtlCommonH
#define HyperionIoCtlCommonH HyperionIoCtlCommonH
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
enum TMethodIoControls
//-----------------------------------------------------------------------------
{
    eMethodBuffered = 0,
    eMethodInDirect = 1,
    eMethodOutDirect = 2,
    eMethodNeither = 3
};

//-----------------------------------------------------------------------------
typedef struct _EepromAccess
//-----------------------------------------------------------------------------
{
    unsigned long SubAddr;
    unsigned long Length;
    unsigned char* Data;
} TEepromAccess;

//-----------------------------------------------------------------------------
typedef struct _RegisterAccess
//-----------------------------------------------------------------------------
{
    unsigned long Offset;
    unsigned long Data;
} TRegisterAccess;

//-----------------------------------------------------------------------------
typedef struct _SPIAccess
//-----------------------------------------------------------------------------
{
    unsigned long Offset;
    unsigned long Data;
    unsigned long Slave;
    union
    {
        unsigned short type;
        struct
        {
            unsigned short sdid : 8;
            unsigned short did : 8;
        } s;
    } u;
} TSPIAccess;

//-----------------------------------------------------------------------------
typedef struct _I2CAccess
//-----------------------------------------------------------------------------
{
    int Address;
    int SubAddress;
    unsigned char Data;
} TI2CAccess;

//-----------------------------------------------------------------------------
typedef struct _UserBuffer
//-----------------------------------------------------------------------------
{
    char* buffer;
    unsigned long count;
} TUserBuffer;

//-----------------------------------------------------------------------------
typedef struct _QueryInformation
//-----------------------------------------------------------------------------
{
    int informationID;
    union
    {
        int intElement;
        void* vPtrElement;
        unsigned long long ullElement;
    } u;
    int typeSize;
} TQueryInformation;

//----------------------------------------------------------------------------
typedef struct _HyperionRequestBuffer
//----------------------------------------------------------------------------
{
    unsigned long      size; // of this struct
    unsigned long long userModePayloadBufferPtr;
    unsigned long long userModePayloadBufferSize;
    unsigned long long leaderBufferPtr;
    unsigned long      leaderBufferSize;
    unsigned long long trailerBufferPtr;
    unsigned long      trailerBufferSize;
} HyperionRequestBuffer;

#endif // HyperionIoCtlCommonH
