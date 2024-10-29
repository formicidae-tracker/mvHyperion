//-----------------------------------------------------------------------------
#ifndef HYPERION_CONST_H
#define HYPERION_CONST_H HYPERION_CONST_H
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// error numbers
//-----------------------------------------------------------------------------
#define EDEVICE_NOT_READY 0xA3
#ifndef STATUS_CANCELLED
#   define STATUS_CANCELLED 0x100
#endif // #ifndef STATUS_CANCELLED

//-----------------------------------------------------------------------------
// mvHYPERION specific constants
//-----------------------------------------------------------------------------
#define MAX_PHASE_ERROR_RETRY_COUNT 10

#endif // HYPERION_CONST_H