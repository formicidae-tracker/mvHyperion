//-----------------------------------------------------------------------------
#ifndef HyperionIoCtlH
#define HyperionIoCtlH HyperionIoCtlH
//-----------------------------------------------------------------------------
#include <linux/types.h>
#include <linux/ioctl.h>

#define DRIVER_NAME "/dev/hyperion"
#define FILE_DEVICE_HYPERION ('h' & 0x3f)       // Linux 'type' restricted to 6 from 8 possible bits

#define MINOR_SERIAL_PORT   64

#define EEPROM_WRITE_ACCESS 0x2103

#ifndef CTL_CODE
//
// Macro definition for defining IOCTL and FSCTL function control codes.
// For Linux it looks like this:

/* ioctl command encoding: 32 bits total, command in lower 16 bits,
 * size of the parameter structure in the lower 14 bits of the
 * upper 16 bits.
 * Encoding the size of the parameter structure in the ioctl request
 * is useful for catching programs compiled with old versions
 * and to avoid overwriting user space outside the user buffer area.
 * The highest 2 bits are reserved for indicating the ``access mode''.
 * NOTE: This limits the max parameter size to 16kB -1 !
 * The generic ioctl numbering scheme doesn't really enforce
 * a type field.  De facto, however, the top 8 bits of the lower 16
 * bits are indeed used as a type field, so we might just as well make
 * this explicit here.
 *
 * #define _IOC_NRBITS  8
 * #define _IOC_TYPEBITS    8
 * #define _IOC_SIZEBITS    14
 * #define _IOC_DIRBITS 2
 *
 * #define _IOC_NRMASK  ((1 << _IOC_NRBITS)-1)
 * #define _IOC_TYPEMASK    ((1 << _IOC_TYPEBITS)-1)
 * #define _IOC_SIZEMASK    ((1 << _IOC_SIZEBITS)-1)
 * #define _IOC_DIRMASK ((1 << _IOC_DIRBITS)-1)
 *
 * #define _IOC_NRSHIFT 0
 * #define _IOC_TYPESHIFT   (_IOC_NRSHIFT+_IOC_NRBITS)
 * #define _IOC_SIZESHIFT   (_IOC_TYPESHIFT+_IOC_TYPEBITS)
 * #define _IOC_DIRSHIFT    (_IOC_SIZESHIFT+_IOC_SIZEBITS)
 *
 * #define _IOC(dir,type,nr,size) \
 *  (((dir)  << _IOC_DIRSHIFT) | \
 *   ((type) << _IOC_TYPESHIFT) | \
 *   ((nr)   << _IOC_NRSHIFT) | \
 *   ((size) << _IOC_SIZESHIFT))
 *
 */

//31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
//D  D  S  S  S  S  S  S  S  S  S  S  S  S  S  S  T  T  T  T  T  T  T  T  N  N  N  N  N  N  N  N

// 'DeviceType' is the bottom 6 bits of the Linux 'type' field (8 bits in total).
// Note: we smuggle the 'Method' parameter into the top 2 bits of the Linux 'type' field.
// 'Access' is the Linux 'dir' field (2 bits).
// 'Function' is the Linux 'nr' field (only 8 bits!).
#define CTL_CODE(DeviceType,Function,Method,Access,size)                    \
    ((__u32)(_IOC((Access & 0x3),(DeviceType & 0x3f)|((Method & 0x3)<<6),(Function & 0xff),(sizeof(size) & 0x3fff))))
// ...and our definition results in this...
//31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
//A  A  S  S  S  S  S  S  S  S  S  S  S  S  S  S  M  M  T  T  T  T  T  T  F  F  F  F  F  F  F  F
#endif

#include <HyperionIoCtlCommon.h>

//-----------------------------------------------------------------------------
enum TFileAccess
//-----------------------------------------------------------------------------
{
    eFileAnyAccess = ( _IOC_READ | _IOC_WRITE ),
    eFileSpecialAccess = ( _IOC_READ | _IOC_WRITE ),
    eFileReadAccess = _IOC_READ,
    eFileWriteAccess = _IOC_WRITE
};

//-----------------------------------------------------------------------------
typedef struct _CameraLinkData
//-----------------------------------------------------------------------------
{
    __u32 port;
    __u32 data;
} CAMERA_LINK_DATA;

//-----------------------------------------------------------------------------
typedef struct _ResultPacket
//-----------------------------------------------------------------------------
{
    __u32 Status;
    void* ResultPacket;
} TResultPacket;

#define IOCTL_QUERYINFO                 CTL_CODE(FILE_DEVICE_HYPERION, 0x00, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_ABORTALLREQUESTS          CTL_CODE(FILE_DEVICE_HYPERION, 0x01, eMethodBuffered, eFileAnyAccess, __u32)
// ioctls for serial communication
#define IOCTL_FLUSHPORT                 CTL_CODE(FILE_DEVICE_HYPERION, 0x02, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_GETNUMOFBYTESAVAIL        CTL_CODE(FILE_DEVICE_HYPERION, 0x03, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_SETBAUDRATE               CTL_CODE(FILE_DEVICE_HYPERION, 0x04, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_SERIALINIT                CTL_CODE(FILE_DEVICE_HYPERION, 0x05, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_SERIALCLOSE               CTL_CODE(FILE_DEVICE_HYPERION, 0x06, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_SERIALREAD                CTL_CODE(FILE_DEVICE_HYPERION, 0x07, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_SERIALWRITE               CTL_CODE(FILE_DEVICE_HYPERION, 0x08, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_READRESULTPACKET          CTL_CODE(FILE_DEVICE_HYPERION, 0x09, eMethodBuffered, eFileAnyAccess, __u32)
// deprecated +
#define IOCTL_EMPTYREQUESTQUEUE         CTL_CODE(FILE_DEVICE_HYPERION, 0x0a, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_EMPTYRESULTQUEUE          CTL_CODE(FILE_DEVICE_HYPERION, 0x0b, eMethodBuffered, eFileAnyAccess, __u32)
// deprecated -
#define IOCTL_RESET_DMA_CONTROLLER      CTL_CODE(FILE_DEVICE_HYPERION, 0x0c, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_READ_EEPROM               CTL_CODE(FILE_DEVICE_HYPERION, 0x0d, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_WRITE_EEPROM              CTL_CODE(FILE_DEVICE_HYPERION, 0x0e, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_CONFIG_IO_BIT             CTL_CODE(FILE_DEVICE_HYPERION, 0x0f, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_WRITE_IO_BIT              CTL_CODE(FILE_DEVICE_HYPERION, 0x10, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_WRITE_MUX_DATA            CTL_CODE(FILE_DEVICE_HYPERION, 0x11, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_CLEANUP_REQUEST_EXTENSION CTL_CODE(FILE_DEVICE_HYPERION, 0x12, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_READ_ASMI_U32             CTL_CODE(FILE_DEVICE_HYPERION, 0x13, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_WRITE_ASMI_U32            CTL_CODE(FILE_DEVICE_HYPERION, 0x14, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_WRITE_HRTC_U32            CTL_CODE(FILE_DEVICE_HYPERION, 0x15, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_RELOAD_DEVICE_INFORMATION CTL_CODE(FILE_DEVICE_HYPERION, 0x16, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_READ_DIGITAL_INPUT        CTL_CODE(FILE_DEVICE_HYPERION, 0x17, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_WRITE_HRTC                CTL_CODE(FILE_DEVICE_HYPERION, 0x18, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_TRIGGER_SOFTWARE          CTL_CODE(FILE_DEVICE_HYPERION, 0x19, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_UNMAP_SG_LIST             CTL_CODE(FILE_DEVICE_HYPERION, 0x1A, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_BOOT_NIOS                 CTL_CODE(FILE_DEVICE_HYPERION, 0x1B, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_POWER_OVER_CL             CTL_CODE(FILE_DEVICE_HYPERION, 0x1C, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_I2C_READ_BYTE             CTL_CODE(FILE_DEVICE_HYPERION, 0x1D, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_I2C_WRITE_BYTE            CTL_CODE(FILE_DEVICE_HYPERION, 0x1E, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_SPI_READ                  CTL_CODE(FILE_DEVICE_HYPERION, 0x1F, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_SPI_WRITE                 CTL_CODE(FILE_DEVICE_HYPERION, 0x20, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_READ_REGISTER_U32         CTL_CODE(FILE_DEVICE_HYPERION, 0x21, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_WRITE_REGISTER_U32        CTL_CODE(FILE_DEVICE_HYPERION, 0x22, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_EEPROM_ACCESS             CTL_CODE(FILE_DEVICE_HYPERION, 0x23, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_WRITE_DMA_BUFFER_SIZE     CTL_CODE(FILE_DEVICE_HYPERION, 0x24, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_SPI_READ_LEN              CTL_CODE(FILE_DEVICE_HYPERION, 0x25, eMethodBuffered, eFileAnyAccess, __u32)
#define IOCTL_QUERY_CAPABILITIES        CTL_CODE(FILE_DEVICE_HYPERION, 0x26, eMethodBuffered, eFileAnyAccess, __u32)


#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9,0)
#define HAVE_UNLOCKED_IOCTL 1
#endif

#endif // HyperionIoCtlH
