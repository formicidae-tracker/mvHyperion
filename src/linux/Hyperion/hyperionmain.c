/*
 * hyperionmain.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: hyperionmain.c,v 1.100 2011-03-29 12:46:53 ug Exp $
 *
 */
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the ter ms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <linux/version.h>

#ifndef KERNEL_VERSION
#define KERNEL_VERSION( a, b, c ) ( ( ( a ) << 16 ) | ( ( b ) << 8 ) | ( c ) )
#endif

// Use versioning if needed
#if defined( CONFIG_MODVERSIONS ) && !defined( MODVERSIONS )
#define MODVERSIONS
#endif

#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#include "ioctl_hyperion.h"
#include "matrix_tools.h" // <== this should be put in a common directory for all drivers!

#include "DigitalIO.h"
#include "HyperionIoCtl.h"
#include "hyperion.h"
#include "hyperion_generic.h"
#include "i2c_access.h"
#include "property.h"
#include "read_write.h"

/* use following #define to use memory base mapping 1 */
// #define USE_MEMBASE_1

#define NO_MORE_HYPERIONS 1

#if DEBUG
#define MSG_DBG_BIT 16
#define TM_DBG_BIT 17
#define SLOT_DBG_BIT 18
#define SEM_DBG_BIT 19
/* from WA */
#define MSGV_DBG_BIT 24 // verbose messages
#define ENDIAN_DBG_BIT 25

#define IRQ_INITIALIZED BIT( 31 )

unsigned long debug
    = 0; // BASE_SYS_DEBUG | _flg(TM) | _flg(SLOT) | _flg(SEM) ;
#endif

static struct hyperion_device *hyperions[MAXHYPERIONS] = { NULL };
static unsigned int irq_map_hyperion[MAXHYPERIONS] = {0, 0, 0, 0,
                                                      0, 0, 0, 0,
                                                      0, 0, 0, 0,
                                                      0, 0, 0, 0
                                                     };

static const char intname[MAXHYPERIONS][12] =
{
    {"hyperion0"}, {"hyperion1"}, {"hyperion2"}, {"hyperion3"},
    {"hyperion4"}, {"hyperion5"}, {"hyperion6"}, {"hyperion7"},
    {"hyperion8"}, {"hyperion9"}, {"hyperion10"}, {"hyperion11"},
    {"hyperion12"}, {"hyperion13"}, {"hyperion14"}, {"hyperion15"}
};

// module parameters
static HYPERION_BASE_REGISTER_DEF CLeRegisterBaseA32[ebrhMax] =
{
#include "hyperion_register_a32.h"
};

static HYPERION_BASE_REGISTER_DEF CL4eRegisterBase[ebrhMax] =
{
#include "hyperion_register_cl4e.h"
};


// used for dynamic allocation of major device
static int major_dev_num = HYPERION_MAJOR;
static int minor_dev_num = 0;

#define HYPERION_BUILD_NUMBER 21
// PCI driver registration
static struct pci_driver hyperion_pci_driver;

// forward declaration
static int init_pci( void );

// module parameters
#include <linux/moduleparam.h>


MODULE_AUTHOR( "MATRIX VISION GmbH <info@matrix-vision.de>" );
MODULE_DESCRIPTION( "MATRIX VISION mvHYPERION Framegrabber" );
MODULE_LICENSE( "GPL" );
#if DEBUG
module_param( debug, long, 0 );
#endif
module_param( major_dev_num, int, 0 );

struct SItem
{
    struct page* page;
    void* vmem;
};


#define MAP_MEMBASE_1 0
//-------------------------------------------------------------------------------------------
void add_to_cleanup_pipe( struct hyperion_device* device, struct page* page, void* vmem )
//-------------------------------------------------------------------------------------------
{
    TItem it;
    BOOLEAN writeOK;
    it.page = page;
    it.vmem = vmem;
    _PIPE_WRITE( device->cleanup_request_pipe, it, writeOK );
}

//-------------------------------------------------------------------------------------------
void remove_objects_from_cleanup_pipe( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    TItem it;
    BOOLEAN readOK;
    while( _ITEMS_IN_PIPE( device->cleanup_request_pipe ) )
    {
        it.page = NULL;
        it.vmem = NULL;
        _PIPE_READ( device->cleanup_request_pipe, &it, readOK );
        if( it.page != NULL )
        {
            kunmap( it.page );
        }
        if( it.vmem != NULL )
        {
            PRINTKM( MEM, ( PKTD " %s free vmem %p\n", device->index, __FUNCTION__,  it.vmem ) );
            vfree( it.vmem );
        }
    }
}
//-------------------------------------------------------------------------------------------
struct hyperion_device* get_hyperion_device( unsigned int index )
//-------------------------------------------------------------------------------------------
{
    if( index < MAXHYPERIONS )
    {
        return hyperions[index];
    }
    else
    {
        return NULL;
    }
}

//-------------------------------------------------------------------------------------------
static int hyperion_open( struct inode* inode, struct file* file )
//-------------------------------------------------------------------------------------------
{
    int result = 0;
    struct hyperion_device* device;
    int minor_num, dev_index;

    //device = container_of( inode->i_cdev, struct hyperion_device, cdev );
    minor_num = MINOR( inode->i_cdev->dev );
    dev_index = minor_num & ( MAXHYPERIONS - 1 );
    device = hyperions[dev_index];

    PRINTKM( FILE, ( PKTD "open mvHYPERION device %p minor %d dev_index %d\n", device->index, device, minor_num, dev_index ) );

    if( file )
    {
        file->private_data = ( void* ) device;
    }

    return result;
}

//-------------------------------------------------------------------------------------------
static int hyperion_close( struct inode* inode, struct file* file )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device ;

    device = container_of( inode->i_cdev, struct hyperion_device, cdev );
    PRINTKM( FILE, ( PKTD "close hyperion %d\n", device->index, device->index ) );

    /// \todo remove device from file->private_data
    /// \free all allocated structs dma controller, interrupt etc.
    //  hyp->flags = dsNotInitialized;

    return 0 ;
}


struct mv_ioctl
{
    int in_size;
    int out_size;
    unsigned long bytes_returned;
};

#define __case(case_label)                          \
    case case_label:                                \
    PRINTKM (IOCTL,(PKTD "case label =" #case_label "\n", hyp->number));
#define IOBUFFER_TO(type) (type)(io_buffer+sizeof(struct mv_ioctl))
//-------------------------------------------------------------------------------------------
#if HAVE_UNLOCKED_IOCTL
long hyperion_ioctl( struct file* file, unsigned int cmd, unsigned long arg )
#else
static int hyperion_ioctl( struct inode* inode, struct file* file, unsigned int cmd, unsigned long arg )
#endif
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device = ( struct hyperion_device* )file->private_data;
    int error = 0;
    //u_long arg_val;
    TUserVirtualAddress uva ;
    u_short dir = _IOC_DIR( cmd );
    u_short size = _IOC_SIZE( cmd );
    struct mv_ioctl mvio;
    char* io_buffer;
    unsigned long io_buffer_size;
    unsigned char write_back_to_user = FALSE;
    uva.uptr = arg;
    PRINTKM( IOCTL, ( PKTD "ioctl cmd=0x%08x, dir=%d, size=%d, arg=0x%08lx\n", device->index, cmd, dir, size, arg ) );
//   if ((dir == _IOC_WRITE) && (size > 0) && (size <= sizeof (u_long)))
    //{
    //  if ((error = read_user ( uva, &arg_val, size)) != 0)
    //  {
    //      printk (PKTD "ioctl * read_user error = %d\n", device->index, error);
    //      return error;
    //  }
    //}

    read_user( uva, &mvio, sizeof( mvio ) );
    io_buffer_size = max( mvio.in_size, mvio.out_size ) + sizeof( mvio );
    io_buffer = kmalloc( io_buffer_size, GFP_KERNEL );
    if( io_buffer == NULL )
    {
        PRINTKM( IOCTL, ( PKTD "ioctl cmd 0x%08x can't allocate io_buffer\n", device->index, cmd ) );
        return -EINVAL;
    }
    read_user( uva, io_buffer, io_buffer_size );
    PRINTKM( IOCTL, ( PKTD "ioctl cmd 0x%08x  insize %d osize %d\n", device->index, cmd, mvio.in_size, mvio.out_size ) );

    switch( cmd )
    {
    case IOCTL_QUERYINFO:
        {
            u32* info_to_query = IOBUFFER_TO( void* );
            switch( *info_to_query )
            {
            case eqiEepromContent:
                break;
            case eqiHRTCFrequency:
                {
                    u32 bytes_returned, hrtc_freq[2];
                    if( IO_READ_32( device->hyperion_base, device->reg_def, ebrhHrtController0, OFF_HRT_CONTROLLER_VER ) > 1 )
                    {
                        hrtc_freq[0] = IO_READ_32( device->hyperion_base, device->reg_def, ebrhHrtController0, OFF_HRT_CONTROLLER_CLK );
                        hrtc_freq[1] = IO_READ_32( device->hyperion_base, device->reg_def, ebrhHrtController1, OFF_HRT_CONTROLLER_CLK );
                    }
                    else
                    {
                        hrtc_freq[0] = HRTC_VER1_FREQ_HZ;
                        hrtc_freq[1] = hrtc_freq[0];
                    }
                    //PRINTKM (IOCTL,(PKTD " hrtcFreq[0] %d hrtcFreq[1] %d cbout %d arraysize %lu\n", device->index, hrtc_freq[0], hrtc_freq[1], mvio.out_size, arraysize(hrtc_freq) ));
                    bytes_returned = arraysize( hrtc_freq ) * sizeof( u32 ) < mvio.out_size ? arraysize( hrtc_freq ) * sizeof( u32 ) : mvio.out_size;
                    memcpy( IOBUFFER_TO( void* ), hrtc_freq, bytes_returned );
                    ( ( struct mv_ioctl* )io_buffer )->bytes_returned = bytes_returned;
                    write_back_to_user = TRUE;
                    break;
                }
            case eqiDigitalOutputAvailable:
                {
                    if( mvio.in_size >= sizeof( unsigned int ) && mvio.out_size >= sizeof( unsigned int ) )
                    {
                        *IOBUFFER_TO( unsigned int* ) = IsDigitalOutputAvailable( device->hyperion_base.base, device->reg_def, *( IOBUFFER_TO( unsigned int* ) + 1 ) );
                        PRINTKM( IOCTL, ( PKTD " %s, output %d available %d\n", device->index, __FUNCTION__, *( IOBUFFER_TO( unsigned int* ) + 1 ), *IOBUFFER_TO( unsigned int* ) ) );
                        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
                        write_back_to_user = TRUE;
                    }
                    else
                    {
                        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = 0;
                    }
                    break;
                }
            case eqiDigitalInputAvailable:
                {
                    if( mvio.in_size >= sizeof( unsigned int ) && mvio.out_size >= sizeof( unsigned int ) )
                    {
                        *IOBUFFER_TO( unsigned int* ) = IsDigitalInputAvailable( device->hyperion_base.base, device->reg_def, *( IOBUFFER_TO( unsigned int* ) + 1 ) );
                        PRINTKM( IOCTL, ( PKTD " %s, input %d available %d\n", device->index, __FUNCTION__, *( IOBUFFER_TO( unsigned int* ) + 1 ), *IOBUFFER_TO( unsigned int* ) ) );
                        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
                        write_back_to_user = TRUE;
                    }
                    else
                    {
                        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = 0;
                    }
                    break;
                }
            case eqiTriggerModeAvailable:
                {
                    if( mvio.in_size >= sizeof( unsigned int ) && mvio.out_size >= sizeof( unsigned int ) )
                    {
                        //currently removed
                        //unsigned int videoin_version = IO_READ_32(device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_VERSION);
                        //*IOBUFFER_TO(unsigned int*) = ((videoin_version & VERSIONS_NUMBER_MSK) >= TRIGGER_MODE_AVAILABLE_VERSION);
                        *IOBUFFER_TO( unsigned int* ) = FALSE;
                        PRINTKM( IOCTL, ( PKTD " %s, trigger_mode available %d\n", device->index, __FUNCTION__, *( IOBUFFER_TO( unsigned int* ) ) ) );
                        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
                        write_back_to_user = TRUE;
                    }
                    else
                    {
                        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = 0;
                    }
                    break;
                }
            }
            break;
        }
    case IOCTL_ABORTALLREQUESTS:
        {
            abort_transfer( device );
            break;
        }
//ioctls for serial communication
    case IOCTL_FLUSHPORT:
        {
            CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
            if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
            {
                reset_read_write_fifo( &device->uart_port[cl_data->port] );
            }
            break;
        }
    case IOCTL_GETNUMOFBYTESAVAIL:
        {
            unsigned long bytes_available = 0;
            CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
            if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
            {
                bytes_available = getnumber_bytes_available( &device->uart_port[cl_data->port] );
            }

            if( mvio.out_size >= sizeof( unsigned long ) )
            {
                memcpy( IOBUFFER_TO( void* ), ( void* )&bytes_available, sizeof( unsigned long ) );
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = sizeof( unsigned long );
                write_back_to_user = TRUE;
            }
            break;
        }
    case IOCTL_SETBAUDRATE:
        {
            CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
            if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
            {
                set_baudrate( &device->uart_port[cl_data->port], cl_data->data );
            }
            break;
        }
    case IOCTL_SERIALINIT:
        {
            CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
            if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
            {
                PRINTKM( IOCTL, ( PKTD "ioctl_serialinit port %d\n", device->index, cl_data->port ) );
                reset_read_write_fifo( &device->uart_port[cl_data->port] );
                error = 1;
                //if successful return 1, on error return 0
            }
            break;
        }
    case IOCTL_SERIALCLOSE:
        {
            break;
        }
    case IOCTL_SERIALREAD:
        {
            unsigned long read_status;
            CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
            if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
            {
                read_status = read_serial_data( &device->uart_port[cl_data->port], cl_data->data, IOBUFFER_TO( unsigned char* ), mvio.out_size );
                if( read_status == secNoError )
                {
                    ( ( struct mv_ioctl* )io_buffer )->bytes_returned = ( read_status == secNoError ) ? mvio.out_size : 0;
                }
                write_back_to_user = TRUE;
            }
            break;
        }
    case IOCTL_SERIALWRITE:
        {
            unsigned long bytes_written;
            CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
            if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
            {
                bytes_written = write_serial_data( &device->uart_port[cl_data->port], cl_data->data, IOBUFFER_TO( unsigned char* ) + sizeof( CAMERA_LINK_DATA ), mvio.in_size - sizeof( CAMERA_LINK_DATA ) );
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = bytes_written;
                write_back_to_user = TRUE;
            }
            break;
        }
    case IOCTL_READRESULTPACKET:
        /*{
            TResultPacket rp;
            result_packet_entry_t *entry = pop_result_packet_entry( &device->result_queue );
            if( entry != NULL )
            {
                rp.ResultPacket = entry->result_packet;
                rp.Status = entry->status;
                free_result_packet_entry( entry );
            }
            else
            {
                rp.ResultPacket = NULL;
                rp.Status = cerrResultQueueEmpty;
            }
            PRINTKM (IOCTL,(PKTD "ioctl cmd=0x%08x resultpacket %p stat %u32\n", device->index, cmd,  rp.ResultPacket, rp.Status ));
            if( mvio.out_buffer != NULL && mvio.out_size >= sizeof( TResultPacket ) )
            {
                uva.uptr = (unsigned long)mvio.out_buffer;
                mvio.bytes_returned = write_user_buffer( &rp, uva, mvio.out_size );
            }
        }*/
        break;
    case IOCTL_EMPTYREQUESTQUEUE:
        {
            break;
        }
    case IOCTL_EMPTYRESULTQUEUE:
        /*{
            empty_result_queue( &device->result_queue );
        }*/
        break;
    case IOCTL_RESET_DMA_CONTROLLER:
        {
            u32 i, system_control;
            for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
            {
                if( device->pdma_object[i]->init_done == FALSE )
                {
                    continue;
                }
                //ResetDMAController( device->pdma_object[0]->dma_controller_pci );
                ResetDMAController( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0] );
                ResetDMAController( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo1] );
                device->pdma_object[i]->restart_current_ioobj = FALSE;
            }
            system_control = IO_READ_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL );
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL, ( system_control | RESET_CL_RECEIVER ) );
            wait_jiffies( 1 ); //delay register access
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL, system_control );
            wait_jiffies( msecs_to_jiffies( 20 ) );
            free_all_ubuf( device );
            break;
        }
    case IOCTL_READ_EEPROM:
        {
            if( mvio.out_size <= EEPROM_SIZE )
            {
                I2CReceiveData( ( unsigned int* )REG_POINTER( device->hyperion_base, device->reg_def, ebrhI2CRead, 0 ), EEPROM_ADDR, 0, mvio.out_size, IOBUFFER_TO( unsigned char* ) );
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
                write_back_to_user = TRUE;
            }
            break;
        }
    case IOCTL_WRITE_EEPROM:
        {
            if( mvio.in_size <= EEPROM_SIZE && device->eeprom_write_access == ( PCI_VENDOR_ID_MATRIX_VISION << 16 && EEPROM_WRITE_ACCESS ) )
            {
                int i;
                unsigned char* eepromdata = IOBUFFER_TO( unsigned char* );
                PRINTKM( IOCTL, ( PKTD " %s, write_eeprom in_size %d\n", device->index, __FUNCTION__, mvio.in_size ) );
                for( i = 0; i < mvio.in_size; i++ )
                {
                    I2CSendData( ( unsigned int* )REG_POINTER( device->hyperion_base, device->reg_def, ebrhI2CRead, 0 ), EEPROM_ADDR, i, 1, &eepromdata[i] );
                }
                device->eeprom_write_access = -1;
            }
            break;
        }
    case IOCTL_EEPROM_ACCESS:
        {
            if( mvio.in_size >= sizeof( int ) )
            {
                int* access = IOBUFFER_TO( int* );
                device->eeprom_write_access = *access;
                PRINTKM( IOCTL, ( PKTD " %s, eeprom_write_access %x\n", device->index, __FUNCTION__, device->eeprom_write_access ) );
            }
            break;
        }
    case IOCTL_CONFIG_IO_BIT:
        {
            TPropertyElement* propElement;
            u32 property_count = 0, output = 0, passthrough_inv = 0, input_to_passthrough = 0;
            unsigned char changed = FALSE;

            property_count = mvio.in_size / sizeof( TPropertyElement );
            propElement = IOBUFFER_TO( TPropertyElement* );
            _READ_PROPERTYI( propElement, property_count, prIOOutput, output, changed );
            _READ_PROPERTYI( propElement, property_count, prPassThroughInverted, passthrough_inv, changed );
            _READ_PROPERTYI( propElement, property_count, prDigInPassThrough, input_to_passthrough, changed );
            ConfigureDigitalOutput( device->hyperion_base.base, device->reg_def, output, passthrough_inv, input_to_passthrough );
            //PRINTKM (IOCTL,(PKTD " configio: output %u syncmode %x syncsel %x\n", device->index, output, sync_mode, input_to_passthrough ));
            wmb();
            break;
        }
    case IOCTL_WRITE_IO_BIT:
        {
            TPropertyElement* propElement;
            u32 property_count = 0, output = 0, state = 0;
            unsigned char changed = FALSE;

            property_count = mvio.in_size / sizeof( TPropertyElement );
            propElement = IOBUFFER_TO( TPropertyElement* );
            _READ_PROPERTYI( propElement, property_count, prIOOutput, output, changed );
            _READ_PROPERTYI( propElement, property_count, prIOState, state, changed );
            WriteDigitalOutput( device->hyperion_base.base, device->reg_def, output, state );
            //PRINTKM (IOCTL,(PKTD " writeiobit: propcnt %u output %u state %u\n", device->index, property_count, output, state ));
            wmb();
            break;
        }
    case IOCTL_WRITE_MUX_DATA:
        {
            if( mvio.in_size <= MUX_CONTROLLER_SEQUENCE_SIZE_BYTES )
            {
                device->mux_seq.changed = TRUE;
                device->mux_seq.size = mvio.in_size;
                memcpy( ( void* )device->mux_seq.muxdata, IOBUFFER_TO( void* ), mvio.in_size );
            }
            break;
        }
    case IOCTL_READ_ASMI_U32:
        {
            if( mvio.in_size >= sizeof( TRegisterAccess ) )
            {
                TRegisterAccess* regacc = IOBUFFER_TO( TRegisterAccess* );
                regacc->Data = ( unsigned long )IO_READ_32( device->hyperion_base, device->reg_def, ebrhAsmiInterface, regacc->Offset );
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.in_size;
                write_back_to_user = TRUE;
                //PRINTKM (IOCTL,(PKTD " IOCTL_READ_ASMI_U32 pasmi %p data read %x\n", device->index, pasmi_reg, regacc->Data ));
            }
            break;
        }
    case IOCTL_WRITE_ASMI_U32:
        {
            if( mvio.in_size >= sizeof( TRegisterAccess ) )
            {
                TRegisterAccess* regacc = IOBUFFER_TO( TRegisterAccess* );
                IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhAsmiInterface, regacc->Offset, regacc->Data );
                //PRINTKM (IOCTL,(PKTD " IOCTL_WRITE_ASMI_U32 pasmi %p data %x\n", device->index, pasmi_reg, regacc->Data ));
            }
            break;
        }
    case IOCTL_WRITE_HRTC_U32:
        {
            if( mvio.in_size >= sizeof( TRegisterAccess ) && device->hrtc_version != -1 )
            {
                TRegisterAccess* regacc = IOBUFFER_TO( TRegisterAccess* );
                //unsigned long *phrtc_reg = (unsigned long*)(device->hyperion_register.base + regacc->Offset);
                //*phrtc_reg = regacc->Data;
                iowrite32( regacc->Data, ( void* )( device->hyperion_base.base + regacc->Offset ) );
            }
            break;
        }
    case IOCTL_CLEANUP_REQUEST_EXTENSION:
        {
#ifdef REMOVE_REQUEST_BUFFER_MAPPING
            release_mapping( device );
#else
            remove_objects_from_cleanup_pipe( device );
#endif
            break;
        }
    case IOCTL_READ_DIGITAL_INPUT:
        {
            if( mvio.in_size >= sizeof( unsigned int ) )
            {
                unsigned int state = ReadDigitalInput( device->hyperion_base.base, device->reg_def, *IOBUFFER_TO( unsigned int* ) );
                *IOBUFFER_TO( unsigned int* ) = state;
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
                write_back_to_user = TRUE;
            }
            break;
        }
    case IOCTL_WRITE_HRTC:
        {
            TPropertyElement* prop_element, *prop_hrtc_ram;
            u32 i, property_count = 0, hrtc_index = 0, hrtc_control = 0;
            unsigned char changed = FALSE;

            property_count = mvio.in_size / sizeof( TPropertyElement );
            prop_element = IOBUFFER_TO( TPropertyElement* );
            _READ_PROPERTYI( prop_element, property_count, prHRTCIndex, hrtc_index, changed );
            _READ_PROPERTYI( prop_element, property_count, prHRTCControl, hrtc_control, changed );
            _SCAN_PROPERTY_LIST( prop_element, prop_hrtc_ram, property_count, prHRTCRAMData );
            if( prop_hrtc_ram )
            {
                property_count -= ( ( ( char* )prop_hrtc_ram - ( char* )prop_element ) / sizeof( TPropertyElement ) );
                for( i = 0; i < property_count; i++ )
                {
                    IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhHrtController0 + hrtc_index, ( OFF_HRT_CONTROLLER_RAM + ( i * sizeof( u32 ) ) ), prop_hrtc_ram->u.intElement );
//      printk(PKTD "IOCTL_WRITE_HRTC off %p val %x\n", device->index, REG_POINTER(device->hyperion_base, device->reg_def, ebrhHrtController0+hrtc_index, OFF_HRT_CONTROLLER_RAM+(i*sizeof(u32))), prop_hrtc_ram->u.intElement );
                    prop_hrtc_ram++;
                }
            }
            IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhHrtController0 + hrtc_index, OFF_HRT_CONTROLLER_CTRL, hrtc_control );
            wmb();
            break;
        }
    case IOCTL_UNMAP_SG_LIST:
        {
#ifndef REMOVE_REQUEST_BUFFER_MAPPING
            if( mvio.in_size >= sizeof( TUserBuffer ) )
            {
                TUserBuffer* ubuf = IOBUFFER_TO( TUserBuffer* );
                free_ubuf_obj( device, ubuf->buffer, ubuf->count );
            }
#endif
            break;
        }
    default:
        printk( PKTD "ioctl: invalid ioctl request, cmd=$%08x, size=%d, dir=%d\n", device->index, cmd, size, dir );
        error = -EINVAL;
        break;
    }

    if( write_back_to_user )
    {
        write_user_buffer( ( void* )io_buffer, uva, io_buffer_size );
    }
    kfree( io_buffer );
#if HAVE_UNLOCKED_IOCTL
    return ( long )error;
#else
    return error;
#endif
}

//-------------------------------------------------------------------------------------------
static int hyperion_mmap( struct file* file, struct vm_area_struct* vma )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device = ( struct hyperion_device* )file->private_data;
    unsigned long start;
    int result = 0;

    start = ( unsigned long ) vma->vm_start;
    if( start & ~PAGE_MASK )
    {
        PRINTKM( MEM, ( PKTD "mmap: invalid start=0x%08lx\n", device->index, start ) );
        return -EFAULT;
    }
#if 0
    // TO DO
    switch( hyp->space_type )
    {
    // check valid space_type
    case DRAM_IX:
    case MMIO_IX:
        result = vmemspace_mmap( hyp->VmemBuffers, vma );
        break;
    case DMA_MMAP_SPACE:                                // map dma buffer
    case PHYS_MMAP_SPACE:                               // map physical memory
        result = mmap_phys_space( vma );
        hyp->space_type = DRAM_IX ; // ????
        break;
    default:
        PRINTKM( MEM, ( PKTD "mmap: invalid space_type=%d\n", hyp->number, hyp->space_type ) );
        result  = -EINVAL;
    }
#endif
    return result;
}

//-------------------------------------------------------------------------------------------
static void* space_map( unsigned long physical_address, unsigned long size )
//-------------------------------------------------------------------------------------------
{
    void* ptr;
    unsigned long base_page   = physical_address & PAGE_MASK;
    unsigned long base_offset = physical_address & ~PAGE_MASK;

    ptr = ( void* )( base_offset | ( unsigned long ) ioremap( base_page, size ) );
    PRINTKM( MEM, ( PKTB "map space 0x%lx -> $%p\n", physical_address, ptr ) );
    return ptr;
}

//-------------------------------------------------------------------------------------------
static void space_unmap( void** ptr )
//-------------------------------------------------------------------------------------------
{
    if( *ptr != NULL )
    {
        iounmap( *ptr );
        *ptr = NULL;
    }
}

//-------------------------------------------------------------------------------------------
static void space_get_memory_resource( struct pci_dev* pci_device, spinlock_t lock,
                                       unsigned long index, unsigned long* address, unsigned long* size )
//-------------------------------------------------------------------------------------------
{
    *address = pci_resource_start( pci_device, index );
    *address &= ~15UL;
    *size = pci_resource_len( pci_device, index );

    PRINTKM( MEM, ( PKTB "get pcibase ix=%ld, 0x%lx..+0x%lx\n", index, *address, *size ) );
}

//-------------------------------------------------------------------------------------------
static struct file_operations hyperion_fops =
//-------------------------------------------------------------------------------------------
{
    .owner      = THIS_MODULE,
    //.read     = hyperion_read,
    //.write    = hyperion_write,

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
    .aio_read   = async_io_read,
    .aio_write  = async_io_write,
#else
    .read_iter   = async_io_read,
    .write_iter  = async_io_write,
#endif
#if HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl = hyperion_ioctl,
#else
    .ioctl      = hyperion_ioctl,
#endif
    .mmap       = hyperion_mmap,
    .open       = hyperion_open,
    .release    = hyperion_close,
    .poll       = hyperion_poll,
};

//-------------------------------------------------------------------------------------------
// sysfs entries
//-------------------------------------------------------------------------------------------
// TODO for Kernel 2.6.20 ??: Much of the sysfs-related code has been changed to use struct device in place of struct class_device.
// The latter structure will eventually go away as the class and device mechanisms are merged.

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_board_type_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_board_type_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device;
    struct _HYPERION_EEPROM_CONTENT hyperion_eeprom;

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    device = ( struct hyperion_device* )dev_get_drvdata( class_dev );
#else
    device = ( struct hyperion_device* )class_get_devdata( class_dev );
#endif
    I2CReceiveData( ( unsigned int* )REG_POINTER( device->hyperion_base, device->reg_def, ebrhI2CRead, 0 ), EEPROM_ADDR, 0, EEPROM_SIZE, ( unsigned char* )&hyperion_eeprom );
    /* return the total number of hyperion devices found */
    return snprintf( buf, PAGE_SIZE, "%s\n", hyperion_eeprom.type );
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( board_type, S_IRUGO, hyperion_board_type_show, NULL );
#else
CLASS_DEVICE_ATTR( board_type, S_IRUGO, hyperion_board_type_show, NULL );
#endif

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_serial_number_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_serial_number_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    /* return the serial number of the hyperion device */
    struct hyperion_device* device;
    struct _HYPERION_EEPROM_CONTENT hyperion_eeprom;

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    device = ( struct hyperion_device* )dev_get_drvdata( class_dev );
#else
    device = ( struct hyperion_device* )class_get_devdata( class_dev );
#endif
    I2CReceiveData( ( unsigned int* )REG_POINTER( device->hyperion_base, device->reg_def, ebrhI2CRead, 0 ), EEPROM_ADDR, 0, EEPROM_SIZE, ( unsigned char* )&hyperion_eeprom );
    printk( KERN_INFO  "%s() iicreg p%p serial %s\n", __FUNCTION__, REG_POINTER( device->hyperion_base, device->reg_def, ebrhI2CRead, 0 ), hyperion_eeprom.serial );
    return snprintf( buf, PAGE_SIZE, "%s\n", hyperion_eeprom.serial );

}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( serialno, S_IRUGO, hyperion_serial_number_show, NULL );
#else
CLASS_DEVICE_ATTR( serialno, S_IRUGO, hyperion_serial_number_show, NULL );
#endif

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_firmware_version_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_firmware_version_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* device;
    u32 videoInVersion, uartVersion, systemVersion, fpgaVersion, hrtcVersion;

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    device = ( struct hyperion_device* )dev_get_drvdata( class_dev );
#else
    device = ( struct hyperion_device* )class_get_devdata( class_dev );
#endif
    videoInVersion = IO_READ_32( device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_VERSION );
    if( videoInVersion != -1 )
    {
        videoInVersion &= VERSIONS_NUMBER_MSK;
    }
    else
    {
        videoInVersion = 0;
    }
    uartVersion = IO_READ_32( device->hyperion_base, device->reg_def, ebrhUart0, OFF_UART_CONTROL ) >> UART_CTRL_VERSION;
    if( uartVersion == 0xff )
    {
        uartVersion = 0;
    }
    systemVersion = IO_READ_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_VERSION );
    if( systemVersion == -1 )
    {
        systemVersion = 0;
    }
    hrtcVersion = IO_READ_32( device->hyperion_base, device->reg_def, ebrhHrtController0, OFF_HRT_CONTROLLER_VER );
    if( hrtcVersion == -1 )
    {
        hrtcVersion = 0;
    }
    fpgaVersion = videoInVersion + uartVersion + systemVersion + hrtcVersion;
    printk( KERN_INFO  "hyperion_firmware_version_show() firmwareversion %d\n", fpgaVersion );
    return snprintf( buf, PAGE_SIZE, "%d\n", fpgaVersion );
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( firmware_version, S_IRUGO, hyperion_firmware_version_show, NULL );
#else
CLASS_DEVICE_ATTR( firmware_version, S_IRUGO, hyperion_firmware_version_show, NULL );
#endif

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_pci_device_id_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_pci_device_id_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    /* return the serial number of the hyperion device */
    struct hyperion_device* device;

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    device = ( struct hyperion_device* )dev_get_drvdata( class_dev );
#else
    device = ( struct hyperion_device* )class_get_devdata( class_dev );
#endif
    return snprintf( buf, PAGE_SIZE, "%d\n", device->vd_id.deviceId );
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( pci_device_id, S_IRUGO, hyperion_pci_device_id_show, NULL );
#else
CLASS_DEVICE_ATTR( pci_device_id, S_IRUGO, hyperion_pci_device_id_show, NULL );
#endif

//-------------------------------------------------------------------------------------------
static void do_create_class_files( struct hyperion_device* device, dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    struct device* hyperion_class_member;
#else
    struct class_device* hyperion_class_member;
#endif
    int err = 0;

    hyperion_class_member = hyperion_class_device_create( ( void* )device, dev_num );
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    if( ( err = device_create_file( hyperion_class_member, &dev_attr_serialno ) ) < 0 )
#else
    if( ( err = class_device_create_file( hyperion_class_member, &class_device_attr_serialno ) ) < 0 )
#endif
    {
        printk( PKET  "Error %d creating hyperion device%d class file: class_device_attr_number_devices", err, device->index );
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    if( ( err = device_create_file( hyperion_class_member, &dev_attr_board_type ) ) < 0 )
#else
    if( ( err = class_device_create_file( hyperion_class_member, &class_device_attr_board_type ) ) < 0 )
#endif
    {
        printk( PKET  "Error %d creating hyperion device%d class file: class_device_attr_number_devices", err, device->index );
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    if( ( err = device_create_file( hyperion_class_member, &dev_attr_firmware_version ) ) < 0 )
#else
    if( ( err = class_device_create_file( hyperion_class_member, &class_device_attr_firmware_version ) ) < 0 )
#endif
    {
        printk( PKET  "Error %d creating hyperion device%d class file: class_device_attr_firmware_version", err, device->index );
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    if( ( err = device_create_file( hyperion_class_member, &dev_attr_pci_device_id ) ) < 0 )
#else
    if( ( err = class_device_create_file( hyperion_class_member, &class_device_attr_pci_device_id ) ) < 0 )
#endif
    {
        printk( PKET  "Error %d creating hyperion device%d class file: class_device_attr_pci_device_id", err, device->index );
    }
    /* add any new class files here */
}

//-------------------------------------------------------------------------------------------
static void do_create_class_files_serial( struct hyperion_device* device, dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    struct device* class_member_serial;
#else
    struct class_device* class_member_serial;
#endif
    class_member_serial = hyperion_serial_class_device_create( ( void* )device, dev_num );
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 3, 4, 0 )
static int hyperion_init_one( struct pci_dev* pdev, const struct pci_device_id* ent )
#else
static int __devinit hyperion_init_one( struct pci_dev* pdev, const struct pci_device_id* ent )
#endif
//-------------------------------------------------------------------------------------------
{
    int result, i, err;
    int device_index;
    dev_t dev_num_serial[MAX_PARALLEL_TRANSFER] = {( dev_t )0, ( dev_t )0}, dev_num = ( dev_t )0;
    struct hyperion_device* device;

    device_index = hyperion_get_device_number();
    if( device_index >= MAXHYPERIONS )
    {
        return -EFAULT;
    }

    dev_num = MKDEV( major_dev_num, minor_dev_num + device_index );

    if( ( device = ( struct hyperion_device* ) kzalloc( sizeof( struct hyperion_device ), GFP_KERNEL ) ) == NULL )
    {
        return -ENOMEM;
    }

    device->pdev = pdev;
    cdev_init( &device->cdev, &hyperion_fops );
    device->cdev.owner = THIS_MODULE;
    device->cdev.ops = &hyperion_fops;
    err = cdev_add( &device->cdev, dev_num, 1 );
    /* Fail gracefully if need be */
    if( err )
    {
        printk( PKET  "Error %d adding hyperion device%d", err, device_index );
        goto err2_out;
    }

    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        // register serial communication
        dev_num_serial[i] = MKDEV( major_dev_num, minor_dev_num + device_index + ( ( i + 1 ) * MINOR_SERIAL_PORT ) );
        cdev_init( &device->cdev_serial[i], &hyperion_fops );
        device->cdev_serial[i].owner = THIS_MODULE;
        device->cdev_serial[i].ops = &hyperion_fops;
        err = cdev_add( &device->cdev_serial[i], dev_num_serial[i], 1 );
        if( err )
        {
            printk( PKET  "Error %d adding hyperion_serial port %d", err, i );
            goto err2_out;
        }
    }

    if( pci_enable_device( pdev ) )
    {
        printk( PKET "hyperion %d: Cannot enable device at 0x%02x:0x%02x!\n", device_index, device->bus, PCI_SLOT( pdev->devfn ) );
        goto err2_out;
    }

    hyperions[device_index] = device;
    device->index = device_index;
    device->bus = pdev->bus->number;
    device->devfunc = pdev->devfn;

    spin_lock_init( &device->s_lock );
    /* get IRQ from PCI device - this also works with APIC */
    device->irqlin = pdev->irq;

    space_get_memory_resource( pdev, device->s_lock, 0, &device->memory_base[0].physical_address, &device->memory_base[0].size );
    device->memory_base[0].base = space_map( device->memory_base[0].physical_address, device->memory_base[0].size );

#if MAP_MEMBASE_1
    space_get_memory_resource( pdev, device->s_lock, 1, &device->memory_base[1].physical_address, &device->memory_base[1].size );
    device->memory_base[1].base = space_map( device->memory_base[1].physical_address, device->memory_base[1].size );
#endif /* USE_MEMBASE_1 */

    PRINTKM( MOD, ( PKTD "hyperion resources got: register p%p addr 0x%lx size 0x%lx irq 0x%x\n",
                    device->index, device->memory_base[0].base, device->memory_base[0].physical_address, device->memory_base[0].size, device->irqlin ) );
#if MAP_MEMBASE_1
    PRINTKM( MOD, ( PKTD "hyperion resources got: ddrram p%p addr 0x%lx size 0x%lx\n",
                    device->memory_base[1].base, device->memory_base[1].physical_address, device->memory_base[1].size ) );
#endif /* USE_MEMBASE_1 */

    space_get_memory_resource( pdev, device->s_lock, 2, &device->memory_base[2].physical_address, &device->memory_base[2].size );
    device->memory_base[2].base = space_map( device->memory_base[2].physical_address, device->memory_base[2].size );
    PRINTKM( MOD, ( PKTD "hyperion resources got: register p%p addr 0x%lx size 0x%lx irq 0x%x\n",
                    device->index, device->memory_base[2].base, device->memory_base[2].physical_address, device->memory_base[2].size, device->irqlin ) );
    pci_set_master( pdev );
    pci_read_config_word( pdev, PCI_VENDOR_ID, &device->vd_id.vendorId );
    pci_read_config_word( pdev, PCI_DEVICE_ID, &device->vd_id.deviceId );
#if MAP_MEMBASE_1
    if( device->memory_base[1].base == NULL )
    {
        printk( PKET  "Error: memory_base[1].base == NULL!\n" );
        goto err_out;
    }
#endif /* USE_MEMBASE_1 */

    if( ( irq_map_hyperion[device->index] ) == 0 ) /* If we don't already have this IRQ for this board */
    {
        for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
        {
            device->pqueues[i] = &device->dq_read_write[i];
            device->pdma_object[i] = &device->dma_transfer_object[i];
            initialize_queue( device->pqueues[i], start_transfer, i );
            //serialOK = CreateSerialCommunication( &device->UartPort[i], (PUCHAR)device->Register.UARTReg[i].UARTRead, i );
            ///< todo check result queue and communication
        }
        device->pqueue_result = &device->dq_result;
        initialize_queue( device->pqueue_result, start_transfer_dummy, 0 );
        switch( device->vd_id.deviceId )
        {
        case PCI_DEVICE_ID_HYPERION_CL4E:
            device->hyperion_base.base = device->memory_base[2].base;
            device->hyperion_base.size = device->memory_base[2].size;
            device->reg_def = CL4eRegisterBase;
            break;
        default:
        case PCI_DEVICE_ID_HYPERION_CLE:
            device->hyperion_base.base = device->memory_base[0].base;
            device->hyperion_base.size = device->memory_base[0].size;
            device->reg_def = CLeRegisterBaseA32;
            break;
        }
        device->address_space_encoding = DMA_ADDRESS_SPACE_ENCODING_32BIT;
        if( !dma_set_mask( &device->pdev->dev, DMA_BIT_MASK( 32 ) ) )
        {
            dma_set_coherent_mask( &device->pdev->dev, DMA_BIT_MASK( 32 ) );
        }
        else
        {
            printk( KERN_WARNING " %s: No suitable DMA available.\n", __FUNCTION__ );
            result = -1000;
        }
        setup_read_write( device );

        PRINTKM( MOD, ( PKTB "setup_read_write() --> irqlin %d\n", device->irqlin ) );

#ifdef CONFIG_PCI_MSI
        if( pci_find_capability( device->pdev, PCI_CAP_ID_MSIX ) )
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 )
            result = pci_enable_msix_range( device->pdev, device->msi_x_entry, HYPERION_MSI_X_MAX_VECTORS, HYPERION_MSI_X_MAX_VECTORS );
#else
            result = pci_enable_msix( device->pdev, device->msi_x_entry, HYPERION_MSI_X_MAX_VECTORS );
#endif
        }
        else
        {
            PRINTKM( MOD, ( PKTB "device is not capable of msi-x\n" ) );
            result = -1;
        }

        if( result == 0 )
        {
            PRINTKM( MOD, ( PKTB "device is  is using msi-x\n" ) );
            device->flags |= HYPERION_FLAG_MSIX;
            //saa7160_drv->flags |= SAA7160_FLAG_MSIX;
            /* video interrupt */
            /*result = request_irq( saa7160_drv->msi_x_entry[SAA7160_MSI_X_VECTOR_VIDEO].vector, saa7160_video_isr,
                    IRQF_SHARED, saa7160_drv->name, (void *)saa7160_drv );
            if( result < 0 ) {
                dprintk( 0, "can't get device %d video IRQ %d\n", saa7160_num, saa7160_drv->msi_x_entry[SAA7160_MSI_X_VECTOR_VIDEO].vector );
                goto fail;
            } */
        }
        else
        {
            if( pci_find_capability( device->pdev, PCI_CAP_ID_MSI ) )
            {
                result = pci_enable_msi( device->pdev );
            }
            else
            {
                PRINTKM( MOD, ( PKTB "device is not capable of msi\n" ) );
                result = -1;
            }

            if( !result )
            {
                PRINTKM( MOD, ( PKTB "device is using msi\n" ) );
                device->flags |= HYPERION_FLAG_MSI;
            }
            else
            {
                struct pci_bus* bus;

                PRINTKM( MOD, ( PKTB "device is using int-x interrupts\n" ) );
                PRINTKM( MOD, ( PKTB "hyperion_drv->no_msi = %d\n", device->pdev->no_msi ) );
                for( bus = device->pdev->bus; bus; bus = bus->parent )
                    if( bus->bus_flags & PCI_BUS_FLAGS_NO_MSI )
                    {
                        PRINTKM( MOD, ( PKTB "pci_enable_msi: bus cannot handle MSI\n" ) );
                    }
            }
        }
#endif /* CONFIG_PCI_MSI */
#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,19)
        if( ( ( device->flags & HYPERION_FLAG_MSIX )  == 0 ) && !request_irq( device->pdev->irq, hyperion_interrupt, SA_SHIRQ, intname[device->index], device ) )
#else
        if( ( ( device->flags & HYPERION_FLAG_MSIX )  == 0 ) && !request_irq( device->pdev->irq, hyperion_interrupt, IRQF_SHARED, intname[device->index], device ) )
#endif
        {
            if( device->flags & HYPERION_FLAG_MSI )
            {
                IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhPCICore, OFF_PCI_EXPRESS_INTERRUPT_ENABLE, AVL_IRQ );
            }

            device->flags |= HYPERION_FLAG_IRQ;
            irq_map_hyperion[device->index] = IRQ_INITIALIZED | device->irqlin;
            PRINTKM( MOD, ( PKTB "%s() irq_map_hyperion 0x%x\n", __FUNCTION__, irq_map_hyperion[device->index] ) );
            device->hyperion_tasklet.next = NULL;
            device->hyperion_tasklet.state = 0;
            device->hyperion_tasklet.count.counter =  0;
            device->hyperion_tasklet.data = device->index;
            device->hyperion_tasklet.func = hyperion_do_tasklet;
        }
        else
        {
            printk( PKTD "request_irq ( %d ) * failed\n", device->index, device->irqlin );
            goto err_out;
        }

        /* This logic deliberately only makes one request_irq() call for
        * each physical IRQ needed on each board. There is no need for more.
        */
    }
    /* create class device */
    do_create_class_files( device, dev_num );
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        do_create_class_files_serial( device, dev_num_serial[i] );
    }
    device->flags |= HYPERION_FLAG_INITIALIZED ;
    init_completion( &device->compl_abort );
    /*  if( device->flags & HYPERION_FLAG_MSI )
        {
            if( device->memory_base[2].base )
            {
                *(unsigned long*)(device->memory_base[2].base + 0x490) = 0x10;
                *(unsigned long*)(device->memory_base[2].base + 0x690) = 0x10;
                printk (KERN_INFO "%s try interrupts int_stat 0x%x\n", __FUNCTION__, ioread32( (void __iomem *)device->hyperion_base.PCIExpressInterruptStatus ) );
                *(device->memory_base[2].base + 0x688) = 0x10;
                printk (KERN_INFO "%s try interrupts int_stat 0x%x\n", __FUNCTION__, ioread32( (void __iomem *)device->hyperion_base.PCIExpressInterruptStatus ) );
    //          wait_jiffies( 2 );
                *(device->memory_base[2].base + 0x488) = 0x10;
                printk (KERN_INFO "%s try interrupts int_stat 0x%x\n", __FUNCTION__, ioread32( (void __iomem *)device->hyperion_base.PCIExpressInterruptStatus ) );
            }
        }*/
    device->eeprom_write_access = -1;
    return 0;

err_out:
    pci_disable_device( pdev );
    space_unmap( ( void* )device->memory_base[0].base );
#if MAP_MEMBASE_1
    space_unmap( ( void* )device->memory_base[1].base );
#endif /* USE_MEMBASE_1 */

err2_out:
    release_mem_region( pci_resource_start( pdev, 0 ), pci_resource_len( pdev, 0 ) );
    kfree( device );
    hyperions[device_index] = NULL;
    printk( KERN_WARNING "mvHYPERION device %d *NOT* ok\n", device->index );
    return -ENODEV;
}

//-------------------------------------------------------------------------------------------
static int hyperion_remove_device( struct hyperion_device* device )
//-------------------------------------------------------------------------------------------
{
    struct pci_dev* device_found;
    void* ptr;
    int i;

#if LINUX_VERSION_CODE >= KERNEL_VERSION (2,6,26)
    device_found = pci_get_slot( device->pdev->bus, device->devfunc );
#else
    device_found = pci_find_slot( device->bus, device->devfunc );
#endif

    if( device_found == NULL )
    {
        PRINTKM( MOD, ( PKTD "hyperion_number = %d: no more matching mvHYPERIONS found\n", device->index, device->index ) );
        return NO_MORE_HYPERIONS;
    }
    release_read_write( device );
    PRINTKM( MOD, ( PKTD "hyperion_remove_device Device vendorId 0x%x found at bus=%d, devfun=0x%02x\n", device->index, PCI_VENDOR_ID_MATRIX_VISION, device->bus, device->devfunc ) );
    if( device->flags & HYPERION_FLAG_MSI )
    {
        IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhPCICore, OFF_PCI_EXPRESS_INTERRUPT_ENABLE, 0 );
    }
    pci_disable_device( device_found );
    /* remove class device */
    hyperion_class_device_destroy( MKDEV( major_dev_num, minor_dev_num + device->index ) );
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        hyperion_serial_class_device_destroy( MKDEV( major_dev_num, minor_dev_num + device->index + ( ( i + 1 )*MINOR_SERIAL_PORT ) ) );
    }
    if( irq_map_hyperion[device->index] )
    {
        irq_map_hyperion[device->index] = 0;
        free_irq( device->pdev->irq, device );
#ifdef CONFIG_PCI_MSI
        if( device->flags & HYPERION_FLAG_MSI )
        {
            pci_disable_msi( device->pdev );
            device->flags &= ~HYPERION_FLAG_MSI;
        }
        if( device->flags & HYPERION_FLAG_MSIX )
        {
            for( i = 0; i < HYPERION_MSI_X_MAX_VECTORS; i++ )
            {
                if( device->msi_x_entry[i].vector )
                {
                    free_irq( device->msi_x_entry[i].vector, device );
                }
            }
            pci_disable_msix( device->pdev );
            device->flags &= ~HYPERION_FLAG_MSIX;
        }
#endif /* CONFIG_PCI_MSI */
    }
    ptr = ( void* )device->memory_base[0].base;
    space_unmap( &ptr );
#if MAP_MEMBASE_1
    ptr = ( void* )device->memory_base[1].base;
    space_unmap( &ptr );
#endif
    ptr = ( void* )device->memory_base[2].base;
    space_unmap( &ptr );
    //release_mem_region( pci_resource_start(device->pdev, 0), pci_resource_len(device->pdev, 0) );
    return 0;
}


//-------------------------------------------------------------------------------------------
static int __init hyperion_init( void )
//-------------------------------------------------------------------------------------------
{
    int result = 0, device_number;
    dev_t dev_num;
    unsigned int firstminor = 0;

    printk( KERN_INFO "MATRIX Vision mvHYPERION - V%d.%d.%d (%lx)\n",
            HYPERION_MAJ_VERSION, HYPERION_MIN_VERSION, HYPERION_BUILD_NUMBER, debug );

    if( major_dev_num )
    {
        dev_num = MKDEV( major_dev_num, minor_dev_num );
        result = register_chrdev_region( dev_num, MAXHYPERIONS, "hyperion" );
    }
    else
    {
        result = alloc_chrdev_region( &dev_num, firstminor, MAX_CONT_DEVICES, "hyperion" );
        major_dev_num = MAJOR( dev_num );
        minor_dev_num = MINOR( dev_num );
    }

    if( result < 0 )
    {
        PRINTKM( MOD, ( PKTB "unable to get major %d for hyperion devices\n", major_dev_num ) );
        return result;
    }

    PRINTKM( MOD, ( PKTB "major for hyperion devices is %d\n", major_dev_num ) );

    for( device_number = 0; device_number < MAXHYPERIONS; device_number++ )
    {
        hyperions[device_number] = NULL;
    }

    if( hyperion_get_device_number() < MAXHYPERIONS )
    {
        init_pci();
    }
    PRINTKM( MOD, ( PKTB "hyperion_init() --> ok, %d Hyperions found.\n", hyperion_get_device_number() ) );
    return 0;
}

//-------------------------------------------------------------------------------------------
static void __exit hyperion_exit( void )
//-------------------------------------------------------------------------------------------
{
    int index;
    dev_t first = MKDEV( major_dev_num, 0 );

    PRINTKM( MOD, ( PKTB "hyperion_exit()\n" ) );
    unregister_chrdev_region( first, MAXHYPERIONS );

    for( index = 0; index < MAXHYPERIONS; index++ )
    {
        struct hyperion_device* device = hyperions[index];
        hyperions[index] = NULL;
        if( device )
        {
            hyperion_remove_device( device );
            kfree( device );
        }
    }
    pci_unregister_driver( &hyperion_pci_driver );
    PRINTKM( MOD, ( PKTB "hyperion_exit() -->\n\n" ) );
}


static struct pci_device_id hyperion_pci_tbl[] =
{
    { PCI_VENDOR_ID_MATRIX_VISION, PCI_DEVICE_ID_HYPERION_CLE, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { 0, }
};

MODULE_DEVICE_TABLE( pci, hyperion_pci_tbl );

int __init init_pci( void )
{
    memset( &hyperion_pci_driver, 0, sizeof( hyperion_pci_driver ) );
    hyperion_pci_driver.name = MODULE_NAME;
    hyperion_pci_driver.id_table = hyperion_pci_tbl;
    hyperion_pci_driver.probe = hyperion_init_one;

    return pci_register_driver( &hyperion_pci_driver );
}

module_init( hyperion_init );
module_exit( hyperion_exit );
