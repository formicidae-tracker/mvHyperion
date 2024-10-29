/*
 * iocontrol.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: iocontrol.c,v 1.7 2011-03-29 12:46:28 ug Exp $
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

#include "drivermain.h"
#include "hyperion_base.h"
#include "clf_func.h"
#include "spi_access.h"

//-------------------------------------------------------------------------------------------
struct mv_ioctl
//-------------------------------------------------------------------------------------------
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
int hyperion_ioctl( struct inode* inode, struct file* file, unsigned int cmd, unsigned long arg )
#endif
//-------------------------------------------------------------------------------------------
{
    struct hyperion* phyperion = ( struct hyperion* )file->private_data;
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    int error = 0;
    TUserVirtualAddress uva ;
    u_short dir = _IOC_DIR( cmd );
    u_short size = _IOC_SIZE( cmd );
    struct mv_ioctl mvio;
    char* io_buffer;
    unsigned long io_buffer_size;
    unsigned char write_back_to_user = FALSE;
    uva.uptr = arg;
    PRINTKM( IOCTL, ( PKTD "ioctl cmd=0x%08x, dir=%d, size=%d, arg=0x%08lx\n", phyperion->number, cmd, dir, size, arg ) );

    read_user( uva, &mvio, sizeof( mvio ) );
    io_buffer_size = max( mvio.in_size, mvio.out_size ) + sizeof( mvio );
    io_buffer = kmalloc( io_buffer_size, GFP_KERNEL );
    if( io_buffer == NULL )
    {
        PRINTKM( IOCTL, ( PKTD "ioctl cmd 0x%08x can't allocate io_buffer\n", phyperion->number, cmd ) );
        return -EINVAL;
    }
    read_user( uva, io_buffer, io_buffer_size );
    PRINTKM( IOCTL, ( PKTD "ioctl cmd 0x%08x  insize %d osize %d\n", phyperion->number, cmd, mvio.in_size, mvio.out_size ) );

    if( phyp_dev->user_flash_enabled == FALSE )
    {
        switch( cmd )
        {
        case IOCTL_READ_ASMI_U32:
        case IOCTL_WRITE_ASMI_U32:
            //switch to IOCTL code defined in CommonIoCtl.h
            break;
        default:
            cmd = -1; //remove this IOCTL with status = STATUS_INVALID_DEVICE_REQUEST
            break;
        }
    }

    switch( cmd )
    {
#include "serialport_ioctl.h"
#include "clf_ioctl.h"
#include "common_ioctl.h"
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

