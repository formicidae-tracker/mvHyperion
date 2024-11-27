/*
 * Tools for MATRIX Vision MatrixFg drivers
 *
 *   Copyright (c) 2006 Matrix Vision GmbH (info@matrix-vision.de)
 *
 $Id: matrix_tools.c,v 1.2 2007-02-21 12:47:34 hg Exp $
 *
 */
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <linux/version.h>
#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a) << 16) | ((b) << 8) | (c))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION (2,5,0)
// Use versioning if needed
#if defined(CONFIG_MODVERSIONS) && ! defined(MODVERSIONS)
#   define MODVERSIONS
#endif
#endif

#include <asm/io.h>
#include "matrix_types.h"
#include "matrix_tools.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 11, 0 )
#include <linux/uaccess.h>
#else
#include <asm/uaccess.h>
#endif
#include <linux/jiffies.h>
#include <linux/sched.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 11, 0 )
#include <linux/wait.h>
#endif
#include "kcompat.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
#   define MV_ACCESS_OK(type, ptr, size) access_ok(ptr, size)
#else
#   define MV_ACCESS_OK(type, ptr, size) access_ok(type, ptr, size)
#endif

#define MODULE_NAME "mv-tools"

//------------------------------------------------------------- read_user ------
int read_user( TUserVirtualAddress uva, void* dest, u_int size )
{
#ifndef USE_ACCESS_OK
    int err;
#endif /* USE_ACCESS_OK */
    if( uva.uptr == 0 || size == 0 )
    {
        return -EINVAL;
    }
#ifdef USE_ACCESS_OK
    if( !MV_ACCESS_OK( VERIFY_READ, ( void* )uva.uptr, size ) )
    {
        return -EFAULT;
    }
#else
    if( ( err = verify_area( VERIFY_READ, ( void* )uva.uptr, size ) ) )
    {
        return err;
    }
#endif

    switch( size )
    {
    case sizeof( u_char ):
        return get_user( *( u_char* )dest, ( u_char* )uva.uptr );
    case sizeof( u_short ):
        return get_user( *( u_short* )dest, ( u_short* )uva.uptr );
    case sizeof( u_long ):
        return get_user( *( u_long* )dest, ( u_long* )uva.uptr );
    default:
        return copy_from_user( dest, ( void* )uva.uptr, size );
    }
}

//------------------------------------------------------ write_user_value ------
int write_user_value( const u_long value, TUserVirtualAddress uva, u_int size )
{
#ifndef USE_ACCESS_OK
    int err;
#endif  /* USE_ACCESS_OK */

    if( uva.uptr == 0 )
    {
        return -EINVAL;
    }
#ifdef USE_ACCESS_OK
    if( !MV_ACCESS_OK( VERIFY_WRITE, ( void* )uva.uptr, size ) )
    {
        return -EFAULT;
    }
#else
    if( ( err = verify_area( VERIFY_WRITE, ( void* )uva.uptr, size ) ) )
    {
        return err;
    }
#endif

    switch( size )
    {
    case 1:
        put_user( ( u_char ) value, ( u_char* ) uva.uptr );
        break;
    case 2:
        put_user( ( u_short ) value, ( u_short* ) uva.uptr );
        break;
    case 4:
        put_user( ( u_long ) value, ( u_long* ) uva.uptr );
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

//----------------------------------------------------------- write_user_buffer ------
int write_user_buffer( const void* ptr, TUserVirtualAddress uva, u_int size )
{
#ifndef USE_ACCESS_OK
    int err;
#endif  /* USE_ACCESS_OK */

    if( uva.uptr == 0 )
    {
        return -EINVAL;
    }
#ifdef USE_ACCESS_OK
    if( !MV_ACCESS_OK( VERIFY_WRITE, ( void* )uva.uptr, size ) )
    {
        return -EFAULT;
    }
#else
    if( ( err = verify_area( VERIFY_WRITE, ( void* )uva.uptr, size ) ) )
    {
        return err;
    }
#endif

    if( copy_to_user( ( void* )uva.uptr, ptr, size ) )
    {
        return -EFAULT;
    }

    return 0;
}

/*
 * Delay for WaitJiffies via timeout and schedule().
 * We need to be technically interruptable for this to work,
 * but we really don't want to be, so we block everything.
 * Don't call from interrupt.
 * TODO kernel 2.6.x consider using jiffies_64 ??
 */
//-----------------------------------------------------------------------------
void
wait_jiffies( u_long waitJiffies )
//-----------------------------------------------------------------------------
{
    u_long endJiffies = jiffies + waitJiffies;
    do
    {
        schedule();
    } while( time_before( jiffies, endJiffies ) );
}

//-----------------------------------------------------------------------------
void sleep_msec( unsigned int milliseconds )
//-----------------------------------------------------------------------------
{
    int delay_j = msecs_to_jiffies( milliseconds );
    int condition = 1;
    wait_queue_head_t waitQueue;
    INIT_WAIT_QUEUE_HEAD( waitQueue ) ;
    wait_event_interruptible_timeout( waitQueue, ( condition == 0 ), delay_j );
}
