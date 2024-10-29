/*
 * Tools for MATRIX Vision Matrix Framegrabber drivers
 *
 *   Copyright (c) 2006 Matrix Vision GmbH (info@matrix-vision.de)
 *
 $Id: matrix_tools.h,v 1.2 2007-10-30 15:52:14 ug Exp $
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

#ifndef _MATRIX_TOOLS_H
#define _MATRIX_TOOLS_H

#include "matrix_types.h"

#ifdef __cplusplus
extern "C" {
#endif

//======= tools ===============================================================
#define arraysize(p) (sizeof(p)/sizeof((p)[0]))

int  read_user( TUserVirtualAddress uva, void* dest, u_int size );
int  write_user_value( const u_long value, TUserVirtualAddress uva, u_int size );
int  write_user_buffer( const void* ptr, TUserVirtualAddress uva, u_int size );
void wait_jiffies( u_long waitJiffies );
void sleep_msec( unsigned int milliseconds );

#ifndef HAVE_MSECS_TO_JIFFIES_EXTRA
#define msecs_to_jiffies(a) \
    (((a) > 0) ? ((((a) * HZ)/1000)+1) : 0)
#endif

#define _varclr(var)    memset(&var,0,sizeof(var))

#ifdef __cplusplus
}
#endif

#endif /* ifndef _MATRIX_TOOLS_H */

