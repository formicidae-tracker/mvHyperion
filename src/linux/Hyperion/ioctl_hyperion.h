/*
 * ioctl constants for MATRIX Vision mvHYPERION driver
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: ioctl_hyperion.h,v 1.1 2006-08-07 14:55:32 hg Exp $
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

#ifndef _IOCTL_HYPERION_H
#define _IOCTL_HYPERION_H

#include <linux/ioctl.h>

#ifdef __cplusplus
extern "C" {
#endif

//---- ioctl commands  ---------------------------------------------------------

#define HYPERION_IOCTL_MAGIC    'h'

#define _IOt(n)             _IO(HYPERION_IOCTL_MAGIC,n)
#define _IORt(n,t)          _IOR(HYPERION_IOCTL_MAGIC,n,t)
#define _IOWt(n,t)          _IOW(HYPERION_IOCTL_MAGIC,n,t)


//#define HYPERION_HW_INFO          _IORt(  1, ???)
//#define HYPERION_VERSION          _IORt(  2, unsigned long)

#ifdef __cplusplus
}
#endif

#endif /* ifndef _IOCTL_HYPERION_H */

