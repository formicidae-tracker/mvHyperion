#ifndef __SPI_ACCESS_H__
#define __SPI_ACCESS_H__ __SPI_ACCESS_H__

/******************************************************************************
*                                                                             *
* License Agreement                                                           *
*                                                                             *
* Copyright (c) 2003 Altera Corporation, San Jose, California, USA.           *
* All rights reserved.                                                        *
*                                                                             *
* Permission is hereby granted, free of charge, to any person obtaining a     *
* copy of this software and associated documentation files (the "Software"),  *
* to deal in the Software without restriction, including without limitation   *
* the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
* and/or sell copies of the Software, and to permit persons to whom the       *
* Software is furnished to do so, subject to the following conditions:        *
*                                                                             *
* The above copyright notice and this permission notice shall be included in  *
* all copies or substantial portions of the Software.                         *
*                                                                             *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
* DEALINGS IN THE SOFTWARE.                                                   *
*                                                                             *
* This agreement shall be governed in all respects by the laws of the State   *
* of California and by the laws of the United States of America.              *
*                                                                             *
******************************************************************************/
#ifdef DRIVER
#   define alt_8 char
#   define alt_u8 UCHAR
#   define alt_16 short
#   define alt_u16 USHORT
#   define alt_u32 ULONG
#   define alt_32 int
#else
#   if defined(linux) || defined(__linux) || defined(__linux__)
#       define alt_8 char
#       define alt_u8 unsigned char
#       define alt_16 short
#       define alt_u16 unsigned short
#       define alt_u32 unsigned long
#       define alt_32 int
#   else
#       include <stddef.h>
#       include "alt_types.h"
#   endif // #if defined(linux) || defined(__linux) || defined(__linux__)
#endif // DRIVER

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/*
 * Macros used by alt_sys_init
 */

#define ALTERA_AVALON_SPI_INSTANCE(name, device) extern int alt_no_storage
#define ALTERA_AVALON_SPI_INIT(name, device) while (0)

/*
 * Use this function to perform one SPI access on your target.  'base' should
 * be the base address of your SPI peripheral, while 'slave' indicates which
 * bit in the slave select register should be set.
 */

/* If you need to make multiple accesses to the same slave then you should
 * set the merge bit in the flags for all of them except the first.
 */
#define ALT_AVALON_SPI_COMMAND_MERGE (0x01)

/*
 * If you need the slave select line to be toggled between words then you
 * should set the toggle bit in the flag.
 */
#define ALT_AVALON_SPI_COMMAND_TOGGLE_SS_N (0x02)


int spi_command( alt_u8* base, alt_u32 slave,
                 alt_u32 write_length, const alt_u8* write_data,
                 alt_u32 read_length, alt_u8* read_data,
                 alt_u32 flags );

int spi_command16( alt_u8* base, alt_u32 slave,
                   alt_u32 write_length, const alt_u16* write_data,
                   alt_u32 read_length, alt_u16* read_data,
                   alt_u32 flags );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __SPI_ACCESS_H__ */
