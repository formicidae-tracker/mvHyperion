#ifndef ExternalBootH
#define ExternalBootH ExternalBootH

/****************************************************************************
 * Copyright 2006 Altera Corporation, San Jose, California, USA.           *
 * All rights reserved. All use of this software and documentation is        *
 * subject to the License Agreement located at the end of this file below.   *
 ****************************************************************************/

/*
 * Size of buffer for processing flash contents
 */
#define FLASH_BUFFER_LENGTH 256


/*
 * The boot images stored in flash memory, have a specific header
 * attached to them.  This is the structure of that header.  The
 * perl script "make_header.pl", included with this example is
 *  used to add this header to your boot image.
 */
typedef struct
{
    unsigned long signature;
    unsigned long version;
    unsigned long timestamp;
    unsigned long data_length;
    unsigned long data_crc;
    unsigned long res1;
    unsigned long res2;
    unsigned long header_crc;
} my_flash_header_type;


void hold_nios_in_reset( unsigned char* base );
void release_main_cpu( unsigned char* base, unsigned int entry_point );
void* copy_from_flash( void* dest, const void* src, size_t num );
unsigned int load_flash_image( unsigned char* instr_buffer, void* flash_image_ptr, int size );
int start_nios_cpu( unsigned char* base, void* flash_image, int size );

/******************************************************************************
 *                                                                             *
 * License Agreement                                                           *
 *                                                                             *
 * Copyright (c) 2006 Altera Corporation, San Jose, California, USA.           *
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
 * Altera does not recommend, suggest or require that this reference design    *
 * file be used in conjunction or combination with any other product.          *
 ******************************************************************************/

#endif //ExternalBootH
