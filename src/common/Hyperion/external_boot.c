/****************************************************************************
* Copyright � 2006 Altera Corporation, San Jose, California, USA.           *
* All rights reserved. All use of this software and documentation is        *
* subject to the License Agreement located at the end of this file below.   *
****************************************************************************/
/*****************************************************************************
*  File: external_boot.c
*
*  Purpose: This is an example of some code that could run on an external
*  processor to control the booting of a Nios II processor.  Using a
*  PIO and the cpu_resetrequest signal, this routine holds the main Nios II
*  processor in reset while it copies an application from a boot record
*  in flash to RAM.  It then calls release_main_cpu(), which calculates the
*  entry point of the application, constructs a Nios II instruction which
*  branches to that entry point, then writes the instruction to the Nios II
*  reset address and releases the Nios II from reset.
*
*  This example code is of course predicated on the fact that the main Nios II
*  CPU's reset address is set to some volatile memory (RAM).  Otherwise, we
*  cant write an instruction to the reset address.
*
*****************************************************************************/
#ifdef DRIVER
#   include "stddcls.h"
#   include "driver.h"
//#   include "DMACtrl.tmh"
#else //linux
#   include <linux/types.h>
#   include <linux/pci.h>
#   include <linux/jiffies.h>
#   include "hyperion_dma_nios.h"
#   include "matrix_tools.h"
#endif
#include "external_boot.h"

#undef WRITE_REG32
#undef READ_REG32

#if defined(linux) || defined(__linux) || defined(__linux__)
#   define WRITE_REG32(reg,val) iowrite32(val,(void __iomem *)(reg))
#   define READ_REG32(reg) ioread32((void __iomem *)(reg))
#   define dbg_print(msg)
#   define delay_msec(dl) wait_jiffies(msecs_to_jiffies(dl))
#else
#   define WRITE_REG32(reg,val) WRITE_REGISTER_ULONG((PULONG)reg,val)
#   define READ_REG32(reg) READ_REGISTER_ULONG((PULONG)reg)
#   define dbg_print(msg)  DbgPrint msg
#   define delay_msec(dl) \
    { \
        LARGE_INTEGER delay; \
        delay.QuadPart = -10000*dl; \
        KeDelayExecutionThread( KernelMode, FALSE, &delay ); \
    }
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
/*
* Here we define what the reset values are for our PIO connected
* to the main CPU's cpu_resetrequest line.
*/
#define RESET_ACTIVE 0x1
#define RESET_INACTIVE 0x0

/*
* This is the reset address of our main CPU for which we're
* loading application code.  There's not really any place handy
* that we can obtain this value automatically, so you'll have to
* plug it in here by hand.
*/
#define MAIN_CPU_RESET_ADDR ONCHIP_MEM_BASE
#define MAIN_CPU_RESET_PIO_BASE 0x02200000


/*****************************************************************************
*  Function: alt_main
*
*  Purpose: This routine loads an application to RAM from flash, then calls
*  release_main_cpu() to handoff execution to start the main Nios II CPU.
*
*****************************************************************************/
//-------------------------------------------------------------------------------------------
void hold_nios_in_reset( unsigned char* base )
//-------------------------------------------------------------------------------------------
{
    WRITE_REG32( ( base + MAIN_CPU_RESET_PIO_BASE ), RESET_INACTIVE );
    WRITE_REG32( ( base + MAIN_CPU_RESET_PIO_BASE + 3 ), 0 ); // capture register is at offset 3.
    // Now we try to reset the main CPU by setting the cpu_resetrequest pin high.
    WRITE_REG32( ( base + MAIN_CPU_RESET_PIO_BASE ), RESET_ACTIVE );
}

//-------------------------------------------------------------------------------------------
int start_nios_cpu( unsigned char* base, void* flash_image, int size )
//-------------------------------------------------------------------------------------------
{

    unsigned int entry_point;
    int l = 0, temp;

    // Before getting started, we need to place and hold the main Nios II CPU in reset
    // But first we need to clear the edge capture register in the PIO, so we
    // can see when the CPU actually achieves reset (using the cpu_resettaken pin)
    WRITE_REG32( ( base + MAIN_CPU_RESET_PIO_BASE ), RESET_INACTIVE );
    WRITE_REG32( ( base + MAIN_CPU_RESET_PIO_BASE + ( 2 * 3 ) ), 0 ); // capture register is at offset 3.
    // Now we try to reset the main CPU by setting the cpu_resetrequest pin high.

    WRITE_REG32( ( base + MAIN_CPU_RESET_PIO_BASE ), RESET_ACTIVE );
    // Here we wait for the edge capture register of the PIO to show us that
    // the cpu_resettaken pin toggled high.

    //temp = READ_REG32( (base + MAIN_CPU_RESET_PIO_BASE) );
    do
    {
        temp = READ_REG32( ( base + MAIN_CPU_RESET_PIO_BASE ) );
        ++l;
    }
    while( temp == 0x0 && l < 1000 );

    //printk(" %s waitfor reset loop_count %d\n", __FUNCTION__, l );
    // Now load the application from the boot record in flash.
    if( ( entry_point = load_flash_image( ( unsigned char* )base, flash_image, size ) ) ) // load the image
    {
        // Release the main CPU from reset now that it has code to run.
        release_main_cpu( base, entry_point );
        delay_msec( 20 );
        return 1;
    }
    else
    {
        return 0;
    }
}

/*****************************************************************************
*  Function: release_main_cpu
*
*  Purpose: This is where the interesting stuff happens.  At this point we've
*  copied an application to the main Nios II program memory and we're holding
*  the CPU in reset.  We want to release it from reset so it can run the
*  application, only the application's entry point may not be at the CPU's
*  reset address.  Luckily we have the entry point from the boot record,
*  passed in here as the function pointer "entry_point".  We compare the
*  entry point to the CPU's reset address and calculate how far the CPU has
*  to branch to get there.  From that information, we construct a Nios II
*  instruction that jumps the appropriate distance, then stuff it into memory
*  at the main CPU's reset address.  We release the CPU from reset, and if all
*  goes well, it executes the branch instruction we put at its reset address
*  and branches to the application's entry point, successfully launching the
*  application.
*
*****************************************************************************/
void release_main_cpu( unsigned char* base, unsigned int entry_point )
{
    int offset;
    unsigned int branch_instruction;

    // Calculate the offset the main cpu needs to jump from its reset address
    offset = ( int )entry_point - MAIN_CPU_RESET_ADDR;

    // We only need to formulate a branch instruction if the reset address
    // does not already point to the entry point of the application. If the
    // reset address already points to the application entry point, we can just
    // release the main Nios II CPU from reset and everything should be happy.
    if( offset )
    {
        // Now construct the appropriate branch instruction "br" we need to stuff
        // into the main cpu reset address.  The relative offset we use for the
        // instruction must be 4 bytes less than the actual entry point because
        // that is how Nios II defines the "br" instruction.

        // Branch instruction encoding
        //  31  29  27  25  23  21  19  17  15  13  11  09  07  05  03  01
        //    30  28  26  24  22  20  18  16  14  12  10  08  06  04  02  00
        //  ----------------------------------------------------------------
        // |    0    |    0    |    16-bit relative jump -4    |   0x06     |
        //  ----------------------------------------------------------------
        branch_instruction = ( ( offset - 4 ) << 6 ) | 0x6;


        // Write the instruction to the main CPU reset address
        dbg_print( ( " %s branch_instruction %x to cpu_reset %p\n", __FUNCTION__, branch_instruction, ( base + MAIN_CPU_RESET_ADDR ) ) );
        WRITE_REG32( ( base + MAIN_CPU_RESET_ADDR ), branch_instruction );
    }
    // Now we can release the main CPU from reset
    //printk(" %s now switch reset to inactive\n", __FUNCTION__ );
    WRITE_REG32( ( base + MAIN_CPU_RESET_PIO_BASE ), RESET_INACTIVE );
    READ_REG32( ( base + MAIN_CPU_RESET_PIO_BASE ) );
#if defined(linux) || defined(__linux) || defined(__linux__)
    wmb();
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

    // We're done.
    return;

}


/*****************************************************************************
*  Function: copy_from_flash
*
*  Purpose:  This subroutine copies data from a flash memory to a buffer
*  The function uses the appropriate copy routine for the flash that is
*  defined by FLASH_TYPE.  EPCS devices cant simply be read from using
*  memcpy().
*
*****************************************************************************/
void* copy_from_flash( void* dest, const void* src, size_t num )
{
#ifdef DRIVER
    RtlCopyMemory( dest, src, num );
#else
    memcpy( dest, src, num );
#endif
    return ( dest );
}

/*****************************************************************************
*  Function: load_flash_image
*
*  Purpose:  This subroutine loads an image from flash into the Nios II
*  memory map.  It decodes boot records in the format produced from the
*  elf2flash utility, and loads the image as directed by those records.
*  The format of the boot record is described in the text of the application
*  note.
*
*  The input operand, "image" is expected to be the image selector indicating
*  which flash image, 1 or 2, should be loaded.
*
*****************************************************************************/
unsigned int load_flash_image( unsigned char* instr_buffer, void* flash_image_ptr, int size )
{
    unsigned char* next_flash_byte;
    unsigned int length;
    unsigned int address;

    dbg_print( ( " %s instr_buf %p, flash_image %p, size %x", __FUNCTION__, instr_buffer, flash_image_ptr, size ) );
    next_flash_byte = ( unsigned char* )flash_image_ptr + 32;
    /*
    * Flash images are not guaranteed to be word-aligned within the flash
    * memory, so a word-by-word copy loop should not be used.
    *
    * The "memcpy()" function works well to copy non-word-aligned data, and
    * it is relativly small, so that's what we'll use.
    */

    // Get the first 4 bytes of the boot record, which should be a length record
    /*{
        int i;
        unsigned int* msg = (unsigned int*)flash_image_ptr;
        dbg_print((  ": - %s - flash_image\n", __FUNCTION__ ));
        for( i = 0; i < 64; i += 4 )
            dbg_print((  " d %8p: 0x%8x 0x%8x 0x%8x 0x%8x\n", &msg[i], (unsigned int)msg[i], (unsigned int)msg[i+1], (unsigned int)msg[i+2], (unsigned int)msg[i+3] ));
    }*/

    copy_from_flash( ( void* )( &length ), ( void* )( next_flash_byte ), ( size_t )( 4 ) );
#if defined(linux) || defined(__linux) || defined(__linux__)
    length = le32_to_cpu( length );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
    next_flash_byte += 4;
    dbg_print( ( ": - %s - length 0x%x\n", __FUNCTION__, length ) );

    // Now loop until we get an entry record, or a halt recotd
    while( ( length != 0 ) && ( length != 0xffffffff ) )
    {
        // Get the next 4 bytes of the boot record, which should be an address
        // record
        copy_from_flash( ( void* )( &address ), ( void* )( next_flash_byte ), ( size_t )( 4 ) );
#if defined(linux) || defined(__linux) || defined(__linux__)
        address = le32_to_cpu( address );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
        next_flash_byte += 4;

        // Copy the next "length" bytes to "address"
        dbg_print( ( " %s cpy( dst %p, src %p, len %x\n", __FUNCTION__, ( void* )( instr_buffer + address ), ( void* )( next_flash_byte ), ( unsigned int )( length ) ) );
        copy_from_flash( ( void* )( instr_buffer + address ), ( void* )( next_flash_byte ), ( size_t )( length ) );
        next_flash_byte += length;

        // Get the next 4 bytes of the boot record, which now should be another
        // length record
        copy_from_flash( ( void* )( &length ), ( void* )( next_flash_byte ), ( size_t )( 4 ) );
#if defined(linux) || defined(__linux) || defined(__linux__)
        length = le32_to_cpu( length );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
        next_flash_byte += 4;
    }
    // "length" was read as either 0x0 or 0xffffffff, which means we are done
    // copying.
    if( length == 0xffffffff )
    {
        // We read a HALT record, so return a 0
        return 0;
    }
    else // length == 0x0
    {
        // We got an entry record, so read the next 4 bytes for the entry address
        copy_from_flash( ( void* )( &address ), ( void* )( next_flash_byte ), ( size_t )( 4 ) );
#if defined(linux) || defined(__linux) || defined(__linux__)
        address = le32_to_cpu( address );
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
        next_flash_byte += 4;

        // Return the entry point address
        dbg_print( ( " %s entry point %x\n", __FUNCTION__, ( unsigned int )address ) );
        return address;
    }
    return 0;
}

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
