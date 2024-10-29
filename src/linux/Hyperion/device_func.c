/*
 * device_func.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: device_func.c,v 1.29 2008-03-31 15:09:16 ug Exp $
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
#include "device_func.h"
#include "HyperionProp.h"
#include "property.h"

#define DEBUG_PRINT 0
#define CPY_MUX_DATA32(pram,data,data_cnt)\
    {\
        u32 *src, *dst;\
        int i;\
        src = (u32*)data;\
        dst = (u32*)pram;\
        for( i = 0; i < data_cnt; i++ )\
        {\
            iowrite32( *src, (void __iomem *)dst );\
            ++src;\
            ++dst;\
        }\
    }
#define CPY_MUX_DATA_DST64(pram,data,data_cnt)\
    {\
        u32 *src, *dst;\
        int i;\
        src = (u32*)data;\
        dst = (u32*)pram;\
        for( i = 0; i < data_cnt; i++ )\
        {\
            iowrite32( *src, (void __iomem *)dst );\
            ++src;\
            ++dst;\
            ++dst;\
        }\
    }
//-------------------------------------------------------------------------------------------
void set_mux_data( struct hyperion_device* device, struct mux_controller_sequence* mux_seq, u_long inputchannel )
//-------------------------------------------------------------------------------------------
{
    u32* muxdata;
#if DEBUG_PRINT
    int i;
#endif
    PRINTKM( MOD, ( PKTD "%s muxdata changed %s size %d\n", device->index, __FUNCTION__, mux_seq->changed ? "true" : "false", mux_seq->size ) );
    if( mux_seq->changed )
    {
        mux_seq->changed = FALSE;
        PRINTKM( DMA, ( PKTD "mux_ram\n",  0 ) );
        if( mux_seq->size < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES )
        {
            muxdata = ( u32* )mux_seq->muxdata;
            CPY_MUX_DATA32( REG_POINTER( device->hyperion_base, device->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER ), mux_seq->muxdata, mux_seq->size / sizeof( u32 ) );
#if DEBUG_PRINT
            muxdata = ( u32* )REG_POINTER( device->hyperion_base, device->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " %p %x\n",  0, &muxdata[i], muxdata[i] ) );
            }
#endif
        }
        else
        {
            CPY_MUX_DATA32( REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_MUX_RAM_REGISTER ), mux_seq->muxdata, MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) ) );
            CPY_MUX_DATA32( REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_MUX_RAM_REGISTER ), &mux_seq->muxdata[MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) )], MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) ) );
#if DEBUG_PRINT
            muxdata = ( u32* )REG_POINTER( device->hyperion_base, device->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " 0x%x\n",  0, muxdata[i] ) );
            }
            muxdata = ( u32* )REG_POINTER( device->hyperion_base, device->reg_def, inputchannel + ebrhCLController1, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " 0x%x\n",  0, muxdata[i] ) );
            }
#endif
        }
        wmb();
    }
}

//-------------------------------------------------------------------------------------------
void set_mux_data_64( struct hyperion_device* device, struct mux_controller_sequence* mux_seq, u_long inputchannel )
//-------------------------------------------------------------------------------------------
{
    u32* muxdata;
#if DEBUG_PRINT
    int i;
#endif
    PRINTKM( MOD, ( PKTD "%s muxdata changed %s size %d\n", device->index, __FUNCTION__, mux_seq->changed ? "true" : "false", mux_seq->size ) );
    if( mux_seq->changed )
    {
        mux_seq->changed = FALSE;
        PRINTKM( DMA, ( PKTD "mux_ram\n",  0 ) );
        if( mux_seq->size < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES )
        {
            muxdata = ( u32* )mux_seq->muxdata;
            CPY_MUX_DATA_DST64( REG_POINTER( device->hyperion_base, device->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER ), mux_seq->muxdata, mux_seq->size / sizeof( u32 ) );
#if DEBUG_PRINT
            muxdata = ( u32* )REG_POINTER( device->hyperion_base, device->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " %p %x\n",  0, &muxdata[i], muxdata[i] ) );
            }
#endif
        }
        else
        {
            CPY_MUX_DATA_DST64( REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, OFF_MUX_RAM_REGISTER ), mux_seq->muxdata, MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) ) );
            CPY_MUX_DATA_DST64( REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController1, OFF_MUX_RAM_REGISTER ), &mux_seq->muxdata[MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) )], MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) ) );
#if DEBUG_PRINT
            muxdata = ( u32* )REG_POINTER( device->hyperion_base, device->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " 0x%x\n",  0, muxdata[i] ) );
            }
            muxdata = ( u32* )REG_POINTER( device->hyperion_base, device->reg_def, inputchannel + ebrhCLController1, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " 0x%x\n",  0, muxdata[i] ) );
            }
#endif
        }
        wmb();
    }
}

//-------------------------------------------------------------------------------------------
void initialize_lut( struct hyperion_device* device, u_long lut_offset )
//-------------------------------------------------------------------------------------------
{
    u_long i;
    u_char* pLUT = vmalloc( LUT_SIZE );
    u_char* lutbase = REG_POINTER( device->hyperion_base, device->reg_def, ebrhCLController0, lut_offset );

    if( pLUT != NULL )
    {
        for( i = 0; i < LUT_SIZE; i++ )
        {
            pLUT[i] = ( u_char )( i & 0xff );
        }
        memcpy( ( void* )lutbase, ( void* )pLUT, LUT_SIZE );
        vfree( pLUT );
    }
}
