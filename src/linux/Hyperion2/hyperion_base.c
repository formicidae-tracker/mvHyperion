/*
 * hyperion.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: hyperion_base.c,v 1.21 2011-01-19 16:15:59 ug Exp $
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

#include "hyperion_base.h"
#include "clf_func.h"
#include "drivermain.h"

#include "HyperionProp.h"
#include "property.h"

#define POST_REQUEST_REMOVAL 0
#if POST_REQUEST_REMOVAL
//-------------------------------------------------------------------------------------------
struct SItem
//-------------------------------------------------------------------------------------------
{
    struct page *page;
    void *vmem;
};
#endif

//-------------------------------------------------------------------------------------------
void hyperion_add_to_cleanup_pipe( struct hyperion* phyperion, struct page* page, void* vmem )
//-------------------------------------------------------------------------------------------
{
#if POST_REQUEST_REMOVAL
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    TItem it;
    BOOLEAN writeOK;
    it.page = page;
    it.vmem = vmem;
    _PIPE_WRITE( phyp_dev->cleanup_request_pipe, it, writeOK );
#endif
}

//-------------------------------------------------------------------------------------------
void hyperion_remove_objects_from_cleanup_pipe( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
#if POST_REQUEST_REMOVAL
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    TItem it;
    BOOLEAN readOK;
    while( _ITEMS_IN_PIPE( phyp_dev->cleanup_request_pipe ) )
    {
        it.page = NULL;
        it.vmem = NULL;
        _PIPE_READ( phyp_dev->cleanup_request_pipe, &it, readOK );
        if( it.page != NULL )
        {
            kunmap( it.page );
        }
        if( it.vmem != NULL )
        {
            PRINTKM( MEM, ( PKTD " %s free vmem %p\n", phyperion->number, __FUNCTION__,  it.vmem ) );
            vfree( it.vmem );
        }
    }
#endif
}

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
void set_mux_data( struct hyperion* phyperion, struct mux_controller_sequence* mux_seq, u_long inputchannel )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    u32* muxdata;
#if DEBUG_PRINT
    int i;
#endif
    PRINTKM( MOD, ( PKTD "%s muxdata changed %s size %d\n", phyperion->number, __FUNCTION__, mux_seq->changed ? "true" : "false", mux_seq->size ) );
    if( mux_seq->changed )
    {
        mux_seq->changed = FALSE;
        if( mux_seq->size < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES )
        {
            muxdata = ( u32* )mux_seq->muxdata;
            CPY_MUX_DATA32( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER ), mux_seq->muxdata, mux_seq->size / sizeof( u32 ) );
#if DEBUG_PRINT
            muxdata = ( u32* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " %p %x\n",  0, &muxdata[i], le32_to_cpu( muxdata[i] ) ) );
            }
#endif
        }
        else
        {
            CPY_MUX_DATA32( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_MUX_RAM_REGISTER ), mux_seq->muxdata, MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) ) );
            CPY_MUX_DATA32( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_MUX_RAM_REGISTER ), &mux_seq->muxdata[MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) )], MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) ) );
#if DEBUG_PRINT
            muxdata = ( u32* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " 0x%x\n",  0, le32_to_cpu( muxdata[i] ) ) );
            }
            muxdata = ( u32* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, inputchannel + ebrhCLController1, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " 0x%x\n",  0, le32_to_cpu( muxdata[i] ) ) );
            }
#endif
        }
        wmb();
    }
}

//-------------------------------------------------------------------------------------------
void set_mux_data_64( struct hyperion* phyperion, struct mux_controller_sequence* mux_seq, u_long inputchannel )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    u32* muxdata;
#if DEBUG_PRINT
    int i;
#endif
    PRINTKM( MOD, ( PKTD "%s muxdata changed %s size %d\n", phyperion->number, __FUNCTION__, mux_seq->changed ? "true" : "false", mux_seq->size ) );
    if( mux_seq->changed )
    {
        mux_seq->changed = FALSE;
        if( mux_seq->size < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES )
        {
            muxdata = ( u32* )mux_seq->muxdata;
            CPY_MUX_DATA_DST64( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER ), mux_seq->muxdata, mux_seq->size / sizeof( u32 ) );
#if DEBUG_PRINT
            muxdata = ( u32* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " %p %x\n",  0, &muxdata[i], le32_to_cpu( muxdata[i] ) ) );
            }
#endif
        }
        else
        {
            CPY_MUX_DATA_DST64( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_MUX_RAM_REGISTER ), mux_seq->muxdata, MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) ) );
            CPY_MUX_DATA_DST64( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_MUX_RAM_REGISTER ), &mux_seq->muxdata[MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) )], MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / ( 2 * sizeof( u32 ) ) );
#if DEBUG_PRINT
            muxdata = ( u32* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, inputchannel + ebrhCLController0, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " 0x%x\n",  0, le32_to_cpu( muxdata[i] ) ) );
            }
            muxdata = ( u32* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, inputchannel + ebrhCLController1, OFF_MUX_RAM_REGISTER );
            for( i = 0; i < MUX_CONTROLLER_SEQUENCE_SIZE_BYTES / sizeof( u32 ); i++ )
            {
                PRINTKM( DMA, ( PKTD " 0x%x\n",  0, le32_to_cpu( muxdata[i] ) ) );
            }
#endif
        }
        wmb();
    }
}

//-------------------------------------------------------------------------------------------
void initialize_lut( struct hyperion* phyperion, u_long lut_offset )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    u_long i;
    u_char* pLUT = vmalloc( LUT_SIZE );
    u_char* lutbase = REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, lut_offset );

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

//-------------------------------------------------------------------------------------------
void register_dump( struct hyperion_device* phyp_dev )
//-------------------------------------------------------------------------------------------
{
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_INTERRUPT_STATUS ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_INTERRUPT_STATUS ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_VERSION ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_VERSION ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_MUX_CONTROLLER ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_SCAN_PIX_LINE0 ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_SCAN_PIX_LINE0 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_SCAN_PIX_LINE1 ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_SCAN_PIX_LINE1 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_SCAN_LINES ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_SCAN_LINES ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_XSTART ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_XSTART ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_XSTOP ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_XSTOP ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_YSTART ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_YSTART ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_YSTOP ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController0, OFF_AOI_YSTOP ) );

    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_MUX_CONTROLLER ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_MUX_CONTROLLER ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_SCAN_PIX_LINE0 ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_SCAN_PIX_LINE0 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_SCAN_PIX_LINE1 ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_SCAN_PIX_LINE1 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_SCAN_LINES ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_SCAN_LINES ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_XSTART ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_XSTART ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_XSTOP ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_XSTOP ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_YSTART ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_YSTART ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_YSTOP ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhCLController1, OFF_AOI_YSTOP ) );

    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CONTROL ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CONTROL ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_VERSION ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_VERSION ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_STAT ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_STAT ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_INTR_ENABLE ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_INTR_ENABLE ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_INTR_STAT_CLEAR ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_INTR_STAT_CLEAR ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_CMD_STAT ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_CMD_STAT ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_CMD_STAT2 ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_CMD_STAT2 ) );
    printk( " %p : 0x%x\n",  REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_XFER_REMAIN2 ), IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DMA_XFER_REMAIN2 ) );
}


//-------------------------------------------------------------------------------------------
void convert_hrtcram_parameter_le( TPropertyElement* list, u32 property_count, u32 hrtc_command_count )
//-------------------------------------------------------------------------------------------
{
    TPropertyElement* prop_hrtc_ram;
    int i;

    _SCAN_PROPERTY_LIST( list, prop_hrtc_ram, property_count, prHRTCRAMData );
    if( prop_hrtc_ram )
    {
        for( i = 0; i < hrtc_command_count && i < emvdHRTCTriggerCommand; i++ )
        {
            prop_hrtc_ram->u.intElement = cpu_to_le32( prop_hrtc_ram->u.intElement );
            ++prop_hrtc_ram;
        }
    }
}


//-------------------------------------------------------------------------------------------
void read_hrtcram_parameter( TPropertyElement* list, u32 property_count, u32 hrtc_command_count, u32* hrtc_ram, u32 buffer_size )
//-------------------------------------------------------------------------------------------
{
    TPropertyElement* prop_hrtc_ram;
    int i;

    memset( ( void* )hrtc_ram, 0, buffer_size );
    _SCAN_PROPERTY_LIST( list, prop_hrtc_ram, property_count, prHRTCRAMData );
    if( prop_hrtc_ram )
    {
        for( i = 0; i < hrtc_command_count && i < emvdHRTCTriggerCommand; i++ )
        {
            PRINTKM( MOD, ( " %s hrtcRAM[%d] %x\n", __FUNCTION__, i, prop_hrtc_ram->u.intElement ) );
            hrtc_ram[i] = prop_hrtc_ram->u.intElement;
            ++prop_hrtc_ram;
        }
    }
}

//-------------------------------------------------------------------------------------------
void get_transfer_param( struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    unsigned long property_count = 0;
    unsigned char changed = FALSE;
    struct transfer_parameter* tp = &phyperion_request_packet->parameters;
    TPropertyElement* list = phyperion_request_packet->request_parameter;
    unsigned long mux_ctrl_count = 0;
    TPropertyElement* pr_mux_ram;

    memset( tp, 0, sizeof( struct transfer_parameter ) );
    _READ_PROPERTYI( list, 1, prInfoPropertyCount, property_count, changed );
    tp->property_count = property_count;
    _READ_PROPERTYI( list, property_count, prInfoRequestAction, tp->reqaction, changed );
    _READ_PROPERTYI( list, property_count, prDMATransferLength, tp->transferlength, changed );
    _READ_PROPERTYI( list, property_count, prInfoFooterOffset, tp->footeroffset, changed );
    _READ_PROPERTYI( list, property_count, prInfoFooterSize, tp->footersize, changed );
    _READ_PROPERTYI( list, property_count, prInfoRequestID, tp->reqid, changed );
    _READ_PROPERTYI( list, property_count, prInfoTimeout, tp->timeoutmsec, changed );
    _READ_PROPERTYI( list, property_count, prCnInput, tp->inputchannel, changed );
    _READ_PROPERTYI( list, property_count, prClXPos, tp->xstart, changed );
    _READ_PROPERTYI( list, property_count, prClWidth, tp->xstop, changed );
    _READ_PROPERTYI( list, property_count, prClYPos, tp->ystart, changed );
    _READ_PROPERTYI( list, property_count, prClHeight, tp->ystop, changed );
    _READ_PROPERTYI( list, property_count, prClScanMode, tp->scanmode, changed );
    _READ_PROPERTYI( list, property_count, prClDataValidEnable, tp->datavalidenable, changed );
    _READ_PROPERTYI( list, property_count, prDMAVideoInDirectTransfer, tp->videoin_direct_transfer, changed );
    _READ_PROPERTYI( list, property_count, prDMAVideoInWaitOfFrameEdge, tp->videoin_wait_of_frameedge, changed );
    _READ_PROPERTYI( list, property_count, prClMediumSource, tp->medium_mode_ctrl, changed );
    _READ_PROPERTYI( list, property_count, prCLEnableAOIMode, tp->enable_aoimode, changed );
    _READ_PROPERTYI( list, property_count, prCLExpandLineValid, tp->expand_lval, changed );
    _READ_PROPERTYI( list, property_count, prDMALineScanStartCondition, tp->line_scan_startcondition, changed );
    _READ_PROPERTYI( list, property_count, prInfoPropertiesChanged, tp->has_properties_changed, changed );
    _READ_PROPERTYI( list, property_count, prCnMuxCtrlCount, mux_ctrl_count, changed );
    _SCAN_PROPERTY_LIST( list, pr_mux_ram, property_count, prCnMuxCtrlRamData );
    if( pr_mux_ram )
    {
        int i;
        struct dma_transfer_object* dto = ( ( struct hyperion_device* )( ( struct hyperion* )( phyperion_request_packet->private ) )->device )->pdma_object[phyperion_request_packet->parameters.inputchannel];
        u32* mux_ram = dto->mux_seq.muxdata;
        dto->mux_seq.changed = 1;
        dto->mux_seq.size = mux_ctrl_count * sizeof( u32 );
        for( i = 0; i < mux_ctrl_count; i++ )
        {
            mux_ram[i] = pr_mux_ram->u.intElement;
            ++pr_mux_ram;
        }
    }
    PRINTKM( MOD, ( PKTD "%s changed %d tf_len 0x%x, footeroff 0x%x, id 0x%x timeout 0x%x w %d h %d sm %d medium_mode_ctrl %d\n", ( ( struct hyperion* )phyperion_request_packet->private )->number, __FUNCTION__, tp->has_properties_changed, tp->transferlength, tp->footeroffset, tp->reqid, tp->timeoutmsec, tp->xstop - tp->xstart, tp->ystop - tp->ystart, tp->scanmode, tp->medium_mode_ctrl ) );
}

//-------------------------------------------------------------------------------------------
void set_device_specific( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    PRINTKM( MOD, ( PKTD " %s vendor 0x%x device 0x%x\n", phyperion->number, __FUNCTION__, ( int )phyperion->vd_id.vendorId, ( int )phyperion->vd_id.deviceId ) );
    switch( phyperion->vd_id.deviceId )
    {
    case PCI_DEVICE_ID_HYPERION_CL4E:
    case PCI_DEVICE_ID_HYPERION_HDSDI_4X:
        phyp_dev->set_mux_data = set_mux_data_64;
        break;
    default:
    case PCI_DEVICE_ID_HYPERION_CLE:
        phyp_dev->set_mux_data = set_mux_data;
        break;
    }
}

//-------------------------------------------------------------------------------------------
int hyperion_board_init( struct hyperion* phyperion, unsigned long control )
//-------------------------------------------------------------------------------------------
{
    if( hyperion_func_init( phyperion, control ) == 0 )
    {
        set_device_specific( phyperion );
        return 0;
    }
    return -EINVAL;
}

//-------------------------------------------------------------------------------------------
int hyperion_board_close( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    hyperion_func_close( phyperion );
    return 0;
}

//-------------------------------------------------------------------------------------------
int hyperion_async_read( struct hyperion* phyperion, struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    int result = -EINVAL, prop_changed = FALSE;
    unsigned long property_count = 0, inputchannel = 0, request_id = 0, request_action = 0;
    TPropertyElement prop_count;

    if( copy_from_user( ( void* )&prop_count, ( void* )( ( UINT_PTR )phyperion_request_packet->request_buffer.leaderBufferPtr ), sizeof( TPropertyElement ) ) )
    {
        PRINTKM( FILE, ( PKTD "copy_from_user() failed\n", phyperion->number ) );
        result = -ENOMEM;
        goto err_out;
    }
    property_count = prop_count.u.intElement;
    if( phyperion_request_packet->size_request_parameter != property_count * sizeof( TPropertyElement ) )
    {
        if( phyperion_request_packet->request_parameter != NULL )
        {
            kfree( phyperion_request_packet->request_parameter );
        }
        phyperion_request_packet->size_request_parameter = property_count * sizeof( TPropertyElement );
        phyperion_request_packet->request_parameter = ( TPropertyElement* ) kmalloc( phyperion_request_packet->size_request_parameter, GFP_KERNEL );
        if( phyperion_request_packet->request_parameter == NULL )
        {
            PRINTKM( FILE, ( PKTD "allocation of request_parameter buffer failed propertycount %d, memsize req 0x%zx, propid %d\n", phyperion->number, prop_count.u.intElement, prop_count.u.intElement * sizeof( TPropertyElement ), prop_count.PropertyID ) );
            return -ENOMEM;
        }
    }
    if( copy_from_user( ( void* )phyperion_request_packet->request_parameter, ( void* )( ( UINT_PTR )phyperion_request_packet->request_buffer.leaderBufferPtr ), phyperion_request_packet->size_request_parameter ) )
    {
        PRINTKM( FILE, ( PKTD "copy_from_user() failed\n", phyperion->number ) );
        goto err_out;
    }
    _READ_PROPERTYI( phyperion_request_packet->request_parameter, property_count, prCnInput, inputchannel, prop_changed );
    _READ_PROPERTYI( phyperion_request_packet->request_parameter, property_count, prInfoRequestID, request_id, prop_changed );
    _READ_PROPERTYI( phyperion_request_packet->request_parameter, property_count, prInfoRequestAction, request_action, prop_changed );
    //PRINTKM(IO,(PKTD "aio_read_write() dma_ch %lu found PROP_START_CODE propCount %lu reqid %lu\n", phyperion->number, inputchannel, property_count, request_id ));
    if( inputchannel  >= MAX_PARALLEL_TRANSFER )
    {
        inputchannel = 0;
    }

    get_transfer_param( phyperion_request_packet );
    result = hyperion_func_async_read( phyperion, phyperion_request_packet );
    if( result < 0 )
    {
        goto err_out;
    }
    return result;

err_out:
    PRINTKM( FILE, ( PKTD "%s() failed result %d\n", phyperion->number, __FUNCTION__, result ) );
    if( phyperion_request_packet->request_parameter != NULL )
    {
        kfree( phyperion_request_packet->request_parameter );
        phyperion_request_packet->request_parameter = NULL;
    }
    return result;
}

//-------------------------------------------------------------------------------------------
void hyperion_abort_transfer( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    hyperion_func_abort_transfer( phyperion );
}

//-------------------------------------------------------------------------------------------
int hyperion_i2c_receive_data( struct hyperion* phyperion, int addr, int sub_addr, int length, unsigned char* pdata )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    int result;
    result = I2CReceiveData( ( unsigned int* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhI2CRead, 0 ), addr, sub_addr, length, pdata );
    return result;
}

//-------------------------------------------------------------------------------------------
int hyperion_get_firmware_version( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_device* phyp_dev = ( struct hyperion_device* )phyperion->device;
    u32 videoInVersion, uartVersion, systemVersion, hrtcVersion;

    videoInVersion = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_VERSION );
    if( videoInVersion != -1 )
    {
        videoInVersion &= VERSIONS_NUMBER_MSK;
    }
    else
    {
        videoInVersion = 0;
    }
    uartVersion = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhUart0, OFF_UART_CONTROL ) >> UART_CTRL_VERSION;
    if( uartVersion == 0xff )
    {
        uartVersion = 0;
    }
    systemVersion = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_VERSION );
    if( systemVersion == -1 )
    {
        systemVersion = 0;
    }
    hrtcVersion = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhHrtController0, OFF_HRT_CONTROLLER_VER );
    if( hrtcVersion == -1 )
    {
        hrtcVersion = 0;
    }
    PRINTKM( MOD, ( PKTD "videoInVersion=0x%x uartVersion=0x%x systemVersion=0x%x hrtcVersion=0x%x \n", phyperion->number, videoInVersion, uartVersion, systemVersion, hrtcVersion ) );
    return ( videoInVersion + uartVersion + systemVersion + hrtcVersion );
}

//-------------------------------------------------------------------------------------------
int hyperion_enable_interrupt( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    return hyperion_func_enable_interrupt( phyperion );
}

//-------------------------------------------------------------------------------------------
void hyperion_release_dma( struct hyperion_request_packet* phyperion_request_packet )
//-------------------------------------------------------------------------------------------
{
    hyperion_func_release_dma( phyperion_request_packet );

    if( phyperion_request_packet->request_parameter != NULL )
    {
        kfree( phyperion_request_packet->request_parameter );
        phyperion_request_packet->request_parameter = NULL;
    }
}

//-------------------------------------------------------------------------------------------
void hyperion_release_hw( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    hyperion_func_release_hw( phyperion );
}

//-------------------------------------------------------------------------------------------
void hyperion_camera_power( struct hyperion* phyperion, int state )
//-------------------------------------------------------------------------------------------
{
    hyperion_func_camera_power( phyperion, state );
}

//-------------------------------------------------------------------------------------------
int hyperion_copy_to_trailer( struct user_buffer_descriptor* puser_buffer_descr, char* src_data, unsigned int size )
//-------------------------------------------------------------------------------------------
{
    int page_idx = 0, page_len = 0;
    char* map_buffer;
    struct page* page;
    int result = 0;

    PRINTKM( MEM, ( " %s( puser_buffer_descr %p, src_data %p, size %d\n", __FUNCTION__, puser_buffer_descr, src_data, size ) );
    while( size > 0 )
    {
        page_len = puser_buffer_descr->sg[page_idx].length;
        PRINTKM( MEM, ( " %s, page_len %d page_off %d\n", __FUNCTION__, puser_buffer_descr->sg[page_idx].length, puser_buffer_descr->sg[page_idx].offset ) );
        if( page_len > size )
        {
            page_len = size;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 24 )
        page = sg_page( &puser_buffer_descr->sg[page_idx] );
#else
        page = puser_buffer_descr->sg[page_idx].page;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 37 )
        map_buffer = ( char* )kmap_atomic( page ) + puser_buffer_descr->sg[page_idx].offset;
#else
        map_buffer = ( char* )kmap_atomic( page, KM_SOFTIRQ0 ) + puser_buffer_descr->sg[page_idx].offset;
#endif
        if( map_buffer != NULL )
        {
            PRINTKM( MEM, ( " %s, map_buffer %p, src_buffer %p, page_len (size to copy) %d\n", __FUNCTION__, map_buffer, src_data, page_len ) );
            memcpy( map_buffer, src_data, page_len );
        }
        else
        {
            size = 0;
            result = -1;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 37 )
        kunmap_atomic( map_buffer );
#else
        kunmap_atomic( map_buffer, KM_SOFTIRQ0 );
#endif
        ++page_idx;
        size -= page_len;
        src_data += page_len;
    }
    return result;
}
