#ifndef CLfIoCtlH
#define CLfIoCtlH CLfIoCtlH

case IOCTL_QUERY_CAPABILITIES:
{
    if( mvio.out_size >= sizeof( int ) )
    {
        *IOBUFFER_TO( int* ) = edcSupportsLeaderTrailerStruct;
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
        write_back_to_user = TRUE;
    }
    break;
}
case IOCTL_QUERYINFO:
{
    u32* info_to_query = IOBUFFER_TO( void* );
    mutex_lock( &phyp_dev->ioctl_lock.s_generic );
    switch( *info_to_query )
    {
    case eqiEepromContent:
        break;
    case eqiHRTCFrequency:
        {
            u32 bytes_returned, hrtc_freq[2];
            if( IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhHrtController0, OFF_HRT_CONTROLLER_VER ) > 1 )
            {
                hrtc_freq[0] = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhHrtController0, OFF_HRT_CONTROLLER_CLK );
                hrtc_freq[1] = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhHrtController1, OFF_HRT_CONTROLLER_CLK );
            }
            else
            {
                hrtc_freq[0] = HRTC_VER1_FREQ_HZ;
                hrtc_freq[1] = hrtc_freq[0];
            }
            //PRINTKM (IOCTL,(PKTD " hrtcFreq[0] %d hrtcFreq[1] %d cbout %d arraysize %lu\n", phyperion->number, hrtc_freq[0], hrtc_freq[1], mvio.out_size, arraysize(hrtc_freq) ));
            bytes_returned = arraysize( hrtc_freq ) * sizeof( u32 ) < mvio.out_size ? arraysize( hrtc_freq ) * sizeof( u32 ) : mvio.out_size;
            memcpy( IOBUFFER_TO( void* ), hrtc_freq, bytes_returned );
            ( ( struct mv_ioctl* )io_buffer )->bytes_returned = bytes_returned;
            write_back_to_user = TRUE;
            break;
        }
    case eqiDigitalOutputAvailable:
        {
            ( ( struct mv_ioctl* )io_buffer )->bytes_returned = 0;
            break;
        }
    case eqiDigitalInputAvailable:
        {
            ( ( struct mv_ioctl* )io_buffer )->bytes_returned = 0;
            break;
        }
    case eqiTriggerModeAvailable:
        {
            ( ( struct mv_ioctl* )io_buffer )->bytes_returned = 0;
            break;
        }
    case eqiMaxRequestObjectCount:
        {
            if( mvio.out_size >= sizeof( unsigned long ) )
            {
                *IOBUFFER_TO( unsigned long* ) = MAX_COUNT_REQUEST_OBJECT;
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
                write_back_to_user = TRUE;
            }
            else
            {
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = 0;
            }
            break;
        }
    case eqiHyperionProduct:
        {
            if( mvio.in_size >= sizeof( TQueryInformation ) )
            {
                TQueryInformation* query_info = IOBUFFER_TO( TQueryInformation* );
                query_info->u.intElement = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL ) & PRODUCT_TYPE_MSK;
                query_info->typeSize = sizeof( query_info->u.intElement );
                //PRINTKM (IOCTL,(PKTD ": " "%s, id %x, %x\n", phyperion->number, __FUNCTION__,queryInformation->informationID, queryInformation->u.intElement ));
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
    mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
    break;
}
case IOCTL_BOOT_NIOS:
{
    if( mvio.in_size <= NIOS_MEMORY_APP_SIZE )
    {
        int proc_status = -1;
        proc_status = hyperion_boot_processor( phyp_dev, IOBUFFER_TO( void* ), mvio.in_size );
        phyp_dev->processor_app_size = mvio.in_size;
        phyp_dev->processor_status = proc_status;
        //PRINTKM (IOCTL,(PKTD " IOCTL_WRITE_ASMI_U32 pasmi %p data %x\n", phyperion->number, pasmi_reg, regacc->Data ));
    }
    break;
}
case IOCTL_RESET_DMA_CONTROLLER:
{
    u32 ctrl_reg;
    switch( phyperion->vd_id.deviceId )
    {
    case PCI_DEVICE_ID_HYPERION_HDSDI_4X:
        {
            u32 delay_msec = 10;
            ctrl_reg = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDecCtrl0, 0 );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDecCtrl0, 0, ( ctrl_reg | SDI_DECODER_RESET ) );
            wait_jiffies( msecs_to_jiffies( delay_msec ) );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDecCtrl0, 0, ctrl_reg );
            ctrl_reg = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDecCtrl1, 0 );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDecCtrl1, 0, ( ctrl_reg | SDI_DECODER_RESET ) );
            wait_jiffies( msecs_to_jiffies( delay_msec ) );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhDecCtrl1, 0, ctrl_reg );
        }
        break;
    case PCI_DEVICE_ID_HYPERION_CL4E:
        {
            ctrl_reg = IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL );
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL, ( ctrl_reg | RESET_CL_RECEIVER ) );
            wait_jiffies( 1 ); //delay register access
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL, ctrl_reg );
            wait_jiffies( msecs_to_jiffies( 20 ) );
            free_all_user_buffer_descriptors( phyperion );
            ChannelLinkReset( &phyp_dev->pocl[0] );
        }
        break;
    }
    break;
}
case IOCTL_CONFIG_IO_BIT:
{
    unsigned int message = MESSAGE_TYPE_METHOD | ( ID_METHOD_CONFIGURE_DIGITAL_OUTPUT & MESSAGE_INDEX_MSK );
    int result = 0;
    TPropertyElement32* list32;
    mutex_lock( &phyp_dev->ioctl_lock.s_digital_io );
    list32 = ( TPropertyElement32* )kmalloc( mvio.in_size, GFP_KERNEL );
    translate_property_to_property32( IOBUFFER_TO( TPropertyElement* ), list32, ( mvio.in_size / sizeof( TPropertyElement ) ) );
    result = transmit_message( phyp_dev, message, 500, list32, mvio.in_size, NULL );
    kfree( list32 );
    mutex_unlock( &phyp_dev->ioctl_lock.s_digital_io );
    PRINTKM( IOCTL, ( PKTD " config_io_bit: transmit_message() --> result %d\n", phyperion->number, result ) );
    break;
}
case IOCTL_WRITE_IO_BIT:
{
    u32 message = MESSAGE_TYPE_METHOD | ( ID_METHOD_WRITE_DIGITAL_OUTPUT & MESSAGE_INDEX_MSK );
    int result = 0;
    TPropertyElement32* list32, *pl32;
    mutex_lock( &phyp_dev->ioctl_lock.s_digital_io );
    list32 = ( TPropertyElement32* )kmalloc( mvio.in_size, GFP_KERNEL );
    pl32 = list32;
    translate_property_to_property32( IOBUFFER_TO( TPropertyElement* ), list32, ( mvio.in_size / sizeof( TPropertyElement ) ) );
    list32 = pl32;
    result = transmit_message( phyp_dev, message, 500, /*IOBUFFER_TO(void*)*/ list32, mvio.in_size, NULL );
    kfree( pl32 );
    mutex_unlock( &phyp_dev->ioctl_lock.s_digital_io );
    PRINTKM( IOCTL, ( PKTD " write_io_bit: transmit_message() --> result %d\n", phyperion->number, result ) );
    break;
}
case IOCTL_READ_DIGITAL_INPUT:
{
#define IO_READ_PROP_CNT 2
    u32 message = MESSAGE_TYPE_METHOD | ( ID_METHOD_READ_DIGITAL_INPUT & MESSAGE_INDEX_MSK );
    int result = 0;
    unsigned long state;
    TPropertyElement* list, *pl;
    TPropertyElement32* list32, *pl32;
    mutex_lock( &phyp_dev->ioctl_lock.s_digital_io );
    list = kmalloc( IO_READ_PROP_CNT * sizeof( TPropertyElement ), GFP_KERNEL );
    pl = list;
    list32 = kmalloc( IO_READ_PROP_CNT * sizeof( TPropertyElement ), GFP_KERNEL );
    pl32 = list32;
    _WRITE_PROPERTYI( list, prIOPropertyCount, 2, TRUE );
    _WRITE_PROPERTYI( list, prIOOutput, *IOBUFFER_TO( unsigned int* ), TRUE );
    list = pl;
    translate_property_to_property32( list, list32, IO_READ_PROP_CNT );
    list = pl;
    list32 = pl32;
    result = transmit_message( phyp_dev, message, 500, list32, IO_READ_PROP_CNT * sizeof( TPropertyElement32 ), &state );
    *IOBUFFER_TO( unsigned int* ) = ( unsigned int )state;
    if( result == 0 )
    {
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
        write_back_to_user = TRUE;
    }
    kfree( list );
    kfree( list32 );
    mutex_unlock( &phyp_dev->ioctl_lock.s_digital_io );
    PRINTKM( IOCTL, ( PKTD " read_io_bit: transmit_message() --> result %d iobit_state %d\n", phyperion->number, result, ( int )state ) );
    break;
}
case IOCTL_CLEANUP_REQUEST_EXTENSION:
{
    break;
}
case IOCTL_SPI_READ:
{
    if( mvio.in_size >= sizeof( TSPIAccess ) )
    {
        const unsigned long data_len = 2;
        unsigned long write_data = 0, read_data[data_len];
        int result;
        const int RW_EN = 1 << 15;
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        write_data = RW_EN | ( unsigned short )( ( IOBUFFER_TO( TSPIAccess* ) )->Offset );
        result = spi_command16( ( unsigned char* )( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSPISimple, 0 ) ), ( IOBUFFER_TO( TSPIAccess* ) )->Slave, 1, ( unsigned short* )&write_data, 1, ( unsigned short* )read_data, 0/*ALT_AVALON_SPI_COMMAND_TOGGLE_SS_N*/ );
        ( IOBUFFER_TO( TRegisterAccess* ) )->Data = read_data[0];
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned =  mvio.out_size;
        write_back_to_user = TRUE;
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
    }
    break;
}
case IOCTL_SPI_READ_LEN:
{
    if( mvio.in_size >= sizeof( TSPIAccess ) && mvio.out_size > 0 )
    {
        unsigned int write_data = 0;
        unsigned short slave, offset;
        int result;
        const int RW_EN = 1 << 15;
        const int AUTO_INC = 1 << 12;
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        offset = ( IOBUFFER_TO( TSPIAccess* ) )->Offset;
        slave = ( IOBUFFER_TO( TSPIAccess* ) )->Slave;
        write_data = RW_EN | AUTO_INC | offset;
        result = spi_command16( ( unsigned char* )( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSPISimple, 0 ) ), slave, 1, ( unsigned short* )&write_data, mvio.out_size / sizeof( unsigned short ), IOBUFFER_TO( unsigned short* ), 0/*ALT_AVALON_SPI_COMMAND_TOGGLE_SS_N*/ );
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned =  mvio.out_size;
        write_back_to_user = TRUE;
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
    }
    break;
}
case IOCTL_SPI_WRITE:
{
    if( mvio.in_size >= sizeof( TSPIAccess ) )
    {
        unsigned long write_data;
        int result;
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        write_data = ( unsigned short )( ( IOBUFFER_TO( TSPIAccess* ) )->Offset ) | ( short )( ( IOBUFFER_TO( TSPIAccess* ) )->Data ) << 16;
        result = spi_command16( ( unsigned char* )( REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhSPISimple, 0 ) ), ( IOBUFFER_TO( TSPIAccess* ) )->Slave, 2, ( unsigned short* )&write_data, 0, NULL, 0/*ALT_AVALON_SPI_COMMAND_TOGGLE_SS_N*/ );
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
    }
    break;
}
case IOCTL_READ_REGISTER_U32:
{
    if( mvio.in_size >= sizeof( TRegisterAccess ) )
    {
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        ( IOBUFFER_TO( TRegisterAccess* ) )->Data = ioread32( ( void __iomem* )( ( unsigned char* )phyp_dev->hyperion_base.base + ( IOBUFFER_TO( TRegisterAccess* ) )->Offset ) );
        //printk( "IOCTL_READ_REGISTER_U32 offset %lx data_read %lx\n", (IOBUFFER_TO(TRegisterAccess*))->Offset, (IOBUFFER_TO(TRegisterAccess*))->Data );
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned =  mvio.out_size;
        write_back_to_user = TRUE;
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
    }
    break;
}
case IOCTL_WRITE_REGISTER_U32:
{
    if( mvio.in_size >= sizeof( TRegisterAccess ) )
    {
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        iowrite32( ( IOBUFFER_TO( TRegisterAccess* ) )->Data, ( void __iomem* )( ( unsigned char* )phyp_dev->hyperion_base.base + ( IOBUFFER_TO( TRegisterAccess* ) )->Offset ) );
        //printk( "IOCTL_WRITE_REGISTER_U32 offset %lx data_read %lx\n", (IOBUFFER_TO(TRegisterAccess*))->Offset, (IOBUFFER_TO(TRegisterAccess*))->Data );
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
    }
    break;
}

#endif //CLfIoCtlH
