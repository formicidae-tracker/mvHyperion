#ifndef CLeIoCtlH
#define CLeIoCtlH CLeIoCtlH

case IOCTL_QUERYINFO:
{
    u32* info_to_query = IOBUFFER_TO( void* );
    switch( *info_to_query )
    {
    case eqiEepromContent:
        break;
    case eqiHRTCFrequency:
        {
            u32 bytes_returned, hrtc_freq[2];
            if( IO_READ_32( device->hyperion_base, device->reg_def, ebrhHrtController0, OFF_HRT_CONTROLLER_VER ) > 1 )
            {
                hrtc_freq[0] = IO_READ_32( device->hyperion_base, device->reg_def, ebrhHrtController0, OFF_HRT_CONTROLLER_CLK );
                hrtc_freq[1] = IO_READ_32( device->hyperion_base, device->reg_def, ebrhHrtController1, OFF_HRT_CONTROLLER_CLK );
            }
            else
            {
                hrtc_freq[0] = HRTC_VER1_FREQ_HZ;
                hrtc_freq[1] = hrtc_freq[0];
            }
            //PRINTKM (IOCTL,(PKTD " hrtcFreq[0] %d hrtcFreq[1] %d cbout %d arraysize %lu\n", device->index, hrtc_freq[0], hrtc_freq[1], mvio.out_size, arraysize(hrtc_freq) ));
            bytes_returned = arraysize( hrtc_freq ) * sizeof( u32 ) < mvio.out_size ? arraysize( hrtc_freq ) * sizeof( u32 ) : mvio.out_size;
            memcpy( IOBUFFER_TO( void* ), hrtc_freq, bytes_returned );
            ( ( struct mv_ioctl* )io_buffer )->bytes_returned = bytes_returned;
            write_back_to_user = TRUE;
            break;
        }
    case eqiDigitalOutputAvailable:
        {
            if( mvio.in_size >= sizeof( unsigned int ) && mvio.out_size >= sizeof( unsigned int ) )
            {
                *IOBUFFER_TO( unsigned int* ) = IsDigitalOutputAvailable( device->hyperion_base.base, device->reg_def, *( IOBUFFER_TO( unsigned int* ) + 1 ) );
                PRINTKM( IOCTL, ( PKTD " %s, output %d available %d\n", device->index, __FUNCTION__, *( IOBUFFER_TO( unsigned int* ) + 1 ), *IOBUFFER_TO( unsigned int* ) ) );
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
                write_back_to_user = TRUE;
            }
            else
            {
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = 0;
            }
            break;
        }
    case eqiDigitalInputAvailable:
        {
            if( mvio.in_size >= sizeof( unsigned int ) && mvio.out_size >= sizeof( unsigned int ) )
            {
                *IOBUFFER_TO( unsigned int* ) = IsDigitalInputAvailable( device->hyperion_base.base, device->reg_def, *( IOBUFFER_TO( unsigned int* ) + 1 ) );
                PRINTKM( IOCTL, ( PKTD " %s, input %d available %d\n", device->index, __FUNCTION__, *( IOBUFFER_TO( unsigned int* ) + 1 ), *IOBUFFER_TO( unsigned int* ) ) );
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
                write_back_to_user = TRUE;
            }
            else
            {
                ( ( struct mv_ioctl* )io_buffer )->bytes_returned = 0;
            }
            break;
        }
    case eqiTriggerModeAvailable:
        {
            if( mvio.in_size >= sizeof( unsigned int ) && mvio.out_size >= sizeof( unsigned int ) )
            {
                //currently removed
                //unsigned int videoin_version = IO_READ_32(device->hyperion_base, device->reg_def, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CTRL_VERSION);
                //*IOBUFFER_TO(unsigned int*) = ((videoin_version & VERSIONS_NUMBER_MSK) >= TRIGGER_MODE_AVAILABLE_VERSION);
                *IOBUFFER_TO( unsigned int* ) = FALSE;
                PRINTKM( IOCTL, ( PKTD " %s, trigger_mode available %d\n", device->index, __FUNCTION__, *( IOBUFFER_TO( unsigned int* ) ) ) );
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
    break;
}
case IOCTL_RESET_DMA_CONTROLLER:
{
    u32 i, system_control;
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        if( device->pdma_object[i]->init_done == FALSE )
        {
            continue;
        }
        //ResetDMAController( device->pdma_object[0]->dma_controller_pci );
        ResetDMAController( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo0] );
        ResetDMAController( device->pdma_object[i]->dma_controller_videoin[cfdCommandFifo1] );
        device->pdma_object[i]->restart_current_ioobj = FALSE;
    }
    system_control = IO_READ_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL );
    IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL, ( system_control | RESET_CL_RECEIVER ) );
    wait_jiffies( 1 ); //delay register access
    IO_WRITE_32( device->hyperion_base, device->reg_def, ebrhSystemRegister, OFF_SYSTEM_CONTROL, system_control );
    wait_jiffies( msecs_to_jiffies( 20 ) );
    free_all_ubuf( device );
    break;
}
case IOCTL_CONFIG_IO_BIT:
{
    TPropertyElement* propElement;
    u32 property_count = 0, output = 0, passthrough_inv = 0, input_to_passthrough = 0;
    unsigned char changed = FALSE;

    property_count = mvio.in_size / sizeof( TPropertyElement );
    propElement = IOBUFFER_TO( TPropertyElement* );
    _READ_PROPERTYI( propElement, property_count, prIOOutput, output, changed );
    _READ_PROPERTYI( propElement, property_count, prPassThroughInverted, passthrough_inv, changed );
    _READ_PROPERTYI( propElement, property_count, prDigInPassThrough, input_to_passthrough, changed );
    ConfigureDigitalOutput( device->hyperion_base.base, device->reg_def, output, passthrough_inv, input_to_passthrough );
    //PRINTKM (IOCTL,(PKTD " configio: output %u syncmode %x syncsel %x\n", device->index, output, sync_mode, input_to_passthrough ));
    wmb();
    break;
}
case IOCTL_READ_DIGITAL_INPUT:
{
    if( mvio.in_size >= sizeof( unsigned int ) )
    {
        unsigned int state = ReadDigitalInput( device->hyperion_base.base, device->reg_def, *IOBUFFER_TO( unsigned int* ) );
        *IOBUFFER_TO( unsigned int* ) = state;
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
        write_back_to_user = TRUE;
    }
    break;
}
case IOCTL_WRITE_IO_BIT:
{
    TPropertyElement* propElement;
    u32 property_count = 0, output = 0, state = 0;
    unsigned char changed = FALSE;

    property_count = mvio.in_size / sizeof( TPropertyElement );
    propElement = IOBUFFER_TO( TPropertyElement* );
    _READ_PROPERTYI( propElement, property_count, prIOOutput, output, changed );
    _READ_PROPERTYI( propElement, property_count, prIOState, state, changed );
    WriteDigitalOutput( device->hyperion_base.base, device->reg_def, output, state );
    //PRINTKM (IOCTL,(PKTD " writeiobit: propcnt %u output %u state %u\n", device->index, property_count, output, state ));
    wmb();
    break;
}
case IOCTL_CLEANUP_REQUEST_EXTENSION:
{
    remove_objects_from_cleanup_pipe( device );
    break;
}

#endif //CLeIoCtlH
