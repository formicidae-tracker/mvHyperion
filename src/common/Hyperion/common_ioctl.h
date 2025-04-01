#ifndef CommonIoCtlH
#define CommonIoCtlH CommonIoCtlH

case IOCTL_ABORTALLREQUESTS:
{
    hyperion_abort_transfer( phyperion );
    break;
}
case IOCTL_READRESULTPACKET:
/*{
    TResultPacket rp;
    result_packet_entry_t *entry = pop_result_packet_entry( &phyperion->result_queue );
    if( entry != NULL )
    {
        rp.ResultPacket = entry->result_packet;
        rp.Status = entry->status;
        free_result_packet_entry( entry );
    }
    else
    {
        rp.ResultPacket = NULL;
        rp.Status = cerrResultQueueEmpty;
    }
    PRINTKM (IOCTL,(PKTD "ioctl cmd=0x%08x resultpacket %p stat %u32\n", phyperion->number, cmd,  rp.ResultPacket, rp.Status ));
    if( mvio.out_buffer != NULL && mvio.out_size >= sizeof( TResultPacket ) )
    {
        uva.uptr = (unsigned long)mvio.out_buffer;
        mvio.bytes_returned = write_user_buffer( &rp, uva, mvio.out_size );
    }
}*/
break;
case IOCTL_EMPTYREQUESTQUEUE:
{
    break;
}
case IOCTL_EMPTYRESULTQUEUE:
/*{
    empty_result_queue( &phyperion->result_queue );
}*/
break;
case IOCTL_READ_EEPROM:
{
    if( mvio.out_size <= EEPROM_SIZE )
    {
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        I2CReceiveData( ( unsigned int* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhI2CRead, 0 ), EEPROM_ADDR, 0, mvio.out_size, IOBUFFER_TO( unsigned char* ) );
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.out_size;
        write_back_to_user = TRUE;
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
    }
    break;
}
case IOCTL_WRITE_EEPROM:
{
    if( mvio.in_size <= EEPROM_SIZE && phyp_dev->eeprom_write_access == ( PCI_VENDOR_ID_MATRIX_VISION << 16 && EEPROM_WRITE_ACCESS ) )
    {
        int i;
        unsigned char* eepromdata = IOBUFFER_TO( unsigned char* );
        PRINTKM( IOCTL, ( PKTD " %s, write_eeprom in_size %d\n", phyperion->number, __FUNCTION__, mvio.in_size ) );
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        for( i = 0; i < mvio.in_size; i++ )
        {
            I2CSendData( ( unsigned int* )REG_POINTER( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhI2CRead, 0 ), EEPROM_ADDR, i, 1, &eepromdata[i] );
        }
        phyp_dev->eeprom_write_access = -1;
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
    }
    break;
}
case IOCTL_EEPROM_ACCESS:
{
    if( mvio.in_size >= sizeof( int ) )
    {
        int* access = IOBUFFER_TO( int* );
        phyp_dev->eeprom_write_access = *access;
        PRINTKM( IOCTL, ( PKTD " %s, eeprom_write_access %x\n", phyperion->number, __FUNCTION__, phyp_dev->eeprom_write_access ) );
    }
    break;
}
case IOCTL_WRITE_MUX_DATA:
{
    if( mvio.in_size <= MUX_CONTROLLER_SEQUENCE_SIZE_BYTES )
    {
        mutex_lock( &phyp_dev->ioctl_lock.s_request );
        phyp_dev->mux_seq.changed = TRUE;
        phyp_dev->mux_seq.size = mvio.in_size;
        memcpy( ( void* )phyp_dev->mux_seq.muxdata, IOBUFFER_TO( void* ), mvio.in_size );
        mutex_unlock( &phyp_dev->ioctl_lock.s_request );
    }
    break;
}
case IOCTL_READ_ASMI_U32:
{
    if( mvio.in_size >= sizeof( TRegisterAccess ) )
    {
        TRegisterAccess* regacc = IOBUFFER_TO( TRegisterAccess* );
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        regacc->Data = ( unsigned long )IO_READ_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhAsmiInterface, regacc->Offset );
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = mvio.in_size;
        write_back_to_user = TRUE;
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
        //PRINTKM (IOCTL,(PKTD " IOCTL_READ_ASMI_U32 pasmi %p data read %x\n", phyperion->number, pasmi_reg, regacc->Data ));
    }
    break;
}
case IOCTL_WRITE_ASMI_U32:
{
    if( mvio.in_size >= sizeof( TRegisterAccess ) )
    {
        TRegisterAccess* regacc = IOBUFFER_TO( TRegisterAccess* );
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhAsmiInterface, regacc->Offset, regacc->Data );
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
        //PRINTKM (IOCTL,(PKTD " IOCTL_WRITE_ASMI_U32 pasmi %p data %x\n", phyperion->number, pasmi_reg, regacc->Data ));
    }
    break;
}
case IOCTL_WRITE_HRTC:
{
    TPropertyElement* prop_element, *prop_hrtc_ram;
    u32 i, property_count = 0, hrtc_index = 0, hrtc_control = 0;
    unsigned char changed = FALSE;

    mutex_lock( &phyp_dev->ioctl_lock.s_digital_io );
    property_count = mvio.in_size / sizeof( TPropertyElement );
    prop_element = IOBUFFER_TO( TPropertyElement* );
    _READ_PROPERTYI( prop_element, property_count, prHRTCIndex, hrtc_index, changed );
    _READ_PROPERTYI( prop_element, property_count, prHRTCControl, hrtc_control, changed );
    IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhHrtController0 + hrtc_index, OFF_HRT_CONTROLLER_CTRL, 0 );
    _SCAN_PROPERTY_LIST( prop_element, prop_hrtc_ram, property_count, prHRTCRAMData );
    if( prop_hrtc_ram )
    {
        property_count -= ( ( ( char* )prop_hrtc_ram - ( char* )prop_element ) / sizeof( TPropertyElement ) );
        for( i = 0; i < property_count; i++ )
        {
            IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhHrtController0 + hrtc_index, ( OFF_HRT_CONTROLLER_RAM + ( i * sizeof( u32 ) ) ), prop_hrtc_ram->u.intElement );
            //printk(PKTD "IOCTL_WRITE_HRTC off %p val %x\n", phyperion->number, REG_POINTER(phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhHrtController0+hrtc_index, OFF_HRT_CONTROLLER_RAM+(i*sizeof(u32))), prop_hrtc_ram->u.intElement );
            prop_hrtc_ram++;
        }
    }
    IO_WRITE_32( phyp_dev->hyperion_base, phyp_dev->reg_def, ebrhHrtController0 + hrtc_index, OFF_HRT_CONTROLLER_CTRL, hrtc_control );
    mutex_unlock( &phyp_dev->ioctl_lock.s_digital_io );
    wmb();
    break;
}
case IOCTL_WRITE_HRTC_U32:
{
    if( mvio.in_size >= sizeof( TRegisterAccess ) /*&& phyperion->hrtc_version != -1*/ )
    {
        TRegisterAccess* regacc = IOBUFFER_TO( TRegisterAccess* );
        //unsigned long *phrtc_reg = (unsigned long*)(phyperion->hyperion_register.base + regacc->Offset);
        //*phrtc_reg = regacc->Data;
        mutex_lock( &phyp_dev->ioctl_lock.s_digital_io );
        iowrite32( regacc->Data, ( void* )( phyp_dev->hyperion_base.base + regacc->Offset ) );
        mutex_unlock( &phyp_dev->ioctl_lock.s_digital_io );
    }
    break;
}
case IOCTL_UNMAP_SG_LIST:
{
#ifdef REMOVE_REQUEST_BUFFER_MAPPING
#else
    if( mvio.in_size >= sizeof( TUserBuffer ) )
    {
        TUserBuffer* ubuf = IOBUFFER_TO( TUserBuffer* );
        mutex_lock( &phyp_dev->ioctl_lock.s_request );
        if( ubuf->buffer == NULL )
        {
            free_all_user_buffer_descriptors( phyperion );
        }
        else
        {
            free_user_buffer_descriptor( phyperion, ubuf->buffer, ubuf->count );
        }
        mutex_unlock( &phyp_dev->ioctl_lock.s_request );
    }
#endif
    break;
}
case IOCTL_POWER_OVER_CL:
{
    if( mvio.in_size >= sizeof( TRegisterAccess ) )
    {
        TRegisterAccess* regacc = IOBUFFER_TO( TRegisterAccess* );
        mutex_lock( &phyp_dev->ioctl_lock.s_generic );
        hyperion_camera_power( phyperion, regacc->Data );
        mutex_unlock( &phyp_dev->ioctl_lock.s_generic );
    }
    break;
}
default:
//printk(PKTD "ioctl: invalid ioctl request, cmd=$%08x, size=%d, dir=%d\n", phyperion->number, cmd, size, dir);
error = -EINVAL;
break;

#endif //CommonIoCtlH
