#ifndef SerialPortIoCtlH
#define SerialPortIoCtlH SerialPortIoCtlH

case IOCTL_FLUSHPORT:
{
    CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
    if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
    {
        reset_read_write_fifo( &phyp_dev->uart_port[cl_data->port] );
    }
    break;
}
case IOCTL_GETNUMOFBYTESAVAIL:
{
    u32 bytes_available = 0;
    CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
    if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
    {
        bytes_available = getnumber_bytes_available( &phyp_dev->uart_port[cl_data->port] );
    }
    //PRINTKM ( IOCTL, ( PKTD "IOCTL_GETNUMOFBYTESAVAIL bytes_available %u, out_size %u (%u)\n", phyp_dev->index, bytes_available, mvio.out_size, sizeof( u32 ) ) );
    if( mvio.out_size >= sizeof( u32 ) )
    {
        memcpy( IOBUFFER_TO( void* ), ( void* )&bytes_available, sizeof( u32 ) );
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = sizeof( u32 );
        write_back_to_user = TRUE;
    }
    break;
}
case IOCTL_SETBAUDRATE:
{
    CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
    if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
    {
        set_baudrate( &phyp_dev->uart_port[cl_data->port], cl_data->data );
    }
    break;
}
case IOCTL_SERIALINIT:
{
    CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
    if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
    {
        PRINTKM( IOCTL, ( PKTD "IOCTL_SERIALINIT port %d\n", phyperion->number, cl_data->port ) );
        error = create_serial( &phyp_dev->hyperion_base, phyp_dev->reg_def, &phyp_dev->uart_port[cl_data->port], cl_data->port + ebrhUart0, phyperion->number );
        //if successful return 1, on error return 0
        //reset_read_write_fifo( &phyp_dev->uart_port[cl_data->port] );
    }
    break;
}
case IOCTL_SERIALCLOSE:
{
    CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
    if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
    {
        PRINTKM( IOCTL, ( PKTD "IOCTL_SERIALCLOSE port %d\n", phyperion->number, cl_data->port ) );
        close_serial( &phyp_dev->uart_port[cl_data->port] );
    }
    break;
}
case IOCTL_SERIALREAD:
{
    unsigned long read_status;
    CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
    if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
    {
        read_status = read_serial_data( &phyp_dev->uart_port[cl_data->port], cl_data->data, IOBUFFER_TO( unsigned char* ), mvio.out_size );
        if( read_status == secNoError )
        {
            ( ( struct mv_ioctl* )io_buffer )->bytes_returned = ( read_status == secNoError ) ? mvio.out_size : 0;
        }
        write_back_to_user = TRUE;
    }
    break;
}
case IOCTL_SERIALWRITE:
{
    unsigned long bytes_written;
    CAMERA_LINK_DATA* cl_data = IOBUFFER_TO( CAMERA_LINK_DATA* );
    if( mvio.in_size >= sizeof( CAMERA_LINK_DATA ) && cl_data->port < UART_NUM )
    {
        bytes_written = write_serial_data( &phyp_dev->uart_port[cl_data->port], cl_data->data, IOBUFFER_TO( unsigned char* ) + sizeof( CAMERA_LINK_DATA ), mvio.in_size - sizeof( CAMERA_LINK_DATA ) );
        ( ( struct mv_ioctl* )io_buffer )->bytes_returned = bytes_written;
        write_back_to_user = TRUE;
    }
    break;
}
#endif //SerialPortIoCtlH
