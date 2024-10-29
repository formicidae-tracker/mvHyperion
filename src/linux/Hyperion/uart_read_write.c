//serial communication with hyperion boards

#include "hyperion.h"
#include "matrix_tools.h"
//----------------------------------------------------------------------------------
/// \brief UART item definition
struct SItem
{
    unsigned char data;
};

//----------------------------------------------------------------------------------
unsigned long read_serial_data( uart_object_t* pua, unsigned long timeoutmsec, unsigned char* data, unsigned long bufferlength )
//----------------------------------------------------------------------------------
{
    unsigned long expires, data_read = 0, j_delay = 2;
    TItem it;
    unsigned char read_ok;
    expires = jiffies + msecs_to_jiffies( timeoutmsec );
    //PRINTKM(IO,(PKTD "%s pua %p timeoutmsec %lu data %p bufferlength %lu bytesavailable %lu\n", pua->device_index, __FUNCTION__, pua, timeoutmsec, data, bufferlength, getnumber_bytes_available( pua ) ));

    it.data = 0;
    do
    {
        if( bufferlength > 0 && getnumber_bytes_available( pua ) >= bufferlength )
        {
            //DoTraceMessage(TRACELEVELUART, " NumberBytesAvailable %d", GetNumberBytesAvailable( pua ) );
            do
            {
                _PIPE_READ( pua->read_pipe, &it, read_ok );
                data[data_read++] = it.data;
            }
            while( data_read < bufferlength );
            pua->read_write_status = secNoError;
            break;
        }
        else
        {
            set_current_state( TASK_INTERRUPTIBLE );
            schedule_timeout( j_delay );
            pua->read_write_status = secErrTimeout;
        }
    }
    while( jiffies < expires );

    return pua->read_write_status;
}

//----------------------------------------------------------------------------------
unsigned long write_serial_data( uart_object_t* pua, unsigned long timeoutmsec, unsigned char* data, unsigned long bufferlength )
//----------------------------------------------------------------------------------
{
    unsigned long data_written = 0, expires, j_delay = 2;
    PRINTKM( IO, ( PKTD "%s timeoutmsec %lu bufferlength %lu\n", pua->device_index, __FUNCTION__, timeoutmsec, bufferlength ) );
    expires = jiffies + msecs_to_jiffies( timeoutmsec );


    do
    {
        if( !( IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0 ) & CTRL_READ_TX_FIFO_FULL ) )
        {
            //PRINTKM(IO,(PKTD "%c\n", pua->device_index, data[data_written] ));
            //printk( KERN_INFO "%c\n", data[data_written] );
            IO_WRITE_8( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_DATA, data[data_written++] );
        }
        else
        {
            set_current_state( TASK_INTERRUPTIBLE );
            schedule_timeout( j_delay );
        }
    }
    while( jiffies < expires && data_written < bufferlength );

    PRINTKM( IO, ( PKTD "%s data_written %lu\n", pua->device_index, __FUNCTION__, data_written ) );
    return data_written;
}

///interrupt oder von einem Timer ausgel�t wird, somit ist sicher gestellt, dass immer nach daten geschaut wird.
//----------------------------------------------------------------------------------
void read_serial( uart_object_t* pua )
//----------------------------------------------------------------------------------
{
    unsigned char writeOK;
    TItem it;
    unsigned long bytes_read = 0;
    /*static int msg_cnt = 0;

    if( msg_cnt++ % 100 == 0 )
        PRINTKM(IO,(PKTD " jiffies %d timeoutjiffies %d data available %d\n", pua->device_index, jiffies, pua->timeout_jiffies, _READ_BIT( pua->uart_reg.UARTRead, Ctrl, rxd_pres ) ));
        //printk( KERN_INFO " jiffies %d timeoutjiffies %d data available %d\n", jiffies, pua->timeout_jiffies, _READ_BIT( pua->uart_reg.UARTRead, Ctrl, rxd_pres ) );*/
    spin_lock_bh( &pua->s_lock );
    while( IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0 ) & CTRL_READ_RXD_PRES )
    {
        it.data = IO_READ_8( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_DATA );
//      if( it.data > 0x10 )
//          PRINTKM(IO,(PKTD "%c\n", pua->device_index, it.data ));
//          printk( KERN_INFO "%c\n", it.data );
        _PIPE_WRITE( pua->read_pipe, it, writeOK );
        if( !writeOK )
        {
            PRINTKM( IO, ( PKTD " our serial read_pipe is full, new entries will be lost\n", pua->device_index ) );
            break;
        }
        ///\todo check error if pipe is full
        bytes_read++;
    }
    spin_unlock_bh( &pua->s_lock );
    //PRINTKM(IO,(PKTD " %s bytes_read %lu ch %lu\n", pua->device_index, __FUNCTION__, bytes_read, pua->channel ));
}


//----------------------------------------------------------------------------------
void jit_timer_read_serial( unsigned long arg )
//----------------------------------------------------------------------------------
{
    uart_object_t* pua = ( uart_object_t* )arg;
    read_serial( pua );
    pua->read_timer.expires = ( jiffies + pua->timeout_jiffies );
    add_timer( &pua->read_timer );
    //PRINTKM(IO,(PKTD " bytes_read %lu ch %lu\n", pua->device_index, bytes_read, pua->channel ));
}

//----------------------------------------------------------------------------------
void reset_fifo( uart_object_t* pua )
//----------------------------------------------------------------------------------
{
    u32 uart_ctrl;

    uart_ctrl = IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0 );
    uart_ctrl |= CTRL_WRITE_FIFO_RESET;
    IO_WRITE_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0, uart_ctrl );
    uart_ctrl &= ~CTRL_WRITE_FIFO_RESET;
    IO_WRITE_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0, uart_ctrl );
    /*_PIPE_RESET( pua->read_pipe );
    _WRITE_BIT( pua->uart_reg.UARTWrite, Ctrl, parity_mode, 0x2 );
    _WRITE_REG( pua->uart_reg.UARTWrite, IntEnable, 1 );*/
    wait_jiffies( 4 );
}

//----------------------------------------------------------------------------------
void reset_read_write_fifo( uart_object_t* pua )
//----------------------------------------------------------------------------------
{
    _PIPE_RESET( pua->read_pipe );
}

//----------------------------------------------------------------------------------
unsigned long getnumber_bytes_available( uart_object_t* pua )
//----------------------------------------------------------------------------------
{
    unsigned long items = _ITEMS_IN_PIPE( pua->read_pipe );
    //PRINTKM(IO,(PKTD "%s items %lu\n", 0, __FUNCTION__, items  ));
    return items;
}

//----------------------------------------------------------------------------------
void set_baudrate( uart_object_t* pua, unsigned long value )
//----------------------------------------------------------------------------------
{
    unsigned long b_rate;
    u32 uart_ctrl;
    switch( value )
    {
    default:
    case 9600:
        b_rate = 0;
        break;
    case 19200:
        b_rate = 1;
        break;
    case 38400:
        b_rate = 2;
        break;
    case 57600:
        b_rate = 3;
        break;
    case 115200:
        b_rate = 4;
        break;
    case 230400:
        b_rate = 5;
        break;
    case 460800:
        b_rate = 6;
        break;
    case 921600:
        b_rate = 7;
        break;
    }
    uart_ctrl = IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0 );
    uart_ctrl &= ~( 0x7 << UART_CTRL_BAUDRATE );
    uart_ctrl |= ( ( b_rate & 0x7 ) << UART_CTRL_BAUDRATE );
    IO_WRITE_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0, uart_ctrl );
    reset_fifo( pua );
    //PRINTKM(IO,(PKTD " br %lu, regval 0x%lx, ctrl p%p *ctrl 0x%x\n", pua->device_index, value, b_rate, pua->uart_reg.UARTRead, _READ_REG( pua->uart_reg.UARTRead, Ctrl ) ));
}

//----------------------------------------------------------------------------------
unsigned char create_serial( struct memory_space* mem_base, HYPERION_BASE_REGISTER_DEF* reg_def, uart_object_t* pua, unsigned long reg_index, unsigned int device_index )
//----------------------------------------------------------------------------------
{
    unsigned long tmp_data = 0;
    PRINTKM( IO, ( PKTD " %s pua %p index %d\n", 0, __FUNCTION__, pua, device_index ) );
    pua->device_index = device_index;
    pua->hyperion_base.base = mem_base->base;
    pua->hyperion_base.size = mem_base->size;
    pua->reg_def = reg_def;
    pua->register_index = reg_index;
    if( ( IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0 ) & CTRL_READ_UART_VERSION ) == CTRL_READ_UART_VERSION )
    {
        return FALSE;
    }
    spin_lock_init( &pua->s_lock );
    _PIPE_INITIALIZE( pua->read_pipe, READ_BUFFER_SIZE );
    IO_WRITE_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0, 0x8000 );
    tmp_data = IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_DATA );
    reset_fifo( pua );

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
    timer_setup( &pua->read_timer, pua->read_timer.function, 0 );
#else
    init_timer( &pua->read_timer );
    pua->read_timer.function = jit_timer_read_serial;
    pua->read_timer.data = ( unsigned long )pua;
#endif

    pua->timeout_jiffies = msecs_to_jiffies( DEF_TIMER_DELAY_MS );
    pua->read_timer.expires = ( jiffies + pua->timeout_jiffies );

    add_timer( &pua->read_timer );
    pua->init_done = TRUE;
    return pua->init_done;
}

//----------------------------------------------------------------------------------
void close_serial( uart_object_t* pua )
//----------------------------------------------------------------------------------
{
    if( pua->init_done )
    {
        del_timer( &pua->read_timer );
        IO_WRITE_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_POCL_INTR_ENABLE, 0 );
        _PIPE_DESTROY( pua->read_pipe );
        memset( pua, 0, sizeof( uart_object_t ) );
    }
}
