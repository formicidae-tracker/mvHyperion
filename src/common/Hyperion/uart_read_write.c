//serial communication with hyperion boards

#include "uart_read_write.h"
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
void switch_uart_local_echo( uart_object_t* pua, unsigned int disable )
//----------------------------------------------------------------------------------
{
    unsigned long uart_ctrl;

    uart_ctrl = IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_CONTROL );
    switch( disable )
    {
    case 0:
    default:
        {
            uart_ctrl &= ~( 1 << UART_CTRL_DISABLE_ECHO );
        }
        break;
    case 1:
        {
            uart_ctrl |= ( 1 << UART_CTRL_DISABLE_ECHO );
        }
        break;
    }
    IO_WRITE_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_CONTROL, uart_ctrl );
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
            //PRINTKM(IO,(PKTD "w %x %c \n", pua->device_index, data[data_written], data[data_written] ));
            IO_WRITE_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_DATA, data[data_written++] );
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

    //PRINTKM(IO,(PKTD " jiffies %d timeoutjiffies %d data available %d\n", pua->device_index, jiffies, pua->timeout_jiffies, _READ_BIT( pua->uart_reg.UARTRead, Ctrl, rxd_pres ) ));
    spin_lock_bh( &pua->s_lock );
    while( IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_CONTROL ) & CTRL_READ_RXD_PRES )
    {
        it.data = IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_DATA );
        //PRINTKM(IO,(PKTD r %x %c\n", pua->device_index, it.data, it.data ));
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
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
//----------------------------------------------------------------------------------
void jit_timer_read_serial( struct timer_list* t )
//----------------------------------------------------------------------------------
{
    uart_object_t* pua = from_timer( pua, t, read_timer );
#else
//----------------------------------------------------------------------------------
void jit_timer_read_serial( unsigned long arg )
//----------------------------------------------------------------------------------
{
    uart_object_t* pua = ( uart_object_t* )arg;
#endif
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
    //PRINTKM(IO,(PKTD " %s ctrl %x\n", pua->device_index, __FUNCTION__, uart_ctrl ));
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
    //PRINTKM(IO,("%s items %lu\n", __FUNCTION__, items  ));
    return items;
}

//----------------------------------------------------------------------------------
void set_baudrate( uart_object_t* pua, unsigned long value )
//----------------------------------------------------------------------------------
{
    unsigned long b_rate;
    u32 uart_ctrl;
    PRINTKM( IO, ( PKTD " %s br %lu\n", pua->device_index, __FUNCTION__, value ) );

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
    if( pua->init_done )
    {
        close_serial( pua );
    }
    pua->device_index = device_index;
    pua->hyperion_base.base = mem_base->base;
    pua->hyperion_base.size = mem_base->size;
    pua->reg_def = reg_def;
    pua->register_index = reg_index;
    PRINTKM( IO, ( " %s pua %p index %d base %p uart %p\n", __FUNCTION__, pua, device_index, mem_base->base, REG_POINTER( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_CONTROL ) ) );
    if( ( IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, 0 ) & CTRL_READ_UART_VERSION ) == CTRL_READ_UART_VERSION )
    {
        return FALSE;
    }
    spin_lock_init( &pua->s_lock );
    _PIPE_INITIALIZE( pua->read_pipe, READ_BUFFER_SIZE );
    IO_WRITE_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_CONTROL, ( UART_PARITY_DISABLE << UART_CTRL_PARITY )/*0x8000*/ | ( 1 << UART_CTRL_ENABLE_NTRISTATE ) | ( 1 << UART_CTRL_TX_SIGNAL_ENABLE ) );
    switch_uart_local_echo( pua, 1 );
//implement interrupt func
    tmp_data = IO_READ_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_UART_DATA );
    set_baudrate( pua, 0 );

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 15, 0 )
    timer_setup( &pua->read_timer, jit_timer_read_serial, 0 );
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
    PRINTKM( IO, ( " %s pua %p\n", __FUNCTION__, pua ) );
    if( pua->init_done )
    {
        del_timer( &pua->read_timer );
        IO_WRITE_32( pua->hyperion_base, pua->reg_def, pua->register_index, OFF_POCL_INTR_ENABLE, 0 );
        _PIPE_DESTROY( pua->read_pipe );
        memset( pua, 0, sizeof( uart_object_t ) );
    }
}
