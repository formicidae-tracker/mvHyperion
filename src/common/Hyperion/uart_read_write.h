#ifndef uartreadwriteH
#define uartreadwriteH uartreadwriteH

#include <linux/timer.h>
#include "hyperion_base.h"
#include "pipe.h"

//----------------------------------------------------------------------------------
#define READ_BUFFER_SIZE (16*1024)
#define DEF_TIMER_DELAY_MS 12

typedef enum serial_error_codes
{
    secNoError = 0,
    secErrTimeout,
    secErrInvalidReference,
} enum_serial_error_codes;

//----------------------------------------------------------------------------------
typedef struct _uart_object
{
    unsigned int init_done;
    int register_index;
    unsigned int device_index;
    struct memory_space hyperion_base;
    HYPERION_BASE_REGISTER_DEF* reg_def;
    TPipe* read_pipe;
    struct timer_list read_timer;
    unsigned long timeout_jiffies;
    unsigned long channel;
    unsigned long read_write_status;
    spinlock_t s_lock;
} uart_object_t;

//----------------------------------------------------------------------------------
unsigned char create_serial( struct memory_space* mem_base, HYPERION_BASE_REGISTER_DEF* reg_def, uart_object_t* pua, unsigned long reg_index, unsigned int device_index );
void close_serial( uart_object_t* pua );
void reset_read_write_fifo( uart_object_t* pua );
unsigned long getnumber_bytes_available( uart_object_t* pua );
void set_baudrate( uart_object_t* pua, unsigned long baudrate );
void read_data_from_serial( uart_object_t* pua );
unsigned long read_serial_data( uart_object_t* pua, unsigned long timeoutmsec, unsigned char* data, unsigned long bufferlength );
unsigned long write_serial_data( uart_object_t* pua, unsigned long timeoutmsec, unsigned char* data, unsigned long bufferlength );
void read_serial( uart_object_t* pua );

#endif // uartreadwriteH
