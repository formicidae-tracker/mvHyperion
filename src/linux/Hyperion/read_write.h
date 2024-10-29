#ifndef readwriteH
#define readwriteH readwriteH

#include <linux/poll.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0)
#    include <linux/uio.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,19)
irqreturn_t hyperion_interrupt( int irq, void* dev_id, struct pt_regs* fake );
#else
irqreturn_t hyperion_interrupt( int irq, void* dev_id );
#endif

void hyperion_do_tasklet( unsigned long index );
#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,19)
ssize_t async_io_read( struct kiocb* iocb, char __user* buffer,  size_t count, loff_t pos );
ssize_t async_io_write( struct kiocb* iocb, const char __user* buffer, size_t count, loff_t pos );
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
ssize_t async_io_read( struct kiocb* iocb, const struct iovec* iocv, unsigned long count, loff_t pos );
ssize_t async_io_write( struct kiocb* iocb, const struct iovec* iocv, unsigned long count, loff_t pos );
#else
ssize_t async_io_read( struct kiocb* iocb, struct iov_iter* iocv_iter );
ssize_t async_io_write( struct kiocb* iocb, struct iov_iter* iocv_iter );
#endif
int setup_read_write( struct hyperion_device* device );
void release_read_write( struct hyperion_device* device );
unsigned int hyperion_poll( struct file* filp, struct poll_table_struct* wait );
void start_transfer( struct io_object* ioobj, int index );
void start_transfer_dummy( struct io_object* ioobj, int index );
void test_interrupt( struct hyperion_device* device );
void abort_transfer( struct hyperion_device* device );
void release_mapping( struct hyperion_device* device );

#endif //readwriteH
