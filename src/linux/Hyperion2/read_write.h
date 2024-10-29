#ifndef readwriteH
#define readwriteH readwriteH

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0)
#    include <linux/uio.h>
#endif

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

unsigned int hyperion_poll( struct file* filp, struct poll_table_struct* wait );
void complete_request( void* ioobj, unsigned char status );
int sgl_unmap_user_pages( struct scatterlist* sgl, const unsigned int nr_pages, int dirtied );
#endif //readwriteH
