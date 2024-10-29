#ifndef readwritedirectH
#define readwritedirectH readwritedirectH

int setup_dmatransfer_object_direct( struct hyperion_device* device, u_long i );
irqreturn_t hyperion_interrupt_direct( int irq, void* dev_id, struct pt_regs* fake );
void hyperion_do_tasklet_direct( unsigned long index );
void setup_transfer_and_start_direct( struct io_object* ioobj, int index );
void release_read_write_direct( struct hyperion_device* device );
void prepare_scatter_gather_list_direct( void* pioobj, u32 nbytes_next_xfer, u32* pnext_xfer, u32 i_scatter_gather );
void abort_transfer_direct( struct hyperion_device* device );

#endif //readwritedirectH
