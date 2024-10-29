#ifndef devicefuncH
#define devicefuncH devicefuncH

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,0,0)
#    include <linux/vmalloc.h>
#endif

#include "hyperion.h"
void initialize_lut( struct hyperion_device* device, u_long lut_offset );
void set_mux_data( struct hyperion_device* device,  struct mux_controller_sequence* mux_seq, u_long inputchannel );
void set_mux_data_64( struct hyperion_device* device, struct mux_controller_sequence* mux_seq, u_long inputchannel );

#endif //devicefuncH
