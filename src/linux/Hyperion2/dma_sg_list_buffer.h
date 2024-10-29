#ifndef DmaSGListBufferH
#define DmaSGListBufferH DmaSGListBufferH

int init_sg_buffer_list( struct hyperion* phyperion );
void release_sg_buffer_list( struct hyperion* phyperion );
struct dma_sg_list_entry* get_sg_list_buffer( struct hyperion_device* phyp_dev );
void free_sg_list_buffer( struct hyperion_device* phyp_dev, struct dma_sg_list_entry* dma_sg_list );
void dump_sg_list_buffer( struct hyperion_device* phyp_dev );

#endif //DmaSGListBufferH
