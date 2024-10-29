#ifndef UserBufferH
#define UserBufferH UserBufferH

//-------------------------------------------------------------------------------------------
struct user_buffer_descriptor
//-------------------------------------------------------------------------------------------
{
    char __user* puser_buffer;
    size_t count;
    struct scatterlist* sg;
    int nr_pages;
    struct list_head list;
};

struct user_buffer_descriptor* get_user_buffer_descriptor( struct hyperion* phyperion, char __user* puser_buffer, size_t count );
void free_all_user_buffer_descriptors( struct hyperion* phyperion );
void free_user_buffer_descriptor( struct hyperion* phyperion, char __user* puser_buffer, size_t count );
void free_user_buffer( struct hyperion* phyperion, struct user_buffer_descriptor* puser_buffer_descr );
void add_user_buffer_descriptor_to_tail( struct hyperion* phyperion, struct user_buffer_descriptor* puser_buffer_descr );
void remove_user_buffer_descriptor_from_list( struct hyperion* phyperion, struct user_buffer_descriptor* puser_buffer_descr );


#endif //UserBufferH
