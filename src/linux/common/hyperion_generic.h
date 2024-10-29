#ifndef HyperionGenericH
#define HyperionGenericH HyperionGenericH

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
extern struct device* hyperion_class_device_create( void* device, dev_t dev_num );
#else
extern struct class_device* hyperion_class_device_create( void* device, dev_t dev_num );
#endif
extern void hyperion_class_device_destroy( dev_t dev_num );
extern int hyperion_get_device_number( void );
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
extern struct device* hyperion_serial_class_device_create( void* device, dev_t dev_num );
#else
extern struct class_device* hyperion_serial_class_device_create( void* device, dev_t dev_num );
#endif
extern void hyperion_serial_class_device_destroy( dev_t dev_num );


#endif //HyperionGenericH
