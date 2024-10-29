#include <linux/version.h>
#include <linux/fs.h>

#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a) << 16) | ((b) << 8) | (c))
#endif

// Use versioning if needed
#if defined(CONFIG_MODVERSIONS) && ! defined(MODVERSIONS)
#   define MODVERSIONS
#endif

#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/module.h>
#include "hyperion_generic.h"

//-------------------------------------------------------------------------------------------
struct device_map
//-------------------------------------------------------------------------------------------
{
    int index;
    int used;
    dev_t dev_num;
};


#define MAX_DEVICE 16
static struct device_map device_map[MAX_DEVICE];

static struct file_operations fops =
{
    .owner = THIS_MODULE,
};

static struct class* hyperion_class = NULL;
//-------------------------------------------------------------------------------------------
int hyperion_get_device_number( void )
//-------------------------------------------------------------------------------------------
{
    int i;
    for( i = 0; i < MAX_DEVICE; i++ )
    {
        if( device_map[i].used == 0 )
        {
            return device_map[i].index;
        }
    }
    return -EFAULT;
}
EXPORT_SYMBOL( hyperion_get_device_number );

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_number_devices_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_number_devices_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    int num = hyperion_get_device_number();
    /* return the total number of hyperion devices found */
    //printk( "%s --> hyperion_number %d\n", __FUNCTION__, num );
    return snprintf( buf, PAGE_SIZE, "%d\n", num );
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( number_devices, S_IRUGO, hyperion_number_devices_show, NULL );
#else
CLASS_DEVICE_ATTR( number_devices, S_IRUGO, hyperion_number_devices_show, NULL );
#endif

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
struct device* hyperion_class_device_create( void* device, dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
    struct device* class_member;
#else
struct class_device* hyperion_class_device_create( void* device, dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
    struct class_device* class_member;
#endif
    int err = 0, device_index = hyperion_get_device_number();

    if( device_index < 0 )
    {
        printk( "%s could not locate free device_map\n", __FUNCTION__ );
        return NULL;
    }
    device_map[device_index].used = 1;
    device_map[device_index].dev_num = dev_num;
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 27 )
    class_member = device_create( hyperion_class, NULL, dev_num, NULL, "hyperion-cl%d", device_index );
#else
    class_member = device_create( hyperion_class, NULL, dev_num, "hyperion-cl%d", device_index );
#endif
    dev_set_drvdata( class_member, ( void* )device );
    if( ( err = device_create_file( class_member, &dev_attr_number_devices ) ) < 0 )
    {
        printk( "Error %d creating hyperion device%d class file: class_device_attr_number_devices", err, device_index );
    }
#else
    class_member = class_device_create( hyperion_class, NULL, dev_num, NULL, "hyperion-cl%d", device_index );
    class_set_devdata( class_member, ( void* )device );
    if( ( err = class_device_create_file( class_member, &class_device_attr_number_devices ) ) < 0 )
    {
        printk( "Error %d creating hyperion device%d class file: class_device_attr_number_devices", err, device_index );
    }
#endif
    return class_member;
}
EXPORT_SYMBOL( hyperion_class_device_create );

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
struct device* hyperion_serial_class_device_create( void* device, dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
    struct device* class_member;
#else
struct class_device* hyperion_serial_class_device_create( void* device, dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
    struct class_device* class_member;
#endif
    unsigned int minor_num;
    if( !hyperion_class )
    {
        return NULL;
    }
    minor_num = MINOR( dev_num );
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 27 )
    class_member = device_create( hyperion_class, NULL, dev_num, NULL, "hyperion-cl%d", minor_num );
#else
    class_member = device_create( hyperion_class, NULL, dev_num, "hyperion-cl%d", minor_num );
#endif
    dev_set_drvdata( class_member, ( void* )device );
#else
    class_member = class_device_create( hyperion_class, NULL, dev_num, NULL, "hyperion-cl%d", minor_num );
    class_set_devdata( class_member, ( void* )device );
#endif
    return class_member;
}
EXPORT_SYMBOL( hyperion_serial_class_device_create );

//-------------------------------------------------------------------------------------------
void hyperion_class_device_destroy( dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
    int i;
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    device_destroy( hyperion_class, dev_num );
#else
    class_device_destroy( hyperion_class, dev_num );
#endif
    for( i = 0; i < MAX_DEVICE; i++ )
    {
        device_map[i].used = 0;
        if( device_map[i].dev_num == dev_num && device_map[i].used )
        {
            device_map[i].used = 0;
            break;
        }
    }
}
EXPORT_SYMBOL( hyperion_class_device_destroy );

//-------------------------------------------------------------------------------------------
void hyperion_serial_class_device_destroy( dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    device_destroy( hyperion_class, dev_num );
#else
    class_device_destroy( hyperion_class, dev_num );
#endif
}
EXPORT_SYMBOL( hyperion_serial_class_device_destroy );

//-------------------------------------------------------------------------------------------
static int __init hyperion_generic_init( void )
//-------------------------------------------------------------------------------------------
{
    int i, result = 0;
    //printk( " %s\n", __FUNCTION__ );
    if( register_chrdev( 240, "hyperion_generic", &fops ) == 0 )
    {
        /* create sysfs class for hyperion */
        hyperion_class = class_create( "hyperion" );
        if( IS_ERR( hyperion_class ) && PTR_ERR( hyperion_class ) != -EEXIST )
        {
            /* tidy up after error */
            result = PTR_ERR( hyperion_class );
            printk( " %s error %d\n", __FUNCTION__, result );
            class_destroy( hyperion_class );
            unregister_chrdev( 240, "hyperion_generic" );
            return result;
        }

        for( i = 0; i < MAX_DEVICE; i++ )
        {
            device_map[i].index = i;
            device_map[i].used = 0;
        }

        return 0;
    };
    return -EIO;
}

//-------------------------------------------------------------------------------------------
static void __exit hyperion_generic_exit( void )
//-------------------------------------------------------------------------------------------
{
    //printk( " %s\n", __FUNCTION__ );
    unregister_chrdev( 240, "hyperion_generic" );
    class_destroy( hyperion_class );
}

module_init( hyperion_generic_init );
module_exit( hyperion_generic_exit );
MODULE_LICENSE( "GPL" );
