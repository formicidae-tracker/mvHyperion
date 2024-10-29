/*
 * hyperionmain.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: drivermain.c,v 1.20 2011-02-25 13:05:49 ug Exp $
 *
 */
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the ter ms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "drivermain.h"
#include "hyperion_base.h"
#include "iocontrol.h"
#include "hyperion_generic.h"

#define NO_MORE_HYPERIONS 1
#if DEBUG
#   define IRQ_INITIALIZED BIT(31)
unsigned long debug = 0 ; // BASE_SYS_DEBUG | _flg(TM) | _flg(SLOT) | _flg(SEM) ;
#endif // #if DEBUG

unsigned long control = _Ctrl( POCL ) ;

static struct hyperion* hyperions[MAXHYPERIONS] = {NULL};
static unsigned int irq_map_hyperion[MAXHYPERIONS] = {0, 0, 0, 0,
                                                      0, 0, 0, 0,
                                                      0, 0, 0, 0,
                                                      0, 0, 0, 0
                                                     };

static const char intname[MAXHYPERIONS][12] =
{
    {"hyperion0"}, {"hyperion1"}, {"hyperion2"}, {"hyperion3"},
    {"hyperion4"}, {"hyperion5"}, {"hyperion6"}, {"hyperion7"},
    {"hyperion8"}, {"hyperion9"}, {"hyperion10"}, {"hyperion11"},
    {"hyperion12"}, {"hyperion13"}, {"hyperion14"}, {"hyperion15"}
};

// used for dynamic allocation of major device
static int major_dev_num = HYPERION_MAJOR;
static int minor_dev_num = 0;

#define HYPERION_MAJ_VERSION    2
#define HYPERION_MIN_VERSION    2
#define HYPERION_NUM_VERSION    (HYPERION_MAJ_VERSION*100+HYPERION_MIN_VERSION)
#define HYPERION_BUILD_NUMBER   0
// PCI driver registration
static struct pci_driver hyperion_pci_driver;

// forward declaration
static int init_pci( void );

// module parameters
#include <linux/moduleparam.h>


MODULE_AUTHOR( "MATRIX VISION GmbH <info@matrix-vision.de>" );
MODULE_DESCRIPTION( "MATRIX VISION mvHYPERION Framegrabber" );
MODULE_LICENSE( "GPL" );
#if DEBUG
module_param( debug, long, 0 );
#endif
module_param( control, long, 0 );
module_param( major_dev_num, int, 0 );

//-------------------------------------------------------------------------------------------
struct hyperion* get_hyperion( unsigned int index )
//-------------------------------------------------------------------------------------------
{
    if( index < MAXHYPERIONS )
    {
        return hyperions[index];
    }
    else
    {
        return NULL;
    }
}

//-------------------------------------------------------------------------------------------
static int hyperion_open( struct inode* inode, struct file* file )
//-------------------------------------------------------------------------------------------
{
    int result = 0;
    struct hyperion* phyperion;
    int minor_num, dev_index;

    minor_num = MINOR( inode->i_cdev->dev );
    dev_index = minor_num & ( MAXHYPERIONS - 1 );
    phyperion = hyperions[dev_index];
    phyperion->owner = ( void* )file;
    ++phyperion->users;

    PRINTKM( FILE, ( PKTD "open hyperion device %p minor %d dev_index %d users %d file %p\n", phyperion->number, phyperion, minor_num, dev_index, phyperion->users, file ) );
    INIT_LIST_HEAD( &phyperion->head_uaddr );
    if( file != NULL )
    {
        file->private_data = ( void* ) phyperion;
    }

    return result;
}

//-------------------------------------------------------------------------------------------
static int hyperion_close( struct inode* inode, struct file* file )
//-------------------------------------------------------------------------------------------
{
    struct hyperion* phyperion ;

    phyperion = container_of( inode->i_cdev, struct hyperion, cdev );
    PRINTKM( FILE, ( PKTD " %s users on device %d owner %p file %p\n", phyperion->number, __FUNCTION__,  phyperion->users, phyperion->owner, ( void* )file ) );
    --phyperion->users;
    if( phyperion->users == 0 )
    {
        hyperion_release_hw( phyperion );
    }
    return 0 ;
}

//-------------------------------------------------------------------------------------------
static int hyperion_mmap( struct file* file, struct vm_area_struct* vma )
//-------------------------------------------------------------------------------------------
{
    struct hyperion* phyperion = ( struct hyperion* )file->private_data;
    unsigned long start;
    int result = 0;

    start = ( unsigned long ) vma->vm_start;
    if( start & ~PAGE_MASK )
    {
        PRINTKM( MEM, ( PKTD "mmap: invalid start=0x%08lx\n", phyperion->number, start ) );
        return -EFAULT;
    }
    return result;
}

//-------------------------------------------------------------------------------------------
static void* space_map( unsigned long physical_address, unsigned long size )
//-------------------------------------------------------------------------------------------
{
    void* ptr;
    unsigned long base_page   = physical_address & PAGE_MASK;
    unsigned long base_offset = physical_address & ~PAGE_MASK;

    ptr = ( void* )( base_offset | ( unsigned long ) ioremap( base_page, size ) );
    PRINTKM( MEM, ( "map space 0x%lx -> $%p\n", physical_address, ptr ) );
    return ptr;
}

//-------------------------------------------------------------------------------------------
static void space_unmap( void** ptr )
//-------------------------------------------------------------------------------------------
{
    if( *ptr != NULL )
    {
        iounmap( *ptr );
        *ptr = NULL;
    }
}

//-------------------------------------------------------------------------------------------
static void space_get_memory_resource( struct pci_dev* pci_device, spinlock_t lock,
                                       unsigned long index, unsigned long* address, unsigned long* size )
//-------------------------------------------------------------------------------------------
{
    *address = pci_resource_start( pci_device, index );
    *address &= ~15UL;
    *size = pci_resource_len( pci_device, index );

    PRINTKM( MEM, ( "get pcibase ix=%ld, 0x%lx..+0x%lx\n", index, *address, *size ) );
}

//-------------------------------------------------------------------------------------------
static struct file_operations hyperion_fops =
//-------------------------------------------------------------------------------------------
{
    .owner      = THIS_MODULE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
    .aio_read   = async_io_read,
    .aio_write  = async_io_write,
#else
    .read_iter   = async_io_read,
    .write_iter  = async_io_write,
#endif
#if HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl = hyperion_ioctl,
#else
    .ioctl      = hyperion_ioctl,
#endif
    .mmap       = hyperion_mmap,
    .open       = hyperion_open,
    .release    = hyperion_close,
    .poll       = hyperion_poll,
};

//-------------------------------------------------------------------------------------------
// sysfs entries
//-------------------------------------------------------------------------------------------
// TODO for Kernel 2.6.20 ??: Much of the sysfs-related code has been changed to use struct device in place of struct class_device.
//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_board_type_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_board_type_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    /* return the serial number of the hyperion device */
    struct hyperion* phyperion;

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    phyperion = ( struct hyperion* )dev_get_drvdata( class_dev );
#else
    phyperion = ( struct hyperion* )class_get_devdata( class_dev );
#endif
    return snprintf( buf, PAGE_SIZE, "%s\n", phyperion->eeprom_content.type );
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( board_type, S_IRUGO, hyperion_board_type_show, NULL );
#else
CLASS_DEVICE_ATTR( board_type, S_IRUGO, hyperion_board_type_show, NULL );
#endif


//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_serial_number_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_serial_number_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    /* return the serial number of the hyperion device */
    struct hyperion* phyperion;

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    phyperion = ( struct hyperion* )dev_get_drvdata( class_dev );
#else
    phyperion = ( struct hyperion* )class_get_devdata( class_dev );
#endif
    return snprintf( buf, PAGE_SIZE, "%s\n", phyperion->eeprom_content.serial );
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( serialno, S_IRUGO, hyperion_serial_number_show, NULL );
#else
CLASS_DEVICE_ATTR( serialno, S_IRUGO, hyperion_serial_number_show, NULL );
#endif

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_firmware_version_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_firmware_version_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    /* return the serial number of the hyperion device */
    struct hyperion* phyperion;

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    phyperion = ( struct hyperion* )dev_get_drvdata( class_dev );
#else
    phyperion = ( struct hyperion* )class_get_devdata( class_dev );
#endif
    return snprintf( buf, PAGE_SIZE, "%d\n", phyperion->firmware_version );
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( firmware_version, S_IRUGO, hyperion_firmware_version_show, NULL );
#else
CLASS_DEVICE_ATTR( firmware_version, S_IRUGO, hyperion_firmware_version_show, NULL );
#endif

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_pci_device_id_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_pci_device_id_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    /* return the serial number of the hyperion device */
    struct hyperion* phyperion;

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    phyperion = ( struct hyperion* )dev_get_drvdata( class_dev );
#else
    phyperion = ( struct hyperion* )class_get_devdata( class_dev );
#endif
    return snprintf( buf, PAGE_SIZE, "%d\n", phyperion->vd_id.deviceId );
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( pci_device_id, S_IRUGO, hyperion_pci_device_id_show, NULL );
#else
CLASS_DEVICE_ATTR( pci_device_id, S_IRUGO, hyperion_pci_device_id_show, NULL );
#endif

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
static ssize_t hyperion_device_version_show( struct device* class_dev, struct device_attribute* aatr, char* buf )
#else
static ssize_t hyperion_device_version_show( struct class_device* class_dev, char* buf )
#endif
//-------------------------------------------------------------------------------------------
{
    /* return the serial number of the hyperion device */
    struct hyperion* phyperion;

#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    phyperion = ( struct hyperion* )dev_get_drvdata( class_dev );
#else
    phyperion = ( struct hyperion* )class_get_devdata( class_dev );
#endif
    return snprintf( buf, PAGE_SIZE, "%s\n", phyperion->eeprom_content.revision );
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
DEVICE_ATTR( device_version, S_IRUGO, hyperion_device_version_show, NULL );
#else
CLASS_DEVICE_ATTR( pci_device_id, S_IRUGO, hyperion_device_version_show, NULL );
#endif

//-------------------------------------------------------------------------------------------
static void do_create_class_files( struct hyperion* phyperion, dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    struct device* hyperion_class_member;
#else
    struct class_device* hyperion_class_member;
#endif
    int err = 0;

    hyperion_class_member = hyperion_class_device_create( ( void* )phyperion, dev_num );
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    if( ( err = device_create_file( hyperion_class_member, &dev_attr_serialno ) ) < 0 )
#else
    if( ( err = class_device_create_file( hyperion_class_member, &class_device_attr_serialno ) ) < 0 )
#endif
    {
        printk( PKET  "Error %d creating hyperion device%d class file: class_device_attr_number_devices", err, phyperion->number );
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    if( ( err = device_create_file( hyperion_class_member, &dev_attr_board_type ) ) < 0 )
#else
    if( ( err = class_device_create_file( hyperion_class_member, &class_device_attr_board_type ) ) < 0 )
#endif
    {
        printk( PKET  "Error %d creating hyperion device%d class file: class_device_attr_number_devices", err, phyperion->number );
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    if( ( err = device_create_file( hyperion_class_member, &dev_attr_firmware_version ) ) < 0 )
#else
    if( ( err = class_device_create_file( hyperion_class_member, &class_device_attr_firmware_version ) ) < 0 )
#endif
    {
        printk( PKET  "Error %d creating hyperion device%d class file: class_device_attr_number_devices", err, phyperion->number );
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    if( ( err = device_create_file( hyperion_class_member, &dev_attr_pci_device_id ) ) < 0 )
#else
    if( ( err = class_device_create_file( hyperion_class_member, &class_device_attr_pci_device_id ) ) < 0 )
#endif
    {
        printk( PKET  "Error %d creating hyperion device%d class file: class_device_attr_pci_device_id", err, phyperion->number );
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    if( ( err = device_create_file( hyperion_class_member, &dev_attr_device_version ) ) < 0 )
#else
    if( ( err = class_device_create_file( hyperion_class_member, &class_device_attr_device_version ) ) < 0 )
#endif
    {
        printk( PKET  "Error %d creating hyperion device%d class file: class_device_attr_device_version", err, phyperion->number );
    }
    /* add any new class files here */
}

//-------------------------------------------------------------------------------------------
static void do_create_class_files_serial( struct hyperion* phyperion, dev_t dev_num )
//-------------------------------------------------------------------------------------------
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
    struct device* class_member_serial;
#else
    struct class_device* class_member_serial;
#endif
    class_member_serial = hyperion_serial_class_device_create( ( void* )phyperion, dev_num );
    /* add any new class files here */
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
static int init_cell_dma_attr( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
#ifdef CONFIG_PPC_OF
    struct device_node* np;
    init_dma_attrs( &phyperion->attrs );
    dma_set_attr( DMA_ATTR_WEAK_ORDERING, &phyperion->attrs );
    np = of_find_compatible_node( NULL, NULL, "IBM,CBEA" );
    if( !np )
    {
        return 0;
    }
    of_node_put( np );

    np = of_find_compatible_node( NULL, NULL, "IBM,CBPLUS-1.0" );
    if( !np )
    {
        return 0;
    }
    of_node_put( np );

    PRINTKM( MOD, ( PKTD " %s set weak_ordering attributes\n", phyperion->number, __FUNCTION__ ) );
    phyperion->pdma_attrs = &phyperion->attrs;
    return 1;
#endif
#endif
    return 0;
}

//-------------------------------------------------------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 3, 4, 0 )
static int hyperion_init_one( struct pci_dev* pdev, const struct pci_device_id* ent )
#else
static int __devinit hyperion_init_one( struct pci_dev* pdev, const struct pci_device_id* ent )
#endif
//-------------------------------------------------------------------------------------------
{
    int result, i, err;
    int number = 0;
    dev_t dev_num_serial[MAX_PARALLEL_TRANSFER] = {( dev_t )0, ( dev_t )0}, dev_num = ( dev_t )0;
    struct hyperion* phyperion;

    number = hyperion_get_device_number();
    if( number >= MAXHYPERIONS )
    {
        return -EFAULT;
    }
    dev_num = MKDEV( major_dev_num, minor_dev_num + number );

    if( ( phyperion = ( struct hyperion* ) kzalloc( sizeof( struct hyperion ), GFP_KERNEL ) ) == NULL )
    {
        return -ENOMEM;
    }

    phyperion->pdev = pdev;
    cdev_init( &phyperion->cdev, &hyperion_fops );
    phyperion->cdev.owner = THIS_MODULE;
    phyperion->cdev.ops = &hyperion_fops;
    err = cdev_add( &phyperion->cdev, dev_num, 1 );
    /* Fail gracefully if need be */
    if( err )
    {
        printk( PKET  "Error %d adding hyperion device%d", err, number );
        goto err2_out;
    }

    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        // register serial communication
        dev_num_serial[i] = MKDEV( major_dev_num, minor_dev_num + number + ( ( i + 1 ) * MINOR_SERIAL_PORT ) );
        cdev_init( &phyperion->cdev_serial[i], &hyperion_fops );
        phyperion->cdev_serial[i].owner = THIS_MODULE;
        phyperion->cdev_serial[i].ops = &hyperion_fops;
        err = cdev_add( &phyperion->cdev_serial[i], dev_num_serial[i], 1 );
        if( err )
        {
            printk( PKET  "Error %d adding hyperion_serial port %d", err, i );
            goto err2_out;
        }
    }

    if( pci_enable_device( pdev ) )
    {
        printk( PKET "hyperion %d: Cannot enable device at 0x%02x:0x%02x!\n", number, phyperion->bus, PCI_SLOT( pdev->devfn ) );
        goto err2_out;
    }

    hyperions[number] = phyperion;
    phyperion->number = number;
    phyperion->bus = pdev->bus->number;
    phyperion->devfunc = pdev->devfn;
    phyperion->owner = NULL;
    phyperion->users = 0;

    spin_lock_init( &phyperion->s_lock );
    /* get IRQ from PCI device - this also works with APIC */
    phyperion->irqlin = pdev->irq;

    space_get_memory_resource( pdev, phyperion->s_lock, 0, &phyperion->memory_base[0].physical_address, &phyperion->memory_base[0].size );
    phyperion->memory_base[0].base = space_map( phyperion->memory_base[0].physical_address, phyperion->memory_base[0].size );
    PRINTKM( MOD, ( PKTD "hyperion resources got: register p%p addr 0x%lx size 0x%lx irq 0x%x\n",
                    phyperion->number, phyperion->memory_base[0].base, phyperion->memory_base[0].physical_address, phyperion->memory_base[0].size, phyperion->irqlin ) );

    space_get_memory_resource( pdev, phyperion->s_lock, 2, &phyperion->memory_base[2].physical_address, &phyperion->memory_base[2].size );
    phyperion->memory_base[2].base = space_map( phyperion->memory_base[2].physical_address, phyperion->memory_base[2].size );
    PRINTKM( MOD, ( PKTD "hyperion resources got: register p%p addr 0x%lx size 0x%lx irq 0x%x\n",
                    phyperion->number, phyperion->memory_base[2].base, phyperion->memory_base[2].physical_address, phyperion->memory_base[2].size, phyperion->irqlin ) );
    pci_set_master( pdev );
    pci_read_config_word( pdev, PCI_VENDOR_ID, &phyperion->vd_id.vendorId );
    pci_read_config_word( pdev, PCI_DEVICE_ID, &phyperion->vd_id.deviceId );

    if( ( irq_map_hyperion[phyperion->number] ) == 0 ) /* If we don't already have this IRQ for this board */
    {
        INIT_LIST_HEAD( &phyperion->head_uaddr );
        result = hyperion_board_init( phyperion, control );
        if( result < 0 )
        {
            printk( "hyperion_board_init() returns error\n" );
            goto err2_out;
        }
#ifdef CONFIG_PCI_MSI
        if( pci_find_capability( phyperion->pdev, PCI_CAP_ID_MSIX ) )
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 )
            result = pci_enable_msix_range( phyperion->pdev, phyperion->msi_x_entry, HYPERION_MSI_X_MAX_VECTORS, HYPERION_MSI_X_MAX_VECTORS );
#else
            result = pci_enable_msix( phyperion->pdev, phyperion->msi_x_entry, HYPERION_MSI_X_MAX_VECTORS );
#endif
        }
        else
        {
            PRINTKM( MOD, ( PKTD "device is not capable of msi-x\n", phyperion->number ) );
            result = -1;
        }

        if( result == 0 )
        {
            PRINTKM( MOD, ( PKTD "device is using msi-x\n", phyperion->number ) );
            phyperion->flags |= HYPERION_FLAG_MSIX;
        }
        else
        {
            if( pci_find_capability( phyperion->pdev, PCI_CAP_ID_MSI ) )
            {
                result = pci_enable_msi( phyperion->pdev );
            }
            else
            {
                PRINTKM( MOD, ( PKTD "device is not capable of msi\n", phyperion->number ) );
                result = -1;
            }

            if( !result )
            {
                PRINTKM( MOD, ( PKTD "device is using msi\n", phyperion->number ) );
                phyperion->flags |= HYPERION_FLAG_MSI;
            }
            else
            {
                struct pci_bus* bus;

                PRINTKM( MOD, ( PKTD "device is using int-x interrupts\n", phyperion->number ) );
                PRINTKM( MOD, ( PKTD "hyperion_drv->no_msi = %d\n", phyperion->number, phyperion->pdev->no_msi ) );
                for( bus = phyperion->pdev->bus; bus; bus = bus->parent )
                    if( bus->bus_flags & PCI_BUS_FLAGS_NO_MSI )
                    {
                        PRINTKM( MOD, ( PKTD "pci_enable_msi: bus cannot handle MSI\n", phyperion->number ) );
                    }
            }
        }
#endif /* CONFIG_PCI_MSI */
#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,19)
        if( ( ( phyperion->flags & HYPERION_FLAG_MSIX )  == 0 ) && !request_irq( phyperion->pdev->irq, hyperion_interrupt, SA_SHIRQ, intname[phyperion->number], phyperion ) )
#else
        if( ( ( phyperion->flags & HYPERION_FLAG_MSIX )  == 0 ) && !request_irq( phyperion->pdev->irq, hyperion_interrupt, IRQF_SHARED, intname[phyperion->number], phyperion ) )
#endif
        {
            hyperion_enable_interrupt( phyperion );
            phyperion->flags |= HYPERION_FLAG_IRQ;
            irq_map_hyperion[phyperion->number] = IRQ_INITIALIZED | phyperion->irqlin;
            PRINTKM( MOD, ( PKTD "%s() irq_map_hyperion 0x%x\n", phyperion->number, __FUNCTION__, irq_map_hyperion[phyperion->number] ) );
            phyperion->hyperion_tasklet.next = NULL;
            phyperion->hyperion_tasklet.state = 0;
            phyperion->hyperion_tasklet.count.counter =  0;
            phyperion->hyperion_tasklet.data = phyperion->number;
            phyperion->hyperion_tasklet.func = hyperion_do_tasklet;
        }
        else
        {
            printk( PKTD "request_irq ( %d ) * failed\n", phyperion->number, phyperion->irqlin );
            goto err_out;
        }

        /* This logic deliberately only makes one request_irq() call for
        * each physical IRQ needed on each board. There is no need for more.
        */
    }
    /* create class device */
    hyperion_i2c_receive_data( phyperion, EEPROM_ADDR, 0, EEPROM_SIZE, ( unsigned char* )&phyperion->eeprom_content );
    phyperion->firmware_version = hyperion_get_firmware_version( phyperion );
    printk( KERN_INFO  "%s boardtype %s\n", __FUNCTION__, phyperion->eeprom_content.type );
    printk( KERN_INFO  "%s serial %s\n", __FUNCTION__, phyperion->eeprom_content.serial );
    printk( KERN_INFO  "%s hyperion_firmware_version_show() firmwareversion %d\n", __FUNCTION__, phyperion->firmware_version );
    do_create_class_files( phyperion, dev_num );
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        do_create_class_files_serial( phyperion, dev_num_serial[i] );
    }

    phyperion->pdma_attrs = NULL;
    init_cell_dma_attr( phyperion );
    phyperion->flags |= HYPERION_FLAG_INITIALIZED ;
    return 0;

err_out:
    pci_disable_device( pdev );
    space_unmap( ( void* )phyperion->memory_base[0].base );

err2_out:
    release_mem_region( pci_resource_start( pdev, 0 ), pci_resource_len( pdev, 0 ) );
    kfree( phyperion );
    hyperions[number] = NULL;
    printk( KERN_WARNING "mvHYPERION device %d *NOT* ok\n", phyperion->number );
    return -ENODEV;
}

//-------------------------------------------------------------------------------------------
static int hyperion_remove_device( struct hyperion* phyperion )
//-------------------------------------------------------------------------------------------
{
    struct pci_dev* device_found;
    void* ptr;
    int i;

#if LINUX_VERSION_CODE >= KERNEL_VERSION (2,6,26)
    device_found = pci_get_slot( phyperion->pdev->bus, phyperion->devfunc );
#else
    device_found = pci_find_slot( phyperion->bus, phyperion->devfunc );
#endif

    if( device_found == NULL )
    {
        PRINTKM( MOD, ( PKTD "hyperion_number = %d: no more matching mvHYPERIONS found\n", phyperion->number, phyperion->number ) );
        return NO_MORE_HYPERIONS;
    }
    hyperion_board_close( phyperion );
    PRINTKM( MOD, ( PKTD "hyperion_remove_device Device vendorId 0x%x found at bus=%d, devfun=0x%02x\n", phyperion->number, PCI_VENDOR_ID_MATRIX_VISION, phyperion->bus, phyperion->devfunc ) );
    pci_disable_device( device_found );
    /* remove class device */
    hyperion_class_device_destroy( MKDEV( major_dev_num, minor_dev_num + phyperion->number ) );
    for( i = 0; i < MAX_PARALLEL_TRANSFER; i++ )
    {
        hyperion_serial_class_device_destroy( MKDEV( major_dev_num, minor_dev_num + phyperion->number + ( ( i + 1 )*MINOR_SERIAL_PORT ) ) );
    }
    if( irq_map_hyperion[phyperion->number] )
    {
        irq_map_hyperion[phyperion->number] = 0;
        free_irq( phyperion->pdev->irq, phyperion );
#ifdef CONFIG_PCI_MSI
        if( phyperion->flags & HYPERION_FLAG_MSI )
        {
            pci_disable_msi( phyperion->pdev );
            phyperion->flags &= ~HYPERION_FLAG_MSI;
        }
        if( phyperion->flags & HYPERION_FLAG_MSIX )
        {
            for( i = 0; i < HYPERION_MSI_X_MAX_VECTORS; i++ )
            {
                if( phyperion->msi_x_entry[i].vector )
                {
                    free_irq( phyperion->msi_x_entry[i].vector, phyperion );
                }
            }
            pci_disable_msix( phyperion->pdev );
            phyperion->flags &= ~HYPERION_FLAG_MSIX;
        }
#endif /* CONFIG_PCI_MSI */
    }
    ptr = ( void* )phyperion->memory_base[0].base;
    space_unmap( &ptr );
    ptr = ( void* )phyperion->memory_base[2].base;
    space_unmap( &ptr );
    return 0;
}


//-------------------------------------------------------------------------------------------
static int __init hyperion_init( void )
//-------------------------------------------------------------------------------------------
{
    int result = 0, device_number;
    dev_t dev_num;
    unsigned int firstminor = 0;

    /*printk ( KERN_INFO "MATRIX Vision mvHYPERION2 - %d.%d.%d (%lx)\n",
             HYPERION_MAJ_VERSION, HYPERION_MIN_VERSION, HYPERION_BUILD_NUMBER, debug );*/

    if( major_dev_num )
    {
        dev_num = MKDEV( major_dev_num, minor_dev_num );
        result = register_chrdev_region( dev_num, MAXHYPERIONS, "hyperion2" );
    }
    else
    {
        result = alloc_chrdev_region( &dev_num, firstminor, MAX_CONT_DEVICES, "hyperion2" );
        major_dev_num = MAJOR( dev_num );
        minor_dev_num = MINOR( dev_num );
    }

    if( result < 0 )
    {
        PRINTKM( MOD, ( "unable to get major %d for hyperion devices\n", major_dev_num ) );
        return result;
    }

    for( device_number = 0; device_number < MAXHYPERIONS; device_number++ )
    {
        hyperions[device_number] = NULL;
    }

    if( hyperion_get_device_number() < MAXHYPERIONS )
    {
        init_pci();
    }

    PRINTKM( MOD, ( "hyperion_init2() --> ok, %d Hyperions found.\n", hyperion_get_device_number() ) );
    return 0;
}

//-------------------------------------------------------------------------------------------
static void __exit hyperion_exit( void )
//-------------------------------------------------------------------------------------------
{
    int index;
    dev_t first = MKDEV( major_dev_num, 0 );

    PRINTKM( MOD, ( "hyperion_exit2()\n" ) );
    unregister_chrdev_region( first, MAXHYPERIONS );

    for( index = 0; index < MAXHYPERIONS; index++ )
    {
        struct hyperion* phyperion = hyperions[index];
        hyperions[index] = NULL;
        if( phyperion )
        {
            hyperion_remove_device( phyperion );
            kfree( phyperion );
        }
    }

    pci_unregister_driver( &hyperion_pci_driver );
    PRINTKM( MOD, ( "hyperion_exit2() -->\n" ) );
}


//-------------------------------------------------------------------------------------------
static struct pci_device_id hyperion_pci_tbl[] =
{
    { PCI_VENDOR_ID_MATRIX_VISION, PCI_DEVICE_ID_HYPERION_CL4E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { PCI_VENDOR_ID_MATRIX_VISION, PCI_DEVICE_ID_HYPERION_HDSDI_4X, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { 0, }
};

MODULE_DEVICE_TABLE( pci, hyperion_pci_tbl );

//-------------------------------------------------------------------------------------------
int __init init_pci( void )
//-------------------------------------------------------------------------------------------
{
    memset( &hyperion_pci_driver, 0, sizeof( hyperion_pci_driver ) );
    hyperion_pci_driver.name = MODULE_NAME;
    hyperion_pci_driver.id_table = hyperion_pci_tbl;
    hyperion_pci_driver.probe = hyperion_init_one;

    return pci_register_driver( &hyperion_pci_driver );
}

module_init( hyperion_init );
module_exit( hyperion_exit );
