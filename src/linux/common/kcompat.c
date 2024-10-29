#include <linux/autoconf.h>
#include <linux/version.h>
#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a) << 16) | ((b) << 8) | (c))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION (2,5,0)
// Use versioning if needed
#if defined(CONFIG_MODVERSIONS) && ! defined(MODVERSIONS)
#   define MODVERSIONS
#endif
#endif

#include <linux/mm.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION (2,4,12)
#include <linux/slab.h>
#else
#include <linux/malloc.h>
#endif
#include <linux/kmod.h>

#include "../kmod/kcompat.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION (2,1,0)
// nothing
#else

//------ *** only 2.0.x. KERNEL *** ---------------------------------------------------
long interruptible_sleep_on_timeout( struct wait_queue** p, long expire )
{
    unsigned long timeout ;

    current->timeout = expire ;
    interruptible_sleep_on( p ) ;
    timeout = current->timeout ;
    current->timeout = 0;
    return timeout ;
}

//------ *** only 2.0.x. KERNEL *** ---------------------------------------------------
int time_before( unsigned long a, unsigned long b )
{
    return( ( long )( ( a ) - ( b ) ) < 0L );
}

typedef struct
{
    int anyuserdata ;
}* PUserSpace ;

//------ *** only 2.0.x. KERNEL *** ---------------------------------------------------
unsigned long copy_to_user( void* to, const void* from, unsigned long n )
{
    memcpy_tofs( to, from, n );
    return 0;
}

//------ *** only 2.0.x. KERNEL *** ---------------------------------------------------
unsigned long copy_from_user( void* to, const void* from, unsigned long n )
{
    memcpy_fromfs( to, from, n );
    return 0;
}
#endif

