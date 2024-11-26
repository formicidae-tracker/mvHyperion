/*
 * read_write.c  -- MATRIX VISION Frame Grabber Driver for mvHyperion series.
 *
 *   Copyright (c) 2006 MATRIX VISION GmbH (info@matrix-vision.de)
 *
 $Id: read_write.c,v 1.16 2010-06-23 14:52:23 ug Exp $
 *
 */
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "drivermain.h"
#include "hyperion_base.h"

//-------------------------------------------------------------------------------------------
void
complete_request( void *prequest, unsigned char status )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_request_packet *phyperion_request_packet
        = (struct hyperion_request_packet *)prequest;
    if( phyperion_request_packet )
    {
        struct hyperion *phyperion
            = (struct hyperion *)phyperion_request_packet->private;
        // PRINTKM(DMA,(PKTD "%s ioobj %p\n", phyperion->number, __FUNCTION__,
        // phyperion_request_packet ));
        hyperion_release_dma( phyperion_request_packet );
        free_user_buffer( phyperion,
                          &phyperion_request_packet->user_buffer_descr );
        free_user_buffer( phyperion,
                          &phyperion_request_packet->trailer_buffer_descr );
#if LINUX_VERSION_CODE < KERNEL_VERSION( 4, 0, 0 )
        aio_complete( phyperion_request_packet->iocb,
                      phyperion_request_packet->count, 0 );
#elif LINUX_VERSION_CODE < KERNEL_VERSION( 5, 16, 0 )
        phyperion_request_packet->iocb->ki_complete(
            phyperion_request_packet->iocb, phyperion_request_packet->count,
            0 );
#else
        phyperion_request_packet->iocb->ki_complete(
            phyperion_request_packet->iocb, phyperion_request_packet->count );
#endif
        kfree( (void *)phyperion_request_packet );
    }
}

/*
//-------------------------------------------------------------------------------------------
static int cancel_async_read_write( struct kiocb *iocb, struct io_event *event )
//-------------------------------------------------------------------------------------------
{
    struct hyperion_request_packet *phyperion_request_packet = (struct hyperion_request_packet *)iocb->private;
    struct dma_transfer_object *dto = (struct dma_transfer_object *)phyperion_request_packet->dma_transfer_object;
    long result = 0, user_result = 0;

    //if( iocb->privatedata->transfering )
    //  stop tranfer
    cancel_iocb( dto->rw_queue, iocb );
    aio_complete( iocb, result, user_result );
    return 0;
}
*/

//-------------------------------------------------------------------------------------------
int sgl_get_max_pages( UINT_PTR uaddr, size_t count )
//-------------------------------------------------------------------------------------------
{
    UINT_PTR end = ( uaddr + count + PAGE_SIZE - 1 ) >> PAGE_SHIFT;
    UINT_PTR start = uaddr >> PAGE_SHIFT;
    const int max_pages = ( int )( end - start );
    PRINTKM( MEM, ( "%s(uaddr=%p, count=0x%zx): max_pages=%u\n", __FUNCTION__, ( void* )( ( UINT_PTR )uaddr ), count, max_pages ) );
    return max_pages;
}

//-------------------------------------------------------------------------------------------
// The following functions may be useful for a larger audience.
// copied from streaming tape device driver st.c
int
sgl_map_user_pages( struct scatterlist *sgl, const unsigned int max_pages,
                    UINT_PTR uaddr, size_t count, int rw )
//-------------------------------------------------------------------------------------------
{
    const int nr_pages = sgl_get_max_pages( uaddr, count );
    int res, i, j;
    struct page **pages;

    PRINTKM( MEM, ( "sgl_map_user_pages() sgl=p%p, max_pages=%u, uaddr=%p, "
                    "count=0x%zx, nr_pages=%u\n",
                    sgl, max_pages, (void *)( (UINT_PTR)uaddr ), count,
                    nr_pages ) );
    /* User attempted Overflow! */
    if( ( uaddr + count ) < uaddr )
    {
        PRINTKM( FILE, ( "sgl_map_user_pages() (uaddr + count) < uaddr \n" ) );
        return -EINVAL;
    }

    /* Too big */
    if( nr_pages > max_pages )
    {
        PRINTKM( FILE, ( "sgl_map_user_pages() nr_pages > max_pages\n" ) );
        return -ENOMEM;
    }

    /* Hmm? */
    if( count == 0 )
    {
        PRINTKM( FILE, ( "sgl_map_user_pages() count == 0\n" ) );
        return 0;
    }

    if( ( pages = vmalloc( max_pages * sizeof( *pages ) ) ) == NULL )
    {
        PRINTKM( FILE, ( "pages == NULL vmalloc returns NULL ( size 0x%zx )\n",
                         ( max_pages * sizeof( *pages ) ) ) );
        return -ENOMEM;
    }

/* Try to fault in all of the necessary pages */
// PRINTKM(MEM,( "sgl_map_user_pages() down_read(&current->mm->mmap_sem)\n" ));
#ifdef MMAP_LOCK_INITIALIZER
    mmap_read_lock( current->mm );
#else
    down_read( &current->mm->mmap_sem );
#endif
    /* rw==READ means read from drive, write into memory area */
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 6, 5, 0 )
    {
        int locked = 1;
        res = get_user_pages_remote( current->mm, uaddr, nr_pages,
                                     rw == READ ? FOLL_WRITE : 0, pages,
                                     &locked );
        if( locked )
        {
            mmap_read_unlock( current->mm );
        }
    }
#elif LINUX_VERSION_CODE >= KERNEL_VERSION( 5, 18, 0 )
    {
        int locked = 1;
        res = get_user_pages_remote( uaddr, nr_pages,
                                     rw == READ ? FOLL_WRITE : 0, pages, NULL,
                                     &locked );
        if( locked )
        {
            mmap_read_unlock( current->mm );
        }
    }
#elif LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 9, 0 )
    {
        int locked = 1;
        res = get_user_pages_locked(
            uaddr, nr_pages, rw == READ ? FOLL_WRITE : 0, pages, &locked );
        if( locked )
        {
#ifdef MMAP_LOCK_INITIALIZER
            mmap_read_unlock( current->mm );
#else
            up_read( &current->mm->mmap_sem );
#endif
        }
    }
#else
    res = get_user_pages( current, current->mm, uaddr, nr_pages, rw == READ, 0,
                          pages, NULL );
    up_read( &current->mm->mmap_sem );
#endif
    // PRINTKM(MEM,( "get_user_pages[_locked]() returns %d nr_pages %d \n",
    // res, nr_pages));

    /* Errors and no page mapped should return here */
    if( res < nr_pages )
    {
        goto out_unmap;
    }

    for( i = 0; i < nr_pages; i++ )
    {
        // FIXME: flush superflous for rw==READ, probably wrong function for
        // rw==WRITE
        flush_dcache_page( pages[i] );
    }

    /* Populate the scatter/gather list */
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 24 )
    sg_init_table( sgl, nr_pages );
    sg_set_page( &sgl[0], pages[0], 0, ( uaddr & ~PAGE_MASK ) );
    sgl[0].length = PAGE_SIZE - sgl[0].offset;
#else
    sgl[0].page = pages[0];
    sgl[0].offset = uaddr & ~PAGE_MASK;
    sgl[0].dma_address = 0;
    sgl[0].length = PAGE_SIZE - sgl[0].offset;
#endif
    /*
    #if !defined(CONFIG_HIGHMEM64G) && (BITS_PER_LONG == 32)
        PRINTKM(MEM,( "sgl_map_user_pages() page %p __pa 0x%x off 0x%x len
    0x%x\n", sg_page( &sgl[0] ), sgl[0].dma_address, sgl[0].offset,
    sgl[0].length)); #else PRINTKM(MEM,( "sgl_map_user_pages() page %p __pa
    0x%llx off 0x%x len 0x%x\n", sg_page( &sgl[0] ), sgl[0].dma_address,
    sgl[0].offset, sgl[0].length)); #endif
    */
    if( nr_pages > 1 )
    {
        sgl[0].length = PAGE_SIZE - sgl[0].offset;
        count -= sgl[0].length;
        for( i = 1; i < nr_pages; i++ )
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 24 )
            sg_set_page( &sgl[i], pages[i],
                         ( count < PAGE_SIZE ? count : PAGE_SIZE ), 0 );
#else
            sgl[i].offset = 0;
            sgl[i].page = pages[i];
            sgl[i].length = count < PAGE_SIZE ? count : PAGE_SIZE;
            sgl[i].dma_address = 0;
#endif
            count -= PAGE_SIZE;

            /*
            #if !defined(CONFIG_HIGHMEM64G) && (BITS_PER_LONG == 32)
                PRINTKM(MEM,( "sgl_map_user_pages() page[%d] %p __pa 0x%x off
            0x%x len 0x%x\n", i, sg_page( &sgl[i] ), sgl[i].dma_address,
            sgl[i].offset, sgl[i].length )); #else PRINTKM(MEM,(
            "sgl_map_user_pages() page[%d] %p __pa 0x%llx off 0x%x len 0x%x\n",
            i, sg_page( &sgl[i] ), sgl[i].dma_address, sgl[i].offset,
            sgl[i].length )); #endif
            */
        }
    }
    else
    {
        sgl[0].length = count;
    }

    vfree( pages );
    return nr_pages;

out_unmap:
    PRINTKM( FILE,
             ( "error: get_user_pages[_locked]() returns %d nr_pages %d\n",
               res, nr_pages ) );
    if( res > 0 )
    {
        for( j = 0; j < res; j++ )
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 6, 0 )
            put_page( pages[j] );
#else
            page_cache_release( pages[j] );
#endif
        }
        res = 0;
    }
    vfree( pages );
    return res;
}

//-------------------------------------------------------------------------------------------
// And unmap them...
// copied from streaming tape device driver st.c
// TODO remove function calls from tasklet routine because of kunmap and call them from IOCTL_READRESULTPACKET
int sgl_unmap_user_pages( struct scatterlist* sgl, const unsigned int nr_pages, int dirtied )
//-------------------------------------------------------------------------------------------
{
    int i;
    struct page* page;

    for( i = 0; i < nr_pages; i++ )
    {
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 24 )
        page = sg_page( &sgl[i] );
#else
        page = sgl[i].page;
#endif
        //PRINTKM(MEM,( "sgl_unmap_user_pages() page[%d] %p, setdirty %d\n", i, page, !PageReserved(page)));
        if( ! PageReserved( page ) )
        {
            SetPageDirty( page );
        }
        /* FIXME: cache flush missing for rw==READ
         * FIXME: call the correct reference counting function
         */
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 6, 0 )
        put_page( page );
#else
        page_cache_release( page );
#endif
    }
    return 0;
}

//-------------------------------------------------------------------------------------------
static int setup_buffering( struct kiocb* iocb, char __user* puser_buffer, size_t count, int is_read, struct user_buffer_descriptor* puser_buffer_descr )
//-------------------------------------------------------------------------------------------
{
    struct hyperion* phyperion = ( struct hyperion* )iocb->ki_filp->private_data;
    int nr_pages;
    unsigned short use_sg;

    PRINTKM( MEM, ( PKTD " %s\n", phyperion->number, __FUNCTION__ ) );
    if( puser_buffer_descr != NULL )
    {
        memset( puser_buffer_descr, 0, sizeof( struct user_buffer_descriptor ) );
        INIT_LIST_HEAD( &puser_buffer_descr->list );
        use_sg = sgl_get_max_pages( ( UINT_PTR )puser_buffer, count );
        puser_buffer_descr->sg = vmalloc( use_sg * sizeof( struct scatterlist ) );
        if( puser_buffer_descr->sg == NULL )
        {
            PRINTKM( MEM, ( PKTD " %s vmalloc for sglist failed use_sg %d sizeof sg %lx\n", phyperion->number, __FUNCTION__, use_sg, ( long unsigned int )sizeof( struct scatterlist ) ) );
            return -ENOMEM;
        }
        puser_buffer_descr->puser_buffer = puser_buffer;
        puser_buffer_descr->count = count;
        PRINTKM( MEM, ( PKTD " >%s, now fill sg list: sg list elements %d, puser_buffer %p, size %zu\n", phyperion->number, __FUNCTION__, use_sg, ( void* )puser_buffer, count ) );
        nr_pages = sgl_map_user_pages( puser_buffer_descr->sg, use_sg, ( UINT_PTR )puser_buffer, count, ( is_read ? READ : WRITE ) );
        PRINTKM( MEM, ( PKTD " %s sgl_map_user_pages(): nr_pages %d phyperion_request_packet->sg p %p\n", phyperion->number, __FUNCTION__, nr_pages, puser_buffer_descr->sg ) );
        if( nr_pages > 0 )
        {
            int dma_map_result = nr_pages, map_result;
#if LINUX_VERSION_CODE >= KERNEL_VERSION( 4, 8, 0 )
            map_result = dma_map_sg_attrs( &phyperion->pdev->dev, puser_buffer_descr->sg, nr_pages, PCI_DMA_FROMDEVICE, ( long unsigned int )phyperion->pdma_attrs );
#elif LINUX_VERSION_CODE >= KERNEL_VERSION( 2, 6, 26 )
            map_result = dma_map_sg_attrs( &phyperion->pdev->dev, puser_buffer_descr->sg, nr_pages, PCI_DMA_FROMDEVICE, ( struct dma_attrs* )phyperion->pdma_attrs );
#else
            map_result = dma_map_sg( &phyperion->pdev->dev, puser_buffer_descr->sg, nr_pages, PCI_DMA_FROMDEVICE );
#endif
            if( map_result <= 0 )
            {
                printk( " %s %d dma_map_sg() failed res %d ubuf %p count %x\n", __FUNCTION__, __LINE__, map_result, puser_buffer, ( unsigned int )count );
                dma_map_result = -ENOMEM;
            }
            if( dma_map_result <= 0 )
            {
                printk( " %s %d dma_map_sg() failed result %d\n", __FUNCTION__, __LINE__, dma_map_result );
                goto error_out_setup_buffering_generic;
            }
            puser_buffer_descr->nr_pages = nr_pages;
            return nr_pages;
        }
        goto error_out_setup_buffering_generic;
    }
    else
    {
        return 0;
    }

error_out_setup_buffering_generic:
    vfree( puser_buffer_descr->sg );
    puser_buffer_descr->sg = NULL;
    return 0;
}

//-------------------------------------------------------------------------------------------
static ssize_t async_read_write( struct kiocb* iocb, char __user* user_buffer, size_t count, loff_t pos, int is_read )
//-------------------------------------------------------------------------------------------
{
    struct hyperion* phyperion = ( struct hyperion* )iocb->ki_filp->private_data;
    struct hyperion_request_packet* phyperion_request_packet = NULL;
    HyperionRequestBuffer request_buffer;
    int bytes_to_copy = 0;
    ssize_t result = -EINVAL;
    unsigned long copy_result = 0;

    PRINTKM( FILE, ( PKTD " %s( %lu ubuffer %p count %d is_read %d is_synch_io %d )\n", phyperion->number, __FUNCTION__, jiffies, user_buffer, ( int )count, is_read, is_sync_kiocb( iocb ) ) );
    phyperion_request_packet = kmalloc( sizeof( struct hyperion_request_packet ), GFP_KERNEL );
    if( phyperion_request_packet == NULL )
    {
        printk( " %s allocating object struct hyperion_request_packet failed \n", __FUNCTION__ );
        return -ENOMEM;
    }
    memset( phyperion_request_packet, 0, sizeof( struct hyperion_request_packet ) );
    bytes_to_copy = count > sizeof( HyperionRequestBuffer ) ? sizeof( HyperionRequestBuffer ) : count;
    copy_from_user( ( void* )&request_buffer, ( void* )user_buffer, bytes_to_copy );
    PRINTKM( MEM, ( PKTD " leader=%p, size=%lu,  payload=%p, size=%llu,  trailer=%p, size=%lu\n", phyperion->number,
                    ( void* )( ( UINT_PTR )request_buffer.leaderBufferPtr ), request_buffer.leaderBufferSize,
                    ( void* )( ( UINT_PTR )request_buffer.userModePayloadBufferPtr ), request_buffer.userModePayloadBufferSize,
                    ( void* )( ( UINT_PTR )request_buffer.trailerBufferPtr ), request_buffer.trailerBufferSize ) );
    PRINTKM( MEM, ( PKTD " %s copy_result = %lx\n", phyperion->number, __FUNCTION__, copy_result ) );
    result = setup_buffering( iocb, ( char* )( ( UINT_PTR )request_buffer.userModePayloadBufferPtr ), request_buffer.userModePayloadBufferSize, is_read, &phyperion_request_packet->user_buffer_descr );
    if( result <= 0 )
    {
        PRINTKM( FILE, ( PKTD "setup_buffering() failed result %zd\n", phyperion->number, result ) );
        result = -EFAULT;
        goto err_out;
    }
    result = setup_buffering( iocb, ( char* )( ( UINT_PTR )request_buffer.trailerBufferPtr ), request_buffer.trailerBufferSize, is_read, &phyperion_request_packet->trailer_buffer_descr );
    if( result <= 0 )
    {
        PRINTKM( FILE, ( PKTD "setup_buffering() failed result %zd\n", phyperion->number, result ) );
        result = -EFAULT;
        goto err_out;
    }
    iocb->private = ( void* )phyperion_request_packet;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
    iocb->ki_cancel = NULL;
#else
    kiocb_set_cancel_fn( iocb, NULL );
#endif
    phyperion_request_packet->private = ( void* )phyperion;
    phyperion_request_packet->iocb = iocb;
    phyperion_request_packet->user_buffer = user_buffer;
    phyperion_request_packet->count = count;
    phyperion_request_packet->pos = pos;
    phyperion_request_packet->start_jiffies = jiffies;
    memcpy( ( void* )&phyperion_request_packet->request_buffer, ( void* )&request_buffer, sizeof( HyperionRequestBuffer ) );
    result = hyperion_async_read( phyperion, phyperion_request_packet );
    if( result < 0 )
    {
        PRINTKM( FILE, ( PKTD "%s hyperion_read() failed result %zd\n", phyperion->number, __FUNCTION__, result ) );
        result = -EFAULT;
        goto err_out;
    }
    return -EIOCBQUEUED;

err_out:
    if( phyperion_request_packet->request_parameter != NULL )
    {
        kfree( phyperion_request_packet->request_parameter );
        phyperion_request_packet->request_parameter = NULL;
    }
    if( phyperion_request_packet->user_buffer_descr.nr_pages > 0 )
    {
        free_user_buffer( phyperion, &phyperion_request_packet->user_buffer_descr );
    }
    if( phyperion_request_packet->trailer_buffer_descr.nr_pages > 0 )
    {
        free_user_buffer( phyperion, &phyperion_request_packet->trailer_buffer_descr );
    }
    kfree( ( void* )phyperion_request_packet );
    PRINTKM( FILE, ( PKTD "%s() failed result %zd\n", phyperion->number, __FUNCTION__, result ) );
    return result;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,19)
//-------------------------------------------------------------------------------------------
ssize_t async_io_read( struct kiocb* iocb, char __user* buffer,  size_t count, loff_t pos )
//-------------------------------------------------------------------------------------------
{
    return async_read_write( iocb, buffer, count, pos, TRUE );
}

//-------------------------------------------------------------------------------------------
ssize_t async_io_write( struct kiocb* iocb, const char __user* buffer, size_t count, loff_t pos )
//-------------------------------------------------------------------------------------------
{
    return 0;
}
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
//-------------------------------------------------------------------------------------------
ssize_t async_io_read( struct kiocb* iocb, const struct iovec* iocv, unsigned long count, loff_t pos )
//-------------------------------------------------------------------------------------------
{
    return async_read_write( iocb, ( char __user* )iocv->iov_base, iocv->iov_len, pos, TRUE );
}

//-------------------------------------------------------------------------------------------
ssize_t async_io_write( struct kiocb* iocb, const struct iovec* iocv, unsigned long count, loff_t pos )
//-------------------------------------------------------------------------------------------
{
    return 0;
}
#else
//-------------------------------------------------------------------------------------------
ssize_t async_io_read( struct kiocb* iocb, struct iov_iter* iocv_iter )
//-------------------------------------------------------------------------------------------
{
    return async_read_write( iocb, ( char __user* )iocv_iter->iov->iov_base, iocv_iter->iov->iov_len, iocv_iter->iov_offset, TRUE );
}

//-------------------------------------------------------------------------------------------
ssize_t async_io_write( struct kiocb* iocb, struct iov_iter* iocv_iter )
//-------------------------------------------------------------------------------------------
{
    return 0;
}
#endif

//-------------------------------------------------------------------------------------------
unsigned int hyperion_poll( struct file* filp, struct poll_table_struct* wait )
//-------------------------------------------------------------------------------------------
{
    return 0;
}
