/*****************************************************************************
 * $picoChipHeaderSubst$
 *****************************************************************************/

/*
 * Copyright (c) 2009 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file dma.c
 * \brief DMA transport module implementation.
 *
 * This file implements Basic DMA transports.
 *
 * Two transport methods are provided:
 *   \li dma1(dl)
 *   \li dma1(ul)
 *
 * The "dl" transport is the main mechanism for loading data into the picoArray
 * using DMA channels.
 * The "ul" transport is for debugging purposesi only, as a transfer from the
 * picoArray to the host only occurs if the host requests the transfer.
 * In both methods the DMA engine is used to perform the transfer which calls
 * the an interrupt handler on exit. This handler will update the FIFO pointers
 * and wake up any sleeping processes.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/picochip/transports/dma.h>
#include <linux/picochip/picoif_ioctl.h>

#include "debug.h"
#include "picoarray.h"
#include "picoif_internal.h"
#include "picoif_module.h"
#include "dma_internal.h"
#include "dma_fifo_internal.h"

/*!
 * \brief The basic DMA module.
 * \extends picoif_module
 */
struct dma_module
{
    struct picoif_module   mod;            /*!< The generic module. */
};

/*!
 * \brief DMA channel parameters (one per DMA channel).
 */
static struct dma_channel
{
    struct picoarray *pa;             /*!< The picoArray running the
                                       *   transport. */
    struct picoif_context *reader;    /*!< The reader of the channel. */
    unsigned isDownlink;              /*!< Boolean defining DMA direction */
    struct pico_resource *chan;       /*!< DMA channel */
    unsigned channelActive;           /*!< Boolean defining channel status */
    unsigned xfer_size;               /*!< Size of transfer in bytes */
    unsigned max_transfer_size;       /*!< Max transfer size in bytes */
    struct kfifo *fifo;               /*!< DMA FIFO for this channel */
    spinlock_t lock;                  /*!< DMA channel lock */
    struct scatterlist sgl[2];        /*!< Transfer scatterlist. */
    int nents;                        /*!< Number of scatterlist entries. */
    struct tasklet_struct tasklet;    /*!< Tasklet for new xfers. */
}
dma_channel;

static void
dma_destructor( struct picoif_module *module );

static struct picoif_context *
dma_create_trans_instance( struct picoif_module *module,
                           const char *description,
                           struct picoif_buf *params );

static void
dma_close_trans_instance( struct picoif_module *module,
                          struct picoif_context *ctx );

static ssize_t
dma_write( struct picoif_module *module,
           struct picoif_context *ctx,
           struct picoif_buf *buf,
           size_t len );

static ssize_t
dma_read( struct picoif_module *module,
          struct picoif_context *ctx,
          struct picoif_buf *buf,
          size_t len );

static int
dma_can_write( struct picoif_module *module,
               struct picoif_context *ctx );

static int
dma_can_read( struct picoif_module *module,
              struct picoif_context *ctx );

static ssize_t
dma_writev( struct picoif_module *module,
            struct picoif_context *ctx,
            const struct iovec *vecs,
            unsigned nr_segs,
            int from_user );

/*! picoIf module operations for the DMA module. */
static struct picoif_module_ops dma_ops = {
    .destructor             = dma_destructor,
    .create_trans_instance  = dma_create_trans_instance,
    .close_instance         = dma_close_trans_instance,
    .write                  = dma_write,
    .read                   = dma_read,
    .can_read               = dma_can_read,
    .can_write              = dma_can_write,
    .writev                 = dma_writev,
};

/*! Transport methods for the DMA transport module */
static const char *dma_tmethods[] = {
    "dl",
    "ul",
};

/*! The DMA transport */
static struct dma_module dma_mod ={
    .mod = {
        .name       = "dma1",
        .tmethods   = dma_tmethods,
        .ops        = &dma_ops,
    },
};

/*!
 * Check if the transport can be written to.
 *
 * @param module The module responsible for the context.
 * @param ctx The context of the transport.
 * @return Returns 1 if the transport can be written, 0 otherwise
 * 0.
 */
static int
dma_can_write( struct picoif_module *module,
               struct picoif_context *ctx )
{
    struct dma_channel *channel = ctx->private_data;
    unsigned fifo_space = kfifo_avail( channel->fifo );
    int ret = 1;

    spin_lock_bh( &channel->lock );
    if ( !channel->isDownlink && !channel->channelActive )
    {
        ret = 1; /* Transfer size requests permitted in the UL */
        goto out;
    }

    if ( !fifo_space || channel->channelActive )
    {
        ret = 0; /* No space in FIFO or transfer is in progress. */
        goto out;
    }

out:
    spin_unlock_bh( &channel->lock );
    return ret;
}

/*!
 * Check if the transport can be read from.
 *
 * @param module The module responsible for the context.
 * @param ctx The context of the transport.
 * @return Returns 1 if the transport can be read, 0 otherwise.
 */
static int
dma_can_read( struct picoif_module *module,
              struct picoif_context *ctx )
{
    struct dma_channel *channel = ctx->private_data;
    if ( channel->isDownlink )
        return 0;  /* Cannot read from a downlink */

    return kfifo_len( channel->fifo ) ? 1 : 0;
}

static void
__dma_push_fifo( struct dma_channel *channel );

static void
dma_tasklet( unsigned long param )
{
    struct dma_channel *channel = ( struct dma_channel * )param;

    __dma_push_fifo( channel );
}

/*!
 * Handler function for interrupts called by a completed DMA transfer.
 *
 * @param cookie The channel associated with the interrupt source.
 * @param errno The error number associated with the DMA transfer
 * @return Returns zero on success, negative on failure.
 */
__must_check static int
dma_handler( void *cookie,
             int errno )
{
    struct dma_channel *channel = cookie;
    int ret = 0;

    BUG_ON( NULL == cookie );

    if ( errno )
    {
       PRINTD( COMPONENT_DMA, DBG_ERROR,"DMA transfer generated "
          "error %d (cookie=0x%p)", errno, cookie );
       channel->channelActive = 0;
       return errno;
    }

    channel->channelActive = 0;

    if ( channel->isDownlink )
    {
        PRINTD( COMPONENT_DMA, DBG_TRACE,"DL transfer of %d bytes completed",
            channel->xfer_size );

        /* Update the FIFO ptrs to indicate more data available */
        kfifo_dma_out_finish( channel->fifo, channel->xfer_size );
        dma_unmap_sg( NULL, channel->sgl, channel->nents, DMA_TO_DEVICE );
        wake_up_interruptible( &channel->reader->writeq );

        tasklet_schedule( &channel->tasklet );
    }
    else
    {
        PRINTD( COMPONENT_DMA, DBG_TRACE,"UL transfer of %d bytes completed",
            channel->xfer_size );

        /* Add data transferred to FIFO control parameters */
        kfifo_dma_in_finish( channel->fifo, channel->xfer_size );
        dma_unmap_sg( NULL, channel->sgl, channel->nents, DMA_FROM_DEVICE );
        wake_up_interruptible( &channel->reader->readq );
    }

    return ret;
}

static void
__dma_push_fifo( struct dma_channel *channel )
{
    struct picoarray *pa = channel->pa;
    size_t len;
    int ret;

    if ( channel->channelActive )
        return;

    len = kfifo_len( channel->fifo ) & ~0x3;
    if ( len > channel->max_transfer_size )
        len = channel->max_transfer_size;

    sg_init_table( channel->sgl, 2 );
    channel->nents = kfifo_dma_out_prepare( channel->fifo, channel->sgl,
                                            2, len );
    channel->nents = dma_map_sg( NULL, channel->sgl, channel->nents,
                                 DMA_TO_DEVICE );
    channel->xfer_size = len;

    if ( len )
    {
        PRINTD( COMPONENT_DMA, DBG_TRACE, "start DL transfer of %zu bytes",
                channel->xfer_size );
        channel->channelActive = 1;
        ret = pa->ops->dma_to_device( pa, channel->chan, channel->sgl,
                                      channel->nents, dma_handler,
                                      channel );
        if ( ret )
            PRINTD( COMPONENT_PC3XX, DBG_WARN,
                    "failed to start DMA to device" );
    }
}

static void
dma_push_fifo( struct dma_channel *channel )
{
    spin_lock_bh( &channel->lock );
    __dma_push_fifo( channel );
    spin_unlock_bh( &channel->lock );
}

/*!
 * Write to the DMA downlink transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the write.
 * @param buf The data to be written.
 * @param len The length of data to be written in bytes.
 * @return Returns the number of bytes written on success, negative on
 * failure.
 */
static ssize_t
dma_write_downlink( struct dma_channel *channel,
                    struct picoif_buf *buf,
                    size_t len )
{
    int ret=0;

    /* Add data to FIFO (assuming there is room) */
    ret = dma_fifo_put( channel->fifo, buf, len );
    if ( ret < 0 )
        goto out;

    if ( 0 == ret )
    {
        ret = -EAGAIN;
        goto out;
    }

    PRINTD( COMPONENT_DMA, DBG_TRACE, "write %u bytes (requested %u bytes)",
            ret, len );

    dma_push_fifo( channel );

out:
    return ret;
}

/*!
 * Write to the uplink DMA transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the write.
 * @param buf The data to be written.
 * @param len The length of data to be transfers in bytes.
 * @return Returns the size of the DMA transfer requested on success,
 * negative on failure.
 */
static ssize_t
dma_write_uplink( struct dma_channel *channel,
                  struct picoif_buf *buf,
                  size_t len )
{
    struct picoarray *pa = channel->pa;
    int ret=0;
    unsigned size = 0;

    if ( len != sizeof( uint32_t )  )
        return -EINVAL;

    if ( channel->channelActive )
        return -EAGAIN;

    spin_lock_bh( &channel->lock );

    ret = picoif_buf_copy_from( &size, buf, 0, sizeof( uint32_t ) );
    if ( ret )
        goto out;

    if ( channel->channelActive )
    {
        ret = -EAGAIN;
        goto out;
    }

    /* Start a DMA transfer */
    channel->xfer_size = kfifo_avail( channel->fifo );
    if ( size < channel->xfer_size )
       channel->xfer_size = size;

    /* Ensure that transfers requests are multiplies of 4 bytes */
    channel->xfer_size &= ~0x03ul;
    if ( channel->xfer_size > channel->max_transfer_size )
        channel->xfer_size = channel->max_transfer_size;

    if ( channel->xfer_size < size )
    {
        ret = -EAGAIN;
        goto out;
    }

    if ( channel->xfer_size )
    {
        sg_init_table( channel->sgl, 2 );
        channel->nents = kfifo_dma_in_prepare( channel->fifo, channel->sgl, 1,
                                               channel->xfer_size );
        channel->nents = dma_map_sg( NULL, channel->sgl, channel->nents,
                                     DMA_FROM_DEVICE );

        PRINTD( COMPONENT_DMA, DBG_TRACE, "request a transfer of %u bytes)",
                channel->xfer_size );

        ret = ( pa->ops->dma_from_device( pa, channel->chan, channel->sgl,
                                          channel->nents, dma_handler,
                                          channel ));
        if ( ret )
            PRINTD( COMPONENT_DMA, DBG_WARN,
                    "failed to start transfer from device" );
        if ( !ret )
            ret = channel->xfer_size;
    }

out:
    spin_unlock_bh( &channel->lock );

    return ret;
}

/*!
 * Write to the DMA transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the write.
 * @param buf The data to be written.
 * @param len The length of data to be written in bytes.
 * @return Returns the number of bytes written (downlink) or
 * the size of the DMA request (uplink) on success, negative on
 * failure.
 */
static ssize_t
dma_write( struct picoif_module *module,
           struct picoif_context *ctx,
           struct picoif_buf *buf,
           size_t len )
{
    struct dma_channel *channel = ctx->private_data;
    int ret = 0;

    if ( channel->isDownlink )
    {
        ret = dma_write_downlink( channel, buf, len );  
    }
    else
    {
        ret = dma_write_uplink( channel, buf, len );
    }

    return ret;
}

/*!
 * Vectored write to the DMA transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the write.
 * @param vecs The vectors of data to write into the transport.
 * @param nr_segs The number of entries in the IO vector.
 * @param from_user Boolean flag to indicate that the vectors point to
 * userspace buffers and must be copied across the address spaces.
 * @return Returns the number of bytes written into the transport on
 * success, negative on failure.
 * failure.
 */
static ssize_t
dma_writev( struct picoif_module *module,
            struct picoif_context *ctx,
            const struct iovec *vecs,
            unsigned nr_segs,
            int from_user )
{
    struct dma_channel *channel = ctx->private_data;
    ssize_t ret = 0;
    unsigned seg = 0;
    ssize_t len = 0;
    unsigned bytes_queued = 0;

    PRINTD( COMPONENT_DMA, DBG_TRACE, "received a vector write of %u segments",
         nr_segs);

    /* Check that the total length of the transfer is a multiple of 4 bytes */
    for ( seg = 0; seg < nr_segs; ++seg )
        len += vecs[seg].iov_len;

    for ( seg = 0; seg < nr_segs; ++seg )
    {
        struct picoif_buf buf = {
            .ubuf       = ( char __user * )vecs[ seg ].iov_base,
            .is_user    = 1,
        };

        ret = dma_fifo_put( channel->fifo, &buf, vecs[seg].iov_len );
        if ( ret < 0 )
            goto out;

        bytes_queued += ret;

        if ( ret < vecs[seg].iov_len )
            break;
    }

    ret = bytes_queued;
    if ( !ret )
        return -EAGAIN;

    PRINTD( COMPONENT_DMA, DBG_TRACE, "vector write %u bytes (requested %u bytes)",
              ret, len );

    dma_push_fifo( channel );

out:

    return ret;
}

/*!
 * Read from the DMA transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the read.
 * @param buf The buffer to write the data into.
 * @param len The number of bytes requested to read.
 * @return Returns the number of bytes read on success, negative on failure.
 */
static ssize_t
dma_read( struct picoif_module *module,
          struct picoif_context *ctx,
          struct picoif_buf *buf,
          size_t len )
{
    struct dma_channel *channel = ctx->private_data;
    struct picoarray *pa = channel->pa;
    int ret = 0;
    unsigned used_bytes=0;

    /* Determine how much data is already in the FIFO */
    used_bytes = kfifo_len( channel->fifo );

    spin_lock_bh( &channel->lock );
    /* Not enough data in the FIFO - request a transfer */
    if ( used_bytes > 3 && used_bytes < len && ( !channel->channelActive) )
    {
        len = ( len - used_bytes ) & ~0x3;
        if ( len > channel->max_transfer_size )
            len = channel->max_transfer_size;
        sg_init_table( channel->sgl, 2 );
        channel->nents = kfifo_dma_in_prepare( channel->fifo, channel->sgl, 1,
                                               len );
        channel->nents = dma_map_sg( NULL, channel->sgl, channel->nents,
                                     DMA_FROM_DEVICE );
        channel->xfer_size = len;

        if ( len )
        {
            channel->channelActive = 1;
            ret = pa->ops->dma_from_device( pa, channel->chan, channel->sgl,
                                            channel->nents, dma_handler,
                                            channel );
            if ( ret )
                PRINTD( COMPONENT_DMA, DBG_WARN,
                        "failed to start transfer from device" );
        }
    }

    /* Now return any bytes available */
    if ( !used_bytes )
    {
        spin_unlock_bh( &channel->lock );
        return -EAGAIN;
    }

    if ( used_bytes > len )
        used_bytes = len;

    ret = dma_fifo_get( channel->fifo, buf, used_bytes );

    PRINTD( COMPONENT_DMA, DBG_TRACE,
            "read %u bytes (requested %u bytes)", used_bytes, len );

    if ( !ret )
        ret = used_bytes;

    spin_unlock_bh( &channel->lock );
    return ret;
}

/*!
 * Destructor for the DMA module.
 *
 * @param module The module being destroyed.
 */
static void
dma_destructor( struct picoif_module *module )
{
}

/*!
 * Create a and start the DMA transport.
 *
 * @param module The module managing the transport.
 * @param description The type of transport to create.
 * @param params Extra parameters for the transport type.
 * @return Returns the transport context on success, or an ERR_PTR on failure.
 */
static struct picoif_context *
dma_create_trans_instance( struct picoif_module *module,
                           const char *description,
                           struct picoif_buf *params )
{
    int ret = -ENOMEM;
    void *private_data = NULL;
    struct picoif_context *ctx = picoif_new_ctx( module, private_data );
    struct picoif_dma_params gparams;
    struct picoarray *pa = NULL;
    struct dma_channel *channel;
    struct pico_resource *chan = NULL;

    ret = picoif_buf_copy_from( &gparams, params, 0,
                                sizeof( gparams ) );
    if ( ret )
        goto out;

    ret = -EINVAL;
    pa = picoif_get_device( gparams.dev_num );
    if ( !pa )
    {
        PRINTD( COMPONENT_GPR_INT, DBG_WARN, "invalid device number: %u",
                gparams.dev_num );
        goto out;
    }

    /* Check that the DMA channel is not already in use */
    ret = -EBUSY;
    chan = pa->ops->get_resource( pa, PICO_RES_DMA_CHANNEL,
                                             gparams.channel, 1 );
    if ( !chan )
    {
        PRINTD( COMPONENT_DMA, DBG_ERROR, "invalid DMA channel:%u",
                gparams.channel );
        goto out;
    }

   if ( !ctx )
       goto out;

    /* Allocate the new channel and initialise the data members. */
    ret = -ENOMEM;
    channel = kmalloc( sizeof( dma_channel ), GFP_KERNEL );
    if (!channel)
        goto out;
    ctx->private_data = channel;

    channel->fifo = dma_buf_cache_alloc( gparams.buf_size );
    if ( !channel->fifo )
        goto bad_fifo_alloc;

    spin_lock_init( &channel->lock );

    channel->reader = ctx;
    channel->channelActive = 0;
    channel->pa = pa;
    channel->chan = chan;
    channel->max_transfer_size = pa->max_dma_sz;
    tasklet_init( &channel->tasklet, dma_tasklet, ( unsigned long )channel );

    if ( !strcmp( description, "dma1(dl)" ) )
        channel->isDownlink = 1;
    else if ( !strcmp( description, "dma1(ul)" ) )
        channel->isDownlink = 0;
    else
    {
        PRINTD( COMPONENT_DMA, DBG_ERROR, "invalid tmethod \"%s\"",
                description );
        ret = -EINVAL;
        goto bad_description;
    }

    ret = pa->ops->dma_open( pa, chan );
    if ( ret )
    {
        PRINTD( COMPONENT_DMA, DBG_ERROR, "failed to open DMA channel" );
        goto bad_description;
    }

    ret = 0;
    goto out;

bad_description:
   dma_buf_cache_free( channel->fifo );

bad_fifo_alloc:
    kfree( channel );

out:
    if ( ret )
    {
        kfree( ctx );
        if ( chan )
            pa->ops->put_resource(  pa, chan );
    }

    return ret ? ERR_PTR( ret ) : ctx;
}

/*!
 * Close an existing transport instance.
 *
 * @param mod The module handling the transport.
 * @param ctx The context that is being closed.
 */
static void
dma_close_trans_instance( struct picoif_module *mod,
                          struct picoif_context *ctx )
{
    struct dma_channel *channel = ctx->private_data;
    struct picoarray *pa = channel->pa;
    (void)pa->ops->dma_close( pa, channel->chan );

    tasklet_disable( &channel->tasklet );
    tasklet_kill( &channel->tasklet );
    pa->ops->put_resource( pa, channel->chan );
    dma_buf_cache_free( channel->fifo );

    if ( channel )
    {
        kfree( channel );
        channel = NULL;
    }

    if ( ctx )
    {
        kfree( ctx );
        ctx = NULL;
    }
}

/* Kernel API and Public functions */
struct picoif_context *
picoif_dma_open_dl( unsigned dev_num,
                    int dma_chan,
                    size_t buf_size )
{
    struct picoif_dma_params dma_params = {
        .dev_num    = dev_num,
        .channel    = dma_chan,
        .buf_size   = buf_size,
    };
    struct picoif_buf buf = {
        .kbuf       = &dma_params,
        .is_user    = 0,
    };

    struct picoif_context *ctx =
        dma_create_trans_instance( &dma_mod.mod,
                                       "dma1(dl)", &buf );
    return ctx;
}
EXPORT_SYMBOL( picoif_dma_open_dl );

struct picoif_context *
picoif_dma_open_ul( unsigned dev_num,
                    int dma_chan,
                    size_t buf_size )
{
    struct picoif_dma_params dma_params = {
        .dev_num    = dev_num,
        .channel    = dma_chan,
        .buf_size   = buf_size,
    };
    struct picoif_buf buf = {
        .kbuf       = &dma_params,
        .is_user    = 0,
    };
    struct picoif_context *ctx =
        dma_create_trans_instance( &dma_mod.mod,
                                       "dma1(ul)", &buf );
    return ctx;
}
EXPORT_SYMBOL( picoif_dma_open_ul );

int
dma_init( void )
{
    return picoif_register_module( &dma_mod.mod );
}


