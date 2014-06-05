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
 * \file directdma.c
 * \brief Direct DMA transport module implementation.
 *
 * This file implements a simple direct, downlink DMA transport.
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
#include "directdma_internal.h"

/*!
 * \brief The basic DMA module.
 * \extends picoif_module
 */
struct directdma_module
{
    struct picoif_module   mod;            /*!< The generic module. */
};

struct directdma_ctx;

struct directdma_completion {
    struct list_head        head;
    struct directdma_ctx    *ctx;
    size_t                  nbytes;
    void                    *cookie;
};

/*!
 * \brief DMA channel parameters (one per DMA channel).
 */
struct directdma_ctx
{
    struct picoarray        *pa;            /*!< The picoArray running the
                                             *   transport. */
    struct pico_resource    *chan;          /*!< DMA channel */
    unsigned                channel_active; /*!< Boolean defining channel
                                             *   status */
    spinlock_t              lock;           /*!< DMA channel lock */
    struct scatterlist      *sg;            /*!< The next scatterlist to
                                             *   transfer so we can do true
                                             *   scatterlists rather than
                                             *   single entries. */
    size_t                  bytes_complete; /*!< The number of bytes
                                             *   transferred. */
    void                    *cookie;        /*!< The completion cookie. */
    void                    ( *callback )( size_t nbytes,
                                           void *cookie );
    struct list_head        completed;      /*!< The completed xfers. */
    struct tasklet_struct   complete_task;  /*!< The task to call completions.*/
};

static void
directdma_destructor( struct picoif_module *module );

static struct picoif_context *
directdma_create_trans_instance( struct picoif_module *module,
                                 const char *description,
                                 struct picoif_buf *params );

static void
directdma_close_trans_instance( struct picoif_module *module,
                                struct picoif_context *ctx );

static ssize_t
directdma_writesg( struct picoif_module *module,
                   struct picoif_context *ctx,
                   struct scatterlist *sgl,
                   unsigned n_ents,
                   void *cookie );

static int
directdma_can_write( struct picoif_module *module,
                     struct picoif_context *ctx )
{
    return -EOPNOTSUPP;
}

static ssize_t
directdma_write( struct picoif_module *module,
                 struct picoif_context *ctx,
                 struct picoif_buf *data,
                 size_t len )
{
    return -EOPNOTSUPP;
}

static int
directdma_can_read( struct picoif_module *module,
                    struct picoif_context *ctx )
{
    return -EOPNOTSUPP;
}

static ssize_t
directdma_read( struct picoif_module *module,
                struct picoif_context *ctx,
                struct picoif_buf *data,
                size_t len )
{
    return -EOPNOTSUPP;
}

/*! picoIf module operations for the DMA module. */
static struct picoif_module_ops directdma_ops = {
    .destructor             = directdma_destructor,
    .create_trans_instance  = directdma_create_trans_instance,
    .close_instance         = directdma_close_trans_instance,
    .writesg                = directdma_writesg,
    .write                  = directdma_write,
    .can_write              = directdma_can_write,
    .read                   = directdma_read,
    .can_read               = directdma_can_read,
};

/*! Transport methods for the DMA transport module */
static const char *directdma_tmethods[] = {
    "dl",
    NULL,
};

/*! The DMA transport */
static struct directdma_module directdma_mod ={
    .mod = {
        .name       = "directdma",
        .tmethods   = directdma_tmethods,
        .ops        = &directdma_ops,
    },
};

static void
directdma_complete( unsigned long param )
{
    struct directdma_ctx *channel = ( struct directdma_ctx * )param;
    struct directdma_completion *pos, *tmp;
    LIST_HEAD( completed );

    spin_lock_bh( &channel->lock );
    list_splice_init( &channel->completed, &completed );
    spin_unlock_bh( &channel->lock );

    list_for_each_entry_safe( pos, tmp, &completed, head )
    {
        list_del( &pos->head );
        channel->callback( pos->nbytes, pos->cookie );
        kfree( pos );
    }
}

static size_t
sg_length( struct scatterlist *sgl )
{
    struct scatterlist *pos = sgl;
    size_t nbytes = 0;

    while ( pos )
    {
        nbytes += pos->length;
        pos = sg_next( pos );
    }

    return nbytes;
}

/*!
 * Handler function for interrupts called by a completed DMA transfer.
 *
 * @param cookie The channel associated with the interrupt source.
 * @param errno The error number associated with the DMA transfer
 * @return Returns zero on success, negative on failure.
 */
static int
dma_handler( void *cookie,
             int errno )
{
    struct directdma_ctx *channel = cookie;
    int ret = 0;
    struct directdma_completion *comp;
    size_t nbytes;

    if ( errno )
    {
        PRINTD( COMPONENT_DMA, DBG_ERROR,"DMA transfer generated "
                "error %d (cookie=0x%p)", errno, cookie );
        channel->channel_active = 0;
        return errno;
    }

    spin_lock_bh( &channel->lock );

    nbytes = sg_length( channel->sg );
    PRINTD( COMPONENT_DMA, DBG_TRACE, "transfer of %zu bytes complete",
            nbytes );

    comp = kmalloc( sizeof( *comp ), GFP_ATOMIC );
    if ( comp )
    {
        PRINTD( COMPONENT_DMA, DBG_TRACE, "sg dma complete, schedule tasklet" );
        comp->nbytes = nbytes;
        comp->cookie = channel->cookie;
        list_add_tail( &comp->head, &channel->completed );
        tasklet_schedule( &channel->complete_task );
    }
    else
        ret = -ENOMEM;

    channel->channel_active = 0;

    spin_unlock_bh( &channel->lock );

    return ret;
}

/*!
 * Write to the DMA transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the write.
 * @param sgl The data to write.
 * @param n_ents The number of entries in the scatterlist.
 * @return Returns the number of bytes written on success, negative on
 * failure.
 */
static ssize_t
directdma_writesg( struct picoif_module *module,
                   struct picoif_context *ctx,
                   struct scatterlist *sgl,
                   unsigned n_ents,
                   void *cookie )
{
    struct directdma_ctx *channel = ctx->private_data;
    struct picoarray *pa = channel->pa;
    int ret = 0;

    /* Start a DMA transfer */
    spin_lock( &channel->lock );

    if ( !channel->channel_active )
    {
        int tmp = 0;

        channel->channel_active = 1;
        channel->sg             = sgl;
        channel->cookie         = cookie;
        channel->bytes_complete = 0;

        PRINTD( COMPONENT_DMA, DBG_TRACE, "start transfer of %zu bytes",
                channel->sg->length );

	tmp = pa->ops->dma_to_device( pa, channel->chan, sgl, n_ents,
				      dma_handler, channel );
        if ( tmp < 0)
            ret = tmp;
    }
    else
    {
        ret = -EBUSY;
    }

    spin_unlock( &channel->lock );

    return ret;
}

/*!
 * Destructor for the DMA module.
 *
 * @param module The module being destroyed.
 */
static void
directdma_destructor( struct picoif_module *module )
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
directdma_create_trans_instance( struct picoif_module *module,
                                 const char *description,
                                 struct picoif_buf *params )
{
    int ret = -ENOMEM;
    void *private_data = NULL;
    struct picoif_context *ctx = picoif_new_ctx( module, private_data );
    struct picoif_directdma_params gparams;
    struct picoarray *pa = NULL;
    struct directdma_ctx *channel;
    struct pico_resource *chan = NULL;

    ret = picoif_buf_copy_from( &gparams, params, 0,
                                sizeof( gparams ) );
    if ( ret )
        goto out;

    ret = -EINVAL;
    pa = picoif_get_device( gparams.dev_num );
    if ( !pa )
    {
        PRINTD( COMPONENT_DMA, DBG_WARN, "invalid device number: %u",
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
    channel = kmalloc( sizeof( *channel ), GFP_KERNEL );
    if (!channel)
        goto out;
    ctx->private_data = channel;

    spin_lock_init( &channel->lock );

    channel->channel_active = 0;
    channel->pa             = pa;
    channel->chan           = chan;
    channel->callback       = gparams.callback;
    tasklet_init( &channel->complete_task, directdma_complete,
                  ( unsigned long )channel );
    INIT_LIST_HEAD( &channel->completed );

    if ( strcmp( description, "directdma(dl)" ) )
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
directdma_close_trans_instance( struct picoif_module *mod,
                                struct picoif_context *ctx )
{
    struct directdma_ctx *channel = ctx->private_data;
    struct picoarray *pa = channel->pa;
    (void)pa->ops->dma_close( pa, channel->chan );

    pa->ops->put_resource( pa, channel->chan );

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
picoif_directdma_open( unsigned dev_num,
                       int dma_chan,
                       void ( *callback )( size_t nbytes,
                                           void *cookie ) )
{
    struct picoif_directdma_params dma_params = {
        .dev_num    = dev_num,
        .channel    = dma_chan,
        .callback   = callback,
    };
    struct picoif_buf buf = {
        .kbuf       = &dma_params,
        .is_user    = 0,
    };

    struct picoif_context *ctx =
        directdma_create_trans_instance( &directdma_mod.mod,
                                         "directdma(dl)", &buf );
    return ctx;
}
EXPORT_SYMBOL( picoif_directdma_open );

int
directdma_init( void )
{
    return picoif_register_module( &directdma_mod.mod );
}
