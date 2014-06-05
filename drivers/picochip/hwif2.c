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
 * \file hwif2.c
 * \brief HwIF2 transport module implementation.
 *
 * This file implements the HwIF2 DMA transport.
 *
 * One transport method is provided:
 *   \li hwif2(ul)
 *
 * This transport creates one context per device which will permit a maximum of
 * eight DMA channels to be configured assuming that the picoArray device can
 * support this number. Each DMA channel requires a interrupt number between 0
 * and 7 to be assigned which must match the interrupt generated in the picoArray
 * interface code. Two GPRs are required per context for handling the handshaking
 * between picoArray and host.
 *
 * There is only limited checking that can be performed for consistancy
 * between the picoArray interface code and the application code.
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

#include "debug.h"
#include "picoarray.h"
#include "picoif_internal.h"
#include "picoif_module.h"
#include "hwif2_internal.h"
#include "dma_fifo_internal.h"
#include <linux/picochip/transports/hwif2.h>
#include <linux/picochip/picoif_ioctl.h>

/*! The maximum number of interrupts that the transport supports. */
#define HWIF2_MAX_INTERRUPT_NUM ( 8 )

/*! The maximum number of devices that the driver supports. */
#define HWIF2_MAX_DEVICES       ( 8 )

/*!
 * \brief The context parameters used on each device
 */
struct hwif2_ctx_params
{
    unsigned interrupts_set;         /*!< The active interrupts on this device */
    int its_gpr;                     /*!< The picoArray to host handshaking GPR */
    int itm_gpr;                     /*!< The host to picoArray handshaking GPR */
    struct pico_resource *its;       /*!< ITS resource */
    struct pico_resource *itm;       /*!< ITM resource */
    struct pico_resource *irq;       /*!< IRQ resource */
    struct picoarray *pa;            /*!< The picoArray running the context */
    struct dma_channel *channel[HWIF2_MAX_INTERRUPT_NUM];
                                     /*!< DMA channels attached to this context */
};

/*!
 * \brief The basic HwIF2 module.
 * \extends picoif_module
 */
struct hwif2_module
{
    struct picoif_module mod;        /*!< The generic module. */
    struct hwif2_ctx_params params[HWIF2_MAX_DEVICES];
                                     /*!< Common parameters this transport */
};

/*!
 * \brief DMA channel parameters (one per DMA channel).
 */
static struct dma_channel
{
    struct picoif_context *reader;    /*!< The reader of the channel. */
    struct pico_resource *chan;       /*!< DMA channel */
    unsigned interruptNumber;         /*!< Interrupt number for the channel */
    unsigned maxTransferSize;         /*!< Max transfer size in bytes */
    unsigned intCount;                /*!< Number of interrupts queued */
    unsigned activeChannel;           /*!< Boolean to determine if the channel
                                       *   is active */
    unsigned xferSize;                /*!< Size of transfer in bytes */
    unsigned totalBytes;
    unsigned xferStartedBytes;
    struct kfifo *fifo;               /*!< DMA FIFO */
    spinlock_t lock;                  /*!< DMA parameter lock */
    struct hwif2_ctx_params *hwif2Ctx;/*!< Pointer to the context parameters used
                                       *   in this channel */
    struct scatterlist sgl[16];        /*!< Transfer scatterlist. */
    int nents;                        /*!< Number of scatterlist entries. */
    struct tasklet_struct tasklet;    /*!< Tasklet for starting transfers. */
} dma_channel;

static void
hwif2_destructor( struct picoif_module *module );

static struct picoif_context *
hwif2_create_trans_instance( struct picoif_module *module,
                             const char *description,
                             struct picoif_buf *params );

static void
hwif2_close_trans_instance( struct picoif_module *module,
                            struct picoif_context *ctx );

static ssize_t
hwif2_write( struct picoif_module *module,
             struct picoif_context *ctx,
             struct picoif_buf *buf,
             size_t len );

static ssize_t
hwif2_read( struct picoif_module *module,
            struct picoif_context *ctx,
            struct picoif_buf *buf,
            size_t len );

static int
hwif2_can_write( struct picoif_module *module,
                 struct picoif_context *ctx );

static int
hwif2_can_read( struct picoif_module *module,
                struct picoif_context *ctx );

static int
hwif2_queue_transfer( struct dma_channel *channel,
                      unsigned size );

/*! picoIf module operations for the DMA module. */
static struct picoif_module_ops hwif2_ops = {
    .destructor             = hwif2_destructor,
    .create_trans_instance  = hwif2_create_trans_instance,
    .close_instance         = hwif2_close_trans_instance,
    .write                  = hwif2_write,
    .read                   = hwif2_read,
    .can_read               = hwif2_can_read,
    .can_write              = hwif2_can_write,
};

/*! Transport methods for the DMA transport module */
static const char *hwif2_tmethods[] = {
    "ul",
};

/*! The HwIF2 transport */
static struct hwif2_module hwif2_mod ={
    .mod = {
        .name       = "hwif2",
        .tmethods   = hwif2_tmethods,
        .ops        = &hwif2_ops,
    },
};

/*!
 * Check if the transport can be written to.
 *
 * @param module The module responsible for the context.
 * @param ctx The context of the transport.
 * @return HwiF2 transports cannot be written and will always return 0
 */
static int
hwif2_can_write( struct picoif_module *module,
                 struct picoif_context *ctx )
{
    return 0;
}

/*!
 * Check if the transport can be read from.
 *
 * @param module The module responsible for the context.
 * @param ctx The context of the transport.
 * @return Returns 1 if the transport can be read (and has data ready),
 * 0 otherwise.
 */
static int
hwif2_can_read( struct picoif_module *module,
                struct picoif_context *ctx )
{
    struct dma_channel *channel = ctx->private_data;
    unsigned bytes_to_read = kfifo_len( channel->fifo );
    int ret = 0;

    if ( bytes_to_read )
        return 1;

    /* Start any queued transfers for this channel */
    spin_lock_irq( &channel->lock );
    ret = hwif2_queue_transfer( channel, 0 );
    spin_unlock_irq( &channel->lock );

    return 0;
}

/*!
 * Handler function for interrupts generated by a completed HwIF2 transfer.
 *
 * @param cookie The channel associated with the interrupt source.
 * @param errno The error number associated with the DMA transfer
 * @return Returns zero on success, negative on failure.
 */
__must_check static int
hwif2_dma_handler( void *cookie,
                   int errno )
{
    struct dma_channel *channel = cookie;

    if ( errno )
        PRINTD( COMPONENT_HWIF2, DBG_ERROR, "DMA transfer terminating "
                "with error %d (Cookie=0x%p)", errno, cookie );

    PRINTD( COMPONENT_HWIF2, DBG_TRACE,"transfer of %d bytes completed",
            channel->xferSize );

    /* Add data transferred to FIFO control parameters */
    kfifo_dma_in_finish( channel->fifo, channel->xferSize );
    dma_unmap_sg( NULL, channel->sgl, channel->nents, DMA_FROM_DEVICE );
    spin_lock_irq( &channel->lock );
    channel->activeChannel = 0;
    wake_up_interruptible( &channel->reader->readq );

    tasklet_schedule( &channel->tasklet );

    spin_unlock_irq( &channel->lock );

    return 0;
}

/*!
 * Write to the DMA transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the write.
 * @param buf The data to be written.
 * @param len The length of data to be written in bytes.
 * @return Returns EINVAL as a write is not permitted for this transport
 */
static ssize_t
hwif2_write( struct picoif_module *module,
             struct picoif_context *ctx,
             struct picoif_buf *buf,
             size_t len )
{
    return -EINVAL;
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
hwif2_read( struct picoif_module *module,
            struct picoif_context *ctx,
            struct picoif_buf *buf,
            size_t len )
{
    struct dma_channel *channel = ctx->private_data;
    unsigned used_bytes=0;
    int ret = 0;

    /* Determine how much data is already in the FIFO */
    used_bytes = dma_fifo_get( channel->fifo, buf, len );

    if ( !used_bytes )
        return -EAGAIN;

    PRINTD( COMPONENT_HWIF2, DBG_TRACE,
            "read %u bytes (requested %u bytes)", used_bytes, len );

    if ( ret )
        return ret;
    else
        return used_bytes;
}

/*!
 * Destructor for the DMA module.
 *
 * @param module The module being destroyed.
 */
static void
hwif2_destructor( struct picoif_module *module )
{
}

/*!
 * Handler function for processing interrupts raised by the GPRs associated with
 * this transport only. This function is passed to the add_irq_handler() method
 * of the picoArray and is called when the specific GPR interrupt is raised.
 * This function will acknowledge the interrupt, and start a DMA transfer
 * of the required size.
 *
 * @param irq The irq resource that raised the interrupt.
 * @param cookie The context associated with the GPR raising the interrupt.
 * @return Returns zero on success, negative on failure.
 */
__must_check static int
hwif2_int_handler( struct pico_resource *irq,
                   void *cookie )
{
    struct hwif2_ctx_params *params = cookie;
    struct picoarray *pa = params->pa;
    int ret = 0;
    uint32_t its=0;
    uint32_t itm=0;
    unsigned size=0;
    unsigned i=0;

    /* Determine the cause of the interrupt */
    ret = pa->ops->register_read( pa, params->its, &its );

    if ( !its )
        return 0;

    /* swap ITM bits 8 & 9 and acknowledge interrupt */
    itm = its & 0x00000300;
    itm ^= 0x00000300;
    ret = pa->ops->register_write( pa, params->itm, itm );

    /* For HwIF2 uplink size (in words) is stored in bits 30:16 of GPR */
    size = ((its >> 16) & 0x7fff) * 4;
    if ( size == 0 )
        BUG();

    /* Now determine the DMA channel that caused the interrupt */
    its = its & 0xFF; /* Bits 0:7 carry interrupt number */

    /* Scan through the active DMA channels to search for the appropriate
       channel to activate */
    for( i=0; i < HWIF2_MAX_INTERRUPT_NUM; i++ )
    {
        if ((params->interrupts_set & (1 << i)) & its)
        {
            spin_lock( &params->channel[ i ]->lock );
            ret = hwif2_queue_transfer( params->channel[i], size );
            spin_unlock( &params->channel[ i ]->lock );
            return ret;
        }
    }

    PRINTD( COMPONENT_HWIF2, DBG_WARN,
            "no DMA channel defined for interrupt %d", its);
    return 0;
}

/*!
 * Function that manages the queuing of packets for DMA transfer. This function
 * will add the data (if provided) to the tail of a queue, and if there is no
 * transfer in progress, extract the nexttransfer from the head of the queue.
 *
 * @param channel The DMA channel handle
 * @param size The size of the transfer to add to the list
 * @return Returns zero on success, negative on failure.
 */
__must_check static int
hwif2_queue_transfer( struct dma_channel *channel,
                      unsigned size )
{

    if ( size )
    {
        channel->totalBytes += size;
        channel->intCount++;

        PRINTD( COMPONENT_HWIF2, DBG_TRACE, "Transfer queued %d bytes", size );
    }

    tasklet_schedule( &channel->tasklet );

    return 0;
}

static void
hwif2_tasklet( unsigned long param )
{
    int size;
    struct dma_channel *channel = ( struct dma_channel * )param;
    struct picoarray *pa = channel->hwif2Ctx->pa;

    if ( channel->activeChannel )
        return;

    spin_lock_irq( &channel->lock );

    size = min( channel->totalBytes - channel->xferStartedBytes,
                kfifo_avail( channel->fifo ) );
    if ( size > channel->maxTransferSize )
    {
        PRINTD(COMPONENT_HWIF2, DBG_WARN,
               "Transfer exceeds FIFO size or max allowed by DMA. Will truncate");
        size = channel->maxTransferSize;
    }
    
    channel->xferSize = size;

    if ( size > 0 )
    {
        int ret;
        channel->xferStartedBytes += size;

        PRINTD( COMPONENT_HWIF2, DBG_TRACE, "start transfer of %zu bytes",
                size );

        sg_init_table( channel->sgl, 16 );
        channel->nents = kfifo_dma_in_prepare( channel->fifo, channel->sgl, 16,
                                               channel->xferSize );
        channel->nents = dma_map_sg( NULL, channel->sgl, channel->nents,
                                     DMA_FROM_DEVICE );

        ret = pa->ops->dma_from_device( pa, channel->chan, channel->sgl,
                                        channel->nents, hwif2_dma_handler,
                                        channel );
        if ( ret )
        {
            PRINTD( COMPONENT_HWIF2, DBG_ERROR,
                    "failed to start DMA from device" );
            BUG();
        }
        else
        {
            PRINTD( COMPONENT_HWIF2, DBG_TRACE, "started hwif2 transfer of=%u",
                    size );
            channel->activeChannel = 1;
        }
    }

    spin_unlock_irq( &channel->lock );

    return;
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
hwif2_create_trans_instance( struct picoif_module *module,
                             const char *description,
                             struct picoif_buf *params )
{
    int ret = -ENOMEM;
    void *private_data = NULL;
    struct picoif_context *ctx = picoif_new_ctx( module, private_data );
    struct picoif_hwif2_params gparams;
    struct picoarray *pa = NULL;
    struct dma_channel *channel;
    struct pico_resource *chan = NULL;
    enum picoarray_device_type dev_type;
    struct pico_resource *its_irq = NULL;
    struct pico_resource *its_gpr = NULL;
    struct pico_resource *itm_gpr = NULL;
    unsigned interrupt_bit = 0;
    if ( !ctx )
        goto bad_ctx;

    ret = picoif_buf_copy_from( &gparams, params, 0,
                                sizeof( gparams ) );
    if ( ret )
        goto out;

    ret = -EINVAL;
    pa = picoif_get_device( gparams.dev_num );
    if ( !pa )
    {
        PRINTD( COMPONENT_HWIF2, DBG_WARN, "invalid device number: %u",
                gparams.dev_num );
        goto out;
    }

    dev_type = pa->ops->get_device_type( pa );
    if ( !( PICOARRAY_PC3XX == dev_type ) )
    {
        PRINTD( COMPONENT_HWIF2, DBG_WARN,
                "device not supported for this transport" );
        goto out;
    }

    /* Check that the DMA channel is not already in use */
    ret = -EBUSY;
    chan = pa->ops->get_resource( pa, PICO_RES_DMA_CHANNEL,
                                             gparams.channel, 1 );
    if ( !chan )
    {
        PRINTD( COMPONENT_HWIF2, DBG_ERROR, "invalid DMA channel:%u",
                gparams.channel );
        goto out;
    }

    /* Check that the interrupt number is valid */
    if (gparams.dev_num >= HWIF2_MAX_INTERRUPT_NUM)
        goto bad_dev_num;


    /* If this is the first transport on the device, set the context
       parameters */
    if ( !hwif2_mod.params[gparams.dev_num].interrupts_set )
    {
        ret = -EBUSY;
        itm_gpr = pa->ops->get_resource( pa, PICO_RES_GPR, gparams.itm_gpr, 1 );
        if ( !itm_gpr )
        {
            PRINTD( COMPONENT_HWIF2, DBG_WARN, "unable to get itm gpr: %u",
                    gparams.itm_gpr );
            goto bad_gpr;
        }

        its_gpr = pa->ops->get_resource( pa, PICO_RES_GPR, gparams.its_gpr, 1 );
        if ( !its_gpr )
        {
            PRINTD( COMPONENT_HWIF2, DBG_WARN, "unable to get its gpr: %u",
                    gparams.its_gpr );
            pa->ops->put_resource( pa, itm_gpr );
            goto bad_gpr;
        }

        its_irq = pa->ops->get_resource( pa, PICO_RES_IRQ, its_gpr->metadata,
                                         1 );
        if ( !its_irq )
        {
            PRINTD( COMPONENT_HWIF2, DBG_WARN, "unable to get irq: %u",
                    gparams.its_gpr );
            pa->ops->put_resource( pa, its_gpr );
            pa->ops->put_resource( pa, itm_gpr );
            goto out;
        }

        hwif2_mod.params[gparams.dev_num].interrupts_set |= (1 << gparams.int_num);
        hwif2_mod.params[gparams.dev_num].its_gpr = gparams.its_gpr;
        hwif2_mod.params[gparams.dev_num].itm_gpr = gparams.itm_gpr;
        hwif2_mod.params[gparams.dev_num].its = its_gpr;
        hwif2_mod.params[gparams.dev_num].itm = itm_gpr;
        hwif2_mod.params[gparams.dev_num].irq = its_irq;
        hwif2_mod.params[gparams.dev_num].pa = pa;
    }
    else
    { /* Check that the common parameters are consistant */
        if (( gparams.its_gpr != hwif2_mod.params[gparams.dev_num].its_gpr ) ||
            ( gparams.itm_gpr != hwif2_mod.params[gparams.dev_num].itm_gpr ) ||
            ( hwif2_mod.params[gparams.dev_num].interrupts_set &
                                                        (1 << gparams.int_num) ))
            goto bad_gpr;

        interrupt_bit =  (((uint32_t)1) << gparams.int_num);
        hwif2_mod.params[gparams.dev_num].interrupts_set |= interrupt_bit;
    }

    /* Allocate the new channel and initialise the data members. */
    ret = -ENOMEM;
    channel = kzalloc( sizeof( dma_channel ), GFP_KERNEL );
    if (!channel)
        goto bad_channel;
    ctx->private_data = channel;

    channel->fifo = dma_buf_cache_alloc( gparams.buf_size );
    if ( !channel->fifo )
        goto bad_fifo_alloc;

    channel->reader = ctx;
    channel->hwif2Ctx = &hwif2_mod.params[gparams.dev_num];
    channel->interruptNumber = gparams.int_num;
    if ( gparams.buf_size > pa->max_dma_sz )
        channel->maxTransferSize = pa->max_dma_sz;
    else
        channel->maxTransferSize = gparams.buf_size;
    channel->chan = chan;
    channel->intCount = 0;
    channel->xferSize = 0;
    channel->activeChannel = 0;
    channel->xferStartedBytes = 0;
    hwif2_mod.params[gparams.dev_num].channel[gparams.int_num] = channel;

    spin_lock_init( &channel->lock );

    if ( strcmp( description, "hwif2(ul)" ) )
    {
        PRINTD( COMPONENT_HWIF2, DBG_ERROR, "invalid tmethod \"%s\"",
                description );
        ret = -EINVAL;
        goto bad_description;
    }

    ret = pa->ops->dma_open( pa, chan );
    if ( ret )
    {
        PRINTD( COMPONENT_HWIF2, DBG_ERROR, "failed to open DMA channel" );
        goto bad_dma_open;
    }

    tasklet_init( &channel->tasklet, hwif2_tasklet, ( unsigned long )channel );

    if ( its_irq )
    {
        ret = pa->ops->add_irq_handler( pa, its_irq, hwif2_int_handler,
                                        channel->hwif2Ctx );
        if ( ret )
        {
            PRINTD( COMPONENT_HWIF2, DBG_ERROR,
                    "failed to register interrupt handler" );
            goto handler_reg_failed;
        }
    }

    return ctx;

handler_reg_failed:
    pa->ops->dma_close( pa, chan );

bad_dma_open:
bad_description:
   dma_buf_cache_free( channel->fifo );

bad_fifo_alloc:
    kfree( channel );

bad_channel:
    hwif2_mod.params[gparams.dev_num].interrupts_set &= ~interrupt_bit;
    if ( its_gpr )
        pa->ops->put_resource( pa, its_gpr );
    if ( itm_gpr )
        pa->ops->put_resource( pa, itm_gpr );
    if ( its_irq )
        pa->ops->put_resource( pa, its_irq );
bad_gpr:
bad_dev_num:
    pa->ops->put_resource(  pa, chan );

out:
    kfree( ctx );
bad_ctx:
   return ERR_PTR( ret );
}

/*!
 * Close an existing transport instance.
 *
 * @param mod The module handling the transport.
 * @param ctx The context that is being closed.
 */
static void
hwif2_close_trans_instance( struct picoif_module *mod,
                            struct picoif_context *ctx )
{
    struct dma_channel *channel = ctx->private_data;
    struct picoarray *pa = channel->hwif2Ctx->pa;
    unsigned interrupt_bit = ((uint32_t)1) << channel->interruptNumber;
    struct hwif2_ctx_params *params = channel->hwif2Ctx;

    tasklet_disable( &channel->tasklet );
    tasklet_kill( &channel->tasklet );
    (void)pa->ops->dma_close( pa, channel->chan );

    PRINTD( COMPONENT_HWIF2, DBG_TRACE, "close transport instance" );

    pa->ops->put_resource( pa, channel->chan );
    dma_buf_cache_free( channel->fifo );

    params->interrupts_set &= ( ~interrupt_bit );
    params->channel[channel->interruptNumber] = NULL;

    if ( !params->interrupts_set )
    {
        PRINTD( COMPONENT_HWIF2, DBG_TRACE,
                "no further hwif2 users for device %u", pa->dev_num );
        pa->ops->remove_irq_handler( pa, params->irq );
        pa->ops->put_resource( pa, params->its );
        pa->ops->put_resource( pa, params->itm );
        pa->ops->put_resource( pa, params->irq );
    }

    if ( channel )
        kfree( channel );

    if ( ctx )
        kfree( ctx );

    PRINTD( COMPONENT_HWIF2, DBG_TRACE, "transport instance closed" );
}

/* Kernel API and Public functions */
struct picoif_context *
picoif_hwif2_dmaul_open( const picoif_hwif2_t hwif2_context,
                         unsigned interrupt_number,
                         int dma_channel,
                         size_t buffer_size )
{
    struct picoif_hwif2_params hwif2_params = {
        .dev_num    = hwif2_context->dev_num,
        .int_num    = interrupt_number,
        .channel    = dma_channel,
        .buf_size   = buffer_size,
        .its_gpr    = hwif2_context->its_gpr,
        .itm_gpr    = hwif2_context->itm_gpr,
    };
    struct picoif_buf buf = {
        .kbuf       = &hwif2_params,
        .is_user    = 0,
    };
    struct picoif_context *ctx =
        hwif2_create_trans_instance( &hwif2_mod.mod,
                                       "hwif2(ul)", &buf );
    return ctx;
}
EXPORT_SYMBOL( picoif_hwif2_dmaul_open );

int
hwif2_init( void )
{
    memset(hwif2_mod.params, sizeof ( hwif2_mod.params ), 0 );

    return picoif_register_module( &hwif2_mod.mod );
}


