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
 * \file dma_fifo.c
 * \brief Main file for the common DMA FIFO functions
 *
 * This file implements the DMA FIFO managing for all transports concerned with
 * the picoArray.
 */

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/kfifo.h>
#include <linux/log2.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/random.h>

#include "picoif_module.h"
#include "dma_fifo_internal.h"

/*! The maximum number of bytes that can be copied to user in a single
 *  transfer */
#define DMA_MAX_TRANSFER_SIZE ( 4096 )

#define DMA_BUF_CACHE_NR_LINES 8

struct dma_buf_cache_line {
    struct kfifo                *fifo;
};

struct dma_buf_cache {
    unsigned long               present;
    unsigned long               in_use;
    struct dma_buf_cache_line   lines[DMA_BUF_CACHE_NR_LINES];
    spinlock_t                  lock;
};

static struct dma_buf_cache dma_buf_cache = {
    .lock = __SPIN_LOCK_UNLOCKED(.lock),
};

void
dma_buf_cache_init( void )
{
//FIX THIS NA1
#if 0 
    int            i;  
    unsigned long  flags;

    spin_lock_irqsave( &dma_buf_cache.lock, flags );

    for ( i = 0; i < DMA_BUF_CACHE_NR_LINES; ++i )
    {   
        if ( !test_bit( i, &dma_buf_cache.present ) ) 
        {   
            struct dma_buf_cache_line  *line;
            void                       *virt;
            dma_addr_t                  phys;
            size_t                      len = 65536;
    
            virt = dma_alloc_coherent( NULL, len, &phys, GFP_ATOMIC );///NA1 CHECK THIS
            if ( virt )
            {   
                line = &dma_buf_cache.lines[ i ];
    
                line->virt = virt;
                line->phys = phys;
                line->len = len;
    
                clear_bit( i, &dma_buf_cache.in_use );
                set_bit( i, &dma_buf_cache.present );
            }   
        }   
    }   
    
    spin_unlock_irqrestore( &dma_buf_cache.lock, flags );
#endif
}


void
dma_buf_cache_exit( void )
{
    int i;

    for_each_set_bit( i, &dma_buf_cache.present, DMA_BUF_CACHE_NR_LINES )
    {
        struct dma_buf_cache_line *line = &dma_buf_cache.lines[ i ];
        kfifo_free( line->fifo );
        kfree( line->fifo );
    }
}

static void
dma_buf_cache_fill( struct kfifo *fifo )
{
    int i;
    struct dma_buf_cache_line *line;

    /*
     * See if we can find an empty cache line first. If not, evict one from
     * the cache. Use a random replacement policy.
     */
    i = find_first_zero_bit( &dma_buf_cache.present, DMA_BUF_CACHE_NR_LINES );
    if ( i >= DMA_BUF_CACHE_NR_LINES )
        i = random32() % DMA_BUF_CACHE_NR_LINES;

    line = &dma_buf_cache.lines[ i ];

    line->fifo = fifo;

    set_bit( i, &dma_buf_cache.in_use );
    set_bit( i, &dma_buf_cache.present );
}

struct kfifo *
dma_buf_cache_alloc( size_t len )
{
    int i;
    unsigned long flags;
    struct kfifo *ret = NULL;
    struct dma_buf_cache_line *line;

    len = roundup_pow_of_two( len );

    spin_lock_irqsave( &dma_buf_cache.lock, flags );

    for_each_set_bit( i, &dma_buf_cache.present, DMA_BUF_CACHE_NR_LINES )
    {
        /* Don't allocate from lines already in use. */
        if ( test_bit( i, &dma_buf_cache.in_use ) )
            continue;

        line = &dma_buf_cache.lines[ i ];

        if ( kfifo_size( line->fifo ) == len )
        {
            set_bit( i, &dma_buf_cache.in_use );
            ret = line->fifo;
            kfifo_reset( line->fifo );

            goto out;
        }
    }

    spin_unlock_irqrestore( &dma_buf_cache.lock, flags );

    /*
     * We haven't been able to reuse a cache line, allocate a new buffer and
     * evict an existing line from the cache.
     */
    ret = kzalloc( sizeof( *ret ), GFP_KERNEL | GFP_DMA );
    if ( !ret )
        goto out;

    if ( kfifo_alloc( ret, len, GFP_KERNEL | GFP_DMA ) )
        goto err_alloc;

    spin_lock_irqsave( &dma_buf_cache.lock, flags );

    dma_buf_cache_fill( ret );

    goto out;

err_alloc:
    kfree( ret );
    ret = NULL;
out:
    spin_unlock_irqrestore( &dma_buf_cache.lock, flags );

    return ret;
}

void
dma_buf_cache_free( struct kfifo *fifo )
{
    int i;
    unsigned long flags;

    spin_lock_irqsave( &dma_buf_cache.lock, flags );

    for ( i = 0; i < DMA_BUF_CACHE_NR_LINES; ++i )
    {
        struct dma_buf_cache_line *line;

        if ( !test_bit( i, &dma_buf_cache.present ) )
            continue;

        line = &dma_buf_cache.lines[ i ];
        if ( line->fifo != fifo )
            continue;

        clear_bit( i, &dma_buf_cache.in_use );
        spin_unlock_irqrestore( &dma_buf_cache.lock, flags );

        return;
    }

    spin_unlock_irqrestore( &dma_buf_cache.lock, flags );

    /* The buffer isn't in the cache any more. */
    kfifo_free( fifo );
    kfree( fifo );
}

int
dma_fifo_put( struct kfifo *fifo,
              struct picoif_buf *buf,
              size_t len )
{
    unsigned copied = 0;
    int err = 0;

    if ( buf->is_user )
        err = kfifo_from_user( fifo, buf->ubuf, len, &copied );
    else
        copied = kfifo_in( fifo, buf, len );

    return err ?: ( int )copied;
}

int
dma_fifo_get( struct kfifo *fifo,
              struct picoif_buf *buf,
              size_t len )
{
    unsigned copied = 0;
    int err = 0;

    if ( buf->is_user )
        err = kfifo_to_user( fifo, buf->ubuf, len, &copied );
    else
        copied = kfifo_out( fifo, buf, len );

    return err ?: ( int )copied;
}
