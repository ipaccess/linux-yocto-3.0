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
 * \file dma_fifo_internal.h
 * \brief Common DMA FIFO managing functions
 *
 * This file contains common functions for managing the DMA FIFOs.
 */

#ifndef __PICOIF_DMA_FIFO_INTERNAL__
#define __PICOIF_DMA_FIFO_INTERNAL__

#include <linux/kfifo.h>

#include "picoif_internal.h"

/*!
 * Preload the DMA buffer cache at module entry.
 */
void
dma_buf_cache_init( void );

/*!
 * Cleanup the DMA buffer cache at module exit.
 */
void
dma_buf_cache_exit( void );

/*!
 * Allocate a DMA fifo from a DMA fifo cache.
 *
 * @param len The length of the fifo in bytes.
 * @return Returns a pointer to the fifo on success.
 */
struct kfifo *
dma_buf_cache_alloc( size_t len );

/*!
 * Free an existing DMA fifo. If this is still in the cache then it will be
 * available for reuse. If not then it will be released back to the kernel
 * memory manager.
 *
 * @param fifo The fifo to free.
 */
void
dma_buf_cache_free( struct kfifo *fifo );

/*!
 * Add some data to the fifo.
 *
 * @fifo The fifo to add the data to.
 * @buf The buffer to add the data from.
 * @len The number of bytes to add.
 * @return Returns the number of bytes added on success, negative on failure.
 */
int
dma_fifo_put( struct kfifo *fifo,
              struct picoif_buf *buf,
              size_t len );

/*!
 * Get some data from the fifo.
 *
 * @fifo The fifo to get the data from.
 * @buf The buffer to add the data to.
 * @len The number of bytes to copy.
 * @return Returns the number of bytes copied on success, negative on failure.
 */
int
dma_fifo_get( struct kfifo *fifo,
              struct picoif_buf *buf,
              size_t len );

#endif /* !__PICOIF_DMA_FIFO_INTERNAL__ */
