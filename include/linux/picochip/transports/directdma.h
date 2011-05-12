/*****************************************************************************
 * $picoChipHeaderSubst$
 *****************************************************************************/

/*
 * Copyright (c) 2009 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file directdma.h
 *
 * \brief Public picoIf Direct DMA transport method kernel API.
 *  \addtogroup kernelAPI
 *  @{
 *   \addtogroup DMA
 *    @{
 */

#ifndef __PICOIF_PUBLIC_DIRECTDMA_H__
#define __PICOIF_PUBLIC_DIRECTDMA_H__
#include <linux/types.h>
#include <linux/picochip/picoif.h>

struct picoif_context;

/*!
 * Open a new instance of the direct DMA transport.
 *
 * @param dev_num The logical device number of the picoArray to use.
 * @param dma_chan The identifier of the DMA channel to use. This should be
 * taken from the picoifDMAId_t enumeration for the appropriate device type.
 * @param buf_size The size of buffer in bytes to allocate for this channel
 * @param callback The callback to call at transfer completion.
 * @return Returns a context on success, or an ERR_PTR encoded error on
 * failure.
 */
struct picoif_context *
picoif_directdma_open( unsigned dev_num,
                       int dma_chan,
                       void ( *callback )( size_t nbytes,
                                           void *cookie ) );

/*!
 * Write to the data supplied in a kernel mapped buffer into a DMA transport.
 *
 * @param ctx The open context of the transport.
 * @param sgl The data to write.
 * @param n_ents The number of entries in the scatterlist.
 * @return Returns the number of bytes written to the transport on success,
 * negative on failure. If there is no space in the transport to buffer the
 * data the -EAGAIN is returned.
 */
static inline ssize_t
picoif_directdma_writesg( struct picoif_context *ctx,
                          struct scatterlist *sgl,
                          unsigned n_ents,
			  void *cookie )
{
    return picoif_transport_generic_writesg( ctx, sgl, n_ents, cookie );
}

/*!
 * Close an open instance of a direct DMA transport.
 *
 * @param ctx The context to close.
 */
static inline void
picoif_directdma_close( struct picoif_context *ctx )
{
    picoif_transport_generic_close( ctx );
}

/*! @} */
/*! @} */

#endif /* !__PICOIF_PUBLIC_DIRECTDMA_H__ */
