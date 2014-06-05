/*****************************************************************************
 * $picoChipHeaderSubst$
 *****************************************************************************/

/*
 * Copyright (c) 2010 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file directdma_internal.h
 * \brief Initialisation function for Direct DMA transport.
 *
 * Internal Direct DMA transport function definitions
 *
 */

#ifndef __PICOIF_DIRECTDMA_INTERNAL_H__
#define __PICOIF_DIRECTDMA_INTERNAL_H__

/*!
 * Initialise the direct DMA transport.
 *
 * @return Returns zero on success, negative on failure.
 */
int
directdma_init( void );

#endif /* !__PICOIF_DIRECTDMA_INTERNAL_H__ */
