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
 * \file pc302.h
 * \brief PC302 resource definition file.
 *
 * This file defines a list of resources including DMA channels, GPRs and
 * interrupt sources in the PC302 device. This file is deprecated, please
 * use include file pc3xx.h
 */

#ifndef __PICOIF_PC302_H__
#define __PICOIF_PC302_H__

/*!
 * \brief DMA channel identifiers for PC302 devices.
 *
 * This enum defines a list of DMA channels that are available for data
 * transport in PC302 devices.
 */
enum picoifDMAId_PC302
{
    PC302_DMA_AXI2PICO_0 = 0x0000,
    PC302_DMA_AXI2PICO_1,
    PC302_DMA_AXI2PICO_2,
    PC302_DMA_AXI2PICO_3,
    PC302_DMA_AXI2PICO_4,
    PC302_DMA_AXI2PICO_5,
    PC302_DMA_AXI2PICO_6,
    PC302_DMA_AXI2PICO_7,
    PICO_NUM_DMA_CHANNELS,

} __attribute__ ((deprecated));

/*!
 * \brief GPR identifiers for PC302 devices.
 *
 * This enum defines a list of general purpose registers (GPRs) that are
 * available for reading/writing and transport use in PC302 devices.
 */
enum picoifGPRId_PC302
{
    PC302_GPR_AXI2PICO_0 = 0x1000,
    PC302_GPR_AXI2PICO_1,
    PC302_GPR_AXI2PICO_2,
    PC302_GPR_AXI2PICO_3,
    PC302_GPR_AXI2PICO_4,
    PC302_GPR_AXI2PICO_5,
    PC302_GPR_AXI2PICO_6,
    PC302_GPR_AXI2PICO_7,
    PC302_GPR_AXI2PICO_8,
    PC302_GPR_AXI2PICO_9,
    PC302_GPR_AXI2PICO_10,
    PC302_GPR_AXI2PICO_11,
    PC302_GPR_AXI2PICO_12,
    PC302_GPR_AXI2PICO_13,
    PC302_GPR_AXI2PICO_14,
    PC302_GPR_AXI2PICO_15,
    PC302_GPR_AXI2PICO_16,
    PC302_GPR_AXI2PICO_17,
    PC302_GPR_AXI2PICO_18,
    PC302_GPR_AXI2PICO_19,
    PC302_GPR_AXI2PICO_20,
    PC302_GPR_AXI2PICO_21,
    PC302_GPR_AXI2PICO_22,
    PC302_GPR_AXI2PICO_23,
    PICO_NUM_GPRS,

} __attribute__ ((deprecated));

/*!
 * \brief IRQ identifiers for PC302 devices.
 *
 * This enum defines a list of interrupt sources that are available for
 * transport use in PC302 devices.
 */
enum picoifIRQId_PC302
{
    PC302_IRQ_AXI2PICO_0 = 0x2000,
    PC302_IRQ_AXI2PICO_1,
    PC302_IRQ_AXI2PICO_2,
    PC302_IRQ_AXI2PICO_3,
    PC302_IRQ_AXI2PICO_4,
    PC302_IRQ_AXI2PICO_5,
    PC302_IRQ_AXI2PICO_6,
    PC302_IRQ_AXI2PICO_7,
    PC302_IRQ_AXI2PICO_8,
    PC302_IRQ_AXI2PICO_9,
    PC302_IRQ_AXI2PICO_10,
    PC302_IRQ_AXI2PICO_11,
    PC302_IRQ_AXI2PICO_12,
    PC302_IRQ_AXI2PICO_13,
    PC302_IRQ_AXI2PICO_14,
    PC302_IRQ_AXI2PICO_15,
    PC302_IRQ_AXI2PICO_16,
    PC302_IRQ_AXI2PICO_17,
    PC302_IRQ_AXI2PICO_18,
    PC302_IRQ_AXI2PICO_19,
    PC302_IRQ_AXI2PICO_20,
    PC302_IRQ_AXI2PICO_21,
    PC302_IRQ_AXI2PICO_22,
    PC302_IRQ_AXI2PICO_23,
    PICO_NUM_IRQS,

} __attribute__ ((deprecated));

/*!
 * \brief Type for identifying registers in a picoArray.
 *
 * GPR identifier. This is used to define a GPR resource for transports and
 * for GPR accesses - these should be used rather than absolute GPR numbers.
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifGPRId_PC302 picoifGPRId_t;

/*!
 * \brief Type for identifying DMA channels in a picoArray.
 *
 * DMA identifier. This is used to define a DMA resource for transports these
 * should be used rather than absolute DMA channel numbers.
 *
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifDMAId_PC302 picoifDMAId_t;

/*!
 * \brief Type for identifying IRQ numbers in a picoArray.
 *
 * IRQ identifier. This is used to define an IRQ resource for transports these
 * should be used rather than absolute IRQ numbers.
 *
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifIRQId_PC302 picoifIRQId_t;

#endif /* !__PICOIF_PC302_H__ */
