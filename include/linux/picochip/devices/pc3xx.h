/*****************************************************************************
 * $picoChipHeaderSubst$
 *****************************************************************************/

/*
 * Copyright (c) 2010 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file pc3xx.h
 * \brief PC3XX resource definition file.
 *
 * This file defines a list of resources including DMA channels, GPRs and
 * interrupt sources in the PC3XX device.
 */

#ifndef __PICOIF_PC3XX_H__
#define __PICOIF_PC3XX_H__

/*!
 * \brief DMA channel identifiers for PC3XX devices.
 *
 * This enum defines a list of DMA channels that are available for data
 * transport in PC3XX devices.
 */
enum picoifDMAId_PC3XX
{
    PC3XX_DMA_AXI2PICO_0 = 0x0000,
    PC3XX_DMA_AXI2PICO_1,
    PC3XX_DMA_AXI2PICO_2,
    PC3XX_DMA_AXI2PICO_3,
    PC3XX_DMA_AXI2PICO_4,
    PC3XX_DMA_AXI2PICO_5,
    PC3XX_DMA_AXI2PICO_6,
    PC3XX_DMA_AXI2PICO_7,
    PICO_NUM_DMA_CHANNELS,

};

/*!
 * \brief GPR identifiers for PC3XX devices.
 *
 * This enum defines a list of general purpose registers (GPRs) that are
 * available for reading/writing and transport use in PC3XX devices.
 */
enum picoifGPRId_PC3XX
{
    PC3XX_GPR_AXI2PICO_0 = 0x1000,
    PC3XX_GPR_AXI2PICO_1,
    PC3XX_GPR_AXI2PICO_2,
    PC3XX_GPR_AXI2PICO_3,
    PC3XX_GPR_AXI2PICO_4,
    PC3XX_GPR_AXI2PICO_5,
    PC3XX_GPR_AXI2PICO_6,
    PC3XX_GPR_AXI2PICO_7,
    PC3XX_GPR_AXI2PICO_8,
    PC3XX_GPR_AXI2PICO_9,
    PC3XX_GPR_AXI2PICO_10,
    PC3XX_GPR_AXI2PICO_11,
    PC3XX_GPR_AXI2PICO_12,
    PC3XX_GPR_AXI2PICO_13,
    PC3XX_GPR_AXI2PICO_14,
    PC3XX_GPR_AXI2PICO_15,
    PC3XX_GPR_AXI2PICO_16,
    PC3XX_GPR_AXI2PICO_17,
    PC3XX_GPR_AXI2PICO_18,
    PC3XX_GPR_AXI2PICO_19,
    PC3XX_GPR_AXI2PICO_20,
    PC3XX_GPR_AXI2PICO_21,
    PC3XX_GPR_AXI2PICO_22,
    PC3XX_GPR_AXI2PICO_23,
    PICO_NUM_GPRS,

};

/*!
 * \brief IRQ identifiers for PC3XX devices.
 *
 * This enum defines a list of interrupt sources that are available for
 * transport use in PC3XX devices.
 */
enum picoifIRQId_PC3XX
{
    PC3XX_IRQ_AXI2PICO_0 = 0x2000,
    PC3XX_IRQ_AXI2PICO_1,
    PC3XX_IRQ_AXI2PICO_2,
    PC3XX_IRQ_AXI2PICO_3,
    PC3XX_IRQ_AXI2PICO_4,
    PC3XX_IRQ_AXI2PICO_5,
    PC3XX_IRQ_AXI2PICO_6,
    PC3XX_IRQ_AXI2PICO_7,
    PC3XX_IRQ_AXI2PICO_8,
    PC3XX_IRQ_AXI2PICO_9,
    PC3XX_IRQ_AXI2PICO_10,
    PC3XX_IRQ_AXI2PICO_11,
    PC3XX_IRQ_AXI2PICO_12,
    PC3XX_IRQ_AXI2PICO_13,
    PC3XX_IRQ_AXI2PICO_14,
    PC3XX_IRQ_AXI2PICO_15,
    PC3XX_IRQ_AXI2PICO_16,
    PC3XX_IRQ_AXI2PICO_17,
    PC3XX_IRQ_AXI2PICO_18,
    PC3XX_IRQ_AXI2PICO_19,
    PC3XX_IRQ_AXI2PICO_20,
    PC3XX_IRQ_AXI2PICO_21,
    PC3XX_IRQ_AXI2PICO_22,
    PC3XX_IRQ_AXI2PICO_23,
    PICO_NUM_IRQS,

};

/*!
 * \brief Type for identifying registers in a picoArray.
 *
 * GPR identifier. This is used to define a GPR resource for transports and
 * for GPR accesses - these should be used rather than absolute GPR numbers.
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifGPRId_PC3XX picoifGPRId_t;

/*!
 * \brief Type for identifying DMA channels in a picoArray.
 *
 * DMA identifier. This is used to define a DMA resource for transports these
 * should be used rather than absolute DMA channel numbers.
 *
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifDMAId_PC3XX picoifDMAId_t;

/*!
 * \brief Type for identifying IRQ numbers in a picoArray.
 *
 * IRQ identifier. This is used to define an IRQ resource for transports these
 * should be used rather than absolute IRQ numbers.
 *
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifIRQId_PC3XX picoifIRQId_t;

#endif /* !__PICOIF_PC3XX_H__ */
