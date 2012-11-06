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
 * \file pc3xx.c
 * \brief PC3XX device implementation.
 *
 * This file implements the PC3XX support of picoIf. All implementation in
 * this file is private and should not be accessed directly by users and only
 * provides the necessary basic services with which transports can be built
 * upon and devices configured.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dw_dmac.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <mach/picoxcell/axi2cfg.h>
#include <mach/picoxcell/picoxcell.h>

#include <linux/picochip/picoif.h>
#include <linux/picochip/devices/pc3xx.h>

#include "picoarray.h"
#include "resource.h"
#include "picoif_internal.h"
#include "debug.h"
#include "utilities_internal.h"

/*! The address of the AXI2Pico interrupt status register. This is an offset
 * from the AXI2Pico base address. */
#define AXI2PICO_INT_STATUS_OFFSET  ( 0x0200 )

/*! The number of virtual ports in the AXI2Pico. */
#define AXI2PICO_NUM_VPS           ( 32 )

/*! The spacing of virtual ports in the AXI2Pico. */
#define AXI2PICO_VP_SPACING         ( 0x10 )

/*! The offset of a virtual port config register from the virtual port
 *  register base. */
#define AXI2PICO_VP_CONFIG_OFFSET   ( 0x00 )

/*! The offset of a virtual port status register from the virtual port
 *  register base. */
#define AXI2PICO_VP_STATUS_OFFSET   ( 0x04 )

/*! The offset of the data register for a virtual port in the AXI2Pico. */
#define AXI2PICO_VP_DATA_OFFSET     ( 0x08 )

/*! The CAEID of the procif. */
#define PC3XX_AXI2CFG_CAEID          ( 0x48 )

/*! The CAEID of the AXI2Pico. */
#define PC3XX_AXI2PICO_CAEID	     ( 0x9000 )

/*! The CAEID of the Frac-N. */
#define PC3XX_FRACN_CAEID            ( 0x8080 )

/*! The offset in the procif for the multi step count registers (4). */
#define PC3XX_PROCIF_MULTI_CNT_OFFSET ( 0x4004 )

/*! The offset in the procif for the operation request register. */
#define PC3XX_AXI2CFG_OP_REQ_OFFSET  ( 0x4018 )

/*! The offset of the sync bit in the operation request register */
#define PC3XX_AXI2CFG_OP_REQ_SYNC      ( 1 << 0 )

/*! The offset of the start request bit in the operation request register. */
#define PC3XX_AXI2CFG_OP_REQ_START   ( 1 << 1 )

/*! The offset of the step request bit in the operation request register. */
#define PC3XX_AXI2CFG_OP_REQ_STEP    ( 1 << 2 )

/*! The offset in the procif for the operation status register. */
#define PC3XX_AXI2CFG_OP_STATUS_OFFSET  ( 0x401c )

/*! The offset of the step  bit in the operation status register */
#define PC3XX_AXI2CFG_OP_STATUS_STEPPING ( 1 << 0 )

/*! The offset of the start bit in the operation status register */
#define PC3XX_AXI2CFG_OP_STATUS_RUNNING  ( 1 << 1 )

/*! The offset of the sync bit in the operation status register */
#define PC3XX_AXI2CFG_OP_STATUS_IN_SYNC  ( 0 << 2 )

/*! The number of multi count registers */
#define PC3XX_NUM_MULTI_COUNT_REGS    ( 4 )

/*! The position of the interrupt enable bit in a virtual port config
 *  register. */
#define AXI2PICO_INT_EN             ( 1 << 0 )

/*! The offset in an AE for the sleep register. */
#define SLEEP_REG_OFFSET	    ( 0xA060 )

/*! Timeout for DMA load of picoArray */
#define TIMEOUT_10MS                ( HZ / 100 )

/*! DMA burst size in words */
#define PC3XX_DMA_BURST_SIZE        ( 1 )

/*! The maximum number of transactions per transfer */
#define DMAH_CHX_MAX_BLK_SIZE       ( 4095 )

/*! Determine the msize and max transfer size that is permitted */
#if ((PC3XX_DMA_BURST_SIZE == 1) || (PC3XX_DMA_BURST_SIZE == 2))
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC3XX_DMA_MS_1_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 4)
#elif (PC3XX_DMA_BURST_SIZE == 4)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC3XX_DMA_MS_4_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 8)
#elif (PC3XX_DMA_BURST_SIZE == 8)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC3XX_DMA_MS_8_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 16)
#elif (PC3XX_DMA_BURST_SIZE == 16)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC3XX_DMA_MS_16_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 32)
#elif (PC3XX_DMA_BURST_SIZE == 32)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC3XX_DMA_MS_32_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 64)
#else
    #error
#endif

/*!
 * Determine if a GPR port is enabled
 */
#define IS_PORT_ENABLED(status_reg) (status_reg & 0x1)

/*!
 *  Determine if a GPR port is configured as a receive port
 */
#define IS_PORT_A_RECEIVE(status_reg) (status_reg & 0x2)

/*!
 *  Determine if a GPR port is configured as a non blocking port
 */
#define IS_PORT_NONBLOCKING(status_reg) (status_reg & 0x4)

/*!
 *  Determine if a GPR port is ready to give or receive data
 */
#define IS_PORT_READY_FOR_DATA(status_reg) (status_reg & 0x8)

/*! The number of registered PC3XXs in the system. Until this reaches zero, we
 * can't unregister the platform driver. */
static unsigned num_pc3xxs;

static void
pc3xx_destroy( struct picoarray *pa );

/*!
 * \brief PC3XX IRQ handler.
 *
 * This structure defines an IRQ handler for PC3XX devices and is an internal
 * structure that could be reused for other device types.
 */
struct pc3xx_irq_handler
{
    /*! The position in the list of handlers for an interrupt source. */
    struct list_head        list;

    /*! The IRQ that this handles. */
    struct pico_resource    *irq;

    /*! A cookie to pass to the callback function when the interrupt is
     * raised. */
    void                    *cookie;

    /*! The callback function to call when the interrupt is raised. */
    int                     ( *callback )( struct pico_resource *irq,
                                           void *cookie );
};

/*!
 * \brief PC3XX DMA handler
 *
 * This structure defines a DMA handler for PC3XX devices and is an internal
 * strucutre that might be reused for other device types that uses the Synopsys
 * DMAC. There will be one instance of this structure per DMA channel in the
 * system.
 */
struct pc3xx_dma_channel
{
    struct dw_dma_slave slave;

    struct dma_chan *chan;

    /*! picoArray DMA channel number */
    unsigned channel;

    /*! Channel status (1 for active and 0 for waiting) */
    int stateActive;

    /*! Physical address for the picoArray DMA channel */
    dma_addr_t pico_addr;

    /*! The callback function to call when the DMA transfer is completed. */
    int ( *callback )( void *cookie,
                       int errno );

    /*! User data */
    void *cookie;

    struct dma_async_tx_descriptor *curr_desc;
};

/*!
 * \brief Private representation of a PC3XX device.
 *
 * This describes all private data required in the PC3XX implementation of
 * this driver.
 *
 * \extends picoarray
 */
struct pc3xx
{
    /*! The picoArray base class. */
    struct picoarray            pa;

    /*! The physical address of the lower axi2cfg registers. */
    dma_addr_t                  axi2cfg1_base_phys;

    /*! The length of the lowe axi2cfg registers in bytes. */
    size_t                      axi2cfg1_base_len;

    /*! The virtually mapped address of the lower axi2cfg registers. */
    void __iomem                *axi2cfg1_base;

    /*! The physical address of the upper axi2cfg registers. */
    dma_addr_t                  axi2cfg2_base_phys;

    /*! The length of the upper axi2cfg registers in bytes. */
    size_t                      axi2cfg2_base_len;

    /*! The physical address of the AXI2Pico registers. */
    dma_addr_t                  axi2pico_base_phys;

    /*! The length of the AXI2Pico registers in bytes. */
    size_t                      axi2pico_base_len;

    /*! The virtually mappped address of the AXI2Pico registers. */
    void __iomem                *axi2pico_base;

    /*! The AXI2Pico IRQ number. The AXI2Pico supports a number of interrupt
     * sources, but these are all raised through a single IRQ line. */
    unsigned                    gpr_irq;

    /*! The handlers registered for the AXI2Pico IRQ. */
    struct pc3xx_irq_handler    axi2pico_irq_handlers;

    /*! DMA channel data */
    struct pc3xx_dma_channel    dma_channel[PICO_NUM_DMA_CHANNELS];

    struct device               *axi2pico_dmac;
    struct device               *axi2cfg_dmac;

    struct clk			*axi2pico_clk;
};

/*!
 * Get the PC3XX structure given a picoArray base class.
 *
 * @param pa The base class pointer.
 * @return Returns a pointer to the PC3XX structure on success, NULL on
 * failure.
 */
static inline struct pc3xx *
to_pc3xx( struct picoarray *pa )
{
    return pa ? container_of( pa, struct pc3xx, pa ) : NULL;
}

/*!
 * Write the value of an AXI2Pico register.
 *
 * @param dev The PC3XX owning the AXI2Pico.
 * @param offset The offset of the register in bytes.
 * @param value The value of the register to write.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_axi2pico_reg_write( struct pc3xx *dev,
                          unsigned offset,
                          u32 value )
{
    PRINTD( COMPONENT_PC3XX, DBG_TRACE, "pA[%u] axi2pico, %04x:=%08x",
            dev->pa.dev_num, offset, value );
    picoif_out32( value, dev->axi2pico_base + offset );
    return 0;
}

/*!
 * Read the value of an AXI2Pico register.
 *
 * @param dev The PC3XX owning the AXI2Pico.
 * @param offset The offset of the register in bytes.
 * @param[out] value Pointer to the address to store the register value in.
 */
static void
pc3xx_axi2pico_reg_read( struct pc3xx *dev,
                         unsigned offset,
                         u32 *value )
{
    *value = picoif_in32( dev->axi2pico_base + offset );
}

/*!
 * Read a number of 16 bit words from the PC3XX axi2cfg.
 *
 * @param pa The device to read from.
 * @param caeid The CAEID of the AE to read from.
 * @param address The start address in the AE to begin reading from.
 * @param count The number of 16 bit words to read.
 * @param[out] data The buffer to store the data in.
 * @return Returns the number of words read on success, negative on failure.
 */
static int
pc3xx_config_read( struct picoarray *pa,
                   u16 caeid,
                   u16 address,
                   u16 count,
                   u16 *data )
{
    PRINTD( COMPONENT_PC3XX, DBG_TRACE,
            "pa[%u]: config read %u words %04x@%04x", pa->dev_num,
            count, caeid, address );

    return axi2cfg_config_read( caeid, address, data, count );
}

/*!
 * Write a number of 16 bit words to the PC3XX axi2cfg.
 *
 * @param pa The device to write to.
 * @param caeid The CAEID of the AE to write to.
 * @param address The start address in the AE to begin writing to.
 * @param count The number of 16 bit words to write.
 * @param[in] data The buffer to write from.
 * @return Returns the number of words written on success, negative on failure.
 */
static int
pc3xx_config_write( struct picoarray *pa,
                    u16 caeid,
                    u16 address,
                    u16 count,
                    u16 *data )
{
    PRINTD( COMPONENT_PC3XX, DBG_TRACE,
            "pa[%u]: config write %u words %04x@%04x", pa->dev_num,
            count, caeid, address );

    axi2cfg_config_write( caeid, address, data, count );

    return count;
}

/*!
 * Sync the PC3XX device. PC3XX requires no synchronisation as it is a single
 * device so do nothing.
 *
 * @param pa The device to sync.
 * @return Always returns zero.
 */
static int
pc3xx_sync( struct picoarray *pa )
{
    return 0;
}

static int
pc3xx_is_running( struct picoarray *pa )
{
    u16 val;

    if ( 1 != pa->ops->config_read( pa, PC3XX_AXI2CFG_CAEID,
                                    PC3XX_AXI2CFG_OP_STATUS_OFFSET, 1, &val ) )
        return 0;

    return val & ( PC3XX_AXI2CFG_OP_STATUS_RUNNING |
                   PC3XX_AXI2CFG_OP_STATUS_STEPPING );
}

/*!
 * Start the PC3XX device running.
 *
 * @param pa The device to start.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_start( struct picoarray *pa )
{
    u16 val;
    u16 cycle_count[PC3XX_NUM_MULTI_COUNT_REGS];
    int ret = 0;
    
    /* Only perform a start on the master device (assumed 0 ) */
    if ( pa->dev_num == 0 )
    {
        if ( pc3xx_is_running( pa ) )
        {
            PRINTD( COMPONENT_PC3XX, DBG_WARN, "system already running");
            goto out;
        }

        /* Read multi cycle count register to determine if we are to perform
           a run forever or a step for a number of cycles */
        ret = pa->ops->config_read( pa, PC3XX_AXI2CFG_CAEID,
            PC3XX_PROCIF_MULTI_CNT_OFFSET, PC3XX_NUM_MULTI_COUNT_REGS, cycle_count );
        if ( ret != PC3XX_NUM_MULTI_COUNT_REGS )
        {
            ret = -EINVAL;
            PRINTD( COMPONENT_PC3XX, DBG_ERROR,
                "failed to read multi count reg" );
            goto out;
        }

        /* Only run if the 50 bit count register is zero */
        if ( ( cycle_count[0] | cycle_count[1] | cycle_count[2] |
              (cycle_count[3] & 0x3) ) == 0 )
        {
            val = PC3XX_AXI2CFG_OP_REQ_START;
        }
        else
        {
        val = PC3XX_AXI2CFG_OP_REQ_STEP;
        }
    
        ret = pa->ops->config_write( pa, PC3XX_AXI2CFG_CAEID,
                                 PC3XX_AXI2CFG_OP_REQ_OFFSET, 1, &val );
        if ( 1 != ret )
            goto out;
    }

    ret = 0;
out:
    return ret;
}

/*!
 * Stop the PC3XX device running.
 *
 * @param pa The device to stop.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_stop( struct picoarray *pa )
{
    u16 val;
    int ret = 0;

    /* Only perform a start on the master device (assumed 0 ) */
    if ( pa->dev_num == 0 )
    {
        if ( !pc3xx_is_running( pa ) )
        {
            PRINTD( COMPONENT_PC3XX, DBG_WARN, "system is not running" );
            goto out;
        }

        val = 0;
        ret = pa->ops->config_write( pa, PC3XX_AXI2CFG_CAEID,
            PC3XX_AXI2CFG_OP_REQ_OFFSET, 1, &val );
        if ( 1 != ret )
            goto out;
    }

    ret = 0;
out:
    return ret;
}

/*! The CAEID of the MemIf. */
#define PC3XX_MEMIF_CAEID           ( 0x8020 )
/*! The address of the FIFO status lower register. */
#define PC3XX_MEMIF_FIFO_STAT_LOW   ( 0x0080 )

/*! The CAEID of the PAI */
#define PC3XX_PAI_CAEID             ( 0x8080 )
/*! The address of the pai_io_ctrl register. */
#define PC3XX_PAI_IO_CTRL_REG       ( 0x0009 )

/*!
 * Reset the PC3XX. This performs a soft reset that returns the picoArray to
 * be as close as possible to the hardware reset state without affecting the
 * ARM subsystem.
 *
 * @param pa The device being reset.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_reset( struct picoarray *pa )
{
    int ret;
    unsigned long flags;
    u16 val;
    u16 io_ctrl_value;

    u16 buf_vals[ 0x14 ] = {
                            0xf0, 0, 0xf0, 0, 0xf0, 0, 0xf0, 0, 0xf0, 0,
                            0xf0, 0, 0xf0, 0, 0xf0, 0, 0xf0, 0, 0xf0, 0,
                           };
    u16 error_vals[ 4 ] = { 0, 0, 0, 0, };

    PRINTD( COMPONENT_PC3XX, DBG_TRACE, "pa[%u]: reset", pa->dev_num );

    spin_lock_irqsave( &pa->lock, flags );
    ret = pa->ops->config_write( pa, PC3XX_MEMIF_CAEID, 0, 0x14, buf_vals );
    if ( 0x14 != ret )
        goto out;

    ret = pa->ops->config_write( pa, PC3XX_MEMIF_CAEID,
                                 PC3XX_MEMIF_FIFO_STAT_LOW, 4, error_vals );
    if ( 4 != ret )
        goto out;

    /* Save the PAI io_ctrl register value */
    ret = pa->ops->config_read( pa, PC3XX_PAI_CAEID,
                                PC3XX_PAI_IO_CTRL_REG, 1,
                                &io_ctrl_value );
    if ( 1 != ret )
        goto out;

    axi2cfg_writel( axi2cfg_readl( AXI2CFG_SYSCFG_REG_OFFSET ) |
		    AXI2CFG_SYSCFG_PA_RST_MASK,
		    AXI2CFG_SYSCFG_REG_OFFSET );

    /* Wait for the reset to clear. */
    while ( axi2cfg_readl( AXI2CFG_SYSCFG_REG_OFFSET ) &
	    AXI2CFG_SYSCFG_PA_RST_MASK )
        cpu_relax();

    /* Wake the AXI2Pico and the Frac-N back up. We need to do this so that the
     * GPIO driver and Frac-N drivers can keep running without losing access to
     * their blocks. */
    val = 0;
    ret =  pa->ops->config_write( pa, PC3XX_AXI2PICO_CAEID,
				  SLEEP_REG_OFFSET, 1, &val );
    if ( 1 != ret)
        goto out;

    ret = pa->ops->config_write( pa, PC3XX_FRACN_CAEID,
				 SLEEP_REG_OFFSET, 1, &val );
    if ( 1 != ret)
        goto out;

    /* Wake up the PAI up so we can restore the io_ctrl register
     * back to what it was prior to the reset happening. */
    ret = pa->ops->config_write( pa, PC3XX_PAI_CAEID,
                                 SLEEP_REG_OFFSET, 1, &val );
    if ( 1 != ret)
        goto out;

    /* Restore the PAI io_ctrl register value */
    ret = pa->ops->config_write( pa, PC3XX_PAI_CAEID,
                                 PC3XX_PAI_IO_CTRL_REG, 1,
                                 &io_ctrl_value );
    if ( 1 != ret)
        goto out;

    ret = 0;

out:
    spin_unlock_irqrestore( &pa->lock, flags );
    return ret;
}

/*!
 * Read a GPR (general purpose register) in the PC3XX.
 *
 * @param pa The device being read from.
 * @param reg The register to read.
 * @param[out] value The address to store the register value in.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_register_read( struct picoarray *pa,
                     struct pico_resource *reg,
                     u32 *value )
{
    struct pc3xx *dev = to_pc3xx( pa );
    u32 gprStatus = 0;

    if ( !reg || PICO_RES_GPR != reg->type )
        return -EINVAL;

    if ( reg->value < PC3XX_GPR_AXI2PICO_0 ||
         reg->value > PC3XX_GPR_AXI2PICO_23 )
        return -EINVAL;

    if ( !pc3xx_is_running( pa ) )
        return -EPERM;

    pc3xx_axi2pico_reg_read( dev, reg->offset + AXI2PICO_VP_STATUS_OFFSET,
                             &gprStatus );
    if ((!IS_PORT_ENABLED(gprStatus)) ||
        (!IS_PORT_A_RECEIVE(gprStatus)) ||
        (!IS_PORT_READY_FOR_DATA(gprStatus)))
      return -EINVAL;

    pc3xx_axi2pico_reg_read( dev, reg->offset + AXI2PICO_VP_DATA_OFFSET,
                             value );

    return 0;
}

/*!
 * Write a GPR (general purpose register) in the PC3XX.
 *
 * @param pa The device being written to.
 * @param reg The register to write.
 * @param value The value to write to the register.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_register_write( struct picoarray *pa,
                      struct pico_resource *reg,
                      u32 value )
{
    struct pc3xx *dev = to_pc3xx( pa );
    u32 gprStatus = 0;

    if ( !reg || PICO_RES_GPR != reg->type )
        return -EINVAL;

    if ( reg->value < PC3XX_GPR_AXI2PICO_0 ||
         reg->value > PC3XX_GPR_AXI2PICO_23 )
        return -EINVAL;

    if ( !pc3xx_is_running( pa ) )
        return -EPERM;

    pc3xx_axi2pico_reg_read( dev, reg->offset + AXI2PICO_VP_STATUS_OFFSET,
          &gprStatus );
    if ((!IS_PORT_ENABLED(gprStatus)) ||
        (IS_PORT_A_RECEIVE(gprStatus)) ||
        (!IS_PORT_READY_FOR_DATA(gprStatus)))
      return -EINVAL;

    pc3xx_axi2pico_reg_write( dev, reg->offset + AXI2PICO_VP_DATA_OFFSET,
                              value );

    return 0;
}

/*!
 * Load the picoArray specified with the array of data
 *
 * @param pa The device being written to.
 * @param data The virtual address of the buffer to write
 * @param sgl The scatter gather list of the source data.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_pa_load( struct picoarray *pa,
               u32 *data,
               struct scatterlist *sgl  )
{
    struct scatterlist *pos;

    for ( pos = sgl; pos; pos = sg_next( pos ) )
    {
        unsigned nr_words = pos->length / sizeof( u32 );

        axi2cfg_write_buf( sg_virt( pos ), nr_words );
    }

    return 0;
}

/*! The resources for a PC3XX device. */
static struct pico_resource pc3xx_resources[] = {
    { .type = PICO_RES_DMA_CHANNEL, .value = PC3XX_DMA_AXI2PICO_0, .offset = 0x00 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC3XX_DMA_AXI2PICO_1, .offset = 0x10 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC3XX_DMA_AXI2PICO_2, .offset = 0x20 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC3XX_DMA_AXI2PICO_3, .offset = 0x30 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC3XX_DMA_AXI2PICO_4, .offset = 0x40 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC3XX_DMA_AXI2PICO_5, .offset = 0x50 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC3XX_DMA_AXI2PICO_6, .offset = 0x60 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC3XX_DMA_AXI2PICO_7, .offset = 0x70 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_0, .metadata = PC3XX_IRQ_AXI2PICO_0, .offset = 0x80 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_1, .metadata = PC3XX_IRQ_AXI2PICO_1, .offset = 0x90 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_2, .metadata = PC3XX_IRQ_AXI2PICO_2, .offset = 0xa0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_3, .metadata = PC3XX_IRQ_AXI2PICO_3, .offset = 0xb0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_4, .metadata = PC3XX_IRQ_AXI2PICO_4, .offset = 0xc0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_5, .metadata = PC3XX_IRQ_AXI2PICO_5, .offset = 0xd0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_6, .metadata = PC3XX_IRQ_AXI2PICO_6, .offset = 0xe0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_7, .metadata = PC3XX_IRQ_AXI2PICO_7, .offset = 0xf0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_8, .metadata = PC3XX_IRQ_AXI2PICO_8, .offset = 0x100 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_9, .metadata = PC3XX_IRQ_AXI2PICO_9, .offset = 0x110 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_10, .metadata = PC3XX_IRQ_AXI2PICO_10, .offset = 0x120 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_11, .metadata = PC3XX_IRQ_AXI2PICO_11, .offset = 0x130 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_12, .metadata = PC3XX_IRQ_AXI2PICO_12, .offset = 0x140 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_13, .metadata = PC3XX_IRQ_AXI2PICO_13, .offset = 0x150 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_14, .metadata = PC3XX_IRQ_AXI2PICO_14, .offset = 0x160 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_15, .metadata = PC3XX_IRQ_AXI2PICO_15, .offset = 0x170 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_16, .metadata = PC3XX_IRQ_AXI2PICO_16, .offset = 0x180 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_17, .metadata = PC3XX_IRQ_AXI2PICO_17, .offset = 0x190 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_18, .metadata = PC3XX_IRQ_AXI2PICO_18, .offset = 0x1a0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_19, .metadata = PC3XX_IRQ_AXI2PICO_19, .offset = 0x1b0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_20, .metadata = PC3XX_IRQ_AXI2PICO_20, .offset = 0x1c0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_21, .metadata = PC3XX_IRQ_AXI2PICO_21, .offset = 0x1d0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_22, .metadata = PC3XX_IRQ_AXI2PICO_22, .offset = 0x1e0 },
    { .type = PICO_RES_GPR, .value = PC3XX_GPR_AXI2PICO_23, .metadata = PC3XX_IRQ_AXI2PICO_23, .offset = 0x1f0 },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_0, .metadata = PC3XX_GPR_AXI2PICO_0, .offset = 8, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_1, .metadata = PC3XX_GPR_AXI2PICO_1, .offset = 9, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_2, .metadata = PC3XX_GPR_AXI2PICO_2, .offset = 10, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_3, .metadata = PC3XX_GPR_AXI2PICO_3, .offset = 11, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_4, .metadata = PC3XX_GPR_AXI2PICO_4, .offset = 12, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_5, .metadata = PC3XX_GPR_AXI2PICO_5, .offset = 13, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_6, .metadata = PC3XX_GPR_AXI2PICO_6, .offset = 14, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_7, .metadata = PC3XX_GPR_AXI2PICO_7, .offset = 15, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_8, .metadata = PC3XX_GPR_AXI2PICO_8, .offset = 16, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_9, .metadata = PC3XX_GPR_AXI2PICO_9, .offset = 17, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_10, .metadata = PC3XX_GPR_AXI2PICO_10, .offset = 18, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_11, .metadata = PC3XX_GPR_AXI2PICO_11, .offset = 19, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_12, .metadata = PC3XX_GPR_AXI2PICO_12, .offset = 20, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_13, .metadata = PC3XX_GPR_AXI2PICO_13, .offset = 21, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_14, .metadata = PC3XX_GPR_AXI2PICO_14, .offset = 22, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_15, .metadata = PC3XX_GPR_AXI2PICO_15, .offset = 23, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_16, .metadata = PC3XX_GPR_AXI2PICO_16, .offset = 24, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_17, .metadata = PC3XX_GPR_AXI2PICO_17, .offset = 25, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_18, .metadata = PC3XX_GPR_AXI2PICO_18, .offset = 26, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_19, .metadata = PC3XX_GPR_AXI2PICO_19, .offset = 27, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_20, .metadata = PC3XX_GPR_AXI2PICO_20, .offset = 28, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_21, .metadata = PC3XX_GPR_AXI2PICO_21, .offset = 29, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_22, .metadata = PC3XX_GPR_AXI2PICO_22, .offset = 30, },
    { .type = PICO_RES_IRQ, .value = PC3XX_IRQ_AXI2PICO_23, .metadata = PC3XX_GPR_AXI2PICO_23, .offset = 31, },
    { .type = 0, .value = 0 }, /* Sentinel value. Do not remove and keep this
                                * at the end. */
};

/*! The offset in the AXI2Pico for GPR virtual ports. */
#define AXI2PICO_GPR_VP_START       ( 0x80 )

/*!
 * Add an IRQ handler to the PC3XX for a given interrupt source.
 *
 * @param pa The device to register the interrupt handler with.
 * @param irq The IRQ to attach the handler to.
 * @param callback The callback function to call when the interrupt is
 * raised.
 * @param cookie The cookie to pass to the callback function. This may be NULL
 * if it is not required.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_add_irq_handler( struct picoarray *pa,
                       struct pico_resource *irq,
                       int ( *callback )( struct pico_resource *irq,
                                          void *cookie ),
                       void *cookie )
{
    struct pc3xx_irq_handler *handler;
    struct pc3xx *dev = to_pc3xx( pa );
    int ret;
    unsigned gpr_num = irq->value - PC3XX_IRQ_AXI2PICO_0;
    unsigned offset =
            AXI2PICO_GPR_VP_START + ( gpr_num * AXI2PICO_VP_SPACING );
    u32 val = 0;
    unsigned long flags;

    if ( irq->type != PICO_RES_IRQ || !irq->exclusive || !callback )
        return -EINVAL;

    spin_lock_irqsave( &pa->lock, flags );

    /* Need to check that the port is bocking before enabling interrupts */
    pc3xx_axi2pico_reg_read( dev, offset + AXI2PICO_VP_STATUS_OFFSET,
                             &val);

    /* If the port is non-blocking then fail as we will receive constant
     * interrupts. We also only support receive GPRs for interrupt
     * generation. */
    if ( !IS_PORT_A_RECEIVE( val ) || IS_PORT_NONBLOCKING( val ) )
    {
        PRINTD( COMPONENT_PC3XX, DBG_WARN,
                "cannot enable IRQ: AXI2Pico VP is not"
                " configured as blocking receive" );
        PRINTD( COMPONENT_PC3XX, DBG_WARN,"offset=%d",offset);
        ret = -EIO;
        goto out;
    }

    ret = -ENOMEM;
    handler = kmalloc( sizeof( *handler ), GFP_ATOMIC );
    if ( !handler )
        goto out;

    handler->callback   = callback;
    handler->cookie     = cookie;
    handler->irq        = irq;

    list_add( &handler->list, &dev->axi2pico_irq_handlers.list );

    /* Enable the interrupt. */
    pc3xx_axi2pico_reg_read( dev, offset + AXI2PICO_VP_CONFIG_OFFSET, &val );
    val |= AXI2PICO_INT_EN;
    pc3xx_axi2pico_reg_write( dev, offset + AXI2PICO_VP_CONFIG_OFFSET, val );

    ret = 0;
out:

    spin_unlock_irqrestore( &pa->lock, flags );
    return ret;
}


/*!
 * Remove an IRQ handler from a PC3XX interrupt source.
 *
 * @param pa The PC3XX to remove the handler from.
 * @param irq The IRQ to remove the handler for.
 */
static void
pc3xx_remove_irq_handler( struct picoarray *pa,
                          struct pico_resource *irq )
{
    struct list_head *pos;
    struct list_head *tmp;
    struct pc3xx *dev = to_pc3xx( pa );
    struct pc3xx_irq_handler *handler;
    unsigned long flags;
    unsigned gpr_num = irq->value - PC3XX_IRQ_AXI2PICO_0;
    unsigned offset =
            AXI2PICO_GPR_VP_START + ( gpr_num * AXI2PICO_VP_SPACING );
    u32 val= 0;

    /*
     * Protect against interrupts being raised whilst we remove the handler.
     */
    spin_lock_irqsave( &pa->lock, flags );

    /* Disable the interrupt. */
    pc3xx_axi2pico_reg_read( dev, offset + AXI2PICO_VP_CONFIG_OFFSET, &val );
    val &= ~AXI2PICO_INT_EN;
    pc3xx_axi2pico_reg_write( dev, offset + AXI2PICO_VP_CONFIG_OFFSET, val );

    list_for_each_safe( pos, tmp, &dev->axi2pico_irq_handlers.list )
    {
        handler = container_of( pos, struct pc3xx_irq_handler, list );
        if ( handler->irq == irq )
        {
            list_del( pos );
            break;
        }
    }

    spin_unlock_irqrestore( &pa->lock, flags );
}

/*!
 * PC3XX AXI2Pico ISR. This ISR will be called when the AXI2Pico generates an
 * interrupt and checks all of the AXI2Pico interrupt sources for raised
 * interrupts and calls the appropriate handler if there is one registered. If
 * there is no interrupt handler for the IRQ source then that source is
 * disabled.
 *
 * @param irq The irq that has been raised.
 * @param dev The PC3XX device that has raised the interrupt.
 * @return Returns IRQ_HANDLED on success.
 */
static irqreturn_t
pc3xx_axi2pico_irq( int irq,
                    void *dev )
{
    struct pc3xx *pc3xxdev = dev;
    struct list_head *pos;
    struct pc3xx_irq_handler *handler;
    int ret;
    u32 int_status;
    unsigned i;
    unsigned handled;

    pc3xx_axi2pico_reg_read( pc3xxdev, AXI2PICO_INT_STATUS_OFFSET,
                             &int_status );
    for ( i = 0; i < AXI2PICO_NUM_VPS; ++i )
    {
        if ( int_status & ( 1 << i ) )
        {
            handled = 0;
            list_for_each( pos, &pc3xxdev->axi2pico_irq_handlers.list )
            {
                handler = container_of( pos, struct pc3xx_irq_handler, list );
                if ( ( handler->irq->offset ) == i  && handler->callback )
                {
                    ret = handler->callback( handler->irq, handler->cookie );
                    if ( !ret )
                    {
                        handled = 1;
                        break;
                    }
                }
            }
            if ( !handled )
            {
                PRINTD( COMPONENT_PC3XX, DBG_WARN,
                        "no interrupt handler for AXI2Pico VP %u", i );
                printk("disabling handler for AXI2Pico VP %u", i);
                /* Disable the interrupt generation for this GPR - there
                 * is no handler installed. */
                pc3xx_axi2pico_reg_write( pc3xxdev,
                       ( i * AXI2PICO_VP_SPACING ) + AXI2PICO_GPR_VP_START,
                        0 );
            }
        }
    }

    return IRQ_HANDLED;
}

/*!
 * Open and enable the specified DMA channel
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to open
 * @return 0 on success, a negative number on failure
 */
static int
pc3xx_dma_open( struct picoarray *pa,
                struct pico_resource *dma_channel )
{
    struct pc3xx *pc3xxdev = to_pc3xx( pa );
    struct pc3xx_dma_channel *dma =
        &pc3xxdev->dma_channel[dma_channel->value];
    u32 gprStatus = 0;

    if ( dma->stateActive )
    {
        PRINTD( COMPONENT_PC3XX, DBG_TRACE, "dma channel %d in use",
                dma_channel->value - PC3XX_DMA_AXI2PICO_0 );
        return -EBUSY;
    }

    PRINTD( COMPONENT_PC3XX, DBG_TRACE,
            "pa_dma %d  => cpu_dma %d",
            ( dma_channel->value-PC3XX_DMA_AXI2PICO_0 ),
            dma_channel->value);

    if ( ( dma_channel->value < PC3XX_DMA_AXI2PICO_0 ) ||
         ( dma_channel->value > PC3XX_DMA_AXI2PICO_7 ) )
        return -EINVAL;

    pc3xx_axi2pico_reg_read( pc3xxdev, dma_channel->offset
              + AXI2PICO_VP_STATUS_OFFSET, &gprStatus );

    if ( !IS_PORT_ENABLED(gprStatus) )
    {
        PRINTD( COMPONENT_PC3XX, DBG_ERROR, "dma %d: is not enabled",
           dma_channel->value);
        return -EINVAL;
    }

    /* Set the DMAC Enable bit (bit1) in the DMA config register */
    if ( IS_PORT_A_RECEIVE(gprStatus) )
       /* Enable the UL DMA channel by setting bit 1, and set the
           watermark to 1 x the burst size on bits 2 to 8.  */
       (void)pc3xx_axi2pico_reg_write( pc3xxdev,
           ((dma_channel->value - PC3XX_DMA_AXI2PICO_0) * AXI2PICO_VP_SPACING)
           + AXI2PICO_VP_CONFIG_OFFSET, 0x02 + (PC3XX_DMA_BURST_SIZE << 2));
    else
        /* Enable the DL DMA channel by setting bit 1, and set the watermark
           to 2 x the burst size on bits 2 to 8 */
        (void)pc3xx_axi2pico_reg_write( pc3xxdev,
         ((dma_channel->value-PC3XX_DMA_AXI2PICO_0)*AXI2PICO_VP_SPACING)
         +AXI2PICO_VP_CONFIG_OFFSET, 0x02 + ((2*PC3XX_DMA_BURST_SIZE) << 2));

    return 0;
}

/*!
 * Close and disable the specified DMA channel
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to close
 * @return 0 in all cases
 */
static int
pc3xx_dma_close( struct picoarray *pa,
                 struct pico_resource *dma_channel )
{
    struct pc3xx *pc3xxdev = to_pc3xx( pa );
    struct pc3xx_dma_channel *dma = &pc3xxdev->dma_channel[dma_channel->value];

    spin_lock_bh( &pa->lock );
    dma->callback = NULL;
    dma->stateActive = 0;
    spin_unlock_bh( &pa->lock );

    return 0;
}

/*!
 * DMA handler function. This function simply resets the channel active flag
 * and passes the cookie onto the handler specified in the pc3xx_to_device
 * and pc3xx_from_device functions.
 *
 * @param cookie The cookie to pass to the callback function.
 * @return 0 on success, non-zero on failure
 */
static void
pc3xx_dma_handler( void *cookie )
{
    struct pc3xx_dma_channel *dma = (struct pc3xx_dma_channel *)cookie;

    PRINTD( COMPONENT_PC3XX, DBG_TRACE, "dma transfer completed" );

    dma->stateActive = 0;
    dma->curr_desc = NULL;

    if ( dma->callback )
        dma->callback( dma->cookie, 0 );
}

/*!
 * DMA a scatter gather list of memory from a kernel mapped scatterlist
 * into a picoArray DMA channel. After the DMA transfer has completed, the
 * callback function will be called with the cookie as the parameter. The
 * caller of this function is responsible for mapping and unmapping the
 * buffers to be transferred into a DMA capable region.
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to use as a destination.
 * @param sgl The scatter gather list of the source data.
 * @param nents The number of entries in the scatterlist.
 * @param callback The callback function to be called when the transfer
 * has completed. The parameter errno will be set to the status of the DMA
 * operation where 0 == success, negative == failure.
 * @param cookie The cookie to pass to the callback function.
 * @return 0 on success, non-zero on failure
 */
static int
pc3xx_dma_to_device( struct picoarray *pa,
                     struct pico_resource *dma_channel,
                     struct scatterlist *sgl,
                     int nents,
                     int ( *callback )( void *cookie,
                                        int errno ),
                     void *cookie )
{
    int ret = 0;
    struct pc3xx *dev = to_pc3xx( pa );
    struct pc3xx_dma_channel *dma = &dev->dma_channel[dma_channel->value];
    dma_cookie_t dma_cookie;

    if ( dma->stateActive )
      return -EAGAIN;

    PRINTD( COMPONENT_DMA, DBG_TRACE, "DMA transfer Chan %d, sgl len %d",
            dma_channel->value, nents );

    dma->slave.fc = DW_DMA_FC_D_M2P;
    dma->curr_desc = dma->chan->device->device_prep_slave_sg( dma->chan, sgl,
                                nents, DMA_TO_DEVICE, DMA_PREP_INTERRUPT );
    if ( !dma->curr_desc )
    {
        PRINTD( COMPONENT_DMA, DBG_WARN, "failed to prep slave transfer" );
        ret = -EIO;
        goto out;
    }

    dma->curr_desc->callback = pc3xx_dma_handler;
    dma->curr_desc->callback_param = dma;

    dma->stateActive = 1;
    dma->callback = callback;
    dma->cookie = cookie;

    dma_cookie = dma->curr_desc->tx_submit( dma->curr_desc );
    if ( dma_submit_error( dma_cookie ) )
    {
        PRINTD( COMPONENT_DMA, DBG_WARN, "failed to start DMA" );
        ret = -EIO;
    }

out:

    return ret;
}

/*!
 * DMA a scatter gather list of memory from a picoArray DMA channel. After
 * the DMA transfer has completed, the callback function will be called
 * with the cookie as the parameter. The caller of this function is
 * responsible for mapping and unmapping the buffers to be transferred
 * into a DMA capable region.
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to use as a source.
 * @param sgl The scatter gather list of the destination buffer.
 * @param nents The number of entries in the scatterlist.
 * @param callback The callback function to be called when the transfer
 * has completed. The parameter errno will be set to the status of the DMA
 * operation where 0 == success, negative == failure.
 * @param cookie The cookie to pass to the callback function.
 * @return 0 on success, non-zero on failure
 */
static int
pc3xx_dma_from_device( struct picoarray *pa,
                       struct pico_resource *dma_channel,
                       struct scatterlist *sgl,
                       int nents,
                       int ( *callback )( void *cookie,
                                          int errno ),
                       void *cookie )
{
    int ret = 0;
    struct pc3xx *dev = to_pc3xx( pa );
    struct pc3xx_dma_channel *dma = &dev->dma_channel[dma_channel->value];
    dma_cookie_t dma_cookie;

    if ( dma->stateActive )
      return -EAGAIN;

    PRINTD( COMPONENT_PC3XX, DBG_TRACE, "DMA transfer Chan %d, sgl len %d",
          dma_channel->value, nents );

    dma->slave.fc = DW_DMA_FC_D_P2M;
    dma->curr_desc = dma->chan->device->device_prep_slave_sg( dma->chan, sgl,
                            nents, DMA_FROM_DEVICE, DMA_PREP_INTERRUPT );
    if ( !dma->curr_desc )
    {
        PRINTD( COMPONENT_PC3XX, DBG_WARN, "failed to prep slave transfer" );
        ret = -EIO;
        goto out;
    }

    dma->curr_desc->callback = pc3xx_dma_handler;
    dma->curr_desc->callback_param = dma;

    dma->stateActive = 1;
    dma->callback = callback;
    dma->cookie = cookie;

    dma_cookie = dma->curr_desc->tx_submit( dma->curr_desc );
    if ( dma_submit_error( dma_cookie ) )
    {
        PRINTD( COMPONENT_DMA, DBG_WARN, "failed to start DMA" );
        BUG();
        ret = -EIO;
    }

    dma->chan->device->device_issue_pending(dma->chan);

out:

    return ret;
}

/*!
 * Get the device type of a PC3XX.
 *
 * @param pa The device to query.
 * @return Always returns PICOARRAY_PC3XX.
 */
static enum picoarray_device_type
pc3xx_get_device_type( struct picoarray *pa )
{
    return PICOARRAY_PC3XX;
}

/*! Operations for the PC3XX devices. */
static struct picoarray_ops pc3xx_ops = {
    .sync                = pc3xx_sync,
    .start               = pc3xx_start,
    .stop                = pc3xx_stop,
    .get_device_type     = pc3xx_get_device_type,
    .config_read         = pc3xx_config_read,
    .config_write        = pc3xx_config_write,
    .register_read       = pc3xx_register_read,
    .register_write      = pc3xx_register_write,
    .reset               = pc3xx_reset,
    .get_resource        = generic_get_resource,
    .put_resource        = generic_put_resource,
    .destructor          = pc3xx_destroy,
    .add_irq_handler     = pc3xx_add_irq_handler,
    .remove_irq_handler  = pc3xx_remove_irq_handler,
    .dma_to_device       = pc3xx_dma_to_device,
    .dma_from_device     = pc3xx_dma_from_device,
    .dma_open            = pc3xx_dma_open,
    .dma_close           = pc3xx_dma_close,
    .pa_load             = pc3xx_pa_load,
};

static bool
dma_filter( struct dma_chan *chan,
            void *param )
{
    struct dw_dma_slave *dws = param;

    if ( !strcmp( dev_name( chan->device->dev ), "dw_dmac.0" ) )
    {
        chan->private = dws;
        return true;
    }

    return false;
}

static int
pc3xx_alloc_dma_channels( struct pc3xx *newdev )
{
    struct pc3xx_dma_channel *dma;
    int i;
    dma_cap_mask_t mask;

    for ( i = 0; i < PICO_NUM_DMA_CHANNELS; ++i )
    {
        dma = &newdev->dma_channel[ i ];

        dma->pico_addr = ( dma_addr_t )newdev->axi2pico_base_phys +
            ( i * AXI2PICO_VP_SPACING) + AXI2PICO_VP_DATA_OFFSET;

        /* Initialise all variables that may become set */
        dma_cap_zero( mask );
        dma_cap_set( DMA_SLAVE, mask );
        dma->slave = ( struct dw_dma_slave ) {
            .tx_reg     = dma->pico_addr,
            .rx_reg     = dma->pico_addr,
            .reg_width  = DW_DMA_SLAVE_WIDTH_32BIT,
            .cfg_hi     = DWC_CFGH_DST_PER( i ) |
                          DWC_CFGH_SRC_PER( i ),
            .cfg_lo     = 0,
            .src_master = ( i + 1 ) & 0x3,
            .dst_master = i & 0x3,
            .dma_dev    = newdev->axi2pico_dmac,
        };

        dma->chan = dma_request_channel( mask, dma_filter, &dma->slave );
        if ( !dma->chan )
        {
            PRINTD( COMPONENT_PC3XX, DBG_ERROR, "failed to open dma channel" );
            return -ENODEV;
        }
        dma->slave.tx_reg = dma->pico_addr;
        dma->slave.rx_reg = dma->pico_addr;

        dma->channel = i;
        dma->stateActive = 0;
        dma->callback = NULL;
        dma->cookie = NULL;
    }

    return 0;
}

/*!
 * Probe method for the PC3XX platform driver. This function creates a new
 * PC3XX instance and is responsible for allocating all of the resources
 * required.
 *
 * @param pdev The platform device that has been probed.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_probe( struct platform_device *pdev )
{
    struct resource *res;
    int ret;
    struct pc3xx *newdev = kzalloc( sizeof( *newdev ), GFP_ATOMIC );
    struct pc3xx_pdata *pc3xx_dmacs = pdev->dev.platform_data;

    if ( !newdev )
        return -ENOMEM;

    newdev->axi2pico_clk = clk_get( &pdev->dev, "axi2pico" );
    if ( IS_ERR( newdev->axi2pico_clk ) )
    {
        ret = PTR_ERR( newdev->axi2pico_clk );
        goto out;
    }

    ret = clk_enable( newdev->axi2pico_clk );
    if ( ret )
        goto out_put_clk;

    device_init_wakeup( &pdev->dev, 1 );

    ret = -ENOMEM;
    newdev->pa.resources = kmalloc( sizeof( pc3xx_resources ), GFP_ATOMIC );
    if ( !newdev->pa.resources )
        goto out_disable_clk;
    memcpy( newdev->pa.resources, pc3xx_resources, sizeof( pc3xx_resources ) );

    newdev->pa.dev_num = pdev->id;
    newdev->pa.ops = &pc3xx_ops;
    newdev->pa.features = PICOARRAY_HAS_DMA;
    newdev->pa.max_dma_sz = PICOARRAY_MAX_TRANSFER;
    newdev->axi2pico_dmac = pc3xx_dmacs->axi2pico_dmac;
    newdev->axi2cfg_dmac = pc3xx_dmacs->axi2cfg_dmac;
    spin_lock_init( &newdev->pa.lock );

    ret = -EINVAL;

    /* Get the IRQ for AXI2Pico GPRs. */
    res = platform_get_resource_byname( pdev, IORESOURCE_IRQ, "gpr_irq" );
    if ( !res )
        goto out;
    newdev->gpr_irq = res->start;

    /* Get the register base address for the lower AXI2CFG registers */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM, "procif" );
    if ( !res )
        goto out;
    newdev->axi2cfg1_base_phys = res->start;
    newdev->axi2cfg1_base_len = ( res->end - res->start ) + 1;

    /* Get the register base address for the upper AXi2CFG registers */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM,
                                        "procif2" );
    if ( !res )
        goto out;
    newdev->axi2cfg2_base_phys = res->start;
    newdev->axi2cfg2_base_len = ( res->end - res->start ) + 1;

    /* Get the register base address for the AXI2Pico. */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM,
                                        "ahb2pico_axi2pico" );
    if ( !res )
        goto out;
    newdev->axi2pico_base_phys = res->start;
    newdev->axi2pico_base_len = ( res->end - res->start ) + 1;

    /* Map the resources. */
    newdev->axi2cfg1_base = request_and_map( "axi2cfg1", newdev->axi2cfg1_base_phys,
                                        newdev->axi2cfg1_base_len );
    if ( !newdev->axi2cfg1_base )
        goto out;

    newdev->axi2pico_base = request_and_map( "axi2pico", newdev->axi2pico_base_phys,
                                        newdev->axi2pico_base_len );
    if ( !newdev->axi2pico_base )
        goto ahb2pico_map_failed;

    ret = request_irq( newdev->gpr_irq, pc3xx_axi2pico_irq, IRQF_DISABLED,
                       pdev->name, newdev );
    if ( ret )
        goto axi2pico_irq_failed;

    device_init_wakeup( &pdev->dev, 1 );
    if ( pc3xx_alloc_dma_channels( newdev ) )
    {
        ret = -EBUSY;
        goto dma_channels_failed;
    }

    /* Initialise the interrupt handler lists. */
    INIT_LIST_HEAD( &newdev->axi2pico_irq_handlers.list );

    ret = picoif_register_dev( &newdev->pa );
    if ( !ret )
        return 0;

dma_channels_failed:
    free_irq( newdev->gpr_irq, newdev );

axi2pico_irq_failed:
    unmap_and_release( newdev->axi2pico_base_phys, newdev->axi2pico_base_len,
                       newdev->axi2pico_base );

ahb2pico_map_failed:
    unmap_and_release( newdev->axi2cfg1_base_phys, newdev->axi2cfg1_base_len,
                       newdev->axi2cfg1_base );
out_disable_clk:
    clk_disable( newdev->axi2pico_clk );
out_put_clk:
    clk_put( newdev->axi2pico_clk );
out:
    if ( ret && newdev->pa.resources )
        kfree( newdev->pa.resources );

    if ( ret )
        kfree( newdev );
    else
        ++num_pc3xxs;

    return ret;
}

/*!
 * Remove method for the PC3XX platform driver. This method is called when the
 * platform driver is removed and must release all resources the driver has
 * been using.
 *
 * @param pdev The platform device being remove.
 * @return Returns zero on success, negative on failure.
 */
static int
pc3xx_remove( struct platform_device *pdev )
{
    struct picoarray *pa = picoif_get_device( pdev->id );
    struct pc3xx *pc3xxdev = to_pc3xx( pa );
    int ret = 0, i = 0;

    clk_disable( pc3xxdev->axi2pico_clk );
    clk_put( pc3xxdev->axi2pico_clk );

    for ( i = 0; i < PICO_NUM_DMA_CHANNELS; ++i )
    {
        if ( pc3xxdev->dma_channel[ i ].chan )
            dma_release_channel( pc3xxdev->dma_channel[ i ].chan );
    }

    free_irq( pc3xxdev->gpr_irq, pc3xxdev );
    unmap_and_release( pc3xxdev->axi2cfg1_base_phys, pc3xxdev->axi2cfg1_base_len,
                       pc3xxdev->axi2cfg1_base );
    unmap_and_release( pc3xxdev->axi2pico_base_phys, pc3xxdev->axi2pico_base_len,
                       pc3xxdev->axi2pico_base );

    kfree( pc3xxdev );

    return ret;
}

/*! The PC3XX platform driver.
 *  \todo Change the name to PC3XX specific rather than generic picoArray.
 */
static struct platform_driver pc3xx_driver = {
    .probe      = pc3xx_probe,
    .remove     = pc3xx_remove,
    .driver     = {
        .name   = "picoArray",
    },
};

int
pc3xx_init( void )
{
    return platform_driver_register( &pc3xx_driver );
}

/*!
 * Destructor to be called when a PC3XX is removed from picoif. This
 * function must decrement the number of PC3XXs registered, and when this
 * reaches zero, remove the platform driver.
 *
 * @param pa The device being removed.
 */
static void
pc3xx_destroy( struct picoarray *pa )
{
    PRINTD( COMPONENT_PC3XX, DBG_TRACE, "pA[%u]: destructor called",
            pa->dev_num );

    /* If we have no more pc3xxs, then remove the driver. */
    if ( 0 == --num_pc3xxs )
        platform_driver_unregister( &pc3xx_driver );
}
