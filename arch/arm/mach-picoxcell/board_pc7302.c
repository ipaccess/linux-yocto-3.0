/*
 * linux/arch/arm/mach-picoxcell/board_pc7302.c
 *
 * Copyright (c) 2010 Picochip Ltd., Jamie Iles
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mtd/nand-gpio.h>
#if !defined(CONFIG_IPACCESS_IP3XXFF) || defined(CONFIG_IPACCESS_IP3XXFF_267)
#include <linux/mtd/physmap.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#endif
#ifdef CONFIG_IPACCESS_IP3XXFF
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio.h>
#include <mach/spi-gpio.h>
#endif

#include <mach/hardware.h>
#include <mach/picoxcell/axi2cfg.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "mux.h"
#include "picoxcell_core.h"

#if defined(CONFIG_IPACCESS_IP3XXFF) && (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE))
/* i2c */
static struct i2c_gpio_platform_data pc3xx_i2c_bus_data = { 
#if defined(CONFIG_IPACCESS_IP3XXFF_267)
        .sda_pin = PC3X3_GPIO_PIN_ARM_7,
#else
        .sda_pin = PC3X3_GPIO_PIN_ARM_2,
#endif
        .scl_pin = PC3X3_GPIO_PIN_ARM_0,
        .udelay  = 2,  /* Between 100kHz and 400kHz, nominally 250kHz */
        .timeout = 100,
};

static struct platform_device pc3xx_i2c_bus_device = { 
        .name           = "i2c-gpio",
        .id             = 0,
        .dev = { 
                .platform_data = &pc3xx_i2c_bus_data,
        }   
};

static struct i2c_board_info __initdata pc3xx_i2c_devices[] = { 
        {   
                I2C_BOARD_INFO("max6635",   0x4B),
        },  
        {   
                I2C_BOARD_INFO("atmel_twi", 0x29),
        },  
        {   
                I2C_BOARD_INFO("ad7995",    0x28),
        },  
        {   
                I2C_BOARD_INFO("micrel",    0x5F),
        },  
    {   
        I2C_BOARD_INFO("lm75",    0x48),
    },  

};

#if defined(CONFIG_IPACCESS_IP3XXFF_267)
/* i2c */
static struct i2c_gpio_platform_data pc3xx_ipa2g_i2c_bus_data = { 
        .sda_pin = PC3X3_GPIO_PIN_ARM_44,
        .scl_pin = PC3X3_GPIO_PIN_ARM_43,
        .udelay  = 2,  /* Between 100kHz and 400kHz, nominally 250kHz */
        .timeout = 100,
};

static struct platform_device pc3xx_ipa2g_i2c_bus_device = { 
        .name           = "i2c-gpio",
        .id             = 1,
        .dev = { 
                .platform_data = &pc3xx_ipa2g_i2c_bus_data,
        }   
};

static struct i2c_board_info __initdata pc3xx_ipa2g_i2c_devices[] = {
   {
        I2C_BOARD_INFO("at91_i2c",    0x49),
    },
    {
        I2C_BOARD_INFO("ipa2g",    0x48),
    },
};
#endif

#endif /* CONFIG_IPACCESS_IP3XXFF && (CONFIG_I2C_GPIO || CONFIG_I2C_GPIO_MODULE) */


#if defined(CONFIG_IPACCESS_IP3XXFF) && (defined(CONFIG_SPI_PICOXCELL_IPA) || defined(CONFIG_SPI_PICOXCELL_IPA_MODULE))

/* SPI:
*/

/* According to sources on the internet, the Synopsys device will deselect
 * the chip select between words in SPI modes 0 and 2, so we have to use
 * modes 3 and 1 respectively instead.
 */
static struct spi_board_info ip3xxff_spi_board_info[] = {
    [0] = { /* Thermal Sensor */
        .modalias       = "max6662",
        .bus_num        = 0,
        .chip_select    = 3,
        .max_speed_hz   = 10000, /* 10kHz */
        .mode           = SPI_3WIRE | SPI_MODE_3,  /* Idle clock is high.  Latch on rising edge (second edge) */
        .platform_data  = NULL,
    },
    [1] = { /* Reference Clock Control DAC */
        .modalias       = "dac7512",
        .bus_num        = 0,
        .chip_select    = 2,
        .max_speed_hz   = 10000, /* 10kHz (not 20MHz, we'd get only 80 CPU cycles between writes) */
        .mode           = SPI_MODE_1,  /* Idle clock is low.  Latch on falling edge (second edge) */
        .platform_data  = NULL,
    },
};


static void
pc3xxspi_platform_release( struct device *dev )
{
    /* This function is intentionally left blank. */
}

static struct resource pc3xxspi_resources[] = {
    {
        .start = PICOXCELL_SSI_BASE,
        .end   = PICOXCELL_SSI_BASE + 0xffff,
        .flags = IORESOURCE_MEM,
    },
    {
        .start = IRQ_SSI,
        .end   = IRQ_SSI,
        .flags = IORESOURCE_IRQ,
    },
};

static struct pc3xx_spi_info pc3xxspi_platform_data = {
    .cs0_active     = 0,  /* EBI: NOR Flash */
    .cs1_active     = 0,  /* EBI: Debug interface */
    .cs2_active     = 1,  /* SPI: Reference Clock Control DAC */
    .cs3_active     = 1,  /* SPI: Thermal Sensor */
    .board_size     = ARRAY_SIZE(ip3xxff_spi_board_info),
    .board_info     = ip3xxff_spi_board_info,
};


static struct platform_device pc3xxspi_device = {
    .name = "pc302-spi",
    .id = 0,
    .dev = {
        .coherent_dma_mask = 0xffffffff,
        .release = pc3xxspi_platform_release,
        .platform_data = &pc3xxspi_platform_data,
    },
    .num_resources = ARRAY_SIZE( pc3xxspi_resources ),
    .resource = pc3xxspi_resources,
};

#endif /* CONFIG_IPACCESS_IP3XXFF && (CONFIG_SPI_PICOXCELL_IPA || CONFIG_SPI_PICOXCELL_IPA_MODULE) */

#if defined(CONFIG_MTD_NAND_GPIO) || defined(CONFIG_MTD_NAND_GPIO_MODULE)
static struct resource pc7302_nand_resource[] = {
        {
                .start = EBI_CS2_BASE,
                .end   = EBI_CS2_BASE + (2 * SZ_1K),
                .flags = IORESOURCE_MEM,
        },
};

static struct mtd_partition pc7302_nand_parts[] = {
        {
                .name   = "Boot",
                .size   = 4 * SZ_128K,
                .offset = 0,
        },
        {
                .name   = "Redundant Boot",
                .size   = 4 * SZ_128K,
                .offset = MTDPART_OFS_APPEND,
        },
        {
                .name   = "Boot Environment",
                .size   = SZ_128K,
                .offset = MTDPART_OFS_APPEND,
        },
        {
                .name   = "Redundant Boot Environment",
                .size   = SZ_128K,
                .offset = MTDPART_OFS_APPEND,
        },
        {
                .name   = "Kernel",
                .size   = 8 * SZ_1M,
                .offset = (12 * SZ_128K),
        },
        {
                .name   = "File System",
                .size   = MTDPART_SIZ_FULL,
                .offset = MTDPART_OFS_APPEND,
        },
};

static struct gpio_nand_platdata pc7302_nand_platdata = {
        .gpio_rdy   = PC3X3_GPIO_PIN_ARM_1,
        .gpio_nce   = PC3X3_GPIO_PIN_ARM_2,
        .gpio_ale   = PC3X3_GPIO_PIN_ARM_3,
        .gpio_cle   = PC3X3_GPIO_PIN_ARM_4,
        .gpio_nwp   = -1,
        .parts      = pc7302_nand_parts,
        .num_parts  = ARRAY_SIZE(pc7302_nand_parts),
};

static struct platform_device pc7302_nand = {
        .name               = "gpio-nand",
        .num_resources      = ARRAY_SIZE(pc7302_nand_resource),
        .resource           = pc7302_nand_resource,
        .id                 = -1,
        .dev.platform_data  = &pc7302_nand_platdata,
};

static void __init pc7302_init_nand(void)
{
	struct clk *ebi_clk = clk_get(NULL, "ebi");
	int err;
	const struct mux_cfg pc3x2_cfg[] = {
		MUXCFG("arm4", MUX_ARM),
	};
	const struct mux_cfg pc3x3_cfg[] = {
		MUXCFG("pai_tx_data0", MUX_PERIPHERAL_PAI),
		MUXCFG("ebi_addr22", MUX_ARM),
	};

	if (picoxcell_is_pc3x3())
		err = mux_configure_table(pc3x3_cfg, ARRAY_SIZE(pc3x3_cfg));
	else
		err = mux_configure_table(pc3x2_cfg, ARRAY_SIZE(pc3x2_cfg));
	if (err) {
		pr_err("unable to set ebi_addr22 for use as gpio-nand cle\n");
		return;
	}

	if (IS_ERR(ebi_clk)) {
		pr_err("failed to get EBI clk, unable to register NAND flash\n");
		return;
	}

	if (clk_enable(ebi_clk)) {
		pr_err("failed to enable EBI clk, unable to register NAND flash\n");
		clk_put(ebi_clk);
		return;
	}

        platform_device_register(&pc7302_nand);
}
#else
#define pc7302_init_nand()
#endif

#if defined(CONFIG_SPI_PICOXCELL) || defined(CONFIG_SPI_PICOXCELL_MODULE)
/*
 * PC7302 platforms have had a variety of different SPI Flash devices fitted.
 *
 * Spansion S25FL128P (128 Mbit) devices
 * Numonyx M25P05 (512 Kbit) devices
 *
 * This setup should be fine for all of them with the proviso that the
 * partition called "SPI: Data" may not actually be available for use.
 */
static struct mtd_partition pc7302_spi_flash_partitions[] = {
	{
		.name		= "SPI: First Stage Boot Loader",
		.size		= SZ_64K,
		.offset		= 0,
	},
	{
		.name		= "SPI: Data",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,
	},
};

static struct flash_platform_data pc7302_spi_flash_data = {
	.name			= "pc7302 spi flash",
	.parts			= pc7302_spi_flash_partitions,
	.nr_parts		= ARRAY_SIZE(pc7302_spi_flash_partitions),
};

static struct spi_board_info pc7302_spi_board_info[] __initdata = {
	{
		.modalias	= "m25p80",
		.platform_data	= &pc7302_spi_flash_data,
		.mode		= SPI_MODE_3,
		.max_speed_hz	= 2000000,
		.chip_select	= 0,
	}
};

static long pc7302_panic_blink(int state)
{
	__raw_writel(state ? 0xFF : 0, IO_ADDRESS(PICOXCELL_GPIO_BASE +
					    GPIO_SW_PORT_C_DR_REG_OFFSET));
	return 0;
}

static void pc7302_panic_init(void)
{
	/*
	 * We have a BOOT_ERROR pin on PC7302. Reuse that for signalling when
	 * the kernel panics. There is only 1 bit wired up to port C but it
	 * won't hurt to configure all of them.
	 */
	__raw_writel(0xF, IO_ADDRESS(PICOXCELL_GPIO_BASE +
			       GPIO_SW_PORT_C_DDR_REG_OFFSET));
	__raw_writel(0x0, IO_ADDRESS(PICOXCELL_GPIO_BASE +
			       GPIO_SW_PORT_C_CTL_REG_OFFSET));

	panic_blink = pc7302_panic_blink;
}
#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP_MODULE)
static struct mtd_partition pc7302_nor_partitions[] = {
	{
		.name		= "Boot",
		.size		= SZ_256K,
		.offset		= 0,
	},
	{
		.name		= "Boot Environment",
		.size		= SZ_128K,
		.offset		= MTDPART_OFS_APPEND,
	},
	{
		.name		= "Kernel",
		.size		= SZ_4M,
		.offset		= MTDPART_OFS_APPEND,
	},
	{
		.name		= "Application",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,
	},
};

static struct physmap_flash_data pc7302_nor_flash_data = {
	.width		= 1,
	.parts		= pc7302_nor_partitions,
	.nr_parts	= ARRAY_SIZE(pc7302_nor_partitions)
};

static struct resource pc7302_nor_resource = {
	.start	= PICOXCELL_FLASH_BASE,
	.end	= PICOXCELL_FLASH_BASE + SZ_128M - 1,
	.flags	= IORESOURCE_MEM,
};

static struct platform_device pc7302_nor = {
	.name		    = "physmap-flash",
	.id		    = -1,
	.dev.platform_data  = &pc7302_nor_flash_data,
	.resource	    = &pc7302_nor_resource,
	.num_resources	    = 1,
};

static void pc7302_init_nor(void)
{
	struct clk *ebi_clk = clk_get(NULL, "ebi");

	if (IS_ERR(ebi_clk)) {
		pr_err("failed to get EBI clk, unable to register NOR flash\n");
		return;
	}

	if (clk_enable(ebi_clk)) {
		pr_err("failed to enable EBI clk, unable to register NOR flash\n");
		clk_put(ebi_clk);
		return;
	}

	platform_device_register(&pc7302_nor);
}
#else
#define pc7302_init_nor()
#endif

#else /* defined(CONFIG_SPI_PICOXCELL) || defined(CONFIG_SPI_PICOXCELL_MODULE) */
#define pc7302_panic_init()
#define pc7302_init_nor()
#endif /* defined(CONFIG_SPI_PICOXCELL) || defined(CONFIG_SPI_PICOXCELL_MODULE) */


FIXED_CLK(pc7302_uart,	3686400, -1, NULL);
static struct clk_lookup pc7302_uart_lookup = CLK_LOOKUP(NULL, "uart",
							 &pc7302_uart_clk);

static void pc7302_register_uarts(void)
{
	picoxcell_clk_add(&pc7302_uart_clk);
	clkdev_add(&pc7302_uart_lookup);
	picoxcell_add_uart(PICOXCELL_UART1_BASE, IRQ_UART1, 0);
	picoxcell_add_uart(PICOXCELL_UART2_BASE, IRQ_UART2, 1);
}

#if defined(CONFIG_IPACCESS_IP3XXFF_267) 
static void ipa3xx_mux_gpios(void)
{
	int err;
	const struct mux_cfg brd267_cfg[] = {
    // TODO :references taken from the the MKS 403711:3,Need to verify their impact
	MUXCFG("arm_gpio16",     MUX_ARM), 
	MUXCFG("arm_gpio17",     MUX_ARM), 
	MUXCFG("arm_gpio18",     MUX_ARM), 
	MUXCFG("arm_gpio19",     MUX_ARM), 

	MUXCFG("pai_tx_data0", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_tx_data1", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_tx_data2", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_tx_data3", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_tx_data4", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_tx_data5", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_tx_data6", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_tx_data7", MUX_PERIPHERAL_PAI), /* PAI Iface */

	MUXCFG("pai_rx_data0", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_rx_data1", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_rx_data2", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_rx_data3", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_rx_data4", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_rx_data5", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_rx_data6", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("pai_rx_data7", MUX_PERIPHERAL_PAI), /* PAI Iface */
	MUXCFG("ebi_addr14",   MUX_ARM),
	MUXCFG("ebi_addr15",   MUX_ARM),
	MUXCFG("ebi_addr16",   MUX_ARM),
	MUXCFG("ebi_addr17",   MUX_ARM),
	MUXCFG("ebi_addr18",   MUX_ARM),
	MUXCFG("ebi_addr19",   MUX_ARM),
	MUXCFG("ebi_addr20",   MUX_ARM),
	MUXCFG("ebi_addr21",   MUX_ARM),
	MUXCFG("ebi_addr22",   MUX_ARM),
	MUXCFG("ebi_addr23",   MUX_ARM),
	MUXCFG("ebi_addr24",   MUX_ARM),
	MUXCFG("ebi_addr25",   MUX_ARM),
	MUXCFG("ebi_addr26",   MUX_ARM),

	MUXCFG("ebi_clk_pin", MUX_ARM),
	MUXCFG("max_tx_ctrl", MUX_ARM),
	MUXCFG("max_ref_clk", MUX_ARM),
	MUXCFG("max_trig_clk", MUX_ARM),
	};

	err = mux_configure_table(brd267_cfg, ARRAY_SIZE(brd267_cfg));

	if (err) {
		pr_err("unable to mux gpios\n");
		return;
	}
}

#endif

static void __init pc7302_init(void)
{
	picoxcell_tsu_init(20000000);
	picoxcell_core_init();

	pc7302_register_uarts();

	if ((axi2cfg_readl(AXI2CFG_SYSCFG_REG_OFFSET) & 0x3) == 0)
		pc7302_init_nor();
	else
		pc7302_init_nand();

#if defined(CONFIG_IPACCESS_IP3XXFF_267)
	ipa3xx_mux_gpios();
#endif

	pc7302_panic_init();

#if defined(CONFIG_SPI_PICOXCELL) || defined(CONFIG_SPI_PICOXCELL_MODULE)
	spi_register_board_info(pc7302_spi_board_info,
				ARRAY_SIZE(pc7302_spi_board_info));
#endif

#if defined(CONFIG_IPACCESS_IP3XXFF) && (defined(CONFIG_SPI_PICOXCELL_IPA) || defined(CONFIG_SPI_PICOXCELL_IPA_MODULE))
    platform_device_register(&pc3xxspi_device);
#endif

#if defined(CONFIG_IPACCESS_IP3XXFF) && (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE))
    platform_device_register(&pc3xx_i2c_bus_device);
    i2c_register_board_info(0, pc3xx_i2c_devices, ARRAY_SIZE(pc3xx_i2c_devices));

#if defined(CONFIG_IPACCESS_IP3XXFF_267)
    platform_device_register(&pc3xx_ipa2g_i2c_bus_device);
    i2c_register_board_info(1, pc3xx_ipa2g_i2c_devices, ARRAY_SIZE(pc3xx_ipa2g_i2c_devices));
#endif
#endif

}

MACHINE_START(PC7302, "PC7302")
	.map_io		= picoxcell_map_io,
	.init_irq	= picoxcell_init_irq,
	.init_early	= picoxcell_init_early,
	.timer		= &picoxcell_sys_timer,
	.init_machine	= pc7302_init,
MACHINE_END
