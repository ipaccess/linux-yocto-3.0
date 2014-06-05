/*
 * linux/arch/arm/mach-picoxcell/board_ipa267.c
 *
 * Copyright (c) 2010 Picochip Ltd., Jamie Iles
 * Copyright (c) 2014 ip.access Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on linux/arch/arm/mach-picoxcell/board_pc7302.c
 *
 * All enquiries to support@ipaccess.com
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mtd/nand-gpio.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>

#include <mach/hardware.h>
#include <mach/picoxcell/axi2cfg.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "mux.h"
#include "picoxcell_core.h"

static long ipa267_panic_blink(int state)
{
	__raw_writel(state ? 0xFF : 0, IO_ADDRESS(PICOXCELL_GPIO_BASE +
					    GPIO_SW_PORT_C_DR_REG_OFFSET));
	return 0;
}

static void ipa267_panic_init(void)
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

	panic_blink = ipa267_panic_blink;
}

static struct resource ipa267_nand_resource[] = {
	{
		.start = EBI_CS2_BASE,
		.end   = EBI_CS2_BASE + 2 * SZ_1K,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = PICOXCELL_GPIO_BASE + 0x08,
		.end   = PICOXCELL_GPIO_BASE + 0x08 + 3,
		.flags = IORESOURCE_MEM,
	}
};

static struct mtd_partition ipa267_nand_parts[] = {
	{
		.name	= "Boot",
		.size	= 4 * SZ_128K,
		.offset	= 8 * SZ_128K,
	},
	{
		.name	= "Redundant Boot",
		.size	= 4 * SZ_128K,
		.offset	= 16 * SZ_128K,
	},
	{
		.name	= "Boot Environment",
		.size	= SZ_128K,
		.offset	= 24 * SZ_128K,
	},
	{
		.name	= "Redundant Boot Environment",
		.size	= SZ_128K,
		.offset	= MTDPART_OFS_APPEND,
	},
	{
		.name	= "Kernel",
		.size	= 8 * SZ_1M,
		.offset	= (28 * SZ_128K),
	},
	{
		.name	= "File System",
		.size	= MTDPART_SIZ_FULL,
		.offset	= MTDPART_OFS_APPEND,
	},
};

static struct gpio_nand_platdata ipa267_nand_platdata = {
	.gpio_rdy   = PC3X2_GPIO_PIN_ARM_1,
	.gpio_nce   = PC3X2_GPIO_PIN_ARM_2,
	.gpio_ale   = PC3X2_GPIO_PIN_ARM_3,
	.gpio_cle   = PC3X2_GPIO_PIN_ARM_4,
	.gpio_nwp   = -1,
	.parts	    = ipa267_nand_parts,
	.num_parts  = ARRAY_SIZE(ipa267_nand_parts),
};

static struct platform_device ipa267_nand = {
	.name		    = "gpio-nand",
	.num_resources	    = ARRAY_SIZE(ipa267_nand_resource),
	.resource	    = ipa267_nand_resource,
	.id		    = -1,
	.dev.platform_data  = &ipa267_nand_platdata,
};

/*
 * We have two i2c busses, and both are driven using GPIO.
 */
static struct i2c_gpio_platform_data ipa267_i2c_bus0_data = {
	.sda_pin = PC3X3_GPIO_PIN_ARM_7,
	.scl_pin = PC3X3_GPIO_PIN_ARM_0,
	.udelay  = 2, /* Between 100kHz and 400kHz, nominally 250kHz */
	.timeout = 100
};

static struct platform_device ipa267_i2c_bus0_device = {
	.name = "i2c-gpio",
	.id = 0, /* should be 0 as I read the code... */
	.dev = {
		.platform_data = &ipa267_i2c_bus0_data,
	}
};

static struct i2c_board_info __initdata ipa267_i2c_bus0_devices[] = {
	{ I2C_BOARD_INFO("max6635",   0x4B), },
	{ I2C_BOARD_INFO("atmel_twi", 0x29), },
	{ I2C_BOARD_INFO("ad7995",    0x28), },
	{ I2C_BOARD_INFO("micrel",    0x5F), },
	{ I2C_BOARD_INFO("lm75",      0x48), }
};

static struct i2c_gpio_platform_data ipa267_i2c_bus1_data = {
	.sda_pin = PC3X3_GPIO_PIN_ARM_44,
	.scl_pin = PC3X3_GPIO_PIN_ARM_43,
	.udelay  = 2, /* Between 100kHz and 400kHz, nominally 250kHz */
	.timeout = 100
};

static struct platform_device ipa267_i2c_bus1_device = {
	.name = "i2c-gpio",
	.id = 1,
	.dev = {
		.platform_data = &ipa267_i2c_bus1_data,
	}
};

static struct i2c_board_info __initdata ipa267_i2c_bus1_devices[] = {
	{ I2C_BOARD_INFO("at91_i2c",  0x49), },
	{ I2C_BOARD_INFO("ipa2g",     0x48), }, /* smells funny */
};

static void ipa267_init_nand(void)
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

	platform_device_register(&ipa267_nand);
}

FIXED_CLK(ipa267_uart,	3686400, -1, NULL);
static struct clk_lookup ipa267_uart_lookup = CLK_LOOKUP(NULL, "uart",
							 &ipa267_uart_clk);

static void ipa267_register_uarts(void)
{
	picoxcell_clk_add(&ipa267_uart_clk);
	clkdev_add(&ipa267_uart_lookup);
	picoxcell_add_uart(PICOXCELL_UART1_BASE, IRQ_UART1, 0);
	picoxcell_add_uart(PICOXCELL_UART2_BASE, IRQ_UART2, 1);
}

/*
 * PC7302 platforms have had a variety of different SPI Flash devices fitted.
 *
 * Spansion S25FL128P (128 Mbit) devices
 * Numonyx M25P05 (512 Kbit) devices
 *
 * This setup should be fine for all of them with the proviso that the
 * partition called "SPI: Data" may not actually be available for use.
 */
static struct mtd_partition ipa267_spi_flash_partitions[] = {
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

static struct flash_platform_data ipa267_spi_flash_data = {
	.name			= "ipa267 spi flash",
	.parts			= ipa267_spi_flash_partitions,
	.nr_parts		= ARRAY_SIZE(ipa267_spi_flash_partitions),
};

/*
 * SPI Board Data for SPI bus 0 (picoxcell-spi)
 */
static struct spi_board_info ipa267_spi_board_info[] __initdata = {
	{
		.modalias	= "m25p80",
		.platform_data	= &ipa267_spi_flash_data,
		.mode		= SPI_MODE_3,
		.max_speed_hz	= 2000000,
		.bus_num	= 0,
		.chip_select	= 0,
	}
	/* CS1: EBI: Debug interface */
	/* CS2: SPI: Reference Clock Control DAC -- dac7512? */
	/* CS3: SPI: Thermal Sensor -- lm75? */
};

static struct platform_device *ipa267_devices[] __initdata = {
        /* &ipa267_nand, -- first sanitise the NAND setup and stop doing duplicate work */
        &ipa267_i2c_bus0_device,
        &ipa267_i2c_bus1_device,
};

/*
 * TODO: SPI Board Data for SPI bus 1 (Radio SPI, GPIO)
 *
 * CS0: ADI/Aux Radio Mux 0
 * CS1: Aux Radio Mux 1
 * CS2: Aux Radio Mux 2
 * CS3: Aux Radio Mux 3
 */

static void ipa267_mux_gpios(void)
{
	int err;
	const struct mux_cfg brd267_cfg[] = {
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
		pr_err("ipa267_mux_gpios: unable to mux gpios\n");
		return;
	}
}

static void __init ipa267_init(void)
{
	picoxcell_tsu_init(20000000);
	picoxcell_core_init();

	ipa267_register_uarts();
	ipa267_init_nand();
	ipa267_mux_gpios();
	ipa267_panic_init();
	platform_add_devices(ipa267_devices, ARRAY_SIZE(ipa267_devices));
	spi_register_board_info(ipa267_spi_board_info,
				ARRAY_SIZE(ipa267_spi_board_info));
	i2c_register_board_info(0, ipa267_i2c_bus0_devices,
				ARRAY_SIZE(ipa267_i2c_bus0_devices));
	i2c_register_board_info(1, ipa267_i2c_bus1_devices,
				ARRAY_SIZE(ipa267_i2c_bus1_devices));
}

MACHINE_START(IPA267, "IPA267")
	.map_io		= picoxcell_map_io,
	.init_irq	= picoxcell_init_irq,
	.init_early	= picoxcell_init_early,
	.timer		= &picoxcell_sys_timer,
	.init_machine	= ipa267_init,
MACHINE_END
