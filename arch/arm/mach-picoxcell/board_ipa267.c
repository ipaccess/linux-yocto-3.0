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
#include <linux/spi/spi_gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>

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
 * Add SPI Busses
 */
struct spi_gpio_platform_data ipa267_spi_gpio_bus1_platform_data = {
	.sck            = PC3X3_GPIO_PIN_ARM_10,
	.miso           = PC3X3_GPIO_PIN_ARM_11,
	.mosi           = PC3X3_GPIO_PIN_ARM_12,
	.num_chipselect = 3,
};

static struct platform_device ipa267_spi_gpio_bus1_device = {
	.name = "spi_gpio",
	.id = 1,
	.dev = {
		.platform_data = &ipa267_spi_gpio_bus1_platform_data,
	}
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
	.id = 0,
	.dev = {
		.platform_data = &ipa267_i2c_bus0_data,
	}
};

static struct i2c_board_info __initdata ipa267_i2c_bus0_devices[] = {
	{ I2C_BOARD_INFO("max6635",   0x4B), }, /* maxim phy? */
	{ I2C_BOARD_INFO("atmel_twi", 0x29), }, /* see at91 stuff, do we have this? */
	{ I2C_BOARD_INFO("ad7995",    0x28), }, /* we do have one of these, but where is it attached? */
	{ I2C_BOARD_INFO("micrel",    0x5F), }, /* micrel phy I assume? */
	{ I2C_BOARD_INFO("lm75",      0x48), } /* lm75 temperature sensor? */
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
	{ I2C_BOARD_INFO("ipa2g",     0x48), },
};

static void ipa267_init_nand(void)
{
	struct clk *ebi_clk = clk_get(NULL, "ebi");
	int err;
	const struct mux_cfg pc3x3_cfg[] = {
		MUXCFG("pai_tx_data0", MUX_PERIPHERAL_PAI),
		MUXCFG("ebi_addr22", MUX_ARM),
	};

	err = mux_configure_table(pc3x3_cfg, ARRAY_SIZE(pc3x3_cfg));
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
	.name			= "spi-flash",
	.parts			= ipa267_spi_flash_partitions,
	.nr_parts		= ARRAY_SIZE(ipa267_spi_flash_partitions),
	.type			= "m25p40",
};

/*
 * SPI Board Data for SPI bus 0 (picoxcell-spi)
 */
static struct spi_board_info ipa267_spi_board_info[] __initdata = {
	/*
	 * CS0 on picoxcell-spi is the SPI Flash
	 */
	{
		.modalias	= "m25p80",
		.platform_data	= &ipa267_spi_flash_data,
		.mode		= SPI_MODE_3,
		.max_speed_hz	= 2000000,
		.bus_num	= 0,
		.chip_select	= 0,
	},
	/*
	 * CS1: EBI: Debug interface
	 */
	/*
	 * CS2: SPI: Reference Clock Control DAC
	 */
	{
		.modalias	= "dac7512",
		.platform_data	= NULL,
		.mode		= SPI_MODE_1,  /* Idle clock is low.  Latch on falling edge (second edge) */
		.max_speed_hz	= 10000, /* 10kHz (not 20MHz, we'd get only 80 CPU cycles between writes) */
		.bus_num	= 0,
		.chip_select	= 2,
	},
	/*
	 * CS3: SPI: Thermal Sensor
	 */
	{
		.modalias	= "max6662",
		.platform_data	= NULL,
		.mode		= SPI_3WIRE | SPI_MODE_3,  /* Idle clock is high.  Latch on rising edge (second edge) */
		.max_speed_hz	= 10000, /* 10kHz */
		.bus_num	= 0,
		.chip_select	= 3,
	}
};

static struct platform_device *ipa267_devices[] __initdata = {
	&ipa267_spi_gpio_bus1_device,
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

static struct mux_def ipa267_mux[] = {
	MUXGPIO(arm_gpio16,    16,       0,     RSVD,   0x34,  16,      -1,     -1,     0),
	MUXGPIO(arm_gpio17,    17,       1,     RSVD,   0x34,  17,      -1,     -1,     0),
	MUXGPIO(arm_gpio18,    18,       2,     RSVD,   0x34,  18,      -1,     -1,     0),
	MUXGPIO(arm_gpio19,    19,       3,     RSVD,   0x34,  19,      -1,     -1,     0),
};

static void ipa267_cfgmux(void)
{
	int err;

	const struct mux_cfg brd267_cfg[] = {
		MUXCFG("arm_gpio16",     MUX_ARM),
		MUXCFG("arm_gpio17",     MUX_ARM),
		MUXCFG("arm_gpio18",     MUX_ARM),
		MUXCFG("arm_gpio19",     MUX_ARM),

		/*
		 * SPI-RF (SPI Bus 1)
		 *
		 * Confusion Reigns - picoXcell docs say this is the immutable mapping:
		 *  ARM_8  : [shd_gpio]        : SPI-RF MISO
		 *  ARM_9  : [boot_mode0]      : SPI-RF MOSI
		 *  ARM_10 : [boot_mode1]      : SPI-RF SCK
		 *  ARM_11 : [sdram_speed_sel] : SPI-RF CS0 / fast GPIO
		 *  ARM_12 : [mii_rev_en]      : SPI-RF CS1 / fast GPIO
		 *  ARM_13 : [mii_rmii_en]     : fast GPIO
		 *  ARM_14 : [mii_speed_sel]   : SPI-RF CS2 / fast GPIO
		 *
		 * Further, they state:
		 *  For customers to make use of the benefits of the Radio API, it is
		 *  mandatory that the customers leave GPIOs (8-14) unused so that the
		 *  Radio API can drive the SPI Interface.
		 *
		 * So what gives? Have we changed things this much?
		 */
		MUXCFG("shd_gpio",       MUX_ARM), /* ARM_8 : SPI-RF CS0 */
		MUXCFG("boot_mode0",     MUX_ARM), /* ARM_9 : SPI-RF CS1 */
		MUXCFG("boot_mode1",     MUX_ARM), /* ARM_10: SPI-RF SCK */
		MUXCFG("sdram_speed_sel",MUX_ARM), /* ARM_11: SPI-RF MISO */
		MUXCFG("mii_rev_en",     MUX_ARM), /* ARM_12: SPI-RF MOSI */
		MUXCFG("mii_rmii_en",    MUX_ARM), /* ARM_13: SPI-RF fast GPIO */
		MUXCFG("mii_speed_sel",  MUX_ARM), /* ARM_14: SPI-RF CS2 / fast GPIO */
		MUXCFG("arm_gpio16",     MUX_ARM), /* ARM_16: SPI-RF CS2 */

#if 0
		/*
		 * SPI-AUX (SPI Bus 2) - this interferes with the SPI flash - it's quite likely the decode pins and wonky
		 */
		MUXCFG("decode0",        MUX_ARM), /* ARM_36: SPI-AUX CS0 */
		MUXCFG("decode1",        MUX_ARM), /* ARM_37: SPI-AUX CS1 */
		MUXCFG("decode2",        MUX_ARM), /* ARM_38: SPI-AUX CS2 */
		MUXCFG("decode3",        MUX_ARM), /* ARM_39: SPI-AUX CS3 */
		MUXCFG("ssi_clk",        MUX_ARM), /* ARM_40: SPI-AUX SCK */
		MUXCFG("ssi_data_in",    MUX_ARM), /* ARM_41: SPI-AUX MISO */
		MUXCFG("ssi_data_out",   MUX_ARM), /* ARM_42: SPI-AUX MOSI */
#endif
		/*
		 * I2C-GPIO Bus 0
		 */
#if 1
		MUXCFG("arm_gpio0",      MUX_ARM), /* ARM_0 : I2C-AUX SCL */
#endif
#if 1
		MUXCFG("pai_tx_data0", MUX_PERIPHERAL_PAI), /* PAI Iface */ /* needed by ipa267_init_nand */
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
		MUXCFG("ebi_addr22",   MUX_ARM), /* needed by ipa267_init_nand */
		MUXCFG("ebi_addr23",   MUX_ARM),
		MUXCFG("ebi_addr24",   MUX_ARM),
		MUXCFG("ebi_addr25",   MUX_ARM), /* ARM_7 : I2C-AUX SDA */
		MUXCFG("ebi_addr26",   MUX_ARM),

		MUXCFG("ebi_clk_pin", MUX_ARM),
		MUXCFG("max_tx_ctrl", MUX_ARM),
		MUXCFG("max_ref_clk", MUX_ARM),
		MUXCFG("max_trig_clk", MUX_ARM),
#endif
	};

	err = mux_configure_table(brd267_cfg, ARRAY_SIZE(brd267_cfg));

	if (err) {
		pr_err("ipa267_cfgmux: unable to mux gpios\n");
		return;
	}
}

static void __init ipa267_init(void)
{
	picoxcell_tsu_init(20000000);
	picoxcell_mux_register(ipa267_mux, ARRAY_SIZE(ipa267_mux));
	picoxcell_core_init();

	ipa267_register_uarts();
#if 1
	ipa267_cfgmux();
#endif
	ipa267_init_nand();
	ipa267_panic_init();
#if 1
	spi_register_board_info(ipa267_spi_board_info,
				ARRAY_SIZE(ipa267_spi_board_info));
#endif
#if 1
	i2c_register_board_info(0, ipa267_i2c_bus0_devices,
				ARRAY_SIZE(ipa267_i2c_bus0_devices));
	i2c_register_board_info(1, ipa267_i2c_bus1_devices,
				ARRAY_SIZE(ipa267_i2c_bus1_devices));
#endif
}

static void __init ipa267_late_init(void)
{
    // These have to be added after the GPIO lines have been added.
    platform_add_devices(ipa267_devices, ARRAY_SIZE(ipa267_devices));
}
late_initcall(ipa267_late_init);

MACHINE_START(IPA267, "IPA267")
	.map_io		= picoxcell_map_io,
	.init_irq	= picoxcell_init_irq,
	.init_early	= picoxcell_init_early,
	.timer		= &picoxcell_sys_timer,
	.init_machine	= ipa267_init,
MACHINE_END
