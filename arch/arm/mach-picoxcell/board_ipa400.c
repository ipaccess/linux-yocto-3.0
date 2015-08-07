/*
 * linux/arch/arm/mach-picoxcell/board_ipa400.c
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
#include <linux/i2c/pca953x.h>

#include <mach/hardware.h>
#include <mach/picoxcell/axi2cfg.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <linux/ext_wdt.h>
#include <linux/dma-mapping.h>

#include "mux.h"
#include "picoxcell_core.h"

static long ipa400_panic_blink(int state)
{
	__raw_writel(state ? 0xFF : 0, IO_ADDRESS(PICOXCELL_GPIO_BASE +
					    GPIO_SW_PORT_C_DR_REG_OFFSET));
	return 0;
}

static void ipa400_panic_init(void)
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

	panic_blink = ipa400_panic_blink;
}

static struct resource ipa400_nand_resource[] = {
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

static struct mtd_partition ipa400_nand_parts[] = {
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

static struct gpio_nand_platdata ipa400_nand_platdata = {
	.gpio_rdy   = PC3X2_GPIO_PIN_ARM_1,
	.gpio_nce   = PC3X2_GPIO_PIN_ARM_2,
	.gpio_ale   = PC3X2_GPIO_PIN_ARM_3,
	.gpio_cle   = PC3X2_GPIO_PIN_ARM_4,
	.gpio_nwp   = -1,
	.parts	    = ipa400_nand_parts,
	.num_parts  = ARRAY_SIZE(ipa400_nand_parts),
};

static struct platform_device ipa400_nand = {
	.name		    = "gpio-nand",
	.num_resources	    = ARRAY_SIZE(ipa400_nand_resource),
	.resource	    = ipa400_nand_resource,
	.id		    = -1,
	.dev.platform_data  = &ipa400_nand_platdata,
};

/*
 * Add SPI Busses
 */
struct spi_gpio_platform_data ipa400_spi_gpio_bus1_platform_data = {
	.sck            = PC3X3_GPIO_PIN_ARM_10,
	.miso           = PC3X3_GPIO_PIN_ARM_11,
	.mosi           = PC3X3_GPIO_PIN_ARM_12,
	.num_chipselect = 3, //TODO should this be 4 (0 to 3)
};

static struct platform_device ipa400_spi_gpio_bus1_device = {
	.name = "spi_gpio",
	.id = 1,
	.dev = {
		.platform_data = &ipa400_spi_gpio_bus1_platform_data,
	}
};

static struct ext_wdt_platform_data ext_wdt_data = {
	 .wdt_enable_gpio = 43 ,
	 .wdt_tick_gpio = 53 ,
	 .interval = 5 * HZ,
	 .first_interval = 0,
};


static struct platform_device ext_wdt_device = {
	.name			= "ext-wdt",
	.id			= -1,
        .dev  = 
                {
                   .platform_data = &ext_wdt_data
                }
};

/*
 * We have two i2c busses, and both are driven using GPIO.
 */
static struct i2c_gpio_platform_data ipa400_i2c_bus0_data = {
	.sda_pin = PC3X3_GPIO_PIN_ARM_7,
	.scl_pin = PC3X3_GPIO_PIN_ARM_0,
	.udelay  = 2, /* Between 100kHz and 400kHz, nominally 250kHz */
	.timeout = 100
};

static struct platform_device ipa400_i2c_bus0_device = {
	.name = "i2c-gpio",
	.id = 0,
	.dev = {
		.platform_data = &ipa400_i2c_bus0_data,
	}
};

static struct pca953x_platform_data io_expander_data =
{
    .gpio_base = 200
};

static struct i2c_board_info __initdata ipa400_i2c_bus0_devices[] = {
	//{ I2C_BOARD_INFO("max6635",   0x4B), }, /* maxim phy? */
	//{ I2C_BOARD_INFO("atmel_twi", 0x29), }, /* see at91 stuff, do we have this? */
	{ I2C_BOARD_INFO("ad7995",    0x28), }, //ADC (power detector)
	//{ I2C_BOARD_INFO("micrel",    0x5F), }, /* micrel phy I assume? */
	{ I2C_BOARD_INFO("lm75",      0x48), }, /* lm75 temperature sensor */
	{ I2C_BOARD_INFO("lm75",      0x4f), }, /* lm75 temperature sensor */
    //TODO others
    { I2C_BOARD_INFO("htu21",0x40)}, //TODO humidity sensor HTU21D
    //{} //TODO UBlox GPS receiver. Can be used with either I2C and/or UART to access the  NMEA interface.
    
    // Radio io-expander control
    {
        .type = "pca9539",
        .addr = 0x74,
        .platform_data = &io_expander_data,
    }
};

static struct i2c_gpio_platform_data ipa400_i2c_bus1_data = {
	.sda_pin = PC3X3_GPIO_PIN_ARM_46,
	.scl_pin = PC3X3_GPIO_PIN_ARM_45,
	.udelay  = 2, /* Between 100kHz and 400kHz, nominally 250kHz */
	.timeout = 100
};

static struct platform_device ipa400_i2c_bus1_device = {
	.name = "i2c-gpio",
	.id = 1,
	.dev = {
		.platform_data = &ipa400_i2c_bus1_data,
	}
};

static struct i2c_board_info __initdata ipa400_i2c_bus1_devices[] = {
//	{ I2C_BOARD_INFO("at91_i2c",  0x49), },
	{ I2C_BOARD_INFO("lm75",     0x48), },  /* lm75 temperature sensor */
};

static void ipa400_init_nand(void)
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

	platform_device_register(&ipa400_nand);
}

FIXED_CLK(ipa400_uart,	3686400, -1, NULL);
static struct clk_lookup ipa400_uart_lookup = CLK_LOOKUP(NULL, "uart",
							 &ipa400_uart_clk);

static void ipa400_register_uarts(void)
{
	picoxcell_clk_add(&ipa400_uart_clk);
	clkdev_add(&ipa400_uart_lookup);
	picoxcell_add_uart(PICOXCELL_UART1_BASE, IRQ_UART1, 0);
	picoxcell_add_uart(PICOXCELL_UART2_BASE, IRQ_UART2, 1);
}

/*
 * We have a 64KiB SPI Flash device fitted.  We create a single
 * partition called 'FSBL0' on this device.
 *
 * If we ever get a larger NOR device we can think about changing
 * this, but since we're only using the first 16KiB of the device
 * there's really no reason to change things.
 */
static struct mtd_partition ipa400_spi_flash_partitions[] = {
	{
		.name		= "FSBL0",
		.size		= SZ_64K,
		.offset		= 0,
	},
};

static struct flash_platform_data ipa400_spi_flash_data = {
	.name			= "spi-flash",
	.parts			= ipa400_spi_flash_partitions,
	.nr_parts		= ARRAY_SIZE(ipa400_spi_flash_partitions),
	.type			= "m25p05",
};

/*
 * SPI Board Data for SPI bus 0 (picoxcell-spi)
 */
// As SPI bus 1 is bit banged (spi_gpio) controller_data must specify the GPIO used for the
// chipselect line for that device.
static struct spi_board_info ipa400_spi_board_info[] __initdata = {
	/*
	 * CS0 on picoxcell-spi is the SPI Flash
	 */
	{
		.modalias	= "m25p05",
		.platform_data	= &ipa400_spi_flash_data,
		.mode		= SPI_MODE_3,
		.max_speed_hz	= 2000000,
		.bus_num	= 0,
		.chip_select	= 0,
	},
	/*
	 * CS1: SD card
	 */
	{
		.modalias	= "mmc_spi",
		.platform_data	= NULL,
		.mode		= SPI_MODE_0,
		.max_speed_hz	= 2000000,
		.bus_num	= 0,
		.chip_select	= 1,
	},
	/*
	 * CS2: SPI: Reference Clock Control DAC
	 */
	{
		.modalias	= "dac7512",        //TODO OK
		.platform_data	= NULL,
		.mode		= SPI_MODE_1,
		.max_speed_hz	= 10000, /* 10kHz (not 20MHz, we'd get only 80 CPU cycles between writes) */
		.bus_num	= 0,
		.chip_select	= 2,
	},
	/*
	 * CS3: SPI: Ethernet switch KSZ8463MLL
	 */
	{
		.modalias	= "ksz8463",
		.platform_data	= NULL,
		.mode		= SPI_MODE_0,  /* Idle clock is low.  Latch on rising edge (second edge) */
		.max_speed_hz	= 1000000, /* 1MHz */
		.bus_num	= 0,
		.chip_select	= 3,
		.irq = IRQ_GPIO6,
	},
    {   // spidev driver allows user space access to bus
        .modalias = "spidev",
        .platform_data = NULL,
        .mode = SPI_MODE_3,
        .max_speed_hz = 10000, //TODO is this the correct speed
        .bus_num = 1,
        .chip_select = 0,
        .controller_data = (void*) PC3X3_GPIO_PIN_ARM_9,
    }
};

//TODO add second spi bus devices

struct resource ipa400_picoxcell_l2_res[] = {
	{
		.start          = PICOXCELL_CIPHER_BASE,
		.end            = PICOXCELL_CIPHER_BASE + 0xFFFF,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_AES,
		.end            = IRQ_AES,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 picoxcell_l2_dmamask = DMA_BIT_MASK(32);
static struct platform_device ipa400_picoxcell_l2_device = {
	.name                   = "l2_engine",
	.id                     = -1,
	.num_resources          = ARRAY_SIZE(ipa400_picoxcell_l2_res),
	.resource               = &ipa400_picoxcell_l2_res,
	.dev                    = {
		.dma_mask               = &picoxcell_l2_dmamask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
	},

};


static struct platform_device *ipa400_devices[] __initdata = {
    &ipa400_i2c_bus0_device,
    &ipa400_i2c_bus1_device,
    &ipa400_picoxcell_l2_device,
    //&ipa400_i2c_bus1_device, //TODO not used at the moment
};

/*
 * TODO: SPI Board Data for SPI bus 1 (Radio SPI, GPIO)
 *
 * CS0: ADI/Aux Radio Mux 0
 * CS1: Aux Radio Mux 1
 * CS2: Aux Radio Mux 2
 * CS3: Aux Radio Mux 3
 */

static struct mux_def ipa400_mux[] = {
	MUXGPIO(arm_gpio16,    16,       0,     RSVD,   0x34,  16,      -1,     -1,     0),
	MUXGPIO(arm_gpio17,    17,       1,     RSVD,   0x34,  17,      -1,     -1,     0),
	MUXGPIO(arm_gpio18,    18,       2,     RSVD,   0x34,  18,      -1,     -1,     0),
	MUXGPIO(arm_gpio19,    19,       3,     RSVD,   0x34,  19,      -1,     -1,     0),
};

static void ipa400_cfgmux(void)
{
	int err;

	const struct mux_cfg brd400_cfg[] = {
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

		/*
		 * SPI - Designware driver works better waggling the CS lines itself rather than
		 *       letting the SPI controller do it.
		 */
		MUXCFG("decode0",        MUX_ARM), /* ARM_36: SPI-AUX CS0 */
		MUXCFG("decode1",        MUX_ARM), /* ARM_37: SPI-AUX CS1 */
		MUXCFG("decode2",        MUX_ARM), /* ARM_38: SPI-AUX CS2 */
		MUXCFG("decode3",        MUX_ARM), /* ARM_39: SPI-AUX CS3 */

		#if 0
		/*
		 * SPI-AUX (SPI Bus 2) - this interferes with the SPI flash - it's quite likely the decode pins and wonky
		 */
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
		MUXCFG("pai_tx_data0", MUX_PERIPHERAL_PAI), /* PAI Iface */ /* needed by ipa400_init_nand */
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
		MUXCFG("ebi_addr22",   MUX_ARM), /* needed by ipa400_init_nand */
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

	err = mux_configure_table(brd400_cfg, ARRAY_SIZE(brd400_cfg));

	if (err) {
		pr_err("ipa400_cfgmux: unable to mux gpios\n");
		return;
	}
}

static void reset_lines_init(void)
{
    int err;

    err = gpio_request(50, "io_expander_reset");
    if (err)
    {
        printk(KERN_ALERT "%s failed io_expander_reset gpio request %d\n", __func__, err);
        return;
    }

    // Take the io expander out of reset
    err = gpio_direction_output(50, 1);
    if (err)
    {
        printk(KERN_ALERT "%s failed to set dir of io_expander_reset %d\n", __func__, err);
        gpio_free(50);
        return;
    }

}

static void __init ipa400_init(void)
{
    int ret;

	picoxcell_tsu_init(20000000);
	picoxcell_mux_register(ipa400_mux, ARRAY_SIZE(ipa400_mux));
	picoxcell_core_init();

	ipa400_register_uarts();
#if 1
	ipa400_cfgmux();
#endif
	ipa400_init_nand();
	ipa400_panic_init();
#if 1
	spi_register_board_info(ipa400_spi_board_info,
				ARRAY_SIZE(ipa400_spi_board_info));

    ret = platform_device_register(&ipa400_spi_gpio_bus1_device);
    printk("%s device reg spi bus 1 ret %d \n",__func__, ret);

    ret = platform_device_register(&ext_wdt_device);
    printk("%s device reg ext watchdog ret %d \n",__func__, ret);
#endif
#if 1
	//i2c_register_board_info(1, ipa400_i2c_bus1_devices,
	//			ARRAY_SIZE(ipa400_i2c_bus1_devices));
#endif
}


static void __init ipa400_late_init(void)
{
    printk("%s machine_arch_type %d \n",__func__,machine_arch_type);
    if(machine_is_ipa400())
    {
        printk("%s\n",__func__);

        // These have to be added after the GPIO lines have been added by picoxcellgpio
        reset_lines_init();
        i2c_register_board_info(0, ipa400_i2c_bus0_devices, ARRAY_SIZE(ipa400_i2c_bus0_devices));
        i2c_register_board_info(1, ipa400_i2c_bus1_devices, ARRAY_SIZE(ipa400_i2c_bus1_devices));

        // These have to be added after the GPIO lines have been added by picoxcellgpio
        platform_add_devices(ipa400_devices, ARRAY_SIZE(ipa400_devices));
    }
}
late_initcall(ipa400_late_init);

MACHINE_START(IPA400, "IPA400")
	.map_io		= picoxcell_map_io,
	.init_irq	= picoxcell_init_irq,
	.init_early	= picoxcell_init_early,
	.timer		= &picoxcell_sys_timer,
	.init_machine	= ipa400_init,
MACHINE_END

