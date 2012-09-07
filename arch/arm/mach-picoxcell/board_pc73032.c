/*
 * linux/arch/arm/mach-picoxcell/board_pc73032.c
 *
 * Copyright (c) 2012 Picochip Ltd., Dave Aldridge
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>

#include <mach/hardware.h>
#include <mach/picoxcell/axi2cfg.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "mux.h"
#include "picoxcell_core.h"

static long pc73032_panic_blink(int state)
{
	__raw_writel(state ? 0x01 : 0, IO_ADDRESS(PICOXCELL_GPIO_BASE +
					    GPIO_SW_PORT_D_DR_REG_OFFSET));
	return 0;
}

static void pc73032_panic_init(void)
{
	/*
	 * We have a BOOT_ERROR led on the PC73032.
	 * Reuse that for signalling when the kernel panics.
	 *
	 * Note: There is only 1 bit wired up to gpio port D
	 */
	__raw_writel(0x01, IO_ADDRESS(PICOXCELL_GPIO_BASE +
			       GPIO_SW_PORT_D_DDR_REG_OFFSET));
	__raw_writel(0x00, IO_ADDRESS(PICOXCELL_GPIO_BASE +
			       GPIO_SW_PORT_D_CTL_REG_OFFSET));

	panic_blink = pc73032_panic_blink;
}

static struct mtd_partition pc73032_nand_parts[] = {
	{
		.name	= "Nand First Stage 0",
		.size	= SZ_128K,
		.offset	= 0
	},
        {
		.name	= "Nand First Stage 1",
		.size	= SZ_128K,
		.offset	= MTDPART_OFS_APPEND,
	},
        {
		.name	= "Nand First Stage 2",
		.size	= SZ_128K,
		.offset	= MTDPART_OFS_APPEND,
	},
        {
		.name	= "Nand First Stage 3",
		.size	= SZ_128K,
		.offset	= MTDPART_OFS_APPEND,
	},
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

static void pc73032_init_nand(void)
{
	int err = picoxcell_add_hw_nand(pc73032_nand_parts,
					ARRAY_SIZE(pc73032_nand_parts));
	if (err)
		pr_err("failed to register nand partitions\n");
}

static void pc73032_register_uarts(void)
{
	int err;
	struct platform_device *pdev;

	pdev = picoxcell_add_uart(PICOXCELL_UART1_BASE, IRQ_UART1, 0);
	if (IS_ERR(pdev))
		pr_err("failed to add uart0\n");

        pdev = picoxcell_add_uart(PICOXCELL_UART2_BASE, IRQ_UART2, 1);
	if (IS_ERR(pdev))
		pr_err("failed to add uart1\n");

	err = picoxcell_add_uicc(PC30XX_UART3_BASE, IRQ_PC30XX_UART3, 2,
				 true);
	if (err)
		pr_err("failed to add uart based uicc controller\n");
}

static struct mtd_partition pc73032_spi_flash_partitions[] = {
	{
		.name		= "SPI Flash",
		.size		= MTDPART_SIZ_FULL,
		.offset		= 0,
	},
};

static struct flash_platform_data pc73032_spi_flash_data = {
	.name			= "pc73032 spi flash",
	.parts			= pc73032_spi_flash_partitions,
	.nr_parts		= ARRAY_SIZE(pc73032_spi_flash_partitions),
};

static struct spi_board_info pc73032_spi_board_info[] __initdata = {
	{
		.modalias	= "m25p80",
		.platform_data	= &pc73032_spi_flash_data,
		.mode		= SPI_MODE_3,
		.max_speed_hz	= 2000000,

                /* Note: Ensure jumper J11 on the PC73032 platform is set
                 *       correctly to connect ebi_decode1 to the
                 *       spi flash chip select.
                 */
                .chip_select	= 1,
	}
};

static void __init pc73032_init(void)
{
	picoxcell_tsu_init(20000000);
	picoxcell_core_init();

	pc73032_register_uarts();
	pc73032_init_nand();
	pc73032_panic_init();
	spi_register_board_info(pc73032_spi_board_info,
				ARRAY_SIZE(pc73032_spi_board_info));
}

MACHINE_START(PC73032, "PC73032")
	.map_io		= picoxcell_map_io,
	.init_irq	= picoxcell_init_irq,
	.init_early	= picoxcell_init_early,
	.timer		= &picoxcell_sys_timer,
	.init_machine	= pc73032_init,
MACHINE_END
