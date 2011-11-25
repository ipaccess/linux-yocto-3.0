/*
 * linux/arch/arm/mach-picoxcell/board_pc7308.c
 *
 * Copyright (c) 2011 Picochip Ltd., Jamie Iles
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

#include <mach/hardware.h>
#include <mach/picoxcell/axi2cfg.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "mux.h"
#include "picoxcell_core.h"

static long pc7308_panic_blink(int state)
{
	__raw_writel(state ? 0x01 : 0, IO_ADDRESS(PICOXCELL_GPIO_BASE +
					    GPIO_SW_PORT_D_DR_REG_OFFSET));
	return 0;
}

static void pc7308_panic_init(void)
{
	/*
	 * We have a BOOT_ERROR pin on PC7308. Reuse that for signalling when
	 * the kernel panics.
	 *
	 * Note: There is only 1 bit wired up to port D
	 */
	__raw_writel(0x01, IO_ADDRESS(PICOXCELL_GPIO_BASE +
			       GPIO_SW_PORT_D_DDR_REG_OFFSET));
	__raw_writel(0x00, IO_ADDRESS(PICOXCELL_GPIO_BASE +
			       GPIO_SW_PORT_D_CTL_REG_OFFSET));

	panic_blink = pc7308_panic_blink;
}

static struct mtd_partition pc7308_nand_parts[] = {
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

static void pc7308_init_nand(void)
{
	int err = picoxcell_add_hw_nand(pc7308_nand_parts,
					ARRAY_SIZE(pc7308_nand_parts));
	if (err)
		pr_err("failed to register nand partitions\n");
}

static void pc7308_register_uarts(void)
{
	int err;
	struct platform_device *pdev;

	pdev = picoxcell_add_uart(PICOXCELL_UART1_BASE, IRQ_UART1, 0);
	if (IS_ERR(pdev))
		pr_err("failed to add uart0\n");

	err = picoxcell_add_uicc(PC30XX_UART3_BASE, IRQ_PC30XX_UART3, 2,
				 false);
	if (err)
		pr_err("failed to add uart based uicc controller\n");
}

static void __init pc7308_init(void)
{
	picoxcell_tsu_init(20000000);
	picoxcell_core_init();

	pc7308_register_uarts();
	pc7308_init_nand();
	pc7308_panic_init();
}

MACHINE_START(PC7308, "PC7308")
	.map_io		= picoxcell_map_io,
	.init_irq	= picoxcell_init_irq,
	.init_early	= picoxcell_init_early,
	.timer		= &picoxcell_sys_timer,
	.init_machine	= pc7308_init,
MACHINE_END
