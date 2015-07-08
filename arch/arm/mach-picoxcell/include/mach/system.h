/*
 * Copyright (c) 2010 Picochip Ltd., Jamie Iles
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/io.h>
#include <mach/picoxcell/picoxcell.h>
#include <mach/picoxcell/wdog.h>
#include <mach/picoxcell/gpio.h>

static inline void arch_idle(void)
{
	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks
	 */
	cpu_do_idle();
}

static inline void arch_reset(int mode, const char *cmd)
{
    unsigned int temp;
    
    /*
     *  First attempt a reset by driving GPIO 17 low...
     */
    printk("%s: Performing hard reset of ipa400 board\n", __func__);
    
    /* Hard reset involves setting GPIO 17 (bit 9 of GPIO port B) 
     *       as an output and then setting its value to 0 */
    temp = __raw_readl(IO_ADDRESS(PICOXCELL_GPIO_BASE +
    GPIO_SW_PORT_B_DDR_REG_OFFSET));
    __raw_writel((temp | 0x00000200), IO_ADDRESS(PICOXCELL_GPIO_BASE +
    GPIO_SW_PORT_B_DDR_REG_OFFSET));
    
    temp = __raw_readl(IO_ADDRESS(PICOXCELL_GPIO_BASE +
    GPIO_SW_PORT_B_DR_REG_OFFSET));
    __raw_writel((temp & 0xFFFFFDFF), IO_ADDRESS(PICOXCELL_GPIO_BASE +
    GPIO_SW_PORT_B_DR_REG_OFFSET));
    
    /* Give it chance to reset. */
    mdelay(500);
    
    /*
	 * If that didn't work (it won't on a 267 board) set the watchdog to expire as
     * soon as possible and reset the system.
	 */
    pr_warn("Shutdown via external circuitry failed, trying internal watchdog...\n");
    __raw_writel(WDOG_CONTROL_REG_WDT_EN_MASK,
	       IO_ADDRESS(PICOXCELL_WDOG_BASE + WDOG_CONTROL_REG_OFFSET));
	__raw_writel(0, IO_ADDRESS(PICOXCELL_WDOG_BASE +
			     WDOG_TIMEOUT_RANGE_REG_OFFSET));

	/* Give it chance to reset. */
	mdelay(500);

	pr_crit("watchdog reset failed - entering infinite loop\n");
}

#endif /* __ASM_ARCH_SYSTEM_H */
