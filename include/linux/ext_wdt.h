#ifndef _EXT_WDT_H_
#define _EXT_WDT_H_

struct ext_wdt_platform_data {
	int	wdt_enable_gpio;/* GPIO line number */
	int	wdt_tick_gpio;	/* GPIO line number */
	int	interval;	/* watchdog reset interval in system ticks */
	int	first_interval;	/* first wd reset interval in system ticks */
};

#endif /* _EXT_WDT_H_ */

