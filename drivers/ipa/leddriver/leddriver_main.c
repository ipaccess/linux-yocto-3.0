/*
 * FILE NAME leddriver_main.c
 *
 * Copyright (c) 2007 ip.access Ltd.
 *
 * BRIEF MODULE DESCRIPTION
 *  Driver for LED
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
 
 
/****************************************************************************
 * Standard Library Includes
 ****************************************************************************/ 
#include <linux/version.h>
#include "leddriver.h"

#if !defined( USER_SPACE_TESTHARNESS )

#include <linux/gpio.h>

#include <mach/hardware.h>

#if defined(CONFIG_ARCH_FIRECRACKER)
#   include "ip202_gpio_usage.h"
#elif defined(CONFIG_ARCH_PC3XX)
#   include "ip302_gpio_usage.h"
#elif defined(CONFIG_ARCH_PICOXCELL)
#   include "ip30xx_gpio_usage.h"
#   include "../picoxcell_core.h"
#else  /* CONFIG_ARCH_... */
#  error "Unknown architecture"
#endif /* CONFIG_ARCH_... */

#endif

/****************************************************************************
  Private Definitions
 ****************************************************************************/

/****************************************************************************
  Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void start_timer( LeddriverDev * dev_p, int periodms );
static void stop_timer( LeddriverDev * dev_p );
static void turnLed( LedState state, unsigned char gpioHigh, unsigned char gpioLow );

/****************************************************************************
 * Private Constants
 ****************************************************************************/
#define NUM_LEDS_MAPPED 8

typedef struct
{
    int    pin;          /* GPIO for this pin */
    int    device;       /* leddriver device using this pin */
    int    defaultValue; /* Default state */
    char * name;         /* Name of LED */
} ledDesc_tag;

#if defined(CONFIG_ARCH_FIRECRACKER)

static const ledDesc_tag pc202_ledMappingTable[NUM_LEDS_MAPPED] = 
{
     {ARM_GPIO_OUT_SERVICE_LED, 0, 0, "SERVICE_LED"},  /*   1 */
     {ARM_GPIO_OUT_SYS_LED_RED, 0, 1, "SYS_LED"    },  /*   2 */
     {ARM_GPIO_OUT_ADD_LED_RED, 0, 0, "ADD_LED_RED"},  /*   4 */
     {ARM_GPIO_OUT_ADD_LED_GPS, 1, 0, "ADD_LED_GPS"},  /*   8 */
     {-1,                       0, 0, ""           },  /*  16 */
     {-1,                       0, 0, ""           },  /*  32 */
     {-1,                       0, 0, ""           },  /*  64 */
     {-1,                       0, 0, ""           }   /* 128 */
};

#else
static const ledDesc_tag pc302_ledMappingTable[NUM_LEDS_MAPPED] = 
{
     {PC302_ARM_GPIO_OUT_SERVICE_LED, 0, 0, "SERVICE_LED"},  /*   1 */
     {PC302_ARM_GPIO_OUT_SYS_LED_RED, 0, 1, "SYS_LED"    },  /*   2 */
     {PC302_ARM_GPIO_OUT_ADD_LED_RED, 0, 0, "ADD_LED_RED"},  /*   4 */
     {PC302_ARM_GPIO_OUT_ADD_LED_GPS, 1, 0, "ADD_LED_GPS"},  /*   8 */
     {-1,                             0, 0, ""           },  /*  16 */
     {-1,                             0, 0, ""           },  /*  32 */
     {-1,                             0, 0, ""           },  /*  64 */
     {-1,                             0, 0, ""           }   /* 128 */
};

static const ledDesc_tag pc3x3_ledMappingTable[NUM_LEDS_MAPPED] = 
{
     {PC3X3_ARM_GPIO_OUT_SERVICE_LED, 0, 0, "SERVICE_LED"},  /*   1 */
     {PC3X3_ARM_GPIO_OUT_SYS_LED_RED, 0, 1, "SYS_LED"    },  /*   2 */
     {PC3X3_ARM_GPIO_OUT_ADD_LED_RED, 0, 0, "ADD_LED_RED"},  /*   4 */
     {PC3X3_ARM_GPIO_OUT_ADD_LED_GPS, 1, 0, "ADD_LED_GPS"},  /*   8 */
     {PC3X3_ARM_GPIO_OUT_2G_LED_GRN,  2, 0, "2G_GRN"     },  /*  16 */
     {PC3X3_ARM_GPIO_OUT_2G_LED_RED,  2, 0, "2G_RED"     },  /*  32 */
     {-1,                             0, 0, ""           },  /*  64 */
     {-1,                             0, 0, ""           }   /* 128 */
};

#if defined(CONFIG_ARCH_PICOXCELL)
static const ledDesc_tag pc30xx_ledMappingTable[NUM_LEDS_MAPPED] = 
{
     {PC30XX_ARM_GPIO_OUT_SERVICE_LED, 0, 1, "SERVICE_LED"},  /*   1 */
     {PC30XX_ARM_GPIO_OUT_SYS_LED_RED, 0, 1, "SYS_LED"    },  /*   2 */
     {PC30XX_ARM_GPIO_OUT_ADD_LED_RED, 0, 1, "ADD_LED_RED"},  /*   4 */
     {PC30XX_ARM_GPIO_OUT_ADD_LED_GPS, 1, 0, "ADD_LED_GPS"},  /*   8 */
     {-1,                              0, 0, ""           },  /*  16 */
     {-1,                              0, 0, ""           },  /*  32 */
     {-1,                              0, 0, ""           },  /*  64 */
     {-1,                              0, 0, ""           }   /* 128 */
};

#endif

#endif

/****************************************************************************
 * Exported Variables
 ****************************************************************************/
LeddriverDev        leddev[MAX_NUM_LED_INSTANCES];

/****************************************************************************
 * Private Variables (Must be declared static)
 ****************************************************************************/

static char led0_patterns_str[]=
"0:0,0,1\n"        // OFF solid (special case)
"0:1,1,0\n"        // ON solid (special case)
"0:2,50,50\n" 
"0:3,50,200\n" 
"0:4,300,50\n" 
"0:5,200,1800\n" 
"0:6,1000,1000\n"
"0:7,1800,200";

static char led1_patterns_str[]=
"1:0,0,1\n"        // OFF solid (special case)
"1:1,1,0\n"        // ON solid (special case)
"1:2,50,50\n" 
"1:3,50,200\n" 
"1:4,300,50\n" 
"1:5,200,1800\n" 
"1:6,1000,1000\n" 
"1:7,1800,200";

static char led2_patterns_str[]=
"2:0,0,1\n"        // OFF solid (special case)
"2:1,1,0\n"        // ON solid (special case)
"2:2,50,50\n" 
"2:3,50,200\n" 
"2:4,300,50\n" 
"2:5,200,1800\n" 
"2:6,1000,1000\n" 
"2:7,1800,200";

static char led0_sequences_str[]=
"0:A,1,\"3G No IP Address (off)\",{0,2,0,2,1,-1,0,0}\n"
"0:B,1,\"3G Not Provisioned\",{5,3,0,2,1,-1,0,0}\n"
"0:C,1,\"3G Provisioned\",{6,3,0,2,1,-1,0,0}\n"
"0:D,1,\"3G In Test\",{6,3,0,2,1,-1,0,0}\n"
"0:E,1,\"3G No Service\",{7,3,0,2,1,-1,0,0}\n"
"0:F,1,\"3G Service Available (on)\",{1,3,0,2,1,-1,0,0}\n"
"0:G,1,\"3G Button Pressed\",{2,3,0,2,1,-1,0,0}\n"
"0:H,1,\"3G Factory Restore\",{3,3,0,2,1,-1,0,0}\n"
"0:I,1,\"3G Firmware Upgrade\",{4,3,0,2,1,-1,0,0}\n"
"0:J,1,\"3G Fault (red)\",{1,0,3,0,3,-1,0,0}\n"  // Fault uses Sys LED Red (Turn 2 OFF!)
"0:K,3,\"3G SOS\",{3,3,0,2,1,3,500,1},{4,3,0,2,1,3,500,2},{3,3,0,2,1,3,1000,0}\n"
"0:L,3,\"3G SOS Red\",{3,6,1,2,5,3,500,1},{4,6,1,2,5,3,500,2},{3,6,1,2,5,3,1000,0}\n";

static char led1_sequences_str[]=
"1:A,1,\"Off\",{0,0,8,0,8,-1,0,0}\n"
"1:B,1,\"Searching\",{5,8,0,0,8,-1,0,0}\n"
"1:C,1,\"Antenna State Change\",{2,8,0,0,8,-1,0,0}\n"
"1:D,1,\"Fix Achieved\",{1,8,0,0,8,-1,0,0}\n";

static char led2_sequences_str[]=
"2:A,1,\"2G No IP Address (off)\",{0,0,48,0,48,-1,0,0}\n"
"2:B,1,\"2G Not Provisioned\",{5,32,16,0,48,-1,0,0}\n"
"2:C,1,\"2G Provisioned\",{6,32,16,32,16,-1,0,0}\n"
"2:D,1,\"2G In Test\",{6,32,16,0,48,-1,0,0}\n"
"2:E,1,\"2G No Service\",{7,32,16,0,48,-1,0,0}\n"
"2:F,1,\"2G Service Available (on)\",{1,32,16,0,48,-1,0,0}\n"
"2:G,1,\"2G Button Pressed\",{2,32,16,0,48,-1,0,0}\n"
"2:H,1,\"2G Factory Restore\",{3,32,16,0,48,-1,0,0}\n"
"2:I,1,\"2G Firmware Upgrade\",{4,32,16,0,48,-1,0,0}\n"
"2:J,1,\"2G Spare\",{3,32,16,0,48,-1,0,0}\n"
"2:K,3,\"2G SOS\",{3,34,16,2,48,3,500,1},{4,34,16,2,48,3,500,2},{3,34,16,2,48,3,1000,0}\n"
"2:L,3,\"2G SOS Red\",{3,18,32,2,48,3,500,1},{4,18,32,2,48,3,500,2},{3,18,32,2,48,3,1000,0}\n";



/****************************************************************************
 * Function Name  : turnLed 
 * Description    : On real HW this should cause the real LED to change state
 ****************************************************************************/
static void turnLed( LedState state, unsigned char gpioHigh, unsigned char gpioLow )
{
#ifdef USER_SPACE_TESTHARNESS
    // Do Nothing! (we don't have a real LED to flash!)
#else   // KERNEL Module
    unsigned char i;
    
    PDEBUG("turnLeds: state: %d, gpioHigh: 0x%x gpioLow: 0x%x\n",
                    state, gpioHigh, gpioLow );
    
    for (i = 0; i < NUM_LEDS_MAPPED; i++)
    {
        const ledDesc_tag * led;
        unsigned char led_pin;

#if defined(CONFIG_ARCH_FIRECRACKER)
        led = &pc202_ledMappingTable[i];
#elif defined(CONFIG_ARCH_PICOXCELL)
        if(picoxcell_is_pc30xx())
        {
           led = &pc30xx_ledMappingTable[i];
        }
        else if(picoxcell_is_pc3x3())
        {
           led = &pc3x3_ledMappingTable[i];
        }
        else
        {
           led = &pc302_ledMappingTable[i];
        }
#else
        led = is_pc3x3() ? &pc3x3_ledMappingTable[i] : &pc302_ledMappingTable[i];
#endif
        
        if (led->pin >= 0)
        {
            led_pin = led->pin & 0xff;

            if (gpioHigh & (1 << i))
            {
                gpio_set_value( led_pin, 1 );
            }

            if (gpioLow & (1 << i))
            {
                gpio_set_value( led_pin, 0 );
            }
        }
    }
    
#endif
}

/****************************************************************************
 * Function Name  : timerHandler 
 * Description    : Determines how long to set the delay for next and calls
 *                  routine to actually change the LED state
 ****************************************************************************/
#if defined(USER_SPACE_TESTHARNESS)
void timerHandler( int signum )
{
    LeddriverDev *dev_p = &leddev[0]; // User space test harness only supports 1 instance
#else // KERNEL Module
void timerHandler( unsigned long data )
{
    LeddriverDev *dev_p = (LeddriverDev *)data;
#endif
    LedSequenceEntry *state_p;
    int period;

    ACQUIRE(dev_p);
    state_p = &(dev_p->stateMachine.currentSequence_p[dev_p->stateMachine.state]);
    
    // Timer has gone off, determine what we're doing next
    if (LED_ON == dev_p->stateMachine.ledState)
    {
        // Next state is LED_OFF - turn off LED
        dev_p->stateMachine.ledState = LED_OFF;
        turnLed( LED_OFF, state_p->gpioOffHigh, state_p->gpioOffLow );
        
        // Set timer for 'off_period_ms'
        period = dev_p->ledPattenTable[state_p->pattern].offPeriodMs;
        
        if ( INDEFINITE_REPEAT != state_p->repeat )
        {   // If this pattern is the last repeat then add on the delay_before_next_pattern_ms
            if ((dev_p->stateMachine.repeatCount+1) == state_p->repeat)
            {
                period += state_p->delay_before_next_pattern_ms;
                PDEBUG("extending delay_before_next_pattern_ms  (%d)\n",
                            state_p->delay_before_next_pattern_ms);
            }
        }
    }
    else
    {
        // Next state is LED_ON - turn on LED
        dev_p->stateMachine.ledState = LED_ON;
        turnLed( LED_ON, state_p->gpioOnHigh, state_p->gpioOnLow );
        
        if ( INDEFINITE_REPEAT != state_p->repeat )
        {// Do we repeat this pattern?
            if (++(dev_p->stateMachine.repeatCount) < state_p->repeat)
            {
                // yes
                period = dev_p->ledPattenTable[state_p->pattern].onPeriodMs;
            }
            else
            {
                // We have to find the next state
                dev_p->stateMachine.state = state_p->nextState;
                // check this is a valid entry
                if ( dev_p->stateMachine.state >=
                         dev_p->sequenceTable[dev_p->stateMachine.currentSequenceIndex].numStates )
                {
                    PDEBUG("Warning detected nextState number invalid on sequence %d\n",
                                 dev_p->stateMachine.currentSequenceIndex);
                    dev_p->stateMachine.state = 0;
                }
                PDEBUG("Changing state to : (state)%d (nextState)%d (currSeqIndex)%d\n",
                    dev_p->stateMachine.state,
                    state_p->nextState,
                    dev_p->stateMachine.currentSequenceIndex);
                    
                // Start a new LedSequenceEntry
                state_p = &(dev_p->stateMachine.currentSequence_p[dev_p->stateMachine.state]);
                dev_p->stateMachine.repeatCount = 0;
                period = dev_p->ledPattenTable[state_p->pattern].onPeriodMs;
            }
        }
        else
        {
            period = dev_p->ledPattenTable[state_p->pattern].onPeriodMs;
        }
    }

    RELEASE(dev_p);
    
    /* If the LED period is a magic value (ON_PERMANENT or OFF_PERMANENT) then don't
     * Start the timer, this will leave the LED stuck in this state until someone changes
     * the Sequence
     */
    if (period != ON_PERMANENT && period != OFF_PERMANENT)
    {
        start_timer(dev_p, period);
    }

    PDEBUG("LedState=%s state=%d, currentSequence[state].pattern = %d, repeatCount = %d\n",
        dev_p->stateMachine.ledState==LED_ON?"ON":"OFF",
        dev_p->stateMachine.state,
        state_p->pattern,
        dev_p->stateMachine.repeatCount);

    
}


#ifdef USER_SPACE_TESTHARNESS
/****************************************************************************
 * Function Name  : start_timer 
 * Description    : ... hmm I wonder
 ****************************************************************************/
/* NOTE: In USER_SPACE_TESTHARNESS This will only work for 1 instance of LeddriverDev!!! */
static void start_timer(LeddriverDev *dev_p, int periodms )
{
	struct itimerval itimer;
	int result;
	int sec = periodms/1000;
	periodms -= (sec*1000);
	
	itimer.it_value.tv_sec = (long) sec;
	
	itimer.it_value.tv_usec = (long) (periodms*1000);
	//PDEBUG("Starting timer %d.%d\n",sec,periodms);
	result = setitimer(ITIMER_REAL, &itimer, NULL);
	
}
static void stop_timer(LeddriverDev *dev_p)
{
	struct itimerval itimer;
	itimer.it_value.tv_sec = 0;
	itimer.it_value.tv_usec = 0;
	PDEBUG("Stopping timer\n");
	setitimer(ITIMER_REAL, &itimer, NULL);
}
#else // KERNEL Module
static void start_timer(LeddriverDev *dev_p, int periodms )
{
    init_timer( &(dev_p->ledTimer) ); 
    dev_p->ledTimer.function = timerHandler;
    dev_p->ledTimer.expires  = jiffies + (periodms * (HZ/1000));
    dev_p->ledTimer.data = (unsigned long) dev_p;   // Pass context back to handler
    add_timer( &(dev_p->ledTimer) );
    PDEBUG("Starting timer (periodms:%d) now: %li expires: %li data: %li\n",periodms, jiffies,
            dev_p->ledTimer.expires, dev_p->ledTimer.data);
}
static void stop_timer(LeddriverDev *dev_p)
{
	del_timer_sync( &(dev_p->ledTimer) );
}
#endif /* USER_SPACE_TESTHARNESS */


/* Allow this function to be called from other modules */
void changeLedSequence( int instance, int index )
{
    LeddriverDev * dev_p = &leddev[0];
    
    if (instance < 0 || instance > MAX_NUM_LED_INSTANCES )
    {
        PDEBUG("Invalid LED Driver instance number\n");
    }
    else
    {
        dev_p = &leddev[instance];
    }
    
    if ( index >= dev_p->numSequences )
    {
        PDEBUG("Invalid LED sequence index\n");
        return;
    }
    
    // Delete the timer
    stop_timer( dev_p );
    
    dev_p->stateMachine.ledState = LED_OFF;
    dev_p->stateMachine.currentSequenceIndex = index;
    dev_p->stateMachine.currentSequence_p = &(dev_p->sequenceTable[index].sequenceEntry[0]);
    dev_p->stateMachine.repeatCount = -1;
    dev_p->stateMachine.state = 0;
    
#ifdef USER_SPACE_TESTHARNESS
	timerHandler(SIGALRM);
#else
	timerHandler( (long unsigned) dev_p );
#endif /* USER_SPACE_TESTHARNESS */
}


#ifdef USER_SPACE_TESTHARNESS
int main(int argc, char *argv[])
{
	signal (SIGALRM, timerHandler);
	
	if (argc == 2)
	{
		int seq = atoi(argv[1]);
		int numSeq = sizeof(sequenceTable)/sizeof(SequencePatternTable); 
		if( seq < 0 || seq > (numSeq-1) )
		{
			printf("Sorry please enter sequence number [0-%d]\n", numSeq-1);
			exit(1);
		}
		changeLedSequence(0, seq);
	}
	else
	{
		changeLedSequence(0, 3 );
	}
	
	while(1)
	{
		int i;
		for(i=0; i<800000;i++) { i=i*2;i/=2; } // Delay to reduce amount of printf's per time period
		printf("%s",stateMachine.ledState==LED_ON ? "^" : "_");
		fflush(stdout);
	}
	
	return 0;
}
#endif /* USER_SPACE_TESTHARNESS */

#if !defined(USER_SPACE_TESTHARNESS)
/*
 * export symbols to the kernel
 */
#define LEDDRIVER_MAJOR         241

int     leddriverMajor           = LEDDRIVER_MAJOR;

#if defined(CONFIG_IPACCESS_IP3XXFF_267)

int     leddriverNumDevs         = 3;

#else

int     leddriverNumDevs         = 2;

#endif

module_param (leddriverMajor, int, 0);
module_param (leddriverNumDevs, int, 0);

MODULE_AUTHOR ("ip.access");
MODULE_LICENSE ("Dual BSD/GPL");

/*
 * invoked when user process issues an open() call (or equivalent) on device file
 */
int leddriver_open (struct inode *inode, struct file *filp)
{
    LeddriverDev    *dev_p; /* device information */

    /*  Find the device */
    dev_p = container_of (inode->i_cdev, LeddriverDev, cdev);

    /* and use filp->private_data to point to the device data */
    filp->private_data = dev_p;

    return 0;          /* success */
}


/*---------------------------------------------------------------------------*/
/*
 * invoked when user process issues a close() call (or equivalent) on device file
 */
int leddriver_release (struct inode *inode, struct file *filp)
{
    return 0;
}


/*---------------------------------------------------------------------------*/
/*
 * invoked when user process issues a read() call (or equivalent) on device file
 */
ssize_t leddriver_read (struct file *filp,
                    char __user *buf_p,
                    size_t      count,
                    loff_t      *f_pos_p)
{
    LeddriverDev        *dev_p = filp->private_data;
    char                returnBuffer [1];

    returnBuffer [0] = dev_p->sequenceCode;

    if (copy_to_user (buf_p, &returnBuffer [0], 1))
    {
        return -EFAULT;
    }

    return 1;
}

/*---------------------------------------------------------------------------*/
/*
 * invoked when user process issues a write() call (or equivalent) on device file
 */
ssize_t leddriver_write (struct file *filp,
                    const char __user *buf_p,
                    size_t      count,
                    loff_t      *f_pos_p)
{
    LeddriverDev        *dev_p = filp->private_data;
    char                writeBuffer [1];
    LedSequenceEntry *seqEntry_p = &dev_p->sequenceTable[0].sequenceEntry[0];

        
    if (copy_from_user (&writeBuffer [0], buf_p, 1))
    {
        return -EFAULT;
    }
    
    // Check that the writeBuffer character is in range
    if (writeBuffer[0] == '0')
    {
        dev_p->sequenceCode = writeBuffer[0];
        stop_timer(dev_p);
        
        turnLed (LED_OFF, seqEntry_p->gpioOffHigh, seqEntry_p->gpioOffLow );
    }
    else if ( writeBuffer[0] == '1')
    {
        dev_p->sequenceCode = writeBuffer[0];
        stop_timer(dev_p);
        turnLed (LED_ON, seqEntry_p->gpioOnHigh, seqEntry_p->gpioOnLow );
    }
    else if ( (writeBuffer[0] >= 'A') && (writeBuffer[0] < ('A' + dev_p->numSequences)) )
    {
        dev_p->sequenceCode = writeBuffer[0];
        changeLedSequence( dev_p->instance, (int) (writeBuffer[0] - 'A') );
    }
    else
    {
        PDEBUG("Invalid pattern : %c\n",writeBuffer[0]);
    }
    
    return 1;
}

/*---------------------------------------------------------------------------*/
/*
 * The ioctl() implementation
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
long leddriver_ioctl (
#else
int leddriver_ioctl (struct inode   *inode_p,
#endif
                 struct file    *filp,
                 unsigned int   cmd,
                 unsigned long  arg)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
    long            ret = 0;
#else
    int             ret = 0;
#endif
 
    /* Keep a stub function here in case we want to use an IOCTL in the future! */
    return ret;
}


/*---------------------------------------------------------------------------*/
/*
 * The fops - this structure tells the kernel which functiosn to invoke to
 * handle user process access to the device file
 */
struct file_operations leddriverFops =
{
    .owner =           THIS_MODULE,
    .read =            leddriver_read,
    .write =           leddriver_write,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
    .unlocked_ioctl =  leddriver_ioctl,
#else
    .ioctl =           leddriver_ioctl,
#endif
    .open =            leddriver_open,
    .release =         leddriver_release,
};

/*---------------------------------------------------------------------------*/
/*
 * The proc filesystem: function to read an entry
 */



/*---------------------------------------------------------------------------*/
/*
 * Initialise a character device kernel structure
 */
static void leddriver_setup_cdev (LeddriverDev *dev_p, int index)
{
    int     err;
    int     devno = MKDEV (leddriverMajor, index);

    cdev_init (&dev_p->cdev, &leddriverFops);
    dev_p->cdev.owner = THIS_MODULE;
    dev_p->cdev.ops = &leddriverFops;
    err = cdev_add (&dev_p->cdev, devno, 1);
    /* Fail gracefully if need be */
    if (err)
    {
        printk (KERN_NOTICE "Error %d adding leddriver%d", err, index);
    }
}


/*---------------------------------------------------------------------------*/
/*
 * Invoked by the kernel when the device driver is loaded (insmod)
 */
int leddriver_init (void)
{
    int         result;
    dev_t       dev = MKDEV (leddriverMajor, 0);
    LeddriverDev * dev_p;
    int    devNum;
    const ledDesc_tag * led;
    int    pinIndex = 0;

    if ((leddriverNumDevs < 1) || (leddriverNumDevs > MAX_NUM_LED_INSTANCES))
    {
        printk (KERN_NOTICE "leddriver: Num devices is out of range 1 to %d (%d)",
                            MAX_NUM_LED_INSTANCES, leddriverNumDevs);
        return -EINVAL;
    }

    /*
     * Register our major device number, and accept a dynamic number.
     */
    if (leddriverMajor)
    {
        result = register_chrdev_region (dev, 1, "leddriver");
    }
    else
    {
        result = alloc_chrdev_region (&dev, 0, 1, "leddriver");
        leddriverMajor = MAJOR (dev);
    }
    if (result < 0)
    {
        goto err;
    }

    for (pinIndex = 0; pinIndex < NUM_LEDS_MAPPED; pinIndex++)
    {
#if defined(CONFIG_ARCH_FIRECRACKER)
        led = &pc202_ledMappingTable[pinIndex];
#elif defined(CONFIG_ARCH_PICOXCELL)
        if(picoxcell_is_pc30xx())
        {
           led = &pc30xx_ledMappingTable[pinIndex];
        }
        else if(picoxcell_is_pc3x3())
        {
           led = &pc3x3_ledMappingTable[pinIndex];
        }
        else
        {
           led = &pc302_ledMappingTable[pinIndex];
        }
#else
        led = is_pc3x3() ? &pc3x3_ledMappingTable[pinIndex] : &pc302_ledMappingTable[pinIndex];
#endif
        if ((led->pin > 0) && (led->device < leddriverNumDevs))
        {
            result = gpio_request(led->pin, led->name);
            if (result < 0)
            {
                goto err;
            }
            gpio_set_value(led->pin, led->defaultValue);
            gpio_direction_output(led->pin, led->defaultValue);
            gpio_set_value(led->pin, led->defaultValue);
        }
    }

    dev_p = &leddev[0];
    for (devNum = 0; devNum < leddriverNumDevs; devNum++)
    {
        leddriver_setup_cdev (dev_p, devNum);
        dev_p->sequenceCode = '0';
        dev_p->instance = devNum;
        dev_p->numPatterns = 0;
        dev_p->numSequences = 0;
        
        dev_p->ledPattenTable = kmalloc( (sizeof(LedFlashPattern)*MAX_NUM_PATTERNS), GFP_KERNEL );
        if ( dev_p->ledPattenTable == NULL )
        {
            printk("leddriver: KMALLOC failed\n");
            result = -ENOMEM;
            goto err;
        }

        dev_p->sequenceTable = kmalloc( (sizeof(SequencePatternTable)*MAX_NUM_SEQUENCES), GFP_KERNEL );
        if ( dev_p->sequenceTable == NULL )
        {
            printk("leddriver: KMALLOC failed\n");
            result = -ENOMEM;
            goto err;
        }

        dev_p->stateMachine.currentSequence_p = &dev_p->sequenceTable[0].sequenceEntry[0];
        dev_p->stateMachine.currentSequenceIndex = 0;
        dev_p->stateMachine.state = 0;
        dev_p->stateMachine.ledState = LED_OFF;
        dev_p->stateMachine.repeatCount = 0;     
                
        dev_p++;
    }

    create_proc_fs();

    // Initialise some default patterns
    parse_pattern_buffer(led0_patterns_str, sizeof(led0_patterns_str));
    parse_sequence_buffer(led0_sequences_str, sizeof(led0_sequences_str));
    
    // On custom hardware additional I/O is available for LED
    if ( leddriverNumDevs > 1)
    {
        parse_pattern_buffer(led1_patterns_str, sizeof(led1_patterns_str));
        parse_sequence_buffer(led1_sequences_str, sizeof(led1_sequences_str));
    }
    
    // On 267 additional I/O is available for LED
    if ( leddriverNumDevs > 2)
    {
        parse_pattern_buffer(led2_patterns_str, sizeof(led2_patterns_str));
        parse_sequence_buffer(led2_sequences_str, sizeof(led2_sequences_str));
    }
    
    printk( "Added leddriver.ko with major number=%d\n",leddriverMajor );

    return 0; /* succeed */
    
err:
    for (pinIndex--; pinIndex >= 0; pinIndex--) /* Work back down the list from where we got to */
    {
#if defined(CONFIG_ARCH_FIRECRACKER)
        led = &pc202_ledMappingTable[pinIndex];
#elif defined(CONFIG_ARCH_PICOXCELL)
        if(picoxcell_is_pc30xx())
        {
           led = &pc30xx_ledMappingTable[pinIndex];
        }
        else if(picoxcell_is_pc3x3())
        {
           led = &pc3x3_ledMappingTable[pinIndex];
        }
        else
        {
           led = &pc302_ledMappingTable[pinIndex];
        }
#else
        led = is_pc3x3() ? &pc3x3_ledMappingTable[pinIndex] : &pc302_ledMappingTable[pinIndex];
#endif
        if ((led->pin > 0) && (led->device < leddriverNumDevs))
        {
            gpio_free(led->pin);
        }
    }

    return result;
}


/*---------------------------------------------------------------------------*/
/*
 * Invoked by the kernel when the device driver is unloaded (rmmod)
 */
void leddriver_cleanup (void)
{
    LeddriverDev *      dev_p;
    int                 devNum;
    int                 pinIndex;
    const ledDesc_tag * led;

    for (pinIndex=0; pinIndex < NUM_LEDS_MAPPED; pinIndex++)
    {
#if defined(CONFIG_ARCH_FIRECRACKER)
        led = &pc202_ledMappingTable[pinIndex];
#elif defined(CONFIG_ARCH_PICOXCELL)
        if(picoxcell_is_pc30xx())
        {
           led = &pc30xx_ledMappingTable[pinIndex];
        }
        else if(picoxcell_is_pc3x3())
        {
           led = &pc3x3_ledMappingTable[pinIndex];
        }
        else
        {
           led = &pc302_ledMappingTable[pinIndex];
        }
#else
        led = is_pc3x3() ? &pc3x3_ledMappingTable[pinIndex] : &pc302_ledMappingTable[pinIndex];
#endif
        if ((led->pin > 0) && (led->device < leddriverNumDevs))
        {
            gpio_free(led->pin);
        }
    }

    dev_p = &leddev[0];
    for (devNum = 0; devNum < leddriverNumDevs; devNum++)
    {
        stop_timer(dev_p);    
        cdev_del (&(dev_p->cdev));

        kfree(dev_p->sequenceTable);
        kfree(dev_p->ledPattenTable);
        dev_p++;
    }

    remove_proc_fs();
    
    unregister_chrdev_region (MKDEV (leddriverMajor, 0), 1);
}


/*---------------------------------------------------------------------------*/
/*
 * Export device driver main entry points.
 */
module_init (leddriver_init);
module_exit (leddriver_cleanup);

EXPORT_SYMBOL(changeLedSequence); // To allow the reset switch driver to change led state
#endif /* USER_SPACE_TESTHARNESS */
