/*
 * FILE NAME ip302_gpio_usage.h
 *
 * Copyright (c) 2009 ip.access Ltd.
 *
 * BRIEF MODULE DESCRIPTION
 *  Header file which determines which GPIO line is used for which purpose
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
 
#if !defined (INCLUDED_IP302_GPIO_USAGE_H)
#define  INCLUDED_IP302_GPIO_USAGE_H

#if defined(CONFIG_IPACCESS_IP3XXFF_267)

/* 267 is PC3X3 only */
/* 267 is only variant with a 2G LED */

#define PC302_ARM_GPIO_OUT_SERVICE_LED        -1
#define PC302_ARM_GPIO_IN_SWITCH              -1
#define PC302_ARM_GPIO_OUT_SYS_LED_RED        -1
#define PC302_ARM_GPIO_OUT_NRESET             -1
#define PC302_ARM_GPIO_OUT_ADD_LED_GPS        -1
#define PC302_ARM_GPIO_OUT_ADD_LED_RED        -1
#define PC302_ARM_GPIO_OUT_2G_LED_RED         -1
#define PC302_ARM_GPIO_OUT_2G_LED_GRN         -1

#define PC3X3_ARM_GPIO_OUT_SERVICE_LED        (PC3X3_GPIO_PIN_ARM_5)
#define PC3X3_ARM_GPIO_IN_SWITCH              (PC3X3_GPIO_PIN_ARM_23)
#define PC3X3_ARM_GPIO_OUT_SYS_LED_RED        (PC3X3_GPIO_PIN_ARM_15)
#define PC3X3_ARM_GPIO_OUT_NRESET             (PC3X3_GPIO_PIN_ARM_17)
#define PC3X3_ARM_GPIO_OUT_ADD_LED_GPS        (PC3X3_GPIO_PIN_ARM_47)
#define PC3X3_ARM_GPIO_OUT_ADD_LED_RED        (PC3X3_GPIO_PIN_ARM_6)
#define PC3X3_ARM_GPIO_OUT_2G_LED_RED         (PC3X3_GPIO_PIN_ARM_48)
#define PC3X3_ARM_GPIO_OUT_2G_LED_GRN         (PC3X3_GPIO_PIN_ARM_49)

#else

#define PC302_ARM_GPIO_OUT_SERVICE_LED        (PC3X2_GPIO_PIN_ARM_5)
#define PC302_ARM_GPIO_IN_SWITCH              (PC3X2_GPIO_PIN_ARM_4)
#define PC302_ARM_GPIO_OUT_SYS_LED_RED        (PC3X2_GPIO_PIN_ARM_3)
#define PC302_ARM_GPIO_OUT_NRESET             (PC3X2_GPIO_PIN_ARM_1)
#define PC302_ARM_GPIO_OUT_ADD_LED_GPS        (PC3X2_GPIO_PIN_ARM_7)
#define PC302_ARM_GPIO_OUT_ADD_LED_RED        (PC3X2_GPIO_PIN_ARM_6)
#define PC302_ARM_GPIO_OUT_2G_LED_RED         -1
#define PC302_ARM_GPIO_OUT_2G_LED_GRN         -1

#define PC3X3_ARM_GPIO_OUT_SERVICE_LED        (PC3X3_GPIO_PIN_ARM_5)
#define PC3X3_ARM_GPIO_IN_SWITCH              (PC3X3_GPIO_PIN_ARM_4)
#define PC3X3_ARM_GPIO_OUT_SYS_LED_RED        (PC3X3_GPIO_PIN_ARM_3)
#define PC3X3_ARM_GPIO_OUT_NRESET             (PC3X3_GPIO_PIN_ARM_1)
#define PC3X3_ARM_GPIO_OUT_ADD_LED_GPS        (PC3X3_GPIO_PIN_ARM_7)
#define PC3X3_ARM_GPIO_OUT_ADD_LED_RED        (PC3X3_GPIO_PIN_ARM_6)
#define PC3X3_ARM_GPIO_OUT_2G_LED_RED         (PC3X3_GPIO_PIN_ARM_48)
#define PC3X3_ARM_GPIO_OUT_2G_LED_GRN         (PC3X3_GPIO_PIN_ARM_49)

#endif

#endif /*INCLUDED_IP302_GPIO_USAGE_H*/
