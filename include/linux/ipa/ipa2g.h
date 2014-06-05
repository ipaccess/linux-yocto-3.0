/* -*- C -*-
 * ipa2g.h -- definitions for the IPA2G Temperature Sensor
 *
 * Copyright (C) 2012 ip.access Ltd
 *
 */

#ifndef IPA2G_H_
#define IPA2G_H_

#include <linux/ioctl.h>

/*
 * Ioctl definitions
 */

#define IPA2G_IOCTL_MAGIC          'L'

#define IPA2G_IOCTL_READ_EEPROM_REG    _IO(IPA2G_IOCTL_MAGIC,   0)
#define IPA2G_IOCTL_WRITE_EEPROM_REG   _IO(IPA2G_IOCTL_MAGIC,   1)

#define IPA2G_IOCTL_MAXNR 1

#define IPA2G_REG_1 0x00
#define IPA2G_REG_2 0x01
#define IPA2G_REG_3 0x02

typedef struct IPA2GParams
{
    signed short reg;
    signed short regWidth;
    signed int value;
} IPA2GParams_t;


#endif /* IPA2G_H_ */
