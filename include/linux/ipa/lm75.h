/* -*- C -*-
 * lm75.h -- definitions for the LM75 Temperature Sensor
 *
 * Copyright (C) 2009 ip.access Ltd
 *
 */

#ifndef LM75_H_
#define LM75_H_

#include <linux/ioctl.h>

/*
 * Ioctl definitions
 */

/* Use 'L' as magic number */
#define LM75_IOCTL_MAGIC          'L'

#define LM75_IOCTL_READ_TEMP    _IO(LM75_IOCTL_MAGIC,   0)
#define LM75_IOCTL_READ_PARAMS  _IO(LM75_IOCTL_MAGIC,   1)
#define LM75_IOCTL_WRITE_PARAMS _IO(LM75_IOCTL_MAGIC,   2)

#define LM75_IOCTL_MAXNR        2

typedef struct LM75Params
{
    signed int temp_max;
    signed int temp_hyst;
} LM75Params_t;


#endif /* LM75_H_ */
