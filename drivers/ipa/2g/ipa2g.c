/***************************************************************************
 *
 *  IP ACCESS -
 *
 *  Copyright (c) 2000-2008 ip.access Ltd.
 *

 ****************************************************************************
 *
 *  File Description :  This file is linux driver for LM75 temperature sensor via I2C
 *                      After installing module under /sys/bus/i2c/devices/0-0048 driver
 *                      will create files as follow
 *                      temp_input(R) - used to read current temperature from sensor
 *                      examples:
 *                      cat /sys/bus/i2c/devices/0-0048/temp_input
 *                      temp_max(RW)  - used to write/read maximum temperature
 *                      exaples:
 *                      cat /sys/bus/i2c/devices/0-0048/temp_max
 *                      echo 85000 > /sys/bus/i2c/devices/0-0048/temp_max
 *                      temp_hyst(RW) - used to write/read temp hysteresis value
 *
 *                      All temperature are multipliet by 100 so
 *                      38.125C = 38125
 ****************************************************************************/

/****************************************************************************
 * Standard Library Includes
 ****************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/ctype.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include "linux/hwmon.h"

/****************************************************************************
 * Project Includes
 ****************************************************************************/
#include "linux/ipa/lm75.h"

/****************************************************************************
  Private Definitions
 ****************************************************************************/
#define LM75_INIT_TEMP_MAX  80000
#define LM75_INIT_TEMP_HYST 70000

/* LM75 register definitions */
#define LM75_REG_TEMP_INPUT 0x00
#define LM75_REG_CONF       0x01
#define LM75_REG_TEMP_HYST  0x02
#define LM75_REG_TEMP_MAX   0x03

/* LM75 temperature range and resolution */
#define LM_TEMP_MIN             (-55)
#define LM_TEMP_MAX             (125)
#define LM_TEMP_RESOLUTION      (125)
#define LM_TEMP_DATA_SHIFT      (5)

/* Converter from temp to register value */
#define LM75_TEMP_MIN_S32 (((s32)LM_TEMP_MIN)*LM_TEMP_RESOLUTION)
#define LM75_TEMP_MAX_S32 (((s32)LM_TEMP_MAX)*LM_TEMP_RESOLUTION)

/* Helper macro to create show value routine used by sysfs */
#define show(value) \
static ssize_t \
show_##value(struct device  *dev,                   \
             struct device_attribute  *attr,        \
             char                     *buf )        \
{                                                   \
    struct i2c_client *client = to_i2c_client(dev); \
    struct lm75_data *data =                        \
        i2c_get_clientdata(client);                 \
                                                    \
    mutex_lock(&data->update_lock);                 \
    lm75_update_from_device(client);                \
    mutex_unlock(&data->update_lock);               \
    return sprintf(buf, "%d\n", data->value);       \
}

/* Helper macro to create set value routine used by sysfs */
#define set(value, reg) \
static ssize_t                                      \
set_##value(struct device            *dev,          \
            struct device_attribute  *attr,         \
            const char               *buf,          \
            size_t                    count)        \
{                                                   \
    struct i2c_client *client = to_i2c_client(dev); \
    struct lm75_data *data =                        \
        i2c_get_clientdata(client);                 \
    int temp = simple_strtoul(buf, NULL, 10);       \
                                                    \
    mutex_lock(&data->update_lock);                 \
    data->value = temp;                             \
    lm75_update_to_device(client);                  \
    mutex_unlock(&data->update_lock);               \
    return count;                                   \
}

/****************************************************************************
  Private Types
 ****************************************************************************/

/* Each client has this additional data */
struct lm75_data {
    struct i2c_client       *client;
    struct miscdevice       miscdev;
    struct mutex            update_lock;    /* Required for read-modify-write of config reg */
    char                    valid;          /* !=0 if registers are valid */
    unsigned long           last_updated;   /* In jiffies */
    s32                     temp_input;
    s32                     temp_max;
    s32                     temp_hyst;
};

/****************************************************************************
 * Private Variables (Must be declared static)
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Constants
 ****************************************************************************/

/****************************************************************************
 * Exported Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables (Must be declared static)
 ****************************************************************************/

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x48, I2C_CLIENT_END };

/* Insmod parameters */
static const struct i2c_device_id lm75_id[] = {
    { "ipa2g", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, lm75_id);

/* Local private data structure */
static struct lm75_data  *lm75_device = NULL;

/* Convert temperature value to I2C value */
static inline u16 LM75_TEMP_TO_REG(s32 temp)
{
    return (u16)((temp / LM_TEMP_RESOLUTION) << LM_TEMP_DATA_SHIFT);
}

/* Convert I2C value into temperature value */
static inline int LM75_TEMP_FROM_REG(u16 reg)
{
    return ((s32)((s16)reg / (1<<LM_TEMP_DATA_SHIFT)) * LM_TEMP_RESOLUTION);
}

/*-----------------------------------------------------------------------*/
/* register access */

/* All registers are word-sized, except for the configuration register.
   LM75 uses a high-byte first convention, which is exactly opposite to
   the SMBus standard. */
static int lm75_read_value(struct i2c_client *client, u8 reg)
{
    int value;
    if (reg == LM75_REG_CONF)
        return i2c_smbus_read_byte_data(client, reg);

    value = i2c_smbus_read_word_data(client, reg);
    return (value < 0) ? value : swab16(value);
}

static int lm75_write_value(struct i2c_client *client, u8 reg, u16 value)
{
    if (reg == LM75_REG_CONF)
        return i2c_smbus_write_byte_data(client, reg, value);
    else
        return i2c_smbus_write_word_data(client, reg, swab16(value));
}

static void lm75_update_from_device( struct i2c_client *client)
{
    struct lm75_data *data = i2c_get_clientdata(client);

    if (time_after(jiffies, data->last_updated + HZ + HZ / 2) || !data->valid) {
        int status;

        dev_dbg(&client->dev, "Starting ipa2g update\n");

        status = lm75_read_value(client, LM75_REG_TEMP_INPUT);
        if (status < 0)
            dev_dbg(&client->dev, "reg %d, err %d\n", LM75_REG_TEMP_INPUT, status);
        else
            data->temp_input = LM75_TEMP_FROM_REG((u16)status);

        status = lm75_read_value(client, LM75_REG_TEMP_MAX);
        if (status < 0)
            dev_dbg(&client->dev, "reg %d, err %d\n", LM75_REG_TEMP_MAX, status);
        else
            data->temp_max = LM75_TEMP_FROM_REG((u16)status);

        status = lm75_read_value(client, LM75_REG_TEMP_HYST);
        if (status < 0)
            dev_dbg(&client->dev, "reg %d, err %d\n", LM75_REG_TEMP_HYST, status);
        else
            data->temp_hyst = LM75_TEMP_FROM_REG((u16)status);

        data->last_updated = jiffies;
        data->valid = 1;
    }
}

static void lm75_update_to_device( struct i2c_client *client)
{
    struct lm75_data *data = i2c_get_clientdata(client);

    int status;

    dev_dbg(&client->dev, "Starting ipa2g store \n");

    status = lm75_write_value(client, LM75_REG_TEMP_MAX, LM75_TEMP_TO_REG(data->temp_max));
    if (status < 0)
        dev_dbg(&client->dev, "reg %d, err %d\n", LM75_REG_TEMP_MAX, status);

    status = lm75_write_value(client, LM75_REG_TEMP_HYST, LM75_TEMP_TO_REG(data->temp_hyst));
    if (status < 0)
        dev_dbg(&client->dev, "reg %d, err %d\n", LM75_REG_TEMP_HYST, status);
}

/* Create function definitions for sysfs files */
show(temp_max);
show(temp_hyst);
show(temp_input);
set(temp_max, REG_TEMP_OS);
set(temp_hyst, REG_TEMP_HYST);

static DEVICE_ATTR(temp_max,    S_IWUSR | S_IRUGO, show_temp_max,   set_temp_max);
static DEVICE_ATTR(temp_hyst,   S_IWUSR | S_IRUGO, show_temp_hyst,  set_temp_hyst);
static DEVICE_ATTR(temp_input,  S_IWUSR | S_IRUGO, show_temp_input, 0);

static int lm75_open(struct inode  *inode, struct file   *filp)
{
	int                   result = 0;
    filp->private_data  = lm75_device;
    return result;
}

static int lm75_release(struct inode  *inode, struct file   *filp)
{
    if (filp->private_data) {
        filp->private_data = 0;
    }
    return 0;
}

static int lm75_ioctl(  struct inode   *inode_p,
                        struct file    *filp,
                        unsigned int    cmd,
                        unsigned long   arg)
{
    struct lm75_data  *lm75 = NULL;
    int               ret     = 0;

    /* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
    if (_IOC_TYPE(cmd) != LM75_IOCTL_MAGIC) return -ENOTTY;
    if (_IOC_NR(cmd)    > LM75_IOCTL_MAXNR) return -ENOTTY;

    lm75 = filp->private_data;

    switch (cmd)
    {
        case LM75_IOCTL_READ_TEMP:
            mutex_lock(&lm75->update_lock);
            lm75_update_from_device(lm75->client);
            mutex_unlock(&lm75->update_lock);

            if (copy_to_user((void*)arg, &lm75->temp_input, sizeof(lm75->temp_input))) {
                ret = -EFAULT;
            }
            break;

        case LM75_IOCTL_READ_PARAMS:
            {
                LM75Params_t params;

                mutex_lock(&lm75->update_lock);
                lm75_update_from_device(lm75->client);
                mutex_unlock(&lm75->update_lock);

                params.temp_hyst = lm75->temp_hyst;
                params.temp_max = lm75->temp_max;

                if (copy_to_user((LM75Params_t *)arg, &params, sizeof(params))) {
                    ret = -EFAULT;
                }
            }
            break;

        case LM75_IOCTL_WRITE_PARAMS:
            {
                LM75Params_t params;

                if (copy_from_user(&params, (void*)arg, sizeof(params)))
                {
                    ret = -EFAULT;
                    break;
                }

                mutex_lock(&lm75->update_lock);

                lm75->temp_hyst = params.temp_hyst;
                lm75->temp_max = params.temp_max;

                lm75_update_to_device(lm75->client);

                mutex_unlock(&lm75->update_lock);
            }
            break;
    }

    return ret;
}

static struct file_operations lm75_fops = {
	.owner      = THIS_MODULE,
	.open       = lm75_open,
	.release    = lm75_release,
	//.ioctl      = lm75_ioctl, //NA1 TODO
};

/* Return 0 if detection is successful, -ENODEV otherwise */
static int lm75_detect(struct i2c_client      *client,
                         struct i2c_board_info  *info)
{
    struct i2c_adapter *adapter = client->adapter;

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA | I2C_FUNC_SMBUS_READ_BYTE))
        return -ENODEV;

    strlcpy(info->type, "ipa2g", I2C_NAME_SIZE);

    return 0;
}

static int lm75_probe(struct i2c_client           *client,
                      const struct i2c_device_id  *id)
{
    struct device       *dev      = &client->dev;
    struct lm75_data    *lm75  = NULL;
    int                  err;


    if (!i2c_check_functionality(client->adapter,
            I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA))
        return -EIO;


    lm75 = kzalloc(sizeof(struct lm75_data), GFP_KERNEL);
    if (lm75 == NULL)
    {
        err = -ENOMEM;
        goto err_no_mem;
    }

    lm75->client = client;

    lm75->miscdev.fops   = &lm75_fops;
    lm75->miscdev.name   = "ipa2g";
    lm75->miscdev.minor  =  2;  /* Our /dev is read only, so use a fixed value */

    err = misc_register( &lm75->miscdev );
    if ( err )
    {
        printk( KERN_INFO "failed to register ipa2g misc device\n" );
        goto err_no_misc_device;
    }

    /* Init i2c_client */
    i2c_set_clientdata(client, lm75);
    mutex_init(&lm75->update_lock);

    err = device_create_file(dev, &dev_attr_temp_input);
    if (err)
    {
        goto err_no_sysfs_temp_input;
    }

    err = device_create_file(dev, &dev_attr_temp_max);
    if (err)
    {
        goto err_no_sysfs_temp_max;
    }

    err = device_create_file(dev, &dev_attr_temp_hyst);
    if (err)
    {
        goto err_no_sysfs_temp_hyst;
    }

    lm75_device = lm75;

    /* Program default values to temp sensor according to HW spec */
    lm75->temp_max = LM75_INIT_TEMP_MAX;
    lm75->temp_hyst = LM75_INIT_TEMP_HYST;
    lm75_update_to_device(lm75->client);
    return 0;

err_no_sysfs_temp_hyst:
    device_remove_file( dev, &dev_attr_temp_max);
err_no_sysfs_temp_max:
    device_remove_file( dev, &dev_attr_temp_input);
err_no_sysfs_temp_input:
    misc_deregister( &lm75->miscdev );
err_no_misc_device:
    kfree(lm75);
err_no_mem:
    return err;
}

static int lm75_remove(struct i2c_client *client)
{
    struct lm75_data    *lm75 = i2c_get_clientdata(client);
    struct device       *dev    = &client->dev;

    lm75_device = NULL;

    device_remove_file( dev, &dev_attr_temp_input);
    device_remove_file( dev, &dev_attr_temp_max);
    device_remove_file( dev, &dev_attr_temp_hyst);

    misc_deregister( &lm75->miscdev );
    kfree(lm75);
	return 0;
}

static struct i2c_driver lm75_driver = {
	.driver = {
		.name	= "ipa2g",
	},
	.probe		= lm75_probe,
	.remove		= lm75_remove,
	.id_table	= lm75_id,
	.detect		= lm75_detect,
};

static int __init lm75_init(void)
{
    return i2c_add_driver(&lm75_driver);
}

static void __exit lm75_exit(void)
{
    i2c_del_driver(&lm75_driver);
}


MODULE_DESCRIPTION("Driver for ipa2g Temp. Sens.");
MODULE_AUTHOR("ip.access Ltd");
MODULE_LICENSE("GPL");

module_init(lm75_init);
module_exit(lm75_exit);

