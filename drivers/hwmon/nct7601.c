// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * nct7601 - Driver for Nuvoton NCT7601
 *
 * Copyright (C) 2014  Guenter Roeck <linux@roeck-us.net>
 * Copyright (C) 2020  Konstantin Klubnichkin <kitsok@nebius.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define DRVNAME "nct7601"

#define REG_CONFIG			0x10	/*
									[7] INIT : 1 indicates initial reset
									[2] CONV_RATE : 0 : low power conversion mode(2Hz); 1 : continuous conversion mode
									[1] ALERT_MSK : 1 : ALERT function is enable; 0 : ALERT function is disable
									[0] START : 1 indicates start monitoring
									*/

#define REG_CONFIG_DEFAULT	0x05    /* Start ADC; disable alerts; set conversion mode continuous */
#define REG_ADV_CONFIG		0x11	/*
									[7]   MOD_RSTIN# : 1 indicates RSTIN#=0 will reset whole chip
									[5]   EN_SMB_TMOUT : 1 indicates SMBus time-out is enable
									[4]   EN_DEEP_SHUTDOWN : 1 indicates deep shut-down is enable
									[3:2] FAULT_QUEUE[1:0] : 00=1 time (default), 01=2 times, 10=4 times, 11=6 times
									[1]   MOD_STS : 1=interrupt status (read status & clear), 0=real-time status
									[0]   MOD_ALERT : ALERT output mode: 1=interrupt mode, 0=compare interrupt mode
									*/

#define REG_CHAN_INP_MODE   0x12	/*
									DN0: 1=D- connect to ground, 0=bias to 0.3~0.4V
									MOD_INX[5:0] : VIN mode: 1=pseudo-differential mode; 0=single-ended mode
									Bit[0]: VIN1~2 (DVIN1_2)
									Bit[1]: VIN3~4 (DVIN3_4)
									Bit[2]: VIN5~6 (DVIN5_6)
									Bit[3]: VIN7~8 (DVIN7_8)
									Bit[4]: VIN9~10 (DVIN9_10)
									Bit[5]: VIN11~12 (DVIN11_12)
									*/
#define CHAN_MODE_DEFAULT	0x0

#define REG_CHAN_EN_1		0x13	/*
									[7] EN_TR8 : enable TR8 channel
									[6] EN_TR7/EN_TD4 : enable TR7/TD4 channel
									[5] EN_TR6 : enable TR6 channel
									[4] EN_TR5/EN_TD3 : enable TR5/TD3 channel
									[3] EN_TR4 : enable TR4 channel
									[2] EN_TR3/EN_TD2 : enable TR3/TD2 channel
									[1] EN_TR2 : enable TR2 channel
									[0] EN_TR1/EN_TD1 : enable TR1/TD1 channel
									*/

#define CHAN_EN_1_THERM		0xff
#define CHAN_EN_1_DIODE		0x55

#define REG_CHAN_EN_2		0x14	/*
									[3] EN_TR12 : enable TR12 channel
									[2] EN_TR11/EN_TD6 : enable TR11/TD6 channel
									[1] EN_TR10 : enable TR10 channel
									[0] EN_TR9/EN_TD5 : enable TR9/TD5 channel
									*/
#define CHAN_EN_2_DEFAULT	0x0

#define REG_INT_MASK_1		0x15	/*
									[7] MSK_TR8 : mask TR8 channel interrupt
									[6] MSK_TR7/MSK_TD4 : mask TR7/TD4 channel interrupt
									[5] MSK_TR6 : mask TR6 channel interrupt
									[4] MSK_TR5/MSK_TD3 : mask TR5/TD3 channel interrupt
									[3] MSK_TR4 : mask TR4 channel interrupt
									[2] MSK_TR3/MSK_TD2 : mask TR3/TD2 channel interrupt
									[1] MSK_TR2 : mask TR2 channel interrupt
									[0] MSK_TR1/MSK_TD1 : mask TR1/TD1 channel interrupt
									*/

#define REG_INT_MASK_2		0x16	/*
									[3] MSK_TR12 : mask TR12 channel interrupt
									[2] MSK_TR11/MSK_TD6 : mask TR11/TD6 channel interrupt
									[1] MSK_TR10 : mask TR2 channel interrupt
									[0] MSK_TR9/MSK_TD5 : mask TR9/TD5 channel interrupt
									*/

#define REG_BUSY_STS		0x1e	/*
									[1] PWR_UP : 1 indicates power is ok
									[0] BUSY : 1 indicates ADC is busy
									*/

#define REG_ONE_SHOT		0x1f	/*
									[0] ONE_SHOT : write 1 ADC will monitor one time
									*/

#define REG_SMBUS_ADDR		0xfc	/*
									[7]   ADDRFEH_EN : mask TR8 channel interrupt
									[6:0] SMBUS_ADDRESS[6:0] : mask TR7/TD4 channel interrupt
									*/

#define REG_CHIP_ID			0xfd	/*
									CHIP_ID[7:0]: Chip ID of NCT7601/NCT7602 (0xd7)
									*/

#define REG_VENDOR_ID		0xfe	/*
									VENDOR_ID[7:0]: Vendor ID of NCT7601/NCT7602 (0x50)
									*/

#define REG_DEVICE_ID		0xff	/*
									DEVICE_ID[7:0]: Device ID of NCT7601/NCT7602 (0x13)
									*/

#define REG_TEMP_LSB		0x0f

#define REG_MNTTR_BASE		0x0

#define SENSOR_TYPE_THERMISTOR	1
#define SENSOR_TYPE_DIODE		2

/*
 * Data structures and manipulation thereof
 */

struct nct7601_data {
	struct regmap *regmap;
	struct mutex access_lock; /* for multi-byte read and write operations */
	u8 sensor_type;
	u8 channels_number;
	bool initialized;
};

static void nct7601_init_chip(struct nct7601_data *data)
{
	int err;
	u8 reg;

	data->initialized = false;
	/* Enable ADC */
	err = regmap_write(data->regmap, REG_CONFIG, REG_CONFIG_DEFAULT);
	if (err)
		return;

	/* Set default channel mode */
	err = regmap_write(data->regmap, REG_CHAN_INP_MODE, CHAN_MODE_DEFAULT);
	if (err)
		return;

	reg = CHAN_EN_1_THERM;
	if (data->sensor_type == SENSOR_TYPE_DIODE)
		reg = CHAN_EN_1_DIODE;

	err = regmap_write(data->regmap, REG_CHAN_EN_1, reg);
	if (err)
		return;

	err = regmap_write(data->regmap, REG_CHAN_EN_2, CHAN_EN_2_DEFAULT);
	if (err)
		return;

	data->initialized = true;
}

static int nct7601_read_temp(struct device *dev, struct nct7601_data *data,
			     u8 idx, u8 reg_temp_low, int *temp)
{
	unsigned int t1, t2 = 0;
	u8 regs[2];
	int err = -ETIMEDOUT;
    int t;

	*temp = 0;

	dev_dbg(dev, "%s: idx=%i\n", __FUNCTION__, idx);

	if (idx > data->channels_number)
		return -ENODEV;

	mutex_lock(&data->access_lock);
	if (! data->initialized)
		nct7601_init_chip(data);

	if (! data->initialized)
		goto abort;

	err = regmap_bulk_read(data->regmap, REG_MNTTR_BASE + idx - 1, regs, 2);
	dev_dbg(dev, "%s: reading from 0x%02x regs[0]=0x%02x regs[1]=0x%02x err=%i\n",
				__FUNCTION__, REG_MNTTR_BASE + idx - 1,
				regs[0], regs[1], err);
	if (err < 0)
		goto abort;
	*temp = ((s8) regs[0]) * 1000;
	*temp += (regs[1] >> 7 ) * 500;
	dev_dbg(dev, "%s: regs[0]=0x%02x regs[1]=0x%02x temp=%i\n", __FUNCTION__,
			regs[0], regs[1], *temp);

abort:
	mutex_unlock(&data->access_lock);
	return err;
}

static ssize_t temp_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct nct7601_data *data = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);
	int err, temp;

	err = nct7601_read_temp(dev, data, sattr->nr, sattr->index, &temp);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", temp);
}

static SENSOR_DEVICE_ATTR_2_RO(temp1_input, temp, 0x01, 0);
static SENSOR_DEVICE_ATTR_2_RO(temp2_input, temp, 0x02, 0);
static SENSOR_DEVICE_ATTR_2_RO(temp3_input, temp, 0x03, 0);
static SENSOR_DEVICE_ATTR_2_RO(temp4_input, temp, 0x04, 0);
static SENSOR_DEVICE_ATTR_2_RO(temp5_input, temp, 0x05, 0);
static SENSOR_DEVICE_ATTR_2_RO(temp6_input, temp, 0x06, 0);
static SENSOR_DEVICE_ATTR_2_RO(temp7_input, temp, 0x07, 0);
static SENSOR_DEVICE_ATTR_2_RO(temp8_input, temp, 0x08, 0);

static struct attribute *nct7601_temp_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp4_input.dev_attr.attr,
	&sensor_dev_attr_temp5_input.dev_attr.attr,
	&sensor_dev_attr_temp6_input.dev_attr.attr,
	&sensor_dev_attr_temp7_input.dev_attr.attr,
	&sensor_dev_attr_temp8_input.dev_attr.attr,

	NULL
};

static const struct attribute_group nct7601_temp_group = {
	.attrs = nct7601_temp_attrs,
};

static const struct attribute_group *nct7601_groups[] = {
	&nct7601_temp_group,
	NULL
};

static int nct7601_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	int reg;

	reg = i2c_smbus_read_byte_data(client, REG_VENDOR_ID);
	if (reg != 0x50)
		return -ENODEV;

	reg = i2c_smbus_read_byte_data(client, REG_CHIP_ID);
	if (reg != 0xd7)
		return -ENODEV;

	reg = i2c_smbus_read_byte_data(client, REG_DEVICE_ID);
	if (reg < 0 || (reg != 0x13))
		return -ENODEV;

	/* Also validate lower bits of temperature registers */
	reg = i2c_smbus_read_byte_data(client, REG_TEMP_LSB);
	if (reg < 0 || (reg & 0x1f))
		return -ENODEV;

	strlcpy(info->type, "nct7601", I2C_NAME_SIZE);
	return 0;
}

static const struct regmap_config nct7601_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static int nct7601_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct nct7601_data *data;
	struct device *hwmon_dev;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	/* By default consider sensor type as thermistor */
	data->sensor_type = SENSOR_TYPE_THERMISTOR;
	data->channels_number = 8;

#ifdef CONFIG_OF
	if (dev->of_node) {
		if (of_property_read_bool(dev->of_node, "sensor-diode")) {
			data->sensor_type = SENSOR_TYPE_DIODE;
			data->channels_number = 4;
		}
	}
#endif

	data->regmap = devm_regmap_init_i2c(client, &nct7601_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	mutex_init(&data->access_lock);

	data->initialized = false;
	nct7601_init_chip(data);

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
							   data,
							   nct7601_groups);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const unsigned short nct7601_address_list[] = {
	0x1d, 0x1e, 0x35, 0x36, I2C_CLIENT_END
};

static const struct i2c_device_id nct7601_idtable[] = {
	{ "nct7601", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nct7601_idtable);

static const struct __maybe_unused of_device_id nct7601_dt_match[] = {
	{ .compatible = "nuvoton,nct7601" },
	{ },
};

static struct i2c_driver nct7601_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = DRVNAME,
		.of_match_table = of_match_ptr(nct7601_dt_match),
	},
	.detect = nct7601_detect,
	.probe = nct7601_probe,
	.id_table = nct7601_idtable,
	.address_list = nct7601_address_list,
};

module_i2c_driver(nct7601_driver);

MODULE_AUTHOR("Konstantin Klubnichkin <kitsok@nebius.com>");
MODULE_DESCRIPTION("NCT7601 Hardware Monitoring Driver");
MODULE_LICENSE("GPL v2");
