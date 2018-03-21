/*
 * Copyright (C) 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include "pmbus.h"

enum max31785_regs {
	MFR_REVISION		= 0x9b,
	MFR_FAULT_RESPONSE	= 0xd9,
	MFR_TEMP_SENSOR_CONFIG	= 0xf0,
	MFR_FAN_CONFIG		= 0xf1,
	MFR_READ_FAN_PWM	= 0xf3,
	MFR_FAN_FAULT_LIMIT	= 0xf5,
	MFR_FAN_WARN_LIMIT	= 0xf6,
	MFR_FAN_PWM_AVG		= 0xf8,
};

#define MAX31785			0x3030
#define MAX31785A			0x3040

#define MFR_TEMP_SENSOR_CONFIG_ENABLE	BIT(15)
#define MFR_TEMP_SENSOR_CONFIG_OFFSET	GENMASK(14, 10)

#define MFR_FAN_CONFIG_FREQ		GENMASK(15, 13)
#define MFR_FAN_CONFIG_DUAL_TACH	BIT(12)
#define MFR_FAN_CONFIG_HYS		GENMASK(11, 10)
#define MFR_FAN_CONFIG_TSFO		BIT(9)
#define MFR_FAN_CONFIG_TACHO		BIT(8)
#define MFR_FAN_CONFIG_RAMP		GENMASK(7, 5)
#define MFR_FAN_CONFIG_HEALTH		BIT(4)
#define MFR_FAN_CONFIG_ROTOR_HI_LO	BIT(3)
#define MFR_FAN_CONFIG_ROTOR		BIT(2)
#define MFR_FAN_CONFIG_SPIN		GENMASK(1, 0)

#define MFR_FAULT_RESPONSE_MONITOR	BIT(0)

#define MAX31785_CAP_READ_DUAL_TACH	BIT(0)

#define MAX31785_NR_PAGES		23

/*
 * MAX31785 dragons ahead
 *
 * It seems there's an undocumented timing constraint when performing
 * back-to-back writes where the first write is to FAN_CONFIG_1_2. The device
 * provides no indication of this besides NACK'ing master Txs; no bits are set
 * in STATUS_BYTE to suggest anything has gone wrong.
 *
 * Given that, do a one-shot retry of the write.
 *
 * The max31785_*_write_*_data() functions should be used at any point where
 * the prior write is to FAN_CONFIG_1_2.
 */
static int max31785_i2c_smbus_write_byte_data(struct i2c_client *client,
					      int command, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, command, data);
	if (ret == -EIO)
		ret = i2c_smbus_write_byte_data(client, command, data);

	return ret;
}

static int max31785_i2c_smbus_write_word_data(struct i2c_client *client,
					      int command, u16 data)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, command, data);
	if (ret == -EIO)
		ret = i2c_smbus_write_word_data(client, command, data);

	return ret;
}

static int max31785_pmbus_write_word_data(struct i2c_client *client, int page,
					  int command, u16 data)
{
	int ret;

	ret = pmbus_write_word_data(client, page, command, data);
	if (ret == -EIO)
		ret = pmbus_write_word_data(client, page, command, data);

	return ret;
}

static int max31785_read_byte_data(struct i2c_client *client, int page,
				   int reg)
{
	switch (reg) {
	case PMBUS_VOUT_MODE:
		if (page < MAX31785_NR_PAGES)
			return -ENODATA;

		return -ENOTSUPP;
	case PMBUS_FAN_CONFIG_12:
		if (page < MAX31785_NR_PAGES)
			return -ENODATA;

		return pmbus_read_byte_data(client, page - MAX31785_NR_PAGES,
					    reg);
	}

	return -ENODATA;
}

static int max31785_write_byte(struct i2c_client *client, int page, u8 value)
{
	if (page < MAX31785_NR_PAGES)
		return -ENODATA;

	return -ENOTSUPP;
}

static int max31785_read_long_data(struct i2c_client *client, int page,
				   int reg, u32 *data)
{
	unsigned char cmdbuf[1];
	unsigned char rspbuf[4];
	int rc;

	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmdbuf),
			.buf = cmdbuf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(rspbuf),
			.buf = rspbuf,
		},
	};

	cmdbuf[0] = reg;

	rc = pmbus_set_page(client, page);
	if (rc < 0)
		return rc;

	rc = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (rc < 0)
		return rc;

	*data = (rspbuf[0] << (0 * 8)) | (rspbuf[1] << (1 * 8)) |
		(rspbuf[2] << (2 * 8)) | (rspbuf[3] << (3 * 8));

	return rc;
}

static int max31785_get_pwm(struct i2c_client *client, int page)
{
	int config;
	int command;

	config = pmbus_read_byte_data(client, page, PMBUS_FAN_CONFIG_12);
	if (config < 0)
		return config;

	command = pmbus_read_word_data(client, page, PMBUS_FAN_COMMAND_1);
	if (command < 0)
		return command;

	if (!(config & PB_FAN_1_RPM)) {
		if (command >= 0x8000)
			return 0;
		else if (command >= 0x2711)
			return 0x2710;

		return command;
	}

	return 0;
}

static int max31785_get_pwm_mode(struct i2c_client *client, int page)
{
	int config;
	int command;

	config = pmbus_read_byte_data(client, page, PMBUS_FAN_CONFIG_12);
	if (config < 0)
		return config;

	command = pmbus_read_word_data(client, page, PMBUS_FAN_COMMAND_1);
	if (command < 0)
		return command;

	if (!(config & PB_FAN_1_RPM)) {
		if (command >= 0x8000)
			return 2;
		else if (command >= 0x2711)
			return 0;

		return 1;
	}

	return (command >= 0x8000) ? 2 : 1;
}

static int max31785_read_word_data(struct i2c_client *client, int page,
				   int reg)
{
	int rv;

	switch (reg) {
	case PMBUS_READ_FAN_SPEED_1:
	{
		u32 val;

		if (page < MAX31785_NR_PAGES)
			return -ENODATA;

		rv = max31785_read_long_data(client, page - MAX31785_NR_PAGES,
					     reg, &val);
		if (rv < 0)
			return rv;

		rv = (val >> 16) & 0xffff;
		break;
	}
	case PMBUS_VIRT_PWM_1:
		if (page >= MAX31785_NR_PAGES)
			return -ENOTSUPP;

		rv = max31785_get_pwm(client, page);
		if (rv < 0)
			return rv;

		rv *= 255;
		rv /= 100;
		break;
	case PMBUS_VIRT_PWM_ENABLE_1:
		if (page >= MAX31785_NR_PAGES)
			return -ENOTSUPP;

		rv = max31785_get_pwm_mode(client, page);
		break;
	default:
		rv = (page >= MAX31785_NR_PAGES) ? -ENXIO : -ENODATA;
		break;
	}

	return rv;
}

static int max31785_update_fan(struct i2c_client *client, int page,
			       u8 config, u8 mask, u16 command)
{
	int from, rv;
	u8 to;

	from = pmbus_read_byte_data(client, page, PMBUS_FAN_CONFIG_12);
	if (from < 0)
		return from;

	to = (from & ~mask) | (config & mask);

	if (to != from) {
		rv = pmbus_write_byte_data(client, page, PMBUS_FAN_CONFIG_12,
					   to);
		if (rv < 0)
			return rv;
	}

	rv = max31785_pmbus_write_word_data(client, page, PMBUS_FAN_COMMAND_1,
					    command);

	return rv;
}

static const int max31785_pwm_modes[] = { 0x7fff, 0x2710, 0xffff };

static int max31785_write_word_data(struct i2c_client *client, int page,
				    int reg, u16 word)
{
	if (page >= MAX31785_NR_PAGES)
		return -ENXIO;

	switch (reg) {
	case PMBUS_VIRT_FAN_TARGET_1:
		return max31785_update_fan(client, page, PB_FAN_1_RPM,
					   PB_FAN_1_RPM, word);
	case PMBUS_VIRT_PWM_1:
	{
		u32 command = word;

		command *= 100;
		command /= 255;
		return max31785_update_fan(client, page, 0, PB_FAN_1_RPM,
					   command);
	}
	case PMBUS_VIRT_PWM_ENABLE_1:
		if (word >= ARRAY_SIZE(max31785_pwm_modes))
			return -EINVAL;

		return max31785_update_fan(client, page, 0, PB_FAN_1_RPM,
					   max31785_pwm_modes[word]);
	default:
		break;
	}

	return -ENODATA;
}

/*
 * Returns negative error codes if an unrecoverable problem is detected, 0 if a
 * recoverable problem is detected, or a positive value on success.
 */
static int max31785_of_fan_config(struct i2c_client *client,
				  struct pmbus_driver_info *info,
				  u32 capabilities, struct device_node *child)
{
	int mfr_cfg = 0, mfr_fault_resp = 0, pb_cfg;
	struct device *dev = &client->dev;
	char *lock_polarity = NULL;
	const char *sval;
	u32 page;
	u32 uval;
	int ret;

	if (!of_device_is_compatible(child, "pmbus-fan"))
		return 0;

	ret = of_property_read_u32(child, "reg", &page);
	if (ret < 0) {
		dev_err(&client->dev, "Missing valid reg property\n");
		return ret;
	}

	if (!(info->func[page] & PMBUS_HAVE_FAN12)) {
		dev_err(dev, "Page %d does not have fan capabilities\n", page);
		return -ENXIO;
	}

	ret = max31785_i2c_smbus_write_byte_data(client, PMBUS_PAGE, page);
	if (ret < 0)
		return ret;

	pb_cfg = i2c_smbus_read_byte_data(client, PMBUS_FAN_CONFIG_12);
	if (pb_cfg < 0)
		return pb_cfg;

	if (of_property_read_bool(child->parent, "use-stored-presence")) {
		if (!(pb_cfg & PB_FAN_1_INSTALLED))
			dev_info(dev, "Fan %d is configured but not installed\n",
				 page);
	} else {
		pb_cfg |= PB_FAN_1_INSTALLED;
	}

	ret = of_property_read_string(child, "maxim,fan-rotor-input", &sval);
	if (ret < 0) {
		dev_err(dev, "Missing valid maxim,fan-rotor-input property for fan %d\n",
				page);
		return ret;
	}

	if (strcmp("tach", sval) && strcmp("lock", sval)) {
		dev_err(dev, "maxim,fan-rotor-input has invalid value for fan %d: %s\n",
				page, sval);
		return -EINVAL;
	} else if (!strcmp("lock", sval)) {
		mfr_cfg |= MFR_FAN_CONFIG_ROTOR;

		ret = i2c_smbus_write_word_data(client, MFR_FAN_FAULT_LIMIT, 1);
		if (ret < 0)
			return ret;

		ret = of_property_read_string(child, "maxim,fan-lock-polarity",
					      &sval);
		if (ret < 0) {
			dev_err(dev, "Missing valid maxim,fan-lock-polarity property for fan %d\n",
					page);
			return ret;
		}

		if (strcmp("low", sval) && strcmp("high", sval)) {
			dev_err(dev, "maxim,fan-lock-polarity has invalid value for fan %d: %s\n",
					page, lock_polarity);
			return -EINVAL;
		} else if (!strcmp("high", sval))
			mfr_cfg |= MFR_FAN_CONFIG_ROTOR_HI_LO;
	}

	if (!of_property_read_string(child, "fan-mode", &sval)) {
		if (!strcmp("rpm", sval))
			pb_cfg |= PB_FAN_1_RPM;
		else if (!strcmp("pwm", sval))
			pb_cfg &= ~PB_FAN_1_RPM;
		else {
			dev_err(dev, "fan-mode has invalid value for fan %d: %s\n",
					page, sval);
			return -EINVAL;
		}
	}

	ret = of_property_read_u32(child, "tach-pulses", &uval);
	if (ret < 0) {
		pb_cfg &= ~PB_FAN_1_PULSE_MASK;
	} else if (uval && (uval - 1) < 4) {
		pb_cfg = ((pb_cfg & ~PB_FAN_1_PULSE_MASK) | ((uval - 1) << 4));
	} else {
		dev_err(dev, "tach-pulses has invalid value for fan %d: %u\n",
				page, uval);
		return -EINVAL;
	}

	if (of_property_read_bool(child, "maxim,fan-health"))
		mfr_cfg |= MFR_FAN_CONFIG_HEALTH;

	if (of_property_read_bool(child, "maxim,fan-no-watchdog") ||
		of_property_read_bool(child, "maxim,tmp-no-fault-ramp"))
		mfr_cfg |= MFR_FAN_CONFIG_TSFO;

	if (of_property_read_bool(child, "maxim,fan-dual-tach")) {
		mfr_cfg |= MFR_FAN_CONFIG_DUAL_TACH;

		if (capabilities & MAX31785_CAP_READ_DUAL_TACH) {
			int virtual = MAX31785_NR_PAGES + page;

			info->pages = max(info->pages, virtual + 1);
			info->func[virtual] |= PMBUS_HAVE_FAN12;
			info->func[virtual] |= PMBUS_PAGE_VIRTUAL;
		}
	}

	if (of_property_read_bool(child, "maxim,fan-no-fault-ramp"))
		mfr_cfg |= MFR_FAN_CONFIG_TACHO;

	if (!of_property_read_u32(child, "maxim,fan-startup", &uval)) {
		uval /= 2;
		if (uval < 5) {
			mfr_cfg |= uval;
		} else {
			dev_err(dev, "maxim,fan-startup has invalid value for fan %d: %u\n",
					page, uval);
			return -EINVAL;
		}
	}

	if (!of_property_read_u32(child, "maxim,fan-ramp", &uval)) {
		if (uval < 8) {
			mfr_cfg |= uval << 5;
		} else {
			dev_err(dev, "maxim,fan-ramp has invalid value for fan %d: %u\n",
					page, uval);
			return -EINVAL;
		}
	}

	if (!of_property_read_u32(child, "maxim,tmp-hysteresis", &uval)) {
		uval /= 2;
		uval -= 1;
		if (uval < 4) {
			mfr_cfg |= uval << 10;
		} else {
			dev_err(dev, "maxim,tmp-hysteresis has invalid value for fan %d, %u\n",
					page, uval);
			return -EINVAL;
		}
	}

	if (!of_property_read_u32(child, "maxim,fan-pwm-freq", &uval)) {
		u16 val;

		if (uval == 30) {
			val = 0;
		} else if (uval == 50) {
			val = 1;
		} else if (uval == 100) {
			val = 2;
		} else if (uval == 150) {
			val = 3;
		} else if (uval == 25000) {
			val = 7;
		} else {
			dev_err(dev, "maxim,fan-pwm-freq has invalid value for fan %d: %u\n",
					page, uval);
			return -EINVAL;
		}

		mfr_cfg |= val << 13;
	}

	if (of_property_read_bool(child, "maxim,fan-fault-pin-mon"))
		mfr_fault_resp |= MFR_FAULT_RESPONSE_MONITOR;

	ret = i2c_smbus_write_byte_data(client, PMBUS_FAN_CONFIG_12,
					pb_cfg & ~PB_FAN_1_INSTALLED);
	if (ret < 0)
		return ret;

	ret = max31785_i2c_smbus_write_word_data(client, MFR_FAN_CONFIG,
						 mfr_cfg);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(client, MFR_FAULT_RESPONSE,
					mfr_fault_resp);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(client, PMBUS_FAN_CONFIG_12, pb_cfg);
	if (ret < 0)
		return ret;

	/*
	 * Fans are on pages 0 - 5. If the page property of a fan node is
	 * greater than 5 we will have errored in checks above out above.
	 * Therefore we don't need to cope with values up to 31, and the int
	 * return type is enough.
	 *
	 * The bit mask return value is used to populate a bitfield of fans
	 * who are both configured in the devicetree _and_ reported as
	 * installed by the hardware. Any fans that are not configured in the
	 * devicetree but are reported as installed by the hardware will have
	 * their hardware configuration updated to unset the installed bit.
	 */
	return BIT(page);
}

static int max31785_of_tmp_config(struct i2c_client *client,
				  struct pmbus_driver_info *info,
				  struct device_node *child)
{
	struct device *dev = &client->dev;
	u16 mfr_tmp_cfg = 0;
	int nr_fans;
	u32 page;
	u32 uval;
	int ret;
	int i;

	if (!of_device_is_compatible(child, "pmbus-temperature"))
		return 0;

	ret = of_property_read_u32(child, "reg", &page);
	if (ret < 0) {
		dev_err(&client->dev, "Missing valid reg property\n");
		return ret;
	}

	if (!(info->func[page] & PMBUS_HAVE_TEMP)) {
		dev_err(dev, "Page %d does not have temp capabilities\n", page);
		return -ENXIO;
	}

	ret = max31785_i2c_smbus_write_byte_data(client, PMBUS_PAGE, page);
	if (ret < 0)
		return ret;

	if (!of_property_read_u32(child, "maxim,tmp-offset", &uval)) {
		if (uval < 32)
			mfr_tmp_cfg |= uval << 10;
	}

	nr_fans = of_property_count_elems_of_size(child, "maxim,tmp-fans",
						  sizeof(u32));
	if (nr_fans > 0) {
		mfr_tmp_cfg |= MFR_TEMP_SENSOR_CONFIG_ENABLE;

		for (i = 0; i < nr_fans; i++) {
			if (of_property_read_u32_index(child, "maxim,tmp-fans",
						       i, &uval))
				continue;

			if (uval < 6)
				mfr_tmp_cfg |= BIT(uval);
		}
	}

	ret = i2c_smbus_write_word_data(client, MFR_TEMP_SENSOR_CONFIG,
					mfr_tmp_cfg);
	if (ret < 0)
		return ret;

	return 0;
}

#define MAX31785_FAN_FUNCS \
	(PMBUS_HAVE_FAN12 | PMBUS_HAVE_STATUS_FAN12 | PMBUS_HAVE_PWM12)

#define MAX31785_TEMP_FUNCS \
	(PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP)

#define MAX31785_VOUT_FUNCS \
	(PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT)

static const struct pmbus_driver_info max31785_info = {
	.pages = MAX31785_NR_PAGES,

	.write_word_data = max31785_write_word_data,
	.read_byte_data = max31785_read_byte_data,
	.read_word_data = max31785_read_word_data,
	.write_byte = max31785_write_byte,

	/* RPM */
	.format[PSC_FAN] = direct,
	.m[PSC_FAN] = 1,
	.b[PSC_FAN] = 0,
	.R[PSC_FAN] = 0,
	/* PWM */
	.format[PSC_PWM] = direct,
	.m[PSC_PWM] = 1,
	.b[PSC_PWM] = 0,
	.R[PSC_PWM] = 2,
	.func[0] = MAX31785_FAN_FUNCS,
	.func[1] = MAX31785_FAN_FUNCS,
	.func[2] = MAX31785_FAN_FUNCS,
	.func[3] = MAX31785_FAN_FUNCS,
	.func[4] = MAX31785_FAN_FUNCS,
	.func[5] = MAX31785_FAN_FUNCS,

	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_TEMPERATURE] = 1,
	.b[PSC_TEMPERATURE] = 0,
	.R[PSC_TEMPERATURE] = 2,
	.func[6]  = MAX31785_TEMP_FUNCS,
	.func[7]  = MAX31785_TEMP_FUNCS,
	.func[8]  = MAX31785_TEMP_FUNCS,
	.func[9]  = MAX31785_TEMP_FUNCS,
	.func[10] = MAX31785_TEMP_FUNCS,
	.func[11] = MAX31785_TEMP_FUNCS,
	.func[12] = MAX31785_TEMP_FUNCS,
	.func[13] = MAX31785_TEMP_FUNCS,
	.func[14] = MAX31785_TEMP_FUNCS,
	.func[15] = MAX31785_TEMP_FUNCS,
	.func[16] = MAX31785_TEMP_FUNCS,

	.format[PSC_VOLTAGE_OUT] = direct,
	.m[PSC_VOLTAGE_OUT] = 1,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = 0,
	.func[17] = MAX31785_VOUT_FUNCS,
	.func[18] = MAX31785_VOUT_FUNCS,
	.func[19] = MAX31785_VOUT_FUNCS,
	.func[20] = MAX31785_VOUT_FUNCS,
	.func[21] = MAX31785_VOUT_FUNCS,
	.func[22] = MAX31785_VOUT_FUNCS,
};

static int max31785_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *child;
	struct pmbus_driver_info *info;
	u32 fans;
	u32 caps;
	s64 ret;
	int i;

	info = devm_kzalloc(dev, sizeof(struct pmbus_driver_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	*info = max31785_info;

	ret = i2c_smbus_write_byte_data(client, PMBUS_PAGE, 255);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_word_data(client, MFR_REVISION);
	if (ret < 0)
		return ret;

	caps = 0;
	if (!strcmp("max31785a", id->name)) {
		if (ret == MAX31785A)
			caps |= MAX31785_CAP_READ_DUAL_TACH;
		else
			dev_warn(dev, "Expected max3175a, found max31785: cannot provide secondary tachometer readings\n");
	} else if (!strcmp("max31785", id->name)) {
		if (ret == MAX31785A)
			dev_info(dev, "Expected max31785, found max3175a: suppressing secondary tachometer attributes\n");
	} else {
		return -EINVAL;
	}


	fans = 0;
	for_each_child_of_node(dev->of_node, child) {
		ret = max31785_of_fan_config(client, info, caps, child);
		if (ret < 0) {
			of_node_put(child);
			return ret;
		}

		if (ret)
			fans |= ret;

		ret = max31785_of_tmp_config(client, info, child);
		if (ret < 0) {
			of_node_put(child);
			return ret;
		}
	}

	for (i = 0; i < MAX31785_NR_PAGES; i++) {
		bool have_fan = !!(info->func[i] & PMBUS_HAVE_FAN12);
		bool fan_configured = !!(fans & BIT(i));

		if (!have_fan || fan_configured)
			continue;

		ret = max31785_i2c_smbus_write_byte_data(client, PMBUS_PAGE,
							 i);
		if (ret < 0)
			return ret;

		ret = i2c_smbus_read_byte_data(client, PMBUS_FAN_CONFIG_12);
		if (ret < 0)
			return ret;

		ret &= ~PB_FAN_1_INSTALLED;
		ret = max31785_i2c_smbus_write_word_data(client,
							 PMBUS_FAN_CONFIG_12,
							 ret);
		if (ret < 0)
			return ret;
	}

	return pmbus_do_probe(client, id, info);
}

static const struct i2c_device_id max31785_id[] = {
	{ "max31785", 0 },
	{ "max31785a", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, max31785_id);

static struct i2c_driver max31785_driver = {
	.driver = {
		.name = "max31785",
	},
	.probe = max31785_probe,
	.remove = pmbus_do_remove,
	.id_table = max31785_id,
};

module_i2c_driver(max31785_driver);

MODULE_AUTHOR("Andrew Jeffery <andrew@aj.id.au>");
MODULE_DESCRIPTION("PMBus driver for the Maxim MAX31785");
MODULE_LICENSE("GPL");
