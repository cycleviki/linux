/*
 * Hardware monitoring driver for UCD90xxx Sequencer and System Health
 * Controller series
 *
 * Copyright (C) 2011 Ericsson AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c/pmbus.h>
#include <linux/gpio.h>
#include <linux/idr.h>
#include "pmbus.h"

enum chips { ucd9000, ucd90120, ucd90124, ucd90160, ucd9090, ucd90910 };

#define UCD9000_MONITOR_CONFIG		0xd5
#define UCD9000_NUM_PAGES		0xd6
#define UCD9000_FAN_CONFIG_INDEX	0xe7
#define UCD9000_FAN_CONFIG		0xe8
#define UCD9000_MFR_STATUS		0xf3
#define UCD9000_GPIO_SELECT		0xfa
#define UCD9000_GPIO_CONFIG		0xfb
#define UCD9000_DEVICE_ID		0xfd

/* GPIO CONFIG bits */
#define UCD9000_GPIO_CONFIG_ENABLE	BIT(0)
#define UCD9000_GPIO_CONFIG_OUT_ENABLE	BIT(1)
#define UCD9000_GPIO_CONFIG_OUT_VALUE	BIT(2)
#define UCD9000_GPIO_CONFIG_STATUS	BIT(3)
#define UCD9000_GPIO_INPUT		0
#define UCD9000_GPIO_OUTPUT		1

#define UCD9000_MON_TYPE(x)	(((x) >> 5) & 0x07)
#define UCD9000_MON_PAGE(x)	((x) & 0x0f)

#define UCD9000_MON_VOLTAGE	1
#define UCD9000_MON_TEMPERATURE	2
#define UCD9000_MON_CURRENT	3
#define UCD9000_MON_VOLTAGE_HW	4

#define UCD9000_NUM_FAN		4
#define UCD9000_NAME_SIZE	24

#define UCD9000_GPIO_NAME_LEN	16
#define UCD90160_NUM_GPIOS	26
#define UCD90160_GPI_COUNT	8
#define UCD90160_GPI_FAULT_BASE	16

static DEFINE_IDA(ucd9000_ida);

struct ucd9000_data {
	u8 fan_data[UCD9000_NUM_FAN][I2C_SMBUS_BLOCK_MAX];
	struct pmbus_driver_info info;
	struct gpio_chip gpio;
	struct dentry *debugfs;
	int idx;
};
#define to_ucd9000_data(_info) container_of(_info, struct ucd9000_data, info)

struct ucd9000_debugfs_entry {
	struct i2c_client *client;
	u8 index;
};

static int ucd9000_get_fan_config(struct i2c_client *client, int fan)
{
	int fan_config = 0;
	struct ucd9000_data *data
	  = to_ucd9000_data(pmbus_get_driver_info(client));

	if (data->fan_data[fan][3] & 1)
		fan_config |= PB_FAN_2_INSTALLED;   /* Use lower bit position */

	/* Pulses/revolution */
	fan_config |= (data->fan_data[fan][3] & 0x06) >> 1;

	return fan_config;
}

static int ucd9000_read_byte_data(struct i2c_client *client, int page, int reg)
{
	int ret = 0;
	int fan_config;

	switch (reg) {
	case PMBUS_FAN_CONFIG_12:
		if (page > 0)
			return -ENXIO;

		ret = ucd9000_get_fan_config(client, 0);
		if (ret < 0)
			return ret;
		fan_config = ret << 4;
		ret = ucd9000_get_fan_config(client, 1);
		if (ret < 0)
			return ret;
		fan_config |= ret;
		ret = fan_config;
		break;
	case PMBUS_FAN_CONFIG_34:
		if (page > 0)
			return -ENXIO;

		ret = ucd9000_get_fan_config(client, 2);
		if (ret < 0)
			return ret;
		fan_config = ret << 4;
		ret = ucd9000_get_fan_config(client, 3);
		if (ret < 0)
			return ret;
		fan_config |= ret;
		ret = fan_config;
		break;
	default:
		ret = -ENODATA;
		break;
	}
	return ret;
}

static const struct i2c_device_id ucd9000_id[] = {
	{"ucd9000", ucd9000},
	{"ucd90120", ucd90120},
	{"ucd90124", ucd90124},
	{"ucd90160", ucd90160},
	{"ucd9090", ucd9090},
	{"ucd90910", ucd90910},
	{}
};
MODULE_DEVICE_TABLE(i2c, ucd9000_id);

static int ucd9000_gpio_read_config(struct i2c_client *client,
				unsigned int offset)
{
	int ret;

	/* No page set required */
	ret = i2c_smbus_write_byte_data(client, UCD9000_GPIO_SELECT, offset);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to select GPIO %d: %d\n", offset,
			ret);

		return ret;
	}

	return i2c_smbus_read_byte_data(client, UCD9000_GPIO_CONFIG);
}

static int ucd9000_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct i2c_client *client  = gpiochip_get_data(gc);
	int ret;

	ret = ucd9000_gpio_read_config(client, offset);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read GPIO %d config: %d\n",
			offset, ret);

		return ret;
	}

	return !!(ret & UCD9000_GPIO_CONFIG_STATUS);
}

static void ucd9000_gpio_set(struct gpio_chip *gc, unsigned int offset,
			int value)
{
	struct i2c_client *client = gpiochip_get_data(gc);
	int ret;

	ret = ucd9000_gpio_read_config(client, offset);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read GPIO %d config: %d\n",
			offset, ret);

		return;
	}

	if (value) {
		if (ret & UCD9000_GPIO_CONFIG_STATUS)
			return;

		ret |= UCD9000_GPIO_CONFIG_STATUS;
	} else {
		if (!(ret & UCD9000_GPIO_CONFIG_STATUS))
			return;

		ret &= ~UCD9000_GPIO_CONFIG_STATUS;
	}

	ret |= UCD9000_GPIO_CONFIG_ENABLE;

	/* Page set not required */
	ret = i2c_smbus_write_byte_data(client, UCD9000_GPIO_CONFIG, ret);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write GPIO %d config: %d\n",
			offset, ret);

		return;
	}

	ret &= ~UCD9000_GPIO_CONFIG_ENABLE;

	ret = i2c_smbus_write_byte_data(client, UCD9000_GPIO_CONFIG, ret);
	if (ret < 0)
		dev_err(&client->dev, "Failed to write GPIO %d config: %d\n",
			offset, ret);
}

static int ucd9000_gpio_get_direction(struct gpio_chip *gc,
					unsigned int offset)
{
	struct i2c_client *client = gpiochip_get_data(gc);
	int ret;

	ret = ucd9000_gpio_read_config(client, offset);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read GPIO %d config: %d\n",
			offset, ret);

		return ret;
	}

	return !(ret & UCD9000_GPIO_CONFIG_OUT_ENABLE);
}

static int ucd9000_gpio_set_direction(struct gpio_chip *gc, unsigned int offset,
					bool direction_out, int requested_out)
{
	struct i2c_client *client = gpiochip_get_data(gc);
	int ret, config, out_val;


	ret = ucd9000_gpio_read_config(client, offset);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read GPIO %d config: %d\n",
			offset, ret);

		return ret;
	}

	if (direction_out) {
		out_val = requested_out ? UCD9000_GPIO_CONFIG_OUT_VALUE : 0;

		if (ret & UCD9000_GPIO_CONFIG_OUT_ENABLE) {
			if ((ret & UCD9000_GPIO_CONFIG_OUT_VALUE) == out_val)
				return 0;
		} else
			ret |= UCD9000_GPIO_CONFIG_OUT_ENABLE;

		if (out_val)
			ret |= UCD9000_GPIO_CONFIG_OUT_VALUE;
		else
			ret &= ~UCD9000_GPIO_CONFIG_OUT_VALUE;

	} else {
		if (!(ret & UCD9000_GPIO_CONFIG_OUT_ENABLE))
			return 0;

		ret &= ~UCD9000_GPIO_CONFIG_OUT_ENABLE;
	}

	ret |= UCD9000_GPIO_CONFIG_ENABLE;
	config = ret;

	/* Page set not required */
	ret = i2c_smbus_write_byte_data(client, UCD9000_GPIO_CONFIG, config);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to write GPIO %d config: %d\n",
			offset, ret);

		return ret;
	}

	config &= ~UCD9000_GPIO_CONFIG_ENABLE;

	return i2c_smbus_write_byte_data(client, UCD9000_GPIO_CONFIG, config);
}

static int ucd9000_gpio_direction_input(struct gpio_chip *gc,
					unsigned int offset)
{
	return ucd9000_gpio_set_direction(gc, offset, UCD9000_GPIO_INPUT, 0);
}

static int ucd9000_gpio_direction_output(struct gpio_chip *gc,
					unsigned int offset, int val)
{
	return ucd9000_gpio_set_direction(gc, offset, UCD9000_GPIO_OUTPUT, val);
}

#if IS_ENABLED(CONFIG_DEBUG_FS)
static int ucd9000_get_mfr_status(struct i2c_client *client, u32 *buffer)
{
	int ret;

	ret = pmbus_set_page(client, 0);
	if (ret < 0) {
		dev_err(&client->dev, "pmbus_set_page failed. rc:%d\n", ret);

		return ret;
	}

	/*
	 * Warning:
	 *
	 * Though not currently supported this will cause stack corruption for
	 * ucd90240!  Command reference, page 81:
	 *
	 *    With the ucd90120 and ucd90124 devices, this command [MFR_STATUS]
	 *    is 2 bytes long (bits 0-15).  With the ucd90240 this command is 5
	 *    bytes long.  With all other devices, it is 4 bytes long.
	 */
	return i2c_smbus_read_block_data(client, UCD9000_MFR_STATUS,
					(u8 *)buffer);
}

static int ucd9000_debugfs_get_mfr_status_bit(void *data, u64 *val)
{
	struct ucd9000_debugfs_entry *entry = data;
	struct i2c_client *client = entry->client;
	int nr = entry->index;
	u32 buffer;
	int ret;

	ret = ucd9000_get_mfr_status(client, &buffer);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read mfr status. rc:%d\n",
			ret);

		return ret;
	}

	*val = !!(ret & BIT(nr));

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(ucd9000_debugfs_ops_mfr_status_bit,
		ucd9000_debugfs_get_mfr_status_bit, NULL, "%1lld\n");

static int ucd9000_debugfs_get_mfr_status_word(void *data, u64 *val)
{
	struct ucd9000_debugfs_entry *entry = data;
	struct i2c_client *client = entry->client;
	u32 buffer;
	int ret;

	ret = ucd9000_get_mfr_status(client, &buffer);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read mfr status. rc:%d\n",
			ret);

		return ret;
	}

	*val = ret;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ucd9000_debugfs_ops_mfr_status_word,
		ucd9000_debugfs_get_mfr_status_word, NULL, "%08llx\n");

static int ucd9000_init_debugfs(struct i2c_client *client,
				struct ucd9000_data *data)
{
	struct ucd9000_debugfs_entry *entries;
	char name[UCD9000_NAME_SIZE];
	int i;

	data->idx = ida_simple_get(&ucd9000_ida, 0, INT_MAX, GFP_KERNEL);
	scnprintf(name, UCD9000_NAME_SIZE, "ucd9000.%d", data->idx);
	data->debugfs = debugfs_create_dir(name, NULL);
	if (IS_ERR(data->debugfs)) {
		dev_warn(&client->dev, "Failed to create debugfs dir: %p\n",
			data->debugfs);
		data->debugfs = NULL;
		ida_simple_remove(&ucd9000_ida, data->idx);
		return 0;
	}

	/*
	 * Warning:
	 *
	 * Makes assumption we're on a ucd90160 type! entries will be different
	 * sizes for other types.
	 */
	entries = devm_kzalloc(&client->dev, sizeof(*entries) *
				(UCD90160_GPI_COUNT + 1) * 10, GFP_KERNEL);
	if (!entries) {
		ida_simple_remove(&ucd9000_ida, data->idx);
		return -ENOMEM;
	}

	/*
	 * Warning:
	 *
	 * This makes the assumption we're probing a ucd90160 type and how the
	 * GPI information is organized.  Needs to account for all other
	 * ucd9000 varieties.
	 */
	for (i = 0; i < UCD90160_GPI_COUNT; i++) {
		entries[i].client = client;
		entries[i].index = UCD90160_GPI_FAULT_BASE + i;
		scnprintf(name, UCD9000_NAME_SIZE, "gpi%d_alarm", i+1);
		debugfs_create_file(name, 0444, data->debugfs, &entries[i],
				&ucd9000_debugfs_ops_mfr_status_bit);
	}
	entries[i].client = client;
	scnprintf(name, UCD9000_NAME_SIZE, "mfr_status");
	debugfs_create_file(name, 0444, data->debugfs, &entries[i],
			&ucd9000_debugfs_ops_mfr_status_word);

	return 0;
}
#else
static int ucd9000_init_debugfs(struct i2c_client *client,
				struct ucd9000_data *data)
{
	return 0;
}
#endif /* IS_ENABLED(CONFIG_DEBUG_FS) */

static int ucd9000_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	u8 block_buffer[I2C_SMBUS_BLOCK_MAX + 1];
	struct ucd9000_data *data;
	struct pmbus_driver_info *info;
	const struct i2c_device_id *mid;
	int i, ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_BLOCK_DATA))
		return -ENODEV;

	ret = i2c_smbus_read_block_data(client, UCD9000_DEVICE_ID,
					block_buffer);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read device ID\n");
		return ret;
	}
	block_buffer[ret] = '\0';
	dev_info(&client->dev, "Device ID %s\n", block_buffer);

	for (mid = ucd9000_id; mid->name[0]; mid++) {
		if (!strncasecmp(mid->name, block_buffer, strlen(mid->name)))
			break;
	}
	if (!mid->name[0]) {
		dev_err(&client->dev, "Unsupported device\n");
		return -ENODEV;
	}

	if (id->driver_data != ucd9000 && id->driver_data != mid->driver_data)
		dev_notice(&client->dev,
			   "Device mismatch: Configured %s, detected %s\n",
			   id->name, mid->name);

	data = devm_kzalloc(&client->dev, sizeof(struct ucd9000_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	info = &data->info;

	ret = i2c_smbus_read_byte_data(client, UCD9000_NUM_PAGES);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed to read number of active pages\n");
		return ret;
	}
	info->pages = ret;
	if (!info->pages) {
		dev_err(&client->dev, "No pages configured\n");
		return -ENODEV;
	}

	/* The internal temperature sensor is always active */
	info->func[0] = PMBUS_HAVE_TEMP;

	/* Everything else is configurable */
	ret = i2c_smbus_read_block_data(client, UCD9000_MONITOR_CONFIG,
					block_buffer);
	if (ret <= 0) {
		dev_err(&client->dev, "Failed to read configuration data\n");
		return -ENODEV;
	}
	for (i = 0; i < ret; i++) {
		int page = UCD9000_MON_PAGE(block_buffer[i]);

		if (page >= info->pages)
			continue;

		switch (UCD9000_MON_TYPE(block_buffer[i])) {
		case UCD9000_MON_VOLTAGE:
		case UCD9000_MON_VOLTAGE_HW:
			info->func[page] |= PMBUS_HAVE_VOUT
			  | PMBUS_HAVE_STATUS_VOUT;
			break;
		case UCD9000_MON_TEMPERATURE:
			info->func[page] |= PMBUS_HAVE_TEMP2
			  | PMBUS_HAVE_STATUS_TEMP;
			break;
		case UCD9000_MON_CURRENT:
			info->func[page] |= PMBUS_HAVE_IOUT
			  | PMBUS_HAVE_STATUS_IOUT;
			break;
		default:
			break;
		}
	}

	/* Fan configuration */
	if (mid->driver_data == ucd90124) {
		for (i = 0; i < UCD9000_NUM_FAN; i++) {
			i2c_smbus_write_byte_data(client,
						  UCD9000_FAN_CONFIG_INDEX, i);
			ret = i2c_smbus_read_block_data(client,
							UCD9000_FAN_CONFIG,
							data->fan_data[i]);
			if (ret < 0)
				return ret;
		}
		i2c_smbus_write_byte_data(client, UCD9000_FAN_CONFIG_INDEX, 0);

		info->read_byte_data = ucd9000_read_byte_data;
		info->func[0] |= PMBUS_HAVE_FAN12 | PMBUS_HAVE_STATUS_FAN12
		  | PMBUS_HAVE_FAN34 | PMBUS_HAVE_STATUS_FAN34;
	}

	/*
	 * Note:
	 *
	 * Pinmux support has not been added to the new gpio_chip.
	 * This support should be added when possible given the mux
	 * behavior of these IO devices.
	 */
	data->gpio.label = (const char *)&client->name;
	data->gpio.get_direction = ucd9000_gpio_get_direction;
	data->gpio.direction_input = ucd9000_gpio_direction_input;
	data->gpio.direction_output = ucd9000_gpio_direction_output;
	data->gpio.get = ucd9000_gpio_get;
	data->gpio.set = ucd9000_gpio_set;
	data->gpio.can_sleep = 1;
	data->gpio.base = -1;

	/*
	 * TODO: set ngpio for ucd9000 devs that aren't 90160 type
	 */
	if (mid->driver_data == ucd90160)
		data->gpio.ngpio = UCD90160_NUM_GPIOS;
	data->gpio.parent = &client->dev;
	data->gpio.owner = THIS_MODULE;

	ret = devm_gpiochip_add_data(&client->dev, &data->gpio, client);
	if (ret) {
		data->gpio.parent = NULL;
		dev_warn(&client->dev, "Could not add gpiochip: %d\n", ret);
		return ret;
	}

	ret = ucd9000_init_debugfs(client, data);
	if (ret < 0)
		dev_warn(&client->dev, "Failed to register debugfs: %d\n", ret);

	return pmbus_do_probe(client, mid, info);
}

static int ucd9000_remove(struct i2c_client *client)
{
	struct ucd9000_data *data
		= to_ucd9000_data(pmbus_get_driver_info(client));

	ida_simple_remove(&ucd9000_ida, data->idx);
	debugfs_remove_recursive(data->debugfs);
	return pmbus_do_remove(client);
}

/* This is the driver that will be inserted */
static struct i2c_driver ucd9000_driver = {
	.driver = {
		.name = "ucd9000",
	},
	.probe = ucd9000_probe,
	.remove = ucd9000_remove,
	.id_table = ucd9000_id,
};

module_i2c_driver(ucd9000_driver);

MODULE_AUTHOR("Guenter Roeck");
MODULE_DESCRIPTION("PMBus driver for TI UCD90xxx");
MODULE_LICENSE("GPL");
