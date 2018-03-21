/*
 * Copyright 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/unaligned.h>
#include "common.h"

#define OCC_NUM_STATUS_ATTRS		9

#define OCC_STAT_MASTER			0x80
#define OCC_STAT_ACTIVE			0x01
#define OCC_EXT_STAT_DVFS_OT		0x80
#define OCC_EXT_STAT_DVFS_POWER		0x40
#define OCC_EXT_STAT_MEM_THROTTLE	0x20
#define OCC_EXT_STAT_QUICK_DROP		0x10

struct temp_sensor_1 {
	u16 sensor_id;
	u16 value;
} __packed;

struct temp_sensor_2 {
	u32 sensor_id;
	u8 fru_type;
	u8 value;
} __packed;

struct freq_sensor_1 {
	u16 sensor_id;
	u16 value;
} __packed;

struct freq_sensor_2 {
	u32 sensor_id;
	u16 value;
} __packed;

struct power_sensor_1 {
	u16 sensor_id;
	u32 update_tag;
	u32 accumulator;
	u16 value;
} __packed;

struct power_sensor_2 {
	u32 sensor_id;
	u8 function_id;
	u8 apss_channel;
	u16 reserved;
	u32 update_tag;
	u64 accumulator;
	u16 value;
} __packed;

struct power_sensor_data {
	u16 value;
	u32 update_tag;
	u64 accumulator;
} __packed;

struct power_sensor_data_and_time {
	u16 update_time;
	u16 value;
	u32 update_tag;
	u64 accumulator;
} __packed;

struct power_sensor_a0 {
	u32 sensor_id;
	struct power_sensor_data_and_time system;
	u32 reserved;
	struct power_sensor_data_and_time proc;
	struct power_sensor_data vdd;
	struct power_sensor_data vdn;
} __packed;

struct caps_sensor_1 {
	u16 curr_powercap;
	u16 curr_powerreading;
	u16 norm_powercap;
	u16 max_powercap;
	u16 min_powercap;
	u16 user_powerlimit;
} __packed;

struct caps_sensor_2 {
	u16 curr_powercap;
	u16 curr_powerreading;
	u16 norm_powercap;
	u16 max_powercap;
	u16 min_powercap;
	u16 user_powerlimit;
	u8 user_powerlimit_source;
} __packed;

struct caps_sensor_3 {
	u16 curr_powercap;
	u16 curr_powerreading;
	u16 norm_powercap;
	u16 max_powercap;
	u16 hard_min_powercap;
	u16 soft_min_powercap;
	u16 user_powerlimit;
	u8 user_powerlimit_source;
} __packed;

struct extended_sensor {
	u8 name[4];
	u8 flags;
	u8 reserved;
	u8 data[6];
} __packed;

void occ_parse_poll_response(struct occ *occ)
{
	unsigned int i, offset = 0, size = 0;
	struct occ_sensor *sensor;
	struct occ_sensors *sensors = &occ->sensors;
	struct occ_response *resp = &occ->resp;
	struct occ_poll_response *poll =
		(struct occ_poll_response *)&resp->data[0];
	struct occ_poll_response_header *header = &poll->header;
	struct occ_sensor_data_block *block = &poll->block;

	for (i = 0; i < header->num_sensor_data_blocks; ++i) {
		block = (struct occ_sensor_data_block *)((u8 *)block + offset);
		offset = (block->header.num_sensors *
			  block->header.sensor_length) + sizeof(block->header);
		size += offset;

		/* validate all the length/size fields */
		if ((size + sizeof(*header)) >= OCC_RESP_DATA_BYTES) {
			dev_warn(occ->bus_dev, "exceeded response buffer\n");
			return;
		}

		if (strncmp(block->header.eye_catcher, "TEMP", 4) == 0)
			sensor = &sensors->temp;
		else if (strncmp(block->header.eye_catcher, "FREQ", 4) == 0)
			sensor = &sensors->freq;
		else if (strncmp(block->header.eye_catcher, "POWR", 4) == 0)
			sensor = &sensors->power;
		else if (strncmp(block->header.eye_catcher, "CAPS", 4) == 0)
			sensor = &sensors->caps;
		else if (strncmp(block->header.eye_catcher, "EXTN", 4) == 0)
			sensor = &sensors->extended;
		else {
			dev_warn(occ->bus_dev, "sensor not supported %.4s\n",
				 block->header.eye_catcher);
			continue;
		}

		sensor->num_sensors = block->header.num_sensors;
		sensor->version = block->header.sensor_format;
		sensor->data = &block->data;
	}
}

void occ_set_error(struct occ *occ, int error)
{
	occ->error_count++;
	if (occ->error_count > OCC_ERROR_COUNT_THRESHOLD)
		occ->error = error;
}

void occ_reset_error(struct occ *occ)
{
	occ->error_count = 0;
	occ->error = 0;
}

int occ_poll(struct occ *occ)
{
	int rc, error = occ->error;
	struct occ_poll_response_header *header =
		(struct occ_poll_response_header *)occ->resp.data;
	u16 checksum = occ->poll_cmd_data + 1;
	u8 cmd[8];
	u8 occs_present = header->occs_present;
	u8 ext_status = header->ext_status;

	cmd[0] = 0;
	cmd[1] = 0;
	cmd[2] = 0;
	cmd[3] = 1;
	cmd[4] = occ->poll_cmd_data;
	cmd[5] = checksum >> 8;
	cmd[6] = checksum & 0xFF;
	cmd[7] = 0;

	rc = occ->send_cmd(occ, cmd);
	if (rc)
		goto done;

	if (header->occ_state == OCC_STATE_SAFE) {
		if (occ->last_safe) {
			if (time_after(jiffies,
				       occ->last_safe + OCC_SAFE_TIMEOUT))
				occ->error = -EHOSTDOWN;
		} else
			occ->last_safe = jiffies;
	} else
		occ->last_safe = 0;

	if (occ->hwmon) {
		if (occs_present != header->occs_present &&
		    (header->status & OCC_STAT_MASTER))
			sysfs_notify(&occ->bus_dev->kobj, NULL,
				     occ->status_attrs[7].dev_attr.attr.name);

		if ((header->ext_status & OCC_EXT_STAT_DVFS_OT) !=
		    (ext_status & OCC_EXT_STAT_DVFS_OT))
			sysfs_notify(&occ->bus_dev->kobj, NULL,
				     occ->status_attrs[2].dev_attr.attr.name);

		if ((header->ext_status & OCC_EXT_STAT_DVFS_POWER) !=
		    (ext_status & OCC_EXT_STAT_DVFS_POWER))
			sysfs_notify(&occ->bus_dev->kobj, NULL,
				     occ->status_attrs[3].dev_attr.attr.name);

		if ((header->ext_status & OCC_EXT_STAT_MEM_THROTTLE) !=
		    (ext_status & OCC_EXT_STAT_MEM_THROTTLE))
			sysfs_notify(&occ->bus_dev->kobj, NULL,
				     occ->status_attrs[4].dev_attr.attr.name);
	}

done:
	/* notify userspace if we change error state and have an error */
	if (occ->error != error && occ->error && occ->error_attr_name)
		sysfs_notify(&occ->bus_dev->kobj, NULL, occ->error_attr_name);

	return rc;
}

int occ_set_user_power_cap(struct occ *occ, u16 user_power_cap)
{
	int rc, error = occ->error;
	u8 cmd[8];
	u16 checksum = 0x24;
	__be16 user_power_cap_be;
	struct occ_poll_response_header *header =
		(struct occ_poll_response_header *)occ->resp.data;

	if (!(header->status & OCC_STAT_MASTER))
		return -EPERM;

	if (!(header->status & OCC_STAT_ACTIVE))
		return -EACCES;

	user_power_cap_be = cpu_to_be16(user_power_cap);

	cmd[0] = 0;
	cmd[1] = 0x22;
	cmd[2] = 0;
	cmd[3] = 2;

	memcpy(&cmd[4], &user_power_cap_be, 2);

	checksum += cmd[4] + cmd[5];
	cmd[6] = checksum >> 8;
	cmd[7] = checksum & 0xFF;

	mutex_lock(&occ->lock);
	rc = occ->send_cmd(occ, cmd);
	mutex_unlock(&occ->lock);

	/* notify userspace if we change error state and have an error*/
	if (occ->error != error && occ->error && occ->error_attr_name)
		sysfs_notify(&occ->bus_dev->kobj, NULL, occ->error_attr_name);

	return rc;
}

int occ_update_response(struct occ *occ)
{
	int rc = 0;

	mutex_lock(&occ->lock);

	if (time_after(jiffies, occ->last_update + OCC_UPDATE_FREQUENCY)) {
		rc = occ_poll(occ);
		occ->last_update = jiffies;
	}

	mutex_unlock(&occ->lock);
	return rc;
}

static ssize_t occ_show_error(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct occ *occ = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", occ->error);
}

static ssize_t occ_show_status(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	int val;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_poll_response_header *header;
	struct sensor_device_attribute *sattr = to_sensor_dev_attr(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	header = (struct occ_poll_response_header *)occ->resp.data;

	switch (sattr->index) {
	case 0:
		val = (header->status & OCC_STAT_MASTER) ? 1 : 0;
		break;
	case 1:
		val = (header->status & OCC_STAT_ACTIVE) ? 1 : 0;
		break;
	case 2:
		val = (header->ext_status & OCC_EXT_STAT_DVFS_OT) ? 1 : 0;
		break;
	case 3:
		val = (header->ext_status & OCC_EXT_STAT_DVFS_POWER) ? 1 : 0;
		break;
	case 4:
		val = (header->ext_status & OCC_EXT_STAT_MEM_THROTTLE) ? 1 : 0;
		break;
	case 5:
		val = (header->ext_status & OCC_EXT_STAT_QUICK_DROP) ? 1 : 0;
		break;
	case 6:
		val = header->occ_state;
		break;
	case 7:
		if (header->status & OCC_STAT_MASTER)
			val = hweight8(header->occs_present);
		else
			val = 1;
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", val);
}

static ssize_t occ_show_temp_1(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u32 val = 0;
	struct temp_sensor_1 *temp;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	temp = ((struct temp_sensor_1 *)sensors->temp.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&temp->sensor_id);
		break;
	case 1:
		val = get_unaligned_be16(&temp->value) * 1000;
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_temp_2(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u32 val = 0;
	struct temp_sensor_2 *temp;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	temp = ((struct temp_sensor_2 *)sensors->temp.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be32(&temp->sensor_id);
		break;
	case 1:
		val = temp->value * 1000;
		break;
	case 2:
		val = temp->fru_type;
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_freq_1(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u16 val = 0;
	struct freq_sensor_1 *freq;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	freq = ((struct freq_sensor_1 *)sensors->freq.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&freq->sensor_id);
		break;
	case 1:
		val = get_unaligned_be16(&freq->value);
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_freq_2(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u32 val = 0;
	struct freq_sensor_2 *freq;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	freq = ((struct freq_sensor_2 *)sensors->freq.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be32(&freq->sensor_id);
		break;
	case 1:
		val = get_unaligned_be16(&freq->value);
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_power_1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int rc;
	u64 val = 0;
	struct power_sensor_1 *power;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	power = ((struct power_sensor_1 *)sensors->power.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&power->sensor_id);
		break;
	case 1:
		val = get_unaligned_be32(&power->update_tag);
		break;
	case 2:
		val = get_unaligned_be32(&power->accumulator);
		break;
	case 3:
		val = get_unaligned_be16(&power->value) * 1000000ULL;
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%llu\n", val);
}

static ssize_t occ_show_power_2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int rc;
	u64 val = 0;
	struct power_sensor_2 *power;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	power = ((struct power_sensor_2 *)sensors->power.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be32(&power->sensor_id);
		break;
	case 1:
		val = get_unaligned_be32(&power->update_tag);
		break;
	case 2:
		val = get_unaligned_be64(&power->accumulator);
		break;
	case 3:
		val = get_unaligned_be16(&power->value) * 1000000ULL;
		break;
	case 4:
		val = power->function_id;
		break;
	case 5:
		val = power->apss_channel;
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%llu\n", val);
}

static ssize_t occ_show_power_a0(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int rc;
	u64 val = 0;
	struct power_sensor_a0 *power;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	power = ((struct power_sensor_a0 *)sensors->power.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be32(&power->sensor_id);
		break;
	case 1:
		val = get_unaligned_be16(&power->system.update_time);
		break;
	case 2:
		val = get_unaligned_be16(&power->system.value) * 1000000ULL;
		break;
	case 3:
		val = get_unaligned_be32(&power->system.update_tag);
		break;
	case 4:
		val = get_unaligned_be64(&power->system.accumulator);
		break;
	case 5:
		val = get_unaligned_be16(&power->proc.update_time);
		break;
	case 6:
		val = get_unaligned_be16(&power->proc.value) * 1000000ULL;
		break;
	case 7:
		val = get_unaligned_be32(&power->proc.update_tag);
		break;
	case 8:
		val = get_unaligned_be64(&power->proc.accumulator);
		break;
	case 9:
		val = get_unaligned_be16(&power->vdd.value) * 1000000ULL;
		break;
	case 10:
		val = get_unaligned_be32(&power->vdd.update_tag);
		break;
	case 11:
		val = get_unaligned_be64(&power->vdd.accumulator);
		break;
	case 12:
		val = get_unaligned_be16(&power->vdn.value) * 1000000ULL;
		break;
	case 13:
		val = get_unaligned_be32(&power->vdn.update_tag);
		break;
	case 14:
		val = get_unaligned_be64(&power->vdn.accumulator);
		break;
	case 15:
		return snprintf(buf, PAGE_SIZE - 1, "system\n");
	case 16:
		return snprintf(buf, PAGE_SIZE - 1, "proc\n");
	case 17:
		return snprintf(buf, PAGE_SIZE - 1, "vdd\n");
	case 18:
		return snprintf(buf, PAGE_SIZE - 1, "vdn\n");
	}

	return snprintf(buf, PAGE_SIZE - 1, "%llu\n", val);
}


static ssize_t occ_show_caps_1(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u16 val = 0;
	struct caps_sensor_1 *caps;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	caps = ((struct caps_sensor_1 *)sensors->caps.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&caps->curr_powercap);
		break;
	case 1:
		val = get_unaligned_be16(&caps->curr_powerreading);
		break;
	case 2:
		val = get_unaligned_be16(&caps->norm_powercap);
		break;
	case 3:
		val = get_unaligned_be16(&caps->max_powercap);
		break;
	case 4:
		val = get_unaligned_be16(&caps->min_powercap);
		break;
	case 5:
		val = get_unaligned_be16(&caps->user_powerlimit);
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_caps_2(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u16 val = 0;
	struct caps_sensor_2 *caps;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	caps = ((struct caps_sensor_2 *)sensors->caps.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&caps->curr_powercap);
		break;
	case 1:
		val = get_unaligned_be16(&caps->curr_powerreading);
		break;
	case 2:
		val = get_unaligned_be16(&caps->norm_powercap);
		break;
	case 3:
		val = get_unaligned_be16(&caps->max_powercap);
		break;
	case 4:
		val = get_unaligned_be16(&caps->min_powercap);
		break;
	case 5:
		val = get_unaligned_be16(&caps->user_powerlimit);
		break;
	case 6:
		val = caps->user_powerlimit_source;
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_caps_3(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u16 val = 0;
	struct caps_sensor_3 *caps;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	caps = ((struct caps_sensor_3 *)sensors->caps.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&caps->curr_powercap);
		break;
	case 1:
		val = get_unaligned_be16(&caps->curr_powerreading);
		break;
	case 2:
		val = get_unaligned_be16(&caps->norm_powercap);
		break;
	case 3:
		val = get_unaligned_be16(&caps->max_powercap);
		break;
	case 4:
		val = get_unaligned_be16(&caps->hard_min_powercap);
		break;
	case 5:
		val = get_unaligned_be16(&caps->user_powerlimit);
		break;
	case 6:
		val = caps->user_powerlimit_source;
		break;
	case 7:
		val = get_unaligned_be16(&caps->soft_min_powercap);
		break;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}


static ssize_t occ_store_caps_user(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int rc;
	u16 user_power_cap;
	struct occ *occ = dev_get_drvdata(dev);

	rc = kstrtou16(buf, 0, &user_power_cap);
	if (rc)
		return rc;

	rc = occ_set_user_power_cap(occ, user_power_cap);
	if (rc)
		return rc;

	return count;
}

static ssize_t occ_show_extended(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int rc;
	struct extended_sensor *extn;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	extn = ((struct extended_sensor *)sensors->extended.data) +
		sattr->index;

	switch (sattr->nr) {
	case 0:
		rc = snprintf(buf, PAGE_SIZE - 1, "%02x%02x%02x%02x\n",
			      extn->name[0], extn->name[1], extn->name[2],
			      extn->name[3]);
		break;
	case 1:
		rc = snprintf(buf, PAGE_SIZE - 1, "%02x\n", extn->flags);
		break;
	case 2:
		rc = snprintf(buf, PAGE_SIZE - 1, "%02x%02x%02x%02x%02x%02x\n",
			      extn->data[0], extn->data[1], extn->data[2],
			      extn->data[3], extn->data[4], extn->data[5]);
		break;
	}

	return rc;
}

int occ_setup_sensor_attrs(struct occ *occ)
{
	unsigned int i, s;
	struct device *dev = occ->bus_dev;
	struct occ_sensors *sensors = &occ->sensors;
	struct occ_attribute *attr;
	ssize_t (*show_temp)(struct device *, struct device_attribute *,
			     char *) = occ_show_temp_1;
	ssize_t (*show_freq)(struct device *, struct device_attribute *,
			     char *) = occ_show_freq_1;
	ssize_t (*show_power)(struct device *, struct device_attribute *,
			      char *) = occ_show_power_1;
	ssize_t (*show_caps)(struct device *, struct device_attribute *,
			     char *) = occ_show_caps_1;

	occ->num_attrs = 0;

	switch (sensors->temp.version) {
	case 1:
		occ->num_attrs += (sensors->temp.num_sensors * 2);
		break;
	case 2:
		occ->num_attrs += (sensors->temp.num_sensors * 3);
		show_temp = occ_show_temp_2;
		break;
	default:
		sensors->temp.num_sensors = 0;
	}

	switch (sensors->freq.version) {
	case 2:
		show_freq = occ_show_freq_2;
		/* fall through */
	case 1:
		occ->num_attrs += (sensors->freq.num_sensors * 2);
		break;
	default:
		sensors->freq.num_sensors = 0;
	}

	switch (sensors->power.version) {
	case 1:
		occ->num_attrs += (sensors->power.num_sensors * 4);
		break;
	case 2:
		occ->num_attrs += (sensors->power.num_sensors * 6);
		show_power = occ_show_power_2;
		break;
	case 0xA0:
		occ->num_attrs += (sensors->power.num_sensors * 19);
		show_power = occ_show_power_a0;
		break;
	default:
		sensors->power.num_sensors = 0;
	}

	switch (sensors->caps.version) {
	case 1:
		occ->num_attrs += (sensors->caps.num_sensors * 6);
		break;
	case 2:
		occ->num_attrs += (sensors->caps.num_sensors * 7);
		show_caps = occ_show_caps_2;
		break;
	case 3:
		occ->num_attrs += (sensors->caps.num_sensors * 8);
		show_caps = occ_show_caps_3;
		break;
	default:
		sensors->caps.num_sensors = 0;
	}

	switch (sensors->extended.version) {
	case 1:
		occ->num_attrs += (sensors->extended.num_sensors * 3);
		break;
	default:
		sensors->extended.num_sensors = 0;
	}

	occ->attrs = devm_kzalloc(dev, sizeof(*occ->attrs) * occ->num_attrs,
				  GFP_KERNEL);
	if (!occ->attrs)
		return -ENOMEM;

	occ->group.attrs = devm_kzalloc(dev, sizeof(*occ->group.attrs) *
					occ->num_attrs + 1, GFP_KERNEL);
	if (!occ->group.attrs)
		return -ENOMEM;

	attr = occ->attrs;

	for (i = 0; i < sensors->temp.num_sensors; ++i) {
		s = i + 1;

		snprintf(attr->name, sizeof(attr->name), "temp%d_label", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444, show_temp, NULL,
					     0, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "temp%d_input", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444, show_temp, NULL,
					     1, i);
		attr++;

		if (sensors->temp.version > 1) {
			snprintf(attr->name, sizeof(attr->name),
				 "temp%d_fru_type", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_temp, NULL, 2, i);
			attr++;
		}
	}

	for (i = 0; i < sensors->freq.num_sensors; ++i) {
		s = i + 1;

		snprintf(attr->name, sizeof(attr->name), "freq%d_label", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444, show_freq, NULL,
					     0, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "freq%d_input", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444, show_freq, NULL,
					     1, i);
		attr++;
	}

	if (sensors->power.version == 0xA0) {
		for (i = 0; i < sensors->power.num_sensors; ++i) {
			s = i + 1;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_id", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 0, i);
			attr++;

			/* system power attributes */
			snprintf(attr->name, sizeof(attr->name),
				 "power%d_label", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 15, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_update_time", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 1, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_input", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 2, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_update_tag", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 3, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_accumulator", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 4, i);
			attr++;

			s++;

			/* proc power attributes */
			snprintf(attr->name, sizeof(attr->name),
				 "power%d_label", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 16, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_update_time", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 5, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_input", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 6, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_update_tag", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 7, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_accumulator", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 8, i);
			attr++;

			s++;

			/* vdd power attributes */
			snprintf(attr->name, sizeof(attr->name),
				 "power%d_label", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 17, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_input", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 9, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_update_tag", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 10, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_accumulator", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 11, i);
			attr++;

			s++;

			/* vdn power attributes */
			snprintf(attr->name, sizeof(attr->name),
				 "power%d_label", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 18, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_input", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 12, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_update_tag", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 13, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_accumulator", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 14, i);
			attr++;
		}
	} else {
		for (i = 0; i < sensors->power.num_sensors; ++i) {
			s = i + 1;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_label", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 0, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_update_tag", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 1, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_accumulator", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 2, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "power%d_input", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_power, NULL, 3, i);
			attr++;

			if (sensors->power.version > 1) {
				snprintf(attr->name, sizeof(attr->name),
					 "power%d_function_id", s);
				attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
							     show_power, NULL,
							     4, i);
				attr++;

				snprintf(attr->name, sizeof(attr->name),
					 "power%d_apss_channel", s);
				attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
							     show_power, NULL,
							     5, i);
				attr++;
			}
		}
	}

	for (i = 0; i < sensors->caps.num_sensors; ++i) {
		s = i + 1;

		snprintf(attr->name, sizeof(attr->name), "caps%d_current", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444, show_caps, NULL,
					     0, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "caps%d_reading", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444, show_caps, NULL,
					     1, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "caps%d_norm", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444, show_caps, NULL,
					     2, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "caps%d_max", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444, show_caps, NULL,
					     3, i);
		attr++;

		if (sensors->caps.version > 2) {
			snprintf(attr->name, sizeof(attr->name),
				 "caps%d_min_hard", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_caps, NULL, 4, i);
			attr++;

			snprintf(attr->name, sizeof(attr->name),
				 "caps%d_min_soft", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_caps, NULL, 7, i);
			attr++;
		} else {
			snprintf(attr->name, sizeof(attr->name), "caps%d_min",
				 s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_caps, NULL, 4, i);
			attr++;
		}

		snprintf(attr->name, sizeof(attr->name), "caps%d_user", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0644, show_caps,
					     occ_store_caps_user, 5, i);
		attr++;

		if (sensors->caps.version > 1) {
			snprintf(attr->name, sizeof(attr->name),
				 "caps%d_user_source", s);
			attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
						     show_caps, NULL, 6, i);
			attr++;
		}
	}

	for (i = 0; i < sensors->extended.num_sensors; ++i) {
		s = i + 1;

		snprintf(attr->name, sizeof(attr->name), "extn%d_label", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     occ_show_extended, NULL, 0, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "extn%d_flags", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     occ_show_extended, NULL, 1, i);
		attr++;

		snprintf(attr->name, sizeof(attr->name), "extn%d_value", s);
		attr->sensor = OCC_INIT_ATTR(attr->name, 0444,
					     occ_show_extended, NULL, 2, i);
		attr++;
	}

	/* put the sensors in the group */
	for (i = 0; i < occ->num_attrs; ++i)
		occ->group.attrs[i] = &occ->attrs[i].sensor.dev_attr.attr;

	return 0;
}

int occ_create_status_attrs(struct occ *occ)
{
	int rc, i;
	struct device *dev = occ->bus_dev;

	occ->status_attrs = devm_kzalloc(dev, sizeof(*occ->status_attrs) *
					 OCC_NUM_STATUS_ATTRS, GFP_KERNEL);
	if (!occ->status_attrs)
		return -ENOMEM;

	occ->status_attrs[0] =
		(struct sensor_device_attribute)SENSOR_ATTR(occ_master, 0444,
							    occ_show_status,
							    NULL, 0);
	occ->status_attrs[1] =
		(struct sensor_device_attribute)SENSOR_ATTR(occ_active, 0444,
							    occ_show_status,
							    NULL, 1);
	occ->status_attrs[2] =
		(struct sensor_device_attribute)SENSOR_ATTR(occ_dvfs_ot, 0444,
							    occ_show_status,
							    NULL, 2);
	occ->status_attrs[3] =
		(struct sensor_device_attribute)SENSOR_ATTR(occ_dvfs_power,
							    0444,
							    occ_show_status,
							    NULL, 3);
	occ->status_attrs[4] =
		(struct sensor_device_attribute)SENSOR_ATTR(occ_mem_throttle,
							    0444,
							    occ_show_status,
							    NULL, 4);
	occ->status_attrs[5] =
		(struct sensor_device_attribute)SENSOR_ATTR(occ_quick_drop,
							    0444,
							    occ_show_status,
							    NULL, 5);
	occ->status_attrs[6] =
		(struct sensor_device_attribute)SENSOR_ATTR(occ_status, 0444,
							    occ_show_status,
							    NULL, 6);
	occ->status_attrs[7] =
		(struct sensor_device_attribute)SENSOR_ATTR(occs_present, 0444,
							    occ_show_status,
							    NULL, 7);

	occ->status_attrs[8] =
		(struct sensor_device_attribute)SENSOR_ATTR(occ_error, 0444,
							    occ_show_error,
							    NULL, 0);
	occ->error_attr_name = occ->status_attrs[8].dev_attr.attr.name;

	for (i = 0; i < OCC_NUM_STATUS_ATTRS; ++i) {
		rc = device_create_file(dev, &occ->status_attrs[i].dev_attr);
		if (rc)
			dev_warn(dev, "error %d creating status attr %d\n", rc,
				 i);
	}

	return 0;
}

void occ_remove_status_attrs(struct occ *occ)
{
	int i;

	for (i = 0; i < OCC_NUM_STATUS_ATTRS; ++i)
		device_remove_file(occ->bus_dev,
				   &occ->status_attrs[i].dev_attr);
}
