/*
 * Copyright 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "common.h"
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/occ.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

struct p9_sbe_occ {
	struct occ occ;
	struct device *sbe;

	/*
	 * Pointer to occ device client. We store this so that we can cancel
	 * the client operations in remove() if necessary. We only need one
	 * pointer since we do one OCC operation (open, write, read, close) at
	 * a time (access to p9_sbe_occ_send_cmd is locked in the common code
	 * with occ.lock).
	 */
	struct occ_client *client;

	/*
	 * This lock controls access to the client pointer and ensures atomic
	 * open, close and NULL assignment. This prevents simultaneous opening
	 * and closing of the client, or closing multiple times.
	 */
	spinlock_t lock;
};

#define to_p9_sbe_occ(x)	container_of((x), struct p9_sbe_occ, occ)

static void p9_sbe_occ_close_client(struct p9_sbe_occ *occ)
{
	struct occ_client *tmp_client;

	spin_lock_irq(&occ->lock);
	tmp_client = occ->client;
	occ->client = NULL;
	occ_drv_release(tmp_client);
	spin_unlock_irq(&occ->lock);
}

static int p9_sbe_occ_send_cmd(struct occ *occ, u8 *cmd)
{
	int rc, error;
	struct occ_response *resp = &occ->resp;
	struct p9_sbe_occ *p9_sbe_occ = to_p9_sbe_occ(occ);

	spin_lock_irq(&p9_sbe_occ->lock);
	if (p9_sbe_occ->sbe)
		p9_sbe_occ->client = occ_drv_open(p9_sbe_occ->sbe, 0);
	spin_unlock_irq(&p9_sbe_occ->lock);

	if (!p9_sbe_occ->client) {
		rc = -ENODEV;
		goto assign;
	}

	rc = occ_drv_write(p9_sbe_occ->client, (const char *)&cmd[1], 7);
	if (rc < 0)
		goto err;

	rc = occ_drv_read(p9_sbe_occ->client, (char *)resp, sizeof(*resp));
	if (rc < 0)
		goto err;

	p9_sbe_occ_close_client(p9_sbe_occ);

	switch (resp->return_status) {
	case RESP_RETURN_CMD_IN_PRG:
		rc = -ETIMEDOUT;
		break;
	case RESP_RETURN_SUCCESS:
		occ_reset_error(occ);
		return 0;
		break;
	case RESP_RETURN_CMD_INVAL:
	case RESP_RETURN_CMD_LEN:
	case RESP_RETURN_DATA_INVAL:
	case RESP_RETURN_CHKSUM:
		rc = -EINVAL;
		break;
	case RESP_RETURN_OCC_ERR:
		rc = -EREMOTE;
		break;
	default:
		rc = -EFAULT;
	}

	error = resp->return_status;
	dev_warn(occ->bus_dev, "occ bad response:%d\n", error);
	goto done;

err:
	p9_sbe_occ_close_client(p9_sbe_occ);
	dev_err(occ->bus_dev, "occ bus op failed rc:%d\n", rc);
assign:
	error = rc;
done:
	occ_set_error(occ, error);
	return rc;
}

static int p9_sbe_occ_setup(struct p9_sbe_occ *p9_sbe_occ)
{
	int rc;
	struct occ *occ = &p9_sbe_occ->occ;

	/* no need to lock */
	rc = occ_poll(occ);
	if (rc < 0) {
		dev_err(occ->bus_dev, "failed to get OCC poll response: %d\n",
			rc);
		return rc;
	}

	occ_parse_poll_response(occ);

	rc = occ_setup_sensor_attrs(occ);
	if (rc) {
		dev_err(occ->bus_dev, "failed to setup p9 attrs: %d\n", rc);
		return rc;
	}

	occ->hwmon = devm_hwmon_device_register_with_groups(occ->bus_dev,
							    "p9_occ", occ,
							    occ->groups);
	if (IS_ERR(occ->hwmon)) {
		rc = PTR_ERR(occ->hwmon);
		dev_err(occ->bus_dev, "failed to register hwmon device: %d\n",
			rc);
		return rc;
	}

	rc = occ_create_status_attrs(occ);
	if (rc)
		dev_err(occ->bus_dev, "failed to setup p9 status attrs: %d\n", rc);

	return rc;
}

static int p9_sbe_occ_probe(struct platform_device *pdev)
{
	int rc;
	struct occ *occ;
	struct p9_sbe_occ *p9_sbe_occ = devm_kzalloc(&pdev->dev,
						     sizeof(*p9_sbe_occ),
						     GFP_KERNEL);
	if (!p9_sbe_occ)
		return -ENOMEM;

	p9_sbe_occ->sbe = pdev->dev.parent;

	occ = &p9_sbe_occ->occ;
	spin_lock_init(&p9_sbe_occ->lock);
	occ->bus_dev = &pdev->dev;
	occ->groups[0] = &occ->group;
	occ->poll_cmd_data = 0x20;
	occ->send_cmd = p9_sbe_occ_send_cmd;
	mutex_init(&occ->lock);

	platform_set_drvdata(pdev, occ);

	rc = p9_sbe_occ_setup(p9_sbe_occ);
	if (rc)
		return rc;

	return rc;
}

static int p9_sbe_occ_remove(struct platform_device *pdev)
{
	struct occ *occ = platform_get_drvdata(pdev);
	struct p9_sbe_occ *p9_sbe_occ = to_p9_sbe_occ(occ);

	p9_sbe_occ->sbe = NULL;
	p9_sbe_occ_close_client(p9_sbe_occ);
	occ_remove_status_attrs(occ);

	return 0;
}

static const struct of_device_id p9_sbe_occ_of_match[] = {
	{ .compatible = "ibm,p9-occ-hwmon" },
	{ },
};

static struct platform_driver p9_sbe_occ_driver = {
	.driver = {
		.name = "occ-hwmon",
		.of_match_table	= p9_sbe_occ_of_match,
	},
	.probe	= p9_sbe_occ_probe,
	.remove = p9_sbe_occ_remove,
};

module_platform_driver(p9_sbe_occ_driver);

MODULE_AUTHOR("Eddie James <eajames@us.ibm.com>");
MODULE_DESCRIPTION("BMC P9 OCC hwmon driver");
MODULE_LICENSE("GPL");
