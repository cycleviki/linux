/*
 * Copyright (C) IBM Corporation 2017
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERGCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/fsi.h>
#include <linux/fsi-sbefifo.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/*
 * The SBEFIFO is a pipe-like FSI device for communicating with
 * the self boot engine on POWER processors.
 */

#define DEVICE_NAME		"sbefifo"
#define FSI_ENGID_SBE		0x22
#define SBEFIFO_BUF_CNT		32

#define SBEFIFO_UP		0x00	/* Up register offset */
#define SBEFIFO_DWN		0x40	/* Down register offset */

#define SBEFIFO_STS		0x04
#define   SBEFIFO_EMPTY			BIT(20)
#define   SBEFIFO_STS_RESET_REQ		BIT(25)
#define SBEFIFO_EOT_RAISE	0x08
#define   SBEFIFO_EOT_MAGIC		0xffffffff
#define SBEFIFO_REQ_RESET	0x0C
#define SBEFIFO_EOT_ACK		0x14

#define SBEFIFO_RESCHEDULE	msecs_to_jiffies(500)
#define SBEFIFO_MAX_RESCHDULE	msecs_to_jiffies(5000)

struct sbefifo {
	struct timer_list poll_timer;
	struct fsi_device *fsi_dev;
	struct miscdevice mdev;
	wait_queue_head_t wait;
	struct list_head xfrs;
	struct kref kref;
	spinlock_t lock;
	char name[32];
	int idx;
	int rc;
};

struct sbefifo_buf {
	u32 buf[SBEFIFO_BUF_CNT];
	unsigned long flags;
#define SBEFIFO_BUF_FULL		1
	u32 *rpos;
	u32 *wpos;
};

struct sbefifo_xfr {
	unsigned long wait_data_timeout;
	struct sbefifo_buf *rbuf;
	struct sbefifo_buf *wbuf;
	struct list_head client;
	struct list_head xfrs;
	unsigned long flags;
#define SBEFIFO_XFR_WRITE_DONE		1
#define SBEFIFO_XFR_RESP_PENDING	2
#define SBEFIFO_XFR_COMPLETE		3
#define SBEFIFO_XFR_CANCEL		4
};

struct sbefifo_client {
	struct sbefifo_buf rbuf;
	struct sbefifo_buf wbuf;
	struct list_head xfrs;
	struct sbefifo *dev;
	struct kref kref;
	unsigned long f_flags;
};

static DEFINE_IDA(sbefifo_ida);

static int sbefifo_inw(struct sbefifo *sbefifo, int reg, u32 *word)
{
	int rc;
	__be32 raw_word;

	rc = fsi_device_read(sbefifo->fsi_dev, reg, &raw_word,
			     sizeof(raw_word));
	if (rc)
		return rc;

	*word = be32_to_cpu(raw_word);

	return 0;
}

static int sbefifo_outw(struct sbefifo *sbefifo, int reg, u32 word)
{
	__be32 raw_word = cpu_to_be32(word);

	return fsi_device_write(sbefifo->fsi_dev, reg, &raw_word,
				sizeof(raw_word));
}

/* Don't flip endianness of data to/from FIFO, just pass through. */
static int sbefifo_readw(struct sbefifo *sbefifo, u32 *word)
{
	return fsi_device_read(sbefifo->fsi_dev, SBEFIFO_DWN, word,
			       sizeof(*word));
}

static int sbefifo_writew(struct sbefifo *sbefifo, u32 word)
{
	return fsi_device_write(sbefifo->fsi_dev, SBEFIFO_UP, &word,
				sizeof(word));
}

static int sbefifo_ack_eot(struct sbefifo *sbefifo)
{
	u32 discard;
	int ret;

	 /* Discard the EOT word. */
	ret = sbefifo_readw(sbefifo, &discard);
	if (ret)
		return ret;

	return sbefifo_outw(sbefifo, SBEFIFO_DWN | SBEFIFO_EOT_ACK,
			    SBEFIFO_EOT_MAGIC);
}

static size_t sbefifo_dev_nwreadable(u32 sts)
{
	static const u32 FIFO_NTRY_CNT_MSK = 0x000f0000;
	static const unsigned int FIFO_NTRY_CNT_SHIFT = 16;

	return (sts & FIFO_NTRY_CNT_MSK) >> FIFO_NTRY_CNT_SHIFT;
}

static size_t sbefifo_dev_nwwriteable(u32 sts)
{
	static const size_t FIFO_DEPTH = 8;

	return FIFO_DEPTH - sbefifo_dev_nwreadable(sts);
}

static void sbefifo_buf_init(struct sbefifo_buf *buf)
{
	WRITE_ONCE(buf->flags, 0);
	WRITE_ONCE(buf->rpos, buf->buf);
	WRITE_ONCE(buf->wpos, buf->buf);
}

static size_t sbefifo_buf_nbreadable(struct sbefifo_buf *buf)
{
	size_t n;
	u32 *rpos = READ_ONCE(buf->rpos);
	u32 *wpos = READ_ONCE(buf->wpos);

	if (test_bit(SBEFIFO_BUF_FULL, &buf->flags))
		n = SBEFIFO_BUF_CNT;
	else if (rpos <= wpos)
		n = wpos - rpos;
	else
		n = (buf->buf + SBEFIFO_BUF_CNT) - rpos;

	return n << 2;
}

static size_t sbefifo_buf_nbwriteable(struct sbefifo_buf *buf)
{
	size_t n;
	u32 *rpos = READ_ONCE(buf->rpos);
	u32 *wpos = READ_ONCE(buf->wpos);

	if (test_bit(SBEFIFO_BUF_FULL, &buf->flags))
		n = 0;
	else if (wpos < rpos)
		n = rpos - wpos;
	else
		n = (buf->buf + SBEFIFO_BUF_CNT) - wpos;

	return n << 2;
}

/*
 * Update pointers and flags after doing a buffer read.  Return true if the
 * buffer is now empty;
 */
static bool sbefifo_buf_readnb(struct sbefifo_buf *buf, size_t n)
{
	u32 *rpos = READ_ONCE(buf->rpos);
	u32 *wpos = READ_ONCE(buf->wpos);

	if (n)
		clear_bit(SBEFIFO_BUF_FULL, &buf->flags);

	rpos += (n >> 2);
	if (rpos == buf->buf + SBEFIFO_BUF_CNT)
		rpos = buf->buf;

	WRITE_ONCE(buf->rpos, rpos);

	return rpos == wpos;
}

/*
 * Update pointers and flags after doing a buffer write.  Return true if the
 * buffer is now full.
 */
static bool sbefifo_buf_wrotenb(struct sbefifo_buf *buf, size_t n)
{
	u32 *rpos = READ_ONCE(buf->rpos);
	u32 *wpos = READ_ONCE(buf->wpos);

	wpos += (n >> 2);
	if (wpos == buf->buf + SBEFIFO_BUF_CNT)
		wpos = buf->buf;
	if (wpos == rpos)
		set_bit(SBEFIFO_BUF_FULL, &buf->flags);

	WRITE_ONCE(buf->wpos, wpos);

	return rpos == wpos;
}

static void sbefifo_free(struct kref *kref)
{
	struct sbefifo *sbefifo = container_of(kref, struct sbefifo, kref);

	kfree(sbefifo);
}

static void sbefifo_get(struct sbefifo *sbefifo)
{
	kref_get(&sbefifo->kref);
}

static void sbefifo_put(struct sbefifo *sbefifo)
{
	kref_put(&sbefifo->kref, sbefifo_free);
}

static struct sbefifo_xfr *sbefifo_enq_xfr(struct sbefifo_client *client)
{
	struct sbefifo *sbefifo = client->dev;
	struct sbefifo_xfr *xfr;

	if (READ_ONCE(sbefifo->rc))
		return ERR_PTR(sbefifo->rc);

	xfr = kzalloc(sizeof(*xfr), GFP_KERNEL);
	if (!xfr)
		return ERR_PTR(-ENOMEM);

	xfr->rbuf = &client->rbuf;
	xfr->wbuf = &client->wbuf;
	list_add_tail(&xfr->xfrs, &sbefifo->xfrs);
	list_add_tail(&xfr->client, &client->xfrs);

	return xfr;
}

static bool sbefifo_xfr_rsp_pending(struct sbefifo_client *client)
{
	struct sbefifo_xfr *xfr = list_first_entry_or_null(&client->xfrs,
							   struct sbefifo_xfr,
							   client);

	if (xfr && test_bit(SBEFIFO_XFR_RESP_PENDING, &xfr->flags))
		return true;

	return false;
}

static struct sbefifo_client *sbefifo_new_client(struct sbefifo *sbefifo)
{
	struct sbefifo_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return NULL;

	kref_init(&client->kref);
	client->dev = sbefifo;
	sbefifo_buf_init(&client->rbuf);
	sbefifo_buf_init(&client->wbuf);
	INIT_LIST_HEAD(&client->xfrs);

	sbefifo_get(sbefifo);

	return client;
}

static void sbefifo_client_release(struct kref *kref)
{
	struct sbefifo *sbefifo;
	struct sbefifo_client *client;
	struct sbefifo_xfr *xfr, *tmp;

	client = container_of(kref, struct sbefifo_client, kref);
	sbefifo = client->dev;

	if (!READ_ONCE(sbefifo->rc)) {
		list_for_each_entry_safe(xfr, tmp, &client->xfrs, client) {
			if (test_bit(SBEFIFO_XFR_COMPLETE, &xfr->flags)) {
				list_del(&xfr->client);
				kfree(xfr);
				continue;
			}

			/*
			 * The client left with pending or running xfrs.
			 * Cancel them.
			 */
			set_bit(SBEFIFO_XFR_CANCEL, &xfr->flags);
			sbefifo_get(sbefifo);
			if (mod_timer(&client->dev->poll_timer, jiffies))
				sbefifo_put(sbefifo);
		}
	}

	sbefifo_put(sbefifo);
	kfree(client);
}

static void sbefifo_get_client(struct sbefifo_client *client)
{
	kref_get(&client->kref);
}

static void sbefifo_put_client(struct sbefifo_client *client)
{
	kref_put(&client->kref, sbefifo_client_release);
}

static struct sbefifo_xfr *sbefifo_next_xfr(struct sbefifo *sbefifo)
{
	struct sbefifo_xfr *xfr, *tmp;

	list_for_each_entry_safe(xfr, tmp, &sbefifo->xfrs, xfrs) {
		if (unlikely(test_bit(SBEFIFO_XFR_CANCEL, &xfr->flags))) {
			/* Discard cancelled transfers. */
			list_del(&xfr->xfrs);
			kfree(xfr);
			continue;
		}

		return xfr;
	}

	return NULL;
}

static void sbefifo_poll_timer(unsigned long data)
{
	static const unsigned long EOT_MASK = 0x000000ff;
	struct sbefifo *sbefifo = (void *)data;
	struct sbefifo_buf *rbuf, *wbuf;
	struct sbefifo_xfr *xfr, *tmp;
	struct sbefifo_buf drain;
	size_t devn, bufn;
	int eot = 0;
	int ret = 0;
	u32 sts;
	int i;

	spin_lock(&sbefifo->lock);
	xfr = list_first_entry_or_null(&sbefifo->xfrs, struct sbefifo_xfr,
				       xfrs);
	if (!xfr)
		goto out_unlock;

	rbuf = xfr->rbuf;
	wbuf = xfr->wbuf;

	if (unlikely(test_bit(SBEFIFO_XFR_CANCEL, &xfr->flags))) {
		/* The client left. */
		rbuf = &drain;
		wbuf = &drain;
		sbefifo_buf_init(&drain);
		if (!test_bit(SBEFIFO_XFR_RESP_PENDING, &xfr->flags))
			set_bit(SBEFIFO_XFR_WRITE_DONE, &xfr->flags);
	}

	 /* Drain the write buffer. */
	while ((bufn = sbefifo_buf_nbreadable(wbuf))) {
		ret = sbefifo_inw(sbefifo, SBEFIFO_UP | SBEFIFO_STS, &sts);
		if (ret)
			goto out;

		devn = sbefifo_dev_nwwriteable(sts);
		if (devn == 0) {
			/* No open slot for write.  Reschedule. */
			sbefifo->poll_timer.expires = jiffies +
				SBEFIFO_RESCHEDULE;
			add_timer(&sbefifo->poll_timer);
			goto out_unlock;
		}

		devn = min_t(size_t, devn, bufn >> 2);
		for (i = 0; i < devn; i++) {
			ret = sbefifo_writew(sbefifo, *wbuf->rpos);
			if (ret)
				goto out;

			sbefifo_buf_readnb(wbuf, 1 << 2);
		}
	}

	 /* Send EOT if the writer is finished. */
	if (test_and_clear_bit(SBEFIFO_XFR_WRITE_DONE, &xfr->flags)) {
		ret = sbefifo_outw(sbefifo, SBEFIFO_UP | SBEFIFO_EOT_RAISE,
				   SBEFIFO_EOT_MAGIC);
		if (ret)
			goto out;

		/* Inform reschedules that the writer is finished. */
		set_bit(SBEFIFO_XFR_RESP_PENDING, &xfr->flags);
	}

	/* Nothing left to do if the writer is not finished. */
	if (!test_bit(SBEFIFO_XFR_RESP_PENDING, &xfr->flags))
		goto out;

	 /* Fill the read buffer. */
	while ((bufn = sbefifo_buf_nbwriteable(rbuf))) {
		ret = sbefifo_inw(sbefifo, SBEFIFO_DWN | SBEFIFO_STS, &sts);
		if (ret)
			goto out;

		devn = sbefifo_dev_nwreadable(sts);
		if (devn == 0) {
			/*
			 * Limit the maximum waiting period for data in the
			 * FIFO. If the SBE isn't running, we will wait
			 * forever.
			 */
			if (!xfr->wait_data_timeout) {
				xfr->wait_data_timeout =
					jiffies + SBEFIFO_MAX_RESCHDULE;
			} else if (time_after(jiffies,
					      xfr->wait_data_timeout)) {
				ret = -ETIME;
				goto out;
			}

			/* No data yet.  Reschedule. */
			sbefifo->poll_timer.expires = jiffies +
				SBEFIFO_RESCHEDULE;
			add_timer(&sbefifo->poll_timer);
			goto out_unlock;
		} else {
			xfr->wait_data_timeout = 0;
		}

		/* Fill.  The EOT word is discarded.  */
		devn = min_t(size_t, devn, bufn >> 2);
		eot = (sts & EOT_MASK) != 0;
		if (eot)
			devn--;

		for (i = 0; i < devn; i++) {
			ret = sbefifo_readw(sbefifo, rbuf->wpos);
			if (ret)
				goto out;

			if (likely(!test_bit(SBEFIFO_XFR_CANCEL, &xfr->flags)))
				sbefifo_buf_wrotenb(rbuf, 1 << 2);
		}

		if (eot) {
			ret = sbefifo_ack_eot(sbefifo);
			if (ret)
				goto out;

			set_bit(SBEFIFO_XFR_COMPLETE, &xfr->flags);
			list_del(&xfr->xfrs);
			if (unlikely(test_bit(SBEFIFO_XFR_CANCEL,
					      &xfr->flags)))
				kfree(xfr);
			break;
		}
	}

out:
	if (unlikely(ret)) {
		sbefifo->rc = ret;
		dev_err(&sbefifo->fsi_dev->dev,
			"Fatal bus access failure: %d\n", ret);
		list_for_each_entry_safe(xfr, tmp, &sbefifo->xfrs, xfrs) {
			list_del(&xfr->xfrs);
			kfree(xfr);
		}
		INIT_LIST_HEAD(&sbefifo->xfrs);

	} else if (eot && sbefifo_next_xfr(sbefifo)) {
		sbefifo_get(sbefifo);
		sbefifo->poll_timer.expires = jiffies;
		add_timer(&sbefifo->poll_timer);
	}

	sbefifo_put(sbefifo);
	wake_up_interruptible(&sbefifo->wait);

out_unlock:
	spin_unlock(&sbefifo->lock);
}

static int sbefifo_open(struct inode *inode, struct file *file)
{
	struct sbefifo *sbefifo = container_of(file->private_data,
					       struct sbefifo, mdev);
	struct sbefifo_client *client;
	int ret;

	ret = READ_ONCE(sbefifo->rc);
	if (ret)
		return ret;

	client = sbefifo_new_client(sbefifo);
	if (!client)
		return -ENOMEM;

	file->private_data = client;
	client->f_flags = file->f_flags;

	return 0;
}

static unsigned int sbefifo_poll(struct file *file, poll_table *wait)
{
	struct sbefifo_client *client = file->private_data;
	struct sbefifo *sbefifo = client->dev;
	unsigned int mask = 0;

	poll_wait(file, &sbefifo->wait, wait);

	if (READ_ONCE(sbefifo->rc))
		mask |= POLLERR;

	if (sbefifo_buf_nbreadable(&client->rbuf))
		mask |= POLLIN;

	if (sbefifo_buf_nbwriteable(&client->wbuf))
		mask |= POLLOUT;

	return mask;
}

static bool sbefifo_read_ready(struct sbefifo *sbefifo,
			       struct sbefifo_client *client, size_t *n,
			       size_t *ret)
{
	struct sbefifo_xfr *xfr = list_first_entry_or_null(&client->xfrs,
							   struct sbefifo_xfr,
							   client);

	*n = sbefifo_buf_nbreadable(&client->rbuf);
	*ret = READ_ONCE(sbefifo->rc);

	return *ret || *n ||
		(xfr && test_bit(SBEFIFO_XFR_COMPLETE, &xfr->flags));
}

static ssize_t sbefifo_read_common(struct sbefifo_client *client,
				   char __user *ubuf, char *kbuf, size_t len)
{
	struct sbefifo *sbefifo = client->dev;
	struct sbefifo_xfr *xfr;
	size_t n;
	ssize_t ret = 0;

	if ((len >> 2) << 2 != len)
		return -EINVAL;

	if ((client->f_flags & O_NONBLOCK) && !sbefifo_xfr_rsp_pending(client))
		return -EAGAIN;

	sbefifo_get_client(client);
	if (wait_event_interruptible(sbefifo->wait,
				     sbefifo_read_ready(sbefifo, client, &n,
							&ret))) {
		ret = -ERESTARTSYS;
		goto out;
	}

	if (ret) {
		INIT_LIST_HEAD(&client->xfrs);
		goto out;
	}

	n = min_t(size_t, n, len);

	if (ubuf) {
		if (copy_to_user(ubuf, READ_ONCE(client->rbuf.rpos), n)) {
			ret = -EFAULT;
			goto out;
		}
	} else {
		memcpy(kbuf, READ_ONCE(client->rbuf.rpos), n);
	}

	if (sbefifo_buf_readnb(&client->rbuf, n)) {
		xfr = list_first_entry_or_null(&client->xfrs,
					       struct sbefifo_xfr, client);
		if (!xfr) {
			/* should be impossible to not have an xfr here */
			WARN_ONCE(1, "no xfr in queue");
			ret = -EPROTO;
			goto out;
		}

		if (!test_bit(SBEFIFO_XFR_COMPLETE, &xfr->flags)) {
			/*
			 * Fill the read buffer back up.
			 */
			sbefifo_get(sbefifo);
			if (mod_timer(&client->dev->poll_timer, jiffies))
				sbefifo_put(sbefifo);
		} else {
			list_del(&xfr->client);
			kfree(xfr);
			wake_up_interruptible(&sbefifo->wait);
		}
	}

	ret = n;

out:
	sbefifo_put_client(client);
	return ret;
}

static ssize_t sbefifo_read(struct file *file, char __user *buf, size_t len,
			    loff_t *offset)
{
	struct sbefifo_client *client = file->private_data;

	return sbefifo_read_common(client, buf, NULL, len);
}

static bool sbefifo_write_ready(struct sbefifo *sbefifo,
				struct sbefifo_xfr *xfr,
				struct sbefifo_client *client, size_t *n)
{
	struct sbefifo_xfr *next = list_first_entry_or_null(&client->xfrs,
							    struct sbefifo_xfr,
							    client);

	*n = sbefifo_buf_nbwriteable(&client->wbuf);
	return READ_ONCE(sbefifo->rc) || (next == xfr && *n);
}

static ssize_t sbefifo_write_common(struct sbefifo_client *client,
				    const char __user *ubuf, const char *kbuf,
				    size_t len)
{
	struct sbefifo *sbefifo = client->dev;
	struct sbefifo_xfr *xfr;
	ssize_t ret = 0;
	size_t n;

	if ((len >> 2) << 2 != len)
		return -EINVAL;

	if (!len)
		return 0;

	sbefifo_get_client(client);
	n = sbefifo_buf_nbwriteable(&client->wbuf);

	spin_lock_irq(&sbefifo->lock);
	xfr = sbefifo_next_xfr(sbefifo);	/* next xfr to be executed */

	if ((client->f_flags & O_NONBLOCK) && xfr && n < len) {
		spin_unlock_irq(&sbefifo->lock);
		ret = -EAGAIN;
		goto out;
	}

	xfr = sbefifo_enq_xfr(client);		/* this xfr queued up */
	if (IS_ERR(xfr)) {
		spin_unlock_irq(&sbefifo->lock);
		ret = PTR_ERR(xfr);
		goto out;
	}

	spin_unlock_irq(&sbefifo->lock);

	/*
	 * Partial writes are not really allowed in that EOT is sent exactly
	 * once per write.
	 */
	while (len) {
		if (wait_event_interruptible(sbefifo->wait,
					     sbefifo_write_ready(sbefifo, xfr,
								 client,
								 &n))) {
			set_bit(SBEFIFO_XFR_CANCEL, &xfr->flags);
			sbefifo_get(sbefifo);
			if (mod_timer(&sbefifo->poll_timer, jiffies))
				sbefifo_put(sbefifo);

			ret = -ERESTARTSYS;
			goto out;
		}

		if (sbefifo->rc) {
			INIT_LIST_HEAD(&client->xfrs);
			ret = sbefifo->rc;
			goto out;
		}

		n = min_t(size_t, n, len);

		if (ubuf) {
			if (copy_from_user(READ_ONCE(client->wbuf.wpos), ubuf,
			    n)) {
				set_bit(SBEFIFO_XFR_CANCEL, &xfr->flags);
				sbefifo_get(sbefifo);
				if (mod_timer(&sbefifo->poll_timer, jiffies))
					sbefifo_put(sbefifo);
				ret = -EFAULT;
				goto out;
			}

			ubuf += n;
		} else {
			memcpy(READ_ONCE(client->wbuf.wpos), kbuf, n);
			kbuf += n;
		}

		sbefifo_buf_wrotenb(&client->wbuf, n);
		len -= n;
		ret += n;

		/*
		 * Set this before starting timer to avoid race condition on
		 * this flag with the timer function writer.
		 */
		if (!len)
			set_bit(SBEFIFO_XFR_WRITE_DONE, &xfr->flags);

		/*
		 * Drain the write buffer.
		 */
		sbefifo_get(sbefifo);
		if (mod_timer(&client->dev->poll_timer, jiffies))
			sbefifo_put(sbefifo);
	}

out:
	sbefifo_put_client(client);
	return ret;
}

static ssize_t sbefifo_write(struct file *file, const char __user *buf,
			     size_t len, loff_t *offset)
{
	struct sbefifo_client *client = file->private_data;

	return sbefifo_write_common(client, buf, NULL, len);
}

static int sbefifo_release(struct inode *inode, struct file *file)
{
	struct sbefifo_client *client = file->private_data;
	struct sbefifo *sbefifo = client->dev;

	sbefifo_put_client(client);

	return READ_ONCE(sbefifo->rc);
}

static const struct file_operations sbefifo_fops = {
	.owner		= THIS_MODULE,
	.open		= sbefifo_open,
	.read		= sbefifo_read,
	.write		= sbefifo_write,
	.poll		= sbefifo_poll,
	.release	= sbefifo_release,
};

struct sbefifo_client *sbefifo_drv_open(struct device *dev,
					unsigned long flags)
{
	struct sbefifo_client *client;
	struct sbefifo *sbefifo = dev_get_drvdata(dev);

	if (!sbefifo)
		return NULL;

	client = sbefifo_new_client(sbefifo);
	if (client)
		client->f_flags = flags;

	return client;
}
EXPORT_SYMBOL_GPL(sbefifo_drv_open);

int sbefifo_drv_read(struct sbefifo_client *client, char *buf, size_t len)
{
	return sbefifo_read_common(client, NULL, buf, len);
}
EXPORT_SYMBOL_GPL(sbefifo_drv_read);

int sbefifo_drv_write(struct sbefifo_client *client, const char *buf,
		      size_t len)
{
	return sbefifo_write_common(client, NULL, buf, len);
}
EXPORT_SYMBOL_GPL(sbefifo_drv_write);

void sbefifo_drv_release(struct sbefifo_client *client)
{
	if (!client)
		return;

	sbefifo_put_client(client);
}
EXPORT_SYMBOL_GPL(sbefifo_drv_release);

static int sbefifo_unregister_child(struct device *dev, void *data)
{
	struct platform_device *child = to_platform_device(dev);

	of_device_unregister(child);
	if (dev->of_node)
		of_node_clear_flag(dev->of_node, OF_POPULATED);

	return 0;
}

static int sbefifo_request_reset(struct sbefifo *sbefifo)
{
	int ret;
	u32 status;
	unsigned long start;
	const unsigned int wait_time = 5;	/* jiffies */
	const unsigned long timeout = msecs_to_jiffies(250);

	ret = sbefifo_outw(sbefifo, SBEFIFO_UP | SBEFIFO_REQ_RESET, 1);
	if (ret)
		return ret;

	start = jiffies;

	do {
		ret = sbefifo_inw(sbefifo, SBEFIFO_UP | SBEFIFO_STS, &status);
		if (ret)
			return ret;

		if (!(status & SBEFIFO_STS_RESET_REQ))
			return 0;

		set_current_state(TASK_INTERRUPTIBLE);
		if (schedule_timeout(wait_time) > 0)
			return -EINTR;
	} while (time_after(start + timeout, jiffies));

	return -ETIME;
}

static int sbefifo_probe(struct device *dev)
{
	struct fsi_device *fsi_dev = to_fsi_dev(dev);
	struct sbefifo *sbefifo;
	struct device_node *np;
	struct platform_device *child;
	char child_name[32];
	u32 up, down;
	int ret, child_idx = 0;

	dev_dbg(dev, "Found sbefifo device\n");
	sbefifo = kzalloc(sizeof(*sbefifo), GFP_KERNEL);
	if (!sbefifo)
		return -ENOMEM;

	sbefifo->fsi_dev = fsi_dev;

	ret = sbefifo_inw(sbefifo, SBEFIFO_UP | SBEFIFO_STS, &up);
	if (ret)
		return ret;

	ret = sbefifo_inw(sbefifo, SBEFIFO_DWN | SBEFIFO_STS, &down);
	if (ret)
		return ret;

	if (!(up & SBEFIFO_EMPTY) || !(down & SBEFIFO_EMPTY)) {
		ret = sbefifo_request_reset(sbefifo);
		if (ret) {
			dev_err(dev,
				"fifos weren't empty and failed the reset\n");
			return ret;
		}
	}

	spin_lock_init(&sbefifo->lock);
	kref_init(&sbefifo->kref);
	init_waitqueue_head(&sbefifo->wait);
	INIT_LIST_HEAD(&sbefifo->xfrs);

	sbefifo->idx = ida_simple_get(&sbefifo_ida, 1, INT_MAX, GFP_KERNEL);
	snprintf(sbefifo->name, sizeof(sbefifo->name), "sbefifo%d",
		 sbefifo->idx);

	/* This bit of silicon doesn't offer any interrupts... */
	setup_timer(&sbefifo->poll_timer, sbefifo_poll_timer,
		    (unsigned long)sbefifo);

	sbefifo->mdev.minor = MISC_DYNAMIC_MINOR;
	sbefifo->mdev.fops = &sbefifo_fops;
	sbefifo->mdev.name = sbefifo->name;
	sbefifo->mdev.parent = dev;
	ret = misc_register(&sbefifo->mdev);
	if (ret) {
		dev_err(dev, "failed to register miscdevice: %d\n", ret);
		ida_simple_remove(&sbefifo_ida, sbefifo->idx);
		sbefifo_put(sbefifo);
		return ret;
	}

	/* create platform devs for dts child nodes (occ, etc) */
	for_each_available_child_of_node(dev->of_node, np) {
		snprintf(child_name, sizeof(child_name), "%s-dev%d",
			 sbefifo->name, child_idx++);
		child = of_platform_device_create(np, child_name, dev);
		if (!child)
			dev_warn(dev, "failed to create child %s dev\n",
				 child_name);
	}

	dev_set_drvdata(dev, sbefifo);

	return 0;
}

static int sbefifo_remove(struct device *dev)
{
	struct sbefifo *sbefifo = dev_get_drvdata(dev);
	struct sbefifo_xfr *xfr, *tmp;

	spin_lock(&sbefifo->lock);

	WRITE_ONCE(sbefifo->rc, -ENODEV);
	list_for_each_entry_safe(xfr, tmp, &sbefifo->xfrs, xfrs) {
		list_del(&xfr->xfrs);
		kfree(xfr);
	}

	INIT_LIST_HEAD(&sbefifo->xfrs);

	spin_unlock(&sbefifo->lock);

	wake_up_all(&sbefifo->wait);

	misc_deregister(&sbefifo->mdev);
	device_for_each_child(dev, NULL, sbefifo_unregister_child);

	ida_simple_remove(&sbefifo_ida, sbefifo->idx);

	if (del_timer_sync(&sbefifo->poll_timer))
		sbefifo_put(sbefifo);

	sbefifo_put(sbefifo);

	return 0;
}

static struct fsi_device_id sbefifo_ids[] = {
	{
		.engine_type = FSI_ENGID_SBE,
		.version = FSI_VERSION_ANY,
	},
	{ 0 }
};

static struct fsi_driver sbefifo_drv = {
	.id_table = sbefifo_ids,
	.drv = {
		.name = DEVICE_NAME,
		.bus = &fsi_bus_type,
		.probe = sbefifo_probe,
		.remove = sbefifo_remove,
	}
};

static int sbefifo_init(void)
{
	return fsi_driver_register(&sbefifo_drv);
}

static void sbefifo_exit(void)
{
	fsi_driver_unregister(&sbefifo_drv);

	ida_destroy(&sbefifo_ida);
}

module_init(sbefifo_init);
module_exit(sbefifo_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brad Bishop <bradleyb@fuzziesquirrel.com>");
MODULE_AUTHOR("Eddie James <eajames@linux.vnet.ibm.com>");
MODULE_DESCRIPTION("Linux device interface to the POWER Self Boot Engine");
