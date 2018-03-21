/*
 * A FSI master controller, using a simple GPIO bit-banging interface
 */

#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/fsi.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "fsi-master.h"

#define	FSI_GPIO_STD_DLY	3	/* Standard pin delay in uS */
#define	FSI_ECHO_DELAY_CLOCKS	16	/* Number clocks for echo delay */
#define	FSI_PRE_BREAK_CLOCKS	50	/* Number clocks to prep for break */
#define	FSI_BREAK_CLOCKS	256	/* Number of clocks to issue break */
#define	FSI_POST_BREAK_CLOCKS	16000	/* Number clocks to set up cfam */
#define	FSI_INIT_CLOCKS		5000	/* Clock out any old data */
#define	FSI_GPIO_STD_DELAY	10	/* Standard GPIO delay in nS */
					/* todo: adjust down as low as */
					/* possible or eliminate */
#define	FSI_GPIO_CMD_DPOLL      0x2
#define	FSI_GPIO_CMD_TERM	0x3f
#define FSI_GPIO_CMD_ABS_AR	0x4

#define	FSI_GPIO_DPOLL_CLOCKS	100      /* < 21 will cause slave to hang */

/* Bus errors */
#define	FSI_GPIO_ERR_BUSY	1	/* Slave stuck in busy state */
#define	FSI_GPIO_RESP_ERRA	2	/* Any (misc) Error */
#define	FSI_GPIO_RESP_ERRC	3	/* Slave reports master CRC error */
#define	FSI_GPIO_MTOE		4	/* Master time out error */
#define	FSI_GPIO_CRC_INVAL	5	/* Master reports slave CRC error */

/* Normal slave responses */
#define	FSI_GPIO_RESP_BUSY	1
#define	FSI_GPIO_RESP_ACK	0
#define	FSI_GPIO_RESP_ACKD	4

#define	FSI_GPIO_MAX_BUSY	100
#define	FSI_GPIO_MTOE_COUNT	1000
#define	FSI_GPIO_DRAIN_BITS	20
#define	FSI_GPIO_CRC_SIZE	4
#define	FSI_GPIO_MSG_ID_SIZE		2
#define	FSI_GPIO_MSG_RESPID_SIZE	2
#define	FSI_GPIO_PRIME_SLAVE_CLOCKS	100

static DEFINE_SPINLOCK(fsi_gpio_cmd_lock);	/* lock around fsi commands */

struct fsi_master_gpio {
	struct fsi_master	master;
	struct device		*dev;
	struct gpio_desc	*gpio_clk;
	struct gpio_desc	*gpio_data;
	struct gpio_desc	*gpio_trans;	/* Voltage translator */
	struct gpio_desc	*gpio_enable;	/* FSI enable */
	struct gpio_desc	*gpio_mux;	/* Mux control */
	bool			external_mode;
};

#define CREATE_TRACE_POINTS
#include <trace/events/fsi_master_gpio.h>

#define to_fsi_master_gpio(m) container_of(m, struct fsi_master_gpio, master)

struct fsi_gpio_msg {
	uint64_t	msg;
	uint8_t		bits;
};

static void clock_toggle(struct fsi_master_gpio *master, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		gpiod_set_value(master->gpio_clk, 0);
		udelay(FSI_GPIO_STD_DLY);
		gpiod_set_value(master->gpio_clk, 1);
		udelay(FSI_GPIO_STD_DLY);
	}
}

static int sda_in(struct fsi_master_gpio *master)
{
	int in;

	in = gpiod_get_value(master->gpio_data);
	return in ? 1 : 0;
}

static void sda_out(struct fsi_master_gpio *master, int value)
{
	gpiod_set_value(master->gpio_data, value);
}

static void set_sda_input(struct fsi_master_gpio *master)
{
	gpiod_direction_input(master->gpio_data);
	if (master->gpio_trans)
		gpiod_set_value(master->gpio_trans, 0);
}

static void set_sda_output(struct fsi_master_gpio *master, int value)
{
	if (master->gpio_trans)
		gpiod_set_value(master->gpio_trans, 1);
	gpiod_direction_output(master->gpio_data, value);
}

static void clock_zeros(struct fsi_master_gpio *master, int count)
{
	set_sda_output(master, 1);
	clock_toggle(master, count);
}

static void serial_in(struct fsi_master_gpio *master, struct fsi_gpio_msg *msg,
			uint8_t num_bits)
{
	uint8_t bit, in_bit;

	set_sda_input(master);

	for (bit = 0; bit < num_bits; bit++) {
		clock_toggle(master, 1);
		in_bit = sda_in(master);
		msg->msg <<= 1;
		msg->msg |= ~in_bit & 0x1;	/* Data is negative active */
	}
	msg->bits += num_bits;

	trace_fsi_master_gpio_in(master, num_bits, msg->msg);
}

static void serial_out(struct fsi_master_gpio *master,
			const struct fsi_gpio_msg *cmd)
{
	uint8_t bit;
	uint64_t msg = ~cmd->msg;	/* Data is negative active */
	uint64_t sda_mask = 0x1ULL << (cmd->bits - 1);
	uint64_t last_bit = ~0;
	int next_bit;

	trace_fsi_master_gpio_out(master, cmd->bits, cmd->msg);

	if (!cmd->bits) {
		dev_warn(master->dev, "trying to output 0 bits\n");
		return;
	}
	set_sda_output(master, 0);

	/* Send the start bit */
	sda_out(master, 0);
	clock_toggle(master, 1);

	/* Send the message */
	for (bit = 0; bit < cmd->bits; bit++) {
		next_bit = (msg & sda_mask) >> (cmd->bits - 1);
		if (last_bit ^ next_bit) {
			sda_out(master, next_bit);
			last_bit = next_bit;
		}
		clock_toggle(master, 1);
		msg <<= 1;
	}
}

static void msg_push_bits(struct fsi_gpio_msg *msg, uint64_t data, int bits)
{
	msg->msg <<= bits;
	msg->msg |= data & ((1ull << bits) - 1);
	msg->bits += bits;
}

static void msg_push_crc(struct fsi_gpio_msg *msg)
{
	uint8_t crc;
	int top;

	top = msg->bits & 0x3;

	/* start bit, and any non-aligned top bits */
	crc = fsi_crc4(0,
			1 << top | msg->msg >> (msg->bits - top),
			top + 1);

	/* aligned bits */
	crc = fsi_crc4(crc, msg->msg, msg->bits - top);

	msg_push_bits(msg, crc, 4);
}

static void build_abs_ar_command(struct fsi_gpio_msg *cmd,
		uint8_t id, uint32_t addr, size_t size, const void *data)
{
	bool write = !!data;
	uint8_t ds;
	int i;

	cmd->bits = 0;
	cmd->msg = 0;

	msg_push_bits(cmd, id, 2);
	msg_push_bits(cmd, FSI_GPIO_CMD_ABS_AR, 3);
	msg_push_bits(cmd, write ? 0 : 1, 1);

	/*
	 * The read/write size is encoded in the lower bits of the address
	 * (as it must be naturally-aligned), and the following ds bit.
	 *
	 *	size	addr:1	addr:0	ds
	 *	1	x	x	0
	 *	2	x	0	1
	 *	4	0	1	1
	 *
	 */
	ds = size > 1 ? 1 : 0;
	addr &= ~(size - 1);
	if (size == 4)
		addr |= 1;

	msg_push_bits(cmd, addr & ((1 << 21) - 1), 21);
	msg_push_bits(cmd, ds, 1);
	for (i = 0; write && i < size; i++)
		msg_push_bits(cmd, ((uint8_t *)data)[i], 8);

	msg_push_crc(cmd);
}

static void build_dpoll_command(struct fsi_gpio_msg *cmd, uint8_t slave_id)
{
	cmd->bits = 0;
	cmd->msg = 0;

	msg_push_bits(cmd, slave_id, 2);
	msg_push_bits(cmd, FSI_GPIO_CMD_DPOLL, 3);
	msg_push_crc(cmd);
}

static void echo_delay(struct fsi_master_gpio *master)
{
	set_sda_output(master, 1);
	clock_toggle(master, FSI_ECHO_DELAY_CLOCKS);
}

static void build_term_command(struct fsi_gpio_msg *cmd, uint8_t slave_id)
{
	cmd->bits = 0;
	cmd->msg = 0;

	msg_push_bits(cmd, slave_id, 2);
	msg_push_bits(cmd, FSI_GPIO_CMD_TERM, 6);
	msg_push_crc(cmd);
}

/*
 * Store information on master errors so handler can detect and clean
 * up the bus
 */
static void fsi_master_gpio_error(struct fsi_master_gpio *master, int error)
{

}

static int read_one_response(struct fsi_master_gpio *master,
		uint8_t data_size, struct fsi_gpio_msg *msgp, uint8_t *tagp)
{
	struct fsi_gpio_msg msg;
	uint8_t id, tag;
	uint32_t crc;
	int i;

	/* wait for the start bit */
	for (i = 0; i < FSI_GPIO_MTOE_COUNT; i++) {
		msg.bits = 0;
		msg.msg = 0;
		serial_in(master, &msg, 1);
		if (msg.msg)
			break;
	}
	if (i >= FSI_GPIO_MTOE_COUNT) {
		dev_dbg(master->dev,
			"Master time out waiting for response\n");
		fsi_master_gpio_error(master, FSI_GPIO_MTOE);
		return -EIO;
	}

	msg.bits = 0;
	msg.msg = 0;

	/* Read slave ID & response tag */
	serial_in(master, &msg, 4);

	id = (msg.msg >> FSI_GPIO_MSG_RESPID_SIZE) & 0x3;
	tag = msg.msg & 0x3;

	/* if we have an ACK, and we're expecting data, clock the
	 * data in too
	 */
	if (tag == FSI_GPIO_RESP_ACK && data_size)
		serial_in(master, &msg, data_size * 8);

	/* read CRC */
	serial_in(master, &msg, FSI_GPIO_CRC_SIZE);

	/* we have a whole message now; check CRC */
	crc = fsi_crc4(0, 1, 1);
	crc = fsi_crc4(crc, msg.msg, msg.bits);
	if (crc) {
		dev_dbg(master->dev, "ERR response CRC\n");
		fsi_master_gpio_error(master, FSI_GPIO_CRC_INVAL);
		return -EIO;
	}

	if (msgp)
		*msgp = msg;
	if (tagp)
		*tagp = tag;

	return 0;
}

static int issue_term(struct fsi_master_gpio *master, uint8_t slave)
{
	struct fsi_gpio_msg cmd;
	uint8_t tag;
	int rc;

	build_term_command(&cmd, slave);
	serial_out(master, &cmd);
	echo_delay(master);

	rc = read_one_response(master, 0, NULL, &tag);
	if (rc) {
		dev_err(master->dev,
				"TERM failed; lost communication with slave\n");
		return -EIO;
	} else if (tag != FSI_GPIO_RESP_ACK) {
		dev_err(master->dev, "TERM failed; response %d\n", tag);
		return -EIO;
	}

	return 0;
}

static int poll_for_response(struct fsi_master_gpio *master,
		uint8_t slave, uint8_t size, void *data)
{
	struct fsi_gpio_msg response, cmd;
	int busy_count = 0, rc, i;
	uint8_t tag;

retry:
	rc = read_one_response(master, size, &response, &tag);
	if (rc)
		return rc;

	switch (tag) {
	case FSI_GPIO_RESP_ACK:
		if (size && data) {
			uint64_t val = response.msg;
			/* clear crc & mask */
			val >>= 4;
			val &= (1ull << (size * 8)) - 1;

			for (i = 0; i < size; i++) {
				((uint8_t *)data)[size-i-1] =
					val & 0xff;
				val >>= 8;
			}
		}
		break;
	case FSI_GPIO_RESP_BUSY:
		/*
		 * Its necessary to clock slave before issuing
		 * d-poll, not indicated in the hardware protocol
		 * spec. < 20 clocks causes slave to hang, 21 ok.
		 */
		clock_zeros(master, FSI_GPIO_DPOLL_CLOCKS);
		if (busy_count++ < FSI_GPIO_MAX_BUSY) {
			build_dpoll_command(&cmd, slave);
			serial_out(master, &cmd);
			echo_delay(master);
			goto retry;
		}
		dev_warn(master->dev,
			"ERR slave is stuck in busy state, issuing TERM\n");
		issue_term(master, slave);
		rc = -EIO;
		break;

	case FSI_GPIO_RESP_ERRA:
	case FSI_GPIO_RESP_ERRC:
		dev_dbg(master->dev, "ERR%c received: 0x%x\n",
			tag == FSI_GPIO_RESP_ERRA ? 'A' : 'C',
			(int)response.msg);
		fsi_master_gpio_error(master, response.msg);
		rc = -EIO;
		break;
	}

	/* Clock the slave enough to be ready for next operation */
	clock_zeros(master, FSI_GPIO_PRIME_SLAVE_CLOCKS);
	return rc;
}

static int fsi_master_gpio_xfer(struct fsi_master_gpio *master, uint8_t slave,
		struct fsi_gpio_msg *cmd, size_t resp_len, void *resp)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&fsi_gpio_cmd_lock, flags);
	if (master->external_mode) {
		spin_unlock_irqrestore(&fsi_gpio_cmd_lock, flags);
		return -EBUSY;
	}
	serial_out(master, cmd);
	echo_delay(master);
	rc = poll_for_response(master, slave, resp_len, resp);
	spin_unlock_irqrestore(&fsi_gpio_cmd_lock, flags);

	return rc;
}

static int fsi_master_gpio_read(struct fsi_master *_master, int link,
		uint8_t id, uint32_t addr, void *val, size_t size)
{
	struct fsi_master_gpio *master = to_fsi_master_gpio(_master);
	struct fsi_gpio_msg cmd;

	if (link != 0)
		return -ENODEV;

	build_abs_ar_command(&cmd, id, addr, size, NULL);
	return fsi_master_gpio_xfer(master, id, &cmd, size, val);
}

static int fsi_master_gpio_write(struct fsi_master *_master, int link,
		uint8_t id, uint32_t addr, const void *val, size_t size)
{
	struct fsi_master_gpio *master = to_fsi_master_gpio(_master);
	struct fsi_gpio_msg cmd;

	if (link != 0)
		return -ENODEV;

	build_abs_ar_command(&cmd, id, addr, size, val);
	return fsi_master_gpio_xfer(master, id, &cmd, 0, NULL);
}

static int fsi_master_gpio_term(struct fsi_master *_master,
		int link, uint8_t id)
{
	struct fsi_master_gpio *master = to_fsi_master_gpio(_master);
	struct fsi_gpio_msg cmd;

	if (link != 0)
		return -ENODEV;

	build_term_command(&cmd, id);
	return fsi_master_gpio_xfer(master, id, &cmd, 0, NULL);
}

/*
 * Issue a break command on link
 */
static int fsi_master_gpio_break(struct fsi_master *_master, int link)
{
	struct fsi_master_gpio *master = to_fsi_master_gpio(_master);
	unsigned long flags;

	if (link != 0)
		return -ENODEV;

	trace_fsi_master_gpio_break(master);

	spin_lock_irqsave(&fsi_gpio_cmd_lock, flags);
	if (master->external_mode) {
		spin_unlock_irqrestore(&fsi_gpio_cmd_lock, flags);
		return -EBUSY;
	}
	set_sda_output(master, 1);
	sda_out(master, 1);
	clock_toggle(master, FSI_PRE_BREAK_CLOCKS);
	sda_out(master, 0);
	clock_toggle(master, FSI_BREAK_CLOCKS);
	echo_delay(master);
	sda_out(master, 1);
	clock_toggle(master, FSI_POST_BREAK_CLOCKS);
	spin_unlock_irqrestore(&fsi_gpio_cmd_lock, flags);

	/* Wait for logic reset to take effect */
	udelay(200);

	return 0;
}

static void fsi_master_gpio_init(struct fsi_master_gpio *master)
{
	if (master->gpio_mux)
		gpiod_direction_output(master->gpio_mux, 1);
	if (master->gpio_trans)
		gpiod_direction_output(master->gpio_trans, 1);
	if (master->gpio_enable)
		gpiod_direction_output(master->gpio_enable, 1);
	gpiod_direction_output(master->gpio_clk, 1);
	gpiod_direction_output(master->gpio_data, 1);

	/* todo: evaluate if clocks can be reduced */
	clock_zeros(master, FSI_INIT_CLOCKS);
}

static void fsi_master_gpio_init_external(struct fsi_master_gpio *master)
{
	gpiod_direction_output(master->gpio_mux, 0);
	gpiod_direction_output(master->gpio_trans, 0);
	gpiod_direction_output(master->gpio_enable, 1);
	gpiod_direction_input(master->gpio_clk);
	gpiod_direction_input(master->gpio_data);
}

static int fsi_master_gpio_link_enable(struct fsi_master *_master, int link)
{
	struct fsi_master_gpio *master = to_fsi_master_gpio(_master);
	unsigned long flags;
	int rc = -EBUSY;

	if (link != 0)
		return -ENODEV;

	spin_lock_irqsave(&fsi_gpio_cmd_lock, flags);
	if (!master->external_mode && master->gpio_enable) {
		gpiod_set_value(master->gpio_enable, 1);
		rc = 0;
	}
	spin_unlock_irqrestore(&fsi_gpio_cmd_lock, flags);

	return rc;
}

static ssize_t external_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsi_master_gpio *master = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%u\n",
			master->external_mode ? 1 : 0);
}

static ssize_t external_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fsi_master_gpio *master = dev_get_drvdata(dev);
	unsigned long flags, val;
	bool external_mode;
	int err;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;

	external_mode = !!val;

	spin_lock_irqsave(&fsi_gpio_cmd_lock, flags);

	if (external_mode == master->external_mode) {
		spin_unlock_irqrestore(&fsi_gpio_cmd_lock, flags);
		return count;
	}

	master->external_mode = external_mode;
	if (master->external_mode)
		fsi_master_gpio_init_external(master);
	else
		fsi_master_gpio_init(master);
	spin_unlock_irqrestore(&fsi_gpio_cmd_lock, flags);

	fsi_master_rescan(&master->master);

	return count;
}

static DEVICE_ATTR(external_mode, 0664,
		external_mode_show, external_mode_store);

static void fsi_master_gpio_release(struct device *dev)
{
}

static int fsi_master_gpio_probe(struct platform_device *pdev)
{
	struct fsi_master_gpio *master;
	struct gpio_desc *gpio;
	int rc;

	master = devm_kzalloc(&pdev->dev, sizeof(*master), GFP_KERNEL);
	if (!master)
		return -ENOMEM;

	master->dev = &pdev->dev;
	master->master.dev.parent = master->dev;
	master->master.dev.of_node = of_node_get(dev_of_node(master->dev));
	master->master.dev.release = fsi_master_gpio_release;

	gpio = devm_gpiod_get(&pdev->dev, "clock", 0);
	if (IS_ERR(gpio)) {
		dev_err(&pdev->dev, "failed to get clock gpio\n");
		return PTR_ERR(gpio);
	}
	master->gpio_clk = gpio;

	gpio = devm_gpiod_get(&pdev->dev, "data", 0);
	if (IS_ERR(gpio)) {
		dev_err(&pdev->dev, "failed to get data gpio\n");
		return PTR_ERR(gpio);
	}
	master->gpio_data = gpio;

	/* Optional GPIOs */
	gpio = devm_gpiod_get_optional(&pdev->dev, "trans", 0);
	if (IS_ERR(gpio)) {
		dev_err(&pdev->dev, "failed to get trans gpio\n");
		return PTR_ERR(gpio);
	}
	master->gpio_trans = gpio;

	gpio = devm_gpiod_get_optional(&pdev->dev, "enable", 0);
	if (IS_ERR(gpio)) {
		dev_err(&pdev->dev, "failed to get enable gpio\n");
		return PTR_ERR(gpio);
	}
	master->gpio_enable = gpio;

	gpio = devm_gpiod_get_optional(&pdev->dev, "mux", 0);
	if (IS_ERR(gpio)) {
		dev_err(&pdev->dev, "failed to get mux gpio\n");
		return PTR_ERR(gpio);
	}
	master->gpio_mux = gpio;

	master->master.n_links = 1;
	master->master.flags = FSI_MASTER_FLAG_SWCLOCK;
	master->master.read = fsi_master_gpio_read;
	master->master.write = fsi_master_gpio_write;
	master->master.term = fsi_master_gpio_term;
	master->master.send_break = fsi_master_gpio_break;
	master->master.link_enable = fsi_master_gpio_link_enable;
	platform_set_drvdata(pdev, master);

	fsi_master_gpio_init(master);

	rc = device_create_file(&pdev->dev, &dev_attr_external_mode);
	if (rc)
		return rc;

	fsi_master_register(&master->master);

	return 0;
}


static int fsi_master_gpio_remove(struct platform_device *pdev)
{
	struct fsi_master_gpio *master = platform_get_drvdata(pdev);

	devm_gpiod_put(&pdev->dev, master->gpio_clk);
	devm_gpiod_put(&pdev->dev, master->gpio_data);
	if (master->gpio_trans)
		devm_gpiod_put(&pdev->dev, master->gpio_trans);
	if (master->gpio_enable)
		devm_gpiod_put(&pdev->dev, master->gpio_enable);
	if (master->gpio_mux)
		devm_gpiod_put(&pdev->dev, master->gpio_mux);
	fsi_master_unregister(&master->master);

	of_node_put(master->master.dev.of_node);

	return 0;
}

static const struct of_device_id fsi_master_gpio_match[] = {
	{ .compatible = "fsi-master-gpio" },
	{ },
};

static struct platform_driver fsi_master_gpio_driver = {
	.driver = {
		.name		= "fsi-master-gpio",
		.of_match_table	= fsi_master_gpio_match,
	},
	.probe	= fsi_master_gpio_probe,
	.remove = fsi_master_gpio_remove,
};

module_platform_driver(fsi_master_gpio_driver);
MODULE_LICENSE("GPL");