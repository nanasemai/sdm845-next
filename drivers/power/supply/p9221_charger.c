// SPDX-License-Identifier: GPL-2.0-only
/*
 * IDT P9221 Qi wireless power receiver driver
 *
 * Copyright (C) 2026 David Heidelberg <david@ixit.cz>
 *
 * Register map above 0x38 is the one used by the customer-specific
 * (customer ID 5) firmware found on Google Pixel 3 devices, taken from the
 * downstream p9221_charger driver. It does not match the public P9221-R
 * datasheet, which only agrees up to the interrupt enable register.
 */

#include <linux/bits.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_wakeirq.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define P9221_CHIP_ID_REG		0x00	/* u16 */
#define P9221_CHIP_ID			0x9220
#define P9221_CUSTOMER_ID_REG		0x03
#define P9221_CUSTOMER_ID_R5		0x05
#define P9221_INT_REG			0x36	/* u16 */
#define P9221_INT_ENABLE_REG		0x38
#define P9221_INT_CLEAR_REG		0x3a	/* u16 */
#define P9221_ILIM_SET_REG		0x3d	/* 100 mA steps, 200 mA offset */
#define P9221_VOUT_REG			0x42	/* u16, mV */
#define P9221_IOUT_REG			0x44	/* u16, mA */
#define P9221_SYSTEM_MODE_REG		0x4c
#define P9221_SYSTEM_MODE_EPP		BIT(3)
#define P9221_COM_REG			0x4e
#define P9221_COM_CLEAR_INT		BIT(5)
#define P9221_FOD_REG			0x70
#define P9221_FOD_SIZE			16
#define P9221_LAST_REG			0x1ff

#define P9221_INT_VRECTON		BIT(6)
#define P9221_INT_UV			BIT(3)
#define P9221_INT_OVT			BIT(2)
#define P9221_INT_OVV			BIT(1)
#define P9221_INT_OVC			BIT(0)
#define P9221_INT_FAULT			(P9221_INT_UV | P9221_INT_OVT | \
					 P9221_INT_OVV | P9221_INT_OVC)
#define P9221_INT_ENABLE_MASK		(P9221_INT_VRECTON | P9221_INT_FAULT)

#define P9221_ILIM_SET_1600MA		0x0e

/*
 * The receiver is only reachable while it sits on a pad, and neither of the
 * interrupts reliably reports it leaving, so presence is re-checked at this
 * interval while online. It is also the delay after which the mode dependent
 * settings are applied a second time: the system mode register is only valid
 * once the transmitter finished negotiation, which the vendor driver observed
 * to take up to 2s after VRECTON.
 */
#define P9221_CHECK_MS			2000

struct p9221 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work check_work;
	struct mutex lock;		/* serializes presence checks and configuration */
	u8 fod_bpp[P9221_FOD_SIZE];
	u8 fod_epp[P9221_FOD_SIZE];
	int fod_bpp_len;
	int fod_epp_len;
	unsigned int configure_left;
	bool online;
	bool fw_warned;
};

static int p9221_read16(struct p9221 *p, unsigned int reg, u16 *val)
{
	__le16 buf;
	int ret;

	ret = regmap_bulk_read(p->regmap, reg, &buf, sizeof(buf));
	if (ret)
		return ret;

	*val = le16_to_cpu(buf);
	return 0;
}

static int p9221_write16(struct p9221 *p, unsigned int reg, u16 val)
{
	__le16 buf = cpu_to_le16(val);

	return regmap_bulk_write(p->regmap, reg, &buf, sizeof(buf));
}

static int p9221_clear_interrupts(struct p9221 *p, u16 mask)
{
	unsigned int com;
	int ret;

	ret = p9221_write16(p, P9221_INT_CLEAR_REG, mask);
	if (ret)
		return ret;

	/* The firmware clears the command register once it processed it */
	ret = regmap_read_poll_timeout(p->regmap, P9221_COM_REG, com, !com,
				       10000, 150000);
	if (ret)
		return ret;

	return regmap_write(p->regmap, P9221_COM_REG, P9221_COM_CLEAR_INT);
}

/* Apply the settings that depend on the negotiated power profile */
static int p9221_configure(struct p9221 *p)
{
	unsigned int mode, cust_id;
	const u8 *fod = p->fod_bpp;
	int fod_len = p->fod_bpp_len;
	const char *step;
	int ret;

	step = "customer id";
	ret = regmap_read(p->regmap, P9221_CUSTOMER_ID_REG, &cust_id);
	if (ret)
		goto err;

	if (cust_id != P9221_CUSTOMER_ID_R5) {
		if (!p->fw_warned)
			dev_warn(p->dev, "unsupported firmware (customer id %#x), not configuring\n",
				 cust_id);
		p->fw_warned = true;
		return 0;
	}

	step = "system mode";
	ret = regmap_read(p->regmap, P9221_SYSTEM_MODE_REG, &mode);
	if (ret)
		goto err;

	if ((mode & P9221_SYSTEM_MODE_EPP) && p->fod_epp_len) {
		fod = p->fod_epp;
		fod_len = p->fod_epp_len;
	}

	if (fod_len) {
		step = "fod";
		ret = regmap_bulk_write(p->regmap, P9221_FOD_REG, fod, fod_len);
		if (ret)
			goto err;
	}

	step = "ilim";
	ret = regmap_write(p->regmap, P9221_ILIM_SET_REG, P9221_ILIM_SET_1600MA);
	if (ret)
		goto err;

	step = "clear interrupts";
	ret = p9221_clear_interrupts(p, P9221_INT_ENABLE_MASK);
	if (ret)
		goto err;

	step = "interrupt enable";
	ret = regmap_write(p->regmap, P9221_INT_ENABLE_REG,
			   P9221_INT_ENABLE_MASK);
	if (ret)
		goto err;

	dev_dbg(p->dev, "configured, mode %#x\n", mode);
	return 0;

err:
	dev_err(p->dev, "failed to configure (%s): %d\n", step, ret);
	return ret;
}

/*
 * The receiver is powered from the coil, so it is present exactly when it
 * answers on I2C. @arrived is set when the receiver signalled that its
 * rectifier came up, which also happens when it was reset while on the pad,
 * so configuration starts over.
 */
static void p9221_update(struct p9221 *p, bool arrived)
{
	bool present;
	u16 id;

	guard(mutex)(&p->lock);

	if (arrived)
		p->configure_left = 2;

	present = !p9221_read16(p, P9221_CHIP_ID_REG, &id) && id == P9221_CHIP_ID;

	if (present) {
		/* Once on arrival and once more after negotiation settled */
		if (p->configure_left && !p9221_configure(p))
			p->configure_left--;
		mod_delayed_work(system_wq, &p->check_work,
				 msecs_to_jiffies(P9221_CHECK_MS));
	} else {
		p->configure_left = 2;
	}

	if (present != p->online) {
		p->online = present;
		dev_info(p->dev, "%s\n", present ? "online" : "offline");
		power_supply_changed(p->psy);
	}
}

static void p9221_check_work(struct work_struct *work)
{
	struct p9221 *p = container_of(work, struct p9221, check_work.work);

	p9221_update(p, false);
}

static irqreturn_t p9221_irq(int irq, void *data)
{
	struct p9221 *p = data;
	u16 src;
	int ret;

	ret = p9221_read16(p, P9221_INT_REG, &src);
	if (ret || !src)
		return IRQ_HANDLED;

	ret = p9221_clear_interrupts(p, src);
	if (ret)
		dev_err(p->dev, "failed to clear interrupts: %d\n", ret);

	if (src & P9221_INT_FAULT)
		dev_warn(p->dev, "fault interrupt %#x\n", src & P9221_INT_FAULT);

	if (src & P9221_INT_VRECTON)
		p9221_update(p, true);

	return IRQ_HANDLED;
}

static irqreturn_t p9221_pg_irq(int irq, void *data)
{
	p9221_update(data, false);

	return IRQ_HANDLED;
}

static int p9221_get_milli(struct p9221 *p, unsigned int reg, int *uval)
{
	u16 val;
	int ret;

	if (!p->online)
		return -ENODATA;

	ret = p9221_read16(p, reg, &val);
	if (ret)
		return ret;

	*uval = val * 1000;
	return 0;
}

static int p9221_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct p9221 *p = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = p->online;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return p9221_get_milli(p, P9221_VOUT_REG, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return p9221_get_milli(p, P9221_IOUT_REG, &val->intval);
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property p9221_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static const struct power_supply_desc p9221_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = p9221_properties,
	.num_properties = ARRAY_SIZE(p9221_properties),
	.get_property = p9221_get_property,
};

static const struct regmap_config p9221_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = P9221_LAST_REG,
};

static int p9221_read_fod(struct device *dev, const char *name, u8 *fod)
{
	int len;

	len = device_property_count_u8(dev, name);
	if (len == -EINVAL)
		return 0;
	if (len < 0)
		return len;
	if (len > P9221_FOD_SIZE)
		return -EINVAL;

	return device_property_read_u8_array(dev, name, fod, len) ?: len;
}

static int p9221_request_irq(struct p9221 *p, const char *name,
			     irq_handler_t handler)
{
	int irq;

	irq = fwnode_irq_get_byname(dev_fwnode(p->dev), name);
	if (irq < 0)
		return dev_err_probe(p->dev, irq, "failed to get %s irq\n", name);

	return devm_request_threaded_irq(p->dev, irq, NULL, handler,
					 IRQF_ONESHOT, name, p) ?: irq;
}

static int p9221_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	struct gpio_desc *enable;
	struct p9221 *p;
	int ret, irq;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->dev = dev;
	i2c_set_clientdata(client, p);

	ret = devm_mutex_init(dev, &p->lock);
	if (ret)
		return ret;

	p->regmap = devm_regmap_init_i2c(client, &p9221_regmap_config);
	if (IS_ERR(p->regmap))
		return dev_err_probe(dev, PTR_ERR(p->regmap),
				     "failed to init regmap\n");

	enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(enable))
		return dev_err_probe(dev, PTR_ERR(enable),
				     "failed to get enable gpio\n");

	ret = p9221_read_fod(dev, "idt,fod-bpp", p->fod_bpp);
	if (ret < 0)
		return dev_err_probe(dev, ret, "invalid idt,fod-bpp\n");
	p->fod_bpp_len = ret;

	ret = p9221_read_fod(dev, "idt,fod-epp", p->fod_epp);
	if (ret < 0)
		return dev_err_probe(dev, ret, "invalid idt,fod-epp\n");
	p->fod_epp_len = ret;

	ret = devm_delayed_work_autocancel(dev, &p->check_work,
					   p9221_check_work);
	if (ret)
		return ret;

	psy_cfg.drv_data = p;
	psy_cfg.fwnode = dev_fwnode(dev);
	p->psy = devm_power_supply_register(dev, &p9221_psy_desc, &psy_cfg);
	if (IS_ERR(p->psy))
		return dev_err_probe(dev, PTR_ERR(p->psy),
				     "failed to register power supply\n");

	ret = p9221_request_irq(p, "int", p9221_irq);
	if (ret < 0)
		return ret;

	irq = p9221_request_irq(p, "pg", p9221_pg_irq);
	if (irq < 0)
		return irq;

	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	ret = devm_pm_set_wake_irq(dev, irq);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set wake irq\n");

	p9221_update(p, true);

	return 0;
}

static const struct of_device_id p9221_of_match[] = {
	{ .compatible = "idt,p9221" },
	{ }
};
MODULE_DEVICE_TABLE(of, p9221_of_match);

static struct i2c_driver p9221_driver = {
	.driver = {
		.name = "p9221",
		.of_match_table = p9221_of_match,
	},
	.probe = p9221_probe,
};
module_i2c_driver(p9221_driver);

MODULE_AUTHOR("David Heidelberg <david@ixit.cz>");
MODULE_DESCRIPTION("IDT P9221 wireless power receiver driver");
MODULE_LICENSE("GPL");
