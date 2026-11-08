// SPDX-License-Identifier: GPL-2.0-only
/*
 * WL2866D regulator driver for the chopin port
 *
 * The panel's DVDD comes from one of these: on this board the panel node's
 * dvdd-supply points at on,wl2866d1@1, that is the first WL2866D's LDO 1,
 * programmed to 1.2 V.  Two chips sit at the same I2C address 0x28 on
 * different buses - wl2866d1 on i2c7, wl2866d2 on i2c9 - and both name their
 * four LDOs identically, so the panel is tied to its supply by phandle rather
 * than by name.
 *
 * Transcribed from the vendor driver at
 * n10p/vendor/kernel_mt6893-chopin_kvm/drivers/regulator/wl2866d.c: the
 * register layout, the voltage arithmetic and the enable semantics are its.
 * Dropped: the ~400 lines of camera-LDO character device (the camera_ldo class
 * and wl2866d_dev_list), which nothing here uses; the optional parent supply,
 * which no node on this board declares; and the min-dropout/iout bookkeeping,
 * which only fed regulator constraints we do not read.
 *
 * The descriptor reports n_voltages = 1, as the vendor's does: the framework
 * only ever enables and disables these rails.  The voltage is written once, at
 * probe, from the node's init-voltage.
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define WL2866D_REG_ENABLE	0x0e
#define WL2866D_REG_LDO0	0x03

#define WL2866D_LDO_VSET_REG(offset)	((offset) + WL2866D_REG_LDO0)

/* dvdd LDOs are offsets 0 and 1, avdd ones 2 and 3; the vendor's arithmetic */
#define WL2866D_VSET_DVDD_BASE_UV	600000
#define WL2866D_VSET_DVDD_STEP_UV	6000
#define WL2866D_VSET_AVDD_BASE_UV	1200000
#define WL2866D_VSET_AVDD_STEP_UV	12500

#define WL2866D_MAX_LDO			4

struct wl2866d_ldo {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_dev *rdev;
	unsigned int index;	/* the chip's index property */
	u16 offset;		/* which LDO: 0..3 */
	char name[32];
};

/*
 * The vendor refuses the first disable of the panel's own supply, and this
 * keeps that.  Nothing claims the rail at that point - the panel driver looks
 * up vrf18, which this board does not have - so the core's late cleanup would
 * otherwise switch off a rail the bootloader had already brought up, out from
 * under a live panel.
 *
 * Note the vendor guards this under the k10a panel's config but tests for
 * index == 1, while this board's panel node takes its dvdd-supply from the
 * index 0 chip.  The device tree is the authority here, so the test is index 0
 * and offset 1.  regulator_ignore_unused is also on the command line, which
 * covers the same ground.
 */
static bool wl2866d_panel_dvdd_kept;

static int wl2866d_ldo_enable(struct regulator_dev *rdev)
{
	struct wl2866d_ldo *ldo = rdev_get_drvdata(rdev);

	int ret = regmap_update_bits(ldo->regmap, WL2866D_REG_ENABLE,
				     1u << ldo->offset, 1u << ldo->offset);

	/*
	 * Report it.  This is the panel's supply coming on, and until now the
	 * only evidence that it had was that the code looked right: the panel
	 * driver writes nothing when it enables a regulator.
	 */
	if (!ret)
		dev_info(ldo->dev, "%s: on\n", ldo->name);
	return ret;
}

static int wl2866d_ldo_disable(struct regulator_dev *rdev)
{
	struct wl2866d_ldo *ldo = rdev_get_drvdata(rdev);

	if (!wl2866d_panel_dvdd_kept && ldo->index == 0 && ldo->offset == 1 &&
	    !strcmp(ldo->name, "wl2866d-dvdd2")) {
		wl2866d_panel_dvdd_kept = true;
		dev_info(ldo->dev, "init phase: keeping %s enabled\n", ldo->name);
		return 0;
	}

	int ret = regmap_update_bits(ldo->regmap, WL2866D_REG_ENABLE,
				     1u << ldo->offset, 0);

	/*
	 * The suspend-time power-down ends here, so say so.  Without this the
	 * question "did the panel's supply actually go off" could only be
	 * answered by reading the code, which is how the guard in this file came
	 * to be mistaken for something still holding the rail on.
	 */
	if (!ret)
		dev_info(ldo->dev, "%s: off\n", ldo->name);
	return ret;
}

static int wl2866d_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct wl2866d_ldo *ldo = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(ldo->regmap, WL2866D_REG_ENABLE, &val);
	if (ret)
		return ret;
	return !!(val & (1u << ldo->offset));
}

static int wl2866d_ldo_get_voltage(struct regulator_dev *rdev)
{
	struct wl2866d_ldo *ldo = rdev_get_drvdata(rdev);
	unsigned int vset;
	int ret;

	ret = regmap_read(ldo->regmap, WL2866D_LDO_VSET_REG(ldo->offset), &vset);
	if (ret)
		return ret;

	if (ldo->offset <= 1)
		return WL2866D_VSET_DVDD_BASE_UV + vset * WL2866D_VSET_DVDD_STEP_UV;
	return WL2866D_VSET_AVDD_BASE_UV + vset * WL2866D_VSET_AVDD_STEP_UV;
}

static const struct regulator_ops wl2866d_ldo_ops = {
	.enable = wl2866d_ldo_enable,
	.disable = wl2866d_ldo_disable,
	.is_enabled = wl2866d_ldo_is_enabled,
	.get_voltage = wl2866d_ldo_get_voltage,
};

static int wl2866d_write_init_voltage(struct wl2866d_ldo *ldo, u32 uv)
{
	u32 base, step;
	unsigned int vset;
	int ret;

	if (ldo->offset <= 1) {
		base = WL2866D_VSET_DVDD_BASE_UV;
		step = WL2866D_VSET_DVDD_STEP_UV;
	} else {
		base = WL2866D_VSET_AVDD_BASE_UV;
		step = WL2866D_VSET_AVDD_STEP_UV;
	}
	if (uv <= base)
		return 0;

	vset = DIV_ROUND_UP(uv - base, step);
	ret = regmap_write(ldo->regmap, WL2866D_LDO_VSET_REG(ldo->offset), vset);
	if (ret)
		dev_err(ldo->dev, "%s: cannot set %u uV: %d\n", ldo->name, uv, ret);
	else
		dev_info(ldo->dev, "%s: set to %u uV (vset %u)\n",
			 ldo->name, base + vset * step, vset);
	return ret;
}

static int wl2866d_register_ldo(struct wl2866d_ldo *ldo,
				struct device_node *node)
{
	struct regulator_config config = { };
	struct regulator_init_data *init_data;
	struct regulator_desc *desc;
	const char *name;
	u32 init_uv = 0;
	int ret;

	ret = of_property_read_string(node, "regulator-name", &name);
	if (ret) {
		dev_err(ldo->dev, "%pOF: no regulator-name\n", node);
		return ret;
	}
	strscpy(ldo->name, name, sizeof(ldo->name));

	ret = of_property_read_u16(node, "offset", &ldo->offset);
	if (ret) {
		dev_err(ldo->dev, "%s: no offset property\n", ldo->name);
		return ret;
	}
	if (ldo->offset >= WL2866D_MAX_LDO) {
		dev_err(ldo->dev, "%s: offset %u out of range\n",
			ldo->name, ldo->offset);
		return -EINVAL;
	}

	if (!of_property_read_u32(node, "init-voltage", &init_uv)) {
		ret = wl2866d_write_init_voltage(ldo, init_uv);
		if (ret)
			return ret;
	}

	desc = devm_kzalloc(ldo->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->name = ldo->name;
	desc->owner = THIS_MODULE;
	desc->type = REGULATOR_VOLTAGE;
	desc->ops = &wl2866d_ldo_ops;
	/*
	 * One voltage, as the vendor has it: the framework enables and disables
	 * these rails and gets their voltage, it does not set it.
	 */
	desc->n_voltages = 1;
	/*
	 * fixed_uV has to be the voltage the rail is actually at, not the
	 * arithmetic base.  init-voltage is what the node asks for and what
	 * wl2866d_write_init_voltage has just programmed; the vendor's
	 * regulator-min-microvolt is the fallback.
	 */
	desc->fixed_uV = init_uv ? init_uv : 1200000;

	/*
	 * The core refuses a consumer's regulator_enable() with -EPERM unless
	 * the regulator allows status changes, and this is what that looks
	 * like: the panel attached to the rail (open_count 1) but its enable
	 * was refused, so use_count stayed 0 and the rail was never switched
	 * on by us.  The vendor sets the same two bits.
	 */
	init_data = of_get_regulator_init_data(ldo->dev, node, desc);
	if (!init_data) {
		dev_err(ldo->dev, "%s: no init data\n", ldo->name);
		return -ENODATA;
	}
	init_data->constraints.valid_ops_mask |= REGULATOR_CHANGE_STATUS |
						 REGULATOR_CHANGE_VOLTAGE;

	config.dev = ldo->dev;
	config.init_data = init_data;
	config.driver_data = ldo;
	config.of_node = node;

	ldo->rdev = devm_regulator_register(ldo->dev, desc, &config);
	if (IS_ERR(ldo->rdev)) {
		ret = PTR_ERR(ldo->rdev);
		dev_err(ldo->dev, "%s: cannot register: %d\n", ldo->name, ret);
		return ret;
	}
	dev_info(ldo->dev, "%s (offset %u) registered\n", ldo->name, ldo->offset);
	return 0;
}

static int wl2866d_probe(struct i2c_client *client)
{
	static const struct regmap_config regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	struct device *dev = &client->dev;
	struct device_node *child;
	struct wl2866d_ldo *ldo;
	struct regmap *regmap;
	u32 index = 0;

	regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "cannot init regmap\n");

	of_property_read_u32(dev->of_node, "index", &index);

	for_each_available_child_of_node(dev->of_node, child) {
		int ret;

		ldo = devm_kzalloc(dev, sizeof(*ldo), GFP_KERNEL);
		if (!ldo) {
			of_node_put(child);
			return -ENOMEM;
		}
		ldo->dev = dev;
		ldo->regmap = regmap;
		ldo->index = index;

		ret = wl2866d_register_ldo(ldo, child);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	return 0;
}

static const struct of_device_id wl2866d_of_match[] = {
	{ .compatible = "willsemi,wl2866d1" },
	{ .compatible = "willsemi,wl2866d2" },
	{ }
};
MODULE_DEVICE_TABLE(of, wl2866d_of_match);

static const struct i2c_device_id wl2866d_i2c_id[] = {
	{ "wl2866d-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wl2866d_i2c_id);

static struct i2c_driver wl2866d_driver = {
	.driver = {
		.name = "wl2866d-regulator",
		.of_match_table = wl2866d_of_match,
	},
	.probe = wl2866d_probe,
	.id_table = wl2866d_i2c_id,
};
module_i2c_driver(wl2866d_driver);

MODULE_AUTHOR("chopin port");
MODULE_DESCRIPTION("WL2866D LDO regulators (panel DVDD among them)");
MODULE_LICENSE("GPL");
