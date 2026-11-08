// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal MT6893 MTCMOS clock provider for the MFG (GPU) power islands.
 *
 * The vendor scpsys@10001000 node exposes every MTCMOS power island as a
 * clock.  Mainline has no MT6893 provider, so the MFG (GPU) subset is
 * implemented for real here and the remaining indices are exposed as no-op
 * placeholders.
 *
 * The placeholders matter: the display node asks for
 *   clocks = <&scpsys 20>, <&mmsys 54>, <&mmsys 20>
 * and with num == 9 that first lookup returned
 *   of_clk_hw_onecell_get: invalid index 20
 * so mtk_drm_get_top_clk() failed, the display top clocks were never
 * programmed, and at 1080x2400@120 Hz the RDMA/OVL FIFO underflowed under a
 * heavier compositor (mutter) - visible as torn/garbled scanout.
 *
 * A no-op is safe for the indices we do not model: the display is already
 * running on whatever power state the bootloader left, and the point is only
 * that clk_get()/prepare_enable() must not fail.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#define SPM_POWERON_CONFIG_EN	0x0000
#define SPM_MFG0_PWR_CON	0x0308
#define SPM_MFG_PWR_CON_STEP	0x0004

#define SPM_PROJECT_CODE	0xb16
#define PWR_ON			BIT(2)
#define PWR_ON_2ND		BIT(3)
#define PWR_RST_B		BIT(0)
#define PWR_ISO			BIT(1)
#define PWR_CLK_DIS		BIT(4)
#define PWR_SRAM_PDN		BIT(8)

#define MT6893_MTCMOS_MFG0	2
#define MT6893_MTCMOS_MFG6	8
/* Highest index any DT consumer may ask for (vendor numbering has ~35). */
#define MT6893_MTCMOS_MAX	40

struct mt6893_pg {
	void __iomem *spm;
	struct clk_hw_onecell_data *clk_data;
};

static int mt6893_pg_enable(struct clk_hw *hw)
{
	struct clk_hw *parent = clk_hw_get_parent(hw);
	struct mt6893_pg *pg = dev_get_drvdata(clk_hw_get_dev(hw));
	unsigned long id = clk_hw_get_flags(hw); /* unused, kept simple */
	u32 ctl;

	(void)parent;
	(void)id;

	/* The clock index is encoded in the clock name suffix. */
	if (strstr(clk_hw_get_name(hw), "async"))
		ctl = SPM_MFG0_PWR_CON;
	else if (strstr(clk_hw_get_name(hw), "core7_8"))
		ctl = SPM_MFG0_PWR_CON + 6 * SPM_MFG_PWR_CON_STEP;
	else if (strstr(clk_hw_get_name(hw), "core5_6"))
		ctl = SPM_MFG0_PWR_CON + 5 * SPM_MFG_PWR_CON_STEP;
	else if (strstr(clk_hw_get_name(hw), "core3_4"))
		ctl = SPM_MFG0_PWR_CON + 4 * SPM_MFG_PWR_CON_STEP;
	else if (strstr(clk_hw_get_name(hw), "core1_2"))
		ctl = SPM_MFG0_PWR_CON + 3 * SPM_MFG_PWR_CON_STEP;
	else if (strstr(clk_hw_get_name(hw), "core0"))
		ctl = SPM_MFG0_PWR_CON + 2 * SPM_MFG_PWR_CON_STEP;
	else
		ctl = SPM_MFG0_PWR_CON + 1 * SPM_MFG_PWR_CON_STEP;

	writel((SPM_PROJECT_CODE << 16) | 0x1, pg->spm + SPM_POWERON_CONFIG_EN);
	writel(readl(pg->spm + ctl) | PWR_ON | PWR_ON_2ND, pg->spm + ctl);
	mdelay(30);

	/* Release the island: clock on, isolation off, reset deasserted,
	 * SRAM power-down cleared (vendor mtcmos power-on ordering). */
	writel(readl(pg->spm + ctl) &
	       ~(PWR_CLK_DIS | PWR_ISO | PWR_SRAM_PDN), pg->spm + ctl);
	writel(readl(pg->spm + ctl) | PWR_RST_B, pg->spm + ctl);
	mdelay(30);
	return 0;
}

static void mt6893_pg_disable(struct clk_hw *hw)
{
	/* Keep the islands on; vendor gpufreq also leaves the rails up. */
}

static int mt6893_pg_is_enabled(struct clk_hw *hw)
{
	return 1;
}

static int mt6893_pg_stub_enable(struct clk_hw *hw)
{
	return 0;
}

static void mt6893_pg_stub_disable(struct clk_hw *hw)
{
}

static int mt6893_pg_stub_is_enabled(struct clk_hw *hw)
{
	return 1;
}

static const struct clk_ops mt6893_pg_stub_ops = {
	.enable = mt6893_pg_stub_enable,
	.disable = mt6893_pg_stub_disable,
	.is_enabled = mt6893_pg_stub_is_enabled,
};

static const struct clk_ops mt6893_pg_ops = {
	.enable = mt6893_pg_enable,
	.disable = mt6893_pg_disable,
	.is_enabled = mt6893_pg_is_enabled,
};

static const char * const mt6893_pg_names[MT6893_MTCMOS_MAX] = {
	[MT6893_MTCMOS_MFG0] = "mtcmos_mfg_async",
	[3] = "mtcmos_mfg",
	[4] = "mtcmos_mfg_core0",
	[5] = "mtcmos_mfg_core1_2",
	[6] = "mtcmos_mfg_core3_4",
	[7] = "mtcmos_mfg_core5_6",
	[8] = "mtcmos_mfg_core7_8",
};

static int mt6893_pg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6893_pg *pg;
	int i, ret;

	pg = devm_kzalloc(dev, sizeof(*pg), GFP_KERNEL);
	if (!pg)
		return -ENOMEM;

	/* scpsys reg order: infracfg, spm, topckgen */
	pg->spm = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(pg->spm))
		return PTR_ERR(pg->spm);

	pg->clk_data = devm_kzalloc(dev, struct_size(pg->clk_data, hws,
						     MT6893_MTCMOS_MAX),
				    GFP_KERNEL);
	if (!pg->clk_data)
		return -ENOMEM;
	pg->clk_data->num = MT6893_MTCMOS_MAX;

	for (i = 0; i < MT6893_MTCMOS_MAX; i++) {
		bool real = (i >= MT6893_MTCMOS_MFG0 &&
			     i <= MT6893_MTCMOS_MFG6 &&
			     mt6893_pg_names[i]);
		struct clk_init_data init = {
			.ops = real ? &mt6893_pg_ops : &mt6893_pg_stub_ops,
			.flags = CLK_IGNORE_UNUSED,
		};
		struct clk_hw *hw;

		if (real)
			init.name = mt6893_pg_names[i];
		else
			init.name = devm_kasprintf(dev, GFP_KERNEL,
						   "mtcmos_stub%d", i);
		if (!init.name)
			return -ENOMEM;

		hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
		if (!hw)
			return -ENOMEM;
		hw->init = &init;

		ret = devm_clk_hw_register(dev, hw);
		if (ret)
			return ret;
		pg->clk_data->hws[i] = hw;
	}

	dev_info(dev, "MTCMOS clocks: %d real (MFG), %d placeholders\n",
		 MT6893_MTCMOS_MFG6 - MT6893_MTCMOS_MFG0 + 1,
		 MT6893_MTCMOS_MAX - (MT6893_MTCMOS_MFG6 - MT6893_MTCMOS_MFG0 + 1));

	platform_set_drvdata(pdev, pg);
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   pg->clk_data);
}

static const struct of_device_id mt6893_pg_of_match[] = {
	{ .compatible = "mediatek,mt6893-scpsys-clk" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6893_pg_of_match);

static struct platform_driver mt6893_pg_driver = {
	.driver = {
		.name = "mt6893-scpsys-clk",
		.of_match_table = mt6893_pg_of_match,
	},
	.probe = mt6893_pg_probe,
};
module_platform_driver(mt6893_pg_driver);

MODULE_DESCRIPTION("MT6893 MFG MTCMOS clock provider");
MODULE_LICENSE("GPL");
