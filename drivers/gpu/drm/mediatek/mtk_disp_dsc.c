// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The chopin mainline port
 *
 * MediaTek MT6893 display DSC (Display Stream Compression) wrap.
 *
 * Every value programmed here was read back from the bootloader's live, working
 * instance on this very board (see port/DSC-REFERENCE.md): the LK-configured DSC
 * drives the k10a panel in DSC mode (DSI PSCTRL reads PS_SEL 5, PS_WC 1080,
 * SIZE_CON width 360 = ps_wc/3) and that is the golden reference we replicate.
 *
 * Note two MT6893 specifics that differ from the MT6895 code:
 *   - ENC_WIDTH (0x3c) is zero here, i.e. unused on this SoC;
 *   - CFG (0x34) holds 0x22, the vendor's "dsc_cfg == 34" branch, not 0x0828.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>


#include <drm/drm_print.h>


#include "mtk_ddp_comp.h"

/* Register offsets, verified against the live instance */
#define DSC_CON			0x000
#define DSC_CON_EN		BIT(0)
#define DSC_CON_BYPASS		BIT(4)
#define DSC_CON_UFOE_SEL	BIT(16)
#define DSC_PIC_W		0x018
#define DSC_PIC_H		0x01c
#define DSC_SLICE_W		0x020
#define DSC_SLICE_H		0x024
#define DSC_CHUNK_SIZE		0x028
#define DSC_BUF_SIZE		0x02c
#define DSC_MODE		0x030
#define DSC_MODE_SLICE_MODE	GENMASK(1, 0)
#define DSC_MODE_RGB_SWAP	BIT(2)
#define DSC_CFG			0x034
#define DSC_PAD			0x038
#define DSC_ENC_WIDTH		0x03c
#define DSC_OBUF		0x070
#define DSC_SHADOW		0x200
#define DSC_PPS0		0x080
#define DSC_PPS_NR		20

/*
 * Geometry of the k10a panel (and of the same-generation l16 we cross-checked
 * against): two slices of 540x20, 8 bits per component / 8 bits per pixel.
 */
#define DSC_SLICE_W_VAL		540
#define DSC_SLICE_H_VAL		20
#define DSC_SLICES		2
#define DSC_BPP			8

struct mtk_disp_dsc {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
};

static void mtk_dsc_write(void __iomem *regs, u32 off, u32 val)
{
	writel(val, regs + off);
}

/*
 * Program the encoder for @width x @height and hand it @pps (DSC_PPS_NR words,
 * exactly as the live instance holds them).
 */
static void mtk_dsc_config(struct mtk_disp_dsc *dsc, unsigned int width,
			   unsigned int height, const u32 *pps)
{
	void __iomem *regs = dsc->regs;
	unsigned int i;

	/* PIC_W = ((width / 3) - 1) << 16 | width  (live: 0x01670438) */
	mtk_dsc_write(regs, DSC_PIC_W,
		      (((width / 3) - 1) << 16) | width);
	/* PIC_H = (height - 1) in both halves (live: 0x095f095f) */
	mtk_dsc_write(regs, DSC_PIC_H, ((height - 1) << 16) | (height - 1));
	/* SLICE_W = ((slice_w / 3) - 1) << 16 | slice_w  (live: 0x00b3021c) */
	mtk_dsc_write(regs, DSC_SLICE_W,
		      (((DSC_SLICE_W_VAL / 3) - 1) << 16) | DSC_SLICE_W_VAL);
	/* SLICE_H: slice_height - 1 in the low half (live: 0x00770013) */
	mtk_dsc_write(regs, DSC_SLICE_H, DSC_SLICE_H_VAL - 1);
	/* CHUNK_SIZE = slice width; BUF_SIZE = chunk * slice height */
	mtk_dsc_write(regs, DSC_CHUNK_SIZE, DSC_SLICE_W_VAL);
	mtk_dsc_write(regs, DSC_BUF_SIZE, DSC_SLICE_W_VAL * DSC_SLICE_H_VAL);
	/* MODE: one bit per extra slice (two slices -> 1) */
	mtk_dsc_write(regs, DSC_MODE, (DSC_SLICES - 1) & DSC_MODE_SLICE_MODE);
	/* CFG: the vendor's dsc_cfg == 34 branch, as read back (0x22) */
	mtk_dsc_write(regs, DSC_CFG, 0x22);
	mtk_dsc_write(regs, DSC_PAD, 0);
	/* ENC_WIDTH is unused on MT6893 (live: 0) */
	mtk_dsc_write(regs, DSC_ENC_WIDTH, 0);

	for (i = 0; i < DSC_PPS_NR; i++)
		mtk_dsc_write(regs, DSC_PPS0 + i * 4, pps[i]);

	/* Shadow block: 0x23 in the live instance (ver 2 -> 0x40 handling does
	 * not apply here, so take the value verbatim). */
	mtk_dsc_write(regs, DSC_SHADOW, 0x23);
	mtk_dsc_write(regs, DSC_OBUF, 0x55d);

	dev_info(dsc->dev,
		 "dsc configured: %ux%u, %u slices of %ux%u, %u bpp\n",
		 width, height, DSC_SLICES, DSC_SLICE_W_VAL, DSC_SLICE_H_VAL,
		 DSC_BPP);
}

static void mtk_dsc_start(struct mtk_disp_dsc *dsc)
{
	u32 con = readl(dsc->regs + DSC_CON);

	writel(con | DSC_CON_EN, dsc->regs + DSC_CON);
	dev_dbg(dsc->dev, "dsc CON=0x%08x\n", readl(dsc->regs + DSC_CON));
}

static void mtk_dsc_stop(struct mtk_disp_dsc *dsc)
{
	u32 con = readl(dsc->regs + DSC_CON);

	writel(con & ~DSC_CON_EN, dsc->regs + DSC_CON);
}

static int mtk_disp_dsc_probe(struct platform_device *pdev)
{
	struct mtk_disp_dsc *dsc;
	int ret;

	dsc = devm_kzalloc(&pdev->dev, sizeof(*dsc), GFP_KERNEL);
	if (!dsc)
		return -ENOMEM;

	dsc->dev = &pdev->dev;
	dsc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsc->regs))
		return dev_err_probe(dsc->dev, PTR_ERR(dsc->regs),
				     "cannot map DSC registers\n");

	dsc->clk = devm_clk_get(dsc->dev, NULL);
	if (IS_ERR(dsc->clk))
		return dev_err_probe(dsc->dev, PTR_ERR(dsc->clk),
				     "cannot get DSC clock\n");

	ret = clk_prepare_enable(dsc->clk);
	if (ret)
		return dev_err_probe(dsc->dev, ret, "cannot enable DSC clock\n");

	platform_set_drvdata(pdev, dsc);
	pm_runtime_enable(dsc->dev);

	dev_info(dsc->dev, "probed: regs mapped, clock enabled\n");
	return 0;
}

static void mtk_disp_dsc_remove(struct platform_device *pdev)
{
	struct mtk_disp_dsc *dsc = platform_get_drvdata(pdev);

	pm_runtime_disable(dsc->dev);
	clk_disable_unprepare(dsc->clk);
}

static const struct of_device_id mtk_disp_dsc_driver_dt_match[] = {
	{ .compatible = "mediatek,mt6893-disp-dsc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_disp_dsc_driver_dt_match);

struct platform_driver mtk_disp_dsc_driver = {
	.probe = mtk_disp_dsc_probe,
	.remove = mtk_disp_dsc_remove,
	.driver = {
		.name = "mediatek-disp-dsc",
		.of_match_table = mtk_disp_dsc_driver_dt_match,
	},
};
