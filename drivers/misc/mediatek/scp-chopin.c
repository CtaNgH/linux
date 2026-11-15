// SPDX-License-Identifier: GPL-2.0
/*
 * chopin: controlled SCP bring-up helper.
 *
 * The vendor SCP driver (drivers/misc/mediatek/scp, ported from the MT6895
 * tree) probes the mailbox/IPI/sensor-hub, but its own start path is entangled
 * with DVFS/PMIC/IPI plumbing this board does not have yet.  This helper drives
 * the same hardware sequence step by step through debugfs so every stage can be
 * validated on its own:
 *
 *   audit     - read-only dump of every register that matters (run this first)
 *   fullreset - cleartcm, loadloader, reginfo, dramrestore
 *   start     - the whole vendor boot sequence (see below)
 *
 * "start" mirrors mt6885/scp_awake.c:scp_sys_reset_ws() and
 * mt6885/scp_helper.c:reset_scp() from the stock kernel, in the same order:
 *
 *   1. assert both cores (SW_RSTN_SET)
 *   2. GPR_CORE0_REBOOT = CORE_REBOOT_OK
 *   3. full reset: zero TCM, copy the loader, restore the region info,
 *      restore the firmware DRAM pool from its backup
 *   4. DRAM_RESV_ADDR/SIZE = the SCP *share* window (reserve-memory-scp_share)
 *   5. release core0 (SW_RSTN_CLR)
 *
 * Step 4 is where this board went wrong: the share window is 0x8f000000 (the
 * runtime AP<->SCP buffers carved out of "mediatek,reserve-memory-scp_share"),
 * NOT 0x9fa00000.  0x9fa00000 is mblock-22 "SCP-reserved", i.e. where the
 * bootloader drops scp.img; the firmware copies its own body out of there into
 * TCM, and that range is not the SCP's MPU window for the AP<->SCP buffers.
 * Writing the image base into DRAM_RESV_ADDR leaves the firmware logging into
 * an unprogrammed window, which is consistent with it wedging inside its first
 * printk while holding the log mutex at TCM 0x7d5e0.
 *
 * The region info the bootloader leaves at TCM+4 on this board is:
 *   ap_loader_start        0x9fa00000   ap_loader_size          0x2000
 *   ap_firmware_start      0x9fa02000   ap_firmware_size        0x143c00
 *   ap_dram_start          0x9fc00000   ap_dram_size            0x5a880
 *   ap_dram_backup_start   0x9fcb5800
 * (LK prints exactly this as "scp_region_info->...".)
 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

/* is_scp_ready() / SCP_A_ID, for the debugfs nodes that touch SCP DRAM */
#include "scp/include/scp.h"

/* AP-side register windows */
#define SCP_TCM_BASE		0x10500000
#define SCP_TCM_SIZE		0x180000
#define SCP_CLKREG_BASE		0x10721000
#define SCP_CLKREG_SIZE		0x1000
#define SCP_CORE0_BASE		0x10730000
#define SCP_CORE1_BASE		0x10740000
#define SCP_CORE_SIZE		0x1000
#define SCP_IMAGE_BASE		0x9fa00000	/* mblock-22: LK scp.img */
#define SCP_IMAGE_SIZE		0x500000

/* scp_clkreg */
#define SCP_CLK_MUX		0x0000
#define SCP_CLK_ENABLE		0x0004
#define SCP_CLK_SAFE_ACK	0x0008
#define SCP_CLK_L1_SRAM_PD	0x002c
#define SCP_CLK_CPU0_SRAM_PD	0x0080
#define SCP_CLK_CPU1_SRAM_PD	0x0084
#define SCP_CLK_TCM_TAIL_PD	0x0094
#define SCP_CLK_EN_SYS		BIT(0)	/* 0 = 26M, 1 = univpll */
#define SCP_CLK_EN_HIGH		BIT(1)

/* scp_cfgreg_coreN */
#define SCP_CORE_SW_RSTN_CLR	0x0000	/* write 1: release reset */
#define SCP_CORE_SW_RSTN_SET	0x0004	/* write 1: assert reset */
#define SCP_CORE_DRAM_RESV	0x0044	/* SCP_A_GENERAL_REG1 */
#define SCP_CORE_DRAM_SIZE	0x0048	/* SCP_A_GENERAL_REG2 */
#define SCP_CORE_REBOOT_OK	0x0054
#define SCP_CORE_MON_STATUS	0x0070
#define SCP_CORE_MON_PC		0x0080
#define CORE_REBOOT_OK		0x1

#define SCP_REGION_INFO_OFFSET	4
#define SCP_REGION_INFO_WORDS	13	/* struct scp_region_info_st */

#define SCP_SHARE_BASE_FALLBACK	0x8f000000
#define SCP_SHARE_SIZE_FALLBACK	0x00500000

/* interesting TCM words, from the firmware disassembly */
static const u32 scp_tcm_peek[] = {
	0x000000, 0x000004, 0x000008, 0x00000c, 0x000010, 0x000014,
	0x000018, 0x00001c, 0x000020, 0x000024, 0x000028, 0x00002c,
	0x000030, 0x000034,
	0x001ff0, 0x002000, 0x002004,	/* firmware body (loader copies it) */
	0x07cc14, 0x07d1f0, 0x07d210, 0x07d214, 0x07d218, 0x07d21c,
	0x07d5d0, 0x07d5e0, 0x07d5f0, 0x07d600, 0x07d610, 0x07d620,
	0x09e600, 0x09e604, 0x09e628, 0x09e62c, 0x0ad2e8,
};

/* share offsets worth looking at: the firmware log ring sits near 0x1ca700 */
static const u32 scp_share_peek[] = {
	0x000000, 0x000040, 0x001000, 0x0ca700,
	0x1ca6c0, 0x1ca6e0, 0x1ca6f0, 0x1ca700, 0x1ca710, 0x1ca720,
};

struct scp_dev {
	struct device *dev;
	void __iomem *tcm;
	void __iomem *clkreg;
	void __iomem *core0;
	void __iomem *core1;
	void __iomem *image;
	void __iomem *share;
	phys_addr_t share_phys;
	u32 share_size;
	const char *share_src;
	u32 region_info[SCP_REGION_INFO_WORDS];
	bool release_core1;
	struct dentry *debugfs;
};

static u32 scp_rd(struct scp_dev *s, void __iomem *base, u32 off)
{
	return readl(base + off);
}

static void scp_wd(struct scp_dev *s, void __iomem *base, u32 off, u32 v)
{
	writel(v, base + off);
}

/* ------------------------------------------------------------------ */
/* the SCP share window (reserve-memory-scp_share)                    */
/* ------------------------------------------------------------------ */

/*
 * resv-base sources, best first:
 *   1. an explicit "mediatek,scp-share-base" property;
 *   2. the kernel's reserved_mem table (what RESERVEDMEM_OF_DECLARE filled in,
 *      i.e. the address the allocator really picked for this boot);
 *   3. DRAM_RESV_ADDR as the bootloader left it (sane range only);
 *   4. the known-good chopin default.
 */
static void scp_find_share(struct scp_dev *s)
{
	struct device_node *rm, *child;
	struct resource res;
	u32 prop;
	bool found = false;

	s->share_src = "fallback";
	s->share_phys = SCP_SHARE_BASE_FALLBACK;
	s->share_size = SCP_SHARE_SIZE_FALLBACK;

	if (!of_property_read_u32(s->dev->of_node, "mediatek,scp-share-base",
				  &prop)) {
		s->share_phys = prop;
		s->share_src = "dts property";
		if (!of_property_read_u32(s->dev->of_node,
					  "mediatek,scp-share-size", &prop))
			s->share_size = prop;
		return;
	}

	rm = of_find_node_by_path("/reserved-memory");
	if (rm) {
		for_each_child_of_node(rm, child) {
			struct reserved_mem *rmem;

			if (!of_device_is_compatible(
				    child, "mediatek,reserve-memory-scp_share"))
				continue;

			rmem = of_reserved_mem_lookup(child);
			if (rmem && rmem->base && rmem->size) {
				s->share_phys = rmem->base;
				s->share_size = rmem->size;
				s->share_src = "reserved_mem";
				found = true;
			} else if (!of_address_to_resource(child, 0, &res) &&
				   res.start) {
				s->share_phys = res.start;
				s->share_size = resource_size(&res);
				s->share_src = "reserved-memory reg";
				found = true;
			}
			of_node_put(child);
			break;
		}
		of_node_put(rm);
	}

	/*
	 * Only fall back to the register when nothing else answered: a stale
	 * 0x9fa00000 (what older builds of this helper wrote) must not be
	 * allowed to override the address the allocator really picked.
	 */
	if (!found) {
		u32 cur = scp_rd(s, s->core0, SCP_CORE_DRAM_RESV);
		u32 cursz = scp_rd(s, s->core0, SCP_CORE_DRAM_SIZE);

		if (cur >= 0x40000000 && cur < 0x90000000 && cursz) {
			s->share_phys = cur;
			s->share_size = cursz;
			s->share_src = "DRAM_RESV reg (LK)";
		}
	}
}

/* ------------------------------------------------------------------ */
/* steps                                                              */
/* ------------------------------------------------------------------ */

static void scp_read_region_info(struct scp_dev *s)
{
	int i;

	for (i = 0; i < SCP_REGION_INFO_WORDS; i++)
		s->region_info[i] = readl(s->tcm + SCP_REGION_INFO_OFFSET + i * 4);
}

/* vendor: scp_enable_sram() in mt6885/scp_awake.c */
static void scp_enable_sram(struct scp_dev *s)
{
	u32 bits;

	/* enable one SRAM block at a time, as the vendor does */
	for (bits = 0xffffffff; bits; ) {
		bits >>= 1;
		scp_wd(s, s->clkreg, SCP_CLK_CPU0_SRAM_PD, bits);
		scp_wd(s, s->clkreg, SCP_CLK_CPU1_SRAM_PD, bits);
	}
	scp_wd(s, s->clkreg, SCP_CLK_CPU0_SRAM_PD, 0);
	scp_wd(s, s->clkreg, SCP_CLK_CPU1_SRAM_PD, 0);
	scp_wd(s, s->clkreg, SCP_CLK_L1_SRAM_PD, 0);
	scp_wd(s, s->clkreg, SCP_CLK_TCM_TAIL_PD, 0);
	wmb();
	dev_info(s->dev,
		 "sram on: cpu0pd=%08x cpu1pd=%08x l1pd=%08x tailpd=%08x\n",
		 scp_rd(s, s->clkreg, SCP_CLK_CPU0_SRAM_PD),
		 scp_rd(s, s->clkreg, SCP_CLK_CPU1_SRAM_PD),
		 scp_rd(s, s->clkreg, SCP_CLK_L1_SRAM_PD),
		 scp_rd(s, s->clkreg, SCP_CLK_TCM_TAIL_PD));
}

/* step 1: clear the whole TCM */
static void scp_cleartcm(struct scp_dev *s)
{
	u32 i;

	for (i = 0; i < SCP_TCM_SIZE; i += 4)
		writel(0, s->tcm + i);
	wmb();
	dev_info(s->dev, "TCM cleared\n");
}

/* step 2: copy the loader from the image the bootloader left in DRAM */
static void scp_loadloader(struct scp_dev *s)
{
	u32 base = s->region_info[0];
	u32 size = s->region_info[1];
	u32 i;

	/* the low three bits may carry SCP_DRAM_RESV_EMBED flags */
	base &= ~0x7u;
	if (base < SCP_IMAGE_BASE || base >= SCP_IMAGE_BASE + SCP_IMAGE_SIZE ||
	    !size || size > 0x20000) {
		base = SCP_IMAGE_BASE;
		size = 0x2000;
	}

	for (i = 0; i < size; i += 4)
		writel(readl(s->image + (base - SCP_IMAGE_BASE) + i),
		       s->tcm + i);
	wmb();
	dev_info(s->dev, "loader copied from %08x (%x bytes), tcm[0]=%08x\n",
		 base, size, readl(s->tcm));
}

/* step 3: write the region info saved at probe time */
static void scp_reginfo(struct scp_dev *s)
{
	int i;

	for (i = 0; i < SCP_REGION_INFO_WORDS; i++)
		writel(s->region_info[i], s->tcm + SCP_REGION_INFO_OFFSET + i * 4);
	wmb();
	dev_info(s->dev, "region info written (loader=%08x/%x dram=%08x/%x)\n",
		 s->region_info[0], s->region_info[1],
		 s->region_info[4], s->region_info[5]);
}

/*
 * step 4: restore the firmware DRAM pool from the backup the bootloader made.
 * Vendor: scp_sys_full_reset() in mt6885/scp_awake.c.  The pool is described by
 * the region info (0x9fc00000 / 0x5a880 on this board); it has nothing to do
 * with the share window above.
 */
static void scp_dramrestore(struct scp_dev *s)
{
	u32 start = s->region_info[4];
	u32 size = s->region_info[5];
	u32 backup = s->region_info[6];
	void __iomem *dram, *bck;
	u32 zero, room, i;

	if (!size || size > SCP_IMAGE_SIZE || start < SCP_IMAGE_BASE ||
	    start >= SCP_IMAGE_BASE + SCP_IMAGE_SIZE ||
	    backup < SCP_IMAGE_BASE ||
	    backup >= SCP_IMAGE_BASE + SCP_IMAGE_SIZE) {
		dev_info(s->dev, "dram restore skipped (%08x/%x/%08x)\n",
			 start, size, backup);
		return;
	}

	zero = roundup(size, 1024) * 2;
	room = SCP_IMAGE_SIZE - (start - SCP_IMAGE_BASE);
	if (zero > room)
		zero = room;

	dram = devm_ioremap(s->dev, start, roundup(size, 1024) * 2);
	bck = devm_ioremap(s->dev, backup, size);
	if (!dram || !bck) {
		dev_err(s->dev, "dram restore: ioremap failed\n");
		return;
	}
	for (i = 0; i < zero; i += 4)
		writel(0, dram + i);
	wmb();
	for (i = 0; i < size; i += 4)
		writel(readl(bck + i), dram + i);
	wmb();
	dev_info(s->dev, "dram restored: %08x <- %08x (%x bytes)\n",
		 start, backup, size);
}

/* alternative step 4: zero the firmware DRAM pool instead of restoring it */
static void scp_dramzero(struct scp_dev *s)
{
	u32 start = s->region_info[4];
	u32 size = s->region_info[5];
	void __iomem *dram;
	u32 i;

	if (!size || size > SCP_IMAGE_SIZE || start < SCP_IMAGE_BASE ||
	    start >= SCP_IMAGE_BASE + SCP_IMAGE_SIZE) {
		dev_info(s->dev, "dram zero skipped (%08x/%x)\n", start, size);
		return;
	}
	dram = devm_ioremap(s->dev, start, size);
	if (!dram) {
		dev_err(s->dev, "dram zero: ioremap failed\n");
		return;
	}
	for (i = 0; i < size; i += 4)
		writel(0, dram + i);
	wmb();
	dev_info(s->dev, "dram zeroed: %08x (%x bytes)\n", start, size);
}


/*
 * The whole boot sequence from the stock kernel: assert, parameterise, load,
 * release.  The full reset is folded in so that a single write always starts
 * the firmware from a known state.
 */
static void scp_start(struct scp_dev *s)
{
	u32 clk, pc0, pc1;

	clk = scp_rd(s, s->clkreg, SCP_CLK_ENABLE);
	dev_info(s->dev, "start: clk=%08x mux=%08x resv=%08x/%08x core1=%d\n",
		 clk, scp_rd(s, s->clkreg, SCP_CLK_MUX),
		 scp_rd(s, s->core0, SCP_CORE_DRAM_RESV),
		 scp_rd(s, s->core0, SCP_CORE_DRAM_SIZE), s->release_core1);

	scp_enable_sram(s);

	/* clock: the bootloader leaves the mux alone; only make sure the SCP
	 * domain clock is on.  Do not touch HIGH_EN, that needs a live PLL.
	 */
	scp_wd(s, s->clkreg, SCP_CLK_ENABLE, clk | SCP_CLK_EN_SYS);

	/* 1. hold both cores in reset while we touch TCM/DRAM (as the vendor
	 * does: it asserts core1 unconditionally and only ever releases core0)
	 */
	scp_wd(s, s->core0, SCP_CORE_SW_RSTN_SET, 1);
	scp_wd(s, s->core1, SCP_CORE_SW_RSTN_SET, 1);
	scp_wd(s, s->core0, SCP_CORE_REBOOT_OK, CORE_REBOOT_OK);
	scp_wd(s, s->core1, SCP_CORE_REBOOT_OK, CORE_REBOOT_OK);
	udelay(200);

	/* 2. reload TCM + the firmware DRAM pool */
	scp_cleartcm(s);
	scp_loadloader(s);
	scp_reginfo(s);
	scp_dramrestore(s);

	/* 3. tell the firmware where the AP<->SCP buffers live */
	scp_wd(s, s->core0, SCP_CORE_DRAM_RESV, (u32)s->share_phys);
	scp_wd(s, s->core0, SCP_CORE_DRAM_SIZE, s->share_size);
	wmb();

	/* 4. release (the stock kernel releases core0 only on mt6893) */
	scp_wd(s, s->core0, SCP_CORE_SW_RSTN_CLR, 1);
	if (s->release_core1)
		scp_wd(s, s->core1, SCP_CORE_SW_RSTN_CLR, 1);
	udelay(2000);

	pc0 = scp_rd(s, s->core0, SCP_CORE_MON_PC);
	pc1 = scp_rd(s, s->core1, SCP_CORE_MON_PC);
	dev_info(s->dev,
		 "started: c0 st=%08x pc=%08x | c1 st=%08x pc=%08x | lock=%08x resv=%08x/%08x\n",
		 scp_rd(s, s->core0, SCP_CORE_MON_STATUS), pc0,
		 scp_rd(s, s->core1, SCP_CORE_MON_STATUS), pc1,
		 readl(s->tcm + 0x7d5e0),
		 scp_rd(s, s->core0, SCP_CORE_DRAM_RESV),
		 scp_rd(s, s->core0, SCP_CORE_DRAM_SIZE));
}

/* ------------------------------------------------------------------ */
/* debugfs                                                            */
/* ------------------------------------------------------------------ */

static int scp_status_show(struct seq_file *f, void *unused)
{
	struct scp_dev *s = f->private;
	int i;

	seq_printf(f, "pc0        %08x\n", scp_rd(s, s->core0, SCP_CORE_MON_PC));
	seq_printf(f, "pc1        %08x\n", scp_rd(s, s->core1, SCP_CORE_MON_PC));
	seq_printf(f, "status0    %08x\n",
		   scp_rd(s, s->core0, SCP_CORE_MON_STATUS));
	seq_printf(f, "status1    %08x\n",
		   scp_rd(s, s->core1, SCP_CORE_MON_STATUS));
	seq_printf(f, "rstn0      clr=%08x set=%08x\n",
		   scp_rd(s, s->core0, SCP_CORE_SW_RSTN_CLR),
		   scp_rd(s, s->core0, SCP_CORE_SW_RSTN_SET));
	seq_printf(f, "rstn1      clr=%08x set=%08x\n",
		   scp_rd(s, s->core1, SCP_CORE_SW_RSTN_CLR),
		   scp_rd(s, s->core1, SCP_CORE_SW_RSTN_SET));
	seq_printf(f, "clk        mux=%08x en=%08x ack=%08x\n",
		   scp_rd(s, s->clkreg, SCP_CLK_MUX),
		   scp_rd(s, s->clkreg, SCP_CLK_ENABLE),
		   scp_rd(s, s->clkreg, SCP_CLK_SAFE_ACK));
	seq_printf(f, "sram_pd    cpu0=%08x cpu1=%08x l1=%08x tail=%08x\n",
		   scp_rd(s, s->clkreg, SCP_CLK_CPU0_SRAM_PD),
		   scp_rd(s, s->clkreg, SCP_CLK_CPU1_SRAM_PD),
		   scp_rd(s, s->clkreg, SCP_CLK_L1_SRAM_PD),
		   scp_rd(s, s->clkreg, SCP_CLK_TCM_TAIL_PD));
	seq_printf(f, "resv       addr=%08x size=%08x\n",
		   scp_rd(s, s->core0, SCP_CORE_DRAM_RESV),
		   scp_rd(s, s->core0, SCP_CORE_DRAM_SIZE));
	seq_printf(f, "share      base=%pa size=%08x (%s)\n",
		   &s->share_phys, s->share_size, s->share_src);
	seq_printf(f, "tcm[0]     %08x\n", readl(s->tcm));
	seq_printf(f, "core1rel   %d\n", s->release_core1);
	for (i = 0; i < SCP_REGION_INFO_WORDS; i++)
		seq_printf(f, "ri[%02d]     %08x\n", i, s->region_info[i]);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(scp_status);

/* read-only: everything worth knowing, in one go */
static int scp_audit_show(struct seq_file *f, void *unused)
{
	struct scp_dev *s = f->private;
	phys_addr_t pa;
	u32 i;

	pa = SCP_TCM_BASE;
	seq_printf(f, "== windows\n");
	seq_printf(f, "tcm        %pa/%x\n", &pa, SCP_TCM_SIZE);
	pa = SCP_IMAGE_BASE;
	seq_printf(f, "image      %pa/%x\n", &pa, SCP_IMAGE_SIZE);
	seq_printf(f, "share      %pa/%x (%s)\n",
		   &s->share_phys, s->share_size, s->share_src);

	seq_printf(f, "== clk\n");
	for (i = 0; i <= 0x94; i += 4) {
		switch (i) {
		case SCP_CLK_MUX:
		case SCP_CLK_ENABLE:
		case SCP_CLK_SAFE_ACK:
		case SCP_CLK_L1_SRAM_PD:
		case SCP_CLK_CPU0_SRAM_PD:
		case SCP_CLK_CPU1_SRAM_PD:
		case SCP_CLK_TCM_TAIL_PD:
			break;
		default:
			continue;
		}
		seq_printf(f, "clk[%03x]   %08x\n", i,
			   scp_rd(s, s->clkreg, i));
	}

	seq_printf(f, "== core0 (rstn clr/set, general regs 0x40..0x58, mon)\n");
	for (i = 0; i <= 0x80; i += 4)
		seq_printf(f, "c0[%03x]    %08x\n", i,
			   scp_rd(s, s->core0, i));

	seq_printf(f, "== core1\n");
	seq_printf(f, "c1[000]    %08x\n",
		   scp_rd(s, s->core1, SCP_CORE_SW_RSTN_CLR));
	seq_printf(f, "c1[004]    %08x\n",
		   scp_rd(s, s->core1, SCP_CORE_SW_RSTN_SET));
	seq_printf(f, "c1[054]    %08x\n",
		   scp_rd(s, s->core1, SCP_CORE_REBOOT_OK));
	seq_printf(f, "c1[070]    %08x\n",
		   scp_rd(s, s->core1, SCP_CORE_MON_STATUS));
	seq_printf(f, "c1[080]    %08x\n",
		   scp_rd(s, s->core1, SCP_CORE_MON_PC));

	seq_printf(f, "== tcm\n");
	for (i = 0; i < ARRAY_SIZE(scp_tcm_peek); i++)
		seq_printf(f, "tcm[%06x] %08x\n", scp_tcm_peek[i],
			   readl(s->tcm + scp_tcm_peek[i]));

	seq_printf(f, "== share (the firmware log ring lives here)\n");
	if (s->share) {
		for (i = 0; i < ARRAY_SIZE(scp_share_peek); i++) {
			u32 off = scp_share_peek[i];

			if (off + 16 > s->share_size)
				continue;
			seq_printf(f, "dr[%08x] %08x %08x %08x %08x\n", off,
				   readl(s->share + off),
				   readl(s->share + off + 4),
				   readl(s->share + off + 8),
				   readl(s->share + off + 12));
		}
	} else {
		seq_printf(f, "(share window not mapped)\n");
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(scp_audit);

#define SCP_DEBUGFS_ACTION(name, fn)					\
static int scp_##name##_set(void *data, u64 val)			\
{									\
	if (val)							\
		fn(data);						\
	return 0;							\
}									\
DEFINE_DEBUGFS_ATTRIBUTE(scp_##name##_fops, NULL, scp_##name##_set, "%llu\n")

/* one-shot: the whole vendor sequence */
SCP_DEBUGFS_ACTION(start, scp_start);

static int scp_fullreset_set(void *data, u64 val)
{
	struct scp_dev *s = data;

	if (!val)
		return 0;
	scp_cleartcm(s);
	scp_loadloader(s);
	scp_reginfo(s);
	scp_dramrestore(s);
	return 0;
}
/*
 * Patch the SCP image copy that LK left in DRAM (0x9fa00000).  The image on
 * the scp_a partition is RSA-signed by MediaTek and verified by LK, so the
 * only place a patch can live is this runtime copy: run "patchscp", then
 * "start" (whose loader copies 0x9fa02000 into TCM without re-checking).
 * Offsets are scp.img file offsets minus 0x200.
 */
static int scp_patchscp_set(void *data, u64 val)
{
	/*
	 * val selects the patch group:
	 *   1 = enable DRDY + force FIFO stream
	 *   2 = the above plus bypass the FSM gate so the accel powers on
	 *   3 = all of the above plus point the sample drain at our DRAM FIFO
	 *   4 = all of the above plus the chip-side INT1 polarity change
	 * Offsets are scp.img file offsets minus 0x200 (the DRAM code base).
	 */
	static const struct {
		u32 off;
		u8 old[4];
		u8 new[4];
		u8 len;
		u8 group;	/* 0 = base, 1 = power gate, 2 = polarity */
	} p[] = {
		{ 0x26EB2, { 0x93, 0xe5, 0x65, 0x00 }, { 0x93, 0xe5, 0xe5, 0x00 }, 4, 0 },
		{ 0x286DE, { 0x11, 0xc5, 0x00, 0x00 }, { 0x01, 0x00, 0x00, 0x00 }, 2, 0 },
		{ 0x23944, { 0x63, 0x9e, 0x25, 0x05 }, { 0x13, 0x00, 0x00, 0x00 }, 4, 1 },
		/* boot-time AccPowerOff: force PWR_MGMT0 = 0x03 (accel low-noise on)
		 * instead of shadow & 0xF0 (which is 0 = both sensors off) */
		{ 0x26020, { 0x93, 0x75, 0x05, 0x0f }, { 0x93, 0x05, 0x30, 0x00 }, 4, 1 },
		{ 0x23AE0, { 0x63, 0x90, 0x55, 0x07 }, { 0x13, 0x00, 0x00, 0x00 }, 4, 1 },
		/* reI: jump straight into the accel power-on block at 0x26078
		 * (set_power_mode(2) -> real PWR_MGMT0 write)
		 * PWR_MGMT0), and the registry always reports READY */
		{ 0x25FAA, { 0x29, 0xce, 0x00, 0x00 }, { 0xf9, 0xa0, 0x00, 0x00 }, 2, 1 },
		{ 0x0A968, { 0x05, 0x89, 0x00, 0x00 }, { 0x05, 0x45, 0x00, 0x00 }, 2, 1 },
		{ 0x26E0C, { 0xad, 0x45, 0x00, 0x00 }, { 0x89, 0x45, 0x00, 0x00 }, 2, 3 },
		/*
		 * "dram info not got, exit write dram".
		 *
		 * The sample drain reads its destination from the global
		 * chre_dram (0xA9E88), which only SET_CONFIG ever fills in - and
		 * SET_CONFIG is unreachable on this firmware because pkt_id 3
		 * dispatches into an empty legacy handler
		 * ("legacy_handler id 11,pkt_id:3 handler is null").  So the
		 * accel is armed correctly at 100 Hz, the FIFO threshold gets
		 * programmed, and then the drain bails out before writing a byte.
		 *
		 * EVERY read of that global has to be redirected, not just the
		 * one behind the "dram info not got" test: two of the sites
		 * dereference the pointer on the very next instruction
		 * (`lw a0, -376(s0)` then `lw s0, 4(a0)`), so patching only some
		 * of them turns the drain into a NULL dereference and the SCP
		 * never reports ready again - measured, that is exactly what
		 * happened with the first two-site version of this patch.
		 *
		 * Each site is a single `lw X, -376(Y)` that only loads a 32-bit
		 * pointer, so each becomes `lui X, 0x9F0CD`: the drain then works
		 * against SCP 0x9F0CD000 == AP 0x8F0CD000, a page inside the SENS
		 * block that nothing else uses.  The kernel owns that header and
		 * must keep FIFOSize in step (see SENS_FIFO_BASE in ipi_comm.c).
		 *
		 * 0x3B048 (SET_CONFIG's logging read) is deliberately left alone.
		 */
		{ 0x39392, { 0x03, 0x25, 0x84, 0xe8 }, { 0x37, 0xd5, 0x0c, 0x9f }, 4, 2 },
		{ 0x3AB82, { 0x03, 0x25, 0x84, 0xe8 }, { 0x37, 0xd5, 0x0c, 0x9f }, 4, 2 },
		{ 0x3ABDE, { 0x03, 0x25, 0x84, 0xe8 }, { 0x37, 0xd5, 0x0c, 0x9f }, 4, 2 },
		{ 0x3ACCC, { 0x03, 0x2b, 0x85, 0xe8 }, { 0x37, 0xdb, 0x0c, 0x9f }, 4, 2 },
	};
	void __iomem *img;
	unsigned int i, j;

	(void)data;
	if (!val)
		return 0;
	if (val > 4)
		val = 4;
	img = ioremap(0x9fa00000, 0x500000);
	if (!img)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(p); i++) {
		void __iomem *a = img + p[i].off;
		u8 cur[4];
		bool ok = true;

		if (p[i].group > val - 1)
			continue;
		for (j = 0; j < p[i].len; j++)
			cur[j] = readb(a + j);
		for (j = 0; j < p[i].len; j++)
			if (cur[j] != p[i].old[j])
				ok = false;
		if (!ok) {
			pr_err("scp-chopin: patchscp %#x expected %02x%02x got %02x%02x\n",
			       p[i].off, p[i].old[0], p[i].old[1], cur[0], cur[1]);
			continue;
		}
		for (j = 0; j < p[i].len; j++)
			writeb(p[i].new[j], a + j);
		for (j = 0; j < p[i].len; j++)
			cur[j] = readb(a + j);
		pr_info("scp-chopin: patchscp %#x -> %02x%02x%02x%02x (group %u)\n",
			p[i].off, cur[0], cur[1], cur[2], cur[3], p[i].group);
	}
	iounmap(img);
	return 0;
}
/*
 * Dump the sensor FIFO the firmware writes into.  The base is the page the
 * drain is patched to (patchscp group 5); records live at base + 0x10 + wp and
 * the size must be a multiple of 44.
 *
 * A read against the SCP's DRAM window while the core is held in reset faults
 * the AP, so refuse to touch it unless the core is up.
 */
static int scp_fifo_set(void *data, u64 val)
{
	void __iomem *f;
	u32 rp, wp, size, i;

	(void)data;
	if (!val)
		return 0;
	if (!is_scp_ready(SCP_A_ID)) {
		pr_info("scp-chopin: SCP not ready, fifo dump skipped\n");
		return 0;
	}
	f = ioremap(0x8F0CD000, 0xFE000);
	if (!f)
		return -ENOMEM;
	rp = readl(f + 0x00);
	wp = readl(f + 0x04);
	size = readl(f + 0x08);
	pr_info("scp-chopin: fifo rp=%u wp=%u size=%u records=%u%s\n",
		rp, wp, size, size ? size / 44 : 0,
		(size && size % 44) ? " [size is not a multiple of 44!]" : "");
	for (i = 1; i <= 3 && size && wp >= i * 44; i++) {
		u32 rec = (wp - i * 44) % size;
		u32 o = 0x10 + rec;

		/* data_unit_t: +0 type, +1 flush, +4 u64 ts, +12 x, +16 y, +20 z */
		pr_info("scp-chopin: rec-%u off=%u type=%u ts=%llu x=%d y=%d z=%d\n",
			i, rec, readl(f + o),
			(u64)readl(f + o + 4) | ((u64)readl(f + o + 8) << 32),
			(s32)readl(f + o + 12), (s32)readl(f + o + 16),
			(s32)readl(f + o + 20));
	}
	iounmap(f);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(scp_fifo_fops, NULL, scp_fifo_set, "%llu\n");

/*
 * The firmware writes its own printk ring into SCP_A_LOGGER_MEM_ID, which the
 * reserved-memory table places at 0x8F1CA700 (the "@@@@ reserved" dump gives
 * id 3 = 0x180000, right after SENS_MEM's 1 MB at 0x8F0CA700).  Reading it needs
 * no protocol at all, so it is the only trustworthy answer to "did the SCP
 * actually act on that packet": SET_CONFIG logs
 * "get dram phy addr=...,size=...,maxEventNumber:...", and the accel path logs
 * "ICM4X6XXSample" / "accGyro, watermark:%d < 1".
 *
 * The ring wraps, and the interesting part is the newest lines, so scan the
 * whole block, keep every line that has the firmware's "[sec.msec](core) "
 * prefix, and print the last SCP_LOG_LINES of them in order.  Reading the raw
 * ring with devmem only ever reaches the oldest lines, which is why the
 * SET_CONFIG trace was missed.
 */
#define SCP_LOG_BASE	0x8F1CA700
#define SCP_LOG_TOTAL	0x180000
#define SCP_LOG_LINES	400
#define SCP_LOG_LINE_MAX 255

struct scp_log_dump {
	char *text;
	size_t len;
};

#define SCP_LOG_MAX_LINES	20000
#define SCP_LOG_PRINT_LINES	1200
/* Firmware log lines are dense; a gap this large means a different boot. */
#define SCP_LOG_MAX_GAP_MS	5000

struct scp_log_line {
	size_t off;
	size_t len;
	u32 t_ms;
};

/* "[0.087](0) message" - everything else is stale DRAM or a torn line. */
static bool scp_log_parse(const char *p, const char *end, u32 *t_ms)
{
	const char *q = p;
	u32 sec = 0, msec = 0;

	if (end - q < 8 || *q++ != '[')
		return false;
	if (q >= end || *q < '0' || *q > '9')
		return false;
	while (q < end && *q >= '0' && *q <= '9')
		sec = sec * 10 + (*q++ - '0');
	if (q >= end || *q++ != '.')
		return false;
	while (q < end && *q >= '0' && *q <= '9' && msec < 1000)
		msec = msec * 10 + (*q++ - '0');
	if ((end - q) < 3 || q[0] != ']' || q[1] != '(')
		return false;
	*t_ms = sec * 1000 + msec;
	return true;
}

/*
 * The firmware appends to this block and only wraps once it is full (its own
 * log prints "dst_len" counting down from 0xFFFFF), so the current boot's log
 * is the ascending run that starts at offset 0.  Anything past the first break
 * is DRAM left over from an earlier boot - which is exactly what a naive "print
 * the tail of the block" shows, and how the SET_CONFIG trace got lost.
 *
 * If the ring has already wrapped, fall back to the run with the newest
 * timestamp.
 */
static void scp_log_collect(char *raw, struct scp_log_line *ln, size_t *count,
			    size_t *run_start, size_t *run_end)
{
	size_t off, n = 0, i, best = 0, best_end = 0;
	u32 t;

	for (off = 0; off < SCP_LOG_TOTAL && n < SCP_LOG_MAX_LINES;) {
		char *nl = memchr(raw + off, '\n', SCP_LOG_TOTAL - off);
		size_t end = nl ? (size_t)(nl - raw) : SCP_LOG_TOTAL;

		if (scp_log_parse(raw + off, raw + end, &t)) {
			ln[n].off = off;
			ln[n].len = min(end - off, (size_t)SCP_LOG_LINE_MAX);
			ln[n].t_ms = t;
			n++;
		}
		off = end + 1;
		if (!nl)
			break;
	}
	*count = n;

	/* Current boot: starts near offset 0 and runs without a large gap. */
	*run_start = 0;
	*run_end = 0;
	if (n && ln[0].off < 0x1000) {
		i = 1;
		while (i < n && ln[i].t_ms >= ln[i - 1].t_ms &&
		       ln[i].t_ms - ln[i - 1].t_ms <= SCP_LOG_MAX_GAP_MS) {
			if (ln[i].t_ms - ln[i - 1].t_ms > 200 &&
			    ln[i].off - ln[i - 1].off > 0x2000)
				break;
			i++;
		}
		*run_end = i;
	}
	if (*run_end >= 20)
		return;

	/* Wrapped ring: pick the run that reaches the largest timestamp. */
	for (i = 0; i < n; i++) {
		size_t j = i + 1;

		while (j < n && ln[j].t_ms >= ln[j - 1].t_ms &&
		       ln[j].t_ms - ln[j - 1].t_ms <= SCP_LOG_MAX_GAP_MS)
			j++;
		if (ln[j - 1].t_ms >= (best_end ? ln[best_end - 1].t_ms : 0)) {
			best = i;
			best_end = j;
		}
		i = j - 1;
	}
	*run_start = best;
	*run_end = best_end;
}

static int scp_log_open(struct inode *inode, struct file *file)
{
	struct scp_log_dump *d;
	struct scp_log_line *ln = NULL;
	void __iomem *log = NULL;
	char *raw = NULL;
	size_t count = 0, run_start = 0, run_end = 0, first, i;
	int ret = 0;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	if (!is_scp_ready(SCP_A_ID)) {
		pr_info("scp-chopin: SCP not ready, log read skipped\n");
		d->text = kstrdup(
			"SCP is down: its DRAM window must not be touched.\n"
			"Release it first (patchscp, fullreset, sram, start);\n"
			"the log ring lives in AP DRAM and survives the restart.\n",
			GFP_KERNEL);
		if (!d->text) {
			kfree(d);
			return -ENOMEM;
		}
		d->len = strlen(d->text);
		file->private_data = d;
		return 0;
	}

	log = ioremap_wc(SCP_LOG_BASE, SCP_LOG_TOTAL);
	if (!log) {
		ret = -ENOMEM;
		goto out;
	}
	raw = kvzalloc(SCP_LOG_TOTAL + 1, GFP_KERNEL);
	ln = kcalloc(SCP_LOG_MAX_LINES, sizeof(*ln), GFP_KERNEL);
	if (!raw || !ln) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy_fromio(raw, log, SCP_LOG_TOTAL);

	scp_log_collect(raw, ln, &count, &run_start, &run_end);

	/* Keep the tail of the run: the newest lines are the interesting ones. */
	first = (run_end - run_start > SCP_LOG_PRINT_LINES) ?
		run_end - SCP_LOG_PRINT_LINES : run_start;

	d->text = kvzalloc((run_end - first + 1) * (SCP_LOG_LINE_MAX + 1) + 256,
			   GFP_KERNEL);
	if (!d->text) {
		ret = -ENOMEM;
		goto out;
	}
	{
		u32 t0 = ln[run_start].t_ms;
		u32 t1 = ln[run_end ? run_end - 1 : 0].t_ms;

		d->len = scnprintf(d->text, 256,
			"# %zu lines in ring, this boot: %zu (t=%u.%03u .. %u.%03u s)%s\n",
			count, run_end - run_start,
			t0 / 1000, t0 % 1000, t1 / 1000, t1 % 1000,
			first > run_start ? " (tail kept)" : "");
	}
	for (i = first; i < run_end; i++) {
		memcpy(d->text + d->len, raw + ln[i].off, ln[i].len);
		d->len += ln[i].len;
		d->text[d->len++] = '\n';
	}

	file->private_data = d;
out:
	kvfree(raw);
	kfree(ln);
	if (log)
		iounmap(log);
	if (ret) {
		kfree(d);
		return ret;
	}
	return 0;
}

static ssize_t scp_log_read(struct file *file, char __user *ubuf,
			    size_t n, loff_t *ppos)
{
	struct scp_log_dump *d = file->private_data;

	return simple_read_from_buffer(ubuf, n, ppos, d->text, d->len);
}

static int scp_log_release(struct inode *inode, struct file *file)
{
	struct scp_log_dump *d = file->private_data;

	kvfree(d->text);
	kfree(d);
	return 0;
}

static const struct file_operations scp_log_fops = {
	.owner = THIS_MODULE,
	.open = scp_log_open,
	.read = scp_log_read,
	.release = scp_log_release,
	.llseek = default_llseek,
};

DEFINE_DEBUGFS_ATTRIBUTE(scp_patchscp_fops, NULL, scp_patchscp_set, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(scp_fullreset_fops, NULL, scp_fullreset_set, "%llu\n");

static int scp_fullreset_zero_set(void *data, u64 val)
{
	struct scp_dev *s = data;

	if (!val)
		return 0;
	scp_cleartcm(s);
	scp_loadloader(s);
	scp_reginfo(s);
	scp_dramzero(s);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(scp_fullreset_zero_fops, NULL, scp_fullreset_zero_set,
			 "%llu\n");

static int scp_core1_set(void *data, u64 val)
{
	struct scp_dev *s = data;

	s->release_core1 = !!val;
	dev_info(s->dev, "release core1: %d\n", s->release_core1);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(scp_core1_fops, NULL, scp_core1_set, "%llu\n");

SCP_DEBUGFS_ACTION(cleartcm, scp_cleartcm);
SCP_DEBUGFS_ACTION(loadloader, scp_loadloader);
SCP_DEBUGFS_ACTION(reginfo, scp_reginfo);
SCP_DEBUGFS_ACTION(dramrestore, scp_dramrestore);
SCP_DEBUGFS_ACTION(dramzero, scp_dramzero);
SCP_DEBUGFS_ACTION(sram, scp_enable_sram);

static int scp_probe(struct platform_device *pdev)
{
	struct scp_dev *s;
	struct dentry *d;

	s = devm_kzalloc(&pdev->dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = &pdev->dev;

	s->tcm = devm_ioremap(&pdev->dev, SCP_TCM_BASE, SCP_TCM_SIZE);
	s->clkreg = devm_ioremap(&pdev->dev, SCP_CLKREG_BASE, SCP_CLKREG_SIZE);
	s->core0 = devm_ioremap(&pdev->dev, SCP_CORE0_BASE, SCP_CORE_SIZE);
	s->core1 = devm_ioremap(&pdev->dev, SCP_CORE1_BASE, SCP_CORE_SIZE);
	s->image = devm_ioremap(&pdev->dev, SCP_IMAGE_BASE, SCP_IMAGE_SIZE);
	if (!s->tcm || !s->clkreg || !s->core0 || !s->core1 || !s->image)
		return -ENOMEM;

	scp_read_region_info(s);
	scp_find_share(s);
	s->share = devm_ioremap(&pdev->dev, s->share_phys, s->share_size);

	platform_set_drvdata(pdev, s);

	dev_info(&pdev->dev,
		 "ready: share=%pa/%x (%s) loader=%08x/%x fw=%08x/%x dram=%08x/%x backup=%08x\n",
		 &s->share_phys, s->share_size, s->share_src,
		 s->region_info[0], s->region_info[1], s->region_info[2],
		 s->region_info[3], s->region_info[4], s->region_info[5],
		 s->region_info[6]);
	dev_info(&pdev->dev, "resv reg %08x/%08x, clk %08x, core0 st %08x pc %08x\n",
		 scp_rd(s, s->core0, SCP_CORE_DRAM_RESV),
		 scp_rd(s, s->core0, SCP_CORE_DRAM_SIZE),
		 scp_rd(s, s->clkreg, SCP_CLK_ENABLE),
		 scp_rd(s, s->core0, SCP_CORE_MON_STATUS),
		 scp_rd(s, s->core0, SCP_CORE_MON_PC));

	d = debugfs_create_dir("scp-chopin", NULL);
	s->debugfs = d;
	debugfs_create_file("status", 0444, d, s, &scp_status_fops);
	debugfs_create_file("audit", 0444, d, s, &scp_audit_fops);
	debugfs_create_file("start", 0200, d, s, &scp_start_fops);
	debugfs_create_file("fullreset", 0200, d, s, &scp_fullreset_fops);
	debugfs_create_file("fullreset_zero", 0200, d, s,
			    &scp_fullreset_zero_fops);
	debugfs_create_file("cleartcm", 0200, d, s, &scp_cleartcm_fops);
	debugfs_create_file("loadloader", 0200, d, s, &scp_loadloader_fops);
	debugfs_create_file("reginfo", 0200, d, s, &scp_reginfo_fops);
	debugfs_create_file("dramrestore", 0200, d, s, &scp_dramrestore_fops);
	debugfs_create_file("dramzero", 0200, d, s, &scp_dramzero_fops);
	debugfs_create_file("sram", 0200, d, s, &scp_sram_fops);
	debugfs_create_file("patchscp", 0200, d, s, &scp_patchscp_fops);
	debugfs_create_file("fifo", 0200, d, s, &scp_fifo_fops);
	debugfs_create_file("log", 0444, d, s, &scp_log_fops);
	debugfs_create_file("core1", 0200, d, s, &scp_core1_fops);
	return 0;
}

static const struct of_device_id scp_of_match[] = {
	{ .compatible = "mediatek,scp-chopin-start" },
	{ }
};
MODULE_DEVICE_TABLE(of, scp_of_match);

static struct platform_driver scp_driver = {
	.probe = scp_probe,
	.driver = {
		.name = "scp-chopin-start",
		.of_match_table = scp_of_match,
	},
};
module_platform_driver(scp_driver);
MODULE_DESCRIPTION("chopin controlled SCP bring-up helper");
MODULE_LICENSE("GPL");
