// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek log_store console — boot log recovery through expdb.
 *
 * chopin (Redmi Note 10 Pro, MT6893) has no usable pstore and no serial
 * console, so the only way to read back a boot that dies before userspace is
 * the bootloader's own log ring: preloader and LK keep a text ring in
 * reserved DRAM at 0x7ffbf000 and relay it into the "expdb" partition on the
 * *next* boot.  This driver puts the kernel's own output into that ring, from
 * the first instruction that can reach DRAM to the last printk before the
 * machine hangs.
 *
 * There are three writers, deliberately independent of each other:
 *
 *  1. the pre-MMU tags in arch/arm64/kernel/head.S (K00..K03) and the
 *     breadcrumbs in arch/arm64/kernel/setup.c, which call
 *     mtk_logstore_breadcrumb{,_early}() here;
 *  2. the vprintk_emit() mirror, mtk_logstore_printk(), which copies every
 *     formatted printk record into the ring *synchronously* — so the last
 *     line in the ring is the last line the kernel ever printed, even if the
 *     console never got a chance to flush;
 *  3. a real console (CON_PRINTBUFFER) registered from an early_initcall,
 *     which replays the whole printk ring the first time it runs.
 *
 * On top of that, fb_band*() paints coloured stripes on the two candidate
 * framebuffers, which is a channel that needs neither expdb nor a working
 * storage stack.  See port/SUMMARY.md section 7.
 *
 * ---------------------------------------------------------------------------
 * Address space rules (arm64) — every one of these was learned the hard way
 * and is load bearing; see port/SUMMARY.md first section:
 *
 *  - early_ioremap() exists only between early_ioremap_init() and
 *    early_ioremap_reset().  It must never be held across the reset, so
 *    every user here maps, writes and unmaps inside a single call.
 *    (Port/SUMMARY.md: a mapping held across the reset killed the kernel on
 *    the first printk after it.)
 *  - __va() exists only after paging_init().  Before that the linear map has
 *    not been built and a "branch to garbage" is what you get.
 *  - ioremap() needs vmalloc_init(), i.e. the initcall phase, and arm64
 *    refuses to ioremap System RAM outright:
 *        WARN_ONCE("ioremap attempted on RAM pfn")  -> NULL
 *    The ring at 0x7ffbf000 is reserved but *not* no-map, so it is ordinary
 *    System RAM: it can only be reached through __va(), and because that
 *    alias is write-back cached every write has to be pushed out with
 *    dcache_clean_poc() — a watchdog hard reset does not flush the caches.
 *
 * So the mirror is attached in two steps:
 *
 *      early_ioremap_init()  ->  mtk_logstore_breadcrumb_early()
 *      paging_init()         ->  mtk_logstore_attach_early()
 *      early_initcall        ->  mtk_logstore_early_init()
 *
 * mtk_logstore_breadcrumb() is the post-paging_init() flavour of the
 * breadcrumb (setup.c uses both, on either side of paging_init()).
 *
 * ---------------------------------------------------------------------------
 * The ring: two windows, "restart when it does not fit".
 *
 * LK only reliably relays a fixed prefix of the ring, so the same text is
 * written twice:
 *
 *      0x0800 .. 0x10000   low window   — always relayed, length-independent
 *      0x20000 .. 0x3f000  high window  — useful when LK honours the length
 *
 * A window is never "filled up and then stopped" (that hides the crash point
 * behind the noise of a full boot): when the next record does not fit in what
 * is left of the window, the window restarts at its beginning.  The newest
 * output is therefore always contiguous from the start of the window and the
 * very last line is the one the kernel died on.
 *
 * ---------------------------------------------------------------------------
 * The SRAM header at 0x11df00 is *not* touched by default.
 *
 * It is a bus access to the always-on SRAM block, and on this SoC a bad one
 * hangs the interconnect instead of faulting — the symptom is a silent death
 * with no output at all, which is exactly what happened here once.  The low
 * window does not need it (that is the whole point of the low window), so
 * CONFIG_MTK_LOGSTORE_SRAM_HEADER defaults to n and everything that reads or
 * writes the header is compiled out.  When it is enabled the fields are
 * reached through offsetof() on the vendor structs in <linux/mtk-logstore.h>,
 * because the header is *not* a struct pl_lk_log at offset 0 — the current
 * log descriptor lives at +0x34 (LK prints "current log header 0x11df34").
 */

#define pr_fmt(fmt)	"mtk-logstore: " fmt

#include <asm-generic/early_ioremap.h>
#include <linux/console.h>
#include <linux/device/devres.h>
#include <linux/dev_printk.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mtk-logstore.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/sprintf.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/cacheflush.h>
#include <asm/memory.h>
#include <asm/page.h>

/*
 * log_store reserved DRAM.  Non-secure, survives a watchdog reset, not
 * managed by mrdump/aee, and relayed into expdb by LK on the next boot.
 */
#define LOGSTORE_LOW_START	0x0800U
#define LOGSTORE_LOW_END	0x10000U
#define LOGSTORE_HIGH_START	0x20000U
#define LOGSTORE_HIGH_END	0x3f000U

/*
 * Highest offset we ever claim to have written.  Deliberately short of the
 * end of the ring: LK has a separate "the ring wrapped" path and we do not
 * want to trip it.
 */
#define LOGSTORE_HIGH_WATER	0x3f200U

/* plausible range for the bootloader's own off_lk */
#define LOGSTORE_MIN_OFF	0x400U

/* keep at least this much room below LOGSTORE_HIGH_WATER */
#define LOGSTORE_WATER_SLACK	0x1000U

/*
 * The C-level breadcrumbs start a little way into the low window.  head.S
 * pokes 4-byte tags at 0x800 + n*16, so 0x40 bytes of lead-in keeps the two
 * from interleaving.
 */
#define LOGSTORE_BC_OFF		0x40U

/* ------------------------------------------------------------------------ */
/* boot progress stripes                                                    */
/*                                                                          */
/* LK leaves the display controller scanning out its logo, and nothing in    */
/* this kernel reprograms it yet, so a pixel written now is visible at once. */
/* Two candidate buffers, because the vendor tree and LK's own log disagree  */
/* about which one is live:                                                 */
/*                                                                          */
/*   FB_CAND0 (green)  LK's last "mt_disp_config_fb_layer, addr = ..."       */
/*   FB_CAND1 (red)    the vendor reserved-memory node called "framebuffer"  */
/*                                                                          */
/* 16 lines per stripe, 4352 bytes stride (1080 px * 4, padded), so one      */
/* stripe is 0x11000 bytes.  head.S draws stages 0..3; setup.c continues     */
/* from stage 4; the driver's own steps are the *yellow* notes, spaced two   */
/* bands apart so each one can be counted on the panel.                      */
/* ------------------------------------------------------------------------ */
#define FB_CAND0		0x78e0c000U
#define FB_CAND1		0x77a20000U
#define FB_STRIDE		4352U
#define FB_WIDTH		1080U
#define FB_BAND_LINES		16U
#define FB_BAND_SPAN		(FB_BAND_LINES * FB_STRIDE)	/* 0x11000 */
/*
 * early_ioremap() budget is 65 pages for the whole kernel, so the early band
 * painter must never hold more than a few pages at a time.
 */
#define FB_BAND_CHUNK		(4 * PAGE_SIZE)
#define FB_PROGRESS_BASE	4U
#define FB_NOTE_BASE		20U
#define FB_NOTE_STEP		2U

#define FB_COLOUR_GREEN		0xff00ff00U
#define FB_COLOUR_GREEN_ALT	0xff00a000U
#define FB_COLOUR_RED		0xffff0000U
#define FB_COLOUR_RED_ALT	0xffa00000U
#define FB_COLOUR_NOTE		0xffffff00U

/*
 * SRAM header field offsets.  Every one of them is derived from the vendor
 * layout in <linux/mtk-logstore.h>; the trailing numbers are what they
 */
#define HDR_SRAM_SIG	(offsetof(struct sram_log_header, sig))			/* 0x00 */
#define HDR_DRAM_SIG	(offsetof(struct sram_log_header, dram_buf) +		\
			 offsetof(struct dram_buf_header, sig))			/* 0x0c */
#define HDR_DRAM_FLAG	(offsetof(struct sram_log_header, dram_buf) +		\
			 offsetof(struct dram_buf_header, flag))		/* 0x10 */
#define HDR_BUF_POINT	(offsetof(struct sram_log_header, dram_buf) +		\
			 offsetof(struct dram_buf_header, buf_point))		/* 0x20 */
#define HDR_KLOG_ADDR	(offsetof(struct sram_log_header, dram_buf) +		\
			 offsetof(struct dram_buf_header, klog_addr))		/* 0x24 */
#define HDR_KLOG_SIZE	(offsetof(struct sram_log_header, dram_buf) +		\
			 offsetof(struct dram_buf_header, klog_size))		/* 0x28 */
#define HDR_PLLK_SIG	(offsetof(struct sram_log_header, dram_curlog_header) +	\
			 offsetof(struct pl_lk_log, sig))			/* 0x34 */
#define HDR_OFF_LK	(offsetof(struct sram_log_header, dram_curlog_header) +	\
			 offsetof(struct pl_lk_log, off_lk))			/* 0x48 */
#define HDR_SZ_LK	(offsetof(struct sram_log_header, dram_curlog_header) +	\
			 offsetof(struct pl_lk_log, sz_lk))			/* 0x4c */
#define HDR_RESERVE0	(offsetof(struct sram_log_header, reserve) +		\
			 SRAM_RECORD_LOG_SIZE * sizeof(u32))			/* 0x5c */
/* the DRAM-side copy of the current-log descriptor at the head of the ring */
#define RING_SZ_LK	offsetof(struct pl_lk_log, sz_lk)			/* 0x18 */
/*
 * One ring, plus the console that replays it.
 */
struct logstore {
	void *buf;
	void *hdr;
	size_t buf_size;
	u32 low_off;
	u32 high_off;
	bool mapped;
	bool cached;
	bool hdr_ok;
	struct console con;
};

static struct logstore boot_ls;
static struct logstore early_ls;
static struct logstore *ls;

/*
 * Write @len bytes from @dst into the [@win_off, @win_end) window of the
 * ring, advancing the window's persistent write offset @cursor.
 *
 * The policy is "first fit, else start over": if the record cannot fit in
 * what is left of the window the window is rewound to its beginning, so the
 * newest output is always contiguous from @win_off and the last line written
 * is the interesting one.  A record larger than the window replaces it.
 *
 * @cached selects the access: plain stores through the linear map (needs
 * dcache_clean_poc() afterwards) or uncached memcpy_toio() through an
 * early_ioremap() alias.
 *
 * A window that has been rewound starts writing at @win_off again, so the oldest
 * surviving bytes are always at the head of the window; a reader walks forward from
 * @win_off to the newest text.
 *
 * No lock is taken anywhere here: printk runs on every CPU in every context, and a
 * diagnostic ring that can deadlock is worse than one with a torn line.
 */
static void win_write(void *base, bool cached, u32 win_off, u32 win_end,
		      u32 *cursor, const char *dst, size_t len)
{
	u32 size = win_end - win_off;

	/*
	 * Too big for the window: keep the tail, the newest bytes are the
	 * ones that matter.
	 */
	if (size <= len) {
		dst += len - (size - 1);
		len = size - 1;
	}

	/* did not fit -> start over from the beginning of the window */
	if (size - *cursor < len)
		*cursor = 0;
	{
		void *p = base + win_off + *cursor;
		if (cached) {
			memcpy(p, dst, len);
			dcache_clean_poc((unsigned long)p, (unsigned long)p + len);
		} else {
			memcpy_toio(p, dst, len);
		}
		/*
		 * Advance the window's cursor.  This is the whole point of the
		 * ring, and it was missing: without it every record overwrote the
		 * same bytes, so the "black box" retained only the newest line
		 * ever printed and a hang left no history at all.  The
		 * breadcrumbs never showed the problem because they keep their
		 * own cursor (@bc_low).
		 *
		 * len <= size - 1 by the rewind above, so the cursor stays inside
		 * the window and the next record either fits or rewinds.
		 */
		*cursor += len;
	}
}

/* One record, both windows. */
static void logstore_append(const char *s, size_t n)
{
	struct logstore *l = READ_ONCE(ls);

	if (!l || !n)
		return;

	win_write(l->buf, l->cached, LOGSTORE_LOW_START, LOGSTORE_LOW_END,
		  &l->low_off, s, n);
	/* the same bytes go into the high window as well */
	win_write(l->buf, l->cached, LOGSTORE_HIGH_START, LOGSTORE_HIGH_END,
		  &l->high_off, s, n);
}

/* Keep LK's idea of the log length honest. */
static inline void logstore_set_length(void *hdr, void *buf)
{
	u32 off_lk, len;

	if (!hdr || !buf)
		return;

	off_lk = readl(hdr + HDR_OFF_LK);

	if (off_lk >= LOGSTORE_MIN_OFF &&
	    off_lk <= LOGSTORE_HIGH_WATER - LOGSTORE_WATER_SLACK)
		len = LOGSTORE_HIGH_WATER - off_lk;
	else
		len = LOGSTORE_HIGH_WATER - LOGSTORE_MIN_OFF;

	writel(len, hdr + HDR_SZ_LK);
	writel(LOGSTORE_HIGH_WATER, hdr + HDR_BUF_POINT);
	writel(len, hdr + HDR_RESERVE0);
	writel(len, buf + RING_SZ_LK);
}

/*
 * vprintk_emit() hook: mirror the record, then refresh the length fields.
 *
 * The mirror always happens — losing the text of an oops that occurs inside
 * this very driver is the exact failure this exists to prevent.  The length
 * update is skipped while the ring is not set up yet (dummy @mapped) or while
 * the SRAM header has not been read successfully (@hdr_ok), because touching
 * it too early risks hanging the bus from inside printk.
 *
 * Both windows receive the same bytes, so a reader that only gets the low window
 * still sees a complete log.
 *
 * Called from vprintk_emit() with the record already formatted.
 *
 */
void mtk_logstore_printk(const char *s, unsigned int n)
{
	struct logstore *l = READ_ONCE(ls);

	/*
	 * Two windows, one record: the low window is the one LK always
	 * relays, the high window pays off when it also honours the length.
	 * Both are written before the length fields are touched, so a crash
	 * anywhere below still leaves the text behind.
	 */
	logstore_append(s, n);
	if (l && l->mapped && l->hdr_ok)
		logstore_set_length(l->hdr, l->buf);
}

EXPORT_SYMBOL_GPL(mtk_logstore_printk);

/*
 * Paint one 16-line stripe at band @band on @base.
 *
 * Pre-paging_init() flavour: @phys is a physical address, the mapping comes
 * from early_ioremap() and is dropped again before returning.  The stores go
 * through writel() because the fixmap alias is not cacheable.
 * ------------------------------------------------------------------------
 * Panel stripes: why these are worth 0x11000 bytes of DRAM per stage
 *
 * When the kernel dies before expdb can be read back - no storage stack, a
 * hung interconnect, a hypervisor trap - the panel is the only channel left.
 * LK hands over a live scanout buffer and nothing reprograms the display
 * controller until fbcon/simplefb take over, so a pixel written now is
 * visible at once.
 *
 * Two things are encoded in the stripes:
 *
 *   the colour says *which* candidate buffer is the live one: green for
 *   0x78e0c000 (LK's last mt_disp_config_fb_layer) and red for 0x77a20000
 *   (the vendor reserved-memory node called "framebuffer").  Only one of
 *   them is being scanned out; the other one is wasted traffic.
 *
 *   the number of stripes says how far the boot got.  head.S draws stages
 *   0..3, setup.c continues from stage 4, and this driver adds yellow notes
 *   for its own internal steps, two bands apart so they can be counted.
 *
 * Everything is best effort: a failed early_ioremap() or a wrong candidate
 * address costs a few microseconds and nothing else.
 *
 */
static void __init fb_band_early(phys_addr_t base, unsigned int band, u32 colour)
{
	phys_addr_t phys = base + (phys_addr_t)band * FB_BAND_SPAN;
	void *map;
	size_t span = FB_BAND_SPAN;
	u32 i;

	/*
	 * Paint in small chunks, NOT the whole band at once.
	 *
	 * early_ioremap() lives in the fixmap and NR_FIX_BTMAPS is a *global*
	 * 65-page budget (FIX_BTMAPS_SLOTS = 7).  A band is 0x11000 bytes = 17
	 * pages, and fb_progress() paints two candidates per breadcrumb, so
	 * mapping a whole band per call exhausts the budget after barely two
	 * breadcrumbs - after which the fixmap state is broken and the boot dies
	 * inside paging_init().  Measured: the ring stopped at
	 * "[BC 05] before paging_init" and nothing after it ever appeared.
	 *
	 * FB_BAND_CHUNK pages are mapped, filled and unmapped at a time, so at
	 * most FB_BAND_CHUNK entries are ever in use.
	 */
	for (i = 0; i < span; i += FB_BAND_CHUNK) {
		size_t left = span - i;
		size_t chunk = left < FB_BAND_CHUNK ? left : FB_BAND_CHUNK;
		u32 j;

		map = early_ioremap(phys + i, chunk);
		if (!map)
			return;
		for (j = 0; j < chunk / sizeof(u32); j++)
			writel(colour, map + j * sizeof(u32));
		early_iounmap(map, chunk);
	}
}

/*
 * Post-paging_init() flavour, for when the linear map already exists.
 *
 * Both candidates live inside the vendor's mblock-19-framebuffer reserved
 * region, which the board tree marks no-map so that simplefb is allowed to
 * ioremap it (see chopin-board.dts).  A no-map region is *not* part of the
 * kernel's linear map - map_mem() skips MEMBLOCK_NOMAP ranges - so a __va()
 * store to it now takes a translation fault instead of landing in DRAM.  The
 * window has to be ioremap()ed like any other device memory.
 *
 * ioremap() itself needs vmalloc_init(), which runs from mm_core_init() well
 * after setup_arch() has finished, so the breadcrumbs between paging_init() and
 * the initcall phase can map nothing at all: neither __va() (region is no-map)
 * nor ioremap() (no vmalloc yet).  Those stages are queued here and replayed as
 * soon as the window exists, which is still before simplefb/fbcon claim the
 * screen, so the panel ends up with exactly one stripe per boot stage.
 *
 * The window covers the whole reserved region, so FB_CAND0 and FB_CAND1 share
 * one mapping and the range check below is the only thing keeping a wrong
 * candidate address from reaching outside it.
 */
#define FB_REGION_BASE		FB_CAND1	/* 0x77a20000 */
#define FB_REGION_SIZE		0x27e0000U	/* mblock-19-framebuffer */
#define FB_PENDING_MAX		16U

struct fb_pending_band {
	u32 base;
	unsigned int band;
	u32 colour;
};

static void __iomem *fb_win;
static struct fb_pending_band fb_pending[FB_PENDING_MAX];
static unsigned int fb_pending_n;

/*
 * ioremap() needs vmalloc_init(), which mm_core_init() runs well after
 * setup_arch() - and the boot stripes are painted from setup_arch().  Asking for
 * the window there does not merely fail, it trips arm64's
 * WARN_ONCE(pfn_is_map_memory(...)) "ioremap attempted on RAM pfn" because the
 * reserved framebuffer is inside a memblock region (nomap, but still mapped).
 * So gate on an explicit flag that only the first non-early user sets, and let
 * the callers queue the band instead.
 */
static bool fb_window_usable;

static void __iomem *fb_window(void)
{
	if (!fb_window_usable)
		return NULL;
	if (!fb_win)
		fb_win = ioremap_wc(FB_REGION_BASE, FB_REGION_SIZE);
	return fb_win;
}

/*
 * Paint one band through the window.  Returns false only when the window is not
 * available *yet* (so the caller can queue the band); an out-of-range band is
 * reported as success and dropped, because retrying it later cannot help.
 */


/*
 * The console, mirrored into the other candidate.
 *
 * LK does not always hand the display over scanning the buffer our board tree
 * names, and fbcon can only draw into that one.  After a warm reboot the panel
 * has been observed reading the *other* candidate (the progress stripes, painted
 * on both, were visible in CAND1's red while fbcon's text, written to CAND0, was
 * nowhere), which leaves a screen that looks hung even though the kernel is fine.
 * The USB console can be gone at the same time, so this is worth fixing here
 * rather than by staring harder at a dark screen.
 *
 * Both candidates live inside the NonCacheable mapping fb_window() owns, so this
 * copy never touches an address the kernel has not mapped: a first attempt using
 * __va() on the framebuffer failed hard, because mblock-19 is reserved *nomap* -
 * it is not in the linear map at all, and the fault landed in a kworker, which
 * took the boot down with it.  Read and write both go through fb_win.
 *
 * The period is deliberately slow: this is a ten megabyte copy through
 * write-combining memory, and the console does not need to be live to the
 * millisecond to be read.  Two seconds costs a few percent of one core and keeps
 * the screen usable whichever buffer the panel is on.
 */
#define FB_MIRROR_BYTES		(FB_STRIDE * 2400U)
#define FB_MIRROR_PERIOD_MS	2000

static struct delayed_work fb_mirror_work;
static bool fb_mirror_on = true;

static void fb_mirror_fn(struct work_struct *work)
{
	void __iomem *win = fb_window();

	if (fb_mirror_on && win &&
	    (u64)FB_CAND1 + FB_MIRROR_BYTES <=
			(u64)FB_REGION_BASE + FB_REGION_SIZE)
		memcpy(win + (FB_CAND1 - FB_REGION_BASE),
		       win + (FB_CAND0 - FB_REGION_BASE), FB_MIRROR_BYTES);

	schedule_delayed_work(&fb_mirror_work,
			      msecs_to_jiffies(FB_MIRROR_PERIOD_MS));
}

static bool fb_band_write(u32 base, unsigned int band, u32 colour)
{
	phys_addr_t phys = (phys_addr_t)base + (phys_addr_t)band * FB_BAND_SPAN;
	void __iomem *p;
	u32 i;

	if (!fb_window())
		return false;

	if (phys < FB_REGION_BASE ||
	    phys + FB_BAND_SPAN > (phys_addr_t)FB_REGION_BASE + FB_REGION_SIZE)
		return true;

	/* The window is Normal-NonCacheable, so a barrier is all that is needed
	 * to make the pixels visible to the display controller - there is no
	 * cache to clean, unlike the cached __va() alias this used to write. */
	p = fb_win + (phys - FB_REGION_BASE);
	for (i = 0; i < FB_BAND_SPAN / sizeof(u32); i++)
		writel(colour, p + i * sizeof(u32));
	wmb();

	return true;
}

static void fb_band_late(u32 base, unsigned int band, u32 colour)
{
	if (fb_band_write(base, band, colour))
		return;

	if (fb_pending_n < FB_PENDING_MAX) {
		fb_pending[fb_pending_n].base = base;
		fb_pending[fb_pending_n].band = band;
		fb_pending[fb_pending_n].colour = colour;
		fb_pending_n++;
	}
}

/*
 * Replay the stages painted while ioremap() was not usable yet.  Called once the
 * window can be created - the logstore early initcall, before any framebuffer
 * driver has registered.
 */
static void fb_flush_pending(void)
{
	unsigned int i;

	if (!fb_window())
		return;

	for (i = 0; i < fb_pending_n; i++)
		fb_band_write(fb_pending[i].base, fb_pending[i].band,
			      fb_pending[i].colour);
	fb_pending_n = 0;
}

/*
 * One yellow "the driver got this far" note.  Slot spacing is two bands, so
 * the yellow stripes are separable and countable on the panel: N yellow
 * stripes means the driver reached step N.
 *
 * The count *is* the step number - there is no text on the panel.
 *
 */
static void fb_note(unsigned int slot)
{
	unsigned int band = FB_NOTE_BASE + slot * FB_NOTE_STEP;
	fb_band_late(FB_CAND0, band, FB_COLOUR_NOTE);
	fb_band_late(FB_CAND1, band, FB_COLOUR_NOTE);
}

/* One green/red stripe pair per boot stage. */
static inline void fb_progress(unsigned int stage, bool early)
{
	u32 c0 = (stage & 1) ? FB_COLOUR_GREEN_ALT : FB_COLOUR_GREEN;
	u32 c1 = (stage & 1) ? FB_COLOUR_RED_ALT : FB_COLOUR_RED;

	if (early) {
		fb_band_early(FB_CAND0, stage + FB_PROGRESS_BASE, c0);
		fb_band_early(FB_CAND1, stage + FB_PROGRESS_BASE, c1);
	} else {
		fb_band_late(FB_CAND0, stage + FB_PROGRESS_BASE, c0);
		fb_band_late(FB_CAND1, stage + FB_PROGRESS_BASE, c1);
	}
}

/*
 * Early breadcrumb state.  @bc_low is the write offset *within* the low
 * window, @bc_seq the stage counter that labels each line and drives the
 * panel stripe.
 *
 * The panel stripe that goes with a breadcrumb uses the same stage number offset
 * by FB_PROGRESS_BASE, so the four pre-MMU stripes drawn by head.S (bands 0..3)
 * are never overwritten.
 *
 * bc_low is relative to LOGSTORE_LOW_START: it is the first byte the C-level
 * breadcrumbs own.  head.S pokes 4-byte tags at 0x800 + n*16, which is why the
 * cursor starts LOGSTORE_BC_OFF bytes in, and why the two writers cannot collide:
 * every breadcrumb re-reads the ring's own low cursor after writing, and that
 * cursor is advanced by the tag writer as well.
 *
 *
 */
static int bc_seq;
static u32 bc_low = LOGSTORE_BC_OFF;

static inline int bc_format(char *buf, size_t len, unsigned int stage, const char *what)
{
	return scnprintf(buf, len, "[BC %02d] %s\n", stage, what);
}

/*
 * Breadcrumb for the window between early_ioremap_init() and
 * paging_init(): __va() does not exist yet and early_ioremap() must not be
 * held across early_ioremap_reset(), so map / write / unmap all happen here.
 *
 * The page containing the cursor is mapped once (2 pages, well under the
 * fixmap budget) and the cursor is written at its in-page offset, which also
 */
void __init mtk_logstore_breadcrumb_early(const char *what)
{
	u32 pos = bc_low + LOGSTORE_LOW_START;
	void *sram, *ring;
	char buf[96];
	unsigned int stage = bc_seq++;
	int n;
	u32 page = pos & ~(u32)(PAGE_SIZE - 1);

	/*
	 * The ring is ordinary System RAM (reserved but not no-map), so
	 * ioremap() would refuse it and __va() is not available yet: this is
	 * the one path that has to go through early_ioremap().
	 */
	n = bc_format(buf, sizeof(buf), stage, what);

	ring = early_ioremap(LOGSTORE_PA + page, 2 * PAGE_SIZE);
	sram = early_ioremap(LOGSTORE_SRAM_HEADER_PA, PAGE_SIZE);
	if (ring)
		memcpy_toio(ring + (pos - page), buf, n);
	bc_low += n;
	if (ring && sram)
		logstore_set_length(sram, ring - page);
	if (ring)
		early_iounmap(ring, 2 * PAGE_SIZE);
	if (sram)
		early_iounmap(sram, PAGE_SIZE);

	fb_progress(stage, true);
}


/* Breadcrumb for after paging_init(): __va() works now. */
void __init mtk_logstore_breadcrumb(const char *what)
{
	char buf[96];
	unsigned int stage = bc_seq++;
	int n;

	/*
	 * Write first and update the length afterwards: if the length update
	 * is the thing that hangs, the breadcrumb is already in the window.
	 */
	n = bc_format(buf, sizeof(buf), stage, what);
	if (ls && ls->low_off < bc_low)
		ls->low_off = bc_low;
	logstore_append(buf, n);
	bc_low = ls ? ls->low_off : bc_low + n;
	if (ls && ls->mapped)
		logstore_set_length(ls->hdr, ls->buf);
	fb_progress(stage, false);
}

/*
 * Called from setup_arch() once paging_init() has built the linear map.
 *
 * The ring is installed as a *placeholder*: the mirror writes immediately, so
 * an oops inside this driver is still captured, but no length is recorded and
 * the console callback stays off until logstore_setup() runs for real.
 */
void __init mtk_logstore_attach_early(void)
{
	if (ls)
		return;

	/* __va() is legal here, and it is a write-back cached alias. */
	boot_ls.buf = __va(LOGSTORE_PA);
	boot_ls.hdr = NULL;
	boot_ls.low_off = 0;
	boot_ls.high_off = 0;
	boot_ls.mapped = false;
	boot_ls.cached = true;
	WRITE_ONCE(ls, &boot_ls);
}

/*
 * The console callback is deliberately empty.
 *
 * The vprintk_emit() hook in kernel/printk/printk.c already mirrors every
 * record, including - again - whatever the CON_PRINTBUFFER replay would hand
 * us.  Having both write meant every line was appended twice, at two
 * independent cursors, which interleaved the windows into gibberish.  That was
 * invisible while win_write() forgot to advance its cursor (the second write
 * simply landed on top of the first); it became obvious the moment it did
 * advance.  The console stays registered so the kernel's console list is
 * unchanged, but it contributes no bytes.
 */
static inline void logstore_write(struct console *co, const char *s, unsigned int n)
{
}

/*
 * Publish the kernel's own printk buffer in the SRAM header so that LK can find
 * it, and mark it BUFF_EARLY_PRINTK.  log_buf is a kernel image address, which
 * virt_addr_valid() rejects, so on this board this does nothing at all.
 *
 * Not __init: its only caller, logstore_setup(), is reachable from the platform
 * driver as well as from the early initcall (see there).
 */
static inline void logstore_publish_klog(struct logstore *l)
{
	char *kbuf = log_buf_addr_get();
	phys_addr_t pa;
	u32 ksize = log_buf_len_get();
	if (!kbuf || !ksize)
		goto no_klog;
	if (is_vmalloc_addr(kbuf) || !virt_addr_valid(kbuf))
		goto no_klog;
	pa = __pa(kbuf);
	if (pa >> 32)
		goto no_klog;
	writel((u32)pa, l->hdr + HDR_KLOG_ADDR);
	writel(ksize, l->hdr + HDR_KLOG_SIZE);
	writel(readl(l->hdr + HDR_DRAM_FLAG) | DRAM_FLAG_EARLY_PRINTK,
	       l->hdr + HDR_DRAM_FLAG);
	pr_info("published printk buffer at 0x%llx (size 0x%x)\n",
		(u64)pa, ksize);
no_klog:
	return;
}
/*
 * One-time ring setup: early_initcall time, or the platform driver.
 *
 * Deliberately *not* __init.  The platform driver is a legitimate caller (the
 * DT node is optional, but when it is present this is the only setup path), and
 * modpost rightly complains when .text calls into .init.text - a probe that
 * runs after free_initmem() would jump into freed memory.  Keeping these two
 * functions resident costs a few hundred bytes.
 */
static inline int logstore_setup(struct logstore *l, phys_addr_t phys, size_t span,
					 struct resource *hres)
{
	/*
	 * A marker line for the self-check below.  Big enough for all three
	 * signatures plus the three lengths, and it lives on the stack so this
	 * works before the allocator is up.
	 */
	u32 off_lk, sz_lk, buf_point;
	char marker[160];
	int n;
	/*
	 * Refuse anything that is not System RAM: __va() must yield a valid
	 * linear-map address, and the whole point of this driver is that it
	 * never ioremaps.  The message is deliberately loud — it also tells a
	 * reader that the DT reserved-memory node is wrong.
	 */
	if (!pfn_is_map_memory(PHYS_PFN(phys))) {
		pr_err("0x%llx is not System RAM, refusing\n", (u64)phys);
		/* no linear map for this address, so there is nothing to use */
		return -EINVAL;
	}

	l->buf = __va(phys);
	l->buf_size = span;
	l->mapped = true;
	l->cached = true;
#if IS_ENABLED(CONFIG_MTK_LOGSTORE_SRAM_HEADER)
	/*
	 * Only ever reached with the Kconfig option on: reading the SRAM
	 * header is the one bus access in this driver that can hang the SoC
	 * instead of faulting.
	 */
	l->hdr = ioremap(hres ? hres->start : LOGSTORE_SRAM_HEADER_PA, PAGE_SIZE);
#else
	l->hdr = NULL;
#endif

	/*
	 * A ring that was already set up (the platform driver running before
	 * the early console, or the other way round) keeps its cursors, so the
	 * two never write over each other.
	 */
	if (ls && ls != l) {
		l->low_off = ls->low_off;
		l->high_off = ls->high_off;
	}
	WRITE_ONCE(ls, l);
	/* Replay the boot stages painted before ioremap() was usable, then paint
	 * this driver's own progress notes on top.  From here on ioremap() works,
	 * so fb_band_late() may paint directly instead of queueing. */
	fb_window_usable = true;
	fb_flush_pending();
	fb_note(1);

	if (l->hdr) {
		/* the current-log descriptor is at +0x34, not at +0 */
		off_lk = readl(l->hdr + HDR_OFF_LK);
		fb_note(2);
		sz_lk = readl(l->hdr + HDR_SZ_LK);
		buf_point = readl(l->hdr + HDR_BUF_POINT);
		l->hdr_ok = true;
	} else {
		fb_note(3);
		off_lk = 0;
		sz_lk = 0;
		buf_point = 0;
	}

	/*
	 * Self-check line: it goes into the ring (not through printk) before
	 * the console exists, so the very first thing that comes back from a
	 * dump proves the whole path works.
	 */
	if (l->hdr) {
		u32 sram_sig = readl(l->hdr + HDR_SRAM_SIG);
		u32 dram_sig = readl(l->hdr + HDR_DRAM_SIG);
		u32 pllk_sig = readl(l->hdr + HDR_PLLK_SIG);

		n = scnprintf(marker, sizeof(marker),
			      "\n[mtk-logstore] sram_sig=%08x dram_sig=%08x pllk_sig=%08x off_lk=%u sz_lk=%u buf_point=%u\n",
			      sram_sig, dram_sig, pllk_sig, off_lk, sz_lk,
			      buf_point);
	} else {
		n = scnprintf(marker, sizeof(marker),
			      "[mtk-logstore] sram_sig=%08x dram_sig=%08x pllk_sig=%08x off_lk=%u sz_lk=%u buf_point=%u\n",
			      0, 0, 0, off_lk, sz_lk, buf_point);
	}
	logstore_append(marker, n);
	logstore_append("[mtk-logstore] linux console follows\n", 37);
	logstore_set_length(l->hdr, l->buf);
	fb_note(4);
	logstore_publish_klog(l);
	/* the console is registered later; only its data is prepared here */
	fb_note(5);
	l->con.write = logstore_write;
	l->con.flags = CON_PRINTBUFFER | CON_ENABLED;
	strscpy(l->con.name, "mtk_logstore_console", sizeof(l->con.name));
	fb_note(6);
	return 0;
}

/* The DT node is optional: early_initcall covers the normal boot. */
static void __init fb_probe_display(const char *when);

static int mtk_logstore_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res, *hres;
	struct logstore *l;
	int ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "buffer");
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	/*
	 * The "header" resource is only used with
	 * CONFIG_MTK_LOGSTORE_SRAM_HEADER; it is looked up regardless so that
	 * a DT with the node present does not warn.
	 */
	hres = platform_get_resource_byname(pdev, IORESOURCE_MEM, "header");
	if (ls)
		return 0;

	l = devm_kzalloc(dev, sizeof(*l), GFP_KERNEL);
	if (!l)
		return -ENOMEM;

	ret = logstore_setup(l, res->start, resource_size(res), hres);
	if (ret)
		return ret;

	WRITE_ONCE(ls, l);
	register_console(&l->con);
	fb_probe_display("later, from the platform probe");
	dev_info(dev,
		 "log_store ring at 0x%llx (size 0x%zx), windows +0x%x and +0x%x used\n",
		 (u64)res->start, l->buf_size, l->low_off, l->high_off);
	return 0;
}

/*
 * The stock chopin DT node is "mediatek,log_store"; "mediatek,mt6893-logstore"
 * is what our own board DTS uses.  Both entries are needed while the two trees
 * coexist.
 */
static const struct of_device_id mtk_logstore_of_match[] = {
	{ .compatible = "mediatek,mt6893-logstore" },
	{ .compatible = "mediatek,log_store" },
	{ }	/* sentinel */
};

MODULE_DEVICE_TABLE(of, mtk_logstore_of_match);

static struct platform_driver mtk_logstore_driver = {
	.probe		= mtk_logstore_probe,
	.driver		= {
		.name		= "mtk-logstore",
		.of_match_table	= mtk_logstore_of_match,
	},
};

/*
 * Make sure the ring is live even in an initramfs boot where no platform
 * device has been created for the DT node yet: this runs as an early_initcall
 * and registers the console with CON_PRINTBUFFER, which replays the entire
 * printk ring into the window — the complete boot log, not just the tail.

 */

/*
 * Which framebuffer is the display controller actually reading?
 *
 * LK hands over a live pipe pointed at one of two addresses: 0x78e0c000 (the
 * one our simplefb/DT use) or 0x77a20000 (the vendor tree's "framebuffer"
 * reserved region).  The stripes encode this as a colour, but a register read is
 * exact, and it decides whether fbcon's output is visible at all - if the live
 * buffer is not the one in /chosen, the panel keeps showing LK's splash while
 * the kernel runs happily.
 *
 * The offset is the vendor driver's own name for it: mtk_disp_rdma.c defines
 * DISP_REG_RDMA_MEM_START_ADDR = 0x0f00.
 */
#define DISP_RDMA0_BASE			0x14003000U
#define DISP_RDMA1_BASE			0x14103000U
#define DISP_OVL0_BASE			0x14000000U
#define DISP_RDMA_MEM_START_ADDR	0x0f00U
#define DISP_OVL_ADDR_MT8173		0x0f40U

static void __init fb_probe_display(const char *when)
{
	static const struct {
		const char *name;
		u32 base;
		u32 off;
		u32 expect;	/* nonzero: a "is this block powered" canary */
	} regs[] = {
		/* OVL enable: tells us whether the display block answers at all */
		{ "ovl0+0x000c",  DISP_OVL0_BASE,  0x000cU,	1 },
		{ "rdma0+0x0f00", DISP_RDMA0_BASE, DISP_RDMA_MEM_START_ADDR, 0 },
		{ "rdma1+0x0f00", DISP_RDMA1_BASE, DISP_RDMA_MEM_START_ADDR, 0 },
		{ "ovl0+0x0f40",  DISP_OVL0_BASE,  DISP_OVL_ADDR_MT8173, 0 },
		{ "ovl1+0x0f40",  0x14100000U,     DISP_OVL_ADDR_MT8173, 0 },
		{ "disp_rdma4+0x0f00", 0x14005000U, DISP_RDMA_MEM_START_ADDR, 0 },
		{ "mdp_rdma0+0x0f00",  0x1f006000U, DISP_RDMA_MEM_START_ADDR, 0 },
	};
	unsigned int i;

	pr_info("[mtk-fb] --- %s ---\n", when);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		u32 pa = regs[i].base + regs[i].off;
		u32 page = pa & ~(u32)(PAGE_SIZE - 1);
		void *map;
		u32 val;

		map = early_ioremap(page, PAGE_SIZE);
		if (!map) {
			pr_info("[mtk-fb] %s: ioremap failed\n", regs[i].name);
			continue;
		}
		val = readl(map + (pa - page));
		early_iounmap(map, PAGE_SIZE);
		pr_info("[mtk-fb] %s = 0x%08x%s\n", regs[i].name, val,
			val == FB_CAND0 ? "  <- 0x78e0c000, what /chosen uses"
			: val == FB_CAND1 ? "  <- 0x77a20000, NOT what /chosen uses"
			: regs[i].expect ? "  (canary: block is alive)" : "");
	}
}

static int __init mtk_logstore_early_init(void)
{
	u32 low = 0;
	u32 high = 0;
	struct logstore *l;

	l = READ_ONCE(ls);
	if (l && l != &boot_ls && l != &early_ls)
		return 0;
	if (l) {
		low = l->low_off;
		high = l->high_off;
	}
	fb_note(0);
	fb_probe_display("early, right after the logstore early_initcall");
	if (logstore_setup(&early_ls, LOGSTORE_PA, LOGSTORE_SIZE, NULL))
		return 0;
	/* continue where the earlier writer stopped, never go backwards */
	if (early_ls.low_off < low && low <= LOGSTORE_LOW_END)
		early_ls.low_off = low;
	if (early_ls.high_off < high && high <= LOGSTORE_HIGH_END)
		early_ls.high_off = high;
	register_console(&early_ls.con);
	fb_note(7);
	pr_info("windows at +0x%x and +0x%x of 0x%x/0x%x used\n",
		LOGSTORE_LOW_START, LOGSTORE_HIGH_START,
		early_ls.low_off, early_ls.high_off);
	fb_note(8);
	return 0;
}
early_initcall(mtk_logstore_early_init);


/*
 * The ring as a readable black box.
 *
 * This ring survives a watchdog hard reset - that is exactly why the bootloader
 * keeps its own log there - so after an unexpected reboot it is the only place
 * the previous boot's last words still exist.  LK only relays a fixed prefix of
 * it into expdb, and that relay is unreliable on this board, so expose the
 * windows directly instead: the initramfs shell can grep them.
 *
 * NUL runs collapse into a single newline.  The ring is zero-filled and only
 * the used part holds text, so this makes it read as lines without inventing
 * any bytes; the unused tail becomes one blank line rather than 60 KiB of them.
 */
struct logstore_window {
	const char *name;
	u32 start;
	u32 end;
};

static const struct logstore_window logstore_windows[] = {
	{ "low",  LOGSTORE_LOW_START,  LOGSTORE_LOW_END  },
	{ "high", LOGSTORE_HIGH_START, LOGSTORE_HIGH_END },
};

static ssize_t logstore_window_read(struct file *file, char __user *to,
				    size_t count, loff_t *ppos)
{
	const struct logstore_window *w = file->private_data;
	struct logstore *l = READ_ONCE(ls);
	loff_t pos = *ppos;
	size_t span, i, n = 0;
	u8 *tmp;
	bool last_nl = true;

	if (!l || !l->buf)
		return -ENODEV;

	span = w->end - w->start;
	if (pos < 0 || pos >= span)
		return 0;
	if (count > span - pos)
		count = span - pos;

	tmp = kmalloc(count, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		u8 c = ((const u8 *)l->buf)[w->start + pos + i];

		if (!c) {
			if (last_nl)
				continue;
			c = '\n';
		}
		last_nl = (c == '\n');
		tmp[n++] = c;
	}
	if (copy_to_user(to, tmp, n))
		n = 0;
	kfree(tmp);
	*ppos = pos + count;
	return n;
}

static const struct file_operations logstore_window_fops = {
	.owner = THIS_MODULE,
	.read = logstore_window_read,
	.open = simple_open,
	.llseek = default_llseek,
};

static int __init logstore_debugfs_init(void)
{
	struct dentry *dir;
	unsigned int i;

	dir = debugfs_create_dir("logstore", NULL);
	if (IS_ERR_OR_NULL(dir))
		return 0;
	for (i = 0; i < ARRAY_SIZE(logstore_windows); i++)
		debugfs_create_file(logstore_windows[i].name, 0444, dir,
				    (void *)&logstore_windows[i],
				    &logstore_window_fops);
	debugfs_create_bool("fb_mirror", 0600, dir, &fb_mirror_on);
	return 0;
}
late_initcall(logstore_debugfs_init);
/*
 * Start the console mirror from a late initcall, not from the early one: by then
 * the workqueue threads exist and ioremap_wc() has vmalloc, so the first copy a
 * second later already carries the whole boot log across.  Doing it early was the
 * mistake that cost a boot - the work ran before the mapping was usable.
 */
static int __init logstore_fb_mirror_init(void)
{
	INIT_DELAYED_WORK(&fb_mirror_work, fb_mirror_fn);
	schedule_delayed_work(&fb_mirror_work, msecs_to_jiffies(1000));
	return 0;
}
late_initcall(logstore_fb_mirror_init);


static int __init mtk_logstore_driver_init(void)
{
	return platform_driver_register(&mtk_logstore_driver);
}
module_init(mtk_logstore_driver_init);

MODULE_DESCRIPTION("MediaTek log_store console (boot log recovery via expdb)");
MODULE_LICENSE("GPL");
