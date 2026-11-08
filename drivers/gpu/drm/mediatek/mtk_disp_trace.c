// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek display bring-up trace buffer.
 *
 * The MT6893 (chopin) port hangs inside the DSI/panel bring-up path, and dmesg
 * is not a reliable channel there: the transfer traces we added as dev_info()
 * never appeared even though printk itself kept working (the shell's sampler
 * logged a heartbeat at the same time), and the tasks dump showed workers
 * stuck in D state in __flush_work / drm_client_dev_hotplug.  Anything that
 * has to take a lock to be observed can be the thing that is deadlocked.
 *
 * So this is deliberately dumb and lock-free: a small fixed ring in memory
 * that a caller appends to with a trylock (never blocking, never allocating),
 * read back through debugfs from a shell that is not part of the DRM stack.
 *
 * Trace marks are meant to be temporary bring-up scaffolding.
 */

#include <linux/debugfs.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "mtk_disp_trace.h"

#define MTK_DISP_TRACE_ENTRIES	128
#define MTK_DISP_TRACE_MSG	48

struct mtk_disp_trace_ev {
	u64 t;
	char msg[MTK_DISP_TRACE_MSG];
};

static struct mtk_disp_trace_ev mtk_disp_trace_ring[MTK_DISP_TRACE_ENTRIES];
static DEFINE_SPINLOCK(mtk_disp_trace_lock);
static unsigned int mtk_disp_trace_head;	/* next slot to write */
static unsigned int mtk_disp_trace_total;
static struct dentry *mtk_disp_trace_dentry;
static struct delayed_work mtk_disp_trace_dump_work;
static unsigned int mtk_disp_trace_dumped;
static bool mtk_disp_trace_work_armed;

static void mtk_disp_trace_dump_work_fn(struct work_struct *work);

void mtk_disp_trace(const char *fmt, ...)
{
	struct mtk_disp_trace_ev *ev;
	unsigned long flags;
	va_list ap;
	unsigned int idx;
	bool arm = false;

	if (!spin_trylock_irqsave(&mtk_disp_trace_lock, flags))
		return;

	idx = mtk_disp_trace_head % MTK_DISP_TRACE_ENTRIES;
	ev = &mtk_disp_trace_ring[idx];
	ev->t = ktime_get_ns();
	va_start(ap, fmt);
	vsnprintf(ev->msg, sizeof(ev->msg), fmt, ap);
	va_end(ap);

	mtk_disp_trace_head++;
	mtk_disp_trace_total++;
	if (!mtk_disp_trace_work_armed) {
		mtk_disp_trace_work_armed = true;
		arm = true;
	}

	spin_unlock_irqrestore(&mtk_disp_trace_lock, flags);

	if (arm) {
		INIT_DELAYED_WORK(&mtk_disp_trace_dump_work,
				  mtk_disp_trace_dump_work_fn);
		schedule_delayed_work(&mtk_disp_trace_dump_work, 2 * HZ);
	}
}
EXPORT_SYMBOL_GPL(mtk_disp_trace);

static int mtk_disp_trace_show(struct seq_file *s, void *data)
{
	unsigned int total, start, i;

	spin_lock_irq(&mtk_disp_trace_lock);
	total = mtk_disp_trace_total;
	start = total > MTK_DISP_TRACE_ENTRIES ? total - MTK_DISP_TRACE_ENTRIES : 0;
	for (i = start; i < total; i++) {
		struct mtk_disp_trace_ev *ev =
			&mtk_disp_trace_ring[i % MTK_DISP_TRACE_ENTRIES];

		seq_printf(s, "[%5llu.%06llu] %s\n",
			   div_u64(ev->t, NSEC_PER_SEC),
			   div_u64(ev->t % NSEC_PER_SEC, NSEC_PER_USEC),
			   ev->msg);
	}
	seq_printf(s, "(head=%u total=%u)\n", mtk_disp_trace_head, total);
	spin_unlock_irq(&mtk_disp_trace_lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_disp_trace);

/*
 * Print whatever is new in the ring to the kernel log every couple of seconds.
 *
 * This exists because debugfs has failed us twice on this board (the register
 * dump and then this trace file both came back "(missing)" from the shell),
 * while printk demonstrably works - the shell's own sampler kept logging
 * heartbeats at 34 s while the modeset worker sat in D state.  So the trace
 * data is pushed into dmesg as well, where `dmesg mtkdisp` can read it, and it
 * only prints when something new arrived.
 */
static void mtk_disp_trace_dump_work_fn(struct work_struct *work)
{
	unsigned int total, i;

	spin_lock_irq(&mtk_disp_trace_lock);
	total = mtk_disp_trace_total;
	/*
	 * If the writer got more than a ring ahead (it can, in a flood), skip
	 * forward: slots older than MTK_DISP_TRACE_ENTRIES have been
	 * overwritten, so printing them would replay stale entries with
	 * non-monotonic timestamps.
	 */
	if (total - mtk_disp_trace_dumped > MTK_DISP_TRACE_ENTRIES)
		mtk_disp_trace_dumped = total - MTK_DISP_TRACE_ENTRIES;
	for (i = mtk_disp_trace_dumped; i < total; i++) {
		struct mtk_disp_trace_ev *ev =
			&mtk_disp_trace_ring[i % MTK_DISP_TRACE_ENTRIES];

		pr_info("mtkdisp %5llu.%06llu %s\n",
			div_u64(ev->t, NSEC_PER_SEC),
			div_u64(ev->t % NSEC_PER_SEC, NSEC_PER_USEC), ev->msg);
	}
	mtk_disp_trace_dumped = total;
	spin_unlock_irq(&mtk_disp_trace_lock);

	schedule_delayed_work(&mtk_disp_trace_dump_work, 2 * HZ);
}

void mtk_disp_trace_init(void)
{
	if (mtk_disp_trace_dentry)
		return;

	mtk_disp_trace_dentry = debugfs_create_file("mtk-disp-trace", 0444, NULL,
						    NULL, &mtk_disp_trace_fops);
	pr_info("mtkdisp: debugfs entry %s\n",
		IS_ERR_OR_NULL(mtk_disp_trace_dentry) ? "FAILED" : "created");
}
EXPORT_SYMBOL_GPL(mtk_disp_trace_init);
