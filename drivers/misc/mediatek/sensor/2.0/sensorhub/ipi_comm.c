// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 *
 * SCP sensor-hub IPI transport. Control transfers are request/response over
 * IPI_IN_SENSOR_CTRL; notifications are one-way over IPI_IN_SENSOR_NOTIFY.
 *
 * A single control transfer is in flight at a time (hw_transfer), matching the
 * firmware's one-command-at-a-time ACK model. Callers must serialise control
 * commands; sensor_comm_ctrl_send() does so via its retry loop, and the
 * higher-level callers hold their own mutexes.
 */

#define pr_fmt(fmt) "ipi_comm " fmt

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/soc/mediatek/mtk_tinysys_ipi.h>

#include "scp.h"
#include <linux/debugfs.h>
#include "sensorhub.h"
#include "ipi_comm.h"

struct ipi_controller {
	spinlock_t lock;
	bool running;
	struct list_head head;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	void (*notify_callback)(int id, void *data, unsigned int len);
};

struct ipi_hw_transfer {
	struct completion done;
	int count;
	/* data buffers */
	int id;
	const unsigned char *tx;
	unsigned char *rx;
	unsigned int tx_len;
	unsigned int rx_len;
	void *context;
};

#define ipi_len(x) (((x) + MBOX_SLOT_SIZE - 1) / MBOX_SLOT_SIZE)

static struct ipi_controller controller;
static struct ipi_hw_transfer hw_transfer;
static DEFINE_SPINLOCK(hw_transfer_lock);
static bool handlers_ready;
static uint8_t ctrl_payload[PIN_IN_SIZE_SENSOR_CTRL * MBOX_SLOT_SIZE];
static uint8_t notify_payload[PIN_IN_SIZE_SENSOR_NOTIFY * MBOX_SLOT_SIZE];

/*
 * The DT has to advertise IPI 39/40 (elliptic ultra) so that
 * mbox_setup_pin_table() assigns the sensor pins the same IRQ-status bits the
 * firmware uses. No Linux client consumes that channel, but an unregistered
 * pin_buf would trip mtk_mbox_isr()'s BUG_ON if the firmware ever sent on it,
 * so it gets a discard handler.
 */
#define IPI_IN_ELLIPTIC_ULTRA_0 40
static uint8_t elliptic_payload[16 * MBOX_SLOT_SIZE];

/*
 * Sensor commands do not travel on their own mbox pin here.
 *
 * The chopin firmware (tinysys-scp-RV33_A, 2023-11-02) only publishes the
 * SCP_MPOOL pins (11/24) in its mbox table: its ISR answers a message on the
 * sensor-ctrl pin with "[MBOX ISR]pin table err" (mbox2 bit17, which is where
 * mbox_setup_pin_table() puts a 34-slots mpool pin followed by a 16-slots
 * sensor pin in is64d mode).  So hand the payload to the legacy mpool
 * transport, which prefixes [u32 id][u32 len] and pushes the whole thing
 * through IPI_OUT_SCP_MPOOL_0/1; the firmware's mpool dispatcher then routes
 * it by that id.
 */
/*
 * The firmware answers a sensor command that arrives on SCP_MPOOL addressed to
 * IPI_SENSOR with "contextHubHandleIpiRxEvent cannot find cmd" for some
 * framings and silently for others, so rotate the framing from userspace:
 *
 *   0: [u32 IPI_SENSOR][u32 len] + struct sensor_comm_ctrl
 *   1: same, with sensor_type/command swapped
 *   2: [u32 IPI_SENSOR]           + struct sensor_comm_ctrl
 */
static unsigned int chre_hdr_mode;
module_param(chre_hdr_mode, uint, 0644);
MODULE_PARM_DESC(chre_hdr_mode,
	"CHRE enable packet framing: 0 = bare ConfigCmd, 1 = {0x31,seq,0x1091,len} header + ConfigCmd");

static unsigned int mpool_mode;
module_param(mpool_mode, uint, 0644);
MODULE_PARM_DESC(mpool_mode,
	"SCP_MPOOL sensor payload framing: 0=[u32 id][u32 len]+ctrl, 1=swapped ctrl, 2=[u32 id]+ctrl");

/*
 * Debug switch: stop sending anything to the SCP and only poll its DRAM FIFO.
 *
 * reB.md showed pkt_id 1/3 land in a CHRE parser that spins on anything it does
 * not recognise, so "does the patched firmware sample on its own" cannot be
 * answered while we keep talking to it.  With this set, the bring-up writes the
 * FIFO header and then sends nothing; whether samples appear can be read from
 * EINT12 status and the firmware's own log ring (0x8F1CA700), not from wp
 * (the firmware never learns our base without SET_CONFIG).
 */
static bool chsensor_zero_ipi;
module_param(chsensor_zero_ipi, bool, 0644);
MODULE_PARM_DESC(chsensor_zero_ipi,
	"skip every control packet to the SCP; poll the DRAM FIFO only");

/*
 * Which mpool packet id carries SCP_SENSOR_HUB_DATA.
 *
 * enum ipi_id is IPI_MPOOL=0, IPI_CHRE=1, IPI_CHREX=2, IPI_SENSOR=3,
 * IPI_SENSOR_INIT_START=4, and reB.md found that id 3 dispatches into
 * "chre_fw_ipi" (0x3b3ee) - a CHRE message parser - not into the SET_CONFIG
 * handler at 0x3b00e.  A CHRE parser that rejects our 48-byte SDATA payload
 * explains why the firmware log never shows SET_CONFIG's own
 * "get dram phy addr=...,size=...,maxEventNumber:..." line: the packet is
 * delivered but decoded by the wrong consumer.
 *
 * The id is therefore a runtime knob: sweep it and watch the firmware log for
 * that line instead of guessing.
 */
static unsigned int chsensor_pkt_id = IPI_SENSOR;
module_param(chsensor_pkt_id, uint, 0644);
MODULE_PARM_DESC(chsensor_pkt_id,
	"mpool packet id for SCP_SENSOR_HUB_DATA (0=MPOOL 1=CHRE 2=CHREX 3=SENSOR 4=INIT_START)");

static int mpool_send_pkt(unsigned int pkt_id, const void *tx, int tx_len,
			  unsigned int mode)
{
	/*
	 * 8 bytes of header + payload; the mpool pin is 34 slots = 136 bytes.
	 *
	 * This buffer must stay on the stack.  The helper is reached from the
	 * enable/getdata delayed works (system_wq, possibly on different CPUs)
	 * and, through ipi_comm_sync() -> ipi_prefetch_messages(), from the
	 * caller's own context; a function-static buffer would be rewritten
	 * while mtk_ipi_send() is still reading it and the SCP would get a
	 * merged or truncated packet.
	 */
	uint8_t pkt[128 + sizeof(uint32_t) * 2];
	uint32_t hdr[2] = { (uint32_t)pkt_id, (uint32_t)tx_len };
	unsigned int hlen = (mode == 2) ? sizeof(uint32_t) : sizeof(hdr);
	int ret, retry = 0;

	if (READ_ONCE(chsensor_zero_ipi))
		return 0;

	if (tx_len < 0 || tx_len + sizeof(hdr) > sizeof(pkt))
		return -EINVAL;

	memcpy(pkt, hdr, hlen);
	if (mode == 1) {
		/* command byte first, then the sensor type */
		pkt[hlen] = tx_len > 1 ? ((const uint8_t *)tx)[1] : 0;
		pkt[hlen + 1] = tx_len > 0 ? ((const uint8_t *)tx)[0] : 0;
		if (tx_len > 2)
			memcpy(pkt + hlen + 2, (const uint8_t *)tx + 2, tx_len - 2);
	} else if (tx_len > 0) {
		memcpy(pkt + hlen, tx, tx_len);
	}

	do {
		ret = mtk_ipi_send(&scp_ipidev, IPI_OUT_SCP_MPOOL_0, 0, pkt,
				   ipi_len(tx_len + hlen), 0);
		if (ret < 0 && ret != IPI_PIN_BUSY)
			return -EIO;
		if (ret == IPI_PIN_BUSY) {
			if (retry++ >= 1000)
				return -EBUSY;
			if (retry % 100 == 0)
				usleep_range(1000, 2000);
		}
	} while (ret == IPI_PIN_BUSY);

	return 0;
}

/*
 * If nothing ever answers, tell the sensor stack that the AP is starting:
 * the firmware id space has IPI_SENSOR_INIT_START next to IPI_SENSOR, and the
 * vendor stack sends it once before the first control command.
 */
static bool init_start_sent;

/*
 * The sample FIFO the firmware drains into.
 *
 * phys(SENS_MEM_ID) is 0x8F0CA700 for 1 MB ("@@@@ reserved" gives SENS_MEM id 2
 * = 0x100000, cumulative start 0x8F0CA700; the SCP logger follows at
 * 0x8F1CA700).  The base is a 4 KB-aligned page inside that block rather than
 * its very start, because the drain is patched with a single `lui` holding the
 * page address (patchscp group 5) - see scp-chopin.c.  Going through
 * SET_CONFIG is not an option: pkt_id 3 lands in an empty legacy handler and
 * the firmware then refuses to write at all ("dram info not got, exit write
 * dram").
 *
 * Everything below is derived from these two constants, and FIFOSize must stay
 * a multiple of 44 and must not run into the logger at 0x8F1CA700.
 */
#define SENS_MEM_BASE	0x8F0CA700
#define SENS_MEM_SIZE	0x100000
#define SENS_FIFO_BASE	0x8F0CD000	/* page-aligned, inside SENS_MEM */
#define SENS_FIFO_END	(SENS_MEM_BASE + SENS_MEM_SIZE)
/* One-shot guard for the DRAM-FIFO bring-up: ipi_retry_transfer() can be
 * reached concurrently from workqueue and syscall context, and running the
 * SET_CONFIG sequence twice sends two different FIFO bases to the firmware. */
static atomic_t mpool_bringup_done = ATOMIC_INIT(0);
static const uint32_t init_start_payload[2];

static void mpool_send_init_start(void)
{
	int ret;

	if (READ_ONCE(init_start_sent))
		return;
	WRITE_ONCE(init_start_sent, true);

	ret = mpool_send_pkt(IPI_SENSOR_INIT_START, init_start_payload,
			     sizeof(init_start_payload), mpool_mode % 3);
	pr_info("init-start sent (%d)\n", ret);
}

/*
 * reD.md: the rate path is pkt_id 1 (AOSP nanoHub "comms", reason 0x1091 =
 * CMD_COMMS_WRITE); payload[0] is the event id and 0x300 =
 * EVT_NO_SENSOR_CONFIG_EVENT hands the vendor ConfigCmd to the sensor.  The
 * SCP validates len == packet.len + 14 and a crc32 over the payload, so the
 * frame below (seq 1, accel, 100 Hz, ENABLE, crc 0x31ea7032) is sent verbatim.
 */
static unsigned int chsensor_delay_ms = 5000;
module_param(chsensor_delay_ms, uint, 0644);
MODULE_PARM_DESC(chsensor_delay_ms, "delay before the first nanoHub enable packet");

static void chsensor_getdata_work(struct work_struct *w);
static struct delayed_work chsensor_getdata_dw;
static unsigned int chsensor_getdata_n;

/* reC.md: pkt_id 3 action 2 is SENSOR_HUB_GET_DATA.  Poll it a few times:
 * if the reply carries a sample the chip read path works and only the
 * interrupt/batching is missing. */
static void chsensor_getdata_work(struct work_struct *w)
{
	uint8_t pkt[48] = { 0 };

	pkt[0] = 1;	/* accelerometer */
	pkt[1] = 2;	/* SENSOR_HUB_GET_DATA */
	pr_info("chsensor: GET_DATA %u\n", chsensor_getdata_n);
	mpool_send_pkt(chsensor_pkt_id, pkt, sizeof(pkt), mpool_mode % 3);
	if (++chsensor_getdata_n < 5)
		schedule_delayed_work(&chsensor_getdata_dw, msecs_to_jiffies(500));
}

static void chsensor_poll_work(struct work_struct *w);
static struct delayed_work chsensor_poll_dw;
static unsigned int chsensor_poll_n;

/* Sample EINT12 status and the sensor FIFO together so one boot shows
 * whether an interrupt, an ack and a FIFO write line up. */
static void chsensor_poll_work(struct work_struct *w)
{
	void __iomem *e = ioremap(0x1000B000, 0x1000);
	void __iomem *f = ioremap_wc(SENS_FIFO_BASE, 0x1000);

	if (e && f)
		pr_info("chsensor poll %u: stat=%08x pol=%08x fifo rp=%u wp=%u\n",
			chsensor_poll_n, readl(e + 0x000), readl(e + 0x300),
			readl(f + 0x00), readl(f + 0x04));
	if (e)
		iounmap(e);
	if (f)
		iounmap(f);
	if (++chsensor_poll_n < 20)
		schedule_delayed_work(&chsensor_poll_dw, msecs_to_jiffies(100));
}

static void chsensor_logger_work(struct work_struct *w);
static struct delayed_work chsensor_logger_dw;
static unsigned int chsensor_logger_tries;

static void chsensor_logger_work(struct work_struct *w)
{
	scp_logger_kick();
	pr_info("chsensor: scp logger kick %u\n", ++chsensor_logger_tries);
	if (chsensor_logger_tries < 5)
		schedule_delayed_work(&chsensor_logger_dw, msecs_to_jiffies(3000));
}

/*
 * FIFO -> IIO dispatch (reK.md): the SCP writes 44-byte data_unit_t records
 * at 0x8F0CA700 + 0x10 + wp and stores the new wp back into the header, so
 * poll the header and forward every new record to the IIO layer.
 */
#define SENS_RECORD_SIZE	44
/* Room left between our header and the SCP logger block, rounded down to whole
 * records: the firmware's wrap is remu(wp + len, FIFOSize), so this must be a
 * multiple of 44 or every record after the first wrap is misaligned. */
#define SENS_FIFO_BYTES		(((SENS_FIFO_END - SENS_FIFO_BASE - 0x10) / \
				  SENS_RECORD_SIZE) * SENS_RECORD_SIZE)
#define SENS_FIFO_MAP_SIZE	(SENS_FIFO_BYTES + 0x10)

static void chsensor_fifo_work(struct work_struct *w);
static struct delayed_work chsensor_fifo_dw;
static struct delayed_work chsensor_enable_dw;
static struct delayed_work chsensor_poll_dw;
static struct delayed_work chsensor_getdata_dw;
static struct delayed_work chsensor_logger_dw;

static void __iomem *chsensor_fifo_map;
static u32 chsensor_fifo_last;		/* byte offset of the next record */
static bool chsensor_fifo_last_valid;
static bool chsensor_fifo_flowing;	/* at least one record was forwarded */
static unsigned long chsensor_fifo_repair_at;

/*
 * (Re)initialise the FIFO header the firmware reads at SET_CONFIG time.
 * The block is phys(SENS_MEM_ID), which our own sensorhub stack also hands to
 * share_mem_init() as the SENS ring with item_size 36, so the header can be
 * rewritten behind our back; everything that needs a known-good header goes
 * through here.
 */
static void chsensor_fifo_write_header(void __iomem *f)
{
	writel(0, f + 0x00);			/* rp */
	writel(0, f + 0x04);			/* wp */
	writel(SENS_FIFO_BYTES, f + 0x08);	/* FIFOSize, multiple of 44 */
	writel(0, f + 0x0c);			/* reserve */

	chsensor_fifo_last = 0;
	chsensor_fifo_last_valid = true;
}

static void chsensor_fifo_work(struct work_struct *w)
{
	void __iomem *f;
	u32 size, wp, delta, n = 0;
	int i;

	/*
	 * Never touch SCP memory while the core is down: it is reset by the
	 * debugfs helper (and by recovery), and a read against a dead SCP
	 * region faults the AP.
	 */
	if (!is_scp_ready(SCP_A_ID)) {
		chsensor_fifo_last_valid = false;
		goto out;
	}

	f = chsensor_fifo_map;
	if (!f) {
		f = ioremap_wc(SENS_FIFO_BASE, SENS_FIFO_MAP_SIZE);
		if (!f)
			goto out;
		chsensor_fifo_map = f;
		chsensor_fifo_last_valid = false;
	}

	size = readl(f + 0x08);
	wp = readl(f + 0x04);

	/*
	 * Geometry check.  When share_mem_config() runs (sensorhub_prime() does
	 * it on every SCP ready) it overwrites this same 16-byte header with
	 * the SENS ring geometry: item_size 36 and buffer_size
	 * (blocksize - 16) / 36 * 36, plus wp = rp = 0.  The firmware keeps
	 * writing 44-byte records, so its wrap and ours no longer agree and
	 * every record after the first lap is a mix of two samples: the axis
	 * values stay plausible but wrong, and they change on every boot.
	 * Detect that instead of forwarding garbage, and put the header back.
	 */
	if (!size || size % SENS_RECORD_SIZE || wp >= size) {
		/*
		 * Rate-limit the repair rather than capping it: one clobber is
		 * always worth fixing, but a writer that fights us every pass
		 * must not turn the 100 ms poll into a write storm.
		 */
		if (time_after(jiffies, chsensor_fifo_repair_at + HZ / 2)) {
			chsensor_fifo_repair_at = jiffies;
			pr_warn("fifo header bad (size %u wp %u), rewriting to %u\n",
				size, wp, SENS_FIFO_BYTES);
			chsensor_fifo_write_header(f);
		}
		goto out;
	}

	if (!chsensor_fifo_last_valid) {
		/* First pass after a (re)map: start from wherever the firmware
		 * is now instead of replaying stale DRAM from a previous
		 * session. */
		chsensor_fifo_last = wp;
		chsensor_fifo_last_valid = true;
		goto out;
	}

	delta = (wp + size - chsensor_fifo_last) % size;
	if (delta > size / 2) {
		/* wp went backwards: the header was reset under us.  Resync
		 * rather than wrapping through records nobody wrote. */
		chsensor_fifo_last = wp;
		delta = 0;
	}
	delta -= delta % SENS_RECORD_SIZE;

	/* forward at most 16 records per pass (100 Hz -> ~2 per 20 ms) */
	for (i = 0; i < 16 && delta; i++) {
		u32 o = 0x10 + chsensor_fifo_last;
		struct sensorhub_sample s = { 0 };

		/*
		 * reK's layout has sensor_type first, but this firmware build
		 * puts it elsewhere (byte 0 reads 0, the u32 is 0x00010000), so
		 * trust the FIFO we registered: it only carries the accel.
		 */
		s.sensor_type = SENSOR_TYPE_ACCELEROMETER;
		s.timestamp = readl(f + o + 4) | ((s64)readl(f + o + 8) << 32);
		s.value[0] = readl(f + o + 12);
		s.value[1] = readl(f + o + 16);
		s.value[2] = readl(f + o + 20);

		sensorhub_push_sample(&s);

		chsensor_fifo_last += SENS_RECORD_SIZE;
		if (chsensor_fifo_last >= size)
			chsensor_fifo_last -= size;	/* stays aligned */
		delta -= SENS_RECORD_SIZE;
		n++;
	}

	if (n) {
		/*
		 * Publish our read pointer.  This firmware never reads rp
		 * (reK.md 0x3accc-0x3ad76 only loads and stores wp), so this is
		 * not required for flow control, but it keeps the ring
		 * self-describing for the debugfs dump and for any build that
		 * does honour it.
		 */
		writel(chsensor_fifo_last, f + 0x00);
		WRITE_ONCE(chsensor_fifo_flowing, true);
		pr_debug_ratelimited("chsensor: fifo pushed %u records\n", n);
	}
out:
	schedule_delayed_work(&chsensor_fifo_dw, msecs_to_jiffies(100));
}



/* Manual retry: the firmware only starts sampling on some boots, and a
 * reboot is a slow way to try again.  echo 1 > /sys/kernel/debug/chsensor/retry
 *
 * The flag is an atomic because the debugfs write runs in the writer's
 * context while the enable work may be running on another CPU.
 */
static atomic_t chsensor_retry_req;

/*
 * Bring-up state.  Each pass costs an IPI on IPI_OUT_SCP_MPOOL_0, so the budget
 * is hard: the FIFO is registered once per SCP session, at most
 * CHSENSOR_MAX_ATTEMPTS enable passes are sent, and everything stops for good
 * once the FIFO write pointer moves.
 *
 * It has to be resettable from outside the work, because the SCP is restarted
 * by the vendor driver on its own schedule (WDT/AED/awake timeout) and each
 * restart throws away what SET_CONFIG bought: the new firmware instance keeps
 * its stale chre_dram pointer and writes nowhere.  Measured on chopin:
 * SET_CONFIG at 16.3 s, SCP restarted at 30.9 s, fifo->wp 0 for the rest of the
 * boot because the driver thought the FIFO was still registered.
 */
#define CHSENSOR_MAX_ATTEMPTS	6
static unsigned int chsensor_attempt;
static bool chsensor_config_done;
static bool chsensor_seq3_sent;

static void chsensor_reset_state(void)
{
	WRITE_ONCE(chsensor_attempt, 0);
	WRITE_ONCE(chsensor_config_done, false);
	WRITE_ONCE(chsensor_seq3_sent, false);
	WRITE_ONCE(chsensor_fifo_flowing, false);
	WRITE_ONCE(chsensor_fifo_last_valid, false);
	chsensor_fifo_repair_at = 0;
}

static int chsensor_retry_open(struct inode *i, struct file *f) { return 0; }
static ssize_t chsensor_retry_write(struct file *f, const char __user *ubuf,
				    size_t n, loff_t *off)
{
	atomic_set(&chsensor_retry_req, 1);
	schedule_delayed_work(&chsensor_enable_dw, msecs_to_jiffies(100));
	return n;
}
static const struct file_operations chsensor_retry_fops = {
	.owner = THIS_MODULE,
	.open  = chsensor_retry_open,
	.write = chsensor_retry_write,
};

static void chsensor_enable_work(struct work_struct *w);

static int __init chsensor_debugfs_init(void)
{
	struct dentry *d;

	/*
	 * Initialise every delayed work exactly once, here.
	 *
	 * The bring-up sequence used to call INIT_DELAYED_WORK() on works that
	 * could still be queued (the poll/getdata works reschedule themselves
	 * every few hundred ms): re-initialising a pending timer overwrites
	 * its list node and corrupts the timer wheel, which is one of the ways
	 * "retry a few more times" turned into a kernel oops instead of just a
	 * few extra IPIs.
	 */
	INIT_DELAYED_WORK(&chsensor_enable_dw, chsensor_enable_work);
	INIT_DELAYED_WORK(&chsensor_fifo_dw, chsensor_fifo_work);
	INIT_DELAYED_WORK(&chsensor_poll_dw, chsensor_poll_work);
	INIT_DELAYED_WORK(&chsensor_getdata_dw, chsensor_getdata_work);
	INIT_DELAYED_WORK(&chsensor_logger_dw, chsensor_logger_work);

	d = debugfs_create_dir("chsensor", NULL);
	debugfs_create_file("retry", 0200, d, NULL, &chsensor_retry_fops);
	return 0;
}
late_initcall(chsensor_debugfs_init);

static void chsensor_enable_work(struct work_struct *w)
{
	/* frames generated by work/accint/nanohub_pkt.py (firmware crc32) */
	static const uint8_t dis_seq1[34] = {
        0x31, 0x01, 0x00, 0x00, 0x00, 0x91, 0x10, 0x00,
        0x00, 0x14, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90,
        0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0xf8, 0xac,
        0x4b, 0x6a,
	};
	static const uint8_t en_seq2[34] = {
        0x31, 0x02, 0x00, 0x00, 0x00, 0x91, 0x10, 0x00,
        0x00, 0x14, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90,
        0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xa6, 0x32,
        0xb2, 0xea,
	};
	static const uint8_t en_seq3[34] = {
        0x31, 0x03, 0x00, 0x00, 0x00, 0x91, 0x10, 0x00,
        0x00, 0x14, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90,
        0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x2a, 0xf3,
        0x85, 0xa3,
	};
	unsigned int next_ms;
	int ret = 0;

	if (atomic_xchg(&chsensor_retry_req, 0))
		chsensor_reset_state();

	/*
	 * The registry entry and the DRAM FIFO both belong to the SCP: with the
	 * core down the header and the IPI path are invalid, and reading SCP
	 * memory there faults the AP.  Nothing is sent in this state, so
	 * polling for readiness is free; the sequence has to be replayed from
	 * scratch once the core is back because it loses its FIFO pointer.
	 */
	if (!is_scp_ready(SCP_A_ID)) {
		chsensor_reset_state();
		pr_info("chsensor: SCP not ready, bring-up deferred\n");
		schedule_delayed_work(&chsensor_enable_dw,
				      msecs_to_jiffies(2000));
		return;
	}

	/* Once the firmware drains, stop talking to it entirely. */
	if (READ_ONCE(chsensor_fifo_flowing))
		return;

	if (READ_ONCE(chsensor_attempt) >= CHSENSOR_MAX_ATTEMPTS) {
		pr_info("chsensor: bring-up gave up after %u passes\n",
			READ_ONCE(chsensor_attempt));
		return;
	}

	/*
	 * Register the DRAM FIFO once per session, not on every pass: SET_CONFIG
	 * makes the firmware re-map the base, re-read FIFOSize and recompute
	 * maxEventNumber, so repeating it mid-stream only moves the goalposts
	 * (and costs three more unacknowledged IPIs).
	 */
	if (!READ_ONCE(chsensor_config_done)) {
		uint8_t cfg[48];
		u32 base = SENS_FIFO_BASE;
		u32 size = SENS_FIFO_BYTES;
		void __iomem *fifo;
		u64 ts;

		/*
		 * The firmware samples FIFOSize out of the header at SET_CONFIG
		 * time, so the header has to be in place before the packet goes
		 * out.  If it cannot be mapped, tell the firmware nothing yet.
		 */
		fifo = ioremap_wc(SENS_FIFO_BASE, SENS_FIFO_MAP_SIZE);
		if (!fifo) {
			pr_err("chsensor: cannot map the FIFO, retrying\n");
			schedule_delayed_work(&chsensor_enable_dw,
					      msecs_to_jiffies(2000));
			return;
		}
		chsensor_fifo_write_header(fifo);
		iounmap(fifo);

		WRITE_ONCE(chsensor_config_done, true);

		memset(cfg, 0, sizeof(cfg));
		cfg[1] = 5;	/* SET_CUST */
		cfg[4] = 12;	/* CUST_ACTION_GET_SENSOR_INFO */
		ret = mpool_send_pkt(chsensor_pkt_id, cfg, sizeof(cfg),
				     mpool_mode % 3);
		pr_info("chsensor: SET_CUST ret %d\n", ret);

		memset(cfg, 0, sizeof(cfg));
		cfg[1] = 8;	/* SET_TIMESTAMP */
		ts = ktime_get_boottime_ns();
		memcpy(cfg + 16, &ts, sizeof(ts));
		ret = mpool_send_pkt(chsensor_pkt_id, cfg, sizeof(cfg),
				     mpool_mode % 3);
		pr_info("chsensor: SET_TIMESTAMP ret %d\n", ret);
	}

	/*
	 * SET_CONFIG is re-sent on every pass, not once per session.
	 *
	 * The firmware registers its mpool legacy handlers some way into its own
	 * boot.  A packet that arrives before that is answered with
	 * "legacy_handler id 11,pkt_id:N handler is null!!!" and dropped, so
	 * chre_dram and maxEventNumber stay unset and the drain then refuses to
	 * write anything ("dram info not got, exit write dram") - fifo->wp never
	 * moves and no sample ever reaches the kernel.
	 *
	 * That is exactly what the once-per-session version broke: its only shot
	 * left at firmware t~1.7 s, well before registration.  The reason the
	 * older builds produced data "on some boots, after 3-6 tries" is that
	 * they re-sent the whole sequence every pass, so a later attempt landed
	 * after the handler existed.  Keep the retry, and let the firmware's own
	 * log be the judge: once "handler is null" stops appearing, the FIFO is
	 * registered.
	 *
	 * Writing the header stays a one-time thing on purpose: re-zeroing wp
	 * under a running drain is what made the samples look unstable.
	 */
	{
		uint8_t cfg[48];
		u32 base = SENS_FIFO_BASE;
		u32 size = SENS_FIFO_BYTES;

		memset(cfg, 0, sizeof(cfg));
		cfg[1] = 4;	/* action: SET_CONFIG */
		memcpy(cfg + 4, &base, 4);
		memcpy(cfg + 8, &size, 4);
		ret = mpool_send_pkt(chsensor_pkt_id, cfg, sizeof(cfg),
				     mpool_mode % 3);
		pr_info("chsensor: SET_CONFIG fifo 0x%x size %u ret %d (pkt_id %u, pass %u)\n",
			base, size, ret, chsensor_pkt_id,
			READ_ONCE(chsensor_attempt) + 1);
	}

	next_ms = 4000;
	if (!READ_ONCE(chsensor_attempt)) {
		/* force the firmware through a full off->on re-arm */
		pr_info("chsensor: DISABLE accel (seq 1)\n");
		ret = mpool_send_pkt(IPI_CHRE, (void *)dis_seq1, sizeof(dis_seq1),
				     mpool_mode % 3);
		next_ms = 300;
	} else if (READ_ONCE(chsensor_attempt) == 1) {
		pr_info("chsensor: ENABLE accel 100Hz (seq 2, re-arm)\n");
		ret = mpool_send_pkt(IPI_CHRE, (void *)en_seq2, sizeof(en_seq2),
				     mpool_mode % 3);
		schedule_delayed_work(&chsensor_poll_dw, msecs_to_jiffies(200));
		next_ms = chsensor_delay_ms;
	} else {
		pr_info("chsensor: ENABLE accel 100Hz (seq 3, retry %u)\n",
			READ_ONCE(chsensor_attempt));
		ret = mpool_send_pkt(IPI_CHRE, (void *)en_seq3, sizeof(en_seq3),
				     mpool_mode % 3);
		/* the GET_DATA probe is a one-off diagnostic, not per pass */
		if (!READ_ONCE(chsensor_seq3_sent)) {
			WRITE_ONCE(chsensor_seq3_sent, true);
			schedule_delayed_work(&chsensor_getdata_dw,
					      msecs_to_jiffies(1000));
		}
	}
	WRITE_ONCE(chsensor_attempt, READ_ONCE(chsensor_attempt) + 1);
	pr_info("chsensor: send ret %d (attempt %u)\n", ret,
		READ_ONCE(chsensor_attempt));

	if (READ_ONCE(chsensor_attempt) >= CHSENSOR_MAX_ATTEMPTS)
		return;
	schedule_delayed_work(&chsensor_enable_dw, msecs_to_jiffies(next_ms));
}

static inline int ipi_retry_transfer(int id, void *tx, int tx_len)
{
	/*
	 * Do not touch SCP_MPOOL_0 any more.
	 *
	 * work/accint/reB.md shows what happens when we do: the SCP's mbox ISR
	 * runs the pin handler inline and only writes the hardware ack after it
	 * returns, and pkt_id 3 (IPI_SENSOR) lands in "chre_fw_ipi" (0x3b3ee),
	 * a CHRE message parser with an unbounded copy-verify loop over the
	 * single shared slot (0xbe1cc).  Anything that does not look like a
	 * CHRE message keeps that loop spinning inside the ISR, so the ack is
	 * never written and the SCP is wedged for good - which is exactly what
	 * happened (the firmware's own log dies at t=1.050s).
	 *
	 * This firmware has no MTK sensor_comm channel at all (the legacy
	 * dispatch table only knows CHRE/CHREX/chre_fw_ipi/elliptic), so the
	 * MTK sensorhub stack cannot drive it.  Fail fast and keep the SCP
	 * healthy until the CHRE path is implemented.
	 */
	/*
	 * reC.md: the 48-byte pkt_id 3 payload is union SCP_SENSOR_HUB_DATA
	 * {u8 sensorType; u8 action; u8 rsvd[2]; u32 data[11];} and action 4
	 * (SET_CONFIG) does not set a rate: it points the firmware's DRAM
	 * sensor FIFO at data[0] (bufferBase).  The FIFO header is
	 * {u32 rp; u32 wp; u32 FIFOSize; u32 reserve} followed by 44-byte
	 * data units, and the firmware reads FIFOSize from that header.
	 *
	 * This used to send SET_CONFIG (plus SET_CUST and SET_TIMESTAMP) right
	 * here, and that is one of the reasons a whole boot could go without a
	 * single sample: this callback fires as soon as the IPI handlers are
	 * registered, which on chopin is ~0.9 s after the SCP is released, and
	 * the firmware drops everything it receives before its own init has
	 * finished (it logs "len 34 > 269, gRxBuf 0x0" and continues).  The
	 * FIFO is therefore never registered, the firmware keeps its stale
	 * chre_dram pointer, and fifo->wp stays 0 for the rest of the boot.
	 *
	 * All firmware-facing configuration now happens in chsensor_enable_work(),
	 * which is scheduled chsensor_delay_ms later, so the registration and
	 * the enable land together once the firmware is actually listening.
	 */
	if (!atomic_xchg(&mpool_bringup_done, 1)) {
		/*
		 * EINT12 polarity probe: the firmware configures the accel
		 * data-ready line as level + active high, while GPIO12 is held
		 * low by the chip.  Writing POL_CLR from userspace did not
		 * change the read-back, so do it here with the kernel mapping
		 * and print every step (this is the last software-visible knob
		 * on the "no data-ready" path).
		 */
		void __iomem *eint = ioremap(0x1000B000, 0x1000);

		if (eint) {
			pr_info("chsensor: eint12 pol=%08x mask=%08x stat=%08x\n",
				readl(eint + 0x300), readl(eint + 0x080),
				readl(eint + 0x000));
			/* v48 showed pol_set is what makes stat assert:
			 * with pol bit12 = 1 the latched status follows
			 * the line (which the chip holds low).  Keep it
			 * set instead of probing both directions. */
			writel(0x1000, eint + 0x340);	/* pol_set */
			pr_info("chsensor: after pol_set pol=%08x stat=%08x\n",
				readl(eint + 0x300), readl(eint + 0x000));
			iounmap(eint);
		}

		/* The cached read position is meaningless after an SCP restart. */
		chsensor_fifo_last_valid = false;

		/*
		 * The enable must NOT be sent from here: the firmware logs
		 * "[0.080] len 34 > 269, gRxBuf 0x0" when a pkt_id 1 packet
		 * arrives before its CHRE host interface is initialised, and
		 * drops it silently.  Queue it on a delayed work instead so it
		 * lands seconds after the firmware has finished its own init
		 * (its boot log ends at ~1.0 s with "accGyro: init done").
		 *
		 * The works themselves are initialised once, in
		 * chsensor_debugfs_init(); re-initialising them here would be
		 * unsafe once they have been queued.
		 */
		schedule_delayed_work(&chsensor_fifo_dw, msecs_to_jiffies(200));
		/* keep retrying until samples arrive (bounded by the attempt cap) */
		pr_info("chsensor: bring-up pass 1\n");
		schedule_delayed_work(&chsensor_logger_dw, msecs_to_jiffies(1000));
		pr_info("chsensor: ENABLE queued in %u ms\n", chsensor_delay_ms);
		schedule_delayed_work(&chsensor_enable_dw,
				      msecs_to_jiffies(chsensor_delay_ms));
	}


	pr_debug_ratelimited("sensor %d: MTK sensor_comm disabled (CHRE-only firmware)\n",
			    id);

	return -ENODEV;
}

static int __maybe_unused mpool_send_legacy(int id, void *tx, int tx_len)
{
	unsigned int mode = mpool_mode % 3;
	int ret, awake_ret;

	pr_info("mpool framing %u: ch %d, %d bytes, mbox2 recv irq 0x%x send irq 0x%x\n",
		mode, id, tx_len,
		mtk_mbox_read_recv_irq(&scp_mboxdev, 2),
		mtk_mbox_check_send_irq(&scp_mboxdev, 2, 0));

	/*
	 * The SCP only services the mailbox while it is awake: v36 showed the
	 * send IRQ being raised and never consumed, with no firmware log at
	 * all.  Assert the awake handshake around the write so the core is out
	 * of its idle state when the interrupt arrives.  This is the advisory
	 * variant of scp_awake_lock(): it pokes INFRA_IRQ_SET and does not wait
	 * for an ack, which this firmware never sends.
	 */
	awake_ret = scp_awake_lock((void *)SCP_A_ID);
	ret = mpool_send_pkt(IPI_SENSOR, tx, tx_len, mode);
	scp_awake_unlock((void *)SCP_A_ID);

	pr_info("mpool framing %u: awake %d send ret %d, after: recv irq 0x%x send irq 0x%x\n",
		mode, awake_ret, ret,
		mtk_mbox_read_recv_irq(&scp_mboxdev, 2),
		mtk_mbox_check_send_irq(&scp_mboxdev, 2, 0));

	return ret;
}

static int ipi_transfer_buffer(struct ipi_transfer *t)
{
	int ret = 0;
	int timeout;
	unsigned long flags;
	struct ipi_hw_transfer *hw = &hw_transfer;

	spin_lock_irqsave(&hw_transfer_lock, flags);
	hw->id = t->id;
	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->tx_len = t->tx_len;
	hw->rx_len = t->rx_len;

	reinit_completion(&hw->done);
	hw->context = &hw->done;
	spin_unlock_irqrestore(&hw_transfer_lock, flags);

	ret = ipi_retry_transfer(hw->id, (void *)hw->tx, hw->tx_len);
	if (ret < 0)
		return ret;

	/*
	 * The firmware answers sensor commands on the mpool transport, but it
	 * does so late: with a 100 ms budget every transfer timed out and the
	 * late reply was thrown away ("ipi_comm dropped transfer").  Give it a
	 * full second, which is also what the vendor's ipi_comm waits.
	 */
	timeout = wait_for_completion_timeout(&hw->done,
			msecs_to_jiffies(1000));
	spin_lock_irqsave(&hw_transfer_lock, flags);
	if (!timeout) {
		hw->count = -ETIMEDOUT;
		pr_err_ratelimited("timeout %u %u %u %u %u\n",
			hw->tx[0], hw->tx[1], hw->tx[2],
			hw->tx[3], hw->tx[4]);
	}
	hw->context = NULL;
	spin_unlock_irqrestore(&hw_transfer_lock, flags);

	return hw->count;
}

static void ipi_complete(void *arg)
{
	complete(arg);
}

static void ipi_transfer_messages(void)
{
	struct ipi_message *m;
	struct ipi_transfer *t = NULL;
	int status = 0;
	unsigned long flags;

	spin_lock_irqsave(&controller.lock, flags);
	if (list_empty(&controller.head) || controller.running)
		goto out;
	controller.running = true;
	while (!list_empty(&controller.head)) {
		m = list_first_entry(&controller.head,
			struct ipi_message, list);
		list_del(&m->list);
		spin_unlock_irqrestore(&controller.lock, flags);
		list_for_each_entry(t, &m->transfers, transfer_list) {
			if (!t->tx_buf && t->tx_len) {
				status = -EINVAL;
				break;
			}
			if (t->tx_len)
				status = ipi_transfer_buffer(t);
			if (status < 0) {
				break;
			} else if (status != t->rx_len) {
				status = -EBADMSG;
				break;
			}
			status = 0;
		}
		m->status = status;
		m->complete(m->context);
		spin_lock_irqsave(&controller.lock, flags);
	}
	controller.running = false;
out:
	spin_unlock_irqrestore(&controller.lock, flags);
}

static void ipi_prefetch_messages(void)
{
	ipi_transfer_messages();
}

static void ipi_work(struct work_struct *work)
{
	ipi_transfer_messages();
}

static int __ipi_transfer(struct ipi_message *m)
{
	unsigned long flags;

	m->status = -EINPROGRESS;

	spin_lock_irqsave(&controller.lock, flags);
	list_add_tail(&m->list, &controller.head);
	queue_work(controller.workqueue, &controller.work);
	spin_unlock_irqrestore(&controller.lock, flags);
	return 0;
}

static int __ipi_xfer(struct ipi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = ipi_complete;
	message->context = &done;

	status = __ipi_transfer(message);

	if (status == 0) {
		ipi_prefetch_messages();
		wait_for_completion(&done);
		status = message->status;
	}
	message->context = NULL;
	return status;
}

static int ipi_sync(int id, const unsigned char *txbuf, unsigned int n_tx,
		unsigned char *rxbuf, unsigned int n_rx)
{
	struct ipi_transfer t;
	struct ipi_message m;

	t.id = id;
	t.tx_buf = txbuf;
	t.tx_len = n_tx;
	t.rx_buf = rxbuf;
	t.rx_len = n_rx;

	ipi_message_init(&m);
	ipi_message_add_tail(&t, &m);

	return __ipi_xfer(&m);
}

static int ipi_async(struct ipi_message *m)
{
	return __ipi_transfer(m);
}

int ipi_comm_sync(int id, unsigned char *tx, unsigned int n_tx,
		unsigned char *rx, unsigned int n_rx)
{
	/*
	 * A synchronous transfer can only complete if the receive callback is
	 * registered. Without it the SCP's answer would be dropped and the
	 * caller would burn its full retry budget waiting for a completion that
	 * can never arrive, so fail immediately instead.
	 */
	if (!READ_ONCE(handlers_ready))
		return -ENODEV;

	return ipi_sync(id, tx, n_tx, rx, n_rx);
}

int ipi_comm_async(struct ipi_message *m)
{
	return ipi_async(m);
}

int ipi_comm_noack(int id, unsigned char *tx, unsigned int n_tx)
{
	return ipi_retry_transfer(id, tx, n_tx);
}

static void ipi_comm_complete(unsigned char *buffer, unsigned int len)
{
	struct ipi_hw_transfer *hw = &hw_transfer;

	spin_lock(&hw_transfer_lock);
	if (!hw->context) {
		pr_err("dropped transfer\n");
		goto out;
	}
	/* only copy hw->rx_len bytes to hw->rx to avoid memory corruption */
	memcpy(hw->rx, buffer, hw->rx_len);
	/* hw->count give real len */
	hw->count = len;
	complete(hw->context);
out:
	spin_unlock(&hw_transfer_lock);
}

static int ipi_comm_ctrl_handler(unsigned int id, void *prdata,
		void *data, unsigned int len)
{
	/* arrives as IPI_IN_SENSOR_CTRL on a direct pin, IPI_SENSOR on the
	 * mpool transport -- this firmware only has the latter */
	WARN_ON(id != IPI_IN_SENSOR_CTRL && id != IPI_SENSOR);
	ipi_comm_complete(data, len);
	return 0;
}

static int ipi_comm_notify_handler(unsigned int id, void *prdata,
		void *data, unsigned int len)
{
	WARN_ON(id != IPI_IN_SENSOR_NOTIFY);
	if (controller.notify_callback)
		controller.notify_callback(id, data, len);
	return 0;
}

/* Accepted and dropped; see IPI_IN_ELLIPTIC_ULTRA_0 above. */
static int ipi_comm_discard_handler(unsigned int id, void *prdata,
		void *data, unsigned int len)
{
	return 0;
}

/* legacy mpool dispatch uses the 3-argument handler shape */
static void ipi_comm_mpool_handler(int id, void *data, unsigned int len)
{
	const u8 *b = data;

	/*
	 * The firmware reports every FIFO write with SCP_SENSOR_HUB_DATA
	 * action 6 at ~100 Hz.  We read the samples from the DRAM FIFO
	 * ourselves, so drop those here instead of running the full control
	 * path (which is expensive and, with the mbox ack timeouts, enough to
	 * stall the system).
	 */
	if (b && len >= 2 && b[1] == 6)
		return;

	const uint8_t *p = data;

	/* Make the SCP->AP direction observable: this is the only place a
	 * nanoHub or sensor_comm reply can show up. */
	if (p && len >= 4)
		pr_debug("mpool rx: id %d len %u bytes %02x %02x %02x %02x d=%02x%02x%02x%02x %02x%02x%02x%02x\n",
			id, len, p[0], p[1], p[2], p[3],
			len > 4 ? p[4] : 0, len > 5 ? p[5] : 0,
			len > 6 ? p[6] : 0, len > 7 ? p[7] : 0,
			len > 8 ? p[8] : 0, len > 9 ? p[9] : 0,
			len > 10 ? p[10] : 0, len > 11 ? p[11] : 0);
	else
		pr_info("mpool rx: id %d len %u (no data)\n", id, len);

	ipi_comm_ctrl_handler(IPI_SENSOR, NULL, data, len);
}

int get_ctrl_id(void)
{
	return IPI_OUT_SENSOR_CTRL;
}

int get_notify_id(void)
{
	return IPI_OUT_SENSOR_CTRL; /* no pin for 31; share the ctrl channel */
}

void ipi_comm_notify_handler_register(
		void (*f)(int id, void *data, unsigned int len))
{
	controller.notify_callback = f;
}

void ipi_comm_notify_handler_unregister(void)
{
	controller.notify_callback = NULL;
}

/*
 * The vendor SCP driver restarts the core on its own schedule (WDT, AED, awake
 * timeout) and announces every transition here.  Both matter:
 *
 *   STOP  - the DRAM window is unsafe to touch until the core is back
 *   READY - the firmware lost its FIFO registration, so SET_CONFIG has to be
 *           sent again or fifo->wp stays frozen for the rest of the boot
 *
 * Without this the bring-up budget is spent on the first SCP instance and every
 * later restart leaves the sensor silently dead.
 */
static int chsensor_scp_notify(struct notifier_block *nb,
			       unsigned long event, void *ptr)
{
	if (event != SCP_EVENT_READY && event != SCP_EVENT_STOP)
		return NOTIFY_DONE;

	pr_info("chsensor: SCP %s, re-arming bring-up\n",
		event == SCP_EVENT_READY ? "ready" : "stopped");

	chsensor_reset_state();
	if (!READ_ONCE(chsensor_zero_ipi))
		schedule_delayed_work(&chsensor_enable_dw, msecs_to_jiffies(2000));

	return NOTIFY_DONE;
}

static struct notifier_block chsensor_scp_nb = {
	.notifier_call = chsensor_scp_notify,
};

int ipi_comm_init(void)
{
	init_completion(&hw_transfer.done);
	INIT_WORK(&controller.work, ipi_work);
	INIT_LIST_HEAD(&controller.head);
	spin_lock_init(&controller.lock);
	controller.workqueue = alloc_workqueue("ipi_comm",
		WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (controller.workqueue == NULL) {
		pr_err("create workqueue fail\n");
		return -ENOMEM;
	}
	return 0;
}

/*
 * Registering the receive callbacks requires the SCP platform driver to have
 * probed already: mtk_ipi_register() fails with IPI_DEV_ILLEGAL until the
 * mailbox/IPI device is initialised. This driver is built in and its initcall
 * can run first, so registration is deferred to the SCP READY notification and
 * retried until it succeeds. It is idempotent and tolerant of a concurrent
 * attempt (IPI_DUPLEX means someone already registered the same pin).
 */
int ipi_comm_register_handlers(void)
{
	int ret;

	if (READ_ONCE(handlers_ready))
		return 0;

	ret = mtk_ipi_register(&scp_ipidev, IPI_IN_SENSOR_CTRL,
		ipi_comm_ctrl_handler, NULL, ctrl_payload);
	if (ret < 0 && ret != IPI_DUPLEX) {
		pr_warn("register ipi %u failed %d\n",
			IPI_IN_SENSOR_CTRL, ret);
		return ret;
	}

	ret = mtk_ipi_register(&scp_ipidev, IPI_IN_SENSOR_NOTIFY,
		ipi_comm_notify_handler, NULL, notify_payload);
	if (ret < 0 && ret != IPI_DUPLEX) {
		pr_warn("register ipi %u failed %d\n",
			IPI_IN_SENSOR_NOTIFY, ret);
		return ret;
	}

	/*
	 * The mpool half of the same channel: the firmware frames sensor
	 * traffic as [u32 id][u32 len] inside SCP_MPOOL and dispatches it by
	 * that id, so the reply to our command comes back as IPI_SENSOR.
	 */
	ret = scp_ipi_registration(IPI_CHRE, ipi_comm_mpool_handler, "chre");
	if (ret != SCP_IPI_DONE)
		pr_warn("register mpool chre handler failed %d\n", ret);

	ret = scp_ipi_registration(IPI_SENSOR, ipi_comm_mpool_handler, "sensor");
	if (ret != SCP_IPI_DONE)
		pr_warn("register mpool sensor handler failed %d\n", ret);

	WRITE_ONCE(handlers_ready, true);

	/* Best effort: keeps a stray elliptic message from tripping BUG_ON. */
	ret = mtk_ipi_register(&scp_ipidev, IPI_IN_ELLIPTIC_ULTRA_0,
		ipi_comm_discard_handler, NULL, elliptic_payload);
	if (ret < 0 && ret != IPI_DUPLEX)
		pr_warn("register ipi %u failed %d (non-fatal)\n",
			IPI_IN_ELLIPTIC_ULTRA_0, ret);

	scp_A_register_notify(&chsensor_scp_nb);

	pr_info("sensor IPI handlers registered (ctrl %u, notify %u)\n",
		IPI_IN_SENSOR_CTRL, IPI_IN_SENSOR_NOTIFY);
	return 0;
}

bool ipi_comm_handlers_ready(void)
{
	return READ_ONCE(handlers_ready);
}

void ipi_comm_exit(void)
{
	WRITE_ONCE(handlers_ready, false);
	scp_A_unregister_notify(&chsensor_scp_nb);
	mtk_ipi_unregister(&scp_ipidev, IPI_IN_SENSOR_CTRL);
	mtk_ipi_unregister(&scp_ipidev, IPI_IN_SENSOR_NOTIFY);
	flush_workqueue(controller.workqueue);
	destroy_workqueue(controller.workqueue);
}
