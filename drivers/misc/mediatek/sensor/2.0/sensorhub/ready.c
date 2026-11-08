// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 *
 * SCP sensor-hub readiness handshake.
 *
 * Two independent conditions must hold before sensors may be used:
 *   1. the SCP platform reports SCP_EVENT_READY, and
 *   2. the sensor-hub firmware answers our READY notification.
 *
 * On SCP_EVENT_READY we send a ready notification (bypassing the scp_status
 * gate, since that is what establishes it) and arm a rescue timer. Downstream
 * resets the whole SCP if the sensor hub never answers. That is an aggressive
 * recovery for a partially ported stack, so it is available but disabled by
 * default; see the rescue_reset module parameter.
 */

#define pr_fmt(fmt) "sensor_ready " fmt

#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/module.h>

#include "scp.h"
#include "hf_sensor_type.h"
#include "ipi_comm.h"
#include "sensor_comm.h"
#include "ready.h"
#include "share_memory.h"

extern int sensorhub_prime(void);

static bool rescue_reset;
module_param(rescue_reset, bool, 0644);
MODULE_PARM_DESC(rescue_reset,
	"Reset SCP if the sensor hub does not answer READY within 5s (default 0)");

static DEFINE_SPINLOCK(sensor_ready_lock);
static bool scp_platform_ready;
static bool scp_sensor_ready;
static bool sensor_ready;
static BLOCKING_NOTIFIER_HEAD(sensor_ready_notifier_head);
static struct workqueue_struct *sensor_ready_workqueue;
static struct work_struct sensor_ready_work;
static struct delayed_work sensor_rescure_work;

void sensor_ready_notifier_chain_register(struct notifier_block *nb)
{
	unsigned long flags = 0;
	bool status = false;

	blocking_notifier_chain_register(&sensor_ready_notifier_head, nb);
	/*
	 * NOTE: must copy sensor_ready to status to avoid sensor_ready
	 * modified during notifier calling, keep notify the same status.
	 */
	spin_lock_irqsave(&sensor_ready_lock, flags);
	status = sensor_ready;
	spin_unlock_irqrestore(&sensor_ready_lock, flags);
	if (status)
		nb->notifier_call(nb, status, NULL);
}

void sensor_ready_notifier_chain_unregister(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&sensor_ready_notifier_head, nb);
}

static void scp_sensor_ready_notify_handler(struct sensor_comm_notify *n,
		void *private_data)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&sensor_ready_lock, flags);
	scp_sensor_ready = true;
	if (scp_platform_ready && scp_sensor_ready) {
		sensor_ready = true;
		cancel_delayed_work(&sensor_rescure_work);
		queue_work(sensor_ready_workqueue, &sensor_ready_work);
	}
	spin_unlock_irqrestore(&sensor_ready_lock, flags);
	pr_info("sensor hub reported ready\n");
}

static int scp_platform_ready_notifier_call(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	unsigned long flags = 0;
	int ret = 0;
	struct sensor_comm_notify notify;

	if (event == SCP_EVENT_STOP) {
		cancel_delayed_work(&sensor_rescure_work);
		spin_lock_irqsave(&sensor_ready_lock, flags);
		scp_platform_ready = false;
		scp_sensor_ready = false;
		sensor_ready = false;
		queue_work(sensor_ready_workqueue, &sensor_ready_work);
		spin_unlock_irqrestore(&sensor_ready_lock, flags);
	} else if (event == SCP_EVENT_READY) {
		/*
		 * The SCP platform driver has now probed and initialised its IPI
		 * device, so this is the earliest safe point to register our
		 * receive callbacks. It must happen before the READY
		 * notification below, otherwise the firmware's answer would
		 * arrive with no handler and the ready handshake would be lost.
		 * This callback runs from a workqueue and may sleep.
		 */
		ret = ipi_comm_register_handlers();
		if (ret < 0) {
			pr_err("sensor IPI registration failed %d, giving up\n",
				ret);
			return NOTIFY_DONE;
		}

		/*
		 * The firmware needs to know the physical addresses of the
		 * shared sensor rings before it starts the sensor hub, and it
		 * only answers the ready notification after that.  Sending the
		 * notification first left it silent for five seconds and the
		 * rescue path reported "sensor hub gave no ready ack" forever.
		 * share_mem_config() is idempotent and needs the handlers that
		 * were just registered, so it belongs here.
		 */
		sensor_comm_set_preready(true);
		ret = sensorhub_prime();
		/*
		 * Keep the window open: this firmware never sends the sensor
		 * ready notification that would set scp_status, yet it does act
		 * on what it is given (its own log shows the accelerometer
		 * enabled at 512Hz and the FIFO armed).  Leaving the gate shut
		 * dropped every sensor command, including the periodic one, so
		 * the read path could only ever report -EAGAIN.
		 */
		if (ret < 0)
			pr_err("shared memory config before ready failed %d\n",
			       ret);

		notify.sequence = 0;
		notify.sensor_type = SENSOR_TYPE_INVALID;
		notify.command = SENS_COMM_NOTIFY_READY_CMD;
		notify.length = 0;
		ret = sensor_comm_notify_bypass(&notify);
		if (ret < 0)
			pr_err("notify ready to scp fail %d\n", ret);
		/*
		 * Only arm the watchdog rescue once the handshake was actually
		 * sent; otherwise a failure here would reset the SCP in a loop.
		 */
		if (ret >= 0)
			queue_delayed_work(sensor_ready_workqueue,
				&sensor_rescure_work, msecs_to_jiffies(5000));
		spin_lock_irqsave(&sensor_ready_lock, flags);
		scp_platform_ready = true;
		if (scp_platform_ready && scp_sensor_ready) {
			sensor_ready = true;
			queue_work(sensor_ready_workqueue,
				&sensor_ready_work);
		}
		spin_unlock_irqrestore(&sensor_ready_lock, flags);
	}

	return NOTIFY_DONE;
}

static struct notifier_block scp_platform_ready_notifier = {
	.notifier_call = scp_platform_ready_notifier_call,
};

static void sensor_ready_work_fn(struct work_struct *work)
{
	unsigned long flags = 0;
	bool status = false;

	/*
	 * NOTE: must copy sensor_ready to status to avoid sensor_ready
	 * modified during notifier calling, keep notify the same status.
	 */
	spin_lock_irqsave(&sensor_ready_lock, flags);
	status = sensor_ready;
	spin_unlock_irqrestore(&sensor_ready_lock, flags);
	pr_info("sensor ready state changed to %d\n", status);
	blocking_notifier_call_chain(&sensor_ready_notifier_head,
		status, NULL);
}

static void sensor_rescure_work_fn(struct work_struct *work)
{
	unsigned long flags = 0;
	bool need_reset = false;

	spin_lock_irqsave(&sensor_ready_lock, flags);
	if (scp_platform_ready && !scp_sensor_ready)
		need_reset = true;
	spin_unlock_irqrestore(&sensor_ready_lock, flags);

	if (!need_reset)
		return;

	if (!READ_ONCE(rescue_reset)) {
		pr_err("sensor hub gave no ready ack; rescue reset disabled\n");
		return;
	}

	pr_alert("rescue sensor by scp reset due to no ready ack\n");
	scp_wdt_reset(0);
}

int host_ready_init(void)
{
	INIT_WORK(&sensor_ready_work, sensor_ready_work_fn);
	INIT_DELAYED_WORK(&sensor_rescure_work, sensor_rescure_work_fn);
	/*
	 * Since 6.9 an ordered workqueue must say whether it is per-CPU or
	 * unbound; omitting both triggers
	 * "workqueue: sensor_ready is using neither WQ_PERCPU or WQ_UNBOUND"
	 * and taints the kernel.  The ready handshake is a one-shot job, so
	 * unbound is the right answer (and matches sensorhub's own wq).
	 */
	sensor_ready_workqueue = alloc_workqueue("sensor_ready",
		WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!sensor_ready_workqueue) {
		pr_err("alloc workqueue fail\n");
		return -ENOMEM;
	}
	/*
	 * NOTE: sensor comm notify must before scp register notify
	 * to avoid lost sensor ready notify.
	 */
	sensor_comm_notify_handler_register(SENS_COMM_NOTIFY_READY_CMD,
		scp_sensor_ready_notify_handler, NULL);
	scp_A_register_notify(&scp_platform_ready_notifier);
	return 0;
}

void host_ready_exit(void)
{
	scp_A_unregister_notify(&scp_platform_ready_notifier);
	sensor_comm_notify_handler_unregister(SENS_COMM_NOTIFY_READY_CMD);
	flush_workqueue(sensor_ready_workqueue);
	destroy_workqueue(sensor_ready_workqueue);
}
