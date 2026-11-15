// SPDX-License-Identifier: GPL-2.0
/*
 * The MT6895 tree provides the AFE <-> SCP 3-way semaphore notifier in its
 * audio driver.  The chopin audio stack (mt6893-mt6359) does not have it,
 * so provide the registration hooks as no-ops; the SCP boots fine without
 * the semaphore handshake.
 */
#include <linux/notifier.h>
#include <linux/errno.h>

int register_3way_semaphore_notifier(struct notifier_block *nb)
{
	return 0;
}

int unregister_3way_semaphore_notifier(struct notifier_block *nb)
{
	return 0;
}

int notify_3way_semaphore_control(unsigned long module, void *v)
{
	return 0;
}

int register_afe_allocate_mem_notifier(struct notifier_block *nb)
{
	return 0;
}

int unregister_afe_allocate_mem_notifier(struct notifier_block *nb)
{
	return 0;
}

int notify_allocate_mem(unsigned long module, void *v)
{
	return 0;
}
