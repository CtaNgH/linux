// SPDX-License-Identifier: GPL-2.0-only
/*
 * Stubs for the vendor's debug and production-test interfaces.
 *
 * nt36xxx_ext_proc.c and nt36xxx_mp_ctrlram.c provide /proc entries and the
 * factory test commands.  Neither is needed to use the touchscreen, and both
 * are written against the v4.14 proc and file_operations APIs, so they are left
 * out of the port and these take their place.  nt36xxx.c still calls them.
 *
 * tp_enable_doubleclick() lives in the vendor's own touch stack; the panel's
 * double-tap wake belongs to the display side and is not part of this port.
 */
#include <linux/types.h>
#include <linux/spi/spi.h>

int32_t nvt_extra_proc_init(void) { return 0; }
void nvt_extra_proc_deinit(void) { }

int32_t nvt_mp_proc_init(void) { return 0; }
void nvt_mp_proc_deinit(void) { }

int32_t nvt_test_data_proc_init(struct spi_device *client) { return 0; }
void nvt_test_data_proc_deinit(void) { }

void tp_enable_doubleclick(bool state) { }
