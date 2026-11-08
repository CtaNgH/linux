/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MTK_DISP_TRACE_H
#define MTK_DISP_TRACE_H

#include <linux/kernel.h>

#if IS_ENABLED(CONFIG_DEBUG_FS)
void mtk_disp_trace(const char *fmt, ...) __printf(1, 2);
void mtk_disp_trace_init(void);
#else
static inline void mtk_disp_trace(const char *fmt, ...) { }
static inline void mtk_disp_trace_init(void) { }
#endif

#endif /* MTK_DISP_TRACE_H */
