/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (c) 2026 The Linux Foundation.
 *
 * MediaTek MT6893 (Dimensity 1100/1200) infracfg reset controller IDs.
 *
 * The register offsets and bit positions below are taken from the vendor
 * kernel shipped by Xiaomi for the Redmi Note 10 Pro (chopin, MT6891Z):
 * drivers/scsi/ufs/mediatek/mt6893/ufs-mtk-platform.h documents
 *
 *	UFSHCI  SW_RST set 0x130 bit 15, clear 0x134 bit 15
 *	UNIPRO  SW_RST set 0x140 bit  7, clear 0x144 bit  7
 *	UFSPHY  SW_RST set 0x140 bit  9, clear 0x144 bit  9
 *	UFSCPT  SW_RST set 0x150 bit 21, clear 0x154 bit 21
 *
 * which are the standard MediaTek INFRA_RST1/2/3 set/clear register pairs
 * (INFRA_RST0_SET_OFFSET = 0x120, +0x10 per bank).
 */

#ifndef _DT_BINDINGS_RESET_CONTROLLER_MT6893
#define _DT_BINDINGS_RESET_CONTROLLER_MT6893

/* INFRACFG_AO resets */
#define MT6893_INFRA_RST1_UFSHCI_SW_RST		0
#define MT6893_INFRA_RST2_UNIPRO_SW_RST		1
#define MT6893_INFRA_RST2_UFSPHY_SW_RST		2
#define MT6893_INFRA_RST3_UFSCPT_SW_RST		3

#define MT6893_INFRA_RST_NR			4

#endif /* _DT_BINDINGS_RESET_CONTROLLER_MT6893 */
