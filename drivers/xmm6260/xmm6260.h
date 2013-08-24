/*
 * arch/arm/mach-tegra/baseband-xmm-power.h
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _XMM6260_H
#define _XMM6260_H

#include <linux/pm.h>
#include <linux/suspend.h>

#define XMM_MAIN_VID	0x1519
#define XMM_MAIN_PID	0x0020
#define XMM_BOOT_VID	0x058b
#define XMM_BOOT_PID	0x0041
#define XMM_CRASH_VID	0x058b
#define XMM_CRASH_PID	0x0015

#define BB_XMM6260	0x6260

struct baseband_power_platform_data {
	unsigned int baseband_type;
	struct platform_device* (*hsic_register)(void);
	void (*hsic_unregister)(struct platform_device *);
	struct platform_device* (*utmip_register)(void);
	void (*utmip_unregister)(struct platform_device *);
	union {
		struct {
			int mdm_reset;
			int mdm_on;
			int ap2mdm_ack;
			int mdm2ap_ack;
			int ap2mdm_ack2;
			int mdm2ap_ack2;
			struct platform_device *device;
		} generic;
		struct {
			int bb_rst;
			int bb_on;
			int bb_vbat;
			int bb_rst_ind;
			int bb_vbus;
			int bb_sw_sel;
			int bb_sim_cd;
			int bb_sar_det;
			int ipc_bb_wake;
			int ipc_ap_wake;
			int ipc_hsic_active;
			int ipc_hsic_sus_req;
			int ipc_bb_force_crash;
			struct platform_device *hsic_device;
		} xmm;
	} modem;
};

/* gpio */
int __init xmm6260_gpio_init(void);
void __exit xmm6260_gpio_exit(void);

/* pwr */
void xmm6260_enable_hsic_power(int enable);
void xmm6260_ril_set_L0(void);
//void xmm6260_host_set_L0(void);
//void xmm6260_host_set_L2(void);
void xmm6260_suspend_prepare(void);
void xmm6260_suspend_cleanup(void);
int xmm6260_modem_crash_dump(int enable);
int __init xmm6260_pwr_init(void);
void __exit xmm6260_pwr_exit(void);

/* ril */
int __init xmm6260_ril_init(void);
void __exit xmm6260_ril_exit(void);

/* ril_prox */
int xmm_prox_init(struct device *rdev, struct workqueue_struct *wq);
int xmm_prox_exit(struct device *rdev);
int xmm_ril_resume(struct platform_device *pdev);

/* ril_sim */
void xmm6260_sync_state(bool pwr);
int xmm_sim_init(struct device *rdev, struct workqueue_struct *wq);
int xmm_sim_exit(struct device *rdev);

/* ril_crash */
int xmm_crash_init(struct device *rdev, struct workqueue_struct *wq);
int xmm_crash_exit(struct device *rdev);
void ril_crash_set(void);
void ril_crash_clear(void);

/* ril_acm */
int __init xmm6260_acm_init(void);
void __exit xmm6260_acm_exit(void);

/* ril_net */
int __init xmm6260_net_init(void);
void __exit xmm6260_net_exit(void);

#endif  /* _XMM6260_H */
