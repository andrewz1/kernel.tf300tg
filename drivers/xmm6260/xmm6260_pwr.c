#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/regulator/consumer.h>

#include <asm/io.h>
#include <pm-irq.h>
#include <gpio-names.h>
#include <board-cardhu.h>

#include "xmm6260_pwr.h"

static struct usb_device_id xmm_main_id[] = {
	{ USB_DEVICE(XMM_MAIN_VID, XMM_MAIN_PID), .driver_info = 0 },
	{}
};

static struct usb_device_id xmm_boot_id[] = {
	{ USB_DEVICE(XMM_BOOT_VID, XMM_BOOT_PID), .driver_info = 0 },
	{}
};

static struct usb_device_id xmm_crash_id[] = {
	{ USB_DEVICE(XMM_CRASH_VID, XMM_CRASH_PID), .driver_info = 0 },
	{}
};

static enum {
	XMM_MODE_NODEV	= -1,
	XMM_MODE_MODEM	= 0,
	XMM_MODE_BOOT	= 1,
	XMM_MODE_CRASH	= 2
} xmm_dev_mode = XMM_MODE_NODEV;

static enum {
//	IPC_AP_WAKE_BOOT	= -2,
	IPC_AP_WAKE_UNINIT	= -1,
	IPC_AP_WAKE_IRQ_READY	= 0,
	IPC_AP_WAKE_INIT1	= 1,
	IPC_AP_WAKE_INIT2	= 2,
	IPC_AP_WAKE_READY	= 3,
	IPC_AP_WAKE_SUSPEND	= 4
} ipc_ap_wake_state = IPC_AP_WAKE_UNINIT; /* current irq state */

static DEFINE_SPINLOCK(p_lock); /* state lock */

static struct regulator *reg_cardhu_hsic; /* modem power regulator - LDO6 */

/* host wakeup */
static bool hostwake; /* host wakeup flag */
static DECLARE_WAIT_QUEUE_HEAD(bb_wait); /* host wakeup wait */

static struct baseband_power_platform_data *pdata;
static struct platform_device *hsic_device;
//static struct platform_device *utmip_device;
static struct usb_device *usbdev; /* xmm usb device */
static struct device *pwrdev; /* for direct power mgmt */
static struct workqueue_struct *state_wq; /* state change wq */
static struct work_struct INIT1_work; /* INIT1 to INIT2 state worker */
static struct work_struct INIT2_work; /* INIT2 to READY state worker */
static struct work_struct L2_resume_modem; /* modem initiated L2->L0 wakeup worker */

static int m_off; /* modem was manually off - don't try to switch it on back, 0 - off, 1 - on, 2 - transit state */
static int nml_reset_count;
static bool xmm_dwd_reset; /* modem firmware update initiated */
static bool system_suspending; /* -> L3 in progress */
static bool xmm_modem_on; /* modem is on - ready flag - for ril sim card detect */
//static DEFINE_SPINLOCK(s_lock); /* lock for system_suspending and resume_from_l3 */

/* shared data */
bool resume_from_l3 = false; /* usb hub need this */
EXPORT_SYMBOL(resume_from_l3);

//#define __xmm_lock_state() do { spin_lock_irqsave(&p_lock, flags); _xd("+++ LOCK +++\n"); } while (0)
//#define __xmm_unlock_state() do { spin_unlock_irqrestore(&p_lock, flags); _xd("+++ UNLOCK +++\n"); } while (0)
#define __set_wake_state(ws) do { \
	unsigned long flags; \
	spin_lock_irqsave(&p_lock, flags); \
	ipc_ap_wake_state = (ws); \
	spin_unlock_irqrestore(&p_lock, flags); \
} while (0)

static __always_inline bool xmm_pwr_hsic_register(void)
{
	if (!pdata)
		return false;
	if (hsic_device || !pdata->hsic_register)
		return false;
	hsic_device = pdata->hsic_register();
	if (!hsic_device)
		return false;
	return true;
}

static __always_inline bool xmm_pwr_hsic_unregister(void)
{
	if (!pdata)
		return false;
	if (!hsic_device || !pdata->hsic_unregister)
		return false;
	pdata->hsic_unregister(hsic_device);
	hsic_device = NULL;
	return true;
}

int xmm6260_modem_crash_dump(int enable)
{
	_xd("+++:%d\n", enable);
	if (!pdata)
		return -EINVAL;
	if (!pdata->hsic_register || !pdata->hsic_unregister)
		return -EINVAL;
	if (!pdata->utmip_register || !pdata->utmip_unregister)
		return -EINVAL;
	switch (enable) {
	case 0:
		xmm_pwr_hsic_unregister();
		break;
	case 1:
//		utmip_device = pdata->utmip_register();
		hsic_device = pdata->utmip_register();
		break;
	default:
		return -EINVAL;
		break;
	}
	_xd("---\n");
	return 0;
}

/* host initiated L0 wakeup routine */
static void xmm_host_set_L0(void)
{
//	unsigned long flags;

	_xd("+++\n");
	if (!gpio_get_value(XMM_GPIO_IPC_AP_WAKE)) {
		_xd("modem wakeup - noop\n");
		return;
	}
	hostwake = true; /* host wakeup flag */
	gpio_set_value(XMM_GPIO_IPC_BB_WAKE, 1); /* set modem wakeup pin */
	switch (wait_event_timeout(bb_wait, !hostwake, HZ)) { /* wait for a second */
	case 0:
		_xd("host wakeup failed - timeout\n");
		return;
		break;
	case -ERESTARTSYS:
		_xd("host wakeup failed\n");
		return;
		break;
	default:
		_xd("host wakeup done\n");
		break;
	}
//	_xd("%s -> L0\n", ps_name[baseband_xmm_powerstate]);
//	__xmm_lock_state();
//	baseband_xmm_powerstate = BBXMM_PS_L0;
//	__xmm_unlock_state();
}

static __always_inline void xmm_enable_hsic(int enable)
{
	_xd("+++:%d\n", enable);
	if (system_suspending) {
		if (enable) {
			_xd("wakeup - wakeup modem\n");
			msleep(10);
//			__set_wake_state(IPC_AP_WAKE_READY);
			xmm_host_set_L0();
		} else
			_xd("suspend - noop\n");
		return;
	}
	gpio_set_value(XMM_GPIO_IPC_HSIC_ACTIVE, !!enable);
	return;
}

/* called from uhsic_phy_power_on/off */
void xmm6260_enable_hsic_power(int enable)
{
	_xd("+++:%d\n", enable);
	if (xmm_dwd_reset) {
		_xi("firmware update - keep power, change IPC_HSIC_ACTIVE\n");
		xmm_enable_hsic(enable);
		return;
	}
	if (!reg_cardhu_hsic) {
		_xe("!reg_cardhu_hsic\n");
		return;
	}
	switch (enable) {
	case 0:
		xmm_enable_hsic(enable);
		msleep(1);
		regulator_disable(reg_cardhu_hsic);
		return;
	default:
		regulator_enable(reg_cardhu_hsic);
		msleep(1);
		xmm_enable_hsic(enable);
		return;
	}
	return;
}

static void xmm_pwr_modem_on(void)
{
	_xd("+++\n");
	__set_wake_state(IPC_AP_WAKE_IRQ_READY);
	enable_irq(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
//	enable_irq_wake(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	gpio_set_value(XMM_GPIO_BB_ON, 0);
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_RST, 0);
	msleep(1);
	gpio_set_value(BB_GPIO_VBAT_ON, 1);
	msleep(100);
	gpio_set_value(XMM_GPIO_BB_RST, 1);
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_ON, 1);
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_ON, 0);
	return;
}

static void xmm_pwr_modem_off(void)
{
	_xd("+++\n");
	__set_wake_state(IPC_AP_WAKE_UNINIT);
	if (!xmm_pwr_hsic_unregister())
		_xe("!xmm_pwr_hsic_unregister()\n");
	gpio_set_value(XMM_GPIO_BB_ON, 0);
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_RST, 0);
	msleep(1);
	gpio_set_value(BB_GPIO_VBAT_ON, 0);
	msleep(1);
//	disable_irq_wake(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	disable_irq(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	return;
}

/* store modem on/off */
static ssize_t store_xmm_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int pwr;

	_xd("+++:%c\n", buf[0]);
	if (sscanf(buf, "%d", &pwr) != 1)
		return -EINVAL;
	switch (pwr) {
	case 0:
		if (!xmm_modem_on)
			return -EINVAL;
		break;
	case 1:
		if (xmm_modem_on)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	switch (pwr) {
	case 0: /* turn modem off */
		m_off = 1;
		xmm_modem_on = false; /* modem is off */
		xmm6260_sync_state(xmm_modem_on);
		xmm_pwr_modem_off();
		break;
	default: /* turn modem on */
		if (xmm_dwd_reset) { /* restart after firmware update */
			_xd("restart after FW update\n");
			xmm_dwd_reset = false;
			if (!xmm_pwr_hsic_unregister()) /* reregister controller */
				_xe("!xmm_pwr_hsic_unregister()\n");
			gpio_set_value(XMM_GPIO_BB_ON, 0);
			msleep(1);
			gpio_set_value(XMM_GPIO_BB_RST, 0);
			msleep(1);
			gpio_set_value(BB_GPIO_VBAT_ON, 0);
			msleep(1);
		}
		m_off = 2; /* transit state */
		xmm_pwr_modem_on();
		break;
	}
	return count;
}

static ssize_t show_xmm_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	_xd("+++\n");
	return sprintf(buf, "manual_off:%d, power_state:%d\n", m_off, xmm_modem_on);
}

static ssize_t store_dwd_reset_modem(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int dwd;

	_xd("+++:%c\n", buf[0]);
	if (sscanf(buf, "%d", &dwd) != 1)
		return -EINVAL;
	switch (dwd) {
	case 1:
		if (!xmm_modem_on) /* modem must be on */
			return -EINVAL;
		break;
	default:
		return -EINVAL;
		break;
	}
	__set_wake_state(IPC_AP_WAKE_UNINIT);
	m_off = 1; /* we don't want any interrupts - manual off */
	xmm_modem_on = false;
	xmm6260_sync_state(xmm_modem_on);
	xmm_dwd_reset = true; /* workaround for disable real power off */
	if (!xmm_pwr_hsic_unregister())
		_xe("!xmm_pwr_hsic_unregister()\n");
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_ON, 0);
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_RST, 0);
	msleep(1);
//	disable_irq_wake(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	disable_irq(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	if (!xmm_pwr_hsic_register())
		_xe("!xmm_pwr_hsic_register()\n");
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_RST, 1);
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_ON, 1);
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_ON, 0);
	return count;
}

static ssize_t show_dwd_reset_modem(struct device *dev, struct device_attribute *attr, char *buf)
{
	_xd("+++\n");
	return sprintf(buf, "fw_update_mode:%d\n", xmm_dwd_reset);
}

static ssize_t store_nml_reset_modem(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int nml;

	_xd("+++:%c\n", buf[0]);
	if (sscanf(buf, "%d", &nml) != 1)
		return -EINVAL;
	switch (nml) {
	case 1:
		break;
	default:
		return -EINVAL;
		break;
	}
	if (m_off) {
		_xi("modem is switched off - noop\n");
		return count;
	}
	nml_reset_count++;
	/* power off */
	m_off = 1;
	xmm_modem_on = false;
	xmm6260_sync_state(xmm_modem_on);
	xmm_pwr_modem_off();
	msleep(100);
	/* power on */
	m_off = 2;
	xmm_pwr_modem_on();
	return count;
}

static ssize_t show_nml_reset_modem(struct device *dev, struct device_attribute *attr, char *buf)
{
	_xd("+++\n");
	return sprintf(buf, "nml_reset_count:%d\n", nml_reset_count);
}

static ssize_t store_force_crash_modem(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int cr;

	_xd("+++:%c\n", buf[0]);
	if (sscanf(buf, "%d", &cr) != 1)
		return -EINVAL;
	switch (cr) {
	case 1:
		break;
	default:
		return -EINVAL;
		break;
	}
// WiP
	return -EINVAL;

//	gpio_set_value(XMM_GPIO_IPC_BB_FORCE_CRASH, cr);

//	xmm6260_modem_crash_dump(0);
//	msleep(300);
//	ril_crash_set();
//	xmm6260_modem_crash_dump(1);

	__set_wake_state(IPC_AP_WAKE_UNINIT);

	m_off = 1; /* we don't want any interrupts - manual off */
	xmm_modem_on = false;
	xmm6260_sync_state(xmm_modem_on);
	xmm_dwd_reset = true; /* workaround for disable real power off */
//	xmm_pwr_modem_off();
//	disable_irq_wake(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));


//	__xmm_lock_state();
//	ipc_ap_wake_state = IPC_AP_WAKE_UNINIT;
//	baseband_xmm_powerstate = BBXMM_PS_UNINIT;
//	__xmm_unlock_state();
	if (!xmm_pwr_hsic_unregister())
		_xe("!xmm_pwr_hsic_unregister()\n");
	gpio_set_value(XMM_GPIO_BB_ON, 0);
	gpio_set_value(XMM_GPIO_BB_RST, 0);
	msleep(200);








//	gpio_set_value(BB_GPIO_VBAT_ON, 1);
	msleep(1);
	hsic_device = pdata->utmip_register();
	if (!hsic_device)
		_xe("!utmip_register()\n");
//	if (!xmm_pwr_hsic_register())
//		_xe("!xmm_pwr_hsic_register()\n");

	msleep(1);

	ril_crash_set();



	msleep(1);
	gpio_set_value(XMM_GPIO_BB_RST, 1);
//	msleep(1);
//	ril_crash_set();
	msleep(1);

//	if (!xmm_pwr_hsic_register())
//		_xe("!xmm_pwr_hsic_register()\n");



//	msleep(1);
	gpio_set_value(XMM_GPIO_BB_ON, 1);
	msleep(1);
	gpio_set_value(XMM_GPIO_BB_ON, 0);
	_xd("ZZZ end of start\n");
	return count;

}

static struct device_attribute xmm_device_attr[] = {
	__ATTR(xmm_onoff,	S_IRUSR | S_IWUSR | S_IRGRP, show_xmm_onoff,		store_xmm_onoff),
	__ATTR(xmm_dwd_reset,	S_IRUSR | S_IWUSR | S_IRGRP, show_dwd_reset_modem,	store_dwd_reset_modem),
	__ATTR(xmm_nml_reset,	S_IRUSR | S_IWUSR | S_IRGRP, show_nml_reset_modem,	store_nml_reset_modem),
	__ATTR(xmm_force_crash,	S_IRUSR | S_IWUSR | S_IRGRP, NULL,			store_force_crash_modem),
	__ATTR_NULL,
};

/* wake modem after SIM changed */
void xmm6260_ril_set_L0(void)
{
	_xd("+++ SIM CARD RESUME +++\n");
//	queue_work(state_wq, &L2_resume_modem);
}

/* resume usb device on modem request work */
static void xmm_pwr_L2_resume_modem(struct work_struct *work)
{
	_xd("+++\n");
	usb_get_dev(usbdev); /* usb device was removed? if not - prevent */
	usb_lock_device(usbdev);
	pm_runtime_get_sync(pwrdev);
	pm_runtime_put_sync(pwrdev);
	pm_runtime_mark_last_busy(pwrdev);
	usb_unlock_device(usbdev);
	usb_put_dev(usbdev);
	_xd("---\n");
	return;
}
static DECLARE_WORK(L2_resume_modem, xmm_pwr_L2_resume_modem);

/* this work will only run in init state so don't need to check state change here */
static void xmm_pwr_INIT1_work(struct work_struct *work)
{
	_xd("+++\n");
	if (!xmm_pwr_hsic_register())
		_xe("!xmm_pwr_hsic_register()\n");
	return;
}
static DECLARE_WORK(INIT1_work, xmm_pwr_INIT1_work);

/* this work will only run in init state so don't need to check state change here */
static void xmm_pwr_INIT2_work(struct work_struct *work)
{
	_xd("+++\n");
	if (m_off == 2)
		m_off = 0;
	xmm_modem_on = true;
	xmm6260_sync_state(xmm_modem_on);
	return;
}
static DECLARE_WORK(INIT2_work, xmm_pwr_INIT2_work);

/* called from cardu_usb_hsic_port_power for cleanup after suspend */
void xmm6260_suspend_cleanup(void)
{
	if (!system_suspending)
		return;
	_xd("+++\n");
	if (m_off == 2)
		m_off = 0;
	xmm_modem_on = true;
	xmm6260_sync_state(xmm_modem_on);
	return;
}

/* called from cardu_usb_hsic_post_phy_off for set level to L3 */
void xmm6260_suspend_prepare(void)
{
	if (!system_suspending)
		return;
	_xd("+++\n");
	m_off = 2;
	xmm_modem_on = false;
	xmm6260_sync_state(xmm_modem_on);
	return;
}

irqreturn_t xmm_power_ipc_ap_wake_irq(int irq, void *dev_id)
{
	int ap, bb, hs;

	if (!pdata)
		return IRQ_HANDLED;
	ap = gpio_get_value(XMM_GPIO_IPC_AP_WAKE);
	bb = gpio_get_value(XMM_GPIO_IPC_BB_WAKE);
	hs = gpio_get_value(XMM_GPIO_IPC_HSIC_ACTIVE);
	_xd("+++:ap:%d:bb:%d:hs:%d\n", ap, bb, hs);
	switch (ipc_ap_wake_state) {
	/* modem initialization/bootup part */
//	case IPC_AP_WAKE_BOOT:
//		_xd("WAKE_BOOT: firmware update\n");
//		break;
	case IPC_AP_WAKE_SUSPEND:
		if (!ap)
			_xd("WAKE_SUSPEND: suspend to L3\n");
		else {
			_xd("WAKE_SUSPEND: resume from L3\n");
			ipc_ap_wake_state = IPC_AP_WAKE_READY;
		}
		break;
	case IPC_AP_WAKE_UNINIT:
		_xd("WAKE_UNINIT: spurious irq\n");
		break;
	case IPC_AP_WAKE_IRQ_READY:
		if (ap) {
			_xd("WAKE_IRQ_READY: rising edge, wait falling edge\n");
			ipc_ap_wake_state = IPC_AP_WAKE_INIT1;
		} else
			_xd("WAKE_IRQ_READY: falling edge, state error\n");
		break;
	case IPC_AP_WAKE_INIT1:
		if (!ap) {
			_xd("WAKE_INIT1: falling edge, register hsic device\n");
			ipc_ap_wake_state = IPC_AP_WAKE_INIT2;
			queue_work(state_wq, &INIT1_work);
		} else
			_xd("WAKE_INIT1: rising edge, state error\n");
		break;
	case IPC_AP_WAKE_INIT2:
		if (ap) {
			_xd("WAKE_INIT2: rising edge, init finished\n");
			ipc_ap_wake_state = IPC_AP_WAKE_READY;
			queue_work(state_wq, &INIT2_work);
		} else
			_xd("WAKE_INIT2: falling edge, state error\n");
		break;
	/* modem wakeup part */
	default: /* IPC_AP_WAKE_READY */
		if (!hs) {
			_xe("ipc_hsic_active = 0, ignoring request\n");
			break;
		}
		if (!ap) { /* start */
			if (bb) { /* host initiated wakeup */
				_xd("WAKE_READY: host initiated wakeup - start, ack\n");
				hostwake = false; /* ack host wakeup */
				smp_mb();
				if (waitqueue_active(&bb_wait))
					wake_up(&bb_wait);
			} else { /* modem initiated wakeup */
				_xd("WAKE_READY: modem initiated wakeup - start\n");
				if (xmm_modem_on)
					queue_work(state_wq, &L2_resume_modem);
			}
		} else { /* finish */
			if (bb) {
				_xd("WAKE_READY: host initiated wakeup - finish, clear\n");
				gpio_set_value(XMM_GPIO_IPC_BB_WAKE, 0);
			} else
				_xd("WAKE_READY: modem initiated wakeup - finish\n");
		}
		break;
	}
	return IRQ_HANDLED;
}

static void xmm_pwr_usb_add_event(struct usb_device *udev)
{
	struct usb_interface *intf = usb_ifnum_to_if(udev, 0);

	if (!intf || (xmm_dev_mode != XMM_MODE_NODEV))
		return;
//	_xd("+++\n");
	if (usb_match_id(intf, xmm_main_id)) {
		xmm_dev_mode = XMM_MODE_MODEM;
		usbdev = udev;
		pwrdev = &udev->dev;
		pm_runtime_allow(pwrdev);
//		pm_runtime_set_autosuspend_delay(pwrdev, 10000);
		_xi("Add device %d <XMM6260 MODEM>\n", udev->devnum);
		_xi("autosuspend_delay: %d seconds\n", pwrdev->power.autosuspend_delay/1000);
	} else if (usb_match_id(intf, xmm_boot_id)) {
		xmm_dev_mode = XMM_MODE_BOOT;
		usbdev = udev;
		pwrdev = &udev->dev;
		pm_runtime_forbid(pwrdev);
		_xi("Add device %d <XMM6260 BOOT>\n", udev->devnum);
	} else if (usb_match_id(intf, xmm_crash_id)) {
		xmm_dev_mode = XMM_MODE_CRASH;
		usbdev = udev;
		pwrdev = &udev->dev;
		pm_runtime_forbid(pwrdev);
		_xi("Add device %d <XMM6260 CRASH>\n", udev->devnum);
	}
}

static void xmm_pwr_usb_remove_event(struct usb_device *udev)
{
	if ((usbdev != udev) || (xmm_dev_mode == XMM_MODE_NODEV))
		return;
//	_xd("+++\n");
	switch (xmm_dev_mode) {
	case XMM_MODE_MODEM:
		pm_runtime_forbid(pwrdev);
		_xi("Remove device %d <XMM6260 MODEM>\n", udev->devnum);
		break;
	case XMM_MODE_BOOT:
		pm_runtime_forbid(pwrdev);
		_xi("Remove device %d <XMM6260 BOOT>\n", udev->devnum);
		break;
	case XMM_MODE_CRASH:
		pm_runtime_forbid(pwrdev);
		_xi("Remove device %d <XMM6260 CRASH>\n", udev->devnum);
		break;
	default:
//		_xe("Remove unknown device %d <%s %s> - BUG!!!\n", udev->devnum, udev->manufacturer, udev->product);
		break;
	}
	usbdev = NULL;
	pwrdev = NULL;
	xmm_dev_mode = XMM_MODE_NODEV;
}

static int xmm_pwr_usb_notifier_event(struct notifier_block *self, unsigned long action, void *blob)
{
	switch (action) {
	case USB_DEVICE_ADD:
		xmm_pwr_usb_add_event(blob);
		break;
	case USB_DEVICE_REMOVE:
		xmm_pwr_usb_remove_event(blob);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block xmm_pwr_usb_notifier = {
	.notifier_call = xmm_pwr_usb_notifier_event,
};

static int xmm_pwr_pm_notifier_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	_xd("+++\n");
	switch (event) {
	case PM_SUSPEND_PREPARE:
		_xd("PM_SUSPEND_PREPARE\n");
		__set_wake_state(IPC_AP_WAKE_UNINIT);
		system_suspending = true;
		resume_from_l3 = true;
		smp_mb();
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		_xd("PM_POST_SUSPEND\n");
		__set_wake_state(IPC_AP_WAKE_READY);
		system_suspending = false;
		resume_from_l3 = false;
		smp_mb();
		return NOTIFY_OK;
	default:
		_xd("EVENT: %lu\n", event);
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block xmm_pwr_pm_notifier = {
	.notifier_call = xmm_pwr_pm_notifier_event,
};

static int xmm_pwr_probe(struct platform_device *pdev)
{
	struct baseband_power_platform_data *p = pdev->dev.platform_data;
	int rv = 0, f, r;

	/* check platform data */
	if (!p || (p->baseband_type != BB_XMM6260))
		return -ENODEV;
	/* init platform data */
	pdata = p;
	/* init state machine */
	__set_wake_state(IPC_AP_WAKE_UNINIT);
	/* create files in /sys/devices/platform/baseband_xmm_power */
	for (f = 0; f < (ARRAY_SIZE(xmm_device_attr) - 1); f++) {
		rv = device_create_file(&pdev->dev, &xmm_device_attr[f]);
		if (rv < 0) {
			_xe("create file %d failed, err = %d\n", f, rv);
			goto error0;
		}
	}
	/* setup regulator LDO6 for hsic power */
	if (!reg_cardhu_hsic) {
		reg_cardhu_hsic = regulator_get(NULL, "vddio_hsic");
		if (IS_ERR(reg_cardhu_hsic)) {
			_xe("regulator_get() HSIC power on LDO6 failed\n");
			reg_cardhu_hsic = NULL;
			rv = PTR_ERR(reg_cardhu_hsic);
			goto error0;
		}
		regulator_disable(reg_cardhu_hsic);
		regulator_set_voltage(reg_cardhu_hsic, 1200000, 1200000); /* normal is 1.2V */
	}
	/* init work queue */
	state_wq = create_singlethread_workqueue("xmm6260_pwr_wq");
	if (!state_wq) {
		_xe("!create_workqueue()\n");
		rv = -ENOMEM;
		goto error1;
	}
	/* notifications */
	usb_register_notify(&xmm_pwr_usb_notifier);
	register_pm_notifier(&xmm_pwr_pm_notifier);
	/* request baseband irq(s) */
	rv = request_irq(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE), xmm_power_ipc_ap_wake_irq,
		IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "IPC_AP_WAKE_IRQ", NULL);
	if (rv < 0) {
		_xe("request IPC_AP_WAKE_IRQ failed\n");
		goto error2;
	}
	tegra_pm_irq_set_wake_type(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE),
		IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);
	disable_irq_wake(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	disable_irq(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	/* start power on */
	m_off = 2;
	xmm_pwr_modem_on();
	return 0;
error2:
	unregister_pm_notifier(&xmm_pwr_pm_notifier);
	usb_unregister_notify(&xmm_pwr_usb_notifier);
	destroy_workqueue(state_wq);
error1:
	regulator_put(reg_cardhu_hsic);
	reg_cardhu_hsic = NULL;
error0:
	for (r = 0; r < f; r++)
		device_remove_file(&pdev->dev, &xmm_device_attr[r]);
	pdata = NULL;
	return rv;
}

static int xmm_pwr_remove(struct platform_device *pdev)
{
	struct baseband_power_platform_data *p = pdev->dev.platform_data;
	int i;

	if (!p)
		return -ENODEV;
	/* unregister usb host controller */
	if (!xmm_pwr_hsic_unregister())
		_xw("!xmm_pwr_hsic_unregister()\n");
	__set_wake_state(IPC_AP_WAKE_UNINIT);
	/* free baseband irq */
	disable_irq_wake(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	disable_irq(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	free_irq(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE), NULL);
	unregister_pm_notifier(&xmm_pwr_pm_notifier);
	usb_unregister_notify(&xmm_pwr_usb_notifier);
	destroy_workqueue(state_wq);
	/* disable regulator LDO6 for hsic power*/
	regulator_disable(reg_cardhu_hsic);
	regulator_put(reg_cardhu_hsic);
	reg_cardhu_hsic = NULL;
	/* delete device file */
	for (i = 0; i < (ARRAY_SIZE(xmm_device_attr) - 1); i++)
		device_remove_file(&pdev->dev, &xmm_device_attr[i]);
	pdata = NULL;
	return 0;
}

static void xmm_pwr_shutdown(struct platform_device *pdev)
{
	_xd("+++\n");
	xmm_pwr_modem_off();
	return;
}

static int xmm_pwr_suspend(struct device *dev)
{
	_xd("+++\n");
//	disable_irq_wake(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	disable_irq(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	return 0;
}

static int xmm_pwr_resume(struct device *dev)
{
	_xd("+++\n");
	enable_irq(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
//	enable_irq_wake(gpio_to_irq(XMM_GPIO_IPC_AP_WAKE));
	return 0;
}

static int xmm_pwr_suspend_ni(struct device *dev)
{
	_xd("+++\n");
	return 0;
}

static int xmm_pwr_resume_ni(struct device *dev)
{
	_xd("+++\n");
	return 0;
}

static const struct dev_pm_ops xmm_pwr_dev_pm_ops = {
	.suspend_noirq = xmm_pwr_suspend_ni,
	.resume_noirq = xmm_pwr_resume_ni,
	.suspend = xmm_pwr_suspend,
	.resume = xmm_pwr_resume
};

static struct platform_driver xmm_pwr_driver = {
	.probe = xmm_pwr_probe,
	.remove = xmm_pwr_remove,
	.shutdown = xmm_pwr_shutdown,
	.driver = {
		.name = "baseband_xmm_power",
		.owner = THIS_MODULE,
		.pm = &xmm_pwr_dev_pm_ops
	}
};

int __init xmm6260_pwr_init(void)
{
	return platform_driver_register(&xmm_pwr_driver);
}

void __exit xmm6260_pwr_exit(void)
{
	platform_driver_unregister(&xmm_pwr_driver);
}
