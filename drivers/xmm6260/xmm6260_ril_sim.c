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
#include <linux/switch.h>

#include <asm/io.h>
#include <pm-irq.h>
#include <gpio-names.h>
#include <board-cardhu.h>

#include "xmm6260_ril.h"

#define SIM_PLUG_STATE_ABSENT  1
#define SIM_PLUG_STATE_PLUGGED 0

#define SYSFS_VAL_PLUG_STATE_FREEZED 1
#define SYSFS_VAL_PLUG_STATE_ACTIVATED 0

static DEFINE_SPINLOCK(s_lock); /* modem_pwr lock */
static bool modem_pwr = false;

static struct workqueue_struct *swq;
static struct switch_dev sim_sdev;
static int sim_plug_state;
static bool is_sim_plug_state_freezed = false;

static ssize_t print_sim_plug_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "ril_sim_plug\n");
}

static ssize_t print_sim_plug_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", sim_plug_state);
}

static int get_sim_plug_state(void)
{
	_xd("+++\n");
	if (1 == gpio_get_value(BB_GPIO_SIM_DET))
		return SIM_PLUG_STATE_PLUGGED;
	else
		return SIM_PLUG_STATE_ABSENT;
}

static void hotplug_work_handle(struct work_struct *work)
{
	int state;

	_xd("+++\n");
	// workaround: if reseting modem, some noisy IRQs would be raised.
	// Ignore plug state here, and we'll correct the state later when
	// finishing reseting.
	if (is_sim_plug_state_freezed) {
		_xd("sim state freezed - noop\n");
		return;
	}
	state = get_sim_plug_state();
	if (state == sim_plug_state) {
		_xd("sim state unchanged - noop\n");
		return;
	}
	_xd("sim state changed %d -> %d\n", sim_plug_state, state);
	sim_plug_state = state;
	switch_set_state(&sim_sdev, sim_plug_state);
	// wake up modem to read card state
	xmm6260_ril_set_L0();
}
static DECLARE_DEFERRED_WORK(hotplug_work_task, hotplug_work_handle);

static void freeze_sim_plug_state_work_handle(struct work_struct *work)
{
	_xi("freeze changing plug state\n");
	is_sim_plug_state_freezed = true;
}
static DECLARE_WORK(modem_reset_start_task, freeze_sim_plug_state_work_handle);

static void release_sim_plug_state_work_handle(struct work_struct *work)
{
	_xi("active changing plug state\n");
	is_sim_plug_state_freezed = false;
	hotplug_work_handle(work);
}
static DECLARE_WORK(modem_reset_finish_task, release_sim_plug_state_work_handle);

static ssize_t store_hotplug_detect_state(struct device *class, struct device_attribute *attr, const char *buf, size_t count)
{
	int val;

	_xd("+++\n");
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	switch (val) {
	case SYSFS_VAL_PLUG_STATE_FREEZED:
		queue_work(swq, &modem_reset_start_task);
		break;
	case SYSFS_VAL_PLUG_STATE_ACTIVATED:
		queue_work(swq, &modem_reset_finish_task);
		break;
	default:
		return -EINVAL;
		break;
	}
	return count;
}

irqreturn_t sim_interrupt_handle(int irq, void *dev_id)
{
//	const unsigned long WORK_DELAY = HZ / 10;
	queue_delayed_work(swq, &hotplug_work_task, (HZ/10));
	return IRQ_HANDLED;
}

static ssize_t show_hotplug_detect_state(struct device *class, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",
		(is_sim_plug_state_freezed) ? SYSFS_VAL_PLUG_STATE_FREEZED : SYSFS_VAL_PLUG_STATE_ACTIVATED);
}

static struct device_attribute ril_sim_attr[] = {
	__ATTR(stop_hotplug_detect, S_IRUSR | S_IWUSR | S_IRGRP, show_hotplug_detect_state, store_hotplug_detect_state),
	__ATTR_NULL
};

void xmm6260_sync_state(bool pwr)
{
	unsigned long flags;

	_xd("+++\n");
	spin_lock_irqsave(&s_lock, flags);
	if (pwr)
		enable_irq(gpio_to_irq(BB_GPIO_SIM_DET));
	else
//		disable_irq_nosync(gpio_to_irq(BB_GPIO_SIM_DET));
		disable_irq(gpio_to_irq(BB_GPIO_SIM_DET));
	modem_pwr = pwr;
	spin_unlock_irqrestore(&s_lock, flags);
}

int xmm_sim_init(struct device *rdev, struct workqueue_struct *wq)
{
	int rv, f;
	unsigned long flags;

	_xd("+++\n");
	swq = wq;
	sim_plug_state = get_sim_plug_state();
	for (f = 0; f < (ARRAY_SIZE(ril_sim_attr) - 1); f++) {
		rv = device_create_file(rdev, &ril_sim_attr[f]);
		if (rv < 0) {
			_xe("create file %d failed, err = %d\n", f, rv);
			goto error0;
		}
	}
	sim_sdev.name = "ril_sim_plug";
	sim_sdev.print_name = print_sim_plug_name;
	sim_sdev.print_state = print_sim_plug_state;
	rv = switch_dev_register(&sim_sdev);
	if (rv < 0) {
		_xe("!switch_dev_register()\n");
		goto error0;
	}
	// Because switch_dev_register will initiate sdev.state to 0,
	// sdev.state will be initiated after switch_dev_register.
	switch_set_state(&sim_sdev, sim_plug_state);
	rv = request_irq(gpio_to_irq(BB_GPIO_SIM_DET), sim_interrupt_handle,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "IPC_SIM_CARD_DET", NULL);
	if (rv < 0) {
		_xe("!request_irq()\n");
		goto error1;
	}
	spin_lock_irqsave(&s_lock, flags);
	if (modem_pwr)
		enable_irq(gpio_to_irq(BB_GPIO_SIM_DET));
	else
		disable_irq(gpio_to_irq(BB_GPIO_SIM_DET));
	spin_unlock_irqrestore(&s_lock, flags);
	tegra_pm_irq_set_wake_type(gpio_to_irq(BB_GPIO_SIM_DET), IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
	enable_irq_wake(gpio_to_irq(BB_GPIO_SIM_DET));
	return 0;
error1:
	switch_dev_unregister(&sim_sdev);
error0:
	while (f--)
		device_remove_file(rdev, &ril_sim_attr[f]);
	return rv;
}

int xmm_sim_exit(struct device *rdev)
{
	int f;

	_xd("+++\n");
	free_irq(gpio_to_irq(BB_GPIO_SIM_DET), NULL);
	switch_dev_unregister(&sim_sdev);
	for (f = 0; f < (ARRAY_SIZE(ril_sim_attr) - 1); f++)
		device_remove_file(rdev, &ril_sim_attr[f]);
	return 0;
}
