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

static DEFINE_SPINLOCK(p_lock); /* proximity lock */
static int proximity_enabled;
static struct workqueue_struct *pwq;
static struct switch_dev prox_sdev;

static ssize_t print_prox_name(struct switch_dev *sdev, char *buf)
{
	_xd("+++\n");
	return sprintf(buf, "prox_sar_det\n");
}

static ssize_t print_prox_state(struct switch_dev *sdev, char *buf)
{
	int state = 0;

	_xd("+++\n");
	if (switch_get_state(sdev))
		state = 1;
	return sprintf(buf, "%d\n", state);
}

static void proximity_disable_irq(void)
{
	unsigned long flags;

	_xd("+++\n");
	spin_lock_irqsave(&p_lock, flags);
	if (proximity_enabled) {
		disable_irq_nosync(gpio_to_irq(BB_GPIO_SAR_DET));
		proximity_enabled = 0;
	}
	spin_unlock_irqrestore(&p_lock, flags);
}

static void proximity_enable_irq(int value)
{
	int check_value;
	unsigned long flags;

	_xd("+++\n");
	spin_lock_irqsave(&p_lock, flags);
	if (!proximity_enabled) {
		enable_irq(gpio_to_irq(BB_GPIO_SAR_DET));
		proximity_enabled = 1;
	}
	spin_unlock_irqrestore(&p_lock, flags);
	/* check if the state of gpio is correct */
	check_value = gpio_get_value(BB_GPIO_SAR_DET);
	if (value != check_value) {
		_xi("re-notify RIL with state %d\n", check_value);
		switch (check_value) {
		case 0:
			switch_set_state(&prox_sdev, 1);
			break;
		case 1:
			switch_set_state(&prox_sdev, 0);
			break;
		}
/*
		if (!check_value)
			switch_set_state(&prox_sdev, 1);
		else
			switch_set_state(&prox_sdev, 0);
*/
	}
}

static int check_sar_det_3g(void)
{
	int val = gpio_get_value(BB_GPIO_SAR_DET);

	_xi("SAR_DET_3G: %d\n", val);
	switch (val) {
	case 0:
	case 1:
		switch_set_state(&prox_sdev, val);
		break;
	}
	return val;
}

int xmm_ril_resume(struct platform_device *pdev)
{
	if (proximity_enabled) {
		_xd("check SAR_DET_3G pin");
		check_sar_det_3g();
	}
	return 0;
}

static void ril_proximity_work_handle(struct work_struct *work)
{
	int value = check_sar_det_3g();
	_xd("+++\n");
	proximity_enable_irq(value);
}
static DECLARE_WORK(prox_task, ril_proximity_work_handle);

irqreturn_t ril_proximity_interrupt(int irq, void *dev_id)
{
	_xd("+++\n");
	proximity_disable_irq();
	queue_work(pwq, &prox_task);
	return IRQ_HANDLED;
}

static ssize_t store_prox_enabled(struct device *class, struct device_attribute *attr, const char *buf, size_t count)
{
	int v;

	_xd("+++\n");
	if (sscanf(buf, "%d", &v) != 1)
		return -EINVAL;
	switch (v) {
	case 0:
	case 1:
		break;
	default:
		return -EINVAL;
		break;
	}
	_xi("prox_onoff: %d\n", v);
	/* when enabled, report the current status immediately.
	   when disabled, set state to 0 to sync with RIL */
	if (v) {
		queue_work(pwq, &prox_task);
	} else {
		proximity_disable_irq();
		switch_set_state(&prox_sdev, 0);
	}
//	return strnlen(buf, count);
	return count;
}

static ssize_t show_prox_enabled(struct device *class, struct device_attribute *attr, char *buf)
{
	_xd("+++\n");
	return sprintf(buf, "%d\n", proximity_enabled);
}

static struct device_attribute ril_prox_attr[] = {
	__ATTR(prox_onoff, S_IRUSR | S_IWUSR | S_IRGRP, show_prox_enabled, store_prox_enabled),
	__ATTR_NULL
};

int xmm_prox_init(struct device *rdev, struct workqueue_struct *wq)
{
	int rv, f;

	_xd("+++\n");
//	proximity_enabled = 0;
	pwq = wq;
	for (f = 0; f < (ARRAY_SIZE(ril_prox_attr) - 1); f++) {
		rv = device_create_file(rdev, &ril_prox_attr[f]);
		if (rv < 0) {
			_xe("create file %d failed, err = %d\n", f, rv);
			goto error0;
		}
	}
	prox_sdev.name = "ril_proximity";
	prox_sdev.print_name = print_prox_name;
	prox_sdev.print_state = print_prox_state;
	rv = switch_dev_register(&prox_sdev);
	if (rv) {
		_xe("!switch_dev_register()\n");
		goto error0;
	}
	rv = request_irq(gpio_to_irq(BB_GPIO_SAR_DET), ril_proximity_interrupt,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "IPC_SAR_DET_3G", NULL);
	if (rv < 0) {
		_xe("!request_irq()\n");
		goto error1;
	}
	disable_irq(gpio_to_irq(BB_GPIO_SAR_DET));
	return 0;
error1:
	switch_dev_unregister(&prox_sdev);
error0:
	while (f--)
		device_remove_file(rdev, &ril_prox_attr[f]);
	return rv;
}

int xmm_prox_exit(struct device *rdev)
{
	int f;

	_xd("+++\n");
	free_irq(gpio_to_irq(BB_GPIO_SAR_DET), NULL);
	switch_dev_unregister(&prox_sdev);
	for (f = 0; f < (ARRAY_SIZE(ril_prox_attr) - 1); f++)
		device_remove_file(rdev, &ril_prox_attr[f]);
	return 0;
}
