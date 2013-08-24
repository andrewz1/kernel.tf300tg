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

static struct workqueue_struct *cwq;
static struct switch_dev crash_sdev;
static int do_crash_dump;

static ssize_t crash_print_name(struct switch_dev *sdev, char *buf)
{
	_xd("+++\n");
	return sprintf(buf, "crash_dump_det\n");
}

static ssize_t crash_print_state(struct switch_dev *sdev, char *buf)
{
	int s;

	_xd("+++\n");
	switch((s = switch_get_state(sdev))) {
	case 0:
	case 1:
		return sprintf(buf, "%d\n", s);
		break;
	}
	return -EINVAL;
}

static ssize_t show_cdump_state(struct device *class, struct device_attribute *attr, char *buf)
{
	_xd("+++\n");
	return sprintf(buf, "%d\n", do_crash_dump);
}

static ssize_t store_cdump_state(struct device *class, struct device_attribute *attr, const char *buf, size_t count)
{
	int val;

	_xd("+++\n");
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	switch (val) {
	case 0:
	case 1:
		do_crash_dump = val;
		break;
	default:
		return -EINVAL;
		break;
	}
	return count;
}

static struct device_attribute ril_crash_attr[] = {
	__ATTR(crash_dump_onoff, S_IRUSR | S_IWUSR | S_IRGRP, show_cdump_state, store_cdump_state),
	__ATTR_NULL
};

/* used in PWR */
void ril_crash_set(void)
{
	gpio_set_value(BB_GPIO_SW_SEL, 1);
	mdelay(1);
	gpio_set_value(BB_GPIO_VBUS_ON, 1);
	mdelay(1);
}

/* used in PWR */
void ril_crash_clear(void)
{
	gpio_set_value(BB_GPIO_SW_SEL, 0);
	mdelay(1);
	gpio_set_value(BB_GPIO_VBUS_ON, 0);
	mdelay(1);
}

static void ril_crash_dump_work(struct work_struct *work)
{
	_xd("+++\n");
	disable_irq(gpio_to_irq(XMM_GPIO_IPC_HSIC_SUS_REQ));
	xmm6260_modem_crash_dump(0);
	msleep(200);
	gpio_set_value(BB_GPIO_SW_SEL, 1);
	mdelay(5);
	gpio_set_value(BB_GPIO_VBUS_ON, 1);
	mdelay(5);
	xmm6260_modem_crash_dump(1);
	switch_set_state(&crash_sdev, 1);
}
static DECLARE_WORK(crash_work, ril_crash_dump_work);

irqreturn_t ril_ipc_sus_req_irq(int irq, void *dev_id)
{
//	_xd("+++\n");
	if (do_crash_dump)
		if (gpio_get_value(XMM_GPIO_IPC_HSIC_SUS_REQ) == 1) {
			_xi("do_crash_dump is on!\n");
			queue_work(cwq, &crash_work);
		}
	return IRQ_HANDLED;
}

int xmm_crash_init(struct device *rdev, struct workqueue_struct *wq)
{
	int rv, f, r;

	_xd("+++\n");
	cwq = wq;
	for (f = 0; f < (ARRAY_SIZE(ril_crash_attr) - 1); f++) {
		rv = device_create_file(rdev, &ril_crash_attr[f]);
		if (rv < 0) {
			_xe("create file %d failed, err = %d\n", f, rv);
			goto error0;
		}
	}
	crash_sdev.name = "crash_dump_det";
	crash_sdev.print_name = crash_print_name;
	crash_sdev.print_state = crash_print_state;
	rv = switch_dev_register(&crash_sdev);
	if (rv < 0) {
		_xe("!switch_dev_register()\n");
		goto error0;
	}
	rv = request_irq(gpio_to_irq(XMM_GPIO_IPC_HSIC_SUS_REQ), ril_ipc_sus_req_irq,
		IRQF_TRIGGER_RISING, "IPC_MOD_SUS_REQ", NULL);
	if (rv < 0) {
		_xe("!request_irq()\n");
		goto error1;
	}
	return 0;
error1:
	switch_dev_unregister(&crash_sdev);
error0:
	for (r = 0; r < f; r++)
		device_remove_file(rdev, &ril_crash_attr[r]);
	return rv;
}

int xmm_crash_exit(struct device *rdev)
{
	int r;

	_xd("+++\n");
	free_irq(gpio_to_irq(XMM_GPIO_IPC_HSIC_SUS_REQ), NULL);
	switch_dev_unregister(&crash_sdev);
	for (r = 0; r < (ARRAY_SIZE(ril_crash_attr) - 1); r++)
		device_remove_file(rdev, &ril_crash_attr[r]);
	return 0;
}
