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

#include "xmm6260_ril.h"

static struct workqueue_struct *ril_wq;
static struct device *rdev;
static struct class *rclass;
static dev_t ril_dev;
static int ril_major;
static int ril_minor;

static ssize_t store_gpio(size_t count, const char *buf, int gpio)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	switch (val) {
	case 0:
	case 1:
		break;
	default:
		return -EINVAL;
		break;
	}
	gpio_set_value(gpio, val);
	return count;
}

static ssize_t show_vbus_state(struct device *class, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(BB_GPIO_VBUS_ON));
}

static ssize_t store_vbus_state(struct device *class, struct device_attribute *attr, const char *buf, size_t count)
{
	return store_gpio(count, buf, BB_GPIO_VBUS_ON);
}

static struct device_attribute ril_device_attr[] = {
	__ATTR(bb_vbus, S_IRUSR | S_IWUSR | S_IRGRP, show_vbus_state, store_vbus_state),
	__ATTR_NULL
};

static int ril_create_files(void)
{
	int rv, f, k;

	rv = alloc_chrdev_region(&ril_dev, ril_minor, 1, "ril");
	if (rv < 0) {
		_xe("!alloc_chrdev_region()\n");
		return rv;
	}
	ril_major = MAJOR(ril_dev);
	_xd("ril_major: %d, ril_minor:%d\n", ril_major, ril_minor);
	rclass = class_create(THIS_MODULE, "ril");
	if (IS_ERR_OR_NULL(rclass)) {
		_xe("!class_create()\n");
		rv = (rclass) ? PTR_ERR(rclass) : -ENOMEM;
		goto error0;
	}
	rdev = device_create(rclass, NULL, ril_dev, NULL, "files");
	if (IS_ERR_OR_NULL(rdev)) {
		_xe("!device_create()\n");
		rv = (rdev) ? PTR_ERR(rdev) : -ENOMEM;
		goto error1;
	}
	for (f = 0; f < (ARRAY_SIZE(ril_device_attr) - 1); f++) {
		rv = device_create_file(rdev, &ril_device_attr[f]);
		if (rv < 0) {
			_xe("create file %d failed, err = %d\n", f, rv);
			goto error2;
		}
	}
	return 0;
error2:
	for (k = 0; k < f; k++)
		device_remove_file(rdev, &ril_device_attr[k]);
	device_destroy(rclass, ril_dev);
error1:
	class_destroy(rclass);
error0:
	unregister_chrdev_region(ril_dev, 1);
	return rv;
}

static void ril_remove_files(void)
{
	int f;

	for (f = 0; f < (ARRAY_SIZE(ril_device_attr) - 1); f++)
		device_remove_file(rdev, &ril_device_attr[f]);
	device_destroy(rclass, ril_dev);
	class_destroy(rclass);
	unregister_chrdev_region(ril_dev, 1);
}

static int xmm_ril_probe(struct platform_device *pdev)
{
	int rv;

	/* init work queue */
	ril_wq = create_singlethread_workqueue("xmm6260_ril_wq");
	if (!ril_wq) {
		_xe("!create_workqueue()\n");
		return -ENOMEM;
	}
	rv = ril_create_files();
	if (rv) {
		_xe("!ril_create_files()\n");
		goto error0;
	}
	rv = xmm_prox_init(rdev, ril_wq);
	if (rv) {
		_xe("!xmm_prox_init()\n");
		goto error1;
	}
	rv = xmm_sim_init(rdev, ril_wq);
	if (rv) {
		_xe("!xmm_sim_init()\n");
		goto error2;
	}
	rv = xmm_crash_init(rdev, ril_wq);
	if (rv) {
		_xe("!xmm_crash_init()\n");
		goto error3;
	}
	return 0;
error3:
	xmm_sim_exit(rdev);
error2:
	xmm_prox_exit(rdev);
error1:
	ril_remove_files();
error0:
	destroy_workqueue(ril_wq);
	return rv;
}

static int xmm_ril_remove(struct platform_device *pdev)
{
	xmm_crash_exit(rdev);
	xmm_sim_exit(rdev);
	xmm_prox_exit(rdev);
	ril_remove_files();
	destroy_workqueue(ril_wq);
	return 0;
}

static struct platform_device xmm_ril_device = {
	.name = "ril",
};

struct platform_driver xmm_ril_driver = {
	.probe = xmm_ril_probe,
	.remove = xmm_ril_remove,
//	.resume = xmm_ril_resume,
	.driver = {
		.name = "ril",
		.owner = THIS_MODULE,
	}
};

int __init xmm6260_ril_init(void)
{
	int rv;

	_xd("+++\n");
	rv = platform_device_register(&xmm_ril_device);
	if (rv) {
		_xe("platform_device_register(ril)\n");
		return rv;
	}
	rv = platform_driver_register(&xmm_ril_driver);
	if(rv) {
		_xe("platform_driver_register(ril)\n");
		platform_device_unregister(&xmm_ril_device);
		return rv;
	}
	return 0;
}

void __exit xmm6260_ril_exit(void)
{
	_xd("+++\n");
	platform_driver_unregister(&xmm_ril_driver);
	platform_device_unregister(&xmm_ril_device);
}
