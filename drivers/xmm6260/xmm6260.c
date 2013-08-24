#include <linux/kernel.h>
#include <linux/module.h>

#include "xmm6260.h"

static int __init xmm6260_init(void)
{
	int rv;

	rv = xmm6260_gpio_init();
	if (rv)
		return rv;
	rv = xmm6260_pwr_init();
	if (rv)
		goto pwr_fail;
	rv = xmm6260_ril_init();
	if (rv)
		goto ril_fail;
	rv = xmm6260_acm_init();
	if (rv)
		goto acm_fail;
	rv = xmm6260_net_init();
	if (rv)
		goto net_fail;
	return 0;
net_fail:
	xmm6260_acm_exit();
acm_fail:
	xmm6260_ril_exit();
ril_fail:
	xmm6260_pwr_exit();
pwr_fail:
	xmm6260_gpio_exit();
	return rv;
}

static void __exit xmm6260_exit(void)
{
	xmm6260_net_exit();
	xmm6260_acm_exit();
	xmm6260_ril_exit();
	xmm6260_pwr_exit();
	xmm6260_gpio_exit();
}

module_init(xmm6260_init);
module_exit(xmm6260_exit);
