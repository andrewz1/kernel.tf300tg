#include <linux/kernel.h>
#include <linux/module.h>
//#include <linux/errno.h>
//#include <linux/init.h>
#include <linux/gpio.h>
#include <gpio-names.h>
#include <board-cardhu.h>

#include "xmm6260_gpio.h"

static struct gpio xmm6260_gpios[] = {
/* PWR */
	{ XMM_GPIO_BB_RST,		GPIOF_OUT_INIT_LOW,	"BB_RSTn"		}, /* Baseband Reset */
	{ XMM_GPIO_BB_ON,		GPIOF_OUT_INIT_LOW,	"BB_ON"			}, /* Baseband ON Pulse */
	{ XMM_GPIO_IPC_BB_WAKE,		GPIOF_OUT_INIT_LOW,	"IPC_BB_WAKE"		}, /* Host->Modem Wakeup */
	{ XMM_GPIO_IPC_AP_WAKE,		GPIOF_IN,		"IPC_AP_WAKE"		}, /* Modem->Host Wakeup */
	{ XMM_GPIO_IPC_HSIC_ACTIVE,	GPIOF_OUT_INIT_LOW,	"IPC_HSIC_ACTIVE"	}, /* HSIC ON PIN */
//	{ XMM_GPIO_IPC_HSIC_SUS_REQ,	GPIOF_IN,		"IPC_HSIC_SUS_REQ"	}, /* used in RIL */
	{ BB_GPIO_VBAT_ON,		GPIOF_OUT_INIT_LOW,	"BB_VBAT_ON"		}, /* Modem main power */
	{ BB_GPIO_RESET_IND,		GPIOF_IN,		"BB_RESET_IND"		}, /* n_MOD_RST_IND - not used */
	{ XMM_GPIO_IPC_BB_FORCE_CRASH,	GPIOF_OUT_INIT_LOW,	"IPC_BB_FORCE_CRASH"	}, /* Force modem in crash mode */
/* RIL */
	{ BB_GPIO_VBUS_ON,		GPIOF_OUT_INIT_LOW,	"BB_VBUS_ON"		}, /* Unknown */
	{ BB_GPIO_SW_SEL,		GPIOF_OUT_INIT_LOW,	"BB_SW_SEL"		}, /* Unknown */
	{ BB_GPIO_SAR_DET,		GPIOF_IN,		"BB_SAR_DET"		}, /* Unknown */
	{ BB_GPIO_SIM_DET,		GPIOF_IN,		"BB_SIM_DET"		}, /* SIM card detect PIN */
	{ XMM_GPIO_IPC_HSIC_SUS_REQ,	GPIOF_IN,		"IPC_SUS_REQ"		}, /* Crash mode? */
};

int __init xmm6260_gpio_init(void)
{
	_xd("+++\n");
	return gpio_request_array(xmm6260_gpios, ARRAY_SIZE(xmm6260_gpios));
}

void __exit xmm6260_gpio_exit(void)
{
	_xd("+++\n");
	gpio_free_array(xmm6260_gpios, ARRAY_SIZE(xmm6260_gpios));

}
