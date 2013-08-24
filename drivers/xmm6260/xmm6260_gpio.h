#ifndef _XMM6260_GPIO_H
#define _XMM6260_GPIO_H

#include "xmm6260.h"

#define _xe(fmt, ...)	pr_err("XMM_GPIO: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#define _xw(fmt, ...)	pr_warning("XMM_GPIO: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#define _xi(fmt, ...)	pr_info("XMM_GPIO: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#ifdef CONFIG_XMM_NET_DEBUG
#define _xd(fmt, ...)	pr_info("XMM_GPIO: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#else
#define _xd(...)
#endif

#endif /* _XMM6260_GPIO_H */
