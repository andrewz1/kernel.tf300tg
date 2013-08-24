#ifndef _XMM6260_NET_H
#define _XMM6260_NET_H

#include "xmm6260.h"

#define _xe(fmt, ...)	pr_err("XMM_NET: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#define _xw(fmt, ...)	pr_warning("XMM_NET: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#define _xi(fmt, ...)	pr_info("XMM_NET: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#ifdef CONFIG_XMM_NET_DEBUG
#define _xd(fmt, ...)	pr_info("XMM_NET: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#else
#define _xd(...)
#endif

/* net if tx timeout in jiffies */
#define XMM_NET_TMO	(HZ/5) /* 0.2s */

/* max net intf count */
//#define XMM_NET_MAXIF	3

/* urb count */
#define XMM_NET_RD	8
#define XMM_NET_WR	8
#define XMM_NET_CT	1

/* carrier mask */
#define XMM_NET_CAR	0x0001

/* suspend to L3 iface flags */
//#define XMM_NET_CIF	0
//#define XMM_NET_DIF	1

/* helper macros */
#define XMM_USB_OK(__x)	((__x) && ((__x)->dev) && ((__x)->cif) && ((__x)->dif))
#define XMM_NET_OK(__x)	((__x) && ((__x)->nif) && (netif_running((__x)->nif)))
#define XMM_URB_OK(__x)	((__x) && ((__x)->transfer_buffer) && ((__x)->transfer_buffer_length))

/* netdev private struct */
struct xmm_priv {
	struct xmm_net		*xp;
	struct net_device_stats	stats;
	struct ethhdr		from_dev; /* from device to outside - src is my mac */
	struct ethhdr		to_dev; /* from outside to device - dst is my mac */
};

struct xmm_urb {
	struct urb	*urb;
	u8		*buf;
	dma_addr_t	dma;
	struct xmm_net	*xp;
	int		i; /* self index */
//	struct sk_buff *skb;
};

struct xmm_net {
/* basic devices */
//	bool			valid;	/* for disconnect */
	int			id;	/* self index */
	struct usb_device	*dev;	/* usb device */
	struct usb_interface	*cif;	/* ctrl if */
	struct usb_interface	*dif;	/* data if */
	struct net_device	*nif;	/* network device */
	struct mutex		nm;	/* network device mutex */
	struct workqueue_struct	*send_qw; /* wq for send in bg */
	struct work_struct	send_w; /* worker - wait for power and then send */
	struct usb_anchor	sa;	/* anchor for store urbs for send */
	atomic_t		wtr;	/* write transaction counter */
	int			cint;	/* interrupt interval */
	wait_queue_head_t	wtr_wait; /* wtr finish wait */
//	bool			opened;
//	DECLARE_BITMAP(if_l3, 2);	/* iface suspended to L3 */
/* ctrl URB */
	DECLARE_BITMAP(ct_used, XMM_NET_CT);
	spinlock_t		clock; /* lock for ct_used */
	struct xmm_urb		ct[XMM_NET_CT];
	unsigned		cpipe;
	size_t			csize;
/* read URB */
	DECLARE_BITMAP(rb_used, XMM_NET_RD);
	spinlock_t		rlock; /* lock for rb_used */
	struct xmm_urb		rb[XMM_NET_RD];
	unsigned		rpipe;
	size_t			rsize;
/* write URB */
	DECLARE_BITMAP(wb_used, XMM_NET_WR);
	spinlock_t		wlock; /* lock for wb_used */
	struct xmm_urb		wb[XMM_NET_WR];
	unsigned		wpipe;
	size_t			wsize;
/* ctrl if data - may be used */
	u16			net_ok; /* network connection notification is received (thread as serial CD) */
	u16			ctrlin; /* Serial Control In Lines */
	u16			ctrlout; /* Serial Control Out Lines */
/* refcount */
	struct kref		kref;
};

typedef struct xmm_priv xmm_priv_t;
typedef struct xmm_urb xmm_urb_t;
typedef struct xmm_net xmm_net_t;

#endif  /* _XMM6260_NET_H */
