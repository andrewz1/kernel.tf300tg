#ifndef _XMM6260_ACM_H
#define _XMM6260_ACM_H

#include "xmm6260.h"

#define _xe(fmt, ...)	pr_err("XMM_ACM: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#define _xw(fmt, ...)	pr_warning("XMM_ACM: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#define _xi(fmt, ...)	pr_info("XMM_ACM: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#ifdef CONFIG_XMM_ACM_DEBUG
#define _xd(fmt, ...)	pr_info("XMM_ACM: [%s] " fmt, __FUNCTION__, ##__VA_ARGS__)
#else
#define _xd(...)
#endif

/* tty major/minor */
#define XMM_TTY_MAJOR	0 /* alloc */
#define XMM_TTY_MINORS	1
#define XMM_TTY_MSTART	0

/* urb descriptors */
#define XMM_ACM_RD	4
#define XMM_ACM_WR	4
#define XMM_ACM_CT	1

/* acm output lines */
#define XMM_CDC_DTR_B		0
#define XMM_CDC_RTS_B		1
#define XMM_CDC_DTR		(1U << XMM_CDC_DTR_B)
#define XMM_CDC_RTS		(1U << XMM_CDC_RTS_B)
#define XMM_CDC_OMASK		(XMM_CDC_DTR|XMM_CDC_RTS)

/* acm input lines */
#define XMM_CDC_DCD_B		0 /* bRxCarrier */
#define XMM_CDC_DSR_B		1 /* bTxCarrier */
#define XMM_CDC_BRK_B		2 /* bBreak */
#define XMM_CDC_RI_B		3 /* bRingSignal */
#define XMM_CDC_DCD		(1U << XMM_CDC_DCD_B) /* bRxCarrier */
#define XMM_CDC_DSR		(1U << XMM_CDC_DSR_B) /* bTxCarrier */
#define XMM_CDC_BRK		(1U << XMM_CDC_BRK_B) /* bBreak */
#define XMM_CDC_RI		(1U << XMM_CDC_RI_B) /* bRingSignal */
#define XMM_CDC_IMASK		(XMM_CDC_DCD|XMM_CDC_DSR|XMM_CDC_BRK|XMM_CDC_RI)

/* acm errors */
#define XMM_CDC_FRAMING_B	4 /* bFraming */
#define XMM_CDC_PARITY_B	5 /* bParity */
#define XMM_CDC_OVERRUN_B	6 /* bOverRun */
#define XMM_CDC_FRAMING		(1U << XMM_CDC_FRAMING_B) /* bFraming */
#define XMM_CDC_PARITY		(1U << XMM_CDC_PARITY_B) /* bParity */
#define XMM_CDC_OVERRUN		(1U << XMM_CDC_OVERRUN_B) /* bOverRun */
#define XMM_CDC_EMASK		(XMM_CDC_FRAMING|XMM_CDC_PARITY|XMM_CDC_OVERRUN)

/* suspend to L3 iface flags */
//#define XMM_ACM_CIF	0
//#define XMM_ACM_DIF	1

/* check macros */
#define XMM_TTY_REG(__x)	((__x)->tdev)
#define XMM_TTY_OPEN(__x)	(xmm_acm_port_count(__x))
#define XMM_TTY_OK(__x)		(XMM_TTY_REG(__x) && XMM_TTY_OPEN(__x))
#define XMM_URB_OK(__x)		((__x) && ((__x)->transfer_buffer) && ((__x)->transfer_buffer_length))

struct xmm_urb {
	struct urb	*urb; /* urb struct */
	u8		*buf;
	dma_addr_t	dma;
	struct xmm_acm	*xp;
	int		i; /* self index */
};

struct xmm_acm {
/* basic devices */
	struct tty_port		xport;
	struct device		*tdev;	/* tty device - used as ready flag */
	struct usb_device	*dev;	/* usb device */
	struct usb_interface	*cif;	/* control interface */
	struct usb_interface	*dif;	/* data interface */
	struct pid		*rild;	/* rild pid struct - just in case */
	atomic_t		wtr;	/* write transaction counter */
	int			in_buff; /* chars in write buffer */
	int			cint;	/* ctrl int time */
	wait_queue_head_t	wtr_wait; /* wtr finish wait */
	wait_queue_head_t	open_wait;
/* ctrl URB */
	DECLARE_BITMAP(ct_used, XMM_ACM_CT);
	spinlock_t		clock; /* lock for ct_used */
	struct xmm_urb		ct[XMM_ACM_CT];
	unsigned		cpipe;
	size_t			csize;
/* read URBs */
	DECLARE_BITMAP(rb_used, XMM_ACM_RD);
	spinlock_t		rlock; /* lock for rb_used */
	struct xmm_urb		rb[XMM_ACM_RD];
	unsigned		rpipe;
	size_t			rsize;
/* write URBs */
	DECLARE_BITMAP(wb_used, XMM_ACM_WR);
	spinlock_t		wlock; /* lock for wb_used */
	struct xmm_urb		wb[XMM_ACM_WR];
	unsigned		wpipe;
	size_t			wsize;
/* serial port stuff */
//	spinlock_t		slock; /* lock for ctrlout */
	u16			net_ok; /* network connection notification is received (thread as serial CD) */
	u16			ctrlin; /* Serial Control In Lines */
	u16			ctrlout; /* Serial Control Out Lines */
/* refcount */
	struct kref		kref;
};

typedef struct xmm_urb xmm_urb_t;
typedef struct xmm_acm xmm_acm_t;

#endif  /* _XMM6260_ACM_H */
