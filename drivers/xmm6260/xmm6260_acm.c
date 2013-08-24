#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/mutex.h>

#include "xmm6260_acm.h"

static const struct usb_device_id xmm6260_acm_ids[] = {
	{ USB_DEVICE(XMM_MAIN_VID, XMM_MAIN_PID) },
	{ USB_DEVICE(XMM_CRASH_VID, XMM_CRASH_PID) },
	{ }
};
MODULE_DEVICE_TABLE(usb, xmm6260_acm_ids);

static struct usb_driver xmm_acm_driver;
static struct tty_driver *xtty;

static __always_inline xmm_acm_t *xmm_acm_tty_xp(struct tty_struct *tty)
{
	if (tty && tty->dev)
		return dev_get_drvdata(tty->dev);
	else
		return NULL;
}

static __always_inline xmm_acm_t *xmm_acm_port_xp(struct tty_port *port)
{
	if (port && port->tty && port->tty->dev)
		return dev_get_drvdata(port->tty->dev);
	else
		return NULL;
}

static __always_inline int xmm_acm_port_count(xmm_acm_t *xp)
{
	unsigned long flags;
	int rv;

	spin_lock_irqsave(&xp->xport.lock, flags);
	rv = xp->xport.count;
	spin_unlock_irqrestore(&xp->xport.lock, flags);
	return rv;
}

static __always_inline int xmm_acm_cget_msg(xmm_acm_t *xp, u8 req, u16 val, void *data, u16 size)
{
	return usb_control_msg(xp->dev, usb_rcvctrlpipe(xp->dev, 0), req,
		USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE, val,
		xp->cif->cur_altsetting->desc.bInterfaceNumber, data, size, HZ);
}

static __always_inline int xmm_acm_cset_msg(xmm_acm_t *xp, u8 req, u16 val, void *data, u16 size)
{
	return usb_control_msg(xp->dev, usb_sndctrlpipe(xp->dev, 0), req,
		USB_DIR_OUT|USB_TYPE_CLASS|USB_RECIP_INTERFACE, val,
		xp->cif->cur_altsetting->desc.bInterfaceNumber, data, size, HZ);
}

static int xmm_acm_set_ctrl(xmm_acm_t *xp)
{
	int rv;
	u16 c;

	_xd("+++\n");
	c = xp->ctrlout;
	rv = xmm_acm_cset_msg(xp, USB_CDC_REQ_SET_CONTROL_LINE_STATE, c, NULL, 0);
	return (rv < 0) ? rv : 0;
}

/* write URB ops */

static int xmm_acm_submit_wr_urb(xmm_acm_t *xp, int id, gfp_t mf)
{
	unsigned long flags;
	int rv;

	if (!XMM_URB_OK(xp->wb[id].urb))
		return -EINVAL;
	atomic_inc(&xp->wtr);
	rv = usb_submit_urb(xp->wb[id].urb, mf);
	spin_lock_irqsave(&xp->wlock, flags);
	if (rv) /* error */
		__clear_bit(id, xp->wb_used);
	else
		xp->in_buff += xp->wb[id].urb->transfer_buffer_length;
	spin_unlock_irqrestore(&xp->wlock, flags);
	if (rv) {
		atomic_dec(&xp->wtr);
		smp_mb();
		if (waitqueue_active(&xp->wtr_wait))
			wake_up(&xp->wtr_wait);
	}
	return rv;
}

static void xmm_acm_wr_cb(struct urb *urb)
{
	xmm_urb_t *wb = urb->context;
	unsigned long flags;

	spin_lock_irqsave(&wb->xp->wlock, flags);
	wb->xp->in_buff -= urb->transfer_buffer_length;
	__clear_bit(wb->i, wb->xp->wb_used);
	spin_unlock_irqrestore(&wb->xp->wlock, flags);
	atomic_dec(&wb->xp->wtr);
	smp_mb();
	if (waitqueue_active(&wb->xp->wtr_wait))
		wake_up(&wb->xp->wtr_wait);
	return;
}

static void xmm_acm_kill_wr(xmm_acm_t *xp)
{
	unsigned long flags;
	int i;

	if (!wait_event_timeout(xp->wtr_wait, !atomic_read(&xp->wtr), HZ))
		_xw("wtr timeout!\n");
	for (i = 0; i < XMM_ACM_WR; i++) {
		spin_lock_irqsave(&xp->wlock, flags);
		if (!test_bit(i, xp->wb_used)) {
			spin_unlock_irqrestore(&xp->wlock, flags);
			continue;
		}
		spin_unlock_irqrestore(&xp->wlock, flags);
		usb_kill_urb(xp->wb[i].urb);
		spin_lock_irqsave(&xp->wlock, flags);
		__clear_bit(i, xp->wb_used);
		spin_unlock_irqrestore(&xp->wlock, flags);
	}
}

static void xmm_acm_free_wr(xmm_acm_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_ACM_WR; i++) {
		if (xp->wb[i].buf) {
			usb_free_coherent(xp->dev, xp->wsize, xp->wb[i].buf, xp->wb[i].dma);
			xp->wb[i].buf = NULL;
		}
		if (xp->wb[i].urb) {
			usb_free_urb(xp->wb[i].urb);
			xp->wb[i].urb = NULL;
		}
		spin_lock_irqsave(&xp->wlock, flags);
		__set_bit(i, xp->wb_used);
		spin_unlock_irqrestore(&xp->wlock, flags);
	}
	return;
}

static int xmm_acm_alloc_wr(xmm_acm_t *xp)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&xp->wlock, flags);
	bitmap_fill(xp->wb_used, XMM_ACM_WR);
	spin_unlock_irqrestore(&xp->wlock, flags);
	for (i = 0; i < XMM_ACM_WR; i++) {
		xp->wb[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!xp->wb[i].urb)
			goto error;
		xp->wb[i].buf = usb_alloc_coherent(xp->dev, xp->wsize, GFP_KERNEL, &xp->wb[i].dma);
		if (!xp->wb[i].buf)
			goto error;
		usb_fill_bulk_urb(xp->wb[i].urb, xp->dev, xp->wpipe, xp->wb[i].buf, 0, xmm_acm_wr_cb, &xp->wb[i]);
		xp->wb[i].urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		xp->wb[i].urb->transfer_flags |= URB_ZERO_PACKET;
		xp->wb[i].urb->transfer_dma = xp->wb[i].dma;
		xp->wb[i].xp = xp;
		xp->wb[i].i = i;
		spin_lock_irqsave(&xp->wlock, flags);
		__clear_bit(i, xp->wb_used);
		spin_unlock_irqrestore(&xp->wlock, flags);
	}
	return 0;
error:
	xmm_acm_free_wr(xp);
	return -ENOMEM;
}

/* read URB ops */

static int xmm_acm_submit_rd_urb(xmm_acm_t *xp, int id, gfp_t mf, bool reuse)
{
	unsigned long flags;
	int rv;

	if (!XMM_URB_OK(xp->rb[id].urb))
		return -EINVAL;
	if (!reuse) { /* new urb */
		spin_lock_irqsave(&xp->rlock, flags);
		rv = __test_and_set_bit(id, xp->rb_used);
		spin_unlock_irqrestore(&xp->rlock, flags);
		if (rv) /* bit not changed - urb is busy */
			return -EINVAL;
	}
	rv = usb_submit_urb(xp->rb[id].urb, mf);
	if (rv) {
		spin_lock_irqsave(&xp->rlock, flags);
		__clear_bit(id, xp->rb_used);
		spin_unlock_irqrestore(&xp->rlock, flags);
	}
	return rv;
}

static void xmm_acm_submit_urb_to_tty(xmm_acm_t *xp, struct urb *urb)
{
	struct tty_struct *tty;
	unsigned char *buf;
	int blen, tlen;

	tty = tty_port_tty_get(&xp->xport);
	if (!tty) {
		_xe("!tty_port_tty_get()\n");
		return;
	}
	buf = urb->transfer_buffer;
	blen = urb->actual_length;
	while (blen) {
		tlen = tty_buffer_request_room(tty, blen);
		tty_insert_flip_string(tty, buf, tlen);
		tty_flip_buffer_push(tty);
		buf += tlen;
		blen -= tlen;
	}
	tty_kref_put(tty);
	return;
}

static void xmm_acm_rd_cb(struct urb *urb)
{
	xmm_urb_t *rb = urb->context;
	unsigned long flags;
	int rv;

	switch (urb->status) {
	case 0: /* OK */
		break;
	case -ENOENT: /* usb_kill_urb */
	case -ECONNRESET: /* usb_unlink_urb with URB_ASYNC_UNLINK */
	case -ESHUTDOWN:
		goto shutdown;
	default: /* broken urb */
		goto resubmit;
	}
	if (XMM_TTY_OPEN(rb->xp) && urb->actual_length) /* send to TTY if opened */
		xmm_acm_submit_urb_to_tty(rb->xp, urb);
resubmit: /* don't touch used bits in this case! */
	rv = xmm_acm_submit_rd_urb(rb->xp, rb->i, GFP_ATOMIC, true);
	if (rv) /* used bit cleared in xmm_acm_submit_rd_urb */
		_xe("Resubmit read URB failed, i:%d, rv:%d\n", rb->i, rv);
	return;
shutdown: /* mark as free */
	spin_lock_irqsave(&rb->xp->rlock, flags);
	__clear_bit(rb->i, rb->xp->rb_used);
	spin_unlock_irqrestore(&rb->xp->rlock, flags);
	return;
}

static void xmm_acm_kill_rd(xmm_acm_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_ACM_RD; i++) {
		spin_lock_irqsave(&xp->rlock, flags);
		if (!test_bit(i, xp->rb_used)) {
			spin_unlock_irqrestore(&xp->rlock, flags);
			continue;
		}
		spin_unlock_irqrestore(&xp->rlock, flags);
		usb_kill_urb(xp->rb[i].urb);
		spin_lock_irqsave(&xp->rlock, flags);
		__clear_bit(i, xp->rb_used);
		spin_unlock_irqrestore(&xp->rlock, flags);
	}
}

static int xmm_acm_submit_rd(xmm_acm_t *xp, gfp_t mf)
{
	int rv, i;

	for (i = 0; i < XMM_ACM_RD; i++) {
		rv = xmm_acm_submit_rd_urb(xp, i, mf, false);
		if (rv)
			break;
	}
	if (rv)
		xmm_acm_kill_rd(xp);
	return rv;
}

static void xmm_acm_free_rd(xmm_acm_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_ACM_RD; i++) {
		if (xp->rb[i].buf) {
			usb_free_coherent(xp->dev, xp->rsize, xp->rb[i].buf, xp->rb[i].dma);
			xp->rb[i].buf = NULL;
		}
		if (xp->rb[i].urb) {
			usb_free_urb(xp->rb[i].urb);
			xp->rb[i].urb = NULL;
		}
		spin_lock_irqsave(&xp->rlock, flags);
		__set_bit(i, xp->rb_used);
		spin_unlock_irqrestore(&xp->rlock, flags);
	}
	return;
}

static int xmm_acm_alloc_rd(xmm_acm_t *xp)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&xp->rlock, flags);
	bitmap_fill(xp->rb_used, XMM_ACM_RD);
	spin_unlock_irqrestore(&xp->rlock, flags);
	for (i = 0; i < XMM_ACM_RD; i++) {
		xp->rb[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!xp->rb[i].urb)
			goto error;
		xp->rb[i].buf = usb_alloc_coherent(xp->dev, xp->rsize, GFP_KERNEL, &xp->rb[i].dma);
		if (!xp->rb[i].buf)
			goto error;
		usb_fill_bulk_urb(xp->rb[i].urb, xp->dev, xp->rpipe, xp->rb[i].buf, xp->rsize, xmm_acm_rd_cb, &xp->rb[i]);
		xp->rb[i].urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		xp->rb[i].urb->transfer_dma = xp->rb[i].dma;
		xp->rb[i].xp = xp;
		xp->rb[i].i = i;
		spin_lock_irqsave(&xp->rlock, flags);
		__clear_bit(i, xp->rb_used);
		spin_unlock_irqrestore(&xp->rlock, flags);
	}
	return 0;
error:
	xmm_acm_free_rd(xp);
	return -ENOMEM;
}

/* ctrl URB ops */

static int xmm_acm_submit_ct_urb(xmm_acm_t *xp, int id, gfp_t mf, bool reuse)
{
	unsigned long flags;
	int rv;

	if (!XMM_URB_OK(xp->ct[id].urb))
		return -EINVAL;
	if (!reuse) {
		spin_lock_irqsave(&xp->clock, flags);
		rv = __test_and_set_bit(id, xp->ct_used);
		spin_unlock_irqrestore(&xp->clock, flags);
		if (rv) /* bit not changed - urb is busy */
			return -EINVAL;
	}
	rv = usb_submit_urb(xp->ct[id].urb, mf);
	if (rv) {
		spin_lock_irqsave(&xp->clock, flags);
		__clear_bit(id, xp->ct_used);
		spin_unlock_irqrestore(&xp->clock, flags);
	}
	return rv;
}

static void xmm_acm_ct_cb(struct urb *urb)
{
	xmm_urb_t *ct = urb->context;
	unsigned long flags;
	int rv;
	u16 net_ok, ctrlin;
	struct usb_cdc_notification *n;
	unsigned char *b;

	switch (urb->status) {
	case 0: /* OK */
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		goto shutdown;
	default:
		goto resubmit;
	}
/* do something with urb data */
	if (urb->actual_length) {
		n = urb->transfer_buffer;
		switch (n->bNotificationType) {
		case USB_CDC_NOTIFY_NETWORK_CONNECTION:
			net_ok = le16_to_cpu(n->wValue);
			if (ct->xp->net_ok != net_ok) {
				ct->xp->net_ok = net_ok;
				_xd("USB_CDC_NOTIFY_NETWORK_CONNECTION: %d\n", net_ok);
				smp_mb();
				if (waitqueue_active(&ct->xp->open_wait))
					wake_up(&ct->xp->open_wait);
			}
			break;
		case USB_CDC_NOTIFY_SERIAL_STATE:
			b = urb->transfer_buffer + sizeof(struct usb_cdc_notification);
			ctrlin = le16_to_cpup((__le16 *)b);
			if (ct->xp->ctrlin != ctrlin) {
				ct->xp->ctrlin = ctrlin;
				_xd("USB_CDC_NOTIFY_SERIAL_STATE: 0x%04x\n", ctrlin);
				smp_mb();
			}
			break;
		default:
			_xd("bNotificationType: %d\n", n->bNotificationType);
			break;
		}
	}
/* resubmit urb */
resubmit:
	rv = xmm_acm_submit_ct_urb(ct->xp, ct->i, GFP_ATOMIC, true);
	if (rv)
		_xe("Resubmit ctrl URB failed, rv:%d\n", rv);
	return;
shutdown:
	spin_lock_irqsave(&ct->xp->clock, flags);
	__clear_bit(ct->i, ct->xp->ct_used);
	spin_unlock_irqrestore(&ct->xp->clock, flags);
	return;
}

static void xmm_acm_kill_ct(xmm_acm_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_ACM_CT; i++) {
		spin_lock_irqsave(&xp->clock, flags);
		if (!test_bit(i, xp->ct_used)) {
			spin_unlock_irqrestore(&xp->clock, flags);
			continue;
		}
		spin_unlock_irqrestore(&xp->clock, flags);
		usb_kill_urb(xp->ct[i].urb);
		spin_lock_irqsave(&xp->clock, flags);
		__clear_bit(i, xp->ct_used);
		spin_unlock_irqrestore(&xp->clock, flags);
	}
}

static int xmm_acm_submit_ct(xmm_acm_t *xp, gfp_t mf)
{
	int rv,i;

	for (i = 0; i < XMM_ACM_CT; i++) {
		rv = xmm_acm_submit_ct_urb(xp, i, mf, false);
		if (rv)
			break;
	}
	if (rv)
		xmm_acm_kill_ct(xp);
	return rv;
}

static void xmm_acm_free_ct(xmm_acm_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_ACM_CT; i++) {
		if (xp->ct[i].buf) {
			usb_free_coherent(xp->dev, xp->csize, xp->ct[i].buf, xp->ct[i].dma);
			xp->ct[i].buf = NULL;
		}
		if (xp->ct[i].urb) {
			usb_free_urb(xp->ct[i].urb);
			xp->ct[i].urb = NULL;
		}
		spin_lock_irqsave(&xp->clock, flags);
		__set_bit(i, xp->ct_used);
		spin_unlock_irqrestore(&xp->clock, flags);
	}
	return;
}

static int xmm_acm_alloc_ct(xmm_acm_t *xp)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&xp->clock, flags);
	bitmap_fill(xp->ct_used, XMM_ACM_CT);
	spin_unlock_irqrestore(&xp->clock, flags);
	for (i = 0; i < XMM_ACM_CT; i++) {
		xp->ct[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!xp->ct[i].urb)
			goto error;
		xp->ct[i].buf = usb_alloc_coherent(xp->dev, xp->csize, GFP_KERNEL, &xp->ct[i].dma);
		if (!xp->ct[i].buf)
			goto error;
		usb_fill_int_urb(xp->ct[i].urb, xp->dev, xp->cpipe, xp->ct[i].buf, xp->csize, xmm_acm_ct_cb, &xp->ct[i], xp->cint);
		xp->ct[i].urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		xp->ct[i].urb->transfer_dma = xp->ct[i].dma;
		xp->ct[i].xp = xp;
		xp->ct[i].i = i;
		spin_lock_irqsave(&xp->clock, flags);
		__clear_bit(i, xp->ct_used);
		spin_unlock_irqrestore(&xp->clock, flags);
	}
	return 0;
error:
	xmm_acm_free_ct(xp);
	return -ENOMEM;
}

/* end URB ops */

static int xmm_acm_tty_open(struct tty_struct *tty, struct file *filp)
{
	xmm_acm_t *xp = xmm_acm_tty_xp(tty);
	int rv;

	_xd("+++\n");
	if (!xp || !XMM_TTY_REG(xp)) /* device removed */
		return -ENODEV;
	if ((rv = XMM_TTY_OPEN(xp))) {
		_xe("Open count: %d\n", rv);
		return -EBUSY;
	}
	tty->driver_data = xp;
	tty->low_latency = 1;
	rv = tty_port_open(&xp->xport, tty, filp); /* we submit urbs in activate */
	if (rv) {
		_xe("!tty_port_open()!\n");
		return rv;
	}
	xp->rild = task_pgrp(current);
	_xd("rild pid: %d\n", pid_nr(xp->rild));
	return 0;
}

static void xmm_acm_tty_close(struct tty_struct *tty, struct file *filp)
{
	xmm_acm_t *xp = xmm_acm_tty_xp(tty);

	_xd("+++\n");
	if (!xp || !XMM_TTY_OK(xp))
		return;
	tty_port_close(&xp->xport, tty, filp);
}

static void xmm_acm_tty_hangup(struct tty_struct *tty)
{
	xmm_acm_t *xp = xmm_acm_tty_xp(tty);

	_xd("+++\n");
	if (!xp || !XMM_TTY_OK(xp))
		return;
	if (xp->rild) {
		_xd("killing rild pid: %d\n", pid_nr(xp->rild));
		kill_pgrp(xp->rild, SIGTERM, 1);
		xp->rild = NULL;
	}
	tty_port_hangup(&xp->xport);
}

static int xmm_acm_tty_write(struct tty_struct *tty, const unsigned char *wbuf, int count)
{
	xmm_acm_t *xp = xmm_acm_tty_xp(tty);
	unsigned long flags;
	int i, rv, len;

	spin_lock_irqsave(&xp->wlock, flags);
	i = find_first_zero_bit(xp->wb_used, XMM_ACM_WR);
	if (i < XMM_ACM_WR) {
		__set_bit(i, xp->wb_used);
		spin_unlock_irqrestore(&xp->wlock, flags);
	} else { /* no free urbs - this is bug */
		spin_unlock_irqrestore(&xp->wlock, flags);
		return -ENOMEM;
	}
	len = (count < xp->wsize) ? count : xp->wsize;
	memcpy(xp->wb[i].urb->transfer_buffer, wbuf, len);
	xp->wb[i].urb->transfer_buffer_length = len;
	rv = xmm_acm_submit_wr_urb(xp, i, GFP_KERNEL);
	if (rv)
		return rv;
	return len;
}

static int xmm_acm_tty_write_room(struct tty_struct *tty)
{
	xmm_acm_t *xp = xmm_acm_tty_xp(tty);
	unsigned long flags;
//	int rv = 0;
	int i, rv;

	spin_lock_irqsave(&xp->wlock, flags);
//	if (find_first_zero_bit(xp->wb_used, XMM_ACM_WR) < XMM_ACM_WR)
//		rv = xp->wsize;
	for (i = 0, rv = 0; i < XMM_ACM_WR; i++)
		if (!test_bit(i, xp->wb_used))
			rv++;
	spin_unlock_irqrestore(&xp->wlock, flags);
//	return rv;
	return (rv * xp->wsize);
}

static int xmm_acm_tty_chars_in_buffer(struct tty_struct *tty)
{
	xmm_acm_t *xp = xmm_acm_tty_xp(tty);
	unsigned long flags;
	int rv;

	spin_lock_irqsave(&xp->wlock, flags);
	rv = xp->in_buff;
	spin_unlock_irqrestore(&xp->wlock, flags);
	return rv;
}

static int xmm_acm_tty_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
{
	_xd("ioctl:0x%04x\n", cmd);
	return tty_mode_ioctl(tty, (struct file *)1, cmd, arg);
}

static void xmm_acm_tty_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
	_xd("+++\n");
	tty_termios_copy_hw(tty->termios, old_termios);
//	_xd("rate:%u:%u\n", tty_termios_baud_rate(tty->termios), tty_termios_input_baud_rate(tty->termios));
	return;
}

static int xmm_acm_tty_tiocmget(struct tty_struct *tty)
{
	xmm_acm_t *xp = xmm_acm_tty_xp(tty);
	int rv = 0;

	_xd("+++\n");
	if (xp->ctrlout & XMM_CDC_DTR)
		rv |= TIOCM_DTR;
	if (xp->ctrlout & XMM_CDC_RTS)
		rv |= TIOCM_RTS;
	if (xp->ctrlin & XMM_CDC_DCD)
		rv |= TIOCM_CD;
	if (xp->ctrlin & XMM_CDC_DSR)
		rv |= TIOCM_DSR;
//	if (xp->ctrlin & XMM_CDC_BRK)
	if (xp->ctrlin & XMM_CDC_RI)
		rv |= TIOCM_RI;
	rv |= TIOCM_CTS;
	return rv;
}

static int xmm_acm_tty_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
{
	xmm_acm_t *xp = xmm_acm_tty_xp(tty);
	u16 ctrlout = xp->ctrlout;
	u16 _set = 0, _clear = 0;

	_xd("+++\n");
	if (set & TIOCM_DTR)
		_set |= XMM_CDC_DTR;
	if (set & XMM_CDC_RTS)
		_set |= XMM_CDC_RTS;
	if (clear & TIOCM_DTR)
		_clear |= XMM_CDC_DTR;
	if (clear & XMM_CDC_RTS)
		_clear |= XMM_CDC_RTS;
	ctrlout &= ~_clear;
	ctrlout |= _set;
	if (xp->ctrlout == ctrlout)
		return 0;
	xp->ctrlout = ctrlout;
	smp_mb();
	return xmm_acm_set_ctrl(xp);
}

/* port ops */

static int xmm_acm_port_carrier_raised(struct tty_port *port)
{
	xmm_acm_t *xp = xmm_acm_port_xp(port);

	_xd("+++\n");
	if (!xp)
		return 0;
	return (xp->net_ok) ? 1 : 0;
}

static void xmm_acm_port_dtr_rts(struct tty_port *port, int on)
{
	xmm_acm_t *xp = xmm_acm_port_xp(port);

	_xd("+++:%d\n", on);
	if (!xp)
		return;
	if (on)
		xp->ctrlout |= (XMM_CDC_DTR|XMM_CDC_RTS);
	else
		xp->ctrlout &= ~(XMM_CDC_DTR|XMM_CDC_RTS);
	smp_mb();
	xmm_acm_set_ctrl(xp);
	return;
}

static void xmm_acm_port_shutdown(struct tty_port *port)
{
	xmm_acm_t *xp = xmm_acm_port_xp(port);

	_xd("+++\n");
	if (!xp)
		return;
	xmm_acm_kill_wr(xp);
	xmm_acm_kill_rd(xp);
	xmm_acm_kill_ct(xp);
	atomic_set(&xp->wtr, 0); /* write transaction counter */
	xp->in_buff = 0;
	smp_mb();
	return;
}

static void xmm_acm_port_drop(struct tty_port *port)
{
	_xd("+++\n");
	xmm_acm_port_shutdown(port);
}

static int xmm_acm_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	xmm_acm_t *xp = xmm_acm_port_xp(port);
	int rv;

	_xd("+++\n");
	if (!xp)
		return -ENODEV;
	rv = xmm_acm_submit_ct(xp, GFP_KERNEL);
	if (rv)
		return rv;
	rv = xmm_acm_submit_rd(xp, GFP_KERNEL);
	if (rv) {
		xmm_acm_kill_ct(xp);
		return rv;
	}
	atomic_set(&xp->wtr, 0); /* write transaction counter */
	xp->in_buff = 0;
	smp_mb();
	if (!wait_event_timeout(xp->open_wait, xp->net_ok, HZ))
		return -EBUSY;
	xmm_acm_port_dtr_rts(port, 1);
	return 0;
}

static const struct tty_port_operations xmm_acm_port_ops = {
	.carrier_raised	= xmm_acm_port_carrier_raised,
	.dtr_rts	= xmm_acm_port_dtr_rts,
	.drop		= xmm_acm_port_drop,
	.shutdown	= xmm_acm_port_shutdown,
	.activate	= xmm_acm_port_activate,
};

static int xmm_acm_probe(struct usb_interface *intf, const struct usb_device_id *ids)
{
	xmm_acm_t *xp;
	int rv;
	struct usb_device *udev = interface_to_usbdev(intf); /* usb device */
	struct usb_endpoint_descriptor *ctep, *rdep, *wrep; /* endpoints */

	_xd("+++\n");
/* check for valid device and iface */
//	if (udev->actconfig->desc.bNumInterfaces != 8)
//		return -ENODEV;
	switch (intf->cur_altsetting->desc.bInterfaceNumber) {
	case 0:
		break;
	case 1:
		_xe("data interface probe!!!\n");
		return -EINVAL;
		break;
	default:
		return -ENODEV;
		break;
	}
/* alloc device struct */
	xp = kzalloc(sizeof(xmm_acm_t), GFP_KERNEL);
	if (!xp)
		return -ENOMEM;
	init_waitqueue_head(&xp->wtr_wait);
	init_waitqueue_head(&xp->open_wait);
	spin_lock_init(&xp->clock);
	spin_lock_init(&xp->rlock);
	spin_lock_init(&xp->wlock);
	kref_init(&xp->kref);
	kref_get(&xp->kref); /* for data interface */
/* init device struct */
	xp->dev = udev;
	xp->cif = intf; /* if 0 */
	xp->dif = usb_ifnum_to_if(udev, intf->cur_altsetting->desc.bInterfaceNumber + 1); /* if 1 */
/* ctrl iface */
	rv = -EINVAL;
	if (xp->cif->cur_altsetting->desc.bNumEndpoints != 1)
		goto error0;
	ctep = &xp->cif->cur_altsetting->endpoint[0].desc;
	if (!usb_endpoint_is_int_in(ctep))
		goto error0;
/* rw iface */
	if (xp->dif->cur_altsetting->desc.bNumEndpoints != 2)
		goto error0;
	rdep = &xp->dif->cur_altsetting->endpoint[0].desc;
	wrep = &xp->dif->cur_altsetting->endpoint[1].desc;
	if (!usb_endpoint_is_bulk_in(rdep))
		goto error0;
	if (!usb_endpoint_is_bulk_out(wrep))
		goto error0;
/* pipes */
	xp->cpipe = usb_rcvintpipe(udev, ctep->bEndpointAddress);
	xp->rpipe = usb_rcvbulkpipe(udev, rdep->bEndpointAddress);
	xp->wpipe = usb_sndbulkpipe(udev, wrep->bEndpointAddress);
/* sizes (for dma free) */
	xp->csize = le16_to_cpu(ctep->wMaxPacketSize);
	xp->rsize = le16_to_cpu(rdep->wMaxPacketSize)*2;
	xp->wsize = le16_to_cpu(wrep->wMaxPacketSize)*2;
/* interrupt interval */
	xp->cint = ctep->bInterval;
/* alloc urbs */
	rv = 0;
	if (xmm_acm_alloc_ct(xp))
		rv++;
	if (xmm_acm_alloc_rd(xp))
		rv++;
	if (xmm_acm_alloc_wr(xp))
		rv++;
	if (rv) {
		rv = -ENOMEM;
		goto error1;
	}
/* init port and register device */
	tty_port_init(&xp->xport);
	xp->xport.ops = &xmm_acm_port_ops;
	XMM_TTY_REG(xp) = tty_register_device(xtty, 0, &udev->dev);
	if (IS_ERR(XMM_TTY_REG(xp))) {
		_xe("!tty_register_device()\n");
		rv = PTR_ERR(XMM_TTY_REG(xp));
		XMM_TTY_REG(xp) = NULL;
		goto error1;
	}
	dev_set_drvdata(XMM_TTY_REG(xp), xp);
	usb_get_dev(xp->dev);
	usb_get_dev(xp->dev);
	usb_get_intf(xp->cif);
	usb_get_intf(xp->dif);
	usb_set_intfdata(xp->cif, xp);
	usb_driver_claim_interface(&xmm_acm_driver, xp->dif, xp);
	return 0;
error1:
	xmm_acm_free_wr(xp);
	xmm_acm_free_rd(xp);
	xmm_acm_free_ct(xp);
error0:
	_xd("error0\n");
	kfree(xp);
	return rv;
}

static void xmm_acm_free(struct kref *kr)
{
	xmm_acm_t *xp = container_of(kr, xmm_acm_t, kref);
	_xd("+++\n");
	xmm_acm_free_wr(xp);
	xmm_acm_free_rd(xp);
	xmm_acm_free_ct(xp);
	kfree(xp);
}

static void xmm_acm_disconnect(struct usb_interface *intf)
{
	xmm_acm_t *xp = usb_get_intfdata(intf);
	struct tty_struct *tty;

	_xd("+++\n");
	if (xp->cif == intf) {
		xmm_acm_kill_ct(xp);
		goto put_free;
	} else if (xp->dif == intf) {
		xmm_acm_kill_wr(xp);
		xmm_acm_kill_rd(xp);
		goto put_free;
	} else
		return;
put_free:
	if (XMM_TTY_REG(xp)) { /* unregister tty device if registered */
		if (XMM_TTY_OPEN(xp)) { /* close device if opened - XMM_TTY_OPEN? */
			tty = tty_port_tty_get(&xp->xport);
			if (tty) {
				_xw("TTY HANGUP\n");
				tty_vhangup(tty);
				tty_kref_put(tty);
			}
		}
		tty_unregister_device(xtty, 0);
		XMM_TTY_REG(xp) = NULL;
	}
	usb_put_intf(intf);
	usb_put_dev(interface_to_usbdev(intf));
	kref_put(&xp->kref, xmm_acm_free);
	return;
}

static int xmm_acm_suspend(struct usb_interface *intf, pm_message_t message)
{
	xmm_acm_t *xp = usb_get_intfdata(intf);

	if (message.event & PM_EVENT_AUTO)
		return -EBUSY;
	_xd("SUSPEND TO L3\n");
	if (intf == xp->cif) {
		xmm_acm_kill_ct(xp);
		return 0;
	} else if (intf == xp->dif) {
		xmm_acm_kill_wr(xp);
		xmm_acm_kill_rd(xp);
		return 0;
	}
	return -ENODEV;
}

static int xmm_acm_resume(struct usb_interface *intf)
{
	xmm_acm_t *xp = usb_get_intfdata(intf);

	_xd("+++\n");
	if (intf == xp->cif)
		return xmm_acm_submit_ct(xp, GFP_NOIO);
	else if (intf == xp->dif)
		return xmm_acm_submit_rd(xp, GFP_NOIO);
	return -ENODEV;
}

static const struct tty_operations xmm_acm_tty_ops = {
	.open			= xmm_acm_tty_open,
	.close			= xmm_acm_tty_close,
	.hangup			= xmm_acm_tty_hangup,
	.write			= xmm_acm_tty_write,
	.write_room		= xmm_acm_tty_write_room,
	.chars_in_buffer	= xmm_acm_tty_chars_in_buffer,
//	.ioctl			= xmm_acm_tty_ioctl,
//	.set_termios		= xmm_acm_tty_set_termios,
//	.tiocmget		= xmm_acm_tty_tiocmget,
//	.tiocmset		= xmm_acm_tty_tiocmset,
};

static struct usb_driver xmm_acm_driver = {
	.name			= "xmm6260_acm",
	.probe			= xmm_acm_probe,
	.disconnect		= xmm_acm_disconnect,
	.suspend		= xmm_acm_suspend,
	.resume			= xmm_acm_resume,
	.reset_resume		= xmm_acm_resume,
	.id_table		= xmm6260_acm_ids,
	.supports_autosuspend	= 1,
};

int __init xmm6260_acm_init(void)
{
	int rv;

	_xd("+++\n");
	xtty = alloc_tty_driver(XMM_TTY_MINORS);
	if (!xtty) {
		_xe("!alloc_tty_driver()\n");
		return -ENOMEM;
	}
	xtty->owner = THIS_MODULE,
	xtty->driver_name = "xmm6260_tty",
	xtty->name = "ttyACM",
	xtty->major = XMM_TTY_MAJOR,
	xtty->minor_start = XMM_TTY_MSTART,
	xtty->type = TTY_DRIVER_TYPE_SERIAL,
	xtty->subtype = SERIAL_TYPE_NORMAL,
	xtty->flags = TTY_DRIVER_REAL_RAW|TTY_DRIVER_DYNAMIC_DEV;
	xtty->init_termios = tty_std_termios;
	xtty->init_termios.c_cflag |= CLOCAL;
	xtty->init_termios.c_lflag = 0;
	tty_set_operations(xtty, &xmm_acm_tty_ops);
	rv = tty_register_driver(xtty);
	if (rv) {
		_xe("!tty_register_driver()\n");
		goto exit1;
	}
	rv = usb_register(&xmm_acm_driver);
	if (rv) {
		_xe("!usb_register()\n");
		goto exit2;
	}
	return 0;
exit2:
	tty_unregister_driver(xtty);
exit1:
	put_tty_driver(xtty);
	return rv;
}

void __exit xmm6260_acm_exit(void)
{
	_xd("+++\n");
	usb_deregister(&xmm_acm_driver);
	tty_unregister_driver(xtty);
	put_tty_driver(xtty);
	return;
}
