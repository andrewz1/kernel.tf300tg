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

#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "xmm6260_net.h"

static const struct usb_device_id xmm6260_net_ids[] = {
	{ USB_DEVICE(XMM_MAIN_VID, XMM_MAIN_PID) },
	{ }
};
MODULE_DEVICE_TABLE(usb, xmm6260_net_ids);

static struct usb_driver xmm_net_driver;

/*
static __always_inline void xmm_net_pm_mark(xmm_net_t *xp)
{
	if (usb_autopm_get_interface(xp->cif) == 0)
		usb_autopm_put_interface_async(xp->cif);
	if (usb_autopm_get_interface(xp->dif) == 0)
		usb_autopm_put_interface_async(xp->dif);
	usb_mark_last_busy(xp->dev);
}

static __always_inline void xmm_net_pm_mark_async(xmm_net_t *xp)
{
	if (usb_autopm_get_interface_async(xp->cif) == 0)
		usb_autopm_put_interface_async(xp->cif);
	if (usb_autopm_get_interface_async(xp->dif) == 0)
		usb_autopm_put_interface_async(xp->dif);
	usb_mark_last_busy(xp->dev);
}
*/

static __always_inline int xmm_net_state(xmm_net_t *xp)
{
	if (!xp)
		return 0;
	return (xp->net_ok && (xp->ctrlin & XMM_NET_CAR));
}

static __always_inline void xmm_net_carrier(xmm_net_t *xp)
{
	if (!xp || !xp->nif)
		return;
	if (xmm_net_state(xp))
		netif_carrier_on(xp->nif);
	else
		netif_carrier_off(xp->nif);
}

static void xmm_net_netif_rx(xmm_net_t *xp, int i)
{
	xmm_priv_t *pp;
	struct sk_buff *skb;

	if (!XMM_NET_OK(xp)) /* add dropped? */
		return;
	pp = netdev_priv(xp->nif);
/* from this point we can safe use stats */
	skb = netdev_alloc_skb(xp->nif, xp->rb[i].urb->actual_length + ETH_HLEN);
	if (!skb) {
		pp->stats.rx_errors++;
		pp->stats.rx_dropped++;
		return;
	}
	skb_copy_to_linear_data(skb, &pp->to_dev, ETH_HLEN);
	skb_copy_to_linear_data_offset(skb, ETH_HLEN,
		xp->rb[i].urb->transfer_buffer, xp->rb[i].urb->actual_length);
	__skb_put(skb, xp->rb[i].urb->actual_length + ETH_HLEN);
	skb->protocol = eth_type_trans(skb, xp->nif);
	if (netif_rx(skb) == NET_RX_SUCCESS) {
		pp->stats.rx_packets++;
		pp->stats.rx_bytes += xp->rb[i].urb->actual_length;
	} else {
		pp->stats.rx_errors++;
		pp->stats.rx_dropped++;
	}
	return;
}

/* write URB ops */

static int xmm_net_submit_wr_urb(xmm_net_t *xp, int i, gfp_t mf)
{
	unsigned long flags;
	int rv;

	if (!XMM_URB_OK(xp->wb[i].urb))
		return -EINVAL;
	atomic_inc(&xp->wtr);
	rv = usb_submit_urb(xp->wb[i].urb, mf);
	spin_lock_irqsave(&xp->wlock, flags);
	if (rv)
		__clear_bit(i, xp->wb_used);
	spin_unlock_irqrestore(&xp->wlock, flags);
	if (rv) {
		atomic_dec(&xp->wtr);
		smp_mb();
		if (waitqueue_active(&xp->wtr_wait))
			wake_up(&xp->wtr_wait);
	}
	return rv;
}

static void xmm_net_wr_cb(struct urb *urb)
{
	xmm_urb_t *wb = urb->context;
	unsigned long flags;

//	xmm_net_pm_mark_async(wb->xp);
	spin_lock_irqsave(&wb->xp->wlock, flags);
	__clear_bit(wb->i, wb->xp->wb_used);
	spin_unlock_irqrestore(&wb->xp->wlock, flags);
	atomic_dec(&wb->xp->wtr);
	smp_mb();
	if (waitqueue_active(&wb->xp->wtr_wait))
		wake_up(&wb->xp->wtr_wait);
	return;
}

static void xmm_net_kill_wr(xmm_net_t *xp)
{
	unsigned long flags;
	int i;

	if (!wait_event_timeout(xp->wtr_wait, !atomic_read(&xp->wtr), HZ))
		_xw("wtr timeout!\n");
	for (i = 0; i < XMM_NET_WR; i++) {
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

static void xmm_net_free_wr(xmm_net_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_NET_WR; i++) {
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

static int xmm_net_alloc_wr(xmm_net_t *xp)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&xp->wlock, flags);
	bitmap_fill(xp->wb_used, XMM_NET_WR);
	spin_unlock_irqrestore(&xp->wlock, flags);
	for (i = 0; i < XMM_NET_WR; i++) {
		xp->wb[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!xp->wb[i].urb)
			goto error;
		xp->wb[i].buf = usb_alloc_coherent(xp->dev, xp->wsize, GFP_KERNEL, &xp->wb[i].dma);
		if (!xp->wb[i].buf)
			goto error;
		usb_fill_bulk_urb(xp->wb[i].urb, xp->dev, xp->wpipe, xp->wb[i].buf, 0, xmm_net_wr_cb, &xp->wb[i]);
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
	xmm_net_free_wr(xp);
	return -ENOMEM;
}

/* read URB ops */

static int xmm_net_submit_rd_urb(xmm_net_t *xp, int i, gfp_t mf, bool reuse)
{
	unsigned long flags;
	int rv;

	if (!XMM_URB_OK(xp->rb[i].urb))
		return -EINVAL;
//	xmm_net_pm_mark_async(xp);
	if (!reuse) { /* new urb */
		spin_lock_irqsave(&xp->rlock, flags);
		rv = __test_and_set_bit(i, xp->rb_used);
		spin_unlock_irqrestore(&xp->rlock, flags);
		if (rv) /* bit not changed - urb is busy */
			return -EINVAL;
	}
	rv = usb_submit_urb(xp->rb[i].urb, mf);
	if (rv) {
		spin_lock_irqsave(&xp->rlock, flags);
		__clear_bit(i, xp->rb_used);
		spin_unlock_irqrestore(&xp->rlock, flags);
	}
	return rv;
}

static void xmm_net_rd_cb(struct urb *urb)
{
	xmm_urb_t *rb = urb->context;
	unsigned long flags;
	int rv;

////	xmm_net_pm_mark_async(rb->xp);
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
	if (urb->actual_length) /* here we submit packet to kernel */
		xmm_net_netif_rx(rb->xp, rb->i);
resubmit: /* with don't touch used bits in this case! */
	rv = xmm_net_submit_rd_urb(rb->xp, rb->i, GFP_ATOMIC, true);
	if (rv) /* used bit cleared in xmm_acm_submit_read_urb */
		_xe("Resubmit read URB failed, if:%d, i:%d, rv:%d\n", rb->xp->id, rb->i, rv);
	return;
shutdown: /* mark as free */
//	_xd("Shutdown read URB, index: %d, err: %d\n", rb->i, urb->status);
	spin_lock_irqsave(&rb->xp->rlock, flags);
	__clear_bit(rb->i, rb->xp->rb_used);
	spin_unlock_irqrestore(&rb->xp->rlock, flags);
	return;
}

static void xmm_net_kill_rd(xmm_net_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_NET_RD; i++) {
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
	return;
}

static int xmm_net_submit_rd(xmm_net_t *xp, gfp_t mf)
{
	int rv, i;

	for (i = 0; i < XMM_NET_RD; i++) {
		rv = xmm_net_submit_rd_urb(xp, i, mf, false);
		if (rv)
			break;
	}
	if (rv)
		xmm_net_kill_rd(xp);
	return rv;
}

static void xmm_net_free_rd(xmm_net_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_NET_RD; i++) {
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

static int xmm_net_alloc_rd(xmm_net_t *xp)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&xp->rlock, flags);
	bitmap_fill(xp->rb_used, XMM_NET_RD);
	spin_unlock_irqrestore(&xp->rlock, flags);
	for (i = 0; i < XMM_NET_RD; i++) {
		xp->rb[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!xp->rb[i].urb)
			goto error;
		xp->rb[i].buf = usb_alloc_coherent(xp->dev, xp->rsize, GFP_KERNEL, &xp->rb[i].dma);
		if (!xp->rb[i].buf)
			goto error;
		usb_fill_bulk_urb(xp->rb[i].urb, xp->dev, xp->rpipe, xp->rb[i].buf, xp->rsize, xmm_net_rd_cb, &xp->rb[i]);
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
	xmm_net_free_rd(xp);
	return -ENOMEM;
}

/* ctrl URB ops */

static int xmm_net_submit_ct_urb(xmm_net_t *xp, int i, gfp_t mf, bool reuse)
{
	unsigned long flags;
	int rv;

	if (!XMM_URB_OK(xp->ct[i].urb))
		return -EINVAL;
//	xmm_net_pm_mark_async(xp);
	if (!reuse) { /* new urb */
		spin_lock_irqsave(&xp->clock, flags);
		rv = __test_and_set_bit(i, xp->ct_used);
		spin_unlock_irqrestore(&xp->clock, flags);
		if (rv) /* bit not changed - urb is locked */
			return -EINVAL;
	}
	rv = usb_submit_urb(xp->ct[i].urb, mf);
	if (rv) {
		spin_lock_irqsave(&xp->clock, flags);
		__clear_bit(i, xp->ct_used);
		spin_unlock_irqrestore(&xp->clock, flags);
	}
	return rv;
}

static void xmm_net_ct_cb(struct urb *urb)
{
	xmm_urb_t *ct = urb->context;
	struct usb_cdc_notification *n;
	unsigned char *b;
	unsigned long flags;
	int rv;
	u16 net_ok, ctrlin;

////	xmm_net_pm_mark_async(ct->xp);
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
				smp_mb();
				xmm_net_carrier(ct->xp);
				_xd("USB_CDC_NOTIFY_NETWORK_CONNECTION: if:%d state:%d\n", ct->xp->id, net_ok); /* eth link ok - carrier */
			}
			break;
		case USB_CDC_NOTIFY_SERIAL_STATE:
			b = urb->transfer_buffer + sizeof(struct usb_cdc_notification);
			ctrlin = le16_to_cpup((__le16 *)b);
			if (ct->xp->ctrlin != ctrlin) {
				ct->xp->ctrlin = ctrlin;
				smp_mb();
				xmm_net_carrier(ct->xp);
				_xd("USB_CDC_NOTIFY_SERIAL_STATE: if:%d state:0x%04x\n", ct->xp->id, ctrlin & XMM_NET_CAR); /* ip link up - modem connected */
//				if (ctrlin & XMM_NET_CAR)
//					netif_carrier_on(ct->xp->nif);
//				else
//					netif_carrier_off(ct->xp->nif);
			}
			break;
		default:
			_xd("bNotificationType: %d\n", n->bNotificationType);
			break;
		}
	}
/* resubmit urb */
resubmit:
	rv = xmm_net_submit_ct_urb(ct->xp, ct->i, GFP_ATOMIC, true);
	if (rv)
		_xe("Resubmit ctrl URB failed, if:%d, rv:%d\n", ct->xp->id, rv);
	return;
shutdown:
//	_xd("Shutdown ctrl URB, err: %d\n", urb->status);
	spin_lock_irqsave(&ct->xp->clock, flags);
	__clear_bit(ct->i, ct->xp->ct_used);
	spin_unlock_irqrestore(&ct->xp->clock, flags);
	return;
}

static void xmm_net_kill_ct(xmm_net_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_NET_CT; i++) {
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
	return;
}

static int xmm_net_submit_ct(xmm_net_t *xp, gfp_t mf)
{
	int rv, i;

	for (i = 0; i < XMM_NET_CT; i++) {
		rv = xmm_net_submit_ct_urb(xp, i, mf, false);
		if (rv)
			break;
	}
	if (rv)
		xmm_net_kill_ct(xp);
	return rv;
}

static void xmm_net_free_ct(xmm_net_t *xp)
{
	unsigned long flags;
	int i;

	for (i = 0; i < XMM_NET_CT; i++) {
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

static int xmm_net_alloc_ct(xmm_net_t *xp)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&xp->clock, flags);
	bitmap_fill(xp->ct_used, XMM_NET_CT);
	spin_unlock_irqrestore(&xp->clock, flags);
	for (i = 0; i < XMM_NET_CT; i++) {
		xp->ct[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!xp->ct[i].urb)
			goto error;
		xp->ct[i].buf = usb_alloc_coherent(xp->dev, xp->csize, GFP_KERNEL, &xp->ct[i].dma);
		if (!xp->ct[i].buf)
			goto error;
		usb_fill_int_urb(xp->ct[i].urb, xp->dev, xp->cpipe, xp->ct[i].buf, xp->csize,
			xmm_net_ct_cb, &xp->ct[i], xp->cint);
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
	xmm_net_free_ct(xp);
	return -ENOMEM;
}

/* end of urb tools */

static void xmm_net_send_work(struct work_struct *w)
{
	xmm_net_t *xp = container_of(w, xmm_net_t, send_w);
	xmm_priv_t *pp = netdev_priv(xp->nif);
	xmm_urb_t *wb;
	struct urb *u;
	int rv;

//	xmm_net_pm_mark(xp); /* sync call!!! */
	while ((u = usb_get_from_anchor(&xp->sa))) {
		usb_put_urb(u);
		usb_unanchor_urb(u);
		wb = u->context;
		rv = xmm_net_submit_wr_urb(xp, wb->i, GFP_NOIO);
		if (rv) {
			pp->stats.tx_dropped++;
			pp->stats.tx_errors++;
		} else {
			pp->stats.tx_packets++;
			pp->stats.tx_bytes += u->transfer_buffer_length;
		}
	}
	return;
}

/* net dev ops */

static int xmm_net_ndo_init(struct net_device *dev)
{
	xmm_priv_t *pp = netdev_priv(dev);

	_xd("+++\n");
	pp->xp->net_ok = 0;
	pp->xp->ctrlin = 0;
	pp->xp->ctrlout = 0;
	xmm_net_carrier(pp->xp);
//	_xd("LS_START:%d, LS_PRESENT:%d, LS_NOCARRIER:%d\n",
//		test_bit(__LINK_STATE_START, &dev->state),
//		test_bit(__LINK_STATE_PRESENT, &dev->state),
//		test_bit(__LINK_STATE_NOCARRIER, &dev->state));
//	netif_carrier_on(dev);
//	clear_bit(__LINK_STATE_NOCARRIER, &dev->state);
//	set_bit(__LINK_STATE_PRESENT, &dev->state);
//	if (dev)
//		netif_carrier_off(dev);
//	_xd("LS_START:%d, LS_PRESENT:%d, LS_NOCARRIER:%d\n",
//		test_bit(__LINK_STATE_START, &dev->state),
//		test_bit(__LINK_STATE_PRESENT, &dev->state),
//		test_bit(__LINK_STATE_NOCARRIER, &dev->state));
	return 0;
}

static void xmm_net_ndo_uninit(struct net_device *dev)
{
	xmm_priv_t *pp = netdev_priv(dev);

	_xd("+++\n");
	pp->xp->net_ok = 0;
	pp->xp->ctrlin = 0;
	pp->xp->ctrlout = 0;
	xmm_net_carrier(pp->xp);
//	_xd("LS_START:%d, LS_PRESENT:%d, LS_NOCARRIER:%d\n",
//		test_bit(__LINK_STATE_START, &dev->state),
//		test_bit(__LINK_STATE_PRESENT, &dev->state),
//		test_bit(__LINK_STATE_NOCARRIER, &dev->state));
//	if (dev)
//		netif_carrier_off(dev);
//	set_bit(__LINK_STATE_NOCARRIER, &dev->state);
//	clear_bit(__LINK_STATE_PRESENT, &dev->state);
//	return;
}

static int xmm_net_ndo_open(struct net_device *dev)
{
	_xd("+++\n");
//	_xd("LS_START:%d, LS_PRESENT:%d, LS_NOCARRIER:%d\n",
//		test_bit(__LINK_STATE_START, &dev->state),
//		test_bit(__LINK_STATE_PRESENT, &dev->state),
//		test_bit(__LINK_STATE_NOCARRIER, &dev->state));
	netif_start_queue(dev);
//	_xd("LS_START:%d, LS_PRESENT:%d, LS_NOCARRIER:%d\n",
//		test_bit(__LINK_STATE_START, &dev->state),
//		test_bit(__LINK_STATE_PRESENT, &dev->state),
//		test_bit(__LINK_STATE_NOCARRIER, &dev->state));
	return 0;
}

static int xmm_net_ndo_stop(struct net_device *dev)
{
	_xd("+++\n");
	netif_stop_queue(dev);
	return 0;
}

static struct net_device_stats *xmm_net_ndo_get_stats(struct net_device *dev)
{
	xmm_priv_t *pp = netdev_priv(dev);

//	_xd("+++\n");
//	_xd("LS_START:%d, LS_PRESENT:%d, LS_NOCARRIER:%d\n",
//		test_bit(__LINK_STATE_START, &dev->state),
//		test_bit(__LINK_STATE_PRESENT, &dev->state),
//		test_bit(__LINK_STATE_NOCARRIER, &dev->state));
	return &pp->stats;
}

static void xmm_net_ndo_tx_timeout(struct net_device *dev)
{
	xmm_priv_t *pp = netdev_priv(dev);

	_xd("+++\n");
	pp->stats.tx_dropped++;
	pp->stats.tx_errors++;
	netif_wake_queue(dev);
	return;
}

/*
static void xmm_net_ndo_set_rx_mode(struct net_device *dev)
{
	_xd("+++\n");
	return;
}
*/

static netdev_tx_t xmm_net_ndo_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	xmm_priv_t *pp;
	xmm_net_t *xp;
	unsigned long flags;
	int i;

	pp = netdev_priv(dev);
	xp = pp->xp;
/* from this point we can safe use stats */
//	if (!xp->net_ok) {
//		_xd("packet delay!\n");
//		return NETDEV_TX_BUSY; /* try to keep skb */
//	}
	if (!netif_running(dev) || !XMM_USB_OK(xp))
		goto drop;
	dev->trans_start = jiffies; /* save the timestamp */
	spin_lock_irqsave(&xp->wlock, flags);
	i = find_first_zero_bit(xp->wb_used, XMM_NET_WR);
	if (i < XMM_NET_WR) {
		__set_bit(i, xp->wb_used);
		spin_unlock_irqrestore(&xp->wlock, flags);
	} else { /* no free urbs - this is bug */
		spin_unlock_irqrestore(&xp->wlock, flags);
		goto drop;
	}
	memcpy(xp->wb[i].urb->transfer_buffer, skb->data + ETH_HLEN, skb->len - ETH_HLEN);
	xp->wb[i].urb->transfer_buffer_length = skb->len - ETH_HLEN;
	usb_anchor_urb(xp->wb[i].urb, &xp->sa);
	queue_work(xp->send_qw, &xp->send_w);
	dev_kfree_skb_any(skb); /* stats updated in wq */
	return NETDEV_TX_OK;
drop:
	_xd("packet drop!\n");
	dev_kfree_skb_any(skb);
	pp->stats.tx_dropped++;
	pp->stats.tx_errors++;
	return NETDEV_TX_OK;
}

static struct net_device_ops xmm_net_netdev_ops = {
	.ndo_init		= xmm_net_ndo_init,
	.ndo_uninit		= xmm_net_ndo_uninit,
	.ndo_open		= xmm_net_ndo_open,
	.ndo_stop		= xmm_net_ndo_stop,
	.ndo_get_stats		= xmm_net_ndo_get_stats,
	.ndo_tx_timeout		= xmm_net_ndo_tx_timeout,
//	.ndo_set_rx_mode	= xmm_net_ndo_set_rx_mode,
	.ndo_start_xmit		= xmm_net_ndo_start_xmit,
};

static int xmm_net_eth_header(struct sk_buff *skb, struct net_device *dev, unsigned short type,
	const void *daddr, const void *saddr, unsigned len)
{
	xmm_priv_t *pp;
	void *hdr;

	hdr = skb_push(skb, ETH_HLEN);
	pp = netdev_priv(dev);
	memcpy(hdr, &pp->from_dev, ETH_HLEN);
	return ETH_HLEN;
}

static struct header_ops xmm_net_header_ops = {
	.create		= xmm_net_eth_header,
};

static void xmm_net_setup(struct net_device *dev)
{
	xmm_priv_t *pp = netdev_priv(dev);

	_xd("+++\n");
	dev->type		= ARPHRD_ETHER;
	dev->hard_header_len	= ETH_HLEN;
	dev->mtu		= ETH_DATA_LEN;
	dev->addr_len		= ETH_ALEN;
	dev->tx_queue_len	= 100;	/* Ethernet wants good queues */
	dev->flags		= IFF_NOARP;
//	dev->priv_flags		|= IFF_TX_SKB_SHARING;
	dev->watchdog_timeo	= XMM_NET_TMO;
//	dev->features		|= NETIF_F_HW_CSUM;
	dev->netdev_ops		= &xmm_net_netdev_ops;
	dev->header_ops		= &xmm_net_header_ops;
//	memcpy(dev->dev_addr, card_mac, ETH_ALEN);
//	memset(dev->broadcast, 0xFF, ETH_ALEN);
	memset(pp, 0, sizeof(xmm_priv_t));
}

static int xmm_net_alloc_netdev(xmm_net_t *xp)
{
	xmm_priv_t *pp;
	char dev_name[IFNAMSIZ];
	unsigned char dev_addr[ETH_ALEN] = "\0rmnet";
	int rv;

	_xd("+++\n");
	mutex_lock(&xp->nm);
	if (xp->nif) {
		mutex_unlock(&xp->nm);
		return -EBUSY;
	}
	memset(dev_name, 0, IFNAMSIZ);
	sprintf(dev_name, "rmnet%d", xp->id);
	xp->nif = alloc_netdev(sizeof(xmm_priv_t), dev_name, xmm_net_setup);
	if (!xp->nif) {
		mutex_unlock(&xp->nm);
		return -ENOMEM;
	}
/* set card MAC address */
	dev_addr[0] = (xp->id * 2);
	memcpy(xp->nif->dev_addr, dev_addr, ETH_ALEN);
	memset(xp->nif->broadcast, 0xff, ETH_ALEN);
	pp = netdev_priv(xp->nif);
	pp->xp = xp;
	pp->from_dev.h_proto = htons(ETH_P_IP);
	memcpy(&pp->from_dev.h_source, dev_addr, ETH_ALEN);
	pp->to_dev.h_proto = htons(ETH_P_IP);
	memcpy(&pp->to_dev.h_dest, dev_addr, ETH_ALEN);
/* register dev */
	rv = register_netdev(xp->nif);
	if (rv) {
		free_netdev(xp->nif);
		xp->nif = NULL;
	}
	mutex_unlock(&xp->nm);
	return rv;
}

static void xmm_net_free_netdev(xmm_net_t *xp)
{
	_xd("+++\n");
	mutex_lock(&xp->nm);
	if (!xp->nif) {
		mutex_unlock(&xp->nm);
		return;
	}
	netif_stop_queue(xp->nif);
	cancel_work_sync(&xp->send_w);
	unregister_netdev(xp->nif);
	free_netdev(xp->nif);
	xp->nif = NULL;
	mutex_unlock(&xp->nm);
	return;
}

static int xmm_net_probe(struct usb_interface *intf, const struct usb_device_id *ids)
{
	xmm_net_t *xp;
	int rv, id;
	struct usb_device *udev = interface_to_usbdev(intf); /* usb device */
	struct usb_endpoint_descriptor *ctep, *rdep, *wrep; /* endpoints */
	char tmp_buff[256];

	_xd("+++\n");
/* check for valid device and iface */
	if (udev->actconfig->desc.bNumInterfaces != 8)
		return -ENODEV; /* skip boot mode */
	switch (intf->cur_altsetting->desc.bInterfaceNumber) {
	case 2:
		id = 0; /* netif index */
		break;
	case 3:
		_xe("data interface probe!!!\n");
		return -EINVAL;
		break;
//	case 4:
//		id = 1;
//		break;
//	case 5:
//		_xe("data interface probe!!!\n");
//		return -EINVAL;
//		break;
	default:
		return -ENODEV;
	}
/* init device struct */
	xp = kzalloc(sizeof(xmm_net_t), GFP_KERNEL);
	if (!xp)
		return -ENOMEM;
	init_waitqueue_head(&xp->wtr_wait);
	spin_lock_init(&xp->clock);
	spin_lock_init(&xp->rlock);
	spin_lock_init(&xp->wlock);
	init_usb_anchor(&xp->sa);
	mutex_init(&xp->nm);
	kref_init(&xp->kref);
	kref_get(&xp->kref); /* for data interface */
/* init device struct */
	xp->id = id;
	xp->dev = udev;
	xp->cif = intf;
	xp->dif = usb_ifnum_to_if(udev, intf->cur_altsetting->desc.bInterfaceNumber + 1);
/* ctrl iface */
	rv = -EINVAL;
	if (xp->cif->cur_altsetting->desc.bNumEndpoints != 1)
		goto error0;
	ctep = &xp->cif->cur_altsetting->endpoint[0].desc;
	if (!usb_endpoint_is_int_in(ctep))
		goto error0;
/* data iface */
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
	xp->rsize = le16_to_cpu(rdep->wMaxPacketSize)*4;
	xp->wsize = le16_to_cpu(wrep->wMaxPacketSize)*4;
/* interrupt interval */
	xp->cint = ctep->bInterval;
/* alloc urbs */
	rv = 0;
	if (xmm_net_alloc_ct(xp))
		rv++;
	if (xmm_net_alloc_rd(xp))
		rv++;
	if (xmm_net_alloc_wr(xp))
		rv++;
	if (rv) {
		rv = -ENOMEM;
		goto error1;
	}
/* alloc send wq */
	memset(tmp_buff, 0, sizeof(tmp_buff));
	sprintf(tmp_buff, "xmm6260_rmnet%d_tx", id);
	xp->send_qw = create_singlethread_workqueue(tmp_buff);
	if (!xp->send_qw) {
		_xe("!create_workqueue()\n");
		rv = -ENOMEM;
		goto error1;
	}
	INIT_WORK(&xp->send_w, xmm_net_send_work);
/* start network here */
	rv = xmm_net_alloc_netdev(xp);
	if (rv)
		goto error2;
	rv = xmm_net_submit_ct(xp, GFP_KERNEL);
	if (rv)
		goto error3;
	rv = xmm_net_submit_rd(xp, GFP_KERNEL);
	if (rv)
		goto error4;
	usb_get_dev(xp->dev);
	usb_get_dev(xp->dev); /* for data interface */
	usb_get_intf(xp->cif);
	usb_get_intf(xp->dif);
	usb_set_intfdata(xp->cif, xp);
	usb_driver_claim_interface(&xmm_net_driver, xp->dif, xp);
	return 0;
error4:
	xmm_net_kill_ct(xp);
error3:
	xmm_net_free_netdev(xp);
error2:
	destroy_workqueue(xp->send_qw);
error1:
	xmm_net_free_wr(xp);
	xmm_net_free_rd(xp);
	xmm_net_free_ct(xp);
error0:
	kfree(xp);
	return rv;
}

static void xmm_net_free(struct kref *kr)
{
	xmm_net_t *xp = container_of(kr, xmm_net_t, kref);

	_xd("+++\n");
	xmm_net_free_netdev(xp);
	destroy_workqueue(xp->send_qw);
	xmm_net_free_wr(xp);
	xmm_net_free_rd(xp);
	xmm_net_free_ct(xp);
	kfree(xp);
}

static void xmm_net_disconnect(struct usb_interface *intf)
{
	xmm_net_t *xp = usb_get_intfdata(intf);

	_xd("+++\n");
	if (xp->cif == intf) {
		xmm_net_kill_ct(xp);
		goto put_free;
	} else if (xp->dif == intf) {
		xmm_net_kill_wr(xp);
		xmm_net_kill_rd(xp);
		goto put_free;
	} else
		return;
put_free:
	usb_put_intf(intf);
	usb_put_dev(interface_to_usbdev(intf));
	kref_put(&xp->kref, xmm_net_free);
	return;
}

static int xmm_net_suspend(struct usb_interface *intf, pm_message_t message)
{
	xmm_net_t *xp = usb_get_intfdata(intf);

	if (message.event & PM_EVENT_AUTO)
		return -EBUSY; /* forbid suspend to L2 because resume will be immediatly after */
	_xd("SUSPEND TO L3\n");
	if (intf == xp->cif) {
		xmm_net_kill_ct(xp);
		return 0;
	} else if (intf == xp->dif) {
		xmm_net_kill_wr(xp);
		xmm_net_kill_rd(xp);
		return 0;
	}
	return -ENODEV;
}

static int xmm_net_resume(struct usb_interface *intf)
{
	xmm_net_t *xp = usb_get_intfdata(intf);

	_xd("+++\n");
	if (intf == xp->cif)
		return xmm_net_submit_ct(xp, GFP_NOIO);
	else if (intf == xp->dif)
		return xmm_net_submit_rd(xp, GFP_NOIO);
	return -ENODEV;
}

static struct usb_driver xmm_net_driver = {
	.name			= "xmm6260_net",
	.probe			= xmm_net_probe,
	.disconnect		= xmm_net_disconnect,
	.suspend		= xmm_net_suspend,
	.resume			= xmm_net_resume,
	.reset_resume		= xmm_net_resume,
	.id_table		= xmm6260_net_ids,
	.supports_autosuspend	= 1
};

int __init xmm6260_net_init(void)
{
	_xd("+++\n");
	return usb_register(&xmm_net_driver);
}

void __exit xmm6260_net_exit(void)
{
	_xd("+++\n");
	usb_deregister(&xmm_net_driver);
}
