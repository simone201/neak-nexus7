/*
<<<<<<< HEAD
 * ASIX AX88179 based USB 3.0 Ethernet Devices
 * Copyright (C) 2003-2005 David Hollis <dhollis@davehollis.com>
 * Copyright (C) 2005 Phil Chang <pchang23@sbcglobal.net>
=======
 * ASIX AX8817X based USB 2.0 Ethernet Devices
 * Copyright (C) 2003-2006 David Hollis <dhollis@davehollis.com>
 * Copyright (C) 2005 Phil Chang <pchang23@sbcglobal.net>
 * Copyright (C) 2006 James Painter <jamie.painter@iname.com>
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
 * Copyright (c) 2002-2003 TiVo Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

<<<<<<< HEAD
/* debug messages, extra info */
/* #define	DEBUG */

#include <linux/version.h>
/*#include <linux/config.h>*/
#ifdef	CONFIG_USB_DEBUG
#define DEBUG
#endif
#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/sched.h>
=======
// #define	DEBUG			// error path messages, extra info
// #define	VERBOSE			// more; success messages

#include <linux/module.h>
#include <linux/kmod.h>
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
<<<<<<< HEAD
#include <linux/if_vlan.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)
#include <linux/usb/usbnet.h>
#else
#include <../drivers/usb/net/usbnet.h>
#endif

#include "asix.h"

#define DRV_VERSION	"1.5.0"

static char version[] =
KERN_INFO "ASIX USB Ethernet Adapter:v" DRV_VERSION
	" " __TIME__ " " __DATE__ "\n"
"		http://www.asix.com.tw\n";

static int msg_enable;
module_param(msg_enable, int, 0);
MODULE_PARM_DESC(msg_enable, "usbnet msg_enable");

static int bsize;
module_param(bsize, int, 0);
MODULE_PARM_DESC(bsize, "RX Bulk IN Queue Size");

static int ifg;
module_param(ifg, int, 0);
MODULE_PARM_DESC(ifg, "RX Bulk IN Inter Frame Gap");


/* ASIX AX88179/178A based USB 3.0/2.0 Gigabit Ethernet Devices */

static int __ax88179_read_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			      u16 size, void *data, int in_pm)
{
	int ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	int (*fn)(struct usbnet *, u8, u8, u16, u16, void *, u16);

	BUG_ON(!dev);

	if (!in_pm)
		fn = usbnet_read_cmd;
	else
		fn = usbnet_read_cmd_nopm;

	ret = fn(dev, cmd, USB_DIR_IN | USB_TYPE_VENDOR |
		 USB_RECIP_DEVICE, value, index, data, size);

	if (unlikely(ret < 0))
		netdev_warn(dev->net, "Failed to read reg index 0x%04x: %d\n",
			    index, ret);
#else
	ret = usb_control_msg(
=======
#include <linux/usb/usbnet.h>
#include <linux/slab.h>

#define DRIVER_VERSION "14-Jun-2006"
static const char driver_name [] = "asix";

/* ASIX AX8817X based USB 2.0 Ethernet Devices */

#define AX_CMD_SET_SW_MII		0x06
#define AX_CMD_READ_MII_REG		0x07
#define AX_CMD_WRITE_MII_REG		0x08
#define AX_CMD_SET_HW_MII		0x0a
#define AX_CMD_READ_EEPROM		0x0b
#define AX_CMD_WRITE_EEPROM		0x0c
#define AX_CMD_WRITE_ENABLE		0x0d
#define AX_CMD_WRITE_DISABLE		0x0e
#define AX_CMD_READ_RX_CTL		0x0f
#define AX_CMD_WRITE_RX_CTL		0x10
#define AX_CMD_READ_IPG012		0x11
#define AX_CMD_WRITE_IPG0		0x12
#define AX_CMD_WRITE_IPG1		0x13
#define AX_CMD_READ_NODE_ID		0x13
#define AX_CMD_WRITE_NODE_ID		0x14
#define AX_CMD_WRITE_IPG2		0x14
#define AX_CMD_WRITE_MULTI_FILTER	0x16
#define AX88172_CMD_READ_NODE_ID	0x17
#define AX_CMD_READ_PHY_ID		0x19
#define AX_CMD_READ_MEDIUM_STATUS	0x1a
#define AX_CMD_WRITE_MEDIUM_MODE	0x1b
#define AX_CMD_READ_MONITOR_MODE	0x1c
#define AX_CMD_WRITE_MONITOR_MODE	0x1d
#define AX_CMD_READ_GPIOS		0x1e
#define AX_CMD_WRITE_GPIOS		0x1f
#define AX_CMD_SW_RESET			0x20
#define AX_CMD_SW_PHY_STATUS		0x21
#define AX_CMD_SW_PHY_SELECT		0x22

#define AX_MONITOR_MODE			0x01
#define AX_MONITOR_LINK			0x02
#define AX_MONITOR_MAGIC		0x04
#define AX_MONITOR_HSFS			0x10

/* AX88172 Medium Status Register values */
#define AX88172_MEDIUM_FD		0x02
#define AX88172_MEDIUM_TX		0x04
#define AX88172_MEDIUM_FC		0x10
#define AX88172_MEDIUM_DEFAULT \
		( AX88172_MEDIUM_FD | AX88172_MEDIUM_TX | AX88172_MEDIUM_FC )

#define AX_MCAST_FILTER_SIZE		8
#define AX_MAX_MCAST			64

#define AX_SWRESET_CLEAR		0x00
#define AX_SWRESET_RR			0x01
#define AX_SWRESET_RT			0x02
#define AX_SWRESET_PRTE			0x04
#define AX_SWRESET_PRL			0x08
#define AX_SWRESET_BZ			0x10
#define AX_SWRESET_IPRL			0x20
#define AX_SWRESET_IPPD			0x40

#define AX88772_IPG0_DEFAULT		0x15
#define AX88772_IPG1_DEFAULT		0x0c
#define AX88772_IPG2_DEFAULT		0x12

/* AX88772 & AX88178 Medium Mode Register */
#define AX_MEDIUM_PF		0x0080
#define AX_MEDIUM_JFE		0x0040
#define AX_MEDIUM_TFC		0x0020
#define AX_MEDIUM_RFC		0x0010
#define AX_MEDIUM_ENCK		0x0008
#define AX_MEDIUM_AC		0x0004
#define AX_MEDIUM_FD		0x0002
#define AX_MEDIUM_GM		0x0001
#define AX_MEDIUM_SM		0x1000
#define AX_MEDIUM_SBP		0x0800
#define AX_MEDIUM_PS		0x0200
#define AX_MEDIUM_RE		0x0100

#define AX88178_MEDIUM_DEFAULT	\
	(AX_MEDIUM_PS | AX_MEDIUM_FD | AX_MEDIUM_AC | \
	 AX_MEDIUM_RFC | AX_MEDIUM_TFC | AX_MEDIUM_JFE | \
	 AX_MEDIUM_RE )

#define AX88772_MEDIUM_DEFAULT	\
	(AX_MEDIUM_FD | AX_MEDIUM_RFC | \
	 AX_MEDIUM_TFC | AX_MEDIUM_PS | \
	 AX_MEDIUM_AC | AX_MEDIUM_RE )

/* AX88772 & AX88178 RX_CTL values */
#define AX_RX_CTL_SO			0x0080
#define AX_RX_CTL_AP			0x0020
#define AX_RX_CTL_AM			0x0010
#define AX_RX_CTL_AB			0x0008
#define AX_RX_CTL_SEP			0x0004
#define AX_RX_CTL_AMALL			0x0002
#define AX_RX_CTL_PRO			0x0001
#define AX_RX_CTL_MFB_2048		0x0000
#define AX_RX_CTL_MFB_4096		0x0100
#define AX_RX_CTL_MFB_8192		0x0200
#define AX_RX_CTL_MFB_16384		0x0300

#define AX_DEFAULT_RX_CTL	\
	(AX_RX_CTL_SO | AX_RX_CTL_AB )

/* GPIO 0 .. 2 toggles */
#define AX_GPIO_GPO0EN		0x01	/* GPIO0 Output enable */
#define AX_GPIO_GPO_0		0x02	/* GPIO0 Output value */
#define AX_GPIO_GPO1EN		0x04	/* GPIO1 Output enable */
#define AX_GPIO_GPO_1		0x08	/* GPIO1 Output value */
#define AX_GPIO_GPO2EN		0x10	/* GPIO2 Output enable */
#define AX_GPIO_GPO_2		0x20	/* GPIO2 Output value */
#define AX_GPIO_RESERVED	0x40	/* Reserved */
#define AX_GPIO_RSE		0x80	/* Reload serial EEPROM */

#define AX_EEPROM_MAGIC		0xdeadbeef
#define AX88172_EEPROM_LEN	0x40
#define AX88772_EEPROM_LEN	0xff

#define PHY_MODE_MARVELL	0x0000
#define MII_MARVELL_LED_CTRL	0x0018
#define MII_MARVELL_STATUS	0x001b
#define MII_MARVELL_CTRL	0x0014

#define MARVELL_LED_MANUAL	0x0019

#define MARVELL_STATUS_HWCFG	0x0004

#define MARVELL_CTRL_TXDELAY	0x0002
#define MARVELL_CTRL_RXDELAY	0x0080

/* This structure cannot exceed sizeof(unsigned long [5]) AKA 20 bytes */
struct asix_data {
	u8 multi_filter[AX_MCAST_FILTER_SIZE];
	u8 mac_addr[ETH_ALEN];
	u8 phymode;
	u8 ledmode;
	u8 eeprom_len;
};

struct ax88172_int_data {
	__le16 res1;
	u8 link;
	__le16 res2;
	u8 status;
	__le16 res3;
} __packed;

static int asix_read_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			    u16 size, void *data)
{
	void *buf;
	int err = -ENOMEM;

	netdev_dbg(dev->net, "asix_read_cmd() cmd=0x%02x value=0x%04x index=0x%04x size=%d\n",
		   cmd, value, index, size);

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		goto out;

	err = usb_control_msg(
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		dev->udev,
		usb_rcvctrlpipe(dev->udev, 0),
		cmd,
		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value,
		index,
<<<<<<< HEAD
		data,
		size,
		USB_CTRL_GET_TIMEOUT);
#endif
	return ret;
}

static int __ax88179_write_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			       u16 size, void *data, int in_pm)
{
	int ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	int (*fn)(struct usbnet *, u8, u8, u16, u16, const void *, u16);

	BUG_ON(!dev);

	if (!in_pm)
		fn = usbnet_write_cmd;
	else
		fn = usbnet_write_cmd_nopm;

	ret = fn(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR |
		 USB_RECIP_DEVICE, value, index, data, size);

	if (unlikely(ret < 0))
		netdev_warn(dev->net, "Failed to write reg index 0x%04x: %d\n",
			    index, ret);
#else
	ret = usb_control_msg(
=======
		buf,
		size,
		USB_CTRL_GET_TIMEOUT);
	if (err == size)
		memcpy(data, buf, size);
	else if (err >= 0)
		err = -EINVAL;
	kfree(buf);

out:
	return err;
}

static int asix_write_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			     u16 size, void *data)
{
	void *buf = NULL;
	int err = -ENOMEM;

	netdev_dbg(dev->net, "asix_write_cmd() cmd=0x%02x value=0x%04x index=0x%04x size=%d\n",
		   cmd, value, index, size);

	if (data) {
		buf = kmemdup(data, size, GFP_KERNEL);
		if (!buf)
			goto out;
	}

	err = usb_control_msg(
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		dev->udev,
		usb_sndctrlpipe(dev->udev, 0),
		cmd,
		USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value,
		index,
<<<<<<< HEAD
		data,
		size,
		USB_CTRL_SET_TIMEOUT);

#endif
	return ret;
}

static int ax88179_read_cmd_nopm(struct usbnet *dev, u8 cmd, u16 value,
				 u16 index, u16 size, void *data, int eflag)
{
	int ret;

	if (eflag && (2 == size)) {
		u16 buf;
		ret = __ax88179_read_cmd(dev, cmd, value, index, size, &buf, 1);
		le16_to_cpus(&buf);
		*((u16 *)data) = buf;
	} else if (eflag && (4 == size)) {
		u32 buf;
		ret = __ax88179_read_cmd(dev, cmd, value, index, size, &buf, 1);
		le32_to_cpus(&buf);
		*((u32 *)data) = buf;
	} else {
		ret = __ax88179_read_cmd(dev, cmd, value, index, size, data, 1);
	}

	return ret;
}

static int ax88179_write_cmd_nopm(struct usbnet *dev, u8 cmd, u16 value,
				  u16 index, u16 size, void *data)
{
	int ret;

	if (2 == size) {
		u16 buf;
		buf = *((u16 *)data);
		cpu_to_le16s(&buf);
		ret = __ax88179_write_cmd(dev, cmd, value, index,
					  size, &buf, 1);
	} else {
		ret = __ax88179_write_cmd(dev, cmd, value, index,
					  size, data, 1);
	}

	return ret;
}

static int ax88179_read_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			    u16 size, void *data, int eflag)
{

	int ret;

	if (eflag && (2 == size)) {
		u16 buf;
		ret = __ax88179_read_cmd(dev, cmd, value, index, size, &buf, 0);
		le16_to_cpus(&buf);
		*((u16 *)data) = buf;
	} else if (eflag && (4 == size)) {
		u32 buf;
		ret = __ax88179_read_cmd(dev, cmd, value, index, size, &buf, 0);
		le32_to_cpus(&buf);
		*((u32 *)data) = buf;
	} else {
		ret = __ax88179_read_cmd(dev, cmd, value, index, size, data, 0);
	}

	return ret;
}

static int ax88179_write_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			     u16 size, void *data)
{
	int ret;

	if (2 == size) {
		u16 buf;
		buf = *((u16 *)data);
		cpu_to_le16s(&buf);
		ret = __ax88179_write_cmd(dev, cmd, value, index,
					  size, &buf, 0);
	} else {
		ret = __ax88179_write_cmd(dev, cmd, value, index,
					  size, data, 0);
	}

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static void ax88179_write_cmd_async(struct usbnet *dev, u8 cmd, u16 value,
				    u16 index, u16 size, void *data)
{
	if (2 == size) {
		u16 buf;
		buf = *((u16 *)data);
		cpu_to_le16s(&buf);
		usbnet_write_cmd_async(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR |
			       USB_RECIP_DEVICE, value, index, &buf, size);
	} else {
		usbnet_write_cmd_async(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR |
			       USB_RECIP_DEVICE, value, index, data, size);
	}
}
#else

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static void ax88179_async_cmd_callback(struct urb *urb, struct pt_regs *regs)
#else
static void ax88179_async_cmd_callback(struct urb *urb)
#endif
{
	struct usb_ctrlrequest *req = (struct usb_ctrlrequest *)urb->context;

	if (urb->status < 0)
		printk(KERN_ERR "ax88179_async_cmd_callback() failed with %d",
		       urb->status);
=======
		buf,
		size,
		USB_CTRL_SET_TIMEOUT);
	kfree(buf);

out:
	return err;
}

static void asix_async_cmd_callback(struct urb *urb)
{
	struct usb_ctrlrequest *req = (struct usb_ctrlrequest *)urb->context;
	int status = urb->status;

	if (status < 0)
		printk(KERN_DEBUG "asix_async_cmd_callback() failed with %d",
			status);

>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	kfree(req);
	usb_free_urb(urb);
}

static void
<<<<<<< HEAD
ax88179_write_cmd_async(struct usbnet *dev, u8 cmd, u16 value, u16 index,
=======
asix_write_cmd_async(struct usbnet *dev, u8 cmd, u16 value, u16 index,
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
				    u16 size, void *data)
{
	struct usb_ctrlrequest *req;
	int status;
	struct urb *urb;
<<<<<<< HEAD
	void *buf = NULL;
	u16 buf_le;

	if (2 == size) {
		buf_le = *((u16 *)data);
		cpu_to_le16s(&buf_le);
	}

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (urb == NULL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Error allocating URB in write_cmd_async!");
#else
		deverr(dev, "Error allocating URB in write_cmd_async!");
#endif
		return;
	}

	if (data) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
		if (2 == size)
			buf = kmemdup(&buf_le, size, GFP_ATOMIC);
		else
			buf = kmemdup(data, size, GFP_ATOMIC);
#else
			buf = kmalloc(size, GFP_ATOMIC);
#endif

		if (!buf) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
			netdev_err(dev->net, "Error allocating buffer in %s!\n",
				   __func__);
#else
		deverr(dev,  "Error allocating buffer in %s!\n", __func__);
#endif
			return;
		} else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
			if (2 == size)
				memcpy(buf, &buf_le, size);
			else
				memcpy(buf, data, size);
#endif
		}
	}

	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC);
	if (req  == NULL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Failed to allocate memory for control request");
#else
		deverr(dev, "Failed to allocate memory for control request");
#endif
=======

	netdev_dbg(dev->net, "asix_write_cmd_async() cmd=0x%02x value=0x%04x index=0x%04x size=%d\n",
		   cmd, value, index, size);
	if ((urb = usb_alloc_urb(0, GFP_ATOMIC)) == NULL) {
		netdev_err(dev->net, "Error allocating URB in write_cmd_async!\n");
		return;
	}

	if ((req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC)) == NULL) {
		netdev_err(dev->net, "Failed to allocate memory for control request\n");
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		usb_free_urb(urb);
		return;
	}

	req->bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	req->bRequest = cmd;
	req->wValue = cpu_to_le16(value);
	req->wIndex = cpu_to_le16(index);
	req->wLength = cpu_to_le16(size);

	usb_fill_control_urb(urb, dev->udev,
			     usb_sndctrlpipe(dev->udev, 0),
<<<<<<< HEAD
			     (void *)req, buf, size,
			     ax88179_async_cmd_callback, req);

	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status < 0) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Error submitting the control message: status=%d",
			   status);
#else
		deverr(dev, "Error submitting the control message: status=%d",
		       status);
#endif
=======
			     (void *)req, data, size,
			     asix_async_cmd_callback, req);

	if((status = usb_submit_urb(urb, GFP_ATOMIC)) < 0) {
		netdev_err(dev->net, "Error submitting the control message: status=%d\n",
			   status);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		kfree(req);
		usb_free_urb(urb);
	}
}
<<<<<<< HEAD
#endif

static void ax88179_status(struct usbnet *dev, struct urb *urb)
{
	struct ax88179_int_data *event;
=======

static int asix_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	u8  *head;
	u32  header;
	char *packet;
	struct sk_buff *ax_skb;
	u16 size;

	head = (u8 *) skb->data;
	memcpy(&header, head, sizeof(header));
	le32_to_cpus(&header);
	packet = head + sizeof(header);

	skb_pull(skb, 4);

	while (skb->len > 0) {
		if ((header & 0x07ff) != ((~header >> 16) & 0x07ff))
			netdev_err(dev->net, "asix_rx_fixup() Bad Header Length\n");

		/* get the packet length */
		size = (u16) (header & 0x000007ff);

		if ((skb->len) - ((size + 1) & 0xfffe) == 0) {
			u8 alignment = (unsigned long)skb->data & 0x3;
			if (alignment != 0x2) {
				/*
				 * not 16bit aligned so use the room provided by
				 * the 32 bit header to align the data
				 *
				 * note we want 16bit alignment as MAC header is
				 * 14bytes thus ip header will be aligned on
				 * 32bit boundary so accessing ipheader elements
				 * using a cast to struct ip header wont cause
				 * an unaligned accesses.
				 */
				u8 realignment = (alignment + 2) & 0x3;
				memmove(skb->data - realignment,
					skb->data,
					size);
				skb->data -= realignment;
				skb_set_tail_pointer(skb, size);
			}
			return 2;
		}

		if (size > dev->net->mtu + ETH_HLEN) {
			netdev_err(dev->net, "asix_rx_fixup() Bad RX Length %d\n",
				   size);
			return 0;
		}
		ax_skb = skb_clone(skb, GFP_ATOMIC);
		if (ax_skb) {
			u8 alignment = (unsigned long)packet & 0x3;
			ax_skb->len = size;

			if (alignment != 0x2) {
				/*
				 * not 16bit aligned use the room provided by
				 * the 32 bit header to align the data
				 */
				u8 realignment = (alignment + 2) & 0x3;
				memmove(packet - realignment, packet, size);
				packet -= realignment;
			}
			ax_skb->data = packet;
			skb_set_tail_pointer(ax_skb, size);
			usbnet_skb_return(dev, ax_skb);
		} else {
			return 0;
		}

		skb_pull(skb, (size + 1) & 0xfffe);

		if (skb->len < sizeof(header))
			break;

		head = (u8 *) skb->data;
		memcpy(&header, head, sizeof(header));
		le32_to_cpus(&header);
		packet = head + sizeof(header);
		skb_pull(skb, 4);
	}

	if (skb->len < 0) {
		netdev_err(dev->net, "asix_rx_fixup() Bad SKB Length %d\n",
			   skb->len);
		return 0;
	}
	return 1;
}

static struct sk_buff *asix_tx_fixup(struct usbnet *dev, struct sk_buff *skb,
					gfp_t flags)
{
	int padlen;
	int headroom = skb_headroom(skb);
	int tailroom = skb_tailroom(skb);
	u32 packet_len;
	u32 padbytes = 0xffff0000;

	padlen = ((skb->len + 4) % 512) ? 0 : 4;

	if ((!skb_cloned(skb)) &&
	    ((headroom + tailroom) >= (4 + padlen))) {
		if ((headroom < 4) || (tailroom < padlen)) {
			skb->data = memmove(skb->head + 4, skb->data, skb->len);
			skb_set_tail_pointer(skb, skb->len);
		}
	} else {
		struct sk_buff *skb2;
		skb2 = skb_copy_expand(skb, 4, padlen, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	skb_push(skb, 4);
	packet_len = (((skb->len - 4) ^ 0x0000ffff) << 16) + (skb->len - 4);
	cpu_to_le32s(&packet_len);
	skb_copy_to_linear_data(skb, &packet_len, sizeof(packet_len));

	if ((skb->len % 512) == 0) {
		cpu_to_le32s(&padbytes);
		memcpy(skb_tail_pointer(skb), &padbytes, sizeof(padbytes));
		skb_put(skb, sizeof(padbytes));
	}
	return skb;
}

static void asix_status(struct usbnet *dev, struct urb *urb)
{
	struct ax88172_int_data *event;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	int link;

	if (urb->actual_length < 8)
		return;

	event = urb->transfer_buffer;
<<<<<<< HEAD
	link = event->link & AX_INT_PPLS_LINK;

	if (netif_carrier_ok(dev->net) != link) {
		if (link)
			usbnet_defer_kevent(dev, EVENT_LINK_RESET);
		else
			netif_carrier_off(dev->net);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_info(dev->net, "ax88179_178a - Link status is: %d\n",
			    link);
#else
		devinfo(dev, "ax88179_178a - Link status is: %d\n", link);
#endif
	}
}

static int ax88179_mdio_read(struct net_device *netdev, int phy_id, int loc)
{
	struct usbnet *dev = netdev_priv(netdev);
	u16 res;

	ax88179_read_cmd(dev, AX_ACCESS_PHY, phy_id, (__u16)loc, 2, &res, 1);
	return res;
}

static void ax88179_mdio_write(struct net_device *netdev, int phy_id, int loc,
			       int val)
{
	struct usbnet *dev = netdev_priv(netdev);
	u16 res = (u16)val;

	ax88179_write_cmd(dev, AX_ACCESS_PHY, phy_id, (__u16)loc, 2, &res);
}

static int ax88179_suspend(struct usb_interface *intf,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 10)
			   pm_message_t message)
#else
			   u32 message)
#endif
{
	struct usbnet *dev = usb_get_intfdata(intf);
	u16 tmp16;
	u8 tmp8;

	usbnet_suspend(intf, message);

	/* Disable RX path */
	ax88179_read_cmd_nopm(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			      2, 2, &tmp16, 1);
	tmp16 &= ~AX_MEDIUM_RECEIVE_EN;
	ax88179_write_cmd_nopm(dev, AX_ACCESS_MAC,  AX_MEDIUM_STATUS_MODE,
			       2, 2, &tmp16);

	/* Force bz */
	ax88179_read_cmd_nopm(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			      2, 2, &tmp16, 1);
	tmp16 |= AX_PHYPWR_RSTCTL_BZ | AX_PHYPWR_RSTCTL_IPRL;
	ax88179_write_cmd_nopm(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			       2, 2, &tmp16);

	/* change clock */
	tmp8 = 0;
	ax88179_write_cmd_nopm(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &tmp8);

	/* Configure RX control register => stop operation */
	tmp16 = AX_RX_CTL_STOP;
	ax88179_write_cmd_nopm(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &tmp16);

	return 0;
}

static int ax88179_resume(struct usb_interface *intf)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	u16 tmp16;
	u8 tmp8;

	netif_carrier_off(dev->net);

	/* Power up ethernet PHY */
	tmp16 = 0;
	ax88179_write_cmd_nopm(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			       2, 2, &tmp16);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
	usleep_range(1000, 2000);
#else
	msleep(1);
#endif
	tmp16 = AX_PHYPWR_RSTCTL_IPRL;
	ax88179_write_cmd_nopm(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
			       2, 2, &tmp16);
	msleep(200);

	/* Ethernet PHY Auto Detach*/
	ax88179_AutoDetach(dev, 1);

	/* change clock */
	ax88179_read_cmd_nopm(dev, AX_ACCESS_MAC,  AX_CLK_SELECT,
			      1, 1, &tmp8, 0);
	tmp8 |= AX_CLK_SELECT_ACS | AX_CLK_SELECT_BCS;
	ax88179_write_cmd_nopm(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &tmp8);
	msleep(100);

	/* Configure RX control register => start operation */
	tmp16 = AX_RX_CTL_DROPCRCERR | AX_RX_CTL_IPE | AX_RX_CTL_START |
		 AX_RX_CTL_AP | AX_RX_CTL_AMALL | AX_RX_CTL_AB;
	ax88179_write_cmd_nopm(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &tmp16);

	return usbnet_resume(intf);
}

static void
ax88179_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{

	struct usbnet *dev = netdev_priv(net);
	u8 *opt;

	opt = kmalloc(1, GFP_KERNEL);
	if (!opt)
		return;

	if (ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MONITOR_MODE,
			     1, 1, opt, 0) < 0) {
		wolinfo->supported = 0;
		wolinfo->wolopts = 0;
		return;
	}

	wolinfo->supported = WAKE_PHY | WAKE_MAGIC;

	if (*opt & AX_MONITOR_MODE_RWLC)
		wolinfo->wolopts |= WAKE_PHY;
	if (*opt & AX_MONITOR_MODE_RWMP)
		wolinfo->wolopts |= WAKE_MAGIC;

	kfree(opt);
}

static int
ax88179_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 *opt;

	opt = kmalloc(1, GFP_KERNEL);
	if (!opt)
		return -ENOMEM;

	*opt = 0;

	if (wolinfo->wolopts & WAKE_PHY)
		*opt |= AX_MONITOR_MODE_RWLC;
	else
		*opt &= ~AX_MONITOR_MODE_RWLC;

	if (wolinfo->wolopts & WAKE_MAGIC)
		*opt |= AX_MONITOR_MODE_RWMP;
	else
		*opt &= ~AX_MONITOR_MODE_RWMP;

	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MONITOR_MODE, 1, 1, opt);

	kfree(opt);

	return 0;
}

static int ax88179_get_eeprom_len(struct net_device *net)
{
	return AX_EEPROM_LEN;
}

static int
ax88179_get_eeprom(struct net_device *net, struct ethtool_eeprom *eeprom,
		   u8 *data)
{
	struct usbnet *dev = netdev_priv(net);
	u16 *eeprom_buff;
	int first_word, last_word;
	int i;

	if (eeprom->len == 0)
		return -EINVAL;

	eeprom->magic = AX88179_EEPROM_MAGIC;

	first_word = eeprom->offset >> 1;
	last_word = (eeprom->offset + eeprom->len - 1) >> 1;
	eeprom_buff = kmalloc(sizeof(u16) * (last_word - first_word + 1),
			      GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	/* ax88179/178A returns 2 bytes from eeprom on read */
	for (i = first_word; i <= last_word; i++) {
		if (ax88179_read_cmd(dev, AX_ACCESS_EEPROM, i, 1, 2,
				     &(eeprom_buff[i - first_word]), 0) < 0) {
			kfree(eeprom_buff);
			return -EIO;
		}
	}

	memcpy(data, (u8 *)eeprom_buff + (eeprom->offset & 1), eeprom->len);
	kfree(eeprom_buff);
	return 0;
}

static void ax88179_get_drvinfo(struct net_device *net,
				struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
	info->eedump_len = 0x3e;
}

static int ax88179_get_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);
	return mii_ethtool_gset(&dev->mii, cmd);
}

static int ax88179_set_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);
	return mii_ethtool_sset(&dev->mii, cmd);
}

static int ax88179_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);
	return  generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 28)
static int ax88179_netdev_stop(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	u16 tmp16;

	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			 2, 2, &tmp16, 1);
	tmp16 &= ~AX_MEDIUM_RECEIVE_EN;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			  2, 2, &tmp16);
	return 0;
}
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
static int ax88179_set_csums(struct usbnet *dev)
{
	struct ax88179_data *ax179_data = (struct ax88179_data *)dev->data;
	u8 checksum;

	if (ax179_data->checksum & AX_RX_CHECKSUM)
		checksum = AX_RXCOE_DEF_CSUM;
	else
		checksum = 0;

	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RXCOE_CTL, 1, 1, &checksum);

	if (ax179_data->checksum & AX_TX_CHECKSUM)
		checksum = AX_TXCOE_DEF_CSUM;
	else
		checksum = 0;

	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_TXCOE_CTL, 1, 1, &checksum);

	return 0;
}

static u32 ax88179_get_tx_csum(struct net_device *netdev)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct ax88179_data *ax179_data = (struct ax88179_data *)dev->data;
	return ax179_data->checksum & AX_TX_CHECKSUM;
}

static u32 ax88179_get_rx_csum(struct net_device *netdev)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct ax88179_data *ax179_data = (struct ax88179_data *)dev->data;
	return ax179_data->checksum & AX_RX_CHECKSUM;
}

static int ax88179_set_rx_csum(struct net_device *netdev, u32 val)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct ax88179_data *ax179_data = (struct ax88179_data *)dev->data;

	if (val)
		ax179_data->checksum |= AX_RX_CHECKSUM;
	else
		ax179_data->checksum &= ~AX_RX_CHECKSUM;
	return ax88179_set_csums(dev);
}

static int ax88179_set_tx_csum(struct net_device *netdev, u32 val)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct ax88179_data *ax179_data = (struct ax88179_data *)dev->data;

	if (val)
		ax179_data->checksum |= AX_TX_CHECKSUM;
	else
		ax179_data->checksum &= ~AX_TX_CHECKSUM;

	ethtool_op_set_tx_csum(netdev, val);

	return ax88179_set_csums(dev);
}

static int ax88179_set_tso(struct net_device *netdev, u32 data)
{
	if (data)
		netdev->features |= NETIF_F_TSO;
	else
		netdev->features &= ~NETIF_F_TSO;

	return 0;
}
#endif

static struct ethtool_ops ax88179_ethtool_ops = {
	.get_drvinfo		= ax88179_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= ax88179_get_wol,
	.set_wol		= ax88179_set_wol,
	.get_eeprom_len	= ax88179_get_eeprom_len,
	.get_eeprom		= ax88179_get_eeprom,
	.get_settings		= ax88179_get_settings,
	.set_settings		= ax88179_set_settings,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
	.set_tx_csum		= ax88179_set_tx_csum,
	.get_tx_csum		= ax88179_get_tx_csum,
	.get_rx_csum		= ax88179_get_rx_csum,
	.set_rx_csum		= ax88179_set_rx_csum,
	.get_tso		= ethtool_op_get_tso,
	.set_tso		= ax88179_set_tso,
#endif
};


static void ax88179_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct ax88179_data *data = (struct ax88179_data *)&dev->data;
	u8 *m_filter = ((u8 *)dev->data) + 12;
	int mc_count;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	mc_count = net->mc_count;
#else
	mc_count = netdev_mc_count(net);
#endif

	data->rxctl = (AX_RX_CTL_START | AX_RX_CTL_AB | AX_RX_CTL_IPE);

	if (net->flags & IFF_PROMISC) {
		data->rxctl |= AX_RX_CTL_PRO;
	} else if (net->flags & IFF_ALLMULTI
		   || mc_count > AX_MAX_MCAST) {
		data->rxctl |= AX_RX_CTL_AMALL;
	} else if (mc_count == 0) {
=======
	link = event->link & 0x01;
	if (netif_carrier_ok(dev->net) != link) {
		if (link) {
			netif_carrier_on(dev->net);
			usbnet_defer_kevent (dev, EVENT_LINK_RESET );
		} else
			netif_carrier_off(dev->net);
		netdev_dbg(dev->net, "Link Status is: %d\n", link);
	}
}

static inline int asix_set_sw_mii(struct usbnet *dev)
{
	int ret;
	ret = asix_write_cmd(dev, AX_CMD_SET_SW_MII, 0x0000, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to enable software MII access\n");
	return ret;
}

static inline int asix_set_hw_mii(struct usbnet *dev)
{
	int ret;
	ret = asix_write_cmd(dev, AX_CMD_SET_HW_MII, 0x0000, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to enable hardware MII access\n");
	return ret;
}

static inline int asix_get_phy_addr(struct usbnet *dev)
{
	u8 buf[2];
	int ret = asix_read_cmd(dev, AX_CMD_READ_PHY_ID, 0, 0, 2, buf);

	netdev_dbg(dev->net, "asix_get_phy_addr()\n");

	if (ret < 0) {
		netdev_err(dev->net, "Error reading PHYID register: %02x\n", ret);
		goto out;
	}
	netdev_dbg(dev->net, "asix_get_phy_addr() returning 0x%04x\n",
		   *((__le16 *)buf));
	ret = buf[1];

out:
	return ret;
}

static int asix_sw_reset(struct usbnet *dev, u8 flags)
{
	int ret;

        ret = asix_write_cmd(dev, AX_CMD_SW_RESET, flags, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to send software reset: %02x\n", ret);

	return ret;
}

static u16 asix_read_rx_ctl(struct usbnet *dev)
{
	__le16 v;
	int ret = asix_read_cmd(dev, AX_CMD_READ_RX_CTL, 0, 0, 2, &v);

	if (ret < 0) {
		netdev_err(dev->net, "Error reading RX_CTL register: %02x\n", ret);
		goto out;
	}
	ret = le16_to_cpu(v);
out:
	return ret;
}

static int asix_write_rx_ctl(struct usbnet *dev, u16 mode)
{
	int ret;

	netdev_dbg(dev->net, "asix_write_rx_ctl() - mode = 0x%04x\n", mode);
	ret = asix_write_cmd(dev, AX_CMD_WRITE_RX_CTL, mode, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to write RX_CTL mode to 0x%04x: %02x\n",
			   mode, ret);

	return ret;
}

static u16 asix_read_medium_status(struct usbnet *dev)
{
	__le16 v;
	int ret = asix_read_cmd(dev, AX_CMD_READ_MEDIUM_STATUS, 0, 0, 2, &v);

	if (ret < 0) {
		netdev_err(dev->net, "Error reading Medium Status register: %02x\n",
			   ret);
		goto out;
	}
	ret = le16_to_cpu(v);
out:
	return ret;
}

static int asix_write_medium_mode(struct usbnet *dev, u16 mode)
{
	int ret;

	netdev_dbg(dev->net, "asix_write_medium_mode() - mode = 0x%04x\n", mode);
	ret = asix_write_cmd(dev, AX_CMD_WRITE_MEDIUM_MODE, mode, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to write Medium Mode mode to 0x%04x: %02x\n",
			   mode, ret);

	return ret;
}

static int asix_write_gpio(struct usbnet *dev, u16 value, int sleep)
{
	int ret;

	netdev_dbg(dev->net, "asix_write_gpio() - value = 0x%04x\n", value);
	ret = asix_write_cmd(dev, AX_CMD_WRITE_GPIOS, value, 0, 0, NULL);
	if (ret < 0)
		netdev_err(dev->net, "Failed to write GPIO value 0x%04x: %02x\n",
			   value, ret);

	if (sleep)
		msleep(sleep);

	return ret;
}

/*
 * AX88772 & AX88178 have a 16-bit RX_CTL value
 */
static void asix_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;
	u16 rx_ctl = AX_DEFAULT_RX_CTL;

	if (net->flags & IFF_PROMISC) {
		rx_ctl |= AX_RX_CTL_PRO;
	} else if (net->flags & IFF_ALLMULTI ||
		   netdev_mc_count(net) > AX_MAX_MCAST) {
		rx_ctl |= AX_RX_CTL_AMALL;
	} else if (netdev_mc_empty(net)) {
		/* just broadcast and directed */
	} else {
		/* We use the 20 byte dev->data
		 * for our 8 byte filter buffer
		 * to avoid allocating memory that
		 * is tricky to free later */
		struct netdev_hw_addr *ha;
		u32 crc_bits;

		memset(data->multi_filter, 0, AX_MCAST_FILTER_SIZE);

		/* Build the multicast hash filter. */
		netdev_for_each_mc_addr(ha, net) {
			crc_bits = ether_crc(ETH_ALEN, ha->addr) >> 26;
			data->multi_filter[crc_bits >> 3] |=
			    1 << (crc_bits & 7);
		}

		asix_write_cmd_async(dev, AX_CMD_WRITE_MULTI_FILTER, 0, 0,
				   AX_MCAST_FILTER_SIZE, data->multi_filter);

		rx_ctl |= AX_RX_CTL_AM;
	}

	asix_write_cmd_async(dev, AX_CMD_WRITE_RX_CTL, rx_ctl, 0, 0, NULL);
}

static int asix_mdio_read(struct net_device *netdev, int phy_id, int loc)
{
	struct usbnet *dev = netdev_priv(netdev);
	__le16 res;

	mutex_lock(&dev->phy_mutex);
	asix_set_sw_mii(dev);
	asix_read_cmd(dev, AX_CMD_READ_MII_REG, phy_id,
				(__u16)loc, 2, &res);
	asix_set_hw_mii(dev);
	mutex_unlock(&dev->phy_mutex);

	netdev_dbg(dev->net, "asix_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x\n",
		   phy_id, loc, le16_to_cpu(res));

	return le16_to_cpu(res);
}

static void
asix_mdio_write(struct net_device *netdev, int phy_id, int loc, int val)
{
	struct usbnet *dev = netdev_priv(netdev);
	__le16 res = cpu_to_le16(val);

	netdev_dbg(dev->net, "asix_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x\n",
		   phy_id, loc, val);
	mutex_lock(&dev->phy_mutex);
	asix_set_sw_mii(dev);
	asix_write_cmd(dev, AX_CMD_WRITE_MII_REG, phy_id, (__u16)loc, 2, &res);
	asix_set_hw_mii(dev);
	mutex_unlock(&dev->phy_mutex);
}

/* Get the PHY Identifier from the PHYSID1 & PHYSID2 MII registers */
static u32 asix_get_phyid(struct usbnet *dev)
{
	int phy_reg;
	u32 phy_id;

	phy_reg = asix_mdio_read(dev->net, dev->mii.phy_id, MII_PHYSID1);
	if (phy_reg < 0)
		return 0;

	phy_id = (phy_reg & 0xffff) << 16;

	phy_reg = asix_mdio_read(dev->net, dev->mii.phy_id, MII_PHYSID2);
	if (phy_reg < 0)
		return 0;

	phy_id |= (phy_reg & 0xffff);

	return phy_id;
}

static void
asix_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 opt;

	if (asix_read_cmd(dev, AX_CMD_READ_MONITOR_MODE, 0, 0, 1, &opt) < 0) {
		wolinfo->supported = 0;
		wolinfo->wolopts = 0;
		return;
	}
	wolinfo->supported = WAKE_PHY | WAKE_MAGIC;
	wolinfo->wolopts = 0;
	if (opt & AX_MONITOR_MODE) {
		if (opt & AX_MONITOR_LINK)
			wolinfo->wolopts |= WAKE_PHY;
		if (opt & AX_MONITOR_MAGIC)
			wolinfo->wolopts |= WAKE_MAGIC;
	}
}

static int
asix_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 opt = 0;

	if (wolinfo->wolopts & WAKE_PHY)
		opt |= AX_MONITOR_LINK;
	if (wolinfo->wolopts & WAKE_MAGIC)
		opt |= AX_MONITOR_MAGIC;
	if (opt != 0)
		opt |= AX_MONITOR_MODE;

	if (asix_write_cmd(dev, AX_CMD_WRITE_MONITOR_MODE,
			      opt, 0, 0, NULL) < 0)
		return -EINVAL;

	return 0;
}

static int asix_get_eeprom_len(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;

	return data->eeprom_len;
}

static int asix_get_eeprom(struct net_device *net,
			      struct ethtool_eeprom *eeprom, u8 *data)
{
	struct usbnet *dev = netdev_priv(net);
	__le16 *ebuf = (__le16 *)data;
	int i;

	/* Crude hack to ensure that we don't overwrite memory
	 * if an odd length is supplied
	 */
	if (eeprom->len % 2)
		return -EINVAL;

	eeprom->magic = AX_EEPROM_MAGIC;

	/* ax8817x returns 2 bytes from eeprom on read */
	for (i=0; i < eeprom->len / 2; i++) {
		if (asix_read_cmd(dev, AX_CMD_READ_EEPROM,
			eeprom->offset + i, 0, 2, &ebuf[i]) < 0)
			return -EINVAL;
	}
	return 0;
}

static void asix_get_drvinfo (struct net_device *net,
				 struct ethtool_drvinfo *info)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;

	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
	strncpy (info->driver, driver_name, sizeof info->driver);
	strncpy (info->version, DRIVER_VERSION, sizeof info->version);
	info->eedump_len = data->eeprom_len;
}

static u32 asix_get_link(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return mii_link_ok(&dev->mii);
}

static int asix_ioctl (struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

static int asix_set_mac_address(struct net_device *net, void *p)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;
	struct sockaddr *addr = p;

	if (netif_running(net))
		return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(net->dev_addr, addr->sa_data, ETH_ALEN);

	/* We use the 20 byte dev->data
	 * for our 6 byte mac buffer
	 * to avoid allocating memory that
	 * is tricky to free later */
	memcpy(data->mac_addr, addr->sa_data, ETH_ALEN);
	asix_write_cmd_async(dev, AX_CMD_WRITE_NODE_ID, 0, 0, ETH_ALEN,
							data->mac_addr);

	return 0;
}

/* We need to override some ethtool_ops so we require our
   own structure so we don't interfere with other usbnet
   devices that may be connected at the same time. */
static const struct ethtool_ops ax88172_ethtool_ops = {
	.get_drvinfo		= asix_get_drvinfo,
	.get_link		= asix_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= asix_get_wol,
	.set_wol		= asix_set_wol,
	.get_eeprom_len		= asix_get_eeprom_len,
	.get_eeprom		= asix_get_eeprom,
	.get_settings		= usbnet_get_settings,
	.set_settings		= usbnet_set_settings,
	.nway_reset		= usbnet_nway_reset,
};

static void ax88172_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct asix_data *data = (struct asix_data *)&dev->data;
	u8 rx_ctl = 0x8c;

	if (net->flags & IFF_PROMISC) {
		rx_ctl |= 0x01;
	} else if (net->flags & IFF_ALLMULTI ||
		   netdev_mc_count(net) > AX_MAX_MCAST) {
		rx_ctl |= 0x02;
	} else if (netdev_mc_empty(net)) {
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		/* just broadcast and directed */
	} else {
		/* We use the 20 byte dev->data
		 * for our 8 byte filter buffer
		 * to avoid allocating memory that
		 * is tricky to free later */
<<<<<<< HEAD
		u32 crc_bits;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
		struct dev_mc_list *mc_list = net->mc_list;
		int i;

		memset(m_filter, 0, AX_MCAST_FILTER_SIZE);

		/* Build the multicast hash filter. */
		for (i = 0; i < net->mc_count; i++) {
			crc_bits =
			    ether_crc(ETH_ALEN,
				      mc_list->dmi_addr) >> 26;
			*(m_filter + (crc_bits >> 3)) |=
				1 << (crc_bits & 7);
			mc_list = mc_list->next;
		}
#else
		struct netdev_hw_addr *ha;
		memset(m_filter, 0, AX_MCAST_FILTER_SIZE);
		netdev_for_each_mc_addr(ha, net) {
			crc_bits = ether_crc(ETH_ALEN, ha->addr) >> 26;
			*(m_filter + (crc_bits >> 3)) |=
				1 << (crc_bits & 7);
		}
#endif
		ax88179_write_cmd_async(dev, AX_ACCESS_MAC,
					AX_MULTI_FILTER_ARRY,
					AX_MCAST_FILTER_SIZE,
					AX_MCAST_FILTER_SIZE, m_filter);

		data->rxctl |= AX_RX_CTL_AM;
	}

	ax88179_write_cmd_async(dev, AX_ACCESS_MAC, AX_RX_CTL,
				2, 2, &data->rxctl);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
static int
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
ax88179_set_features(struct net_device *net, netdev_features_t features)
#else
ax88179_set_features(struct net_device *net, u32 features)
#endif

{
	u8 tmp;
	struct usbnet *dev = netdev_priv(net);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	netdev_features_t changed = net->features ^ features;
#else
	u32 changed = net->features ^ features;
#endif

	if (changed & NETIF_F_IP_CSUM) {
		ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_TXCOE_CTL,
				 1, 1, &tmp, 0);
		tmp ^= AX_TXCOE_TCP | AX_TXCOE_UDP;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_TXCOE_CTL, 1, 1, &tmp);
	}

	if (changed & NETIF_F_IPV6_CSUM) {
		ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_TXCOE_CTL,
				 1, 1, &tmp, 0);
		tmp ^= AX_TXCOE_TCPV6 | AX_TXCOE_UDPV6;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_TXCOE_CTL, 1, 1, &tmp);
	}

	if (changed & NETIF_F_RXCSUM) {
		ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_RXCOE_CTL,
				 1, 1, &tmp, 0);
		tmp ^= AX_RXCOE_IP | AX_RXCOE_TCP | AX_RXCOE_UDP |
		       AX_RXCOE_TCPV6 | AX_RXCOE_UDPV6;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RXCOE_CTL, 1, 1, &tmp);
	}

	return 0;
}
#endif

static int ax88179_change_mtu(struct net_device *net, int new_mtu)
{
	struct usbnet *dev = netdev_priv(net);
	u16 tmp16;

	if (new_mtu <= 0 || new_mtu > 4088)
		return -EINVAL;

	net->mtu = new_mtu;
	dev->hard_mtu = net->mtu + net->hard_header_len;

	if (net->mtu > 1500) {
		ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
				 2, 2, &tmp16, 1);
		tmp16 |= AX_MEDIUM_JUMBO_EN;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
				  2, 2, &tmp16);
	} else {
		ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
				 2, 2, &tmp16, 1);
		tmp16 &= ~AX_MEDIUM_JUMBO_EN;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
				  2, 2, &tmp16);
	}

	return 0;
}

static int ax88179_set_mac_addr(struct net_device *net, void *p)
{
	struct usbnet *dev = netdev_priv(net);
	struct sockaddr *addr = p;

	if (netif_running(net))
		return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(net->dev_addr, addr->sa_data, ETH_ALEN);

	/* Set the MAC address */
	return ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_NODE_ID, ETH_ALEN,
				 ETH_ALEN, net->dev_addr);

}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
static const struct net_device_ops ax88179_netdev_ops = {
=======
		struct netdev_hw_addr *ha;
		u32 crc_bits;

		memset(data->multi_filter, 0, AX_MCAST_FILTER_SIZE);

		/* Build the multicast hash filter. */
		netdev_for_each_mc_addr(ha, net) {
			crc_bits = ether_crc(ETH_ALEN, ha->addr) >> 26;
			data->multi_filter[crc_bits >> 3] |=
			    1 << (crc_bits & 7);
		}

		asix_write_cmd_async(dev, AX_CMD_WRITE_MULTI_FILTER, 0, 0,
				   AX_MCAST_FILTER_SIZE, data->multi_filter);

		rx_ctl |= 0x10;
	}

	asix_write_cmd_async(dev, AX_CMD_WRITE_RX_CTL, rx_ctl, 0, 0, NULL);
}

static int ax88172_link_reset(struct usbnet *dev)
{
	u8 mode;
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);
	mode = AX88172_MEDIUM_DEFAULT;

	if (ecmd.duplex != DUPLEX_FULL)
		mode |= ~AX88172_MEDIUM_FD;

	netdev_dbg(dev->net, "ax88172_link_reset() speed: %u duplex: %d setting mode to 0x%04x\n",
		   ethtool_cmd_speed(&ecmd), ecmd.duplex, mode);

	asix_write_medium_mode(dev, mode);

	return 0;
}

static const struct net_device_ops ax88172_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= usbnet_start_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_change_mtu		= usbnet_change_mtu,
	.ndo_set_mac_address 	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl		= asix_ioctl,
	.ndo_set_multicast_list = ax88172_set_multicast,
};

static int ax88172_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret = 0;
	u8 buf[ETH_ALEN];
	int i;
	unsigned long gpio_bits = dev->driver_info->data;
	struct asix_data *data = (struct asix_data *)&dev->data;

	data->eeprom_len = AX88172_EEPROM_LEN;

	usbnet_get_endpoints(dev,intf);

	/* Toggle the GPIOs in a manufacturer/model specific way */
	for (i = 2; i >= 0; i--) {
		if ((ret = asix_write_cmd(dev, AX_CMD_WRITE_GPIOS,
					(gpio_bits >> (i * 8)) & 0xff, 0, 0,
					NULL)) < 0)
			goto out;
		msleep(5);
	}

	if ((ret = asix_write_rx_ctl(dev, 0x80)) < 0)
		goto out;

	/* Get the MAC address */
	if ((ret = asix_read_cmd(dev, AX88172_CMD_READ_NODE_ID,
				0, 0, ETH_ALEN, buf)) < 0) {
		dbg("read AX_CMD_READ_NODE_ID failed: %d", ret);
		goto out;
	}
	memcpy(dev->net->dev_addr, buf, ETH_ALEN);

	/* Initialize MII structure */
	dev->mii.dev = dev->net;
	dev->mii.mdio_read = asix_mdio_read;
	dev->mii.mdio_write = asix_mdio_write;
	dev->mii.phy_id_mask = 0x3f;
	dev->mii.reg_num_mask = 0x1f;
	dev->mii.phy_id = asix_get_phy_addr(dev);

	dev->net->netdev_ops = &ax88172_netdev_ops;
	dev->net->ethtool_ops = &ax88172_ethtool_ops;

	asix_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR, BMCR_RESET);
	asix_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE,
		ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	mii_nway_restart(&dev->mii);

	return 0;

out:
	return ret;
}

static const struct ethtool_ops ax88772_ethtool_ops = {
	.get_drvinfo		= asix_get_drvinfo,
	.get_link		= asix_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= asix_get_wol,
	.set_wol		= asix_set_wol,
	.get_eeprom_len		= asix_get_eeprom_len,
	.get_eeprom		= asix_get_eeprom,
	.get_settings		= usbnet_get_settings,
	.set_settings		= usbnet_set_settings,
	.nway_reset		= usbnet_nway_reset,
};

static int ax88772_link_reset(struct usbnet *dev)
{
	u16 mode;
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);
	mode = AX88772_MEDIUM_DEFAULT;

	if (ethtool_cmd_speed(&ecmd) != SPEED_100)
		mode &= ~AX_MEDIUM_PS;

	if (ecmd.duplex != DUPLEX_FULL)
		mode &= ~AX_MEDIUM_FD;

	netdev_dbg(dev->net, "ax88772_link_reset() speed: %u duplex: %d setting mode to 0x%04x\n",
		   ethtool_cmd_speed(&ecmd), ecmd.duplex, mode);

	asix_write_medium_mode(dev, mode);

	return 0;
}

static const struct net_device_ops ax88772_netdev_ops = {
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= usbnet_start_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
<<<<<<< HEAD
	.ndo_change_mtu		= ax88179_change_mtu,
	.ndo_do_ioctl		= ax88179_ioctl,
	.ndo_set_mac_address	= ax88179_set_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 2, 0)
	.ndo_set_multicast_list	= ax88179_set_multicast,
#else
	.ndo_set_rx_mode	= ax88179_set_multicast,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	.ndo_set_features	= ax88179_set_features,
#endif
};
#endif

static int ax88179_check_eeprom(struct usbnet *dev)
{
	u8 i;
	u8 buf[2];
	u8 eeprom[20];
	u16 csum, delay = HZ / 10;
	unsigned long jtimeout;

	/* Read EEPROM content */
	for (i = 0 ; i < 6; i++) {

		buf[0] = i;
		if (ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_SROM_ADDR,
				      1, 1, buf) < 0)
			return -EINVAL;

		buf[0] = EEP_RD;
		if (ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_SROM_CMD,
				      1, 1, buf) < 0)
			return -EINVAL;

		jtimeout = jiffies + delay;
		do {
			ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_SROM_CMD,
					 1, 1, buf, 0);

			if (time_after(jiffies, jtimeout))
				return -EINVAL;
		} while (buf[0] & EEP_BUSY);


		ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_SROM_DATA_LOW,
				 2, 2, &eeprom[i * 2], 0);

		if ((i == 0) && (eeprom[0] == 0xFF))
			return -EINVAL;
	}

	csum = eeprom[6] + eeprom[7] + eeprom[8] + eeprom[9];
	csum = (csum >> 8) + (csum & 0xff);

	if ((csum + eeprom[10]) == 0xff)
		return AX_EEP_EFUSE_CORRECT;
	else
		return -EINVAL;
}

static int ax88179_check_efuse(struct usbnet *dev, void *ledmode)
{
	u8	i;
	u8	efuse[64] = {0x00};
	u16	csum = 0;

	if (ax88179_read_cmd(dev, AX_ACCESS_EFUSE, 0, 64, 64, efuse, 0) < 0)
		return -EINVAL;

	if (efuse[0] == 0xFF)
		return -EINVAL;

	for (i = 0; i < 64; i++)
		csum = csum + efuse[i];

	while (csum > 255)
		csum = (csum & 0x00FF) + ((csum >> 8) & 0x00FF);

	if (csum == 0xFF) {
		memcpy((u8 *)ledmode, &efuse[51], 2);
		return AX_EEP_EFUSE_CORRECT;
	} else {
		return -EINVAL;
	}
}

static int ax88179_convert_old_led(struct usbnet *dev, u8 efuse, void *ledvalue)
{
	u8 ledmode;
	u16 tmp;
	u16 led;

	/* loaded the old eFuse LED Mode */
	if (efuse) {
		if (ax88179_read_cmd(dev, AX_ACCESS_EFUSE, 0x18,
				     1, 2, &tmp, 1) < 0)
			return -EINVAL;
		ledmode = (u8)(tmp & 0xFF);
	} else { /* loaded the old EEprom LED Mode */
		if (ax88179_read_cmd(dev, AX_ACCESS_EEPROM, 0x3C,
				     1, 2, &tmp, 1) < 0)
			return -EINVAL;
		ledmode = (u8) (tmp >> 8);
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	netdev_dbg(dev->net, "Old LED Mode = %02X\n", ledmode);
#else
	devdbg(dev, "Old LED Mode = %02X\n", ledmode);
#endif
	switch (ledmode) {
	case 0xFF:
		led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 |
		      LED1_LINK_1000 | LED2_ACTIVE | LED2_LINK_10 |
		      LED2_LINK_100 | LED2_LINK_1000 | LED_VALID;
		break;
	case 0xFE:
		led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 | LED_VALID;
		break;
	case 0xFD:
		led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 |
		      LED2_LINK_10 | LED_VALID;
		break;
	case 0xFC:
		led = LED0_ACTIVE | LED1_ACTIVE | LED1_LINK_1000 | LED2_ACTIVE |
		      LED2_LINK_100 | LED2_LINK_10 | LED_VALID;
		break;
	default:
		led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 |
		      LED1_LINK_1000 | LED2_ACTIVE | LED2_LINK_10 |
		      LED2_LINK_100 | LED2_LINK_1000 | LED_VALID;
		break;
	}

	memcpy((u8 *)ledvalue, &led, 2);

	return 0;
}

static int ax88179_led_setting(struct usbnet *dev)
{
	u8 ledfd, value = 0;
	u16 tmp, ledact, ledlink, ledvalue = 0, delay = HZ / 10;
	unsigned long jtimeout;

	/* Check AX88179 version. UA1 or UA2 */
	ax88179_read_cmd(dev, AX_ACCESS_MAC, GENERAL_STATUS, 1, 1, &value, 0);

	/* UA1 */
	if (!(value & AX_SECLD)) {
		value = AX_GPIO_CTRL_GPIO3EN | AX_GPIO_CTRL_GPIO2EN |
			AX_GPIO_CTRL_GPIO1EN;
		if (ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_GPIO_CTRL,
				      1, 1, &value) < 0)
			return -EINVAL;
	}

	/* check EEprom */
	if (ax88179_check_eeprom(dev) == AX_EEP_EFUSE_CORRECT) {
		value = 0x42;
		if (ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_SROM_ADDR,
				      1, 1, &value) < 0)
			return -EINVAL;

		value = EEP_RD;
		if (ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_SROM_CMD,
				      1, 1, &value) < 0)
			return -EINVAL;

		jtimeout = jiffies + delay;
		do {
			ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_SROM_CMD,
					 1, 1, &value, 0);

			ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_SROM_CMD,
					 1, 1, &value, 0);

			if (time_after(jiffies, jtimeout))
				return -EINVAL;
		} while (value & EEP_BUSY);

		ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_SROM_DATA_HIGH,
				 1, 1, &value, 0);
		ledvalue = (value << 8);
		ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_SROM_DATA_LOW,
				 1, 1, &value, 0);
		ledvalue |= value;

		/* load internal ROM for defaule setting */
		if ((ledvalue == 0xFFFF) || ((ledvalue & LED_VALID) == 0))
			ax88179_convert_old_led(dev, 0, &ledvalue);

	} else if (ax88179_check_efuse(dev, &ledvalue) ==
				       AX_EEP_EFUSE_CORRECT) { /* check efuse */
		if ((ledvalue == 0xFFFF) || ((ledvalue & LED_VALID) == 0))
			ax88179_convert_old_led(dev, 0, &ledvalue);
	} else {
		ax88179_convert_old_led(dev, 0, &ledvalue);
	}

	tmp = GMII_PHY_PAGE_SELECT_EXT;
	ax88179_write_cmd(dev, AX_ACCESS_PHY, AX88179_PHY_ID,
			  GMII_PHY_PAGE_SELECT, 2, &tmp);

	tmp = 0x2c;
	ax88179_write_cmd(dev, AX_ACCESS_PHY, AX88179_PHY_ID,
			  GMII_PHYPAGE, 2, &tmp);

	ax88179_read_cmd(dev, AX_ACCESS_PHY, AX88179_PHY_ID,
			 GMII_LED_ACTIVE, 2, &ledact, 1);

	ax88179_read_cmd(dev, AX_ACCESS_PHY, AX88179_PHY_ID,
			 GMII_LED_LINK, 2, &ledlink, 1);

	ledact &= GMII_LED_ACTIVE_MASK;
	ledlink &= GMII_LED_LINK_MASK;

	if (ledvalue & LED0_ACTIVE)
		ledact |= GMII_LED0_ACTIVE;
	if (ledvalue & LED1_ACTIVE)
		ledact |= GMII_LED1_ACTIVE;
	if (ledvalue & LED2_ACTIVE)
		ledact |= GMII_LED2_ACTIVE;

	if (ledvalue & LED0_LINK_10)
		ledlink |= GMII_LED0_LINK_10;
	if (ledvalue & LED1_LINK_10)
		ledlink |= GMII_LED1_LINK_10;
	if (ledvalue & LED2_LINK_10)
		ledlink |= GMII_LED2_LINK_10;

	if (ledvalue & LED0_LINK_100)
		ledlink |= GMII_LED0_LINK_100;
	if (ledvalue & LED1_LINK_100)
		ledlink |= GMII_LED1_LINK_100;
	if (ledvalue & LED2_LINK_100)
		ledlink |= GMII_LED2_LINK_100;

	if (ledvalue & LED0_LINK_1000)
		ledlink |= GMII_LED0_LINK_1000;
	if (ledvalue & LED1_LINK_1000)
		ledlink |= GMII_LED1_LINK_1000;
	if (ledvalue & LED2_LINK_1000)
		ledlink |= GMII_LED2_LINK_1000;

	tmp = ledact;
	ax88179_write_cmd(dev, AX_ACCESS_PHY, AX88179_PHY_ID,
			  GMII_LED_ACTIVE, 2, &tmp);

	tmp = ledlink;
	ax88179_write_cmd(dev, AX_ACCESS_PHY, AX88179_PHY_ID,
			  GMII_LED_LINK, 2, &tmp);

	tmp = GMII_PHY_PAGE_SELECT_PAGE0;
	ax88179_write_cmd(dev, AX_ACCESS_PHY, AX88179_PHY_ID,
			  GMII_PHY_PAGE_SELECT, 2, &tmp);

	/* LED full duplex setting */
	ledfd = 0;
	if (ledvalue & LED0_FD)
		ledfd |= 0x01;
	else if ((ledvalue & LED0_USB3_MASK) == 0)
		ledfd |= 0x02;


	if (ledvalue & LED1_FD)
		ledfd |= 0x04;
	else if ((ledvalue & LED1_USB3_MASK) == 0)
		ledfd |= 0x08;

	if (ledvalue & LED2_FD) /* LED2_FD */
		ledfd |= 0x10;
	else if ((ledvalue & LED2_USB3_MASK) == 0) /* LED2_USB3 */
		ledfd |= 0x20;

	ax88179_write_cmd(dev, AX_ACCESS_MAC, 0x73, 1, 1, &ledfd);
=======
	.ndo_change_mtu		= usbnet_change_mtu,
	.ndo_set_mac_address 	= asix_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl		= asix_ioctl,
	.ndo_set_multicast_list = asix_set_multicast,
};

static int ax88772_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret, embd_phy;
	u16 rx_ctl;
	struct asix_data *data = (struct asix_data *)&dev->data;
	u8 buf[ETH_ALEN];
	u32 phyid;

	data->eeprom_len = AX88772_EEPROM_LEN;

	usbnet_get_endpoints(dev,intf);

	if ((ret = asix_write_gpio(dev,
			AX_GPIO_RSE | AX_GPIO_GPO_2 | AX_GPIO_GPO2EN, 5)) < 0)
		goto out;

	/* 0x10 is the phy id of the embedded 10/100 ethernet phy */
	embd_phy = ((asix_get_phy_addr(dev) & 0x1f) == 0x10 ? 1 : 0);
	if ((ret = asix_write_cmd(dev, AX_CMD_SW_PHY_SELECT,
				embd_phy, 0, 0, NULL)) < 0) {
		dbg("Select PHY #1 failed: %d", ret);
		goto out;
	}

	if ((ret = asix_sw_reset(dev, AX_SWRESET_IPPD | AX_SWRESET_PRL)) < 0)
		goto out;

	msleep(150);
	if ((ret = asix_sw_reset(dev, AX_SWRESET_CLEAR)) < 0)
		goto out;

	msleep(150);
	if (embd_phy) {
		if ((ret = asix_sw_reset(dev, AX_SWRESET_IPRL)) < 0)
			goto out;
	}
	else {
		if ((ret = asix_sw_reset(dev, AX_SWRESET_PRTE)) < 0)
			goto out;
	}

	msleep(150);
	rx_ctl = asix_read_rx_ctl(dev);
	dbg("RX_CTL is 0x%04x after software reset", rx_ctl);
	if ((ret = asix_write_rx_ctl(dev, 0x0000)) < 0)
		goto out;

	rx_ctl = asix_read_rx_ctl(dev);
	dbg("RX_CTL is 0x%04x setting to 0x0000", rx_ctl);

	/* Get the MAC address */
	if ((ret = asix_read_cmd(dev, AX_CMD_READ_NODE_ID,
				0, 0, ETH_ALEN, buf)) < 0) {
		dbg("Failed to read MAC address: %d", ret);
		goto out;
	}
	memcpy(dev->net->dev_addr, buf, ETH_ALEN);

	/* Initialize MII structure */
	dev->mii.dev = dev->net;
	dev->mii.mdio_read = asix_mdio_read;
	dev->mii.mdio_write = asix_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;
	dev->mii.phy_id = asix_get_phy_addr(dev);

	phyid = asix_get_phyid(dev);
	dbg("PHYID=0x%08x", phyid);

	if ((ret = asix_sw_reset(dev, AX_SWRESET_PRL)) < 0)
		goto out;

	msleep(150);

	if ((ret = asix_sw_reset(dev, AX_SWRESET_IPRL | AX_SWRESET_PRL)) < 0)
		goto out;

	msleep(150);

	dev->net->netdev_ops = &ax88772_netdev_ops;
	dev->net->ethtool_ops = &ax88772_ethtool_ops;

	asix_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR, BMCR_RESET);
	asix_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE,
			ADVERTISE_ALL | ADVERTISE_CSMA);
	mii_nway_restart(&dev->mii);

	if ((ret = asix_write_medium_mode(dev, AX88772_MEDIUM_DEFAULT)) < 0)
		goto out;

	if ((ret = asix_write_cmd(dev, AX_CMD_WRITE_IPG0,
				AX88772_IPG0_DEFAULT | AX88772_IPG1_DEFAULT,
				AX88772_IPG2_DEFAULT, 0, NULL)) < 0) {
		dbg("Write IPG,IPG1,IPG2 failed: %d", ret);
		goto out;
	}

	/* Set RX_CTL to default values with 2k buffer, and enable cactus */
	if ((ret = asix_write_rx_ctl(dev, AX_DEFAULT_RX_CTL)) < 0)
		goto out;

	rx_ctl = asix_read_rx_ctl(dev);
	dbg("RX_CTL is 0x%04x after all initializations", rx_ctl);

	rx_ctl = asix_read_medium_status(dev);
	dbg("Medium Status is 0x%04x after all initializations", rx_ctl);

	/* Asix framing packs multiple eth frames into a 2K usb bulk transfer */
	if (dev->driver_info->flags & FLAG_FRAMING_AX) {
		/* hard_mtu  is still the default - the device does not support
		   jumbo eth frames */
		dev->rx_urb_size = 2048;
	}
	return 0;

out:
	return ret;
}

static struct ethtool_ops ax88178_ethtool_ops = {
	.get_drvinfo		= asix_get_drvinfo,
	.get_link		= asix_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= asix_get_wol,
	.set_wol		= asix_set_wol,
	.get_eeprom_len		= asix_get_eeprom_len,
	.get_eeprom		= asix_get_eeprom,
	.get_settings		= usbnet_get_settings,
	.set_settings		= usbnet_set_settings,
	.nway_reset		= usbnet_nway_reset,
};

static int marvell_phy_init(struct usbnet *dev)
{
	struct asix_data *data = (struct asix_data *)&dev->data;
	u16 reg;

	netdev_dbg(dev->net, "marvell_phy_init()\n");

	reg = asix_mdio_read(dev->net, dev->mii.phy_id, MII_MARVELL_STATUS);
	netdev_dbg(dev->net, "MII_MARVELL_STATUS = 0x%04x\n", reg);

	asix_mdio_write(dev->net, dev->mii.phy_id, MII_MARVELL_CTRL,
			MARVELL_CTRL_RXDELAY | MARVELL_CTRL_TXDELAY);

	if (data->ledmode) {
		reg = asix_mdio_read(dev->net, dev->mii.phy_id,
			MII_MARVELL_LED_CTRL);
		netdev_dbg(dev->net, "MII_MARVELL_LED_CTRL (1) = 0x%04x\n", reg);

		reg &= 0xf8ff;
		reg |= (1 + 0x0100);
		asix_mdio_write(dev->net, dev->mii.phy_id,
			MII_MARVELL_LED_CTRL, reg);

		reg = asix_mdio_read(dev->net, dev->mii.phy_id,
			MII_MARVELL_LED_CTRL);
		netdev_dbg(dev->net, "MII_MARVELL_LED_CTRL (2) = 0x%04x\n", reg);
		reg &= 0xfc0f;
	}
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a

	return 0;
}

<<<<<<< HEAD
static int ax88179_AutoDetach(struct usbnet *dev, int in_pm)
{
	u16 tmp16;
	u8 tmp8;
	int (*fnr)(struct usbnet *, u8, u16, u16, u16, void *, int);
	int (*fnw)(struct usbnet *, u8, u16, u16, u16, void *);

	if (!in_pm) {
		fnr = ax88179_read_cmd;
		fnw = ax88179_write_cmd;
	} else {
		fnr = ax88179_read_cmd_nopm;
		fnw = ax88179_write_cmd_nopm;
	}

	if (fnr(dev, AX_ACCESS_EEPROM, 0x43, 1, 2, &tmp16, 1) < 0)
		return 0;

	if ((tmp16 == 0xFFFF) || (!(tmp16 & 0x0100)))
		return 0;

	/* Enable Auto Detach bit */
	tmp8 = 0;
	fnr(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &tmp8, 0);
	tmp8 |= AX_CLK_SELECT_ULR;
	fnw(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, &tmp8);

	fnr(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, &tmp16, 1);
	tmp16 |= AX_PHYPWR_RSTCTL_AUTODETACH;
	fnw(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, &tmp16);
=======
static int marvell_led_status(struct usbnet *dev, u16 speed)
{
	u16 reg = asix_mdio_read(dev->net, dev->mii.phy_id, MARVELL_LED_MANUAL);

	netdev_dbg(dev->net, "marvell_led_status() read 0x%04x\n", reg);

	/* Clear out the center LED bits - 0x03F0 */
	reg &= 0xfc0f;

	switch (speed) {
		case SPEED_1000:
			reg |= 0x03e0;
			break;
		case SPEED_100:
			reg |= 0x03b0;
			break;
		default:
			reg |= 0x02f0;
	}

	netdev_dbg(dev->net, "marvell_led_status() writing 0x%04x\n", reg);
	asix_mdio_write(dev->net, dev->mii.phy_id, MARVELL_LED_MANUAL, reg);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a

	return 0;
}

<<<<<<< HEAD
static int ax88179_bind(struct usbnet *dev, struct usb_interface *intf)
{
	void *buf;
	u16 *tmp16;
	u8 *tmp;

	struct ax88179_data *ax179_data = (struct ax88179_data *)dev->data;

	usbnet_get_endpoints(dev, intf);

	if (msg_enable != 0)
		dev->msg_enable = msg_enable;

	buf = kmalloc(6, GFP_KERNEL);
	if (!buf) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Cannot allocate memory for buffer");
#else
		deverr(dev, "Cannot allocate memory for buffer");
#endif
		return -ENOMEM;
	}
	tmp16 = (u16 *)buf;
	tmp = (u8 *)buf;

	memset(ax179_data, 0, sizeof(*ax179_data));

	/* Power up ethernet PHY */
	*tmp16 = 0;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, tmp16);
	*tmp16 = AX_PHYPWR_RSTCTL_IPRL;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, tmp16);
	msleep(200);

	*tmp = AX_CLK_SELECT_ACS | AX_CLK_SELECT_BCS;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, tmp);
	msleep(100);

	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_NODE_ID, ETH_ALEN,
			  ETH_ALEN, dev->net->dev_addr, 0);
	memcpy(dev->net->perm_addr, dev->net->dev_addr, ETH_ALEN);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_dbg(dev->net, "MAC [%02x-%02x-%02x-%02x-%02x-%02x]\n",
			   dev->net->dev_addr[0], dev->net->dev_addr[1],
			   dev->net->dev_addr[2], dev->net->dev_addr[3],
			   dev->net->dev_addr[4], dev->net->dev_addr[5]);
#else
		devdbg(dev, "MAC [%02x-%02x-%02x-%02x-%02x-%02x]\n",
		       dev->net->dev_addr[0], dev->net->dev_addr[1],
		       dev->net->dev_addr[2], dev->net->dev_addr[3],
		       dev->net->dev_addr[4], dev->net->dev_addr[5]);
#endif

	/* RX bulk configuration, default for USB3.0 to Giga*/
	memcpy(tmp, &AX88179_BULKIN_SIZE[0], 5);
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_BULKIN_QCTRL, 5, 5, tmp);

	dev->rx_urb_size = 1024 * 20;

	tmp[0] = 0x34;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PAUSE_WATERLVL_LOW, 1, 1, tmp);

	tmp[0] = 0x52;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PAUSE_WATERLVL_HIGH,
			  1, 1, tmp);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30)
	dev->net->do_ioctl = ax88179_ioctl;
	dev->net->set_multicast_list = ax88179_set_multicast;
	dev->net->set_mac_address = ax88179_set_mac_addr;
	dev->net->change_mtu = ax88179_change_mtu;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 28)
	dev->net->stop = ax88179_netdev_stop;
#endif
#else
	dev->net->netdev_ops = &ax88179_netdev_ops;
#endif

	dev->net->ethtool_ops = &ax88179_ethtool_ops;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)
	dev->net->needed_headroom = 8;
#endif

	/* Initialize MII structure */
	dev->mii.dev = dev->net;
	dev->mii.mdio_read = ax88179_mdio_read;
	dev->mii.mdio_write = ax88179_mdio_write;
	dev->mii.phy_id_mask = 0xff;
	dev->mii.reg_num_mask = 0xff;
	dev->mii.phy_id = 0x03;
	dev->mii.supports_gmii = 1;

	dev->net->features |= NETIF_F_IP_CSUM;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 22)
	dev->net->features |= NETIF_F_IPV6_CSUM;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	dev->net->hw_features |= NETIF_F_IP_CSUM;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 22)
	dev->net->hw_features |= NETIF_F_IPV6_CSUM;
#endif
	dev->net->hw_features |= NETIF_F_SG | NETIF_F_TSO;
#endif

	/* Enable checksum offload */
	*tmp = AX_RXCOE_IP | AX_RXCOE_TCP | AX_RXCOE_UDP |
	       AX_RXCOE_TCPV6 | AX_RXCOE_UDPV6;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RXCOE_CTL, 1, 1, tmp);

	*tmp = AX_TXCOE_IP | AX_TXCOE_TCP | AX_TXCOE_UDP |
	       AX_TXCOE_TCPV6 | AX_TXCOE_UDPV6;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_TXCOE_CTL, 1, 1, tmp);

	ax179_data->checksum |= AX_RX_CHECKSUM | AX_TX_CHECKSUM;

	/* Configure RX control register => start operation */
	*tmp16 = AX_RX_CTL_DROPCRCERR | AX_RX_CTL_IPE | AX_RX_CTL_START |
			AX_RX_CTL_AP | AX_RX_CTL_AMALL | AX_RX_CTL_AB;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, tmp16);

	*tmp = AX_MONITOR_MODE_PMETYPE | AX_MONITOR_MODE_PMEPOL |
						AX_MONITOR_MODE_RWMP;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MONITOR_MODE, 1, 1, tmp);

	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MONITOR_MODE, 1, 1, tmp, 0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_dbg(dev->net, "Monitor mode = 0x%02x\n", *tmp);
#else
		devdbg(dev, "Monitor mode = 0x%02x\n", *tmp);
#endif
	/* Configure default medium type => giga */
	*tmp16 = AX_MEDIUM_RECEIVE_EN	 | AX_MEDIUM_TXFLOW_CTRLEN |
		 AX_MEDIUM_RXFLOW_CTRLEN | AX_MEDIUM_ALWAYS_ONE	   |
		 AX_MEDIUM_FULL_DUPLEX	 | AX_MEDIUM_GIGAMODE;

	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			  2, 2, tmp16);

	ax88179_led_setting(dev);

	/* Restart autoneg */
	mii_nway_restart(&dev->mii);

	netif_carrier_off(dev->net);

	kfree(buf);
	printk(version);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_info(dev->net, "mtu %d\n", dev->net->mtu);
#else
		devinfo(dev, "mtu %d\n", dev->net->mtu);
#endif
	return 0;

}

static void ax88179_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	u16 tmp16;
	u8 tmp8;
	struct ax88179_data *ax179_data = (struct ax88179_data *) dev->data;

	if (ax179_data) {
		/* Configure RX control register => stop operation */
		tmp16 = AX_RX_CTL_STOP;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &tmp16);

		tmp8 = 0x0;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_CLK_SELECT,
				  1, 1, &tmp8);

		/* Power down ethernet PHY */
		tmp16 = AX_PHYPWR_RSTCTL_BZ | AX_PHYPWR_RSTCTL_IPRL;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL,
				  2, 2, &tmp16);
		msleep(200);
	}
}

static void
ax88179_rx_checksum(struct sk_buff *skb, u32 *pkt_hdr)
{
	skb->ip_summed = CHECKSUM_NONE;

	/* checksum error bit is set */
	if ((*pkt_hdr & AX_RXHDR_L3CSUM_ERR) ||
	    (*pkt_hdr & AX_RXHDR_L4CSUM_ERR))
		return;

	/* It must be a TCP or UDP packet with a valid checksum */
	if (((*pkt_hdr & AX_RXHDR_L4_TYPE_MASK) == AX_RXHDR_L4_TYPE_TCP) ||
	    ((*pkt_hdr & AX_RXHDR_L4_TYPE_MASK) == AX_RXHDR_L4_TYPE_UDP))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
}

static int ax88179_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	struct sk_buff *ax_skb;
	int pkt_cnt;
	u32 rx_hdr;
	u16 hdr_off;
	u32 *pkt_hdr;

	skb_trim(skb, skb->len - 4);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
	memcpy(&rx_hdr, skb_tail_pointer(skb), sizeof(rx_hdr));
#else
	memcpy(&rx_hdr, skb->tail, sizeof(rx_hdr));
#endif
	le32_to_cpus(&rx_hdr);

	pkt_cnt = (u16)rx_hdr;
	hdr_off = (u16)(rx_hdr >> 16);
	pkt_hdr = (u32 *)(skb->data + hdr_off);

	while (pkt_cnt--) {
		u16 pkt_len;

		le32_to_cpus(pkt_hdr);
		pkt_len = (*pkt_hdr >> 16) & 0x1fff;

		/* Check CRC or runt packet */
		if ((*pkt_hdr & AX_RXHDR_CRC_ERR) ||
		    (*pkt_hdr & AX_RXHDR_DROP_ERR)) {
			skb_pull(skb, (pkt_len + 7) & 0xFFF8);
			pkt_hdr++;
			continue;
		}

		if (pkt_cnt == 0) {
			/* Skip IP alignment psudo header */
			skb_pull(skb, 2);
			skb->len = pkt_len;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
			skb->tail = skb->data + pkt_len;
#else
			skb_set_tail_pointer(skb, pkt_len);
#endif
			skb->truesize = pkt_len + sizeof(struct sk_buff);
			ax88179_rx_checksum(skb, pkt_hdr);
			return 1;
		}

		ax_skb = skb_clone(skb, GFP_ATOMIC);
		if (ax_skb) {
			ax_skb->len = pkt_len;
			ax_skb->data = skb->data + 2;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
			ax_skb->tail = ax_skb->data + pkt_len;
#else
			skb_set_tail_pointer(ax_skb, pkt_len);
#endif
			ax_skb->truesize = pkt_len + sizeof(struct sk_buff);
			ax88179_rx_checksum(ax_skb, pkt_hdr);
			usbnet_skb_return(dev, ax_skb);
		} else {
			return 0;
		}

		skb_pull(skb, (pkt_len + 7) & 0xFFF8);
		pkt_hdr++;
	}
	return 1;
}

static struct sk_buff *
ax88179_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
	u32 tx_hdr1, tx_hdr2;
	int frame_size = dev->maxpacket;
	int mss = skb_shinfo(skb)->gso_size;
	int headroom;
	int tailroom;

	tx_hdr1 = skb->len;
	tx_hdr2 = mss;
	if (((skb->len + 8) % frame_size) == 0) {
		tx_hdr2 |= 0x80008000;	/* Enable padding */
		skb->len += 2;
	}

	if (skb_linearize(skb))
		return NULL;

	headroom = skb_headroom(skb);
	tailroom = skb_tailroom(skb);

	if ((headroom + tailroom) >= 8) {
		if (headroom < 8) {
			skb->data = memmove(skb->head + 8, skb->data, skb->len);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
			skb->tail = skb->data + skb->len;
#else
			skb_set_tail_pointer(skb, skb->len);
#endif
		}
	} else {
		struct sk_buff *skb2;
		skb2 = skb_copy_expand(skb, 8, 0, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	skb_push(skb, 4);
	cpu_to_le32s(&tx_hdr2);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
	memcpy(skb->data, &tx_hdr2, 4);
#else
	skb_copy_to_linear_data(skb, &tx_hdr2, 4);
#endif

	skb_push(skb, 4);
	cpu_to_le32s(&tx_hdr1);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
	memcpy(skb->data, &tx_hdr1, 4);
#else
	skb_copy_to_linear_data(skb, &tx_hdr1, 4);
#endif

	return skb;
}

static int ax88179_link_reset(struct usbnet *dev)
{
	struct ax88179_data *data = (struct ax88179_data *)&dev->data;
	u8 tmp[5], link_sts;
	u16 mode, tmp16, delay = HZ/10;
	u32 tmp32 = 0x40000000;
	unsigned long jtimeout;

	jtimeout = jiffies + delay;

	while (tmp32 & 0x40000000) {
		mode = 0;
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_CTL, 2, 2, &mode);
		ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_CTL,
				  2, 2, &data->rxctl);

		/* link up, check the usb device control TX FIFO full or empty*/
		ax88179_read_cmd(dev, 0x81, 0x8c, 0, 4, &tmp32, 1);

		if (time_after(jiffies, jtimeout))
			return 0;
	}

	mode = AX_MEDIUM_RECEIVE_EN    | AX_MEDIUM_TXFLOW_CTRLEN |
		   AX_MEDIUM_RXFLOW_CTRLEN | AX_MEDIUM_ALWAYS_ONE;

	ax88179_read_cmd(dev, AX_ACCESS_MAC, PHYSICAL_LINK_STATUS,
			 1, 1, &link_sts, 0);
	ax88179_read_cmd(dev, AX_ACCESS_PHY, AX88179_PHY_ID,
			 GMII_PHY_PHYSR, 2, &tmp16, 1);

	if (!(tmp16 & GMII_PHY_PHYSR_LINK))
		return 0;
	else if (GMII_PHY_PHYSR_GIGA == (tmp16 & GMII_PHY_PHYSR_SMASK)) {
		mode |= AX_MEDIUM_GIGAMODE | AX_MEDIUM_EN_125MHZ;
		if (dev->net->mtu > 1500)
			mode |= AX_MEDIUM_JUMBO_EN;

		if (link_sts & AX_USB_SS)
			memcpy(tmp, &AX88179_BULKIN_SIZE[0], 5);
		else if (link_sts & AX_USB_HS)
			memcpy(tmp, &AX88179_BULKIN_SIZE[1], 5);
		else
			memcpy(tmp, &AX88179_BULKIN_SIZE[3], 5);
	} else if (GMII_PHY_PHYSR_100 == (tmp16 & GMII_PHY_PHYSR_SMASK)) {
		mode |= AX_MEDIUM_PS;	/* Bit 9 : PS */
		if (link_sts & (AX_USB_SS | AX_USB_HS))
			memcpy(tmp, &AX88179_BULKIN_SIZE[2], 5);
		else
			memcpy(tmp, &AX88179_BULKIN_SIZE[3], 5);
	} else
		memcpy(tmp, &AX88179_BULKIN_SIZE[3], 5);

	if (bsize != 0) {
		if (bsize > 24)
			bsize = 24;
		tmp[3] = (u8)bsize;
	}

	if (ifg != 0) {
		if (ifg > 255)
			ifg = 255;
		tmp[4] = (u8)ifg;
	}

	/* RX bulk configuration */
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_BULKIN_QCTRL, 5, 5, tmp);

	if (tmp16 & GMII_PHY_PHYSR_FULL)
		mode |= AX_MEDIUM_FULL_DUPLEX;	/* Bit 1 : FD */

	dev->rx_urb_size = (1024 * (tmp[3] + 2));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_info(dev->net, "Write medium type: 0x%04x\n", mode);
#else
		devinfo(dev, "Write medium type: 0x%04x\n", mode);
#endif
	/* Configure default medium type => giga */
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			  2, 2, &mode);
	mii_check_media(&dev->mii, 1, 1);

	return 0;
}

static int ax88179_reset(struct usbnet *dev)
{
	void *buf;
	u16 *tmp16;
	u8 *tmp;
	struct ax88179_data *ax179_data = (struct ax88179_data *) dev->data;
	buf = kmalloc(6, GFP_KERNEL);

	tmp16 = (u16 *)buf;
	tmp = (u8 *)buf;

	/* Power up ethernet PHY */
	*tmp16 = 0;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, tmp16);
	*tmp16 = AX_PHYPWR_RSTCTL_IPRL;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PHYPWR_RSTCTL, 2, 2, tmp16);
	msleep(200);

	*tmp = AX_CLK_SELECT_ACS | AX_CLK_SELECT_BCS;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_CLK_SELECT, 1, 1, tmp);
	msleep(100);

	/* Ethernet PHY Auto Detach*/
	ax88179_AutoDetach(dev, 0);

	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_NODE_ID, ETH_ALEN, ETH_ALEN,
			 dev->net->dev_addr, 0);
	memcpy(dev->net->perm_addr, dev->net->dev_addr, ETH_ALEN);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	netdev_dbg(dev->net, "MAC [%02x-%02x-%02x-%02x-%02x-%02x]\n",
	dev->net->dev_addr[0], dev->net->dev_addr[1],
	dev->net->dev_addr[2], dev->net->dev_addr[3],
	dev->net->dev_addr[4], dev->net->dev_addr[5]);
#else
	devdbg(dev, "MAC [%02x-%02x-%02x-%02x-%02x-%02x]\n",
	dev->net->dev_addr[0], dev->net->dev_addr[1],
	dev->net->dev_addr[2], dev->net->dev_addr[3],
	dev->net->dev_addr[4], dev->net->dev_addr[5]);
#endif

	/* RX bulk configuration */
	memcpy(tmp, &AX88179_BULKIN_SIZE[0], 5);
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RX_BULKIN_QCTRL, 5, 5, tmp);

	dev->rx_urb_size = 1024 * 20;

	tmp[0] = 0x34;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PAUSE_WATERLVL_LOW, 1, 1, tmp);

	tmp[0] = 0x52;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_PAUSE_WATERLVL_HIGH,
			  1, 1, tmp);

	dev->net->features |= NETIF_F_IP_CSUM;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 22)
	dev->net->features |= NETIF_F_IPV6_CSUM;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	dev->net->hw_features |= NETIF_F_IP_CSUM;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 22)
	dev->net->hw_features |= NETIF_F_IPV6_CSUM;
#endif
	dev->net->hw_features |= NETIF_F_SG | NETIF_F_TSO;
#endif

	/* Enable checksum offload */
	*tmp = AX_RXCOE_IP | AX_RXCOE_TCP | AX_RXCOE_UDP |
	       AX_RXCOE_TCPV6 | AX_RXCOE_UDPV6;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_RXCOE_CTL, 1, 1, tmp);

	*tmp = AX_TXCOE_IP | AX_TXCOE_TCP | AX_TXCOE_UDP |
	       AX_TXCOE_TCPV6 | AX_TXCOE_UDPV6;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_TXCOE_CTL, 1, 1, tmp);

	ax179_data->checksum |= AX_RX_CHECKSUM | AX_TX_CHECKSUM;

	/* Configure RX control register => start operation */
	*tmp16 = AX_RX_CTL_DROPCRCERR | AX_RX_CTL_IPE | AX_RX_CTL_START |
			AX_RX_CTL_AP | AX_RX_CTL_AMALL | AX_RX_CTL_AB;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, 0x0b, 2, 2, tmp16);


	*tmp = AX_MONITOR_MODE_PMETYPE | AX_MONITOR_MODE_PMEPOL |
						AX_MONITOR_MODE_RWMP;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MONITOR_MODE, 1, 1, tmp);

	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MONITOR_MODE, 1, 1, tmp, 0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	netdev_dbg(dev->net, "Monitor mode = 0x%02x\n", *tmp);
#else
	devdbg(dev, "Monitor mode = 0x%02x\n", *tmp);
#endif

	/* Configure default medium type => giga */
	*tmp16 = AX_MEDIUM_RECEIVE_EN	 | AX_MEDIUM_TXFLOW_CTRLEN |
		 AX_MEDIUM_RXFLOW_CTRLEN | AX_MEDIUM_ALWAYS_ONE	   |
		 AX_MEDIUM_FULL_DUPLEX	 | AX_MEDIUM_GIGAMODE;

	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			  2, 2, tmp16);

	ax88179_led_setting(dev);

	/* Restart autoneg */
	mii_nway_restart(&dev->mii);

	netif_carrier_off(dev->net);

	kfree(buf);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	netdev_dbg(dev->net, "mtu %d\n", dev->net->mtu);
#else
	devdbg(dev, "mtu %d\n", dev->net->mtu);
#endif

	return 0;

}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static int ax88179_stop(struct usbnet *dev)
{
	u16 tmp16;

	ax88179_read_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			 2, 2, &tmp16, 1);
	tmp16 &= ~AX_MEDIUM_RECEIVE_EN;
	ax88179_write_cmd(dev, AX_ACCESS_MAC, AX_MEDIUM_STATUS_MODE,
			  2, 2, &tmp16);
	return 0;
}
#endif

static const struct driver_info ax88179_info = {
	.description = "ASIX AX88179 USB 3.0 Gigibit Ethernet",
	.bind = ax88179_bind,
	.unbind = ax88179_unbind,
	.status = ax88179_status,
	.link_reset = ax88179_link_reset,
	.reset = ax88179_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	.stop = ax88179_stop,
#endif
	.flags = FLAG_ETHER | FLAG_FRAMING_AX,
	.rx_fixup = ax88179_rx_fixup,
	.tx_fixup = ax88179_tx_fixup,
};


static const struct driver_info ax88178a_info = {
	.description = "ASIX AX88178A USB 2.0 Gigibit Ethernet",
	.bind = ax88179_bind,
	.unbind = ax88179_unbind,
	.status = ax88179_status,
	.link_reset = ax88179_link_reset,
	.reset = ax88179_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	.stop = ax88179_stop,
#endif
	.flags = FLAG_ETHER | FLAG_FRAMING_AX,
	.rx_fixup = ax88179_rx_fixup,
	.tx_fixup = ax88179_tx_fixup,
};

static const struct driver_info sitecom_info = {
	.description = "Sitecom USB 3.0 to Gigabit Adapter",
	.bind = ax88179_bind,
	.unbind = ax88179_unbind,
	.status = ax88179_status,
	.link_reset = ax88179_link_reset,
	.reset = ax88179_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	.stop = ax88179_stop,
#endif
	.flags = FLAG_ETHER | FLAG_FRAMING_AX,
	.rx_fixup = ax88179_rx_fixup,
	.tx_fixup = ax88179_tx_fixup,
};

static const struct driver_info lenovo_info = {
	.description = "ThinkPad OneLinkDock USB GigaLAN",
	.bind = ax88179_bind,
	.unbind = ax88179_unbind,
	.status = ax88179_status,
	.link_reset = ax88179_link_reset,
	.reset = ax88179_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	.stop = ax88179_stop,
#endif
	.flags = FLAG_ETHER | FLAG_FRAMING_AX,
	.rx_fixup = ax88179_rx_fixup,
	.tx_fixup = ax88179_tx_fixup,
};

static const struct driver_info toshiba_info = {
	.description = "Toshiba USB 3.0 to Gigabit LAN Adapter",
	.bind = ax88179_bind,
	.unbind = ax88179_unbind,
	.status = ax88179_status,
	.link_reset = ax88179_link_reset,
	.reset = ax88179_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	.stop = ax88179_stop,
#endif
	.flags = FLAG_ETHER | FLAG_FRAMING_AX,
	.rx_fixup = ax88179_rx_fixup,
	.tx_fixup = ax88179_tx_fixup,
};

static const struct usb_device_id	products[] = {
{
	/* ASIX AX88179 10/100/1000 */
	USB_DEVICE(0x0b95, 0x1790),
	.driver_info = (unsigned long) &ax88179_info,
}, {
	/* ASIX AX88178A 10/100/1000 */
	USB_DEVICE(0x0b95, 0x178a),
	.driver_info = (unsigned long) &ax88178a_info,
}, {
	/* Sitecom USB 3.0 to Gigabit Adapter */
	USB_DEVICE(0x0df6, 0x0072),
	.driver_info = (unsigned long) &sitecom_info,
}, {
	/* ThinkPad OneLinkDock USB GigaLAN */
	USB_DEVICE(0x17ef, 0x304b),
	.driver_info = (unsigned long) &lenovo_info,
}, {
	/* Toshiba USB3.0 to Gigabit LAN Adapter */
	USB_DEVICE(0x0930, 0x0a13),
	.driver_info = (unsigned long) &toshiba_info,
},
	{ },		/* END */
=======
static int ax88178_link_reset(struct usbnet *dev)
{
	u16 mode;
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };
	struct asix_data *data = (struct asix_data *)&dev->data;
	u32 speed;

	netdev_dbg(dev->net, "ax88178_link_reset()\n");

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);
	mode = AX88178_MEDIUM_DEFAULT;
	speed = ethtool_cmd_speed(&ecmd);

	if (speed == SPEED_1000)
		mode |= AX_MEDIUM_GM;
	else if (speed == SPEED_100)
		mode |= AX_MEDIUM_PS;
	else
		mode &= ~(AX_MEDIUM_PS | AX_MEDIUM_GM);

	mode |= AX_MEDIUM_ENCK;

	if (ecmd.duplex == DUPLEX_FULL)
		mode |= AX_MEDIUM_FD;
	else
		mode &= ~AX_MEDIUM_FD;

	netdev_dbg(dev->net, "ax88178_link_reset() speed: %u duplex: %d setting mode to 0x%04x\n",
		   speed, ecmd.duplex, mode);

	asix_write_medium_mode(dev, mode);

	if (data->phymode == PHY_MODE_MARVELL && data->ledmode)
		marvell_led_status(dev, speed);

	return 0;
}

static void ax88178_set_mfb(struct usbnet *dev)
{
	u16 mfb = AX_RX_CTL_MFB_16384;
	u16 rxctl;
	u16 medium;
	int old_rx_urb_size = dev->rx_urb_size;

	if (dev->hard_mtu < 2048) {
		dev->rx_urb_size = 2048;
		mfb = AX_RX_CTL_MFB_2048;
	} else if (dev->hard_mtu < 4096) {
		dev->rx_urb_size = 4096;
		mfb = AX_RX_CTL_MFB_4096;
	} else if (dev->hard_mtu < 8192) {
		dev->rx_urb_size = 8192;
		mfb = AX_RX_CTL_MFB_8192;
	} else if (dev->hard_mtu < 16384) {
		dev->rx_urb_size = 16384;
		mfb = AX_RX_CTL_MFB_16384;
	}

	rxctl = asix_read_rx_ctl(dev);
	asix_write_rx_ctl(dev, (rxctl & ~AX_RX_CTL_MFB_16384) | mfb);

	medium = asix_read_medium_status(dev);
	if (dev->net->mtu > 1500)
		medium |= AX_MEDIUM_JFE;
	else
		medium &= ~AX_MEDIUM_JFE;
	asix_write_medium_mode(dev, medium);

	if (dev->rx_urb_size > old_rx_urb_size)
		usbnet_unlink_rx_urbs(dev);
}

static int ax88178_change_mtu(struct net_device *net, int new_mtu)
{
	struct usbnet *dev = netdev_priv(net);
	int ll_mtu = new_mtu + net->hard_header_len + 4;

	netdev_dbg(dev->net, "ax88178_change_mtu() new_mtu=%d\n", new_mtu);

	if (new_mtu <= 0 || ll_mtu > 16384)
		return -EINVAL;

	if ((ll_mtu % dev->maxpacket) == 0)
		return -EDOM;

	net->mtu = new_mtu;
	dev->hard_mtu = net->mtu + net->hard_header_len;
	ax88178_set_mfb(dev);

	return 0;
}

static const struct net_device_ops ax88178_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= usbnet_start_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_set_mac_address 	= asix_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_multicast_list = asix_set_multicast,
	.ndo_do_ioctl 		= asix_ioctl,
	.ndo_change_mtu 	= ax88178_change_mtu,
};

static int ax88178_bind(struct usbnet *dev, struct usb_interface *intf)
{
	struct asix_data *data = (struct asix_data *)&dev->data;
	int ret;
	u8 buf[ETH_ALEN];
	__le16 eeprom;
	u8 status;
	int gpio0 = 0;
	u32 phyid;

	usbnet_get_endpoints(dev,intf);

	asix_read_cmd(dev, AX_CMD_READ_GPIOS, 0, 0, 1, &status);
	dbg("GPIO Status: 0x%04x", status);

	asix_write_cmd(dev, AX_CMD_WRITE_ENABLE, 0, 0, 0, NULL);
	asix_read_cmd(dev, AX_CMD_READ_EEPROM, 0x0017, 0, 2, &eeprom);
	asix_write_cmd(dev, AX_CMD_WRITE_DISABLE, 0, 0, 0, NULL);

	dbg("EEPROM index 0x17 is 0x%04x", eeprom);

	if (eeprom == cpu_to_le16(0xffff)) {
		data->phymode = PHY_MODE_MARVELL;
		data->ledmode = 0;
		gpio0 = 1;
	} else {
		data->phymode = le16_to_cpu(eeprom) & 7;
		data->ledmode = le16_to_cpu(eeprom) >> 8;
		gpio0 = (le16_to_cpu(eeprom) & 0x80) ? 0 : 1;
	}
	dbg("GPIO0: %d, PhyMode: %d", gpio0, data->phymode);

	asix_write_gpio(dev, AX_GPIO_RSE | AX_GPIO_GPO_1 | AX_GPIO_GPO1EN, 40);
	if ((le16_to_cpu(eeprom) >> 8) != 1) {
		asix_write_gpio(dev, 0x003c, 30);
		asix_write_gpio(dev, 0x001c, 300);
		asix_write_gpio(dev, 0x003c, 30);
	} else {
		dbg("gpio phymode == 1 path");
		asix_write_gpio(dev, AX_GPIO_GPO1EN, 30);
		asix_write_gpio(dev, AX_GPIO_GPO1EN | AX_GPIO_GPO_1, 30);
	}

	asix_sw_reset(dev, 0);
	msleep(150);

	asix_sw_reset(dev, AX_SWRESET_PRL | AX_SWRESET_IPPD);
	msleep(150);

	asix_write_rx_ctl(dev, 0);

	/* Get the MAC address */
	if ((ret = asix_read_cmd(dev, AX_CMD_READ_NODE_ID,
				0, 0, ETH_ALEN, buf)) < 0) {
		dbg("Failed to read MAC address: %d", ret);
		goto out;
	}
	memcpy(dev->net->dev_addr, buf, ETH_ALEN);

	/* Initialize MII structure */
	dev->mii.dev = dev->net;
	dev->mii.mdio_read = asix_mdio_read;
	dev->mii.mdio_write = asix_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0xff;
	dev->mii.supports_gmii = 1;
	dev->mii.phy_id = asix_get_phy_addr(dev);

	dev->net->netdev_ops = &ax88178_netdev_ops;
	dev->net->ethtool_ops = &ax88178_ethtool_ops;

	phyid = asix_get_phyid(dev);
	dbg("PHYID=0x%08x", phyid);

	if (data->phymode == PHY_MODE_MARVELL) {
		marvell_phy_init(dev);
		msleep(60);
	}

	asix_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR,
			BMCR_RESET | BMCR_ANENABLE);
	asix_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE,
			ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	asix_mdio_write(dev->net, dev->mii.phy_id, MII_CTRL1000,
			ADVERTISE_1000FULL);

	mii_nway_restart(&dev->mii);

	if ((ret = asix_write_medium_mode(dev, AX88178_MEDIUM_DEFAULT)) < 0)
		goto out;

	if ((ret = asix_write_rx_ctl(dev, AX_DEFAULT_RX_CTL)) < 0)
		goto out;

	/* Asix framing packs multiple eth frames into a 2K usb bulk transfer */
	if (dev->driver_info->flags & FLAG_FRAMING_AX) {
		/* hard_mtu  is still the default - the device does not support
		   jumbo eth frames */
		dev->rx_urb_size = 2048;
	}
	return 0;

out:
	return ret;
}

static const struct driver_info ax8817x_info = {
	.description = "ASIX AX8817x USB 2.0 Ethernet",
	.bind = ax88172_bind,
	.status = asix_status,
	.link_reset = ax88172_link_reset,
	.reset = ax88172_link_reset,
	.flags =  FLAG_ETHER | FLAG_LINK_INTR,
	.data = 0x00130103,
};

static const struct driver_info dlink_dub_e100_info = {
	.description = "DLink DUB-E100 USB Ethernet",
	.bind = ax88172_bind,
	.status = asix_status,
	.link_reset = ax88172_link_reset,
	.reset = ax88172_link_reset,
	.flags =  FLAG_ETHER | FLAG_LINK_INTR,
	.data = 0x009f9d9f,
};

static const struct driver_info netgear_fa120_info = {
	.description = "Netgear FA-120 USB Ethernet",
	.bind = ax88172_bind,
	.status = asix_status,
	.link_reset = ax88172_link_reset,
	.reset = ax88172_link_reset,
	.flags =  FLAG_ETHER | FLAG_LINK_INTR,
	.data = 0x00130103,
};

static const struct driver_info hawking_uf200_info = {
	.description = "Hawking UF200 USB Ethernet",
	.bind = ax88172_bind,
	.status = asix_status,
	.link_reset = ax88172_link_reset,
	.reset = ax88172_link_reset,
	.flags =  FLAG_ETHER | FLAG_LINK_INTR,
	.data = 0x001f1d1f,
};

static const struct driver_info ax88772_info = {
	.description = "ASIX AX88772 USB 2.0 Ethernet",
	.bind = ax88772_bind,
	.status = asix_status,
	.link_reset = ax88772_link_reset,
	.reset = ax88772_link_reset,
	.flags = FLAG_ETHER | FLAG_FRAMING_AX | FLAG_LINK_INTR,
	.rx_fixup = asix_rx_fixup,
	.tx_fixup = asix_tx_fixup,
};

static const struct driver_info ax88178_info = {
	.description = "ASIX AX88178 USB 2.0 Ethernet",
	.bind = ax88178_bind,
	.status = asix_status,
	.link_reset = ax88178_link_reset,
	.reset = ax88178_link_reset,
	.flags = FLAG_ETHER | FLAG_FRAMING_AX | FLAG_LINK_INTR,
	.rx_fixup = asix_rx_fixup,
	.tx_fixup = asix_tx_fixup,
};

static const struct usb_device_id	products [] = {
{
	// Linksys USB200M
	USB_DEVICE (0x077b, 0x2226),
	.driver_info =	(unsigned long) &ax8817x_info,
}, {
	// Netgear FA120
	USB_DEVICE (0x0846, 0x1040),
	.driver_info =  (unsigned long) &netgear_fa120_info,
}, {
	// DLink DUB-E100
	USB_DEVICE (0x2001, 0x1a00),
	.driver_info =  (unsigned long) &dlink_dub_e100_info,
}, {
	// Intellinet, ST Lab USB Ethernet
	USB_DEVICE (0x0b95, 0x1720),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// Hawking UF200, TrendNet TU2-ET100
	USB_DEVICE (0x07b8, 0x420a),
	.driver_info =  (unsigned long) &hawking_uf200_info,
}, {
	// Billionton Systems, USB2AR
	USB_DEVICE (0x08dd, 0x90ff),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// ATEN UC210T
	USB_DEVICE (0x0557, 0x2009),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// Buffalo LUA-U2-KTX
	USB_DEVICE (0x0411, 0x003d),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// Buffalo LUA-U2-GT 10/100/1000
	USB_DEVICE (0x0411, 0x006e),
	.driver_info =  (unsigned long) &ax88178_info,
}, {
	// Sitecom LN-029 "USB 2.0 10/100 Ethernet adapter"
	USB_DEVICE (0x6189, 0x182d),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// corega FEther USB2-TX
	USB_DEVICE (0x07aa, 0x0017),
	.driver_info =  (unsigned long) &ax8817x_info,
}, {
	// Surecom EP-1427X-2
	USB_DEVICE (0x1189, 0x0893),
	.driver_info = (unsigned long) &ax8817x_info,
}, {
	// goodway corp usb gwusb2e
	USB_DEVICE (0x1631, 0x6200),
	.driver_info = (unsigned long) &ax8817x_info,
}, {
	// JVC MP-PRX1 Port Replicator
	USB_DEVICE (0x04f1, 0x3008),
	.driver_info = (unsigned long) &ax8817x_info,
}, {
	// ASIX AX88772B 10/100
	USB_DEVICE (0x0b95, 0x772b),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// ASIX AX88772 10/100
	USB_DEVICE (0x0b95, 0x7720),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// ASIX AX88178 10/100/1000
	USB_DEVICE (0x0b95, 0x1780),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// Logitec LAN-GTJ/U2A
	USB_DEVICE (0x0789, 0x0160),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// Linksys USB200M Rev 2
	USB_DEVICE (0x13b1, 0x0018),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// 0Q0 cable ethernet
	USB_DEVICE (0x1557, 0x7720),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// DLink DUB-E100 H/W Ver B1
	USB_DEVICE (0x07d1, 0x3c05),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// DLink DUB-E100 H/W Ver B1 Alternate
	USB_DEVICE (0x2001, 0x3c05),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// Linksys USB1000
	USB_DEVICE (0x1737, 0x0039),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// IO-DATA ETG-US2
	USB_DEVICE (0x04bb, 0x0930),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// Belkin F5D5055
	USB_DEVICE(0x050d, 0x5055),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// Apple USB Ethernet Adapter
	USB_DEVICE(0x05ac, 0x1402),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// Cables-to-Go USB Ethernet Adapter
	USB_DEVICE(0x0b95, 0x772a),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// ABOCOM for pci
	USB_DEVICE(0x14ea, 0xab11),
	.driver_info = (unsigned long) &ax88178_info,
}, {
	// ASIX 88772a
	USB_DEVICE(0x0db0, 0xa877),
	.driver_info = (unsigned long) &ax88772_info,
}, {
	// Asus USB Ethernet Adapter
	USB_DEVICE (0x0b95, 0x7e2b),
	.driver_info = (unsigned long) &ax88772_info,
},
	{ },		// END
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
};
MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver asix_driver = {
<<<<<<< HEAD
	.name =		"ax88179_178a",
	.id_table =	products,
	.probe =	usbnet_probe,
	.suspend =	ax88179_suspend,
	.resume =	ax88179_resume,
	.disconnect =	usbnet_disconnect,
};


static int __init asix_init(void)
{
	return usb_register(&asix_driver);
=======
	.name =		"asix",
	.id_table =	products,
	.probe =	usbnet_probe,
	.suspend =	usbnet_suspend,
	.resume =	usbnet_resume,
	.disconnect =	usbnet_disconnect,
	.supports_autosuspend = 1,
};

static int __init asix_init(void)
{
 	return usb_register(&asix_driver);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
}
module_init(asix_init);

static void __exit asix_exit(void)
{
<<<<<<< HEAD
	usb_deregister(&asix_driver);
=======
 	usb_deregister(&asix_driver);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
}
module_exit(asix_exit);

MODULE_AUTHOR("David Hollis");
<<<<<<< HEAD
MODULE_DESCRIPTION("ASIX AX88179_178A based USB 2.0/3.0 Gigabit Ethernet Devices");
=======
MODULE_DESCRIPTION("ASIX AX8817X based USB 2.0 Ethernet Devices");
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
MODULE_LICENSE("GPL");

