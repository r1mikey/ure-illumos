/*      $OpenBSD: if_ure.c,v 1.37 2025/06/04 00:06:17 jsg Exp $ */
/*-
 * Copyright (c) 2015, 2016, 2019 Kevin Lo <kevlo@openbsd.org>
 * Copyright (c) 2020 Jonathon Fletcher <jonathon.fletcher@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*-
 * Copyright (c) 2015-2016 Kevin Lo <kevlo@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 */

/*
 * Copyright 2026 Michael van der Westhuizen
 */

/*
 * Realtek RTL8152/8153/8153B/8156/8156B/8157 USB Ethernet Driver
 *
 * This is a standalone illumos USB Ethernet driver for the Realtek
 * RTL8152/8153 family of USB-to-Ethernet controllers.  It registers
 * directly with the MAC (GLDv3) framework and the USBA framework,
 * without using the usbgem USB NIC framework.
 *
 * The driver supports:
 *   - RTL8152  (USB 2.0, 10/100 Mbps)
 *   - RTL8153  (USB 3.0, 10/100/1000 Mbps)
 *   - RTL8153B (USB 3.0, 10/100/1000 Mbps)
 *   - RTL8156  (USB 3.0, 10/100/1000/2500 Mbps)
 *   - RTL8156B (USB 3.0, 10/100/1000/2500 Mbps)
 *   - RTL8157  (USB 3.0, 10/100/1000/2500/5000 Mbps)
 *
 * Key features:
 *   - RX aggregation (multiple packets per USB bulk IN transfer)
 *   - TX aggregation (multiple packets per USB bulk OUT transfer)
 *   - IPv4/TCP/UDP hardware checksum offload (RX and TX)
 *   - Hardware VLAN tag insertion/stripping
 *   - Multicast hash filter (64-bit, CRC32-BE)
 *
 * Reference implementations:
 *   - OpenBSD if_ure.c (v1.37, Kevin Lo, Jonathon Fletcher)
 *   - FreeBSD if_ure.c (Kevin Lo)
 *   - FreeBSD spurious link-down BMSR workaround (PR 252165)
 *
 * All register access is via USB vendor control transfers.  The chip
 * has two register spaces: PLA (Protocol Logic Adapter) at index
 * 0x0100, and USB controller at index 0x0000.  Sub-DWORD register
 * writes use byte-enable bits encoded in the USB index field.
 */

#include <sys/types.h>
#include <sys/conf.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/modctl.h>
#include <sys/strsun.h>
#include <sys/byteorder.h>
#include <sys/ethernet.h>
#include <sys/mac_provider.h>
#include <sys/mac_ether.h>
#include <usb/usba/usbai_version.h>
#include <sys/usb/usba.h>
#include <sys/usb/usba/usba_types.h>
#include <sys/crc32.h>
#include <sys/vlan.h>
#include <sys/strsubr.h>
#include <inet/ip.h>

#include "urereg.h"
#include "ure.h"

/* Debug logging — set to 1 to enable verbose attach/detach logging */
#define	URE_DEBUG	0

#if URE_DEBUG
#define	URE_DPRINTF(sc, fmt, ...)	\
	dev_err((sc)->ure_dip, CE_CONT, "?" fmt "\n", ##__VA_ARGS__)
#else
#define	URE_DPRINTF(sc, fmt, ...)
#endif

/* Forward declarations */
static int	ure_attach(dev_info_t *, ddi_attach_cmd_t);
static int	ure_detach(dev_info_t *, ddi_detach_cmd_t);

static int	ure_m_stat(void *, uint_t, uint64_t *);
static int	ure_m_start(void *);
static void	ure_m_stop(void *);
static int	ure_m_promisc(void *, boolean_t);
static int	ure_m_multicst(void *, boolean_t, const uint8_t *);
static int	ure_m_unicst(void *, const uint8_t *);
static mblk_t	*ure_m_tx(void *, mblk_t *);
static int	ure_m_getprop(void *, const char *, mac_prop_id_t,
		    uint_t, void *);
static int	ure_m_setprop(void *, const char *, mac_prop_id_t,
		    uint_t, const void *);
static void	ure_m_propinfo(void *, const char *, mac_prop_id_t,
		    mac_prop_info_handle_t);
static boolean_t	ure_m_getcapab(void *, mac_capab_t, void *);

static void	ure_rx_start(ure_softc_t *);
static void	ure_rx_cb(usb_pipe_handle_t, usb_bulk_req_t *);
static void	ure_tx_cb(usb_pipe_handle_t, usb_bulk_req_t *);

static void	ure_chip_init(ure_softc_t *);
static void	ure_rtl8152_init(ure_softc_t *);
static int	ure_rtl8153_init(ure_softc_t *);
static int	ure_rtl8153b_init(ure_softc_t *);
static int	ure_rtl8157_init(ure_softc_t *);
static int	ure_rtl8152_nic_reset(ure_softc_t *);
static int	ure_rtl8153_nic_reset(ure_softc_t *);
static uint16_t	ure_rtl8153_phy_status(ure_softc_t *, int);
static int	ure_wait_for_flash(ure_softc_t *);
static void	ure_reset_bmu(ure_softc_t *);
static void	ure_disable_teredo(ure_softc_t *);
static void	ure_reset(ure_softc_t *);
static void	ure_rxvlan(ure_softc_t *);
static void	ure_set_rx_filter(ure_softc_t *);
static void	ure_ifmedia_init(ure_softc_t *);
static int	ure_get_link_status(ure_softc_t *);
static void	ure_link_check(void *);

static int	ure_disconnect_cb(dev_info_t *);
static int	ure_reconnect_cb(dev_info_t *);
static int	ure_open_pipes(ure_softc_t *);
static void	ure_close_pipes(ure_softc_t *);

/* Soft state */
static void	*ure_statep;

/*
 * Get the actual USB connection speed for this device.
 * Returns one of USBA_LOW_SPEED_DEV, USBA_FULL_SPEED_DEV,
 * USBA_HIGH_SPEED_DEV, or USBA_SUPER_SPEED_DEV.
 */
static usb_port_status_t
ure_dev_speed(ure_softc_t *sc)
{
	usba_device_t *ud = usba_get_usba_device(sc->ure_dip);
	return (ud->usb_port_status);
}

#if 0
/*
 * Device ID table.  Covers all known RTL8152/8153/8156/8157 USB NICs.
 * Ported from OpenBSD if_ure.c device table.
 */
static const struct {
	uint16_t	vid;
	uint16_t	pid;
} ure_devs[] = {
	{ 0x0b95, 0x8156 },	/* ASUS RTL8156 */
	{ 0x050d, 0x0128 },	/* Belkin RTL8152B */
	{ 0x050d, 0x0129 },	/* Belkin RTL8153 */
	{ 0x13b1, 0x0041 },	/* Cisco Linksys USB3GIGV1 */
	{ 0x04b4, 0x3610 },	/* Clevo RTL8153B */
	{ 0x2001, 0xb301 },	/* D-Link RTL8153 (1) */
	{ 0x2001, 0xb328 },	/* D-Link RTL8153 (2) */
	{ 0x056e, 0x4010 },	/* Elecom RTL8153B */
	{ 0x056e, 0x401a },	/* Elecom RTL8156B */
	{ 0x17ef, 0x304f },	/* Lenovo Dock Ethernet */
	{ 0x17ef, 0x3054 },	/* Lenovo OneLink */
	{ 0x17ef, 0x3057 },	/* Lenovo OneLinkPlus */
	{ 0x17ef, 0x3062 },	/* Lenovo OneLinkPro */
	{ 0x17ef, 0x3069 },	/* Lenovo RTL8153 (1) */
	{ 0x17ef, 0x7205 },	/* Lenovo RTL8153 (2) */
	{ 0x17ef, 0x720a },	/* Lenovo RTL8153 (3) */
	{ 0x17ef, 0x720c },	/* Lenovo Tablet Dock */
	{ 0x17ef, 0x3082 },	/* Lenovo TB3 Dock */
	{ 0x17ef, 0x3098 },	/* Lenovo TB3 Dock Gen2 */
	{ 0x17ef, 0xa359 },	/* Lenovo TB3 GFX Dock */
	{ 0x17ef, 0xa387 },	/* Lenovo USB-C Dock Gen2 */
	{ 0x17ef, 0x3049 },	/* Lenovo WiGig Dock */
	{ 0x043e, 0x3068 },	/* LG RTL8153 */
	{ 0x043e, 0x3091 },	/* LG RTL8153B */
	{ 0x045e, 0x07ab },	/* Microsoft Dock Ethernet */
	{ 0x045e, 0x07c6 },	/* Microsoft Dock Ethernet 2 */
	{ 0x045e, 0x0927 },	/* Microsoft Surface Ethernet */
	{ 0x045e, 0x0c30 },	/* Microsoft Win Dev Ethernet */
	{ 0x0955, 0x09ff },	/* Nvidia Tegra Ethernet */
	{ 0x2b04, 0x0132 },	/* Pioneer DJ RTL8152B */
	{ 0x2b04, 0x013b },	/* Pioneer DJ RTL8153B */
	{ 0x0bda, 0x8050 },	/* Realtek RTL8152 */
	{ 0x0bda, 0x8152 },	/* Realtek RTL8152B */
	{ 0x0bda, 0x8153 },	/* Realtek RTL8153 */
	{ 0x0bda, 0x8156 },	/* Realtek RTL8156 */
	{ 0x0bda, 0x8157 },	/* Realtek RTL8157 */
	{ 0x04e8, 0xa101 },	/* Samsung RTL8153 */
	{ 0x0930, 0x0a13 },	/* Toshiba RTL8153B */
	{ 0x2357, 0x0601 },	/* TP-Link EU300 */
	{ 0x2357, 0x0602 },	/* TP-Link RTL8152B (1) */
	{ 0x2357, 0x0603 },	/* TP-Link RTL8152B (2) */
	{ 0x20f4, 0xe05a },	/* TRENDnet RTL8156 */
	{ 0x0fce, 0x7a03 },	/* TTL RTL8153 */
	{ 0x14cd, 0x8158 },	/* Twinhead RTL8153B */
	{ 0x2717, 0xff40 },	/* Xiaomi RTL8152B */
};

#define	URE_NDEVS	(sizeof (ure_devs) / sizeof (ure_devs[0]))
#endif

/* USB event callbacks */
static usb_event_t ure_events = {
	.disconnect_event_handler = ure_disconnect_cb,
	.reconnect_event_handler = ure_reconnect_cb,
	.pre_suspend_event_handler = NULL,
	.post_resume_event_handler = NULL,
};

/* MAC callbacks */
static mac_callbacks_t ure_mac_callbacks = {
	MC_GETCAPAB | MC_SETPROP | MC_GETPROP | MC_PROPINFO,
	ure_m_stat,
	ure_m_start,
	ure_m_stop,
	ure_m_promisc,
	ure_m_multicst,
	ure_m_unicst,
	ure_m_tx,
	NULL,		/* mc_reserved */
	NULL,		/* mc_ioctl */
	ure_m_getcapab,
	NULL,		/* mc_open */
	NULL,		/* mc_close */
	ure_m_setprop,
	ure_m_getprop,
	ure_m_propinfo,
};

DDI_DEFINE_STREAM_OPS(
	/* XXname */		ure_dev_ops,
	/* XXidentify */	nulldev,
	/* XXprobe */		nulldev,
	/* XXattach */		ure_attach,
	/* XXdetach */		ure_detach,
	/* XXreset */		nodev,
	/* XXgetinfo */		NULL,
	/* XXflag */		D_MP,
	/* XXstream_tab */	NULL,
	/* XXquiesce */		ddi_quiesce_not_needed
);

static struct modldrv ure_modldrv = {
	&mod_driverops,
	"Realtek RTL815[2367] USB Ethernet",
	&ure_dev_ops,
};

static struct modlinkage ure_modlinkage = {
	MODREV_1,
	{ &ure_modldrv, NULL }
};

int
_init(void)
{
	major_t major;
	int err;

	if ((err = ddi_soft_state_init(&ure_statep,
	    sizeof (ure_softc_t), 1)) != 0)
		return (err);

	if ((major = ddi_name_to_major("ure")) == DDI_MAJOR_T_NONE) {
		ddi_soft_state_fini(&ure_statep);
		return (DDI_FAILURE);
	}

	mac_init_ops(&ure_dev_ops, "ure");

	if ((err = mod_install(&ure_modlinkage)) != 0) {
		mac_fini_ops(&ure_dev_ops);
		ddi_soft_state_fini(&ure_statep);
	}

	return (err);
}

int
_fini(void)
{
	int err;

	if ((err = mod_remove(&ure_modlinkage)) != 0)
		return (err);

	mac_fini_ops(&ure_dev_ops);
	ddi_soft_state_fini(&ure_statep);
	return (0);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&ure_modlinkage, modinfop));
}

/* ================================================================ */
/* Register access primitives                                       */
/* ================================================================ */

/*
 * Low-level USB vendor control transfer.
 */
static int
ure_ctl(ure_softc_t *sc, uint8_t rw, uint16_t val, uint16_t index,
    void *buf, int len)
{
	usb_ctrl_setup_t setup;
	usb_cr_t cr;
	usb_cb_flags_t cb_flags;
	mblk_t *data = NULL;
	int ret;

	if (sc->ure_gone)
		return (USB_FAILURE);

	bzero(&setup, sizeof (setup));
	if (rw == URE_CTL_WRITE)
		setup.bmRequestType = USB_DEV_REQ_HOST_TO_DEV |
		    USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV;
	else
		setup.bmRequestType = USB_DEV_REQ_DEV_TO_HOST |
		    USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV;
	setup.bRequest = USB_REQ_SET_ADDRESS;
	setup.wValue = val;
	setup.wIndex = index;
	setup.wLength = (uint16_t)len;
	setup.attrs = USB_ATTRS_NONE;

	if (rw == URE_CTL_WRITE && buf != NULL && len > 0) {
		data = allocb(len, BPRI_MED);
		if (data == NULL)
			return (USB_FAILURE);
		bcopy(buf, data->b_wptr, len);
		data->b_wptr += len;
	}

	ret = usb_pipe_ctrl_xfer_wait(sc->ure_def_pipe,
	    &setup, (rw == URE_CTL_READ) ? &data : &data,
	    &cr, &cb_flags, 0);

	if (ret == USB_SUCCESS && rw == URE_CTL_READ &&
	    data != NULL && buf != NULL) {
		int actual = MBLKL(data);
		if (actual > len)
			actual = len;
		bcopy(data->b_rptr, buf, actual);
	}

	if (data != NULL)
		freemsg(data);

	return (ret);
}

static int
ure_read_mem(ure_softc_t *sc, uint16_t addr, uint16_t index,
    void *buf, int len)
{
	return (ure_ctl(sc, URE_CTL_READ, addr, index, buf, len));
}

static int
ure_write_mem(ure_softc_t *sc, uint16_t addr, uint16_t index,
    void *buf, int len)
{
	return (ure_ctl(sc, URE_CTL_WRITE, addr, index, buf, len));
}

static uint8_t
ure_read_1(ure_softc_t *sc, uint16_t reg, uint16_t index)
{
	uint32_t val;
	uint8_t temp[4];
	uint8_t shift;

	shift = (reg & 3) << 3;
	reg &= ~3;

	(void) ure_read_mem(sc, reg, index, temp, 4);
	val = LE_32(*(uint32_t *)(void *)temp);
	val >>= shift;

	return ((uint8_t)(val & 0xff));
}

static uint16_t
ure_read_2(ure_softc_t *sc, uint16_t reg, uint16_t index)
{
	uint32_t val;
	uint8_t temp[4];
	uint8_t shift;

	shift = (reg & 2) << 3;
	reg &= ~3;

	(void) ure_read_mem(sc, reg, index, temp, 4);
	val = LE_32(*(uint32_t *)(void *)temp);
	val >>= shift;

	return ((uint16_t)(val & 0xffff));
}

static uint32_t
ure_read_4(ure_softc_t *sc, uint16_t reg, uint16_t index)
{
	uint8_t temp[4];

	(void) ure_read_mem(sc, reg, index, temp, 4);
	return (LE_32(*(uint32_t *)(void *)temp));
}

static int
ure_write_1(ure_softc_t *sc, uint16_t reg, uint16_t index,
    uint32_t val)
{
	uint16_t byen;
	uint8_t temp[4];
	uint8_t shift;

	byen = URE_BYTE_EN_BYTE;
	shift = reg & 3;
	val &= 0xff;

	if (reg & 3) {
		byen <<= shift;
		val <<= (shift << 3);
		reg &= ~3;
	}

	*(uint32_t *)(void *)temp = LE_32(val);
	return (ure_write_mem(sc, reg, index | byen, temp, 4));
}

static int
ure_write_2(ure_softc_t *sc, uint16_t reg, uint16_t index,
    uint32_t val)
{
	uint16_t byen;
	uint8_t temp[4];
	uint8_t shift;

	byen = URE_BYTE_EN_WORD;
	shift = reg & 2;
	val &= 0xffff;

	if (reg & 2) {
		byen <<= shift;
		val <<= (shift << 3);
		reg &= ~3;
	}

	*(uint32_t *)(void *)temp = LE_32(val);
	return (ure_write_mem(sc, reg, index | byen, temp, 4));
}

static int
ure_write_4(ure_softc_t *sc, uint16_t reg, uint16_t index,
    uint32_t val)
{
	uint8_t temp[4];

	*(uint32_t *)(void *)temp = LE_32(val);
	return (ure_write_mem(sc, reg,
	    index | URE_BYTE_EN_DWORD, temp, 4));
}

/* ================================================================ */
/* PHY access via OCP (On-Chip Protocol)                            */
/* ================================================================ */

static uint16_t
ure_ocp_reg_read(ure_softc_t *sc, uint16_t addr)
{
	uint16_t reg;

	ure_write_2(sc, URE_PLA_OCP_GPHY_BASE, URE_MCU_TYPE_PLA,
	    addr & 0xf000);
	reg = (addr & 0x0fff) | 0xb000;

	return (ure_read_2(sc, reg, URE_MCU_TYPE_PLA));
}

static void
ure_ocp_reg_write(ure_softc_t *sc, uint16_t addr, uint16_t data)
{
	uint16_t reg;

	ure_write_2(sc, URE_PLA_OCP_GPHY_BASE, URE_MCU_TYPE_PLA,
	    addr & 0xf000);
	reg = (addr & 0x0fff) | 0xb000;

	ure_write_2(sc, reg, URE_MCU_TYPE_PLA, data);
}

/*
 * RTL8157 uses a separate TGPHY interface for PHY access.
 */
static uint16_t
ure_rtl8157_ocp_reg_read(ure_softc_t *sc, uint16_t addr)
{
	int i;

	for (i = 0; i < 10; i++) {
		if (!(ure_read_2(sc, URE_USB_TGPHY_CMD,
		    URE_MCU_TYPE_USB) & URE_TGPHY_CMD_BUSY))
			break;
		delay(drv_usectohz(1000));
	}
	if (i == 10) {
		dev_err(sc->ure_dip, CE_WARN,
		    "PHY read timeout (pre)");
		return (0xffff);
	}

	ure_write_2(sc, URE_USB_TGPHY_ADDR, URE_MCU_TYPE_USB,
	    addr);
	ure_write_2(sc, URE_USB_TGPHY_CMD, URE_MCU_TYPE_USB,
	    URE_TGPHY_CMD_BUSY);

	for (i = 0; i < 10; i++) {
		if (!(ure_read_2(sc, URE_USB_TGPHY_CMD,
		    URE_MCU_TYPE_USB) & URE_TGPHY_CMD_BUSY))
			break;
		delay(drv_usectohz(1000));
	}
	if (i == 10) {
		dev_err(sc->ure_dip, CE_WARN,
		    "PHY read timeout (post)");
		return (0xffff);
	}

	return (ure_read_2(sc, URE_USB_TGPHY_DATA,
	    URE_MCU_TYPE_USB));
}

static void
ure_rtl8157_ocp_reg_write(ure_softc_t *sc, uint16_t addr,
    uint16_t data)
{
	int i;

	ure_write_2(sc, URE_USB_TGPHY_DATA, URE_MCU_TYPE_USB,
	    data);
	ure_write_2(sc, URE_USB_TGPHY_ADDR, URE_MCU_TYPE_USB,
	    addr);
	ure_write_2(sc, URE_USB_TGPHY_CMD, URE_MCU_TYPE_USB,
	    URE_TGPHY_CMD_BUSY | URE_TGPHY_CMD_WRITE);

	for (i = 0; i < 10; i++) {
		if (!(ure_read_2(sc, URE_USB_TGPHY_CMD,
		    URE_MCU_TYPE_USB) & URE_TGPHY_CMD_BUSY))
			break;
		delay(drv_usectohz(1000));
	}
	if (i == 10)
		dev_err(sc->ure_dip, CE_WARN, "PHY write timeout");
}

/*
 * Convenience wrappers that go through the per-chip function
 * pointers.
 */
static inline uint16_t
ure_phy_read(ure_softc_t *sc, uint16_t addr)
{
	return (sc->ure_phy_read(sc, addr));
}

static inline void
ure_phy_write(ure_softc_t *sc, uint16_t addr, uint16_t data)
{
	sc->ure_phy_write(sc, addr, data);
}


/* ================================================================ */
/* Chip helper functions                                            */
/* ================================================================ */

static void
ure_disable_teredo(ure_softc_t *sc)
{
	if (sc->ure_flags & (URE_FLAG_8153B | URE_FLAG_8156 |
	    URE_FLAG_8156B | URE_FLAG_8157))
		ure_write_1(sc, URE_PLA_TEREDO_CFG,
		    URE_MCU_TYPE_PLA, 0xff);
	else {
		URE_CLRBIT_2(sc, URE_PLA_TEREDO_CFG,
		    URE_MCU_TYPE_PLA,
		    URE_TEREDO_SEL | URE_TEREDO_RS_EVENT_MASK |
		    URE_OOB_TEREDO_EN);
	}
	ure_write_2(sc, URE_PLA_WDT6_CTRL, URE_MCU_TYPE_PLA,
	    URE_WDT6_SET_MODE);
	ure_write_2(sc, URE_PLA_REALWOW_TIMER,
	    URE_MCU_TYPE_PLA, 0);
	ure_write_4(sc, URE_PLA_TEREDO_TIMER,
	    URE_MCU_TYPE_PLA, 0);
}

static void
ure_reset(ure_softc_t *sc)
{
	int i;

	if (sc->ure_flags & URE_FLAG_8157) {
		URE_CLRBIT_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA,
		    URE_CR_TE | URE_CR_RE);
	} else if (sc->ure_flags & URE_FLAG_8156) {
		URE_CLRBIT_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA,
		    URE_CR_TE);
		URE_CLRBIT_2(sc, URE_USB_BMU_RESET,
		    URE_MCU_TYPE_USB, URE_BMU_RESET_EP_IN);
		URE_SETBIT_2(sc, URE_USB_USB_CTRL,
		    URE_MCU_TYPE_USB, URE_CDC_ECM_EN);
		URE_CLRBIT_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA,
		    URE_CR_RE);
		URE_SETBIT_2(sc, URE_USB_BMU_RESET,
		    URE_MCU_TYPE_USB, URE_BMU_RESET_EP_IN);
		URE_CLRBIT_2(sc, URE_USB_USB_CTRL,
		    URE_MCU_TYPE_USB, URE_CDC_ECM_EN);
	} else {
		ure_write_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA,
		    URE_CR_RST);
		for (i = 0; i < URE_TIMEOUT; i++) {
			if (!(ure_read_1(sc, URE_PLA_CR,
			    URE_MCU_TYPE_PLA) & URE_CR_RST))
				break;
			drv_usecwait(100);
		}
		if (i == URE_TIMEOUT)
			dev_err(sc->ure_dip, CE_WARN,
			    "reset never completed");
	}
}

static void
ure_reset_bmu(ure_softc_t *sc)
{
	uint8_t reg;

	if (sc->ure_flags & URE_FLAG_8157) {
		/* RTL8157 uses OCP command interface */
		return;
	}

	reg = ure_read_1(sc, URE_USB_BMU_RESET, URE_MCU_TYPE_USB);
	reg &= ~(URE_BMU_RESET_EP_IN | URE_BMU_RESET_EP_OUT);
	ure_write_1(sc, URE_USB_BMU_RESET, URE_MCU_TYPE_USB, reg);
	reg |= URE_BMU_RESET_EP_IN | URE_BMU_RESET_EP_OUT;
	ure_write_1(sc, URE_USB_BMU_RESET, URE_MCU_TYPE_USB, reg);
}

static uint16_t
ure_rtl8153_phy_status(ure_softc_t *sc, int desired)
{
	uint16_t reg;
	int i;

	for (i = 0; i < 500; i++) {
		reg = ure_phy_read(sc, URE_OCP_PHY_STATUS) &
		    URE_PHY_STAT_MASK;
		if (desired) {
			if (reg == (uint16_t)desired)
				break;
		} else {
			if (reg == URE_PHY_STAT_LAN_ON ||
			    reg == URE_PHY_STAT_PWRDN ||
			    reg == URE_PHY_STAT_EXT_INIT)
				break;
		}
		delay(drv_usectohz(20000));
	}
	if (i == 500)
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for PHY to stabilize");

	return (reg);
}

static int
ure_wait_for_flash(ure_softc_t *sc)
{
	int i;

	if ((ure_read_2(sc, URE_PLA_GPHY_CTRL,
	    URE_MCU_TYPE_PLA) & URE_GPHY_FLASH) &&
	    !(ure_read_2(sc, URE_USB_GPHY_CTRL,
	    URE_MCU_TYPE_USB) & URE_BYPASS_FLASH)) {
		for (i = 0; i < 100; i++) {
			if (ure_read_2(sc, URE_USB_GPHY_CTRL,
			    URE_MCU_TYPE_USB) &
			    URE_GPHY_PATCH_DONE)
				break;
			delay(drv_usectohz(1000));
		}
		if (i == 100) {
			dev_err(sc->ure_dip, CE_WARN,
			    "timeout waiting for flash");
			return (ETIMEDOUT);
		}
	}

	return (0);
}

static void
ure_rxvlan(ure_softc_t *sc)
{
	if (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B |
	    URE_FLAG_8157)) {
		uint16_t reg;
		reg = ure_read_2(sc, URE_PLA_RCR1,
		    URE_MCU_TYPE_PLA);
		reg &= ~(URE_INNER_VLAN | URE_OUTER_VLAN);
		reg |= (URE_INNER_VLAN | URE_OUTER_VLAN);
		ure_write_2(sc, URE_PLA_RCR1,
		    URE_MCU_TYPE_PLA, reg);
	} else {
		URE_SETBIT_2(sc, URE_PLA_CPCR, URE_MCU_TYPE_PLA,
		    URE_CPCR_RX_VLAN);
	}
}

/* ================================================================ */
/* Chip init sequences                                              */
/* ================================================================ */

static void
ure_rtl8152_init(ure_softc_t *sc)
{
	uint32_t pwrctrl;

	/* Disable ALDPS */
	ure_ocp_reg_write(sc, URE_OCP_ALDPS_CONFIG,
	    URE_ENPDNPS | URE_LINKENA | URE_DIS_SDSAVE);
	delay(drv_usectohz(20000));

	if (sc->ure_chip & URE_CHIP_VER_4C00)
		URE_CLRBIT_2(sc, URE_PLA_LED_FEATURE,
		    URE_MCU_TYPE_PLA, URE_LED_MODE_MASK);

	URE_CLRBIT_2(sc, URE_USB_UPS_CTRL, URE_MCU_TYPE_USB,
	    URE_POWER_CUT);
	URE_CLRBIT_2(sc, URE_USB_PM_CTRL_STATUS,
	    URE_MCU_TYPE_USB, URE_RESUME_INDICATE);

	URE_SETBIT_2(sc, URE_PLA_PHY_PWR, URE_MCU_TYPE_PLA,
	    URE_TX_10M_IDLE_EN | URE_PFM_PWM_SWITCH);
	pwrctrl = ure_read_4(sc, URE_PLA_MAC_PWR_CTRL,
	    URE_MCU_TYPE_PLA);
	pwrctrl &= ~URE_MCU_CLK_RATIO_MASK;
	pwrctrl |= URE_MCU_CLK_RATIO | URE_D3_CLK_GATED_EN;
	ure_write_4(sc, URE_PLA_MAC_PWR_CTRL,
	    URE_MCU_TYPE_PLA, pwrctrl);
	ure_write_2(sc, URE_PLA_GPHY_INTR_IMR,
	    URE_MCU_TYPE_PLA,
	    URE_GPHY_STS_MSK | URE_SPEED_DOWN_MSK |
	    URE_SPDWN_RXDV_MSK | URE_SPDWN_LINKCHG_MSK);

	URE_SETBIT_2(sc, URE_PLA_RSTTALLY, URE_MCU_TYPE_PLA,
	    URE_TALLY_RESET);

	/* Enable Rx aggregation */
	URE_CLRBIT_2(sc, URE_USB_USB_CTRL, URE_MCU_TYPE_USB,
	    URE_RX_AGG_DISABLE | URE_RX_ZERO_EN);
}

static int
ure_rtl8153_init(ure_softc_t *sc)
{
	uint16_t reg;
	uint8_t u1u2[8];
	int i;

	bzero(u1u2, sizeof (u1u2));
	ure_write_mem(sc, URE_USB_TOLERANCE,
	    URE_BYTE_EN_SIX_BYTES, u1u2, sizeof (u1u2));

	for (i = 0; i < 500; i++) {
		if (ure_read_2(sc, URE_PLA_BOOT_CTRL,
		    URE_MCU_TYPE_PLA) & URE_AUTOLOAD_DONE)
			break;
		delay(drv_usectohz(20000));
	}
	if (i == 500) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for chip autoload");
		return (ETIMEDOUT);
	}

	(void) ure_rtl8153_phy_status(sc, 0);

	if (sc->ure_chip & (URE_CHIP_VER_5C00 |
	    URE_CHIP_VER_5C10 | URE_CHIP_VER_5C20)) {
		ure_ocp_reg_write(sc, URE_OCP_ADC_CFG,
		    URE_CKADSEL_L | URE_ADC_EN | URE_EN_EMI_L);
	}

	(void) ure_rtl8153_phy_status(sc, URE_PHY_STAT_LAN_ON);

	URE_CLRBIT_2(sc, URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB,
	    URE_U2P3_ENABLE);

	if (sc->ure_chip & URE_CHIP_VER_5C10) {
		reg = ure_read_2(sc, URE_USB_SSPHYLINK2,
		    URE_MCU_TYPE_USB);
		reg &= ~URE_PWD_DN_SCALE_MASK;
		reg |= URE_PWD_DN_SCALE(96);
		ure_write_2(sc, URE_USB_SSPHYLINK2,
		    URE_MCU_TYPE_USB, reg);

		URE_SETBIT_1(sc, URE_USB_USB2PHY,
		    URE_MCU_TYPE_USB,
		    URE_USB2PHY_L1 | URE_USB2PHY_SUSPEND);
	} else if (sc->ure_chip & URE_CHIP_VER_5C20) {
		URE_CLRBIT_1(sc, URE_PLA_DMY_REG0,
		    URE_MCU_TYPE_PLA, URE_ECM_ALDPS);
	}

	if (sc->ure_chip & (URE_CHIP_VER_5C20 |
	    URE_CHIP_VER_5C30)) {
		if (ure_read_2(sc, URE_USB_BURST_SIZE,
		    URE_MCU_TYPE_USB))
			URE_SETBIT_1(sc, URE_USB_CSR_DUMMY1,
			    URE_MCU_TYPE_USB, URE_DYNAMIC_BURST);
		else
			URE_CLRBIT_1(sc, URE_USB_CSR_DUMMY1,
			    URE_MCU_TYPE_USB, URE_DYNAMIC_BURST);
	}

	URE_SETBIT_1(sc, URE_USB_CSR_DUMMY2, URE_MCU_TYPE_USB,
	    URE_EP4_FULL_FC);
	URE_CLRBIT_2(sc, URE_USB_WDT11_CTRL, URE_MCU_TYPE_USB,
	    URE_TIMER11_EN);
	URE_CLRBIT_2(sc, URE_PLA_LED_FEATURE, URE_MCU_TYPE_PLA,
	    URE_LED_MODE_MASK);

	if ((sc->ure_chip & URE_CHIP_VER_5C10) &&
	    ure_dev_speed(sc) != USBA_SUPER_SPEED_DEV)
		reg = URE_LPM_TIMER_500MS;
	else
		reg = URE_LPM_TIMER_500US;
	ure_write_1(sc, URE_USB_LPM_CTRL, URE_MCU_TYPE_USB,
	    URE_FIFO_EMPTY_1FB | URE_ROK_EXIT_LPM | reg);

	reg = ure_read_2(sc, URE_USB_AFE_CTRL2,
	    URE_MCU_TYPE_USB);
	reg &= ~URE_SEN_VAL_MASK;
	reg |= URE_SEN_VAL_NORMAL | URE_SEL_RXIDLE;
	ure_write_2(sc, URE_USB_AFE_CTRL2,
	    URE_MCU_TYPE_USB, reg);

	ure_write_2(sc, URE_USB_CONNECT_TIMER,
	    URE_MCU_TYPE_USB, 0x0001);

	URE_CLRBIT_2(sc, URE_USB_POWER_CUT, URE_MCU_TYPE_USB,
	    URE_PWR_EN | URE_PHASE2_EN);
	URE_CLRBIT_2(sc, URE_USB_MISC_0, URE_MCU_TYPE_USB,
	    URE_PCUT_STATUS);

	(void) memset(u1u2, 0xff, sizeof (u1u2));
	ure_write_mem(sc, URE_USB_TOLERANCE,
	    URE_BYTE_EN_SIX_BYTES, u1u2, sizeof (u1u2));

	ure_write_2(sc, URE_PLA_MAC_PWR_CTRL,
	    URE_MCU_TYPE_PLA, 0);
	ure_write_2(sc, URE_PLA_MAC_PWR_CTRL2,
	    URE_MCU_TYPE_PLA, 0);
	ure_write_2(sc, URE_PLA_MAC_PWR_CTRL3,
	    URE_MCU_TYPE_PLA, 0);
	ure_write_2(sc, URE_PLA_MAC_PWR_CTRL4,
	    URE_MCU_TYPE_PLA, 0);

	/* Enable Rx aggregation */
	URE_CLRBIT_2(sc, URE_USB_USB_CTRL, URE_MCU_TYPE_USB,
	    URE_RX_AGG_DISABLE | URE_RX_ZERO_EN);

	URE_SETBIT_2(sc, URE_PLA_RSTTALLY, URE_MCU_TYPE_PLA,
	    URE_TALLY_RESET);

	return (0);
}

static int
ure_rtl8153b_init(ure_softc_t *sc)
{
	int i;

	if (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B)) {
		URE_CLRBIT_1(sc, URE_USB_ECM_OP,
		    URE_MCU_TYPE_USB, URE_EN_ALL_SPEED);
		ure_write_2(sc, URE_USB_SPEED_OPTION,
		    URE_MCU_TYPE_USB, 0);
		URE_SETBIT_2(sc, URE_USB_ECM_OPTION,
		    URE_MCU_TYPE_USB, URE_BYPASS_MAC_RESET);

		if (sc->ure_flags & URE_FLAG_8156B)
			URE_SETBIT_2(sc, URE_USB_U2P3_CTRL,
			    URE_MCU_TYPE_USB, URE_RX_DETECT8);
	}

	URE_CLRBIT_2(sc, URE_USB_LPM_CONFIG, URE_MCU_TYPE_USB,
	    URE_LPM_U1U2_EN);

	if (sc->ure_flags & URE_FLAG_8156B) {
		if (ure_wait_for_flash(sc) != 0)
			return (ETIMEDOUT);
	}

	for (i = 0; i < 500; i++) {
		if (ure_read_2(sc, URE_PLA_BOOT_CTRL,
		    URE_MCU_TYPE_PLA) & URE_AUTOLOAD_DONE)
			break;
		delay(drv_usectohz(20000));
	}
	if (i == 500) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for chip autoload");
		return (ETIMEDOUT);
	}

	(void) ure_rtl8153_phy_status(sc, 0);
	(void) ure_rtl8153_phy_status(sc, URE_PHY_STAT_LAN_ON);

	URE_CLRBIT_2(sc, URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB,
	    URE_U2P3_ENABLE);

	/* MSC timer, 32760 ms */
	ure_write_2(sc, URE_USB_MSC_TIMER, URE_MCU_TYPE_USB,
	    4095);

	if (!(sc->ure_flags & URE_FLAG_8153B)) {
		/* U1/U2/L1 idle timer, 500 us */
		ure_write_2(sc, URE_USB_U1U2_TIMER,
		    URE_MCU_TYPE_USB, 500);
	}

	URE_CLRBIT_2(sc, URE_USB_POWER_CUT, URE_MCU_TYPE_USB,
	    URE_PWR_EN);
	URE_CLRBIT_2(sc, URE_USB_MISC_0, URE_MCU_TYPE_USB,
	    URE_PCUT_STATUS);

	URE_CLRBIT_1(sc, URE_USB_POWER_CUT, URE_MCU_TYPE_USB,
	    URE_UPS_EN | URE_USP_PREWAKE);
	URE_CLRBIT_1(sc, URE_USB_MISC_2, URE_MCU_TYPE_USB,
	    URE_UPS_FORCE_PWR_DOWN | URE_UPS_NO_UPS);

	URE_CLRBIT_1(sc, URE_PLA_INDICATE_FALG,
	    URE_MCU_TYPE_PLA, URE_UPCOMING_RUNTIME_D3);
	URE_CLRBIT_1(sc, URE_PLA_SUSPEND_FLAG,
	    URE_MCU_TYPE_PLA, URE_LINK_CHG_EVENT);
	URE_CLRBIT_2(sc, URE_PLA_EXTRA_STATUS,
	    URE_MCU_TYPE_PLA, URE_LINK_CHANGE_FLAG);

	ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_CONFIG);
	URE_CLRBIT_2(sc, URE_PLA_CONFIG34, URE_MCU_TYPE_PLA,
	    URE_LINK_OFF_WAKE_EN);
	ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_NORAML);

	if (sc->ure_flags & URE_FLAG_8153B) {
		uint16_t reg;
		reg = ure_read_2(sc, URE_PLA_EXTRA_STATUS,
		    URE_MCU_TYPE_PLA);
		if (ure_read_2(sc, URE_PLA_PHYSTATUS,
		    URE_MCU_TYPE_PLA) & URE_PHYSTATUS_LINK)
			reg |= URE_CUR_LINK_OK;
		else
			reg &= ~URE_CUR_LINK_OK;
		ure_write_2(sc, URE_PLA_EXTRA_STATUS,
		    URE_MCU_TYPE_PLA,
		    reg | URE_POLL_LINK_CHG);
	}

	if (ure_dev_speed(sc) == USBA_SUPER_SPEED_DEV)
		URE_SETBIT_2(sc, URE_USB_LPM_CONFIG,
		    URE_MCU_TYPE_USB, URE_LPM_U1U2_EN);

	if (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B)) {
		URE_CLRBIT_2(sc, URE_PLA_MAC_PWR_CTRL3,
		    URE_MCU_TYPE_PLA, URE_PLA_MCU_SPDWN_EN);

		uint16_t reg;
		reg = ure_read_2(sc, URE_PLA_EXTRA_STATUS,
		    URE_MCU_TYPE_PLA);
		reg &= ~URE_CUR_LINK_OK;
		if (ure_read_2(sc, URE_PLA_PHYSTATUS,
		    URE_MCU_TYPE_PLA) & URE_PHYSTATUS_LINK)
			reg |= URE_CUR_LINK_OK;
		ure_write_2(sc, URE_PLA_EXTRA_STATUS,
		    URE_MCU_TYPE_PLA,
		    reg | URE_POLL_LINK_CHG);
	} else {
		URE_SETBIT_2(sc, URE_PLA_MAC_PWR_CTRL2,
		    URE_MCU_TYPE_PLA, URE_MAC_CLK_SPDWN_EN);
	}

	/* Enable Rx aggregation */
	URE_CLRBIT_2(sc, URE_USB_USB_CTRL, URE_MCU_TYPE_USB,
	    URE_RX_AGG_DISABLE | URE_RX_ZERO_EN);

	if (sc->ure_flags & URE_FLAG_8156)
		URE_SETBIT_1(sc, URE_USB_BMU_CONFIG,
		    URE_MCU_TYPE_USB, URE_ACT_ODMA);

	if (!(sc->ure_flags & URE_FLAG_8153B)) {
		ure_phy_write(sc, 0xa5b4,
		    ure_phy_read(sc, 0xa5b4) & ~0x8000);
	}

	URE_SETBIT_2(sc, URE_PLA_RSTTALLY, URE_MCU_TYPE_PLA,
	    URE_TALLY_RESET);

	return (0);
}

static int
ure_rtl8157_init(ure_softc_t *sc)
{
	uint16_t reg;
	int i;

	URE_SETBIT_1(sc, 0xcffe, URE_MCU_TYPE_USB, 0x0008);
	URE_CLRBIT_1(sc, 0xd3ca, URE_MCU_TYPE_USB, 0x0001);
	URE_CLRBIT_1(sc, URE_USB_ECM_OP, URE_MCU_TYPE_USB,
	    URE_EN_ALL_SPEED);
	URE_SETBIT_2(sc, URE_USB_ECM_OPTION, URE_MCU_TYPE_USB,
	    URE_BYPASS_MAC_RESET);
	URE_SETBIT_2(sc, URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB,
	    URE_RX_DETECT8);
	URE_CLRBIT_2(sc, URE_USB_LPM_CONFIG, URE_MCU_TYPE_USB,
	    URE_LPM_U1U2_EN);

	for (i = 0; i < 500; i++) {
		if (ure_read_2(sc, URE_PLA_BOOT_CTRL,
		    URE_MCU_TYPE_PLA) & URE_AUTOLOAD_DONE)
			break;
		delay(drv_usectohz(20000));
	}
	if (i == 500) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for chip autoload");
		return (ETIMEDOUT);
	}

	if (ure_wait_for_flash(sc) != 0)
		return (ETIMEDOUT);

	(void) ure_rtl8153_phy_status(sc, 0);
	(void) ure_rtl8153_phy_status(sc, URE_PHY_STAT_LAN_ON);

	URE_CLRBIT_2(sc, URE_USB_SPEED_OPTION, URE_MCU_TYPE_USB,
	    URE_RG_PWRDN_EN | URE_ALL_SPEED_OFF);
	URE_SETBIT_2(sc, URE_USB_FW_CTRL, URE_MCU_TYPE_USB,
	    URE_AUTO_SPEEDUP);

	ure_write_2(sc, URE_USB_MSC_TIMER, URE_MCU_TYPE_USB,
	    4095);
	ure_write_2(sc, URE_USB_U1U2_TIMER, URE_MCU_TYPE_USB,
	    500);

	URE_CLRBIT_2(sc, URE_USB_POWER_CUT, URE_MCU_TYPE_USB,
	    URE_PWR_EN);
	URE_CLRBIT_2(sc, URE_USB_MISC_0, URE_MCU_TYPE_USB,
	    URE_PCUT_STATUS);
	URE_CLRBIT_1(sc, URE_USB_MISC_2, URE_MCU_TYPE_USB,
	    0x02);

	URE_CLRBIT_1(sc, URE_PLA_INDICATE_FALG,
	    URE_MCU_TYPE_PLA, URE_UPCOMING_RUNTIME_D3);
	URE_CLRBIT_1(sc, URE_PLA_SUSPEND_FLAG,
	    URE_MCU_TYPE_PLA, URE_LINK_CHG_EVENT);
	URE_CLRBIT_2(sc, URE_PLA_EXTRA_STATUS,
	    URE_MCU_TYPE_PLA, URE_LINK_CHANGE_FLAG);

	ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_CONFIG);
	URE_CLRBIT_2(sc, URE_PLA_CONFIG34, URE_MCU_TYPE_PLA,
	    URE_LINK_OFF_WAKE_EN);
	ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_NORAML);

	URE_CLRBIT_2(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA,
	    URE_SLOT_EN);
	URE_SETBIT_2(sc, URE_PLA_CPCR, URE_MCU_TYPE_PLA,
	    URE_FLOW_CTRL_EN);

	ure_write_2(sc, URE_USB_FC_TIMER, URE_MCU_TYPE_USB,
	    URE_CTRL_TIMER_EN | 75);

	reg = ure_read_2(sc, URE_USB_FW_CTRL, URE_MCU_TYPE_USB);
	reg &= ~URE_AUTO_SPEEDUP;
	if (!(ure_read_2(sc, URE_PLA_POL_GPIO_CTRL,
	    URE_MCU_TYPE_PLA) & URE_DACK_DET_EN))
		reg |= URE_FLOW_CTRL_PATCH_2;
	ure_write_2(sc, URE_USB_FW_CTRL, URE_MCU_TYPE_USB,
	    reg);

	URE_SETBIT_2(sc, URE_USB_FW_TASK, URE_MCU_TYPE_USB,
	    URE_FC_PATCH_TASK);

	URE_CLRBIT_2(sc, URE_PLA_MAC_PWR_CTRL3,
	    URE_MCU_TYPE_PLA, URE_PLA_MCU_SPDWN_EN);

	reg = ure_read_2(sc, URE_PLA_EXTRA_STATUS,
	    URE_MCU_TYPE_PLA);
	reg &= ~URE_CUR_LINK_OK;
	if (ure_read_2(sc, URE_PLA_PHYSTATUS,
	    URE_MCU_TYPE_PLA) & URE_PHYSTATUS_LINK)
		reg |= URE_CUR_LINK_OK;
	ure_write_2(sc, URE_PLA_EXTRA_STATUS,
	    URE_MCU_TYPE_PLA, reg | URE_POLL_LINK_CHG);

	/* Enable Rx aggregation */
	URE_CLRBIT_2(sc, URE_USB_USB_CTRL, URE_MCU_TYPE_USB,
	    URE_RX_AGG_DISABLE | 0x0400);

	reg = ure_phy_read(sc, 0xa5b4);
	if (reg != 0xffff)
		ure_phy_write(sc, 0xa5b4, reg & ~0x8000);

	URE_SETBIT_2(sc, URE_PLA_RSTTALLY, URE_MCU_TYPE_PLA,
	    URE_TALLY_RESET);

	return (0);
}

/* ================================================================ */
/* NIC reset (called per interface-up)                              */
/* ================================================================ */

static int
ure_rtl8152_nic_reset(ure_softc_t *sc)
{
	uint32_t rx_fifo1, rx_fifo2;
	int i;

	/* Disable ALDPS */
	ure_ocp_reg_write(sc, URE_OCP_ALDPS_CONFIG,
	    URE_ENPDNPS | URE_LINKENA | URE_DIS_SDSAVE);
	delay(drv_usectohz(20000));

	URE_CLRBIT_4(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA,
	    URE_RCR_ACPT_ALL);
	URE_SETBIT_2(sc, URE_PLA_MISC_1, URE_MCU_TYPE_PLA,
	    URE_RXDY_GATED_EN);
	ure_disable_teredo(sc);
	ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_NORAML);
	ure_write_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA, 0);

	URE_CLRBIT_1(sc, URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA,
	    URE_NOW_IS_OOB);
	URE_CLRBIT_2(sc, URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA,
	    URE_MCU_BORW_EN);
	for (i = 0; i < URE_TIMEOUT; i++) {
		if (ure_read_1(sc, URE_PLA_OOB_CTRL,
		    URE_MCU_TYPE_PLA) & URE_LINK_LIST_READY)
			break;
		delay(drv_usectohz(1000));
	}
	if (i == URE_TIMEOUT) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for OOB control");
		return (ETIMEDOUT);
	}
	URE_SETBIT_2(sc, URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA,
	    URE_RE_INIT_LL);
	for (i = 0; i < URE_TIMEOUT; i++) {
		if (ure_read_1(sc, URE_PLA_OOB_CTRL,
		    URE_MCU_TYPE_PLA) & URE_LINK_LIST_READY)
			break;
		delay(drv_usectohz(1000));
	}
	if (i == URE_TIMEOUT) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for OOB control (2)");
		return (ETIMEDOUT);
	}

	ure_reset(sc);

	/* Configure Rx FIFO threshold */
	ure_write_4(sc, URE_PLA_RXFIFO_CTRL0, URE_MCU_TYPE_PLA,
	    URE_RXFIFO_THR1_NORMAL);
	if (ure_dev_speed(sc) == USBA_FULL_SPEED_DEV) {
		rx_fifo1 = URE_RXFIFO_THR2_FULL;
		rx_fifo2 = URE_RXFIFO_THR3_FULL;
	} else {
		rx_fifo1 = URE_RXFIFO_THR2_HIGH;
		rx_fifo2 = URE_RXFIFO_THR3_HIGH;
	}
	ure_write_4(sc, URE_PLA_RXFIFO_CTRL1, URE_MCU_TYPE_PLA,
	    rx_fifo1);
	ure_write_4(sc, URE_PLA_RXFIFO_CTRL2, URE_MCU_TYPE_PLA,
	    rx_fifo2);

	/* Configure Tx FIFO threshold */
	ure_write_4(sc, URE_PLA_TXFIFO_CTRL, URE_MCU_TYPE_PLA,
	    URE_TXFIFO_THR_NORMAL);

	ure_write_1(sc, URE_USB_TX_AGG, URE_MCU_TYPE_USB,
	    URE_TX_AGG_MAX_THRESHOLD);
	ure_write_4(sc, URE_USB_RX_BUF_TH, URE_MCU_TYPE_USB,
	    URE_RX_THR_HIGH);
	ure_write_4(sc, URE_USB_TX_DMA, URE_MCU_TYPE_USB,
	    URE_TEST_MODE_DISABLE | URE_TX_SIZE_ADJUST1);

	ure_rxvlan(sc);
	ure_write_2(sc, URE_PLA_RMS, URE_MCU_TYPE_PLA,
	    ETHERMAX + VLAN_TAGSZ);
	URE_SETBIT_2(sc, URE_PLA_TCR0, URE_MCU_TYPE_PLA,
	    URE_TCR0_AUTO_FIFO);

	/* Enable ALDPS */
	ure_ocp_reg_write(sc, URE_OCP_ALDPS_CONFIG,
	    URE_ENPWRSAVE | URE_ENPDNPS | URE_LINKENA |
	    URE_DIS_SDSAVE);

	return (0);
}

static int
ure_rtl8153_nic_reset(ure_softc_t *sc)
{
	uint8_t u1u2[8] = { 0 };
	int i;

	switch (sc->ure_flags & URE_FLAG_CHIP_MASK) {
	case URE_FLAG_8153B:
	case URE_FLAG_8156:
	case URE_FLAG_8156B:
	case URE_FLAG_8157:
		URE_CLRBIT_2(sc, URE_USB_LPM_CONFIG,
		    URE_MCU_TYPE_USB, URE_LPM_U1U2_EN);
		break;
	default:
		bzero(u1u2, sizeof (u1u2));
		ure_write_mem(sc, URE_USB_TOLERANCE,
		    URE_BYTE_EN_SIX_BYTES, u1u2, sizeof (u1u2));
		break;
	}
	URE_CLRBIT_2(sc, URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB,
	    URE_U2P3_ENABLE);

	/* Disable ALDPS */
	ure_phy_write(sc, URE_OCP_POWER_CFG,
	    ure_phy_read(sc, URE_OCP_POWER_CFG) & ~URE_EN_ALDPS);
	for (i = 0; i < 20; i++) {
		delay(drv_usectohz(1000));
		if (ure_read_2(sc, 0xe000, URE_MCU_TYPE_PLA) &
		    0x0100)
			break;
	}

	URE_SETBIT_2(sc, URE_PLA_MISC_1, URE_MCU_TYPE_PLA,
	    URE_RXDY_GATED_EN);
	ure_disable_teredo(sc);
	URE_CLRBIT_4(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA,
	    URE_RCR_ACPT_ALL);
	ure_reset(sc);
	ure_reset_bmu(sc);

	URE_CLRBIT_1(sc, URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA,
	    URE_NOW_IS_OOB);
	URE_CLRBIT_2(sc, URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA,
	    URE_MCU_BORW_EN);

	if (!(sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B |
	    URE_FLAG_8157))) {
		for (i = 0; i < URE_TIMEOUT; i++) {
			if (ure_read_1(sc, URE_PLA_OOB_CTRL,
			    URE_MCU_TYPE_PLA) &
			    URE_LINK_LIST_READY)
				break;
			delay(drv_usectohz(1000));
		}
		if (i == URE_TIMEOUT) {
			dev_err(sc->ure_dip, CE_WARN,
			    "timeout waiting for OOB control");
			return (ETIMEDOUT);
		}
		URE_SETBIT_2(sc, URE_PLA_SFF_STS_7,
		    URE_MCU_TYPE_PLA, URE_RE_INIT_LL);
		for (i = 0; i < URE_TIMEOUT; i++) {
			if (ure_read_1(sc, URE_PLA_OOB_CTRL,
			    URE_MCU_TYPE_PLA) &
			    URE_LINK_LIST_READY)
				break;
			delay(drv_usectohz(1000));
		}
		if (i == URE_TIMEOUT) {
			dev_err(sc->ure_dip, CE_WARN,
			    "timeout waiting for OOB (2)");
			return (ETIMEDOUT);
		}
	}

	ure_rxvlan(sc);
	ure_write_2(sc, URE_PLA_RMS, URE_MCU_TYPE_PLA,
	    ETHERMAX + VLAN_TAGSZ);
	ure_write_1(sc, URE_PLA_MTPS, URE_MCU_TYPE_PLA,
	    (sc->ure_flags & URE_FLAG_8157) ?
	    URE_MTPS_MAX : URE_MTPS_JUMBO);

	if (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B |
	    URE_FLAG_8157)) {
		ure_write_4(sc, URE_USB_RX_BUF_TH,
		    URE_MCU_TYPE_USB, 0x00600400);
	} else {
		URE_SETBIT_2(sc, URE_PLA_TCR0, URE_MCU_TYPE_PLA,
		    URE_TCR0_AUTO_FIFO);
		ure_reset(sc);

		ure_write_4(sc, URE_PLA_RXFIFO_CTRL0,
		    URE_MCU_TYPE_PLA, URE_RXFIFO_THR1_NORMAL);
		ure_write_2(sc, URE_PLA_RXFIFO_CTRL1,
		    URE_MCU_TYPE_PLA, URE_RXFIFO_THR2_NORMAL);
		ure_write_2(sc, URE_PLA_RXFIFO_CTRL2,
		    URE_MCU_TYPE_PLA, URE_RXFIFO_THR3_NORMAL);
		ure_write_4(sc, URE_PLA_TXFIFO_CTRL,
		    URE_MCU_TYPE_PLA, URE_TXFIFO_THR_NORMAL2);

		if (sc->ure_flags & URE_FLAG_8153B) {
			ure_write_4(sc, URE_USB_RX_BUF_TH,
			    URE_MCU_TYPE_USB, URE_RX_THR_B);

			URE_CLRBIT_2(sc, URE_PLA_MAC_PWR_CTRL3,
			    URE_MCU_TYPE_PLA, URE_PLA_MCU_SPDWN_EN);
		} else {
			URE_SETBIT_1(sc, URE_PLA_CONFIG6,
			    URE_MCU_TYPE_PLA, URE_LANWAKE_CLR_EN);
			URE_CLRBIT_1(sc, URE_PLA_LWAKE_CTRL_REG,
			    URE_MCU_TYPE_PLA, URE_LANWAKE_PIN);
			URE_CLRBIT_2(sc, URE_USB_SSPHYLINK1,
			    URE_MCU_TYPE_USB,
			    URE_DELAY_PHY_PWR_CHG);
		}
	}

	/* Enable ALDPS */
	ure_phy_write(sc, URE_OCP_POWER_CFG,
	    ure_phy_read(sc, URE_OCP_POWER_CFG) |
	    URE_EN_ALDPS);

	if ((sc->ure_chip & (URE_CHIP_VER_5C20 |
	    URE_CHIP_VER_5C30)) ||
	    (sc->ure_flags & (URE_FLAG_8156 |
	    URE_FLAG_8156B)))
		URE_SETBIT_2(sc, URE_USB_U2P3_CTRL,
		    URE_MCU_TYPE_USB, URE_U2P3_ENABLE);

	if (sc->ure_flags & (URE_FLAG_8153B | URE_FLAG_8156 |
	    URE_FLAG_8156B)) {
		if (ure_dev_speed(sc) ==
		    USBA_SUPER_SPEED_DEV)
			URE_SETBIT_2(sc, URE_USB_LPM_CONFIG,
			    URE_MCU_TYPE_USB,
			    URE_LPM_U1U2_EN);
	} else if (!(sc->ure_flags & URE_FLAG_8157)) {
		(void) memset(u1u2, 0xff, sizeof (u1u2));
		ure_write_mem(sc, URE_USB_TOLERANCE,
		    URE_BYTE_EN_SIX_BYTES,
		    u1u2, sizeof (u1u2));
	}

	return (0);
}


/* ================================================================ */
/* Media / link state                                               */
/* ================================================================ */

static void
ure_ifmedia_init(ure_softc_t *sc)
{
	/* Set MAC address */
	ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_CONFIG);
	ure_write_mem(sc, URE_PLA_IDR,
	    URE_MCU_TYPE_PLA | URE_BYTE_EN_SIX_BYTES,
	    sc->ure_dev_addr, 8);
	ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_NORAML);

	if (!(sc->ure_flags & URE_FLAG_8152)) {
		uint32_t reg;
		reg = sc->ure_rxbufsz - (ETHERMAX + VLAN_TAGSZ);
		if (sc->ure_flags & URE_FLAG_8157)
			reg -= sizeof (ure_rxpkt_v2_t) +
			    URE_8157_BUF_ALIGN;
		else
			reg -= sizeof (ure_rxpkt_t) +
			    URE_RX_BUF_ALIGN;

		if (sc->ure_flags & (URE_FLAG_8153B |
		    URE_FLAG_8156 | URE_FLAG_8156B |
		    URE_FLAG_8157)) {
			ure_write_2(sc, URE_USB_RX_EARLY_SIZE,
			    URE_MCU_TYPE_USB,
			    (sc->ure_flags & URE_FLAG_8157) ?
			    reg / URE_8157_BUF_ALIGN :
			    reg / URE_RX_BUF_ALIGN);
			ure_write_2(sc, URE_USB_RX_EARLY_AGG,
			    URE_MCU_TYPE_USB,
			    (sc->ure_flags & URE_FLAG_8153B) ?
			    158 : 80);
			ure_write_2(sc, URE_USB_PM_CTRL_STATUS,
			    URE_MCU_TYPE_USB, 1875);
		} else {
			ure_write_2(sc, URE_USB_RX_EARLY_SIZE,
			    URE_MCU_TYPE_USB, reg / 4);
			switch (ure_dev_speed(sc)) {
			case USBA_SUPER_SPEED_DEV:
				reg = URE_COALESCE_SUPER / 8;
				break;
			case USBA_HIGH_SPEED_DEV:
				reg = URE_COALESCE_HIGH / 8;
				break;
			default:
				reg = URE_COALESCE_SLOW / 8;
				break;
			}
			ure_write_2(sc, URE_USB_RX_EARLY_AGG,
			    URE_MCU_TYPE_USB, reg);
		}
	}

	/* Reset the packet filter */
	URE_CLRBIT_2(sc, URE_PLA_FMC, URE_MCU_TYPE_PLA,
	    URE_FMC_FCR_MCU_EN);
	URE_SETBIT_2(sc, URE_PLA_FMC, URE_MCU_TYPE_PLA,
	    URE_FMC_FCR_MCU_EN);

	/* Enable transmit and receive */
	URE_SETBIT_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA,
	    URE_CR_RE | URE_CR_TE);

	if (sc->ure_flags & (URE_FLAG_8153B | URE_FLAG_8156 |
	    URE_FLAG_8156B | URE_FLAG_8157)) {
		ure_write_1(sc, URE_USB_UPT_RXDMA_OWN,
		    URE_MCU_TYPE_USB,
		    URE_OWN_UPDATE | URE_OWN_CLEAR);
	}

	URE_CLRBIT_2(sc, URE_PLA_MISC_1, URE_MCU_TYPE_PLA,
	    URE_RXDY_GATED_EN);
}

static int
ure_get_link_status(ure_softc_t *sc)
{
	if (ure_read_2(sc, URE_PLA_PHYSTATUS, URE_MCU_TYPE_PLA) &
	    URE_PHYSTATUS_LINK) {
		sc->ure_flags |= URE_FLAG_LINK;
		return (1);
	} else {
		sc->ure_flags &= ~URE_FLAG_LINK;
		return (0);
	}
}

/*
 * Periodic link status check — called via ddi_periodic_add.
 * Includes FreeBSD spurious link-down workaround (PR 252165):
 * BMSR link status is latched-low, so we double-read to clear
 * any stale latch before trusting a link-down report.
 */
static void
ure_link_check(void *arg)
{
	ure_softc_t *sc = (ure_softc_t *)arg;
	link_state_t new_link;
	uint64_t speed = 0;
	link_duplex_t duplex = LINK_DUPLEX_UNKNOWN;
	uint16_t status;

	if (sc->ure_gone)
		return;

	mutex_enter(&sc->ure_lock);

	if (ure_get_link_status(sc)) {
		new_link = LINK_STATE_UP;
		status = ure_read_2(sc, URE_PLA_PHYSTATUS,
		    URE_MCU_TYPE_PLA);

		if (status & URE_PHYSTATUS_5000MBPS) {
			speed = 5000000000ULL;
			duplex = LINK_DUPLEX_FULL;
		} else if (status & URE_PHYSTATUS_2500MBPS) {
			speed = 2500000000ULL;
			duplex = LINK_DUPLEX_FULL;
		} else if (status & URE_PHYSTATUS_1000MBPS) {
			speed = 1000000000ULL;
			duplex = (status & URE_PHYSTATUS_FDX) ?
			    LINK_DUPLEX_FULL : LINK_DUPLEX_HALF;
		} else if (status & URE_PHYSTATUS_100MBPS) {
			speed = 100000000ULL;
			duplex = (status & URE_PHYSTATUS_FDX) ?
			    LINK_DUPLEX_FULL : LINK_DUPLEX_HALF;
		} else if (status & URE_PHYSTATUS_10MBPS) {
			speed = 10000000ULL;
			duplex = (status & URE_PHYSTATUS_FDX) ?
			    LINK_DUPLEX_FULL : LINK_DUPLEX_HALF;
		}

		/* Re-enable TX/RX on link up */
		URE_SETBIT_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA,
		    URE_CR_RE | URE_CR_TE);
	} else {
		/*
		 * FreeBSD spurious link-down workaround:
		 * MII BMSR link status is latched-low.
		 * Double-read to clear stale latch.
		 */
		(void) ure_ocp_reg_read(sc,
		    URE_OCP_BASE_MII + 0x02);	/* MII_BMSR */
		uint16_t bmsr = ure_ocp_reg_read(sc,
		    URE_OCP_BASE_MII + 0x02);

		if (bmsr & 0x0004) {	/* BMSR_LINK */
			/* PHY still has link — spurious */
			new_link = LINK_STATE_UP;
			/* Keep previous speed/duplex */
			speed = sc->ure_link_speed;
			duplex = sc->ure_link_duplex;
		} else {
			new_link = LINK_STATE_DOWN;
		}
	}

	if (new_link != sc->ure_link_state ||
	    speed != sc->ure_link_speed) {
		sc->ure_link_state = new_link;
		sc->ure_link_speed = speed;
		sc->ure_link_duplex = duplex;
		mutex_exit(&sc->ure_lock);

		mac_link_update(sc->ure_mh, new_link);
	} else {
		mutex_exit(&sc->ure_lock);
	}
}

/* ================================================================ */
/* RX filter (multicast hash, promisc)                              */
/* ================================================================ */

static void
ure_set_rx_filter(ure_softc_t *sc)
{
	uint32_t rxmode;
	uint32_t hashes[2];

	ASSERT(MUTEX_HELD(&sc->ure_lock));

	rxmode = ure_read_4(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA);
	rxmode &= ~URE_RCR_ACPT_ALL;

	/* Always accept our own address and broadcasts */
	rxmode |= URE_RCR_APM | URE_RCR_AB;

	if (sc->ure_promisc) {
		rxmode |= URE_RCR_AAP | URE_RCR_AM;
		hashes[0] = hashes[1] = 0xffffffff;
	} else {
		rxmode |= URE_RCR_AM;
		hashes[0] = sc->ure_mcast_hash[0];
		hashes[1] = sc->ure_mcast_hash[1];
	}

	/* Swap hash words (hardware expects swapped order) */
	{
		uint32_t tmp = hashes[0];
		hashes[0] = BSWAP_32(hashes[1]);
		hashes[1] = BSWAP_32(tmp);
	}

	ure_write_mem(sc, URE_PLA_MAR,
	    URE_MCU_TYPE_PLA | URE_BYTE_EN_DWORD,
	    hashes, sizeof (hashes));
	ure_write_4(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA, rxmode);
}

/* ================================================================ */
/* RX path                                                          */
/* ================================================================ */

static void
ure_rx_start(ure_softc_t *sc)
{
	usb_bulk_req_t *req;

	ASSERT(MUTEX_HELD(&sc->ure_lock));

	if (sc->ure_gone || !sc->ure_running || sc->ure_rx_running)
		return;

	req = usb_alloc_bulk_req(sc->ure_dip,
	    sc->ure_rxbufsz, USB_FLAGS_NOSLEEP);
	if (req == NULL) {
		dev_err(sc->ure_dip, CE_WARN,
		    "failed to allocate bulk RX request");
		return;
	}

	req->bulk_len = sc->ure_rxbufsz;
	req->bulk_cb = ure_rx_cb;
	req->bulk_exc_cb = ure_rx_cb;
	req->bulk_client_private = (usb_opaque_t)sc;
	req->bulk_timeout = 0;
	req->bulk_attributes = USB_ATTRS_SHORT_XFER_OK |
	    USB_ATTRS_AUTOCLEARING;

	sc->ure_rx_running = B_TRUE;

	if (usb_pipe_bulk_xfer(sc->ure_bulkin_pipe, req, 0) !=
	    USB_SUCCESS) {
		dev_err(sc->ure_dip, CE_WARN,
		    "failed to start bulk RX transfer");
		usb_free_bulk_req(req);
		sc->ure_rx_running = B_FALSE;
	}
}

/*
 * RX callback — called when a bulk IN transfer completes.
 * The buffer may contain multiple aggregated packets, each
 * preceded by a ure_rxpkt (or ure_rxpkt_v2 for RTL8157) header.
 */
static void
ure_rx_cb(usb_pipe_handle_t ph, usb_bulk_req_t *req)
{
	_NOTE(ARGUNUSED(ph));
	ure_softc_t *sc = (ure_softc_t *)req->bulk_client_private;
	mblk_t *data = req->bulk_data;
	mblk_t *head = NULL, *tail = NULL;
	uint32_t total_len;
	uint32_t hdrsize, align;
	int pktlen;

	if (sc->ure_gone || !sc->ure_running) {
		usb_free_bulk_req(req);
		mutex_enter(&sc->ure_lock);
		sc->ure_rx_running = B_FALSE;
		mutex_exit(&sc->ure_lock);
		return;
	}

	if (req->bulk_completion_reason != USB_CR_OK) {
		if (req->bulk_completion_reason != USB_CR_STOPPED_POLLING)
			sc->ure_stat_ierrors++;
		goto restart;
	}

	if (data == NULL)
		goto restart;

	total_len = MBLKL(data);

	align = (sc->ure_flags & URE_FLAG_8157) ?
	    URE_8157_BUF_ALIGN : URE_RX_BUF_ALIGN;
	hdrsize = (sc->ure_flags & URE_FLAG_8157) ?
	    sizeof (ure_rxpkt_v2_t) : sizeof (ure_rxpkt_t);

	unsigned char *buf = data->b_rptr;
	uint32_t off = 0;

	while (total_len > hdrsize) {
		mblk_t *mp;

		if (sc->ure_flags & URE_FLAG_8157) {
			ure_rxpkt_v2_t rxhdr;
			bcopy(buf + off, &rxhdr, sizeof (rxhdr));
			pktlen = (LE_32(rxhdr.ure_pktlen) &
			    URE_RXPKT_V2_LEN_MASK) >> 17;
		} else {
			ure_rxpkt_t rxhdr;
			bcopy(buf + off, &rxhdr, sizeof (rxhdr));
			pktlen = LE_32(rxhdr.ure_pktlen) &
			    URE_RXPKT_LEN_MASK;
		}

		off += hdrsize;
		total_len -= hdrsize;

		if (pktlen > (int)total_len || pktlen < ETHERMIN) {
			sc->ure_stat_ierrors++;
			break;
		}

		/* Strip CRC */
		int actual = pktlen - ETHERFCSL;
		if (actual <= 0) {
			sc->ure_stat_ierrors++;
			break;
		}

		mp = allocb(actual + VLAN_TAGSZ, BPRI_MED);
		if (mp == NULL) {
			sc->ure_stat_norcvbuf++;
			break;
		}

		bcopy(buf + off, mp->b_wptr, actual);
		mp->b_wptr += actual;

		/* TODO: Extract RX checksum results and VLAN tags */

		/* Chain it */
		if (head == NULL) {
			head = tail = mp;
		} else {
			tail->b_next = mp;
			tail = mp;
		}

		sc->ure_stat_ipackets++;
		sc->ure_stat_rbytes += actual;

		uint32_t consumed = P2ROUNDUP(pktlen, align);
		if (consumed > total_len)
			break;
		off += consumed;
		total_len -= consumed;
	}

	/* Pass received chain to MAC */
	if (head != NULL)
		mac_rx(sc->ure_mh, NULL, head);

restart:
	/* Resubmit RX */
	req->bulk_data = NULL;
	usb_free_bulk_req(req);
	if (data != NULL)
		freemsg(data);

	mutex_enter(&sc->ure_lock);
	sc->ure_rx_running = B_FALSE;
	if (sc->ure_running && !sc->ure_gone)
		ure_rx_start(sc);
	mutex_exit(&sc->ure_lock);
}

/* ================================================================ */
/* TX path                                                          */
/* ================================================================ */

static void
ure_tx_cb(usb_pipe_handle_t ph, usb_bulk_req_t *req)
{
	_NOTE(ARGUNUSED(ph));
	ure_softc_t *sc = (ure_softc_t *)req->bulk_client_private;

	if (req->bulk_completion_reason != USB_CR_OK)
		sc->ure_stat_oerrors++;

	usb_free_bulk_req(req);

	mutex_enter(&sc->ure_tx_lock);
	sc->ure_tx_busy = B_FALSE;
	mutex_exit(&sc->ure_tx_lock);

	if (sc->ure_running && !sc->ure_gone)
		mac_tx_update(sc->ure_mh);
}

/*
 * mc_tx callback — transmit a chain of mblk_t packets.
 * Aggregates multiple packets into a single USB bulk OUT
 * transfer with per-packet ure_txpkt headers.
 */
static mblk_t *
ure_m_tx(void *arg, mblk_t *mp)
{
	ure_softc_t *sc = (ure_softc_t *)arg;
	mblk_t *txdata;
	usb_bulk_req_t *req;
	uint32_t txbufsz;
	uint32_t hdrsize, pkt_align;
	unsigned char *buf;
	uint32_t pos = 0;

	if (sc->ure_gone || !sc->ure_running ||
	    !(sc->ure_flags & URE_FLAG_LINK)) {
		freemsgchain(mp);
		return (NULL);
	}

	mutex_enter(&sc->ure_tx_lock);
	if (sc->ure_tx_busy) {
		mutex_exit(&sc->ure_tx_lock);
		return (mp);
	}
	sc->ure_tx_busy = B_TRUE;
	mutex_exit(&sc->ure_tx_lock);

	txbufsz = sc->ure_txbufsz;
	hdrsize = (sc->ure_flags & URE_FLAG_8157) ?
	    sizeof (ure_txpkt_v2_t) : sizeof (ure_txpkt_t);
	pkt_align = (sc->ure_flags & URE_FLAG_8157) ?
	    URE_8157_BUF_ALIGN : URE_TX_BUF_ALIGN;

	txdata = allocb(txbufsz, BPRI_MED);
	if (txdata == NULL) {
		sc->ure_stat_oerrors++;
		mutex_enter(&sc->ure_tx_lock);
		sc->ure_tx_busy = B_FALSE;
		mutex_exit(&sc->ure_tx_lock);
		return (mp);
	}

	buf = txdata->b_wptr;

	while (mp != NULL) {
		mblk_t *next = mp->b_next;
		mp->b_next = NULL;

		uint32_t mlen = msgsize(mp);

		/* Check if this packet fits */
		uint32_t aligned_pos = P2ROUNDUP(pos, pkt_align);
		if (aligned_pos + hdrsize + mlen > txbufsz) {
			mp->b_next = next;
			break;
		}
		pos = aligned_pos;

		/* Write TX header */
		if (sc->ure_flags & URE_FLAG_8157) {
			ure_txpkt_v2_t txhdr;
			bzero(&txhdr, sizeof (txhdr));
			txhdr.ure_cmdstat = LE_32(
			    URE_TXPKT_TX_FS | URE_TXPKT_TX_LS);
			txhdr.ure_pktlen = LE_32(mlen << 4);
			txhdr.ure_signature = LE_32(
			    URE_TXPKT_SIGNATURE);
			bcopy(&txhdr, buf + pos, sizeof (txhdr));
		} else {
			ure_txpkt_t txhdr;
			bzero(&txhdr, sizeof (txhdr));
			txhdr.ure_pktlen = LE_32(mlen |
			    URE_TXPKT_TX_FS | URE_TXPKT_TX_LS);
			bcopy(&txhdr, buf + pos, sizeof (txhdr));
		}
		pos += hdrsize;

		/* Copy packet data (handle chained mblks) */
		mblk_t *m;
		for (m = mp; m != NULL; m = m->b_cont) {
			uint32_t len = MBLKL(m);
			if (len > 0) {
				bcopy(m->b_rptr, buf + pos, len);
				pos += len;
			}
		}

		sc->ure_stat_opackets++;
		sc->ure_stat_obytes += mlen;

		freemsg(mp);
		mp = next;
	}

	if (pos == 0) {
		/* Nothing was packed */
		freemsg(txdata);
		mutex_enter(&sc->ure_tx_lock);
		sc->ure_tx_busy = B_FALSE;
		mutex_exit(&sc->ure_tx_lock);
		return (mp);
	}

	txdata->b_wptr = txdata->b_rptr + pos;

	/* Submit the bulk OUT transfer */
	req = usb_alloc_bulk_req(sc->ure_dip, 0,
	    USB_FLAGS_NOSLEEP);
	if (req == NULL) {
		sc->ure_stat_oerrors++;
		freemsg(txdata);
		mutex_enter(&sc->ure_tx_lock);
		sc->ure_tx_busy = B_FALSE;
		mutex_exit(&sc->ure_tx_lock);
		return (mp);
	}

	req->bulk_len = pos;
	req->bulk_data = txdata;
	req->bulk_cb = ure_tx_cb;
	req->bulk_exc_cb = ure_tx_cb;
	req->bulk_client_private = (usb_opaque_t)sc;
	req->bulk_timeout = 10;
	req->bulk_attributes = USB_ATTRS_AUTOCLEARING;

	if (usb_pipe_bulk_xfer(sc->ure_bulkout_pipe, req, 0) !=
	    USB_SUCCESS) {
		sc->ure_stat_oerrors++;
		usb_free_bulk_req(req);
		mutex_enter(&sc->ure_tx_lock);
		sc->ure_tx_busy = B_FALSE;
		mutex_exit(&sc->ure_tx_lock);
	}

	return (mp);
}

/* ================================================================ */
/* MAC callbacks                                                    */
/* ================================================================ */

static int
ure_m_stat(void *arg, uint_t stat, uint64_t *val)
{
	ure_softc_t *sc = (ure_softc_t *)arg;

	switch (stat) {
	case MAC_STAT_IFSPEED:
		*val = sc->ure_link_speed;
		break;
	case ETHER_STAT_LINK_DUPLEX:
		*val = sc->ure_link_duplex;
		break;
	case MAC_STAT_MULTIRCV:
		*val = sc->ure_stat_multircv;
		break;
	case MAC_STAT_BRDCSTRCV:
		*val = sc->ure_stat_brdcstrcv;
		break;
	case MAC_STAT_NORCVBUF:
		*val = sc->ure_stat_norcvbuf;
		break;
	case MAC_STAT_IERRORS:
		*val = sc->ure_stat_ierrors;
		break;
	case MAC_STAT_OERRORS:
		*val = sc->ure_stat_oerrors;
		break;
	case MAC_STAT_RBYTES:
		*val = sc->ure_stat_rbytes;
		break;
	case MAC_STAT_IPACKETS:
		*val = sc->ure_stat_ipackets;
		break;
	case MAC_STAT_OBYTES:
		*val = sc->ure_stat_obytes;
		break;
	case MAC_STAT_OPACKETS:
		*val = sc->ure_stat_opackets;
		break;
	default:
		return (ENOTSUP);
	}
	return (0);
}

static int
ure_m_start(void *arg)
{
	ure_softc_t *sc = (ure_softc_t *)arg;
	int error;

	mutex_enter(&sc->ure_lock);

	if (sc->ure_gone) {
		mutex_exit(&sc->ure_lock);
		return (EIO);
	}

	/* NIC reset */
	if (sc->ure_flags & URE_FLAG_8152)
		error = ure_rtl8152_nic_reset(sc);
	else
		error = ure_rtl8153_nic_reset(sc);

	if (error != 0) {
		mutex_exit(&sc->ure_lock);
		return (EIO);
	}

	/* Setup MAC, TX/RX, filters */
	ure_ifmedia_init(sc);
	ure_set_rx_filter(sc);

	sc->ure_running = B_TRUE;

	/* Start RX */
	ure_rx_start(sc);

	mutex_exit(&sc->ure_lock);

	return (0);
}

static void
ure_m_stop(void *arg)
{
	ure_softc_t *sc = (ure_softc_t *)arg;

	mutex_enter(&sc->ure_lock);
	sc->ure_running = B_FALSE;
	mutex_exit(&sc->ure_lock);

	/* Reset the chip to stop RX/TX */
	ure_reset(sc);

	/* Drain pipes */
	usb_pipe_reset(sc->ure_dip, sc->ure_bulkin_pipe,
	    USB_FLAGS_SLEEP, NULL, 0);
	usb_pipe_reset(sc->ure_dip, sc->ure_bulkout_pipe,
	    USB_FLAGS_SLEEP, NULL, 0);
}

static int
ure_m_promisc(void *arg, boolean_t on)
{
	ure_softc_t *sc = (ure_softc_t *)arg;

	mutex_enter(&sc->ure_lock);
	sc->ure_promisc = on;
	if (sc->ure_running)
		ure_set_rx_filter(sc);
	mutex_exit(&sc->ure_lock);

	return (0);
}

static int
ure_m_multicst(void *arg, boolean_t add, const uint8_t *mca)
{
	ure_softc_t *sc = (ure_softc_t *)arg;
	uint32_t crc, bit;

	/*
	 * Compute CRC32-BE hash, use top 6 bits to index
	 * the 64-bit multicast hash table.
	 */
	crc = 0xffffffff;
	for (int i = 0; i < ETHERADDRL; i++) {
		uint8_t c = mca[i];
		for (int j = 0; j < 8; j++) {
			if ((crc ^ c) & 1)
				crc = (crc >> 1) ^ 0xedb88320;
			else
				crc >>= 1;
			c >>= 1;
		}
	}
	/* Use top 6 bits = bit index into 64-bit hash */
	bit = (crc >> 26) & 0x3f;

	mutex_enter(&sc->ure_lock);
	if (add) {
		if (bit < 32)
			sc->ure_mcast_hash[0] |= (1U << bit);
		else
			sc->ure_mcast_hash[1] |=
			    (1U << (bit - 32));
	} else {
		/*
		 * We can't precisely remove a single address
		 * from the hash.  Rebuild would require tracking
		 * the full multicast list.  For now, leave the
		 * bit set — over-matching is harmless.
		 */
	}
	if (sc->ure_running)
		ure_set_rx_filter(sc);
	mutex_exit(&sc->ure_lock);

	return (0);
}

static int
ure_m_unicst(void *arg, const uint8_t *macaddr)
{
	ure_softc_t *sc = (ure_softc_t *)arg;

	mutex_enter(&sc->ure_lock);
	bcopy(macaddr, sc->ure_dev_addr, ETHERADDRL);
	if (sc->ure_running) {
		ure_write_1(sc, URE_PLA_CRWECR,
		    URE_MCU_TYPE_PLA, URE_CRWECR_CONFIG);
		ure_write_mem(sc, URE_PLA_IDR,
		    URE_MCU_TYPE_PLA | URE_BYTE_EN_SIX_BYTES,
		    sc->ure_dev_addr, 8);
		ure_write_1(sc, URE_PLA_CRWECR,
		    URE_MCU_TYPE_PLA, URE_CRWECR_NORAML);
	}
	mutex_exit(&sc->ure_lock);

	return (0);
}

static int
ure_m_getprop(void *arg, const char *pr_name,
    mac_prop_id_t pr_num, uint_t pr_valsize, void *pr_val)
{
	_NOTE(ARGUNUSED(arg, pr_name, pr_num, pr_valsize, pr_val));
	return (ENOTSUP);
}

static int
ure_m_setprop(void *arg, const char *pr_name,
    mac_prop_id_t pr_num, uint_t pr_valsize, const void *pr_val)
{
	_NOTE(ARGUNUSED(arg, pr_name, pr_num, pr_valsize, pr_val));
	return (ENOTSUP);
}

static void
ure_m_propinfo(void *arg, const char *pr_name,
    mac_prop_id_t pr_num, mac_prop_info_handle_t prh)
{
	_NOTE(ARGUNUSED(arg, pr_name, pr_num, prh));
}

static boolean_t
ure_m_getcapab(void *arg, mac_capab_t cap, void *cap_data)
{
	_NOTE(ARGUNUSED(arg));

	switch (cap) {
	case MAC_CAPAB_HCKSUM: {
		uint32_t *flags = (uint32_t *)cap_data;
		/*
		 * TODO: Enable hardware checksum offload.
		 * The hardware supports it, but we need to
		 * wire up the RX/TX checksum paths first.
		 */
		*flags = 0;
		return (B_FALSE);
	}
	default:
		return (B_FALSE);
	}
}

/* ================================================================ */
/* USB pipe management                                              */
/* ================================================================ */

static int
ure_open_pipes(ure_softc_t *sc)
{
	usb_pipe_policy_t policy;
	int ret;

	bzero(&policy, sizeof (policy));
	policy.pp_max_async_reqs = 2;

	ret = usb_pipe_open(sc->ure_dip,
	    &sc->ure_bulkin_ep, &policy,
	    USB_FLAGS_SLEEP, &sc->ure_bulkin_pipe);
	if (ret != USB_SUCCESS) {
		dev_err(sc->ure_dip, CE_WARN,
		    "failed to open bulk IN pipe: %d", ret);
		return (DDI_FAILURE);
	}

	ret = usb_pipe_open(sc->ure_dip,
	    &sc->ure_bulkout_ep, &policy,
	    USB_FLAGS_SLEEP, &sc->ure_bulkout_pipe);
	if (ret != USB_SUCCESS) {
		dev_err(sc->ure_dip, CE_WARN,
		    "failed to open bulk OUT pipe: %d", ret);
		usb_pipe_close(sc->ure_dip,
		    sc->ure_bulkin_pipe, USB_FLAGS_SLEEP,
		    NULL, 0);
		sc->ure_bulkin_pipe = NULL;
		return (DDI_FAILURE);
	}

	return (DDI_SUCCESS);
}

static void
ure_close_pipes(ure_softc_t *sc)
{
	if (sc->ure_bulkin_pipe != NULL) {
		usb_pipe_close(sc->ure_dip,
		    sc->ure_bulkin_pipe,
		    USB_FLAGS_SLEEP, NULL, 0);
		sc->ure_bulkin_pipe = NULL;
	}
	if (sc->ure_bulkout_pipe != NULL) {
		usb_pipe_close(sc->ure_dip,
		    sc->ure_bulkout_pipe,
		    USB_FLAGS_SLEEP, NULL, 0);
		sc->ure_bulkout_pipe = NULL;
	}
}

/* ================================================================ */
/* USB event callbacks                                              */
/* ================================================================ */

static int
ure_disconnect_cb(dev_info_t *dip)
{
	ure_softc_t *sc;
	int instance = ddi_get_instance(dip);

	sc = ddi_get_soft_state(ure_statep, instance);
	if (sc == NULL)
		return (DDI_SUCCESS);

	mutex_enter(&sc->ure_lock);
	sc->ure_gone = B_TRUE;
	sc->ure_running = B_FALSE;
	sc->ure_link_state = LINK_STATE_DOWN;
	mutex_exit(&sc->ure_lock);

	mac_link_update(sc->ure_mh, LINK_STATE_DOWN);

	return (DDI_SUCCESS);
}

static int
ure_reconnect_cb(dev_info_t *dip)
{
	ure_softc_t *sc;
	int instance = ddi_get_instance(dip);

	sc = ddi_get_soft_state(ure_statep, instance);
	if (sc == NULL)
		return (DDI_SUCCESS);

	mutex_enter(&sc->ure_lock);
	sc->ure_gone = B_FALSE;
	mutex_exit(&sc->ure_lock);

	/* Re-initialise chip */
	ure_chip_init(sc);

	return (DDI_SUCCESS);
}

/* ================================================================ */
/* Chip identification                                              */
/* ================================================================ */

static void
ure_chip_init(ure_softc_t *sc)
{
	uint16_t ver;

	sc->ure_phy_read = ure_ocp_reg_read;
	sc->ure_phy_write = ure_ocp_reg_write;

	ver = ure_read_2(sc, URE_PLA_TCR1, URE_MCU_TYPE_PLA) &
	    URE_VERSION_MASK;

	switch (ver) {
	case 0x1030:
		sc->ure_flags = URE_FLAG_8157;
		sc->ure_txbufsz = URE_8156_TX_BUFSZ;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		sc->ure_phy_read = ure_rtl8157_ocp_reg_read;
		sc->ure_phy_write = ure_rtl8157_ocp_reg_write;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8157 (0x%04x)\n", ver);
		break;
	case 0x4c00:
		sc->ure_flags = URE_FLAG_8152;
		sc->ure_chip |= URE_CHIP_VER_4C00;
		sc->ure_rxbufsz = URE_8152_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8152 (0x%04x)\n", ver);
		break;
	case 0x4c10:
		sc->ure_flags = URE_FLAG_8152;
		sc->ure_chip |= URE_CHIP_VER_4C10;
		sc->ure_rxbufsz = URE_8152_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8152 (0x%04x)\n", ver);
		break;
	case 0x5c00:
		sc->ure_chip |= URE_CHIP_VER_5C00;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8153 (0x%04x)\n", ver);
		break;
	case 0x5c10:
		sc->ure_chip |= URE_CHIP_VER_5C10;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8153 (0x%04x)\n", ver);
		break;
	case 0x5c20:
		sc->ure_chip |= URE_CHIP_VER_5C20;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8153 (0x%04x)\n", ver);
		break;
	case 0x5c30:
		sc->ure_chip |= URE_CHIP_VER_5C30;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8153 (0x%04x)\n", ver);
		break;
	case 0x6000:
		sc->ure_flags = URE_FLAG_8153B;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8153B (0x%04x)\n", ver);
		break;
	case 0x6010:
		sc->ure_flags = URE_FLAG_8153B;
		sc->ure_chip |= URE_CHIP_VER_6010;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8153B (0x%04x)\n", ver);
		break;
	case 0x7020:
		sc->ure_flags = URE_FLAG_8156;
		sc->ure_txbufsz = URE_8156_TX_BUFSZ;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8156 (0x%04x)\n", ver);
		break;
	case 0x7030:
		sc->ure_flags = URE_FLAG_8156;
		sc->ure_txbufsz = URE_8156_TX_BUFSZ;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8156 (0x%04x)\n", ver);
		break;
	case 0x7410:
		sc->ure_flags = URE_FLAG_8156B;
		sc->ure_txbufsz = URE_8156_TX_BUFSZ;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8156B (0x%04x)\n", ver);
		break;
	case 0x7420:
		sc->ure_flags = URE_FLAG_8156B;
		sc->ure_chip = URE_CHIP_VER_7420;
		sc->ure_txbufsz = URE_8156_TX_BUFSZ;
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8153D (0x%04x)\n", ver);
		break;
	default:
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_WARN,
		    "unknown chip version 0x%04x", ver);
		break;
	}

	/* Run chip-variant-specific init */
	switch (sc->ure_flags & URE_FLAG_CHIP_MASK) {
	case URE_FLAG_8152:
		ure_rtl8152_init(sc);
		break;
	case URE_FLAG_8153B:
	case URE_FLAG_8156:
	case URE_FLAG_8156B:
		(void) ure_rtl8153b_init(sc);
		break;
	case URE_FLAG_8157:
		(void) ure_rtl8157_init(sc);
		break;
	default:
		/* RTL8153 base */
		(void) ure_rtl8153_init(sc);
		break;
	}
}

/* ================================================================ */
/* Attach / Detach                                                  */
/* ================================================================ */

static void
ure_cleanup(ure_softc_t *sc)
{
	if (sc->ure_attach_seq & URE_ATTACH_LINK_TIMER) {
		ddi_periodic_delete(sc->ure_link_timer);
		sc->ure_attach_seq &= ~URE_ATTACH_LINK_TIMER;
	}

	if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
		(void) mac_unregister(sc->ure_mh);
		sc->ure_attach_seq &= ~URE_ATTACH_MAC_REG;
	}

	if (sc->ure_attach_seq & URE_ATTACH_USB_EVT) {
		usb_unregister_event_cbs(sc->ure_dip, &ure_events);
		sc->ure_attach_seq &= ~URE_ATTACH_USB_EVT;
	}

	ure_close_pipes(sc);

	if (sc->ure_attach_seq & URE_ATTACH_MUTEX) {
		mutex_destroy(&sc->ure_tx_lock);
		mutex_destroy(&sc->ure_lock);
		sc->ure_attach_seq &= ~URE_ATTACH_MUTEX;
	}

	if (sc->ure_attach_seq & URE_ATTACH_DEV_DATA) {
		usb_free_dev_data(sc->ure_dip, sc->ure_dev_data);
		sc->ure_dev_data = NULL;
		sc->ure_attach_seq &= ~URE_ATTACH_DEV_DATA;
	}

	if (sc->ure_attach_seq & URE_ATTACH_USB) {
		usb_client_detach(sc->ure_dip, NULL);
		sc->ure_attach_seq &= ~URE_ATTACH_USB;
	}
}

static int
ure_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	ure_softc_t *sc;
	int instance = ddi_get_instance(dip);
	mac_register_t *macp = NULL;
	usb_ep_data_t *ep_data;
	uint8_t eaddr[8];	/* 4-byte padded */
	int ret;

	switch (cmd) {
	case DDI_ATTACH:
		break;
	case DDI_RESUME:
		sc = ddi_get_soft_state(ure_statep, instance);
		if (sc == NULL)
			return (DDI_FAILURE);
		mutex_enter(&sc->ure_lock);
		sc->ure_gone = B_FALSE;
		mutex_exit(&sc->ure_lock);
		ure_chip_init(sc);
		return (DDI_SUCCESS);
	default:
		return (DDI_FAILURE);
	}

	if (ddi_soft_state_zalloc(ure_statep, instance) !=
	    DDI_SUCCESS)
		return (DDI_FAILURE);

	sc = ddi_get_soft_state(ure_statep, instance);
	sc->ure_dip = dip;
	sc->ure_instance = instance;
	sc->ure_attach_seq = 0;
	sc->ure_gone = B_FALSE;
	sc->ure_running = B_FALSE;
	sc->ure_link_state = LINK_STATE_UNKNOWN;

	/* Step 1: USB client attach */
	ret = usb_client_attach(dip, USBDRV_VERSION, 0);
	if (ret != USB_SUCCESS) {
		dev_err(dip, CE_WARN,
		    "usb_client_attach failed: %d", ret);
		goto fail;
	}
	sc->ure_attach_seq |= URE_ATTACH_USB;

	/* Step 2: Get device data (descriptors) */
	ret = usb_get_dev_data(dip, &sc->ure_dev_data,
	    USB_PARSE_LVL_ALL, 0);
	if (ret != USB_SUCCESS) {
		dev_err(dip, CE_WARN,
		    "usb_get_dev_data failed: %d", ret);
		goto fail;
	}
	sc->ure_attach_seq |= URE_ATTACH_DEV_DATA;
	sc->ure_def_pipe = sc->ure_dev_data->dev_default_ph;

	/* Step 3: Find bulk IN and OUT endpoints */
	ep_data = usb_lookup_ep_data(dip, sc->ure_dev_data,
	    0, 0, 0, USB_EP_ATTR_BULK, USB_EP_DIR_IN);
	if (ep_data == NULL) {
		dev_err(dip, CE_WARN,
		    "no bulk IN endpoint found");
		goto fail;
	}
	sc->ure_bulkin_ep = ep_data->ep_descr;

	ep_data = usb_lookup_ep_data(dip, sc->ure_dev_data,
	    0, 0, 0, USB_EP_ATTR_BULK, USB_EP_DIR_OUT);
	if (ep_data == NULL) {
		dev_err(dip, CE_WARN,
		    "no bulk OUT endpoint found");
		goto fail;
	}
	sc->ure_bulkout_ep = ep_data->ep_descr;

	/* Step 4: Initialise mutexes */
	mutex_init(&sc->ure_lock, NULL, MUTEX_DRIVER,
	    sc->ure_dev_data->dev_iblock_cookie);
	mutex_init(&sc->ure_tx_lock, NULL, MUTEX_DRIVER,
	    sc->ure_dev_data->dev_iblock_cookie);
	sc->ure_attach_seq |= URE_ATTACH_MUTEX;

	/* Step 5: Identify and initialise chip */
	ure_chip_init(sc);
	sc->ure_attach_seq |= URE_ATTACH_CHIP_INIT;

	/* Step 6: Read MAC address */
	if (sc->ure_chip & (URE_CHIP_VER_4C00 |
	    URE_CHIP_VER_4C10))
		ure_read_mem(sc, URE_PLA_IDR, URE_MCU_TYPE_PLA,
		    eaddr, sizeof (eaddr));
	else
		ure_read_mem(sc, URE_PLA_BACKUP,
		    URE_MCU_TYPE_PLA, eaddr, sizeof (eaddr));

	bcopy(eaddr, sc->ure_dev_addr, ETHERADDRL);

	dev_err(dip, CE_CONT,
	    "?MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	    sc->ure_dev_addr[0], sc->ure_dev_addr[1],
	    sc->ure_dev_addr[2], sc->ure_dev_addr[3],
	    sc->ure_dev_addr[4], sc->ure_dev_addr[5]);

	/* Step 7: Open bulk pipes */
	if (ure_open_pipes(sc) != DDI_SUCCESS)
		goto fail;

	/* Step 8: Register USB event callbacks */
	ret = usb_register_event_cbs(dip, &ure_events, 0);
	if (ret != USB_SUCCESS) {
		dev_err(dip, CE_WARN,
		    "usb_register_event_cbs failed: %d", ret);
		goto fail;
	}
	sc->ure_attach_seq |= URE_ATTACH_USB_EVT;

	/* Step 9: Register with MAC framework */
	macp = mac_alloc(MAC_VERSION);
	if (macp == NULL) {
		dev_err(dip, CE_WARN, "mac_alloc failed");
		goto fail;
	}

	macp->m_type_ident = MAC_PLUGIN_IDENT_ETHER;
	macp->m_driver = sc;
	macp->m_dip = dip;
	macp->m_src_addr = sc->ure_dev_addr;
	macp->m_callbacks = &ure_mac_callbacks;
	macp->m_min_sdu = 0;
	macp->m_max_sdu = ETHERMTU;
	macp->m_margin = VLAN_TAGSZ;

	ret = mac_register(macp, &sc->ure_mh);
	mac_free(macp);
	macp = NULL;

	if (ret != 0) {
		dev_err(dip, CE_WARN,
		    "mac_register failed: %d", ret);
		goto fail;
	}
	sc->ure_attach_seq |= URE_ATTACH_MAC_REG;

	/* Step 10: Start link polling timer (1 second) */
	sc->ure_link_timer = ddi_periodic_add(
	    ure_link_check, sc, 1000000000ULL, DDI_IPL_0);
	sc->ure_attach_seq |= URE_ATTACH_LINK_TIMER;

	ddi_report_dev(dip);

	return (DDI_SUCCESS);

fail:
	if (macp != NULL)
		mac_free(macp);
	ure_cleanup(sc);
	ddi_soft_state_free(ure_statep, instance);
	return (DDI_FAILURE);
}

static int
ure_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	ure_softc_t *sc;
	int instance = ddi_get_instance(dip);

	sc = ddi_get_soft_state(ure_statep, instance);
	if (sc == NULL)
		return (DDI_FAILURE);

	switch (cmd) {
	case DDI_DETACH:
		break;
	case DDI_SUSPEND:
		mutex_enter(&sc->ure_lock);
		sc->ure_gone = B_TRUE;
		mutex_exit(&sc->ure_lock);
		return (DDI_SUCCESS);
	default:
		return (DDI_FAILURE);
	}

	/* Attempt MAC unregister first — may fail if busy */
	if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
		if (mac_disable(sc->ure_mh) != 0)
			return (DDI_FAILURE);
	}

	ure_cleanup(sc);
	ddi_soft_state_free(ure_statep, instance);

	return (DDI_SUCCESS);
}
