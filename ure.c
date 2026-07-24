/*	$OpenBSD: if_ure.c,v 1.37 2025/06/04 00:06:17 jsg Exp $	*/
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
 * - RTL8152  (USB 2.0, 10/100 Mbps)
 * - RTL8153  (USB 3.0, 10/100/1000 Mbps)
 * - RTL8153B (USB 3.0, 10/100/1000 Mbps)
 * - RTL8156  (USB 3.0, 10/100/1000/2500 Mbps)
 * - RTL8156B (USB 3.0, 10/100/1000/2500 Mbps)
 * - RTL8157  (USB 3.0, 10/100/1000/2500/5000 Mbps)
 *
 * Key features:
 * - RX aggregation (multiple packets per USB bulk IN transfer)
 * - TX aggregation (multiple packets per USB bulk OUT transfer)
 * - IPv4/IPv6/TCP/UDP hardware checksum offload (RX and TX)
 * - TCP segmentation offload (TSO/LSO) (not for RTL8152)
 *
 * Reference implementations:
 * - OpenBSD if_ure.c (v1.37, Kevin Lo, Jonathon Fletcher)
 * - FreeBSD if_ure.c (Kevin Lo)
 * - FreeBSD spurious link-down BMSR workaround (PR 252165)
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
#include <sys/dlpi.h>
#include <sys/pattr.h>
#include <usb/usba/usbai_version.h>
#include <sys/usb/usba.h>
#include <sys/usb/usba/usba_types.h>
#include <sys/crc32.h>
#include <sys/vlan.h>
#include <sys/strsubr.h>
#include <sys/atomic.h>
#include <inet/ip.h>
#include <inet/ip6.h>

#include "urereg.h"
#include "ure.h"

/* Debug logging: set to 1 to enable verbose attach/detach logging */
#define	URE_DEBUG	0

static uint_t ure_errlevel = USB_LOG_L4;
static uint_t ure_errmask = (uint_t)-1;
static uint_t ure_instance_debug = (uint_t)-1;

#if URE_DEBUG
#define	URE_DPRINTF(sc, fmt, ...)	\
	dev_err((sc)->ure_dip, CE_CONT, "?" fmt "\n", ##__VA_ARGS__)
#else
#define	URE_DPRINTF(sc, fmt, ...)
#endif

/*
 * RX checksum debug: when non-zero, print raw RX header DWORDs for the
 * first N received packets to verify field mapping.  Set via mdb:
 *   ure_rxcsum_debug/W 0t10
 */
static volatile int ure_rxcsum_debug = 0;

static const uint8_t ure_bcast_addr[ETHERADDRL] =
	{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

typedef struct ure_tx_offload {
	uint32_t	uto_opts1;
	uint32_t	uto_opts2;
	boolean_t	uto_sw_csum;
	uint32_t	uto_sw_start;
	uint32_t	uto_sw_stuff;
} ure_tx_offload_t;

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
static void	ure_txc_submit(ure_softc_t *, ure_tx_chain_t *);
static void	ure_txc_flush_locked(ure_softc_t *);
static boolean_t	ure_tx_offload(ure_softc_t *, mblk_t **,
		    ure_tx_offload_t *);
static boolean_t	ure_tx_sw_csum(const ure_tx_offload_t *, uchar_t *,
		    uint32_t);
static void	ure_txc_discard(ure_softc_t *);
static void	ure_txc_timeout(void *);

static int	ure_chip_init(ure_softc_t *);
static void	ure_chip_uninit(ure_softc_t *);
static int	ure_rtl8152_init(ure_softc_t *);
static int	ure_rtl8153_init(ure_softc_t *);
static int	ure_rtl8153b_init(ure_softc_t *);
static int	ure_rtl8157_init(ure_softc_t *);
static int	ure_rtl8152_nic_reset(ure_softc_t *);
static int	ure_rtl8153_nic_reset(ure_softc_t *);
static int	ure_rtl8153_phy_status(ure_softc_t *, int, uint16_t *);
static int	ure_wait_for_flash(ure_softc_t *);
static int	ure_reset_bmu(ure_softc_t *);
static int	ure_disable_teredo(ure_softc_t *);
static int	ure_reset(ure_softc_t *);
static int	ure_phy_powerdown(ure_softc_t *);
static int	ure_phy_powerup(ure_softc_t *);
static int	ure_rxvlan(ure_softc_t *);
static int	ure_set_rx_filter(ure_softc_t *);
static int	ure_ifmedia_init(ure_softc_t *);
static void	ure_link_check(void *);
static void	ure_link_timer_start(ure_softc_t *);
static void	ure_link_timer_stop(ure_softc_t *);

static int	ure_disconnect_cb(dev_info_t *);
static int	ure_reconnect_cb(dev_info_t *);
static int	ure_open_pipes(ure_softc_t *);
static void	ure_close_pipes(ure_softc_t *);

/* TX chain kmem_cache constructor/destructor */
static int	ure_tx_chain_construct(void *, void *, int);
static void	ure_tx_chain_destroy(void *, void *);

/* Register access and PHY primitives */
static int	ure_ctl(ure_softc_t *, uint8_t, uint16_t, uint16_t,
		    void *, int);
static int	ure_read_mem(ure_softc_t *, uint16_t, uint16_t,
		    void *, int);
static int	ure_write_mem(ure_softc_t *, uint16_t, uint16_t,
		    void *, int);
static int	ure_read_1(ure_softc_t *, uint16_t, uint16_t,
		    uint8_t *);
static int	ure_read_2(ure_softc_t *, uint16_t, uint16_t,
		    uint16_t *);
static int	ure_read_4(ure_softc_t *, uint16_t, uint16_t,
		    uint32_t *);
static int	ure_write_1(ure_softc_t *, uint16_t, uint16_t, uint32_t);
static int	ure_write_2(ure_softc_t *, uint16_t, uint16_t, uint32_t);
static int	ure_write_4(ure_softc_t *, uint16_t, uint16_t, uint32_t);
static int	ure_setbit_1(ure_softc_t *, uint16_t, uint16_t, uint8_t);
static int	ure_setbit_2(ure_softc_t *, uint16_t, uint16_t, uint16_t);
static int	ure_clrbit_1(ure_softc_t *, uint16_t, uint16_t, uint8_t);
static int	ure_clrbit_2(ure_softc_t *, uint16_t, uint16_t, uint16_t);
static int	ure_clrbit_4(ure_softc_t *, uint16_t, uint16_t, uint32_t);
static int	ure_ocp_reg_read(ure_softc_t *, uint16_t, uint16_t *);
static int	ure_ocp_reg_write(ure_softc_t *, uint16_t, uint16_t);
static int	ure_ocp_cmd_read(ure_softc_t *, uint16_t, int, uint32_t *);
static int	ure_ocp_cmd_write(ure_softc_t *, uint16_t, int, uint32_t);
static int	ure_ocp_cmd_setbit(ure_softc_t *, uint16_t, int, uint32_t);
static int	ure_ocp_cmd_clrbit(ure_softc_t *, uint16_t, int, uint32_t);
static int	ure_rtl8157_ocp_reg_read(ure_softc_t *, uint16_t,
		    uint16_t *);
static int	ure_rtl8157_ocp_reg_write(ure_softc_t *, uint16_t,
		    uint16_t);

/* Soft state */
static void	*ure_statep;

/*
 * Get the actual USB connection speed for this device.
 * Returns one of USBA_LOW_SPEED_DEV, USBA_FULL_SPEED_DEV,
 * USBA_HIGH_SPEED_DEV, or USBA_SUPER_SPEED_DEV.
 *
 * There is no public USBA API to query device speed, so we access
 * usba_device_t internals directly.  This is a common pattern in
 * illumos USB drivers (see usbgem, usbecm, etc.).
 */
static usb_port_status_t
ure_dev_speed(ure_softc_t *sc)
{
	usba_device_t *ud = usba_get_usba_device(sc->ure_dip);
	return (ud->usb_port_status);
}

/* USB event callbacks */
static usb_event_t ure_events = {
	.disconnect_event_handler = ure_disconnect_cb,
	.reconnect_event_handler = ure_reconnect_cb,
	.pre_suspend_event_handler = NULL,
	.post_resume_event_handler = NULL,
};

/* MAC callbacks */
static mac_callbacks_t ure_mac_callbacks = {
	.mc_callbacks	= MC_GETCAPAB | MC_SETPROP | MC_GETPROP | MC_PROPINFO,
	.mc_getstat	= ure_m_stat,
	.mc_start	= ure_m_start,
	.mc_stop	= ure_m_stop,
	.mc_setpromisc	= ure_m_promisc,
	.mc_multicst	= ure_m_multicst,
	.mc_unicst	= ure_m_unicst,
	.mc_tx		= ure_m_tx,
	.mc_getcapab	= ure_m_getcapab,
	.mc_setprop	= ure_m_setprop,
	.mc_getprop	= ure_m_getprop,
	.mc_propinfo	= ure_m_propinfo,
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
	.drv_modops	= &mod_driverops,
	.drv_linkinfo	= "RTL815[2367] USB Ethernet",
	.drv_dev_ops	= &ure_dev_ops,
};

static struct modlinkage ure_modlinkage = {
	.ml_rev		= MODREV_1,
	.ml_linkage	= { &ure_modldrv, NULL }
};

int
_init(void)
{
	major_t major;
	int err;

	if ((err = ddi_soft_state_init(&ure_statep,
	    sizeof (ure_softc_t), 1)) != 0) {
		return (err);
	}

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

	if ((err = mod_remove(&ure_modlinkage)) != 0) {
		return (err);
	}

	mac_fini_ops(&ure_dev_ops);
	ddi_soft_state_fini(&ure_statep);
	return (0);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&ure_modlinkage, modinfop));
}

/*
 * Register access primitives
 */

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

	if (sc->ure_gone) {
		return (USB_FAILURE);
	}

	bzero(&setup, sizeof (setup));
	if (rw == URE_CTL_WRITE) {
		setup.bmRequestType = USB_DEV_REQ_HOST_TO_DEV |
		    USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV;
	} else {
		setup.bmRequestType = USB_DEV_REQ_DEV_TO_HOST |
		    USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_RCPT_DEV;
	}
	setup.bRequest = USB_REQ_SET_ADDRESS;
	setup.wValue = val;
	setup.wIndex = index;
	setup.wLength = (uint16_t)len;
	setup.attrs = USB_ATTRS_NONE;

	if (rw == URE_CTL_WRITE && buf != NULL && len > 0) {
		data = allocb(len, BPRI_MED);
		if (data == NULL) {
			return (USB_FAILURE);
		}
		bcopy(buf, data->b_wptr, len);
		data->b_wptr += len;
	}

	ret = usb_pipe_ctrl_xfer_wait(sc->ure_def_pipe,
	    &setup, (rw == URE_CTL_READ) ? &data : &data,
	    &cr, &cb_flags, 0);

	if (ret == USB_SUCCESS && rw == URE_CTL_READ &&
	    data != NULL && buf != NULL) {
		int actual = MBLKL(data);
		if (actual > len) {
			actual = len;
		}
		bcopy(data->b_rptr, buf, actual);
	}

	if (data != NULL) {
		freemsg(data);
	}

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

static int
ure_read_1(ure_softc_t *sc, uint16_t reg, uint16_t index, uint8_t *valp)
{
	uint32_t val;
	uint8_t temp[4] = { 0 };
	uint8_t shift;
	int err;

	shift = (reg & 3) << 3;
	reg &= ~3;

	err = ure_read_mem(sc, reg, index, temp, 4);
	if (err != USB_SUCCESS) {
		*valp = 0;
		return (err);
	}
	val = LE_32(*(uint32_t *)(void *)temp);
	val >>= shift;

	*valp = (uint8_t)(val & 0xff);
	return (USB_SUCCESS);
}

static int
ure_read_2(ure_softc_t *sc, uint16_t reg, uint16_t index, uint16_t *valp)
{
	uint32_t val;
	uint8_t temp[4] = { 0 };
	uint8_t shift;
	int err;

	shift = (reg & 2) << 3;
	reg &= ~3;

	err = ure_read_mem(sc, reg, index, temp, 4);
	if (err != USB_SUCCESS) {
		*valp = 0;
		return (err);
	}
	val = LE_32(*(uint32_t *)(void *)temp);
	val >>= shift;

	*valp = (uint16_t)(val & 0xffff);
	return (USB_SUCCESS);
}

static int
ure_read_4(ure_softc_t *sc, uint16_t reg, uint16_t index, uint32_t *valp)
{
	uint8_t temp[4] = { 0 };
	int err;

	err = ure_read_mem(sc, reg, index, temp, 4);
	if (err != USB_SUCCESS) {
		*valp = 0;
		return (err);
	}
	*valp = LE_32(*(uint32_t *)(void *)temp);
	return (USB_SUCCESS);
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

static int
ure_setbit_1(ure_softc_t *sc, uint16_t reg, uint16_t index, uint8_t bits)
{
	uint8_t val;
	int err;

	if ((err = ure_read_1(sc, reg, index, &val)) != USB_SUCCESS) {
		return (err);
	}
	return (ure_write_1(sc, reg, index, val | bits));
}

static int
ure_setbit_2(ure_softc_t *sc, uint16_t reg, uint16_t index, uint16_t bits)
{
	uint16_t val;
	int err;

	if ((err = ure_read_2(sc, reg, index, &val)) != USB_SUCCESS) {
		return (err);
	}
	return (ure_write_2(sc, reg, index, val | bits));
}
static int
ure_clrbit_1(ure_softc_t *sc, uint16_t reg, uint16_t index, uint8_t bits)
{
	uint8_t val;
	int err;

	if ((err = ure_read_1(sc, reg, index, &val)) != USB_SUCCESS) {
		return (err);
	}
	return (ure_write_1(sc, reg, index, val & ~bits));
}

static int
ure_clrbit_2(ure_softc_t *sc, uint16_t reg, uint16_t index, uint16_t bits)
{
	uint16_t val;
	int err;

	if ((err = ure_read_2(sc, reg, index, &val)) != USB_SUCCESS) {
		return (err);
	}
	return (ure_write_2(sc, reg, index, val & ~bits));
}

static int
ure_clrbit_4(ure_softc_t *sc, uint16_t reg, uint16_t index, uint32_t bits)
{
	uint32_t val;
	int err;

	if ((err = ure_read_4(sc, reg, index, &val)) != USB_SUCCESS) {
		return (err);
	}
	return (ure_write_4(sc, reg, index, val & ~bits));
}

/*
 * PHY access via OCP (On-Chip Protocol)
 */

static int
ure_ocp_reg_read(ure_softc_t *sc, uint16_t addr, uint16_t *valp)
{
	uint16_t base = addr & 0xf000;
	uint16_t reg;
	int err;

	if (sc->ure_ocp_base != base) {
		if ((err = ure_write_2(sc, URE_PLA_OCP_GPHY_BASE,
		    URE_MCU_TYPE_PLA, base)) != USB_SUCCESS) {
			*valp = 0;
			return (err);
		}
		sc->ure_ocp_base = base;
	}
	reg = (addr & 0x0fff) | 0xb000;

	return (ure_read_2(sc, reg, URE_MCU_TYPE_PLA, valp));
}

static int
ure_ocp_reg_write(ure_softc_t *sc, uint16_t addr, uint16_t data)
{
	uint16_t base = addr & 0xf000;
	uint16_t reg;
	int err;

	if (sc->ure_ocp_base != base) {
		if ((err = ure_write_2(sc, URE_PLA_OCP_GPHY_BASE,
		    URE_MCU_TYPE_PLA, base)) != USB_SUCCESS) {
			return (err);
		}
		sc->ure_ocp_base = base;
	}
	reg = (addr & 0x0fff) | 0xb000;

	return (ure_write_2(sc, reg, URE_MCU_TYPE_PLA, data));
}

/*
 * RTL8157 OCP command interface.
 *
 * The RTL8157 has a separate 32-bit command register block for BMU and
 * IP-layer access.  Commands are issued through URE_USB_CMD_ADDR,
 * URE_USB_CMD_DATA, and URE_USB_CMD.  The busy flag in URE_USB_CMD
 * is polled before and after each command.
 */
static int
ure_ocp_cmd_read(ure_softc_t *sc, uint16_t addr, int type, uint32_t *valp)
{
	uint16_t cmd, busy;
	int err, i;

	cmd = (type == URE_CMD_TYPE_BMU) ? URE_CMD_BMU : URE_CMD_IP;

	/* Wait for any previous command to finish */
	for (i = 0; i < 10; i++) {
		if ((err = ure_read_2(sc, URE_USB_CMD,
		    URE_MCU_TYPE_USB, &busy)) != USB_SUCCESS) {
			*valp = 0;
			return (err);
		}
		if (!(busy & URE_CMD_BUSY)) {
			break;
		}
		delay(drv_usectohz(1000));
	}
	if (i == 10) {
		dev_err(sc->ure_dip, CE_WARN, "OCP cmd read pre-busy timeout");
		*valp = 0;
		return (USB_FAILURE);
	}

	if ((err = ure_write_2(sc, URE_USB_CMD_ADDR,
	    URE_MCU_TYPE_USB, addr)) != USB_SUCCESS) {
		*valp = 0;
		return (err);
	}
	if ((err = ure_write_2(sc, URE_USB_CMD,
	    URE_MCU_TYPE_USB, cmd | URE_CMD_BUSY)) != USB_SUCCESS) {
		*valp = 0;
		return (err);
	}

	/* Wait for command completion */
	for (i = 0; i < 10; i++) {
		if ((err = ure_read_2(sc, URE_USB_CMD,
		    URE_MCU_TYPE_USB, &busy)) != USB_SUCCESS) {
			*valp = 0;
			return (err);
		}
		if (!(busy & URE_CMD_BUSY)) {
			break;
		}
		delay(drv_usectohz(1000));
	}
	if (i == 10) {
		dev_err(sc->ure_dip, CE_WARN, "OCP cmd read post-busy timeout");
		*valp = 0;
		return (USB_FAILURE);
	}

	return (ure_read_4(sc, URE_USB_CMD_DATA, URE_MCU_TYPE_USB, valp));
}

static int
ure_ocp_cmd_write(ure_softc_t *sc, uint16_t addr, int type, uint32_t data)
{
	uint16_t cmd, busy;
	int err, i;

	cmd = (type == URE_CMD_TYPE_BMU) ? URE_CMD_BMU : URE_CMD_IP;

	/* Wait for any previous command to finish */
	for (i = 0; i < 10; i++) {
		if ((err = ure_read_2(sc, URE_USB_CMD,
		    URE_MCU_TYPE_USB, &busy)) != USB_SUCCESS) {
			return (err);
		}
		if (!(busy & URE_CMD_BUSY)) {
			break;
		}
		delay(drv_usectohz(1000));
	}
	if (i == 10) {
		dev_err(sc->ure_dip, CE_WARN, "OCP cmd write pre-busy timeout");
		return (USB_FAILURE);
	}

	if ((err = ure_write_4(sc, URE_USB_CMD_DATA,
	    URE_MCU_TYPE_USB, data)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_USB_CMD_ADDR,
	    URE_MCU_TYPE_USB, addr)) != USB_SUCCESS) {
		return (err);
	}
	return (ure_write_2(sc, URE_USB_CMD,
	    URE_MCU_TYPE_USB, cmd | URE_CMD_BUSY | URE_CMD_WRITE));
}

static int
ure_ocp_cmd_setbit(ure_softc_t *sc, uint16_t addr, int type, uint32_t bits)
{
	uint32_t val;
	int err;

	if ((err = ure_ocp_cmd_read(sc, addr, type, &val)) != USB_SUCCESS) {
		return (err);
	}
	return (ure_ocp_cmd_write(sc, addr, type, val | bits));
}

static int
ure_ocp_cmd_clrbit(ure_softc_t *sc, uint16_t addr, int type, uint32_t bits)
{
	uint32_t val;
	int err;

	if ((err = ure_ocp_cmd_read(sc, addr, type, &val)) != USB_SUCCESS) {
		return (err);
	}
	return (ure_ocp_cmd_write(sc, addr, type, val & ~bits));
}

/*
 * RTL8157 uses a separate TGPHY interface for PHY access.
 * The busy-waits use drv_usecwait() rather than delay() so
 * that these functions are safe in quiesce/panic context.
 */
static int
ure_rtl8157_ocp_reg_read(ure_softc_t *sc, uint16_t addr,
    uint16_t *valp)
{
	uint16_t cmd;
	int err;
	int i;

	for (i = 0; i < 10; i++) {
		if ((err = ure_read_2(sc, URE_USB_TGPHY_CMD,
		    URE_MCU_TYPE_USB, &cmd)) != USB_SUCCESS) {
			*valp = 0;
			return (err);
		}
		if (!(cmd & URE_TGPHY_CMD_BUSY)) {
			break;
		}
		drv_usecwait(1000);
	}
	if (i == 10) {
		dev_err(sc->ure_dip, CE_WARN,
		    "PHY read timeout (pre)");
		*valp = 0xffff;
		return (USB_FAILURE);
	}

	if ((err = ure_write_2(sc, URE_USB_TGPHY_ADDR,
	    URE_MCU_TYPE_USB, addr)) != USB_SUCCESS) {
		*valp = 0;
		return (err);
	}
	if ((err = ure_write_2(sc, URE_USB_TGPHY_CMD,
	    URE_MCU_TYPE_USB,
	    URE_TGPHY_CMD_BUSY)) != USB_SUCCESS) {
		*valp = 0;
		return (err);
	}

	for (i = 0; i < 10; i++) {
		if ((err = ure_read_2(sc, URE_USB_TGPHY_CMD,
		    URE_MCU_TYPE_USB, &cmd)) != USB_SUCCESS) {
			*valp = 0;
			return (err);
		}
		if (!(cmd & URE_TGPHY_CMD_BUSY)) {
			break;
		}
		drv_usecwait(1000);
	}
	if (i == 10) {
		dev_err(sc->ure_dip, CE_WARN,
		    "PHY read timeout (post)");
		*valp = 0xffff;
		return (USB_FAILURE);
	}

	return (ure_read_2(sc, URE_USB_TGPHY_DATA,
	    URE_MCU_TYPE_USB, valp));
}

static int
ure_rtl8157_ocp_reg_write(ure_softc_t *sc, uint16_t addr,
    uint16_t data)
{
	uint16_t cmd;
	int err;
	int i;

	if ((err = ure_write_2(sc, URE_USB_TGPHY_DATA,
	    URE_MCU_TYPE_USB, data)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_USB_TGPHY_ADDR,
	    URE_MCU_TYPE_USB, addr)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_USB_TGPHY_CMD,
	    URE_MCU_TYPE_USB,
	    URE_TGPHY_CMD_BUSY | URE_TGPHY_CMD_WRITE)) != USB_SUCCESS) {
		return (err);
	}

	for (i = 0; i < 10; i++) {
		if ((err = ure_read_2(sc, URE_USB_TGPHY_CMD,
		    URE_MCU_TYPE_USB, &cmd)) != USB_SUCCESS) {
			return (err);
		}
		if (!(cmd & URE_TGPHY_CMD_BUSY)) {
			break;
		}
		drv_usecwait(1000);
	}
	if (i == 10) {
		/*
		 * The register write was sent to the device but the
		 * TGPHY busy flag did not clear.  This can be transient,
		 * so log and continue rather than aborting init.
		 *
		 * Unlike ure_rtl8157_ocp_reg_read, which returns
		 * USB_FAILURE on timeout (no data to trust), writes
		 * return USB_SUCCESS here because the data was already
		 * dispatched via USB control transfers; only the
		 * completion poll timed out.
		 */
		dev_err(sc->ure_dip, CE_WARN, "PHY write timeout");
	}
	return (USB_SUCCESS);
}

/*
 * Convenience wrappers that go through the per-chip function
 * pointers.
 */
static inline int
ure_phy_read(ure_softc_t *sc, uint16_t addr, uint16_t *valp)
{
	return (sc->ure_phy_read(sc, addr, valp));
}

static inline int
ure_phy_write(ure_softc_t *sc, uint16_t addr, uint16_t data)
{
	return (sc->ure_phy_write(sc, addr, data));
}

/*
 * Power down the PHY by setting BMCR PDOWN.  This drops the link
 * on the wire so that the connected switch sees the port go down.
 */
static int
ure_phy_powerdown(ure_softc_t *sc)
{
	uint16_t val;
	int err;

	if ((err = ure_phy_read(sc, URE_OCP_BMCR, &val)) != 0) {
		return (err);
	}
	if (!(val & URE_OCP_BMCR_PDOWN)) {
		val |= URE_OCP_BMCR_PDOWN;
		if ((err = ure_phy_write(sc, URE_OCP_BMCR,
		    val)) != 0) {
			return (err);
		}
	}
	return (USB_SUCCESS);
}

/*
 * Clear BMCR PDOWN and restart auto-negotiation.  Called from the
 * NIC reset path (ure_m_start) before enabling the receiver.
 */
static int
ure_phy_powerup(ure_softc_t *sc)
{
	uint16_t val;
	int err;

	if ((err = ure_phy_read(sc, URE_OCP_BMCR, &val)) != 0) {
		return (err);
	}
	if (val & URE_OCP_BMCR_PDOWN) {
		val &= ~URE_OCP_BMCR_PDOWN;
		val |= URE_OCP_BMCR_ANE | URE_OCP_BMCR_RSAN;
		if ((err = ure_phy_write(sc, URE_OCP_BMCR,
		    val)) != 0) {
			return (err);
		}
	}
	return (USB_SUCCESS);
}


/*
 * Chip helper functions
 */

static boolean_t
ure_has_2500fdx(ure_softc_t *sc)
{
	if (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8157)) {
		return (B_TRUE);
	}

	if ((sc->ure_flags & URE_FLAG_8156B) &&
	    !(sc->ure_chip & URE_CHIP_VER_7420)) {
		return (B_TRUE);
	}

	return (B_FALSE);
}

static boolean_t
ure_has_5000fdx(ure_softc_t *sc)
{
	return ((sc->ure_flags & URE_FLAG_8157) != 0);
}

static int
ure_disable_teredo(ure_softc_t *sc)
{
	int err;

	if (sc->ure_flags & (URE_FLAG_8153B | URE_FLAG_8156 |
	    URE_FLAG_8156B | URE_FLAG_8157)) {
		if ((err = ure_write_1(sc, URE_PLA_TEREDO_CFG,
		    URE_MCU_TYPE_PLA, 0xff)) != USB_SUCCESS) {
			return (err);
		}
	} else {
		if ((err = ure_clrbit_2(sc, URE_PLA_TEREDO_CFG,
		    URE_MCU_TYPE_PLA,
		    URE_TEREDO_SEL | URE_TEREDO_RS_EVENT_MASK |
		    URE_OOB_TEREDO_EN)) != USB_SUCCESS) {
			return (err);
		}
	}
	if ((err = ure_write_2(sc, URE_PLA_WDT6_CTRL,
	    URE_MCU_TYPE_PLA, URE_WDT6_SET_MODE)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_PLA_REALWOW_TIMER,
	    URE_MCU_TYPE_PLA, 0)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_4(sc, URE_PLA_TEREDO_TIMER,
	    URE_MCU_TYPE_PLA, 0)) != USB_SUCCESS) {
		return (err);
	}
	return (USB_SUCCESS);
}

static int
ure_reset(ure_softc_t *sc)
{
	uint8_t reg8;
	int err;
	int i;

	if (sc->ure_flags & URE_FLAG_8157) {
		if ((err = ure_clrbit_1(sc, URE_PLA_CR,
		    URE_MCU_TYPE_PLA,
		    URE_CR_TE | URE_CR_RE)) != USB_SUCCESS) {
			return (err);
		}
	} else if (sc->ure_flags & URE_FLAG_8156) {
		if ((err = ure_clrbit_1(sc, URE_PLA_CR,
		    URE_MCU_TYPE_PLA, URE_CR_TE)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_clrbit_2(sc, URE_USB_BMU_RESET,
		    URE_MCU_TYPE_USB,
		    URE_BMU_RESET_EP_IN)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_setbit_2(sc, URE_USB_USB_CTRL,
		    URE_MCU_TYPE_USB,
		    URE_CDC_ECM_EN)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_clrbit_1(sc, URE_PLA_CR,
		    URE_MCU_TYPE_PLA, URE_CR_RE)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_setbit_2(sc, URE_USB_BMU_RESET,
		    URE_MCU_TYPE_USB,
		    URE_BMU_RESET_EP_IN)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_clrbit_2(sc, URE_USB_USB_CTRL,
		    URE_MCU_TYPE_USB,
		    URE_CDC_ECM_EN)) != USB_SUCCESS) {
			return (err);
		}
	} else {
		if ((err = ure_write_1(sc, URE_PLA_CR,
		    URE_MCU_TYPE_PLA,
		    URE_CR_RST)) != USB_SUCCESS) {
			return (err);
		}
		for (i = 0; i < URE_TIMEOUT; i++) {
			if ((err = ure_read_1(sc, URE_PLA_CR,
			    URE_MCU_TYPE_PLA,
			    &reg8)) != USB_SUCCESS) {
				return (err);
			}
			if (!(reg8 & URE_CR_RST)) {
				break;
			}
			drv_usecwait(100);
		}
		if (i == URE_TIMEOUT) {
			dev_err(sc->ure_dip, CE_WARN,
			    "reset never completed");
			return (USB_FAILURE);
		}
		/*
		 * PLA_CR_RST clears all PLA registers including
		 * PLA_OCP_GPHY_BASE, so invalidate the cached
		 * OCP base to force the next PHY access to
		 * reprogram it.
		 */
		sc->ure_ocp_base = 0;
	}
	return (USB_SUCCESS);
}

static int
ure_reset_bmu(ure_softc_t *sc)
{
	uint8_t reg;
	int err;

	if (sc->ure_flags & URE_FLAG_8157) {
		if ((err = ure_ocp_cmd_setbit(sc, URE_PLA_BMU_RX_IN,
		    URE_CMD_TYPE_BMU, URE_PLA_BMU_RESET_0X02)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_ocp_cmd_setbit(sc, URE_PLA_BMU_RX_OUT,
		    URE_CMD_TYPE_BMU, URE_PLA_BMU_RESET_0X01)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_ocp_cmd_setbit(sc, URE_PLA_BMU_RX_IN,
		    URE_CMD_TYPE_BMU, URE_PLA_BMU_RESET_0X01)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_ocp_cmd_setbit(sc, URE_PLA_BMU_TX_IN,
		    URE_CMD_TYPE_BMU, URE_PLA_BMU_RESET_0X02)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_ocp_cmd_setbit(sc, URE_PLA_BMU_TX_OUT,
		    URE_CMD_TYPE_BMU, URE_PLA_BMU_RESET_0X01)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_ocp_cmd_setbit(sc, URE_PLA_BMU_TX_IN,
		    URE_CMD_TYPE_BMU, URE_PLA_BMU_RESET_0X01)) != USB_SUCCESS) {
			return (err);
		}
		return (USB_SUCCESS);
	}

	if ((err = ure_read_1(sc, URE_USB_BMU_RESET,
	    URE_MCU_TYPE_USB, &reg)) != USB_SUCCESS) {
		return (err);
	}
	reg &= ~(URE_BMU_RESET_EP_IN | URE_BMU_RESET_EP_OUT);
	if ((err = ure_write_1(sc, URE_USB_BMU_RESET,
	    URE_MCU_TYPE_USB, reg)) != USB_SUCCESS) {
		return (err);
	}
	reg |= URE_BMU_RESET_EP_IN | URE_BMU_RESET_EP_OUT;
	return (ure_write_1(sc, URE_USB_BMU_RESET,
	    URE_MCU_TYPE_USB, reg));
}

static int
ure_rtl8153_phy_status(ure_softc_t *sc, int desired, uint16_t *regp)
{
	uint16_t reg;
	int err;
	int i;

	for (i = 0; i < 500; i++) {
		if ((err = ure_phy_read(sc, URE_OCP_PHY_STATUS,
		    &reg)) != 0) {
			*regp = 0;
			return (err);
		}
		reg &= URE_PHY_STAT_MASK;
		if (desired) {
			if (reg == (uint16_t)desired) {
				break;
			}
		} else {
			if (reg == URE_PHY_STAT_LAN_ON ||
			    reg == URE_PHY_STAT_PWRDN ||
			    reg == URE_PHY_STAT_EXT_INIT) {
				break;
			}
		}
		delay(drv_usectohz(20000));
	}
	if (i == 500) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for PHY to stabilize");
		*regp = reg;
		return (USB_FAILURE);
	}

	*regp = reg;
	return (USB_SUCCESS);
}

static int
ure_wait_for_flash(ure_softc_t *sc)
{
	uint16_t reg16;
	int err;
	int i;

	if ((err = ure_read_2(sc, URE_PLA_GPHY_CTRL,
	    URE_MCU_TYPE_PLA, &reg16)) != USB_SUCCESS) {
		return (USB_FAILURE);
	}
	if (reg16 & URE_GPHY_FLASH) {
		if ((err = ure_read_2(sc, URE_USB_GPHY_CTRL,
		    URE_MCU_TYPE_USB, &reg16)) != USB_SUCCESS) {
			return (USB_FAILURE);
		}
		if (!(reg16 & URE_BYPASS_FLASH)) {
			for (i = 0; i < 100; i++) {
				if ((err = ure_read_2(sc,
				    URE_USB_GPHY_CTRL,
				    URE_MCU_TYPE_USB,
				    &reg16)) != USB_SUCCESS) {
					return (USB_FAILURE);
				}
				if (reg16 & URE_GPHY_PATCH_DONE) {
					break;
				}
				delay(drv_usectohz(1000));
			}
			if (i == 100) {
				dev_err(sc->ure_dip, CE_WARN,
				    "timeout waiting for flash");
				return (USB_FAILURE);
			}
		}
	}

	return (USB_SUCCESS);
}

static int
ure_rxvlan(ure_softc_t *sc)
{
	/*
	 * Disable hardware VLAN tag stripping.  The illumos MAC framework
	 * expects VLAN tags to be present in-band in the Ethernet frame;
	 * there is no out-of-band mechanism to pass a stripped tag to
	 * mac_rx().  Leave stripping disabled so tagged frames are
	 * delivered intact and the MAC framework handles VLAN demux.
	 */
	if (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B |
	    URE_FLAG_8157)) {
		return (ure_clrbit_2(sc, URE_PLA_RCR1,
		    URE_MCU_TYPE_PLA,
		    URE_INNER_VLAN | URE_OUTER_VLAN));
	} else {
		return (ure_clrbit_2(sc, URE_PLA_CPCR,
		    URE_MCU_TYPE_PLA, URE_CPCR_RX_VLAN));
	}
}

/*
 * Chip init sequences
 */

static int
ure_rtl8152_init(ure_softc_t *sc)
{
	uint32_t pwrctrl;
	int err;

	/* Disable ALDPS */
	if ((err = ure_ocp_reg_write(sc, URE_OCP_ALDPS_CONFIG,
	    URE_ENPDNPS | URE_LINKENA | URE_DIS_SDSAVE)) != 0) {
		return (err);
	}
	delay(drv_usectohz(20000));

	if (sc->ure_chip & URE_CHIP_VER_4C00) {
		if ((err = ure_clrbit_2(sc, URE_PLA_LED_FEATURE,
		    URE_MCU_TYPE_PLA,
		    URE_LED_MODE_MASK)) != USB_SUCCESS) {
			return (err);
		}
	}

	if ((err = ure_clrbit_2(sc, URE_USB_UPS_CTRL,
	    URE_MCU_TYPE_USB, URE_POWER_CUT)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_USB_PM_CTRL_STATUS,
	    URE_MCU_TYPE_USB,
	    URE_RESUME_INDICATE)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_setbit_2(sc, URE_PLA_PHY_PWR,
	    URE_MCU_TYPE_PLA,
	    URE_TX_10M_IDLE_EN | URE_PFM_PWM_SWITCH)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_read_4(sc, URE_PLA_MAC_PWR_CTRL,
	    URE_MCU_TYPE_PLA, &pwrctrl)) != USB_SUCCESS) {
		return (err);
	}
	pwrctrl &= ~URE_MCU_CLK_RATIO_MASK;
	pwrctrl |= URE_MCU_CLK_RATIO | URE_D3_CLK_GATED_EN;
	if ((err = ure_write_4(sc, URE_PLA_MAC_PWR_CTRL,
	    URE_MCU_TYPE_PLA, pwrctrl)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_PLA_GPHY_INTR_IMR,
	    URE_MCU_TYPE_PLA,
	    URE_GPHY_STS_MSK | URE_SPEED_DOWN_MSK |
	    URE_SPDWN_RXDV_MSK | URE_SPDWN_LINKCHG_MSK)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_setbit_2(sc, URE_PLA_RSTTALLY,
	    URE_MCU_TYPE_PLA,
	    URE_TALLY_RESET)) != USB_SUCCESS) {
		return (err);
	}

	/* Enable Rx aggregation */
	if ((err = ure_clrbit_2(sc, URE_USB_USB_CTRL,
	    URE_MCU_TYPE_USB,
	    URE_RX_AGG_DISABLE | URE_RX_ZERO_EN)) != USB_SUCCESS) {
		return (err);
	}

	return (USB_SUCCESS);
}

static int
ure_rtl8153_init(ure_softc_t *sc)
{
	uint16_t reg;
	uint8_t u1u2[8];
	int err;
	int i;

	bzero(u1u2, sizeof (u1u2));
	(void) ure_write_mem(sc, URE_USB_TOLERANCE,
	    URE_BYTE_EN_SIX_BYTES, u1u2, sizeof (u1u2));

	for (i = 0; i < 500; i++) {
		if ((err = ure_read_2(sc, URE_PLA_BOOT_CTRL,
		    URE_MCU_TYPE_PLA, &reg)) != USB_SUCCESS) {
			return (err);
		}
		if (reg & URE_AUTOLOAD_DONE) {
			break;
		}
		delay(drv_usectohz(20000));
	}
	if (i == 500) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for chip autoload");
		return (USB_FAILURE);
	}

	if ((err = ure_rtl8153_phy_status(sc, 0, &reg)) != 0) {
		return (err);
	}

	if (sc->ure_chip & (URE_CHIP_VER_5C00 |
	    URE_CHIP_VER_5C10 | URE_CHIP_VER_5C20)) {
		if ((err = ure_ocp_reg_write(sc, URE_OCP_ADC_CFG,
		    URE_CKADSEL_L | URE_ADC_EN |
		    URE_EN_EMI_L)) != 0) {
			return (err);
		}
	}

	(void) ure_rtl8153_phy_status(sc, URE_PHY_STAT_LAN_ON, &reg);

	if ((err = ure_clrbit_2(sc, URE_USB_U2P3_CTRL,
	    URE_MCU_TYPE_USB, URE_U2P3_ENABLE)) != USB_SUCCESS) {
		return (err);
	}

	if (sc->ure_chip & URE_CHIP_VER_5C10) {
		if ((err = ure_read_2(sc, URE_USB_SSPHYLINK2,
		    URE_MCU_TYPE_USB, &reg)) != USB_SUCCESS) {
			return (err);
		}
		reg &= ~URE_PWD_DN_SCALE_MASK;
		reg |= URE_PWD_DN_SCALE(96);
		if ((err = ure_write_2(sc, URE_USB_SSPHYLINK2,
		    URE_MCU_TYPE_USB, reg)) != USB_SUCCESS) {
			return (err);
		}

		if ((err = ure_setbit_1(sc, URE_USB_USB2PHY,
		    URE_MCU_TYPE_USB,
		    URE_USB2PHY_L1 |
		    URE_USB2PHY_SUSPEND)) != USB_SUCCESS) {
			return (err);
		}
	} else if (sc->ure_chip & URE_CHIP_VER_5C20) {
		if ((err = ure_clrbit_1(sc, URE_PLA_DMY_REG0,
		    URE_MCU_TYPE_PLA,
		    URE_ECM_ALDPS)) != USB_SUCCESS) {
			return (err);
		}
	}

	if (sc->ure_chip & (URE_CHIP_VER_5C20 |
	    URE_CHIP_VER_5C30)) {
		if ((err = ure_read_2(sc, URE_USB_BURST_SIZE,
		    URE_MCU_TYPE_USB, &reg)) != USB_SUCCESS) {
			return (err);
		}
		if (reg) {
			if ((err = ure_setbit_1(sc,
			    URE_USB_CSR_DUMMY1, URE_MCU_TYPE_USB,
			    URE_DYNAMIC_BURST)) != USB_SUCCESS) {
				return (err);
			}
		} else {
			if ((err = ure_clrbit_1(sc,
			    URE_USB_CSR_DUMMY1, URE_MCU_TYPE_USB,
			    URE_DYNAMIC_BURST)) != USB_SUCCESS) {
				return (err);
			}
		}
	}

	if ((err = ure_setbit_1(sc, URE_USB_CSR_DUMMY2,
	    URE_MCU_TYPE_USB,
	    URE_EP4_FULL_FC)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_USB_WDT11_CTRL,
	    URE_MCU_TYPE_USB, URE_TIMER11_EN)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_PLA_LED_FEATURE,
	    URE_MCU_TYPE_PLA,
	    URE_LED_MODE_MASK)) != USB_SUCCESS) {
		return (err);
	}

	if ((sc->ure_chip & URE_CHIP_VER_5C10) &&
	    ure_dev_speed(sc) != USBA_SUPER_SPEED_DEV) {
		reg = URE_LPM_TIMER_500MS;
	} else {
		reg = URE_LPM_TIMER_500US;
	}
	if ((err = ure_write_1(sc, URE_USB_LPM_CTRL,
	    URE_MCU_TYPE_USB,
	    URE_FIFO_EMPTY_1FB | URE_ROK_EXIT_LPM |
	    reg)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_read_2(sc, URE_USB_AFE_CTRL2,
	    URE_MCU_TYPE_USB, &reg)) != USB_SUCCESS) {
		return (err);
	}
	reg &= ~URE_SEN_VAL_MASK;
	reg |= URE_SEN_VAL_NORMAL | URE_SEL_RXIDLE;
	if ((err = ure_write_2(sc, URE_USB_AFE_CTRL2,
	    URE_MCU_TYPE_USB, reg)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_write_2(sc, URE_USB_CONNECT_TIMER,
	    URE_MCU_TYPE_USB, 0x0001)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_2(sc, URE_USB_POWER_CUT,
	    URE_MCU_TYPE_USB,
	    URE_PWR_EN | URE_PHASE2_EN)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_USB_MISC_0,
	    URE_MCU_TYPE_USB,
	    URE_PCUT_STATUS)) != USB_SUCCESS) {
		return (err);
	}

	/*
	 * Disable all MAC power management features (clock gating,
	 * EEE/L1/U1U2 speed-down, packet-available speed-down, etc.)
	 * to avoid link instability and spontaneous disconnects.
	 * This matches OpenBSD.
	 */
	if ((err = ure_write_2(sc, URE_PLA_MAC_PWR_CTRL,
	    URE_MCU_TYPE_PLA, 0)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_PLA_MAC_PWR_CTRL2,
	    URE_MCU_TYPE_PLA, 0)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_PLA_MAC_PWR_CTRL3,
	    URE_MCU_TYPE_PLA, 0)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_PLA_MAC_PWR_CTRL4,
	    URE_MCU_TYPE_PLA, 0)) != USB_SUCCESS) {
		return (err);
	}

	/* Enable Rx aggregation */
	if ((err = ure_clrbit_2(sc, URE_USB_USB_CTRL,
	    URE_MCU_TYPE_USB,
	    URE_RX_AGG_DISABLE | URE_RX_ZERO_EN)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_setbit_2(sc, URE_PLA_RSTTALLY,
	    URE_MCU_TYPE_PLA,
	    URE_TALLY_RESET)) != USB_SUCCESS) {
		return (err);
	}

	return (USB_SUCCESS);
}

static int
ure_rtl8153b_init(ure_softc_t *sc)
{
	uint16_t reg;
	int err;
	int i;

	if (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B)) {
		if ((err = ure_clrbit_1(sc, URE_USB_ECM_OP,
		    URE_MCU_TYPE_USB,
		    URE_EN_ALL_SPEED)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_write_2(sc, URE_USB_SPEED_OPTION,
		    URE_MCU_TYPE_USB, 0)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_setbit_2(sc, URE_USB_ECM_OPTION,
		    URE_MCU_TYPE_USB,
		    URE_BYPASS_MAC_RESET)) != USB_SUCCESS) {
			return (err);
		}

		if (sc->ure_flags & URE_FLAG_8156B) {
			if ((err = ure_setbit_2(sc, URE_USB_U2P3_CTRL,
			    URE_MCU_TYPE_USB,
			    URE_RX_DETECT8)) != USB_SUCCESS) {
				return (err);
			}
		}
	}

	if ((err = ure_clrbit_2(sc, URE_USB_LPM_CONFIG,
	    URE_MCU_TYPE_USB,
	    URE_LPM_U1U2_EN)) != USB_SUCCESS) {
		return (err);
	}

	if (sc->ure_flags & URE_FLAG_8156B) {
		if (ure_wait_for_flash(sc) != 0) {
			return (USB_FAILURE);
		}
	}

	for (i = 0; i < 500; i++) {
		if ((err = ure_read_2(sc, URE_PLA_BOOT_CTRL,
		    URE_MCU_TYPE_PLA, &reg)) != USB_SUCCESS) {
			return (err);
		}
		if (reg & URE_AUTOLOAD_DONE) {
			break;
		}
		delay(drv_usectohz(20000));
	}
	if (i == 500) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for chip autoload");
		return (USB_FAILURE);
	}

	if ((err = ure_rtl8153_phy_status(sc, 0,
	    &reg)) != 0) {
		return (err);
	}

	/*
	 * If the PHY reports EXT_INIT, clear firmware-internal
	 * initialization bits before proceeding.  Matches FreeBSD.
	 * OCP 0xa468 and 0xa466 are undocumented Realtek PHY
	 * registers used during extended initialization.
	 */
	if ((reg == URE_PHY_STAT_EXT_INIT) &&
	    (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B))) {
		uint16_t tmp;

		if ((err = ure_phy_read(sc, 0xa468, &tmp)) != 0) {
			return (err);
		}
		if ((err = ure_phy_write(sc, 0xa468,
		    tmp & ~0x000a)) != 0) {
			return (err);
		}
		if (sc->ure_flags & URE_FLAG_8156B) {
			if ((err = ure_phy_read(sc, 0xa466,
			    &tmp)) != 0) {
				return (err);
			}
			if ((err = ure_phy_write(sc, 0xa466,
			    tmp & ~0x0001)) != 0) {
				return (err);
			}
		}
	}

	/*
	 * Clear BMCR PDOWN if firmware left the PHY powered down.
	 * Some firmware versions leave this set after initialization;
	 * without clearing it the link never comes up.  Matches
	 * FreeBSD.  The resume path calls ure_phy_powerup() which
	 * also clears PDOWN, but this covers the initial attach.
	 */
	{
		uint16_t bmcr;

		if ((err = ure_phy_read(sc, URE_OCP_BMCR,
		    &bmcr)) != 0) {
			return (err);
		}
		if (bmcr & URE_OCP_BMCR_PDOWN) {
			bmcr &= ~URE_OCP_BMCR_PDOWN;
			if ((err = ure_phy_write(sc, URE_OCP_BMCR,
			    bmcr)) != 0) {
				return (err);
			}
		}
	}

	(void) ure_rtl8153_phy_status(sc, URE_PHY_STAT_LAN_ON, &reg);

	if ((err = ure_clrbit_2(sc, URE_USB_U2P3_CTRL,
	    URE_MCU_TYPE_USB,
	    URE_U2P3_ENABLE)) != USB_SUCCESS) {
		return (err);
	}

	/* MSC timer, 32760 ms */
	if ((err = ure_write_2(sc, URE_USB_MSC_TIMER,
	    URE_MCU_TYPE_USB, 4095)) != USB_SUCCESS) {
		return (err);
	}

	if (!(sc->ure_flags & URE_FLAG_8153B)) {
		/* U1/U2/L1 idle timer, 500 us */
		if ((err = ure_write_2(sc, URE_USB_U1U2_TIMER,
		    URE_MCU_TYPE_USB,
		    500)) != USB_SUCCESS) {
			return (err);
		}
	}

	if ((err = ure_clrbit_2(sc, URE_USB_POWER_CUT,
	    URE_MCU_TYPE_USB,
	    URE_PWR_EN)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_USB_MISC_0,
	    URE_MCU_TYPE_USB,
	    URE_PCUT_STATUS)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_1(sc, URE_USB_POWER_CUT,
	    URE_MCU_TYPE_USB,
	    URE_UPS_EN | URE_USP_PREWAKE)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_1(sc, URE_USB_MISC_2,
	    URE_MCU_TYPE_USB,
	    URE_UPS_FORCE_PWR_DOWN |
	    URE_UPS_NO_UPS)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_1(sc, URE_PLA_INDICATE_FALG,
	    URE_MCU_TYPE_PLA,
	    URE_UPCOMING_RUNTIME_D3)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_1(sc, URE_PLA_SUSPEND_FLAG,
	    URE_MCU_TYPE_PLA,
	    URE_LINK_CHG_EVENT)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_PLA_EXTRA_STATUS,
	    URE_MCU_TYPE_PLA,
	    URE_LINK_CHANGE_FLAG)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_write_1(sc, URE_PLA_CRWECR,
	    URE_MCU_TYPE_PLA,
	    URE_CRWECR_CONFIG)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_PLA_CONFIG34,
	    URE_MCU_TYPE_PLA,
	    URE_LINK_OFF_WAKE_EN)) != USB_SUCCESS) {
		(void) ure_write_1(sc, URE_PLA_CRWECR,
		    URE_MCU_TYPE_PLA, URE_CRWECR_NORMAL);
		return (err);
	}
	if ((err = ure_write_1(sc, URE_PLA_CRWECR,
	    URE_MCU_TYPE_PLA,
	    URE_CRWECR_NORMAL)) != USB_SUCCESS) {
		return (err);
	}

	if (sc->ure_flags & URE_FLAG_8153B) {
		if ((err = ure_read_2(sc, URE_PLA_EXTRA_STATUS,
		    URE_MCU_TYPE_PLA, &reg)) != USB_SUCCESS) {
			return (err);
		}
		{
			uint16_t physt;
			if ((err = ure_read_2(sc, URE_PLA_PHYSTATUS,
			    URE_MCU_TYPE_PLA,
			    &physt)) != USB_SUCCESS) {
				return (err);
			}
			if (physt & URE_PHYSTATUS_LINK) {
				reg |= URE_CUR_LINK_OK;
			} else {
				reg &= ~URE_CUR_LINK_OK;
			}
		}
		if ((err = ure_write_2(sc, URE_PLA_EXTRA_STATUS,
		    URE_MCU_TYPE_PLA,
		    reg | URE_POLL_LINK_CHG)) != USB_SUCCESS) {
			return (err);
		}
	}

	/*
	 * Do not re-enable U1/U2 device-side link power management.
	 * These Realtek chips are known to initiate U1/U2 transitions
	 * that cause link instability and spontaneous disconnects;
	 * Linux carries USB_QUIRK_NO_LPM for 0bda:8153 to suppress
	 * host-side U1/U2 negotiation.  Since illumos USBA has no
	 * device quirk mechanism, we keep U1/U2 disabled at the
	 * device register level instead.
	 */

	if (sc->ure_flags & URE_FLAG_8156B) {
		uint16_t tmp;

		if ((err = ure_clrbit_2(sc, URE_PLA_RCR,
		    URE_MCU_TYPE_PLA,
		    URE_SLOT_EN)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_setbit_2(sc, URE_PLA_CPCR,
		    URE_MCU_TYPE_PLA,
		    URE_FLOW_CTRL_EN)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_write_2(sc, URE_USB_FC_TIMER,
		    URE_MCU_TYPE_USB,
		    URE_CTRL_TIMER_EN | 75)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_read_2(sc, URE_USB_FW_CTRL,
		    URE_MCU_TYPE_USB, &reg)) != USB_SUCCESS) {
			return (err);
		}
		reg &= ~URE_AUTO_SPEEDUP;
		if ((err = ure_read_2(sc, URE_PLA_POL_GPIO_CTRL,
		    URE_MCU_TYPE_PLA, &tmp)) != USB_SUCCESS) {
			return (err);
		}
		if (!(tmp & URE_DACK_DET_EN)) {
			reg |= URE_FLOW_CTRL_PATCH_2;
		}
		if ((err = ure_write_2(sc, URE_USB_FW_CTRL,
		    URE_MCU_TYPE_USB, reg)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_setbit_2(sc, URE_USB_FW_TASK,
		    URE_MCU_TYPE_USB,
		    URE_FC_PATCH_TASK)) != USB_SUCCESS) {
			return (err);
		}
	}

	if (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B)) {
		if ((err = ure_clrbit_2(sc, URE_PLA_MAC_PWR_CTRL3,
		    URE_MCU_TYPE_PLA,
		    URE_PLA_MCU_SPDWN_EN)) != USB_SUCCESS) {
			return (err);
		}

		if ((err = ure_read_2(sc, URE_PLA_EXTRA_STATUS,
		    URE_MCU_TYPE_PLA, &reg)) != USB_SUCCESS) {
			return (err);
		}
		reg &= ~URE_CUR_LINK_OK;
		{
			uint16_t physt;
			if ((err = ure_read_2(sc, URE_PLA_PHYSTATUS,
			    URE_MCU_TYPE_PLA,
			    &physt)) != USB_SUCCESS) {
				return (err);
			}
			if (physt & URE_PHYSTATUS_LINK) {
				reg |= URE_CUR_LINK_OK;
			}
		}
		if ((err = ure_write_2(sc, URE_PLA_EXTRA_STATUS,
		    URE_MCU_TYPE_PLA,
		    reg | URE_POLL_LINK_CHG)) != USB_SUCCESS) {
			return (err);
		}
	} else {
		if ((err = ure_setbit_2(sc, URE_PLA_MAC_PWR_CTRL2,
		    URE_MCU_TYPE_PLA,
		    URE_MAC_CLK_SPDWN_EN)) != USB_SUCCESS) {
			return (err);
		}
	}

	/* Enable Rx aggregation */
	if ((err = ure_clrbit_2(sc, URE_USB_USB_CTRL,
	    URE_MCU_TYPE_USB,
	    URE_RX_AGG_DISABLE | URE_RX_ZERO_EN)) != USB_SUCCESS) {
		return (err);
	}

	if (sc->ure_flags & URE_FLAG_8156) {
		if ((err = ure_setbit_1(sc, URE_USB_BMU_CONFIG,
		    URE_MCU_TYPE_USB,
		    URE_ACT_ODMA)) != USB_SUCCESS) {
			return (err);
		}
	}

	if (!(sc->ure_flags & URE_FLAG_8153B)) {
		if ((err = ure_phy_read(sc, URE_OCP_PHY_0XA5B4,
		    &reg)) != 0) {
			return (err);
		}
		if ((err = ure_phy_write(sc, URE_OCP_PHY_0XA5B4,
		    reg & ~URE_OCP_PHY_0XA5B4_DIS)) != 0) {
			return (err);
		}
	}

	if ((err = ure_setbit_2(sc, URE_PLA_RSTTALLY,
	    URE_MCU_TYPE_PLA,
	    URE_TALLY_RESET)) != USB_SUCCESS) {
		return (err);
	}

	return (USB_SUCCESS);
}

static int
ure_rtl8157_init(ure_softc_t *sc)
{
	uint16_t reg;
	uint16_t tmp;
	int err;
	int i;

	if ((err = ure_setbit_1(sc, URE_USB_UNDOC_CFFE, URE_MCU_TYPE_USB,
	    0x0008)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_1(sc, URE_USB_UNDOC_D3CA, URE_MCU_TYPE_USB,
	    0x0001)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_1(sc, URE_USB_ECM_OP, URE_MCU_TYPE_USB,
	    URE_EN_ALL_SPEED)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_setbit_2(sc, URE_USB_ECM_OPTION, URE_MCU_TYPE_USB,
	    URE_BYPASS_MAC_RESET)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_setbit_2(sc, URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB,
	    URE_RX_DETECT8)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_USB_LPM_CONFIG, URE_MCU_TYPE_USB,
	    URE_LPM_U1U2_EN)) != USB_SUCCESS) {
		return (err);
	}

	for (i = 0; i < 500; i++) {
		if ((err = ure_read_2(sc, URE_PLA_BOOT_CTRL,
		    URE_MCU_TYPE_PLA, &reg)) != USB_SUCCESS) {
			return (err);
		}
		if (reg & URE_AUTOLOAD_DONE) {
			break;
		}
		delay(drv_usectohz(20000));
	}
	if (i == 500) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for chip autoload");
		return (USB_FAILURE);
	}

	if (ure_wait_for_flash(sc) != 0) {
		return (USB_FAILURE);
	}

	if ((err = ure_rtl8153_phy_status(sc, 0, &reg)) != USB_SUCCESS) {
		return (err);
	}
	(void) ure_rtl8153_phy_status(sc, URE_PHY_STAT_LAN_ON, &reg);

	/* Disable U2P3 via OCP command interface */
	if ((err = ure_ocp_cmd_clrbit(sc, URE_USB_U2P3_CTRL2,
	    URE_CMD_TYPE_IP, URE_U2P3_CTRL2_ENABLE)) != USB_SUCCESS) {
		return (err);
	}

	/* Disable interrupt mitigation */
	if ((err = ure_clrbit_1(sc, URE_USB_INT_MITIGATION,
	    URE_MCU_TYPE_USB, URE_USB_INT_MIT_MASK)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_2(sc, URE_USB_SPEED_OPTION, URE_MCU_TYPE_USB,
	    URE_RG_PWRDN_EN | URE_ALL_SPEED_OFF)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_setbit_2(sc, URE_USB_FW_CTRL, URE_MCU_TYPE_USB,
	    URE_AUTO_SPEEDUP)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_write_2(sc, URE_USB_MSC_TIMER, URE_MCU_TYPE_USB,
	    4095)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_USB_U1U2_TIMER, URE_MCU_TYPE_USB,
	    500)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_2(sc, URE_USB_POWER_CUT, URE_MCU_TYPE_USB,
	    URE_PWR_EN)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_USB_MISC_0, URE_MCU_TYPE_USB,
	    URE_PCUT_STATUS)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_1(sc, URE_USB_MISC_2, URE_MCU_TYPE_USB,
	    0x02)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_1(sc, URE_PLA_INDICATE_FALG,
	    URE_MCU_TYPE_PLA, URE_UPCOMING_RUNTIME_D3)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_1(sc, URE_PLA_SUSPEND_FLAG,
	    URE_MCU_TYPE_PLA, URE_LINK_CHG_EVENT)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_PLA_EXTRA_STATUS,
	    URE_MCU_TYPE_PLA, URE_LINK_CHANGE_FLAG)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_CONFIG)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_PLA_CONFIG34, URE_MCU_TYPE_PLA,
	    URE_LINK_OFF_WAKE_EN)) != USB_SUCCESS) {
		(void) ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
		    URE_CRWECR_NORMAL);
		return (err);
	}
	if ((err = ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_NORMAL)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_2(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA,
	    URE_SLOT_EN)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_setbit_2(sc, URE_PLA_CPCR, URE_MCU_TYPE_PLA,
	    URE_FLOW_CTRL_EN)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_write_2(sc, URE_USB_FC_TIMER, URE_MCU_TYPE_USB,
	    URE_CTRL_TIMER_EN | 75)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_read_2(sc, URE_USB_FW_CTRL, URE_MCU_TYPE_USB,
	    &reg)) != USB_SUCCESS) {
		return (err);
	}
	reg &= ~URE_AUTO_SPEEDUP;
	if ((err = ure_read_2(sc, URE_PLA_POL_GPIO_CTRL,
	    URE_MCU_TYPE_PLA, &tmp)) != USB_SUCCESS) {
		return (err);
	}
	if (!(tmp & URE_DACK_DET_EN)) {
		reg |= URE_FLOW_CTRL_PATCH_2;
	}
	if ((err = ure_write_2(sc, URE_USB_FW_CTRL, URE_MCU_TYPE_USB,
	    reg)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_setbit_2(sc, URE_USB_FW_TASK, URE_MCU_TYPE_USB,
	    URE_FC_PATCH_TASK)) != USB_SUCCESS) {
		return (err);
	}

	/* Disable bypass_turn_off_clk_in_aldps */
	if ((err = ure_clrbit_1(sc, URE_PLA_BYPASS_ALDPS,
	    URE_MCU_TYPE_PLA, 0x01)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_2(sc, URE_PLA_MAC_PWR_CTRL3,
	    URE_MCU_TYPE_PLA, URE_PLA_MCU_SPDWN_EN)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_read_2(sc, URE_PLA_EXTRA_STATUS,
	    URE_MCU_TYPE_PLA, &reg)) != USB_SUCCESS) {
		return (err);
	}
	reg &= ~URE_CUR_LINK_OK;
	if ((err = ure_read_2(sc, URE_PLA_PHYSTATUS,
	    URE_MCU_TYPE_PLA, &tmp)) != USB_SUCCESS) {
		return (err);
	}
	if (tmp & URE_PHYSTATUS_LINK) {
		reg |= URE_CUR_LINK_OK;
	}
	if ((err = ure_write_2(sc, URE_PLA_EXTRA_STATUS,
	    URE_MCU_TYPE_PLA, reg | URE_POLL_LINK_CHG)) != USB_SUCCESS) {
		return (err);
	}

	/* Enable Rx aggregation */
	if ((err = ure_clrbit_2(sc, URE_USB_USB_CTRL, URE_MCU_TYPE_USB,
	    URE_RX_AGG_DISABLE | URE_USB_RX_AGG_0X0400)) != USB_SUCCESS) {
		return (err);
	}

	/* Disable Rx zero length via OCP command */
	if ((err = ure_ocp_cmd_clrbit(sc, URE_PLA_BMU_0X2300,
	    URE_CMD_TYPE_BMU, URE_PLA_BMU_0X2300_RZL)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_1(sc, URE_USB_UNDOC_D4AE,
	    URE_MCU_TYPE_USB, 0x02)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_phy_read(sc, URE_OCP_PHY_0XA5B4, &reg)) != USB_SUCCESS) {
		return (err);
	}
	if (reg == 0xffff) {
		dev_err(sc->ure_dip, CE_WARN,
		    "PHY register 0xa5b4 returned 0xffff, PHY not responding");
		return (USB_FAILURE);
	}
	if ((err = ure_phy_write(sc, URE_OCP_PHY_0XA5B4,
	    reg & ~URE_OCP_PHY_0XA5B4_DIS)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_setbit_2(sc, URE_PLA_RSTTALLY, URE_MCU_TYPE_PLA,
	    URE_TALLY_RESET)) != USB_SUCCESS) {
		return (err);
	}

	return (USB_SUCCESS);
}

/*
 * NIC reset (called per interface-up)
 */

static int
ure_rtl8152_nic_reset(ure_softc_t *sc)
{
	uint32_t rx_fifo1, rx_fifo2;
	uint8_t tmp8;
	int err;
	int i;

	/* Invalidate OCP base cache; ure_reset() clears PLA registers */
	sc->ure_ocp_base = 0;

	/* Disable ALDPS */
	if ((err = ure_ocp_reg_write(sc, URE_OCP_ALDPS_CONFIG,
	    URE_ENPDNPS | URE_LINKENA | URE_DIS_SDSAVE)) != USB_SUCCESS) {
		return (err);
	}
	delay(drv_usectohz(20000));

	if ((err = ure_clrbit_4(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA,
	    URE_RCR_ACPT_ALL)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_setbit_2(sc, URE_PLA_MISC_1, URE_MCU_TYPE_PLA,
	    URE_RXDY_GATED_EN)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_disable_teredo(sc)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
	    URE_CRWECR_NORMAL)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA,
	    0)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_1(sc, URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA,
	    URE_NOW_IS_OOB)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA,
	    URE_MCU_BORW_EN)) != USB_SUCCESS) {
		return (err);
	}
	for (i = 0; i < URE_TIMEOUT; i++) {
		if ((err = ure_read_1(sc, URE_PLA_OOB_CTRL,
		    URE_MCU_TYPE_PLA, &tmp8)) != USB_SUCCESS) {
			return (err);
		}
		if (tmp8 & URE_LINK_LIST_READY) {
			break;
		}
		delay(drv_usectohz(1000));
	}
	if (i == URE_TIMEOUT) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for OOB control");
		return (USB_FAILURE);
	}
	if ((err = ure_setbit_2(sc, URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA,
	    URE_RE_INIT_LL)) != USB_SUCCESS) {
		return (err);
	}
	for (i = 0; i < URE_TIMEOUT; i++) {
		if ((err = ure_read_1(sc, URE_PLA_OOB_CTRL,
		    URE_MCU_TYPE_PLA, &tmp8)) != USB_SUCCESS) {
			return (err);
		}
		if (tmp8 & URE_LINK_LIST_READY) {
			break;
		}
		delay(drv_usectohz(1000));
	}
	if (i == URE_TIMEOUT) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for OOB control (2)");
		return (USB_FAILURE);
	}

	if ((err = ure_reset(sc)) != USB_SUCCESS) {
		return (err);
	}

	/* Configure Rx FIFO threshold */
	if ((err = ure_write_4(sc, URE_PLA_RXFIFO_CTRL0, URE_MCU_TYPE_PLA,
	    URE_RXFIFO_THR1_NORMAL)) != USB_SUCCESS) {
		return (err);
	}
	if (ure_dev_speed(sc) == USBA_FULL_SPEED_DEV) {
		rx_fifo1 = URE_RXFIFO_THR2_FULL;
		rx_fifo2 = URE_RXFIFO_THR3_FULL;
	} else {
		rx_fifo1 = URE_RXFIFO_THR2_HIGH;
		rx_fifo2 = URE_RXFIFO_THR3_HIGH;
	}
	if ((err = ure_write_4(sc, URE_PLA_RXFIFO_CTRL1, URE_MCU_TYPE_PLA,
	    rx_fifo1)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_4(sc, URE_PLA_RXFIFO_CTRL2, URE_MCU_TYPE_PLA,
	    rx_fifo2)) != USB_SUCCESS) {
		return (err);
	}

	/* Configure Tx FIFO threshold (NORMAL2 for RTL8152) */
	if ((err = ure_write_4(sc, URE_PLA_TXFIFO_CTRL, URE_MCU_TYPE_PLA,
	    URE_TXFIFO_THR_NORMAL2)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_write_1(sc, URE_USB_TX_AGG, URE_MCU_TYPE_USB,
	    URE_TX_AGG_MAX_THRESHOLD)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_4(sc, URE_USB_RX_BUF_TH, URE_MCU_TYPE_USB,
	    URE_RX_THR_HIGH)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_4(sc, URE_USB_TX_DMA, URE_MCU_TYPE_USB,
	    URE_TEST_MODE_DISABLE | URE_TX_SIZE_ADJUST1)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_rxvlan(sc)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_PLA_RMS, URE_MCU_TYPE_PLA,
	    ETHERMAX + VLAN_TAGSZ)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_setbit_2(sc, URE_PLA_TCR0, URE_MCU_TYPE_PLA,
	    URE_TCR0_AUTO_FIFO)) != USB_SUCCESS) {
		return (err);
	}

	/*
	 * Advertise 802.3x flow control (PAUSE) capability so the switch
	 * can negotiate PAUSE frames, reducing RX drops under heavy load.
	 * Linux does this in r8152b_enable_fc(); OpenBSD omits it.
	 */
	{
		uint16_t anar;

		if ((err = ure_ocp_reg_read(sc, URE_OCP_ANAR,
		    &anar)) != USB_SUCCESS) {
			return (err);
		}
		anar |= URE_ANAR_PAUSE | URE_ANAR_ASYM_PAUSE;
		if ((err = ure_ocp_reg_write(sc, URE_OCP_ANAR,
		    anar)) != USB_SUCCESS) {
			return (err);
		}
	}

	/* Enable ALDPS */
	if ((err = ure_ocp_reg_write(sc, URE_OCP_ALDPS_CONFIG,
	    URE_ENPWRSAVE | URE_ENPDNPS | URE_LINKENA |
	    URE_DIS_SDSAVE)) != USB_SUCCESS) {
		return (err);
	}

	return (USB_SUCCESS);
}

static int
ure_rtl8153_nic_reset(ure_softc_t *sc)
{
	uint8_t u1u2[8] = { 0 };
	uint16_t val;
	uint8_t val8;
	int i;
	int err;

	/* Invalidate OCP base cache; ure_reset() clears PLA registers */
	sc->ure_ocp_base = 0;

	switch (sc->ure_flags & URE_FLAG_CHIP_MASK) {
	case URE_FLAG_8153B:
	case URE_FLAG_8156:
	case URE_FLAG_8156B:
	case URE_FLAG_8157:
		if ((err = ure_clrbit_2(sc, URE_USB_LPM_CONFIG,
		    URE_MCU_TYPE_USB, URE_LPM_U1U2_EN)) != USB_SUCCESS) {
			return (err);
		}
		break;
	default:
		bzero(u1u2, sizeof (u1u2));
		if ((err = ure_write_mem(sc, URE_USB_TOLERANCE,
		    URE_BYTE_EN_SIX_BYTES, u1u2,
		    sizeof (u1u2))) != USB_SUCCESS) {
			return (err);
		}
		break;
	}
	if ((err = ure_clrbit_2(sc, URE_USB_U2P3_CTRL, URE_MCU_TYPE_USB,
	    URE_U2P3_ENABLE)) != USB_SUCCESS) {
		return (err);
	}

	/* Disable ALDPS */
	if ((err = ure_phy_read(sc, URE_OCP_POWER_CFG, &val)) !=
	    USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_phy_write(sc, URE_OCP_POWER_CFG,
	    val & ~URE_EN_ALDPS)) != USB_SUCCESS) {
		return (err);
	}
	for (i = 0; i < 20; i++) {
		delay(drv_usectohz(1000));
		if ((err = ure_read_2(sc, URE_PLA_ALDPS_STATUS, URE_MCU_TYPE_PLA,
		    &val)) != USB_SUCCESS) {
			return (err);
		}
		if (val & URE_ALDPS_STATUS_IDLE) {
			break;
		}
	}
	if (i == 20) {
		dev_err(sc->ure_dip, CE_WARN,
		    "timeout waiting for ALDPS idle");
	}

	if ((err = ure_setbit_2(sc, URE_PLA_MISC_1, URE_MCU_TYPE_PLA,
	    URE_RXDY_GATED_EN)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_disable_teredo(sc)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_4(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA,
	    URE_RCR_ACPT_ALL)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_reset(sc)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_reset_bmu(sc)) != USB_SUCCESS) {
		return (err);
	}

	if ((err = ure_clrbit_1(sc, URE_PLA_OOB_CTRL, URE_MCU_TYPE_PLA,
	    URE_NOW_IS_OOB)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_clrbit_2(sc, URE_PLA_SFF_STS_7, URE_MCU_TYPE_PLA,
	    URE_MCU_BORW_EN)) != USB_SUCCESS) {
		return (err);
	}

	if (!(sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B |
	    URE_FLAG_8157))) {
		for (i = 0; i < URE_TIMEOUT; i++) {
			if ((err = ure_read_1(sc, URE_PLA_OOB_CTRL,
			    URE_MCU_TYPE_PLA, &val8)) != USB_SUCCESS) {
				return (err);
			}
			if (val8 & URE_LINK_LIST_READY) {
				break;
			}
			delay(drv_usectohz(1000));
		}
		if (i == URE_TIMEOUT) {
			dev_err(sc->ure_dip, CE_WARN,
			    "timeout waiting for OOB control");
			return (USB_FAILURE);
		}
		if ((err = ure_setbit_2(sc, URE_PLA_SFF_STS_7,
		    URE_MCU_TYPE_PLA, URE_RE_INIT_LL)) != USB_SUCCESS) {
			return (err);
		}
		for (i = 0; i < URE_TIMEOUT; i++) {
			if ((err = ure_read_1(sc, URE_PLA_OOB_CTRL,
			    URE_MCU_TYPE_PLA, &val8)) != USB_SUCCESS) {
				return (err);
			}
			if (val8 & URE_LINK_LIST_READY) {
				break;
			}
			delay(drv_usectohz(1000));
		}
		if (i == URE_TIMEOUT) {
			dev_err(sc->ure_dip, CE_WARN,
			    "timeout waiting for OOB (2)");
			return (USB_FAILURE);
		}
	}

	if ((err = ure_rxvlan(sc)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_2(sc, URE_PLA_RMS, URE_MCU_TYPE_PLA,
	    ETHERMAX + VLAN_TAGSZ)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_write_1(sc, URE_PLA_MTPS, URE_MCU_TYPE_PLA,
	    (sc->ure_flags & URE_FLAG_8157) ?
	    URE_MTPS_MAX : URE_MTPS_JUMBO)) != USB_SUCCESS) {
		return (err);
	}

	if (sc->ure_flags & (URE_FLAG_8156 | URE_FLAG_8156B |
	    URE_FLAG_8157)) {
		uint16_t reg;

		if ((err = ure_write_2(sc, URE_PLA_RX_FIFO_FULL,
		    URE_MCU_TYPE_PLA,
		    (sc->ure_flags & URE_FLAG_8156) ?
		    1024 : 512)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_write_2(sc, URE_PLA_RX_FIFO_EMPTY,
		    URE_MCU_TYPE_PLA,
		    (sc->ure_flags & URE_FLAG_8156) ?
		    2048 : 1024)) != USB_SUCCESS) {
			return (err);
		}

		/* TX share FIFO free credit full threshold */
		if ((err = ure_write_2(sc, URE_PLA_TXFIFO_CTRL,
		    URE_MCU_TYPE_PLA, 8)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_write_2(sc, URE_PLA_TXFIFO_FULL,
		    URE_MCU_TYPE_PLA, 128)) != USB_SUCCESS) {
			return (err);
		}

		if (sc->ure_flags & URE_FLAG_8156) {
			if ((err = ure_setbit_2(sc, URE_USB_BMU_CONFIG,
			    URE_MCU_TYPE_USB,
			    URE_ACT_ODMA)) != USB_SUCCESS) {
				return (err);
			}
		}

		/* FIFO settings */
		if ((err = ure_read_2(sc, URE_PLA_RXFIFO_FULL,
		    URE_MCU_TYPE_PLA, &reg)) != USB_SUCCESS) {
			return (err);
		}
		reg &= ~URE_RXFIFO_FULL_MASK;
		if ((err = ure_write_2(sc, URE_PLA_RXFIFO_FULL,
		    URE_MCU_TYPE_PLA, reg | 0x0008)) != USB_SUCCESS) {
			return (err);
		}

		if ((err = ure_clrbit_2(sc, URE_PLA_MAC_PWR_CTRL3,
		    URE_MCU_TYPE_PLA,
		    URE_PLA_MCU_SPDWN_EN)) != USB_SUCCESS) {
			return (err);
		}

		if (!(sc->ure_flags & URE_FLAG_8157)) {
			if ((err = ure_clrbit_2(sc, URE_USB_SPEED_OPTION,
			    URE_MCU_TYPE_USB,
			    URE_RG_PWRDN_EN | URE_ALL_SPEED_OFF)) !=
			    USB_SUCCESS) {
				return (err);
			}
		}

		if ((err = ure_write_4(sc, URE_USB_RX_BUF_TH,
		    URE_MCU_TYPE_USB, 0x00600400)) != USB_SUCCESS) {
			return (err);
		}
	} else {
		if ((err = ure_setbit_2(sc, URE_PLA_TCR0, URE_MCU_TYPE_PLA,
		    URE_TCR0_AUTO_FIFO)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_reset(sc)) != USB_SUCCESS) {
			return (err);
		}

		if ((err = ure_write_4(sc, URE_PLA_RXFIFO_CTRL0,
		    URE_MCU_TYPE_PLA,
		    URE_RXFIFO_THR1_NORMAL)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_write_2(sc, URE_PLA_RXFIFO_CTRL1,
		    URE_MCU_TYPE_PLA,
		    URE_RXFIFO_THR2_NORMAL)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_write_2(sc, URE_PLA_RXFIFO_CTRL2,
		    URE_MCU_TYPE_PLA,
		    URE_RXFIFO_THR3_NORMAL)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_write_4(sc, URE_PLA_TXFIFO_CTRL,
		    URE_MCU_TYPE_PLA,
		    URE_TXFIFO_THR_NORMAL2)) != USB_SUCCESS) {
			return (err);
		}

		if (sc->ure_flags & URE_FLAG_8153B) {
			if ((err = ure_write_4(sc, URE_USB_RX_BUF_TH,
			    URE_MCU_TYPE_USB, URE_RX_THR_B)) !=
			    USB_SUCCESS) {
				return (err);
			}

			if ((err = ure_clrbit_2(sc, URE_PLA_MAC_PWR_CTRL3,
			    URE_MCU_TYPE_PLA,
			    URE_PLA_MCU_SPDWN_EN)) != USB_SUCCESS) {
				return (err);
			}
		} else {
			if ((err = ure_setbit_1(sc, URE_PLA_CONFIG6,
			    URE_MCU_TYPE_PLA,
			    URE_LANWAKE_CLR_EN)) != USB_SUCCESS) {
				return (err);
			}
			if ((err = ure_clrbit_1(sc, URE_PLA_LWAKE_CTRL_REG,
			    URE_MCU_TYPE_PLA,
			    URE_LANWAKE_PIN)) != USB_SUCCESS) {
				return (err);
			}
			if ((err = ure_clrbit_2(sc, URE_USB_SSPHYLINK1,
			    URE_MCU_TYPE_USB,
			    URE_DELAY_PHY_PWR_CHG)) != USB_SUCCESS) {
				return (err);
			}
		}
	}

	/* Enable ALDPS */
	if ((err = ure_phy_read(sc, URE_OCP_POWER_CFG, &val)) !=
	    USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_phy_write(sc, URE_OCP_POWER_CFG,
	    val | URE_EN_ALDPS)) != USB_SUCCESS) {
		return (err);
	}

	if (sc->ure_flags & URE_FLAG_8157) {
		/* Clear SDR */
		if ((err = ure_setbit_1(sc, URE_USB_SDR_D378, URE_MCU_TYPE_USB,
		    0x0080)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_clrbit_2(sc, URE_USB_SDR_CD06, URE_MCU_TYPE_USB,
		    0x8000)) != USB_SUCCESS) {
			return (err);
		}
	}

	if ((sc->ure_chip & (URE_CHIP_VER_5C20 |
	    URE_CHIP_VER_5C30)) ||
	    (sc->ure_flags & (URE_FLAG_8156 |
	    URE_FLAG_8156B))) {
		if ((err = ure_setbit_2(sc, URE_USB_U2P3_CTRL,
		    URE_MCU_TYPE_USB, URE_U2P3_ENABLE)) != USB_SUCCESS) {
			return (err);
		}
	}

	/*
	 * U1/U2 device-side link power management is intentionally
	 * left disabled.  These Realtek chips cause link instability
	 * when U1/U2 transitions are enabled.
	 */

	return (USB_SUCCESS);
}


/*
 * Media / link state
 */

static int
ure_ifmedia_init(ure_softc_t *sc)
{
	int err;

	/* Set MAC address */
	{
		uint8_t addr[8] = {0};
		bcopy(sc->ure_dev_addr, addr, ETHERADDRL);
		if ((err = ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
		    URE_CRWECR_CONFIG)) != USB_SUCCESS) {
			return (err);
		}
		if ((err = ure_write_mem(sc, URE_PLA_IDR,
		    URE_MCU_TYPE_PLA | URE_BYTE_EN_SIX_BYTES,
		    addr, sizeof (addr))) != USB_SUCCESS) {
			(void) ure_write_1(sc, URE_PLA_CRWECR,
			    URE_MCU_TYPE_PLA, URE_CRWECR_NORMAL);
			return (err);
		}
		if ((err = ure_write_1(sc, URE_PLA_CRWECR, URE_MCU_TYPE_PLA,
		    URE_CRWECR_NORMAL)) != USB_SUCCESS) {
			return (err);
		}
	}

	if (!(sc->ure_flags & URE_FLAG_8152)) {
		uint32_t reg;

		if (sc->ure_flags & (URE_FLAG_8156B | URE_FLAG_8157)) {
			if ((err = ure_clrbit_2(sc, URE_USB_RX_AGGR_NUM,
			    URE_MCU_TYPE_USB,
			    URE_RX_AGGR_NUM_MASK)) != USB_SUCCESS) {
				return (err);
			}
		}

		reg = sc->ure_rxbufsz - (ETHERMAX + VLAN_TAGSZ);
		if (sc->ure_flags & URE_FLAG_8157) {
			reg -= sizeof (ure_rxpkt_v2_t) +
			    URE_8157_BUF_ALIGN;
		} else {
			reg -= sizeof (ure_rxpkt_t) +
			    URE_RX_BUF_ALIGN;
		}

		if (sc->ure_flags & (URE_FLAG_8153B |
		    URE_FLAG_8156 | URE_FLAG_8156B |
		    URE_FLAG_8157)) {
			if ((err = ure_write_2(sc, URE_USB_RX_EARLY_SIZE,
			    URE_MCU_TYPE_USB,
			    (sc->ure_flags & URE_FLAG_8157) ?
			    reg / URE_8157_BUF_ALIGN :
			    reg / URE_RX_BUF_ALIGN)) != USB_SUCCESS) {
				return (err);
			}
			if ((err = ure_write_2(sc, URE_USB_RX_EARLY_AGG,
			    URE_MCU_TYPE_USB,
			    (sc->ure_flags & URE_FLAG_8153B) ?
			    158 : 80)) != USB_SUCCESS) {
				return (err);
			}
			if ((err = ure_write_2(sc, URE_USB_PM_CTRL_STATUS,
			    URE_MCU_TYPE_USB, 1875)) != USB_SUCCESS) {
				return (err);
			}

			if (ure_dev_speed(sc) ==
			    USBA_HIGH_SPEED_DEV) {
				uint16_t l1reg;
				if ((err = ure_read_2(sc,
				    URE_USB_L1_CTRL,
				    URE_MCU_TYPE_USB,
				    &l1reg)) != USB_SUCCESS) {
					return (err);
				}
				l1reg &= ~0x0f;
				if ((err = ure_write_2(sc, URE_USB_L1_CTRL,
				    URE_MCU_TYPE_USB,
				    l1reg | 0x01)) != USB_SUCCESS) {
					return (err);
				}
			}
		} else {
			if ((err = ure_write_2(sc, URE_USB_RX_EARLY_SIZE,
			    URE_MCU_TYPE_USB, reg / 4)) != USB_SUCCESS) {
				return (err);
			}
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
			if ((err = ure_write_2(sc, URE_USB_RX_EARLY_AGG,
			    URE_MCU_TYPE_USB, reg)) != USB_SUCCESS) {
				return (err);
			}
		}

		if (sc->ure_chip & URE_CHIP_VER_7420) {
			if ((err = ure_setbit_2(sc, URE_PLA_MAC_PWR_CTRL4,
			    URE_MCU_TYPE_PLA,
			    URE_IDLE_SPDWN_EN)) != USB_SUCCESS) {
				return (err);
			}
		}

		if ((sc->ure_chip & URE_CHIP_VER_6010) ||
		    (sc->ure_flags & URE_FLAG_8156B)) {
			if ((err = ure_clrbit_2(sc, URE_USB_FW_TASK,
			    URE_MCU_TYPE_USB,
			    URE_FC_PATCH_TASK)) != USB_SUCCESS) {
				return (err);
			}
			delay(drv_usectohz(1000));
			if ((err = ure_setbit_2(sc, URE_USB_FW_TASK,
			    URE_MCU_TYPE_USB,
			    URE_FC_PATCH_TASK)) != USB_SUCCESS) {
				return (err);
			}
		}
	}

	/* Reset the packet filter */
	if ((err = ure_clrbit_2(sc, URE_PLA_FMC, URE_MCU_TYPE_PLA,
	    URE_FMC_FCR_MCU_EN)) != USB_SUCCESS) {
		return (err);
	}
	if ((err = ure_setbit_2(sc, URE_PLA_FMC, URE_MCU_TYPE_PLA,
	    URE_FMC_FCR_MCU_EN)) != USB_SUCCESS) {
		return (err);
	}

	/* Enable transmit and receive */
	if ((err = ure_setbit_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA,
	    URE_CR_RE | URE_CR_TE)) != USB_SUCCESS) {
		return (err);
	}

	if (sc->ure_flags & (URE_FLAG_8153B | URE_FLAG_8156 |
	    URE_FLAG_8156B | URE_FLAG_8157)) {
		if ((err = ure_write_1(sc, URE_USB_UPT_RXDMA_OWN,
		    URE_MCU_TYPE_USB,
		    URE_OWN_UPDATE | URE_OWN_CLEAR)) != USB_SUCCESS) {
			return (err);
		}
	}

	if ((err = ure_clrbit_2(sc, URE_PLA_MISC_1, URE_MCU_TYPE_PLA,
	    URE_RXDY_GATED_EN)) != USB_SUCCESS) {
		return (err);
	}

	return (USB_SUCCESS);
}

/*
 * Periodic link status check, called via ddi_periodic_add.
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
	uint16_t ocp_val = 0;
	boolean_t link_changed;

	if (sc->ure_gone) {
		return;
	}

	mutex_enter(&sc->ure_lock);
	if (!sc->ure_running) {
		mutex_exit(&sc->ure_lock);
		return;
	}
	mutex_exit(&sc->ure_lock);

	/*
	 * Read PHYSTATUS once for both link detection and speed/duplex.
	 * Two separate reads would create a TOCTOU window where a
	 * link change between reads could produce an inconsistent
	 * snapshot.  All USB I/O runs without ure_lock held.
	 */
	if (ure_read_2(sc, URE_PLA_PHYSTATUS, URE_MCU_TYPE_PLA,
	    &status) != USB_SUCCESS) {
		return;
	}

	if (status & URE_PHYSTATUS_LINK) {
		new_link = LINK_STATE_UP;

		if (ure_has_5000fdx(sc) &&
		    (status & URE_PHYSTATUS_5000MBPS)) {
			speed = 5000000000ULL;
			duplex = LINK_DUPLEX_FULL;
		} else if (ure_has_2500fdx(sc) &&
		    (status & URE_PHYSTATUS_2500MBPS)) {
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

		/*
		 * Cache link partner 2.5G/5G ability from the
		 * 10GBASE-T AN status register.
		 */
		if (ure_has_2500fdx(sc)) {
			if (ure_ocp_reg_read(sc, URE_OCP_10GBT_STAT,
			    &ocp_val) != USB_SUCCESS) {
				return;
			}
		}

		/* Re-enable TX/RX on link up */
		if (ure_setbit_1(sc, URE_PLA_CR, URE_MCU_TYPE_PLA,
		    URE_CR_RE | URE_CR_TE) != USB_SUCCESS) {
			return;
		}
	} else {
		uint16_t bmsr;

		/*
		 * FreeBSD spurious link-down workaround:
		 * MII BMSR link status is latched-low.
		 * Double-read to clear stale latch.
		 */
		if (ure_ocp_reg_read(sc,
		    URE_OCP_BASE_MII + 0x02,
		    &bmsr) != USB_SUCCESS) {	/* MII_BMSR */
			return;
		}
		if (ure_ocp_reg_read(sc,
		    URE_OCP_BASE_MII + 0x02,
		    &bmsr) != USB_SUCCESS) {
			return;
		}

		if (bmsr & 0x0004) {	/* BMSR_LINK */
			/* PHY still has link, spurious */
			new_link = LINK_STATE_UP;
		} else {
			new_link = LINK_STATE_DOWN;
		}
	}

	/* Reacquire lock to update cached link state */
	mutex_enter(&sc->ure_lock);
	if (!sc->ure_running || sc->ure_gone) {
		mutex_exit(&sc->ure_lock);
		return;
	}

	if (status & URE_PHYSTATUS_LINK) {
		sc->ure_flags |= URE_FLAG_LINK;
		if (ure_has_2500fdx(sc)) {
			sc->ure_10gbt_stat = ocp_val;
		}
	} else {
		sc->ure_flags &= ~URE_FLAG_LINK;
		if (new_link == LINK_STATE_UP) {
			/* Spurious down - keep previous speed/duplex */
			speed = sc->ure_link_speed;
			duplex = sc->ure_link_duplex;
		} else {
			sc->ure_10gbt_stat = 0;
		}
	}

	link_changed = (new_link != sc->ure_link_state ||
	    speed != sc->ure_link_speed);
	if (link_changed) {
		sc->ure_link_state = new_link;
		sc->ure_link_speed = speed;
		sc->ure_link_duplex = duplex;
	}
	mutex_exit(&sc->ure_lock);

	if (link_changed) {
		mac_link_update(sc->ure_mh, new_link);
	}

	/*
	 * TX watchdog: detect transfers stuck beyond the bulk_timeout.
	 * The 15-second threshold is 3× the 5-second bulk_timeout; if
	 * USBA's exception callback hasn't fired by now, something is
	 * genuinely wedged and a pipe reset is the only recovery.
	 */
	mutex_enter(&sc->ure_tx_lock);
	if (sc->ure_tx_cnt > 0) {
		if (sc->ure_tx_watchdog == 0) {
			sc->ure_tx_watchdog = gethrtime();
		} else if (gethrtime() - sc->ure_tx_watchdog >
		    15 * NANOSEC) {
			uint_t stuck = sc->ure_tx_cnt;
			sc->ure_tx_watchdog = 0;
			mutex_exit(&sc->ure_tx_lock);
			dev_err(sc->ure_dip, CE_WARN,
			    "TX watchdog: %u transfers stuck for "
			    ">15s, resetting pipe", stuck);
			usb_pipe_reset(sc->ure_dip,
			    sc->ure_bulkout_pipe,
			    USB_FLAGS_SLEEP, NULL, 0);
			ure_txc_discard(sc);
			return;
		}
	} else {
		sc->ure_tx_watchdog = 0;
	}
	mutex_exit(&sc->ure_tx_lock);
}

static void
ure_link_timer_start(ure_softc_t *sc)
{
	if (sc->ure_attach_seq & URE_ATTACH_LINK_TIMER) {
		return;
	}

	sc->ure_link_timer = ddi_periodic_add(
	    ure_link_check, sc, 1000000000ULL, DDI_IPL_0);
	sc->ure_attach_seq |= URE_ATTACH_LINK_TIMER;
}

static void
ure_link_timer_stop(ure_softc_t *sc)
{
	ddi_periodic_t timer;

	if ((sc->ure_attach_seq & URE_ATTACH_LINK_TIMER) == 0) {
		return;
	}

	timer = sc->ure_link_timer;
	sc->ure_link_timer = NULL;
	sc->ure_attach_seq &= ~URE_ATTACH_LINK_TIMER;

	ddi_periodic_delete(timer);
}

/*
 * RX filter (multicast hash, promisc)
 */

static int
ure_set_rx_filter(ure_softc_t *sc)
{
	uint32_t rxmode;
	uint32_t hashes[2];

	if (ure_read_4(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA, &rxmode) !=
	    USB_SUCCESS) {
		return (EIO);
	}
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

	if (ure_write_mem(sc, URE_PLA_MAR,
	    URE_MCU_TYPE_PLA | URE_BYTE_EN_DWORD,
	    hashes, sizeof (hashes)) != USB_SUCCESS) {
		return (EIO);
	}
	if (ure_write_4(sc, URE_PLA_RCR, URE_MCU_TYPE_PLA, rxmode) !=
	    USB_SUCCESS) {
		return (EIO);
	}

	return (0);
}

/*
 * RX path
 */

static void
ure_rx_start(ure_softc_t *sc)
{
	ASSERT(MUTEX_HELD(&sc->ure_lock));

	while (sc->ure_rx_cnt < URE_RX_LIST_CNT) {
		usb_bulk_req_t *req;

		if (sc->ure_gone || !sc->ure_running) {
			return;
		}

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

		sc->ure_rx_cnt++;

		if (usb_pipe_bulk_xfer(sc->ure_bulkin_pipe, req,
		    USB_FLAGS_NOSLEEP) != USB_SUCCESS) {
			dev_err(sc->ure_dip, CE_WARN,
			    "failed to start bulk RX transfer");
			usb_free_bulk_req(req);
			sc->ure_rx_cnt--;
			return;
		}
	}
}

/*
 * RX callback: called when a bulk IN transfer completes.
 *
 * Parses the aggregation buffer inline and passes the resulting mblk
 * chain to mac_rx().  This simplifies the stop path: since all RX
 * processing happens inside the USBA callback, usb_pipe_reset with
 * USB_FLAGS_SLEEP drains both in-flight USB transfers and their
 * callbacks.  Once the pipe reset returns, no further callbacks can
 * fire, so no mac_rx() call can occur after mc_stop returns (MAC
 * rule R17).  The ure_running/ure_gone checks inside this callback
 * are an optimisation for the common case, not the R17 guarantee.
 */
static void
ure_rx_cb(usb_pipe_handle_t ph, usb_bulk_req_t *req)
{
	_NOTE(ARGUNUSED(ph));
	ure_softc_t *sc = (ure_softc_t *)req->bulk_client_private;
	mblk_t *data = req->bulk_data;

	/* Snapshot state under lock to avoid bare-read races */
	mutex_enter(&sc->ure_lock);
	if (!sc->ure_running || sc->ure_gone) {
		ASSERT(sc->ure_rx_cnt > 0);
		sc->ure_rx_cnt--;
		mutex_exit(&sc->ure_lock);
		usb_free_bulk_req(req);
		return;
	}
	mutex_exit(&sc->ure_lock);

	if (req->bulk_completion_reason != USB_CR_OK) {
		if (req->bulk_completion_reason != USB_CR_STOPPED_POLLING) {
			atomic_add_64(&sc->ure_stat_ierrors, 1);
		}
		goto resubmit;
	}

	if (data == NULL) {
		goto resubmit;
	}

	/* Detach the data buffer from the USB request */
	req->bulk_data = NULL;

	/* Parse aggregated packets and build an mblk chain */
	{
		mblk_t *head = NULL, *tail = NULL;
		uint32_t total_len = MBLKL(data);
		uint32_t hdrsize, align;

		align = (sc->ure_flags & URE_FLAG_8157) ?
		    URE_8157_BUF_ALIGN : URE_RX_BUF_ALIGN;
		hdrsize = (sc->ure_flags & URE_FLAG_8157) ?
		    sizeof (ure_rxpkt_v2_t) : sizeof (ure_rxpkt_t);

		unsigned char *buf = data->b_rptr;
		uint32_t off = 0;

		while (total_len > hdrsize) {
			mblk_t *mp;
			int pktlen;
			uint32_t rxvlan = 0, rxcsum = 0;

			if (sc->ure_flags & URE_FLAG_8157) {
				ure_rxpkt_v2_t rxhdr;
				bcopy(buf + off, &rxhdr, sizeof (rxhdr));
				pktlen = (LE_32(rxhdr.ure_pktlen) &
				    URE_RXPKT_V2_LEN_MASK) >> 17;
				rxcsum = LE_32(rxhdr.ure_csum);
			} else {
				ure_rxpkt_t rxhdr;
				bcopy(buf + off, &rxhdr, sizeof (rxhdr));
				pktlen = LE_32(rxhdr.ure_pktlen) &
				    URE_RXPKT_LEN_MASK;
				rxvlan = LE_32(rxhdr.ure_vlan);
				rxcsum = LE_32(rxhdr.ure_csum);
			}

			off += hdrsize;
			total_len -= hdrsize;

			if (pktlen > ETHERMAX + VLAN_TAGSZ + ETHERFCSL) {
				atomic_add_64(&sc->ure_stat_ierrors, 1);
				break;
			}

			if (pktlen > (int)total_len || pktlen < ETHERMIN + ETHERFCSL) {
				atomic_add_64(&sc->ure_stat_ierrors, 1);
				break;
			}

			/* Strip CRC */
			int actual = pktlen - ETHERFCSL;
			if (actual <= 0) {
				atomic_add_64(&sc->ure_stat_ierrors, 1);
				break;
			}

			mp = allocb(actual + VLAN_TAGSZ, BPRI_MED);
			if (mp == NULL) {
				atomic_add_64(&sc->ure_stat_norcvbuf, 1);
				break;
			}

			bcopy(buf + off, mp->b_wptr, actual);
			mp->b_wptr += actual;

			/*
			 * RX checksum offload.
			 *
			 * v1 (8152/8153/8153B/8156/8156B): protocol
			 * type bits are in ure_vlan (DWORD 1), error
			 * flags in ure_csum (DWORD 2).
			 *
			 * v2 (8157): both protocol type and error
			 * flags are in ure_csum (DWORD 2).
			 */
			uint32_t hck_flags = 0;

			if (sc->ure_flags & URE_FLAG_8157) {
				if ((rxcsum & URE_RXPKT_V2_IPV4) &&
				    !(rxcsum & URE_RXPKT_V2_IPSUMBAD)) {
					hck_flags |= HCK_IPV4_HDRCKSUM_OK;
				}
				if ((rxcsum & (URE_RXPKT_V2_IPV4 |
				    URE_RXPKT_V2_IPV6)) &&
				    (((rxcsum & URE_RXPKT_V2_TCP) &&
				    !(rxcsum & URE_RXPKT_V2_TCPSUMBAD)) ||
				    ((rxcsum & URE_RXPKT_V2_UDP) &&
				    !(rxcsum & URE_RXPKT_V2_UDPSUMBAD)))) {
					hck_flags |= HCK_FULLCKSUM_OK;
				}
			} else {
				if ((rxvlan & URE_RXPKT_IPV4) &&
				    !(rxcsum & URE_RXPKT_IPSUMBAD)) {
					hck_flags |= HCK_IPV4_HDRCKSUM_OK;
				}
				if ((rxvlan & (URE_RXPKT_IPV4 |
				    URE_RXPKT_IPV6)) &&
				    (((rxvlan & URE_RXPKT_TCP) &&
				    !(rxcsum & URE_RXPKT_TCPSUMBAD)) ||
				    ((rxvlan & URE_RXPKT_UDP) &&
				    !(rxcsum & URE_RXPKT_UDPSUMBAD)))) {
					hck_flags |= HCK_FULLCKSUM_OK;
				}
			}

			if (hck_flags != 0 && sc->ure_hcksum_en) {
				mac_hcksum_set(mp, 0, 0, 0, 0, hck_flags);
			}

			if (ure_rxcsum_debug > 0) {
				ure_rxcsum_debug--;
				dev_err(sc->ure_dip, CE_NOTE,
				    "!RX csum: vlan=0x%08x csum=0x%08x "
				    "hck=0x%x pktlen=%d",
				    rxvlan, rxcsum, hck_flags, pktlen);
			}

			/* Chain it */
			if (head == NULL) {
				head = tail = mp;
			} else {
				tail->b_next = mp;
				tail = mp;
			}

			atomic_add_64(&sc->ure_stat_ipackets, 1);
			atomic_add_64(&sc->ure_stat_rbytes, actual);

			/* Multicast/broadcast RX stats */
			if (mp->b_rptr[0] & 0x01) {
				if (bcmp(mp->b_rptr, ure_bcast_addr,
				    ETHERADDRL) == 0) {
					atomic_add_64(
					    &sc->ure_stat_brdcstrcv, 1);
				} else {
					atomic_add_64(
					    &sc->ure_stat_multircv, 1);
				}
			}

			uint32_t consumed = P2ROUNDUP(pktlen, align);
			if (consumed > total_len) {
				break;
			}
			off += consumed;
			total_len -= consumed;
		}

		/* Pass received chain to MAC */
		if (head != NULL) {
			if (sc->ure_running && !sc->ure_gone) {
				mac_rx(sc->ure_mh, NULL, head);
			} else {
				freemsgchain(head);
			}
		}
	}

	freemsg(data);
	data = NULL;

resubmit:
	if (req->bulk_data != NULL) {
		req->bulk_data = NULL;
	}
	usb_free_bulk_req(req);
	if (data != NULL) {
		freemsg(data);
	}

	mutex_enter(&sc->ure_lock);
	ASSERT(sc->ure_rx_cnt > 0);
	sc->ure_rx_cnt--;
	if (sc->ure_running && !sc->ure_gone) {
		ure_rx_start(sc);
	}
	mutex_exit(&sc->ure_lock);
}
/*
 * TX path
 */

/*
 * TX completion callback - called by USBA when a bulk OUT transfer
 * completes (success or failure).  Updates statistics, detaches the
 * pre-allocated aggregation mblk (so usb_free_bulk_req doesn't free
 * it), returns the chain to the slab cache, wakes the MAC framework
 * if the TX pipeline was at capacity, and kicks any pending
 * coalescing data so the pipe stays busy.
 */
static void
ure_tx_cb(usb_pipe_handle_t ph, usb_bulk_req_t *req)
{
	_NOTE(ARGUNUSED(ph));
	ure_tx_chain_t *chain = (ure_tx_chain_t *)req->bulk_client_private;
	ure_softc_t *sc = chain->uc_sc;
	boolean_t was_full, do_update, pipe_draining, ok;
	uint64_t npkts, nbytes;

	/* Snapshot stats before we touch anything */
	ok = (req->bulk_completion_reason == USB_CR_OK);
	npkts = chain->uc_npkts;
	nbytes = chain->uc_nbytes;

	/* Detach our mblk before USBA frees req */
	req->bulk_data = NULL;
	usb_free_bulk_req(req);

	/*
	 * Decrement tx_cnt immediately so we know the pipe state.
	 * The pipe is IDLE in USBA right now (set before our callback),
	 * so any usb_pipe_bulk_xfer() we issue goes straight to the HCD.
	 */
	mutex_enter(&sc->ure_tx_lock);
	was_full = (sc->ure_tx_cnt >= URE_TX_MAX);
	sc->ure_tx_cnt--;
	pipe_draining = (sc->ure_tx_cnt <= 1);
	mutex_exit(&sc->ure_tx_lock);

	/*
	 * If the pipe is idle or down to one queued transfer, flush the
	 * coalescing buffer NOW, before any other bookkeeping, to
	 * minimise the window where no TD is on the xHCI ring.
	 *
	 * When tx_cnt == 0 the pipe is already idle and our
	 * usb_pipe_bulk_xfer() goes straight to the HCD.  When
	 * tx_cnt == 1 there is one transfer left in the USBA queue;
	 * flushing pre-stages the next transfer so that
	 * usba_start_next_req() always has work ready and the pipe
	 * never goes dry.
	 */
	if (pipe_draining) {
		mutex_enter(&sc->ure_txc_lock);
		if (sc->ure_txc_chain != NULL &&
		    sc->ure_txc_chain->uc_npkts > 0) {
			ure_txc_flush_locked(sc);
		}
		mutex_exit(&sc->ure_txc_lock);
	}

	/* Stats and cleanup: off the critical path now */
	if (ok) {
		atomic_add_64(&sc->ure_stat_opackets, npkts);
		atomic_add_64(&sc->ure_stat_obytes, nbytes);
	} else {
		atomic_add_64(&sc->ure_stat_oerrors, npkts);
	}

	chain->uc_buf->b_wptr = chain->uc_buf->b_rptr;
	kmem_cache_free(sc->ure_tx_cache, chain);

	if (was_full) {
		mutex_enter(&sc->ure_lock);
		do_update = sc->ure_running && !sc->ure_gone;
		mutex_exit(&sc->ure_lock);
		if (do_update) {
			mac_tx_update(sc->ure_mh);
		}
	}
}

/*
 * TX coalescing.
 *
 * USBA serialises bulk pipe submissions: only one transfer is active
 * on the wire at a time.  Without coalescing, MAC hands us one packet
 * per mc_tx call and each ~1500-byte frame costs a full USB round-trip,
 * capping throughput far below line rate.
 *
 * We solve this by accumulating packets in a coalescing buffer
 * (ure_txc_chain) and flushing to USB either when the buffer is full
 * or when a short timer fires.  This lets a single USB transfer carry
 * tens of kilobytes of aggregated frames.
 *
 * Lock discipline:
 *   ure_txc_lock protects ure_txc_chain, ure_txc_pos, ure_txc_tid.
 *   Never held across USB I/O or untimeout() calls.
 *   Lock order: ure_txc_lock -> ure_tx_lock.
 */

/*
 * Submit a filled TX chain to USB.  Called with NO locks held.
 * On failure the chain is freed and packets counted as oerrors.
 */
static void
ure_txc_submit(ure_softc_t *sc, ure_tx_chain_t *chain)
{
	usb_bulk_req_t *req;
	boolean_t was_full;
	boolean_t do_update;
	int rval;

	req = usb_alloc_bulk_req(sc->ure_dip, 0, USB_FLAGS_NOSLEEP);
	if (req == NULL) {
		atomic_add_64(&sc->ure_stat_oerrors, chain->uc_npkts);
		chain->uc_buf->b_wptr = chain->uc_buf->b_rptr;
		kmem_cache_free(sc->ure_tx_cache, chain);
		mutex_enter(&sc->ure_tx_lock);
		was_full = (sc->ure_tx_cnt >= URE_TX_MAX);
		sc->ure_tx_cnt--;
		mutex_exit(&sc->ure_tx_lock);
		if (was_full) {
			mutex_enter(&sc->ure_lock);
			do_update = sc->ure_running && !sc->ure_gone;
			mutex_exit(&sc->ure_lock);
			if (do_update) {
				mac_tx_update(sc->ure_mh);
			}
		}
		return;
	}

	req->bulk_data = chain->uc_buf;
	req->bulk_len = MBLKL(chain->uc_buf);
	req->bulk_timeout = 5;
	req->bulk_cb = ure_tx_cb;
	req->bulk_exc_cb = ure_tx_cb;
	req->bulk_client_private = (usb_opaque_t)chain;
	req->bulk_attributes = USB_ATTRS_AUTOCLEARING;

	rval = usb_pipe_bulk_xfer(sc->ure_bulkout_pipe, req,
	    USB_FLAGS_NOSLEEP);
	if (rval != USB_SUCCESS) {
		atomic_add_64(&sc->ure_stat_oerrors, chain->uc_npkts);
		req->bulk_data = NULL;
		usb_free_bulk_req(req);
		chain->uc_buf->b_wptr = chain->uc_buf->b_rptr;
		kmem_cache_free(sc->ure_tx_cache, chain);
		mutex_enter(&sc->ure_tx_lock);
		was_full = (sc->ure_tx_cnt >= URE_TX_MAX);
		sc->ure_tx_cnt--;
		mutex_exit(&sc->ure_tx_lock);
		if (was_full) {
			mutex_enter(&sc->ure_lock);
			do_update = sc->ure_running && !sc->ure_gone;
			mutex_exit(&sc->ure_lock);
			if (do_update) {
				mac_tx_update(sc->ure_mh);
			}
		}
	}
}

/*
 * Flush the coalescing buffer: detach it and submit to USB.
 * Called with ure_txc_lock HELD; drops and reacquires it.
 */
static void
ure_txc_flush_locked(ure_softc_t *sc)
{
	ure_tx_chain_t *chain;
	timeout_id_t tid;

	ASSERT(MUTEX_HELD(&sc->ure_txc_lock));

	chain = sc->ure_txc_chain;
	if (chain == NULL || chain->uc_npkts == 0) {
		return;
	}

	/* Finalize the mblk length */
	chain->uc_buf->b_wptr = chain->uc_buf->b_rptr + sc->ure_txc_pos;

	/* Detach from coalescing state */
	tid = sc->ure_txc_tid;
	sc->ure_txc_chain = NULL;
	sc->ure_txc_pos = 0;
	sc->ure_txc_tid = 0;

	mutex_exit(&sc->ure_txc_lock);

	if (tid != 0) {
		(void) untimeout(tid);
	}

	ure_txc_submit(sc, chain);

	mutex_enter(&sc->ure_txc_lock);
}

/*
 * Discard the coalescing buffer without submitting.
 * Used during stop, suspend, and disconnect.
 * Safe to call when there is no coalescing state.
 */
static void
ure_txc_discard(ure_softc_t *sc)
{
	ure_tx_chain_t *chain;
	timeout_id_t tid;

	mutex_enter(&sc->ure_txc_lock);
	chain = sc->ure_txc_chain;
	tid = sc->ure_txc_tid;
	sc->ure_txc_chain = NULL;
	sc->ure_txc_pos = 0;
	sc->ure_txc_tid = 0;
	mutex_exit(&sc->ure_txc_lock);

	if (tid != 0) {
		(void) untimeout(tid);
	}

	if (chain != NULL) {
		chain->uc_buf->b_wptr = chain->uc_buf->b_rptr;
		kmem_cache_free(sc->ure_tx_cache, chain);
		mutex_enter(&sc->ure_tx_lock);
		sc->ure_tx_cnt--;
		mutex_exit(&sc->ure_tx_lock);
	}
}

/*
 * Coalescing timer callback: flush whatever has accumulated.
 */
static void
ure_txc_timeout(void *arg)
{
	ure_softc_t *sc = (ure_softc_t *)arg;
	ure_tx_chain_t *chain;

	mutex_enter(&sc->ure_txc_lock);
	sc->ure_txc_tid = 0;

	chain = sc->ure_txc_chain;
	if (chain == NULL || chain->uc_npkts == 0) {
		mutex_exit(&sc->ure_txc_lock);
		return;
	}

	/* Finalize and detach */
	chain->uc_buf->b_wptr = chain->uc_buf->b_rptr + sc->ure_txc_pos;
	sc->ure_txc_chain = NULL;
	sc->ure_txc_pos = 0;

	mutex_exit(&sc->ure_txc_lock);

	ure_txc_submit(sc, chain);
}

/*
 * TX offload descriptor setup.
 *
 * Derive offload descriptor bits from the mblk chain using
 * mac_ether_offload_info() instead of manually parsing headers
 * from mp->b_rptr.  MAC does not guarantee that the Ethernet, IP,
 * and L4 headers are contiguous in the first mblk, so direct
 * pointer casts into mp->b_rptr are unsafe for chained mblks.
 *
 * For TSO, the chip requires the IP length field to be zeroed
 * before transmission.  This mutation requires contiguous access
 * to the IP header, so we pull up the necessary bytes first.
 *
 * Returns B_TRUE on success (offload state filled, mp possibly
 * reallocated via pullup).  Returns B_FALSE if the requested
 * offload cannot be set up safely; caller should drop it.
 */
static boolean_t
ure_tx_offload(ure_softc_t *sc, mblk_t **mpp, ure_tx_offload_t *ofl)
{
	mblk_t *mp = *mpp;
	uint32_t mss_val, lso_flag;
	mac_ether_offload_info_t meoi;

	bzero(ofl, sizeof (*ofl));
	mac_lso_get(mp, &mss_val, &lso_flag);

	if (lso_flag == HW_LSO) {
		/*
		 * TSO: need L2/L3/L4 info to set the transport
		 * offset and zero the IP length field.
		 */
		mac_ether_offload_info(mp, &meoi);

		if (!(meoi.meoi_flags & MEOI_L2INFO_SET) ||
		    !(meoi.meoi_flags & MEOI_L3INFO_SET) ||
		    !(meoi.meoi_flags & MEOI_L4INFO_SET) ||
		    meoi.meoi_l4proto != IPPROTO_TCP) {
			return (B_FALSE);
		}

		uint32_t l4off = meoi.meoi_l2hlen + meoi.meoi_l3hlen;

		/*
		 * The chip needs the IP length field zeroed.
		 * Ensure the headers are contiguous so we can
		 * safely write through a pointer.
		 */
		if (MBLKL(mp) < l4off) {
			mblk_t *nmp = msgpullup(mp, l4off);
			if (nmp == NULL) {
				return (B_FALSE);
			}
			freemsg(mp);
			mp = nmp;
			*mpp = mp;
		}

		if (meoi.meoi_l3proto == ETHERTYPE_IP) {
			ipha_t *ipha = (ipha_t *)
			    (mp->b_rptr + meoi.meoi_l2hlen);
			ipha->ipha_length = 0;
			ofl->uto_opts1 |= URE_TXPKT_GTSENDV4;
		} else if (meoi.meoi_l3proto == ETHERTYPE_IPV6) {
			ip6_t *ip6 = (ip6_t *)
			    (mp->b_rptr + meoi.meoi_l2hlen);
			ip6->ip6_plen = 0;
			ofl->uto_opts1 |= URE_TXPKT_GTSENDV6;
		} else {
			return (B_FALSE);
		}

		if (l4off > URE_TXPKT_GTTCPHO_MAX) {
			return (B_FALSE);
		}
		ofl->uto_opts1 |= l4off << URE_TXPKT_GTTCPHO_SHIFT;
		ofl->uto_opts2 = MIN(mss_val, URE_TXPKT_MSS_MAX) <<
		    URE_TXPKT_MSS_SHIFT;
	} else {
		/*
		 * Non-TSO checksum offload.
		 */
		uint32_t hck_flags, hck_start, hck_stuff;
		mac_hcksum_get(mp, &hck_start, &hck_stuff, NULL, NULL,
		    &hck_flags);

		if (hck_flags & HCK_IPV4_HDRCKSUM) {
			ofl->uto_opts2 |= URE_TXPKT_IPV4;
		}
		if (hck_flags & HCK_PARTIALCKSUM) {
			mac_ether_offload_info(mp, &meoi);

			if (!((meoi.meoi_flags & MEOI_L2INFO_SET) &&
			    (meoi.meoi_flags & MEOI_L3INFO_SET) &&
			    (meoi.meoi_flags & MEOI_L4INFO_SET))) {
				return (B_FALSE);
			}

			uint32_t l4off = meoi.meoi_l2hlen +
			    meoi.meoi_l3hlen;
			uint32_t l4max = URE_TXPKT_L4_OFFSET_MAX;
			uint32_t mlen = msgsize(mp);
			uint32_t sw_start = meoi.meoi_l2hlen + hck_start;
			uint32_t sw_stuff = meoi.meoi_l2hlen + hck_stuff;

			if (sw_start >= mlen || sw_stuff < sw_start ||
			    sw_stuff > mlen ||
			    mlen - sw_stuff < sizeof (uint16_t)) {
				return (B_FALSE);
			}

			if (meoi.meoi_l3proto != ETHERTYPE_IP &&
			    meoi.meoi_l3proto != ETHERTYPE_IPV6) {
				return (B_FALSE);
			}

			if (meoi.meoi_l4proto != IPPROTO_TCP &&
			    meoi.meoi_l4proto != IPPROTO_UDP) {
				return (B_FALSE);
			}

			if (sc->ure_flags & URE_FLAG_8157) {
				l4max = URE_TXPKT_V2_L4_OFFSET_MAX;
			}
			if (l4off > l4max) {
				ofl->uto_sw_csum = B_TRUE;
				ofl->uto_sw_start = sw_start;
				ofl->uto_sw_stuff = sw_stuff;
				return (B_TRUE);
			}

			if (meoi.meoi_l3proto == ETHERTYPE_IP) {
				ofl->uto_opts2 |= URE_TXPKT_IPV4;
			} else if (meoi.meoi_l3proto == ETHERTYPE_IPV6) {
				ofl->uto_opts2 |= URE_TXPKT_IPV6;
			}

			if (meoi.meoi_l4proto == IPPROTO_TCP) {
				ofl->uto_opts2 |= URE_TXPKT_TCP;
			} else {
				ofl->uto_opts2 |= URE_TXPKT_UDP;
			}

			ofl->uto_opts2 |= l4off << URE_TXPKT_L4_OFFSET_SHIFT;
		}
	}

	return (B_TRUE);
}

static boolean_t
ure_tx_sw_csum(const ure_tx_offload_t *ofl, uchar_t *buf, uint32_t len)
{
	unsigned int sum;
	uint16_t csum;

	if (!ofl->uto_sw_csum) {
		return (B_TRUE);
	}

	if (ofl->uto_sw_start >= len ||
	    ofl->uto_sw_stuff < ofl->uto_sw_start ||
	    ofl->uto_sw_stuff > len ||
	    len - ofl->uto_sw_stuff < sizeof (csum)) {
		return (B_FALSE);
	}

	sum = bcksum(buf + ofl->uto_sw_start,
	    len - ofl->uto_sw_start, 0);
	sum = (sum >> 16) + (sum & 0xffff);
	sum = (sum >> 16) + (sum & 0xffff);
	csum = (uint16_t)~sum;
	if (csum == 0) {
		csum = 0xffff;
	}

	bcopy(&csum, buf + ofl->uto_sw_stuff, sizeof (csum));
	return (B_TRUE);
}

/*
 * mc_tx callback: transmit a chain of mblk_t packets.
 *
 * Packs frames into a coalescing buffer.  The buffer is flushed to
 * USB when full or when the coalescing timer fires, whichever comes
 * first.  Packet data is copied immediately and the original mblks
 * freed, so no mblk lifetime spans the coalescing window.
 */
static mblk_t *
ure_m_tx(void *arg, mblk_t *mp)
{
	ure_softc_t *sc = (ure_softc_t *)arg;
	uint32_t hdrsize, pkt_align;

	/* 1. State check */
	mutex_enter(&sc->ure_lock);
	if (sc->ure_gone || !sc->ure_running ||
	    !(sc->ure_flags & URE_FLAG_LINK)) {
		mutex_exit(&sc->ure_lock);
		freemsgchain(mp);
		return (NULL);
	}
	mutex_exit(&sc->ure_lock);

	hdrsize = (sc->ure_flags & URE_FLAG_8157) ?
	    sizeof (ure_txpkt_v2_t) : sizeof (ure_txpkt_t);
	pkt_align = (sc->ure_flags & URE_FLAG_8157) ?
	    URE_8157_BUF_ALIGN : URE_TX_BUF_ALIGN;

	mutex_enter(&sc->ure_txc_lock);

	/* Re-check after lock transition: disconnect may have fired */
	if (sc->ure_gone) {
		mutex_exit(&sc->ure_txc_lock);
		freemsgchain(mp);
		return (NULL);
	}

	/* 2. Pack frames into the coalescing buffer */
	while (mp != NULL) {
		uint32_t mlen, aligned_pos, data_pos;
		unsigned char *buf;
		ure_tx_offload_t ofl;

		/* 2a. Ensure we have a coalescing buffer */
		if (sc->ure_txc_chain == NULL) {
			mutex_enter(&sc->ure_tx_lock);
			if (sc->ure_tx_cnt >= URE_TX_MAX) {
				mutex_exit(&sc->ure_tx_lock);
				break;		/* backpressure */
			}
			sc->ure_tx_cnt++;
			mutex_exit(&sc->ure_tx_lock);

			sc->ure_txc_chain = kmem_cache_alloc(
			    sc->ure_tx_cache, KM_NOSLEEP);
			if (sc->ure_txc_chain == NULL) {
				mutex_enter(&sc->ure_tx_lock);
				sc->ure_tx_cnt--;
				mutex_exit(&sc->ure_tx_lock);
				break;		/* backpressure */
			}
			sc->ure_txc_chain->uc_npkts = 0;
			sc->ure_txc_chain->uc_nbytes = 0;
			sc->ure_txc_pos = 0;
		}

		/* 2b. Check fit */
		mlen = msgsize(mp);
		aligned_pos = P2ROUNDUP(sc->ure_txc_pos, pkt_align);

		/*
		 * mlen is bounded by the MAC MTU (at most ~9000 for
		 * jumbo, typically ~1518), so the addition below
		 * cannot overflow uint32_t in practice.
		 */
		if (aligned_pos + hdrsize + mlen >
		    sc->ure_txc_chain->uc_bufmax) {
			if (sc->ure_txc_chain->uc_npkts == 0) {
				/*
				 * Single frame exceeds the TX buffer.
				 * Drop it and continue with the next.
				 */
				mblk_t *next = mp->b_next;
				mp->b_next = NULL;
				freemsg(mp);
				mp = next;
				atomic_add_64(
				    &sc->ure_stat_oerrors, 1);
				continue;
			}
			/* Buffer full, flush and retry */
			ure_txc_flush_locked(sc);
			continue;
		}

		sc->ure_txc_pos = aligned_pos;
		buf = sc->ure_txc_chain->uc_buf->b_rptr;

		/* 2c. Write TX header */
		{
			mblk_t *next = mp->b_next;
			mp->b_next = NULL;

			if (!ure_tx_offload(sc, &mp, &ofl)) {
				/*
				 * Offload setup failed.  Drop the packet rather
				 * than sending it with an invalid descriptor.
				 */
				freemsg(mp);
				mp = next;
				atomic_add_64(&sc->ure_stat_oerrors, 1);
				continue;
			}
			mp->b_next = next;

			/*
			 * msgsize may have changed after pullup,
			 * update our copy.
			 */
			mlen = msgsize(mp);

			if (sc->ure_flags & URE_FLAG_8157) {
				ure_txpkt_v2_t txhdr;

				bzero(&txhdr, sizeof (txhdr));
				txhdr.ure_cmdstat = LE_32(mlen |
				    URE_TXPKT_TX_FS |
				    URE_TXPKT_TX_LS | ofl.uto_opts1);
				txhdr.ure_vlan = LE_32(ofl.uto_opts2);
				txhdr.ure_pktlen =
				    LE_32(mlen << 4);
				txhdr.ure_signature = LE_32(
				    URE_TXPKT_SIGNATURE);
				bcopy(&txhdr,
				    buf + sc->ure_txc_pos,
				    sizeof (txhdr));
			} else {
				ure_txpkt_t txhdr;

				bzero(&txhdr, sizeof (txhdr));
				txhdr.ure_pktlen = LE_32(mlen |
				    URE_TXPKT_TX_FS |
				    URE_TXPKT_TX_LS | ofl.uto_opts1);
				txhdr.ure_vlan = LE_32(ofl.uto_opts2);
				bcopy(&txhdr,
				    buf + sc->ure_txc_pos,
				    sizeof (txhdr));
			}
		}
		sc->ure_txc_pos += hdrsize;
		data_pos = sc->ure_txc_pos;

		/* 2d. Copy packet data */
		{
			mblk_t *m;
			for (m = mp; m != NULL; m = m->b_cont) {
				uint32_t len = MBLKL(m);
				if (len > 0) {
					bcopy(m->b_rptr,
					    buf + sc->ure_txc_pos, len);
					sc->ure_txc_pos += len;
				}
			}
		}

		if (!ure_tx_sw_csum(&ofl, buf + data_pos, mlen)) {
			mblk_t *next = mp->b_next;
			sc->ure_txc_pos = aligned_pos;
			mp->b_next = NULL;
			freemsg(mp);
			mp = next;
			atomic_add_64(&sc->ure_stat_oerrors, 1);
			continue;
		}

		sc->ure_txc_chain->uc_npkts++;
		sc->ure_txc_chain->uc_nbytes += mlen;

		/* TX multicast/broadcast stats */
		if (mp->b_rptr[0] & 0x01) {
			if (bcmp(mp->b_rptr, ure_bcast_addr,
			    ETHERADDRL) == 0) {
				atomic_add_64(
				    &sc->ure_stat_brdcstxmt, 1);
			} else {
				atomic_add_64(
				    &sc->ure_stat_multixmt, 1);
			}
		}

		/* 2e. Free this mblk and advance */
		{
			mblk_t *next = mp->b_next;
			mp->b_next = NULL;
			freemsg(mp);
			mp = next;
		}
	}

	/* 3. Arm the coalescing timer if data is pending */
	if (sc->ure_txc_chain != NULL && sc->ure_txc_chain->uc_npkts > 0 &&
	    sc->ure_txc_tid == 0) {
		sc->ure_txc_tid = timeout(ure_txc_timeout, sc,
		    drv_usectohz(URE_TX_COAL_USEC));
	}

	mutex_exit(&sc->ure_txc_lock);

	return (mp);
}

/*
 * MAC callbacks
 */

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
	case ETHER_STAT_CAP_2500FDX:
		*val = ure_has_2500fdx(sc);
		break;
	case ETHER_STAT_ADV_CAP_2500FDX:
		*val = ure_has_2500fdx(sc) &&
		    !!(sc->ure_10gbt_ctrl & URE_ADV_2500TFDX);
		break;
	case ETHER_STAT_LP_CAP_2500FDX:
		*val = ure_has_2500fdx(sc) &&
		    !!(sc->ure_10gbt_stat & URE_LP_2500TFDX);
		break;
	case ETHER_STAT_CAP_5000FDX:
		*val = ure_has_5000fdx(sc);
		break;
	case ETHER_STAT_ADV_CAP_5000FDX:
		*val = ure_has_5000fdx(sc) &&
		    !!(sc->ure_10gbt_ctrl & URE_ADV_5000TFDX);
		break;
	case ETHER_STAT_LP_CAP_5000FDX:
		*val = ure_has_5000fdx(sc) &&
		    !!(sc->ure_10gbt_stat & URE_LP_5000TFDX);
		break;
	case MAC_STAT_MULTIRCV:
		*val = sc->ure_stat_multircv;
		break;
	case MAC_STAT_BRDCSTRCV:
		*val = sc->ure_stat_brdcstrcv;
		break;
	case MAC_STAT_MULTIXMT:
		*val = sc->ure_stat_multixmt;
		break;
	case MAC_STAT_BRDCSTXMT:
		*val = sc->ure_stat_brdcstxmt;
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
	mutex_exit(&sc->ure_lock);

	/* USB I/O - must not hold ure_lock (MUTEX_DRIVER) */
	if ((error = ure_phy_powerup(sc)) != 0) {
		return (EIO);
	}

	if (sc->ure_flags & URE_FLAG_8152) {
		error = ure_rtl8152_nic_reset(sc);
	} else {
		error = ure_rtl8153_nic_reset(sc);
	}
	if (error != 0) {
		(void) ure_phy_powerdown(sc);
		return (EIO);
	}

	if ((error = ure_ifmedia_init(sc)) != 0) {
		(void) ure_phy_powerdown(sc);
		return (EIO);
	}
	if (ure_set_rx_filter(sc) != 0) {
		(void) ure_phy_powerdown(sc);
		return (EIO);
	}

	/* Reacquire lock to update state and start RX */
	mutex_enter(&sc->ure_lock);
	if (sc->ure_gone) {
		mutex_exit(&sc->ure_lock);
		(void) ure_phy_powerdown(sc);
		return (EIO);
	}

	sc->ure_running = B_TRUE;
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
	sc->ure_flags &= ~URE_FLAG_LINK;
	sc->ure_link_state = LINK_STATE_DOWN;
	mutex_exit(&sc->ure_lock);

	mac_link_update(sc->ure_mh, LINK_STATE_DOWN);

	/* Discard any pending coalescing buffer */
	ure_txc_discard(sc);

	/* Reset the chip to stop RX/TX (best-effort in stop path) */
	(void) ure_reset(sc);

	/*
	 * Power down the PHY so the link drops on the wire.
	 * The nic_reset path in ure_m_start clears PDOWN.
	 * Best-effort in stop path.
	 */
	(void) ure_phy_powerdown(sc);

	/*
	 * Drain pipes.  usb_pipe_reset(USB_FLAGS_SLEEP) waits for
	 * all in-flight transfers to complete and their callbacks
	 * to return before it returns.  Because RX packet processing
	 * runs inline in the callback, this guarantees that all
	 * mac_rx() upcalls have completed (MAC rule R17).
	 */
	usb_pipe_reset(sc->ure_dip, sc->ure_bulkin_pipe,
	    USB_FLAGS_SLEEP, NULL, 0);
	usb_pipe_reset(sc->ure_dip, sc->ure_bulkout_pipe,
	    USB_FLAGS_SLEEP, NULL, 0);

	/* Verify all TX transfers drained */
	mutex_enter(&sc->ure_tx_lock);
	ASSERT(sc->ure_tx_cnt == 0);
	mutex_exit(&sc->ure_tx_lock);

	/* Verify all RX transfers drained */
	mutex_enter(&sc->ure_lock);
	ASSERT(sc->ure_rx_cnt == 0);
	mutex_exit(&sc->ure_lock);
}

static int
ure_m_promisc(void *arg, boolean_t on)
{
	ure_softc_t *sc = (ure_softc_t *)arg;
	boolean_t old_promisc;
	boolean_t running;

	mutex_enter(&sc->ure_lock);
	old_promisc = sc->ure_promisc;
	sc->ure_promisc = on;
	running = sc->ure_running;
	mutex_exit(&sc->ure_lock);

	if (running) {
		if (ure_set_rx_filter(sc) != 0) {
			mutex_enter(&sc->ure_lock);
			sc->ure_promisc = old_promisc;
			mutex_exit(&sc->ure_lock);
			return (EIO);
		}
	}

	return (0);
}

/*
 * Compute CRC32-BE hash for a multicast address and return the
 * 6-bit hash table index.
 */
static uint32_t
ure_mcast_hash_bit(const uint8_t *addr)
{
	uint32_t crc = 0xffffffff;
	int i, j;

	/*
	 * Compute the Ethernet multicast hash using the standard CRC-32
	 * polynomial (0x04c11db7) with each byte processed LSB-first to
	 * match the Ethernet wire bit ordering.  This is equivalent to
	 * Linux's ether_crc() (= bitrev32(crc32_le())).
	 */
	for (i = 0; i < ETHERADDRL; i++) {
		uint8_t c = addr[i];
		for (j = 0; j < 8; j++) {
			if (((crc >> 31) ^ (c & 1)) != 0) {
				crc = (crc << 1) ^ 0x04c11db7;
			} else {
				crc <<= 1;
			}
			c >>= 1;
		}
	}
	return ((crc >> 26) & 0x3f);
}

/*
 * Rebuild the 64-bit multicast hash from the tracked address list.
 * Caller must hold ure_lock.
 */
static void
ure_mcast_hash_rebuild(ure_softc_t *sc)
{
	ure_mcast_entry_t *me;
	uint32_t bit;

	ASSERT(MUTEX_HELD(&sc->ure_lock));

	sc->ure_mcast_hash[0] = 0;
	sc->ure_mcast_hash[1] = 0;

	for (me = list_head(&sc->ure_mcast_list); me != NULL;
	    me = list_next(&sc->ure_mcast_list, me)) {
		bit = ure_mcast_hash_bit(me->addr);
		if (bit < 32) {
			sc->ure_mcast_hash[0] |= (1U << bit);
		} else {
			sc->ure_mcast_hash[1] |= (1U << (bit - 32));
		}
	}
}

static int
ure_m_multicst(void *arg, boolean_t add, const uint8_t *mca)
{
	ure_softc_t *sc = (ure_softc_t *)arg;
	ure_mcast_entry_t *me;
	boolean_t running;

	mutex_enter(&sc->ure_lock);
	if (add) {
		me = kmem_zalloc(sizeof (*me), KM_NOSLEEP);
		if (me == NULL) {
			mutex_exit(&sc->ure_lock);
			return (ENOMEM);
		}
		bcopy(mca, me->addr, ETHERADDRL);
		list_insert_tail(&sc->ure_mcast_list, me);
	} else {
		for (me = list_head(&sc->ure_mcast_list); me != NULL;
		    me = list_next(&sc->ure_mcast_list, me)) {
			if (bcmp(me->addr, mca, ETHERADDRL) == 0) {
				list_remove(&sc->ure_mcast_list, me);
				break;
			}
		}
	}
	ure_mcast_hash_rebuild(sc);
	running = sc->ure_running;
	mutex_exit(&sc->ure_lock);

	if (running) {
		if (ure_set_rx_filter(sc) != 0) {
			/*
			 * Roll back the list change so software state
			 * stays in sync with what the hardware has.
			 */
			mutex_enter(&sc->ure_lock);
			if (add) {
				list_remove(&sc->ure_mcast_list, me);
				kmem_free(me, sizeof (*me));
			} else if (me != NULL) {
				list_insert_tail(&sc->ure_mcast_list, me);
			}
			ure_mcast_hash_rebuild(sc);
			mutex_exit(&sc->ure_lock);
			return (EIO);
		}
	}
	/* Free the removed entry only after the filter is programmed */
	if (!add && me != NULL) {
		kmem_free(me, sizeof (*me));
	}

	return (0);
}

static int
ure_m_unicst(void *arg, const uint8_t *macaddr)
{
	ure_softc_t *sc = (ure_softc_t *)arg;
	uint8_t old_addr[ETHERADDRL];
	boolean_t running;
	int err = 0;

	mutex_enter(&sc->ure_lock);
	bcopy(sc->ure_dev_addr, old_addr, ETHERADDRL);
	bcopy(macaddr, sc->ure_dev_addr, ETHERADDRL);
	running = sc->ure_running;
	mutex_exit(&sc->ure_lock);

	if (running) {
		uint8_t addr[8] = {0};
		bcopy(macaddr, addr, ETHERADDRL);
		if (ure_write_1(sc, URE_PLA_CRWECR,
		    URE_MCU_TYPE_PLA,
		    URE_CRWECR_CONFIG) != USB_SUCCESS) {
			err = EIO;
		} else {
			if (ure_write_mem(sc, URE_PLA_IDR,
			    URE_MCU_TYPE_PLA | URE_BYTE_EN_SIX_BYTES,
			    addr, sizeof (addr)) != USB_SUCCESS) {
				err = EIO;
			}
			if (ure_write_1(sc, URE_PLA_CRWECR,
			    URE_MCU_TYPE_PLA,
			    URE_CRWECR_NORMAL) != USB_SUCCESS) {
				err = EIO;
			}
		}
	}

	if (err != 0) {
		mutex_enter(&sc->ure_lock);
		bcopy(old_addr, sc->ure_dev_addr, ETHERADDRL);
		mutex_exit(&sc->ure_lock);
	}

	return (err);
}

static int
ure_m_getprop(void *arg, const char *pr_name,
    mac_prop_id_t pr_num, uint_t pr_valsize, void *pr_val)
{
	ure_softc_t *sc = (ure_softc_t *)arg;

	_NOTE(ARGUNUSED(pr_name));

	switch (pr_num) {
	case MAC_PROP_DUPLEX:
		ASSERT(pr_valsize >= sizeof (link_duplex_t));
		bcopy(&sc->ure_link_duplex, pr_val,
		    sizeof (link_duplex_t));
		break;
	case MAC_PROP_SPEED:
		ASSERT(pr_valsize >= sizeof (uint64_t));
		bcopy(&sc->ure_link_speed, pr_val, sizeof (uint64_t));
		break;
	case MAC_PROP_STATUS:
		ASSERT(pr_valsize >= sizeof (link_state_t));
		bcopy(&sc->ure_link_state, pr_val,
		    sizeof (link_state_t));
		break;
	case MAC_PROP_AUTONEG:
		*(uint8_t *)pr_val = 1;
		break;
	case MAC_PROP_FLOWCTRL: {
		link_flowctrl_t fc = LINK_FLOWCTRL_NONE;
		ASSERT(pr_valsize >= sizeof (link_flowctrl_t));
		bcopy(&fc, pr_val, sizeof (fc));
		break;
	}
	case MAC_PROP_MTU:
		ASSERT(pr_valsize >= sizeof (uint32_t));
		bcopy(&sc->ure_mtu, pr_val, sizeof (uint32_t));
		break;
	case MAC_PROP_ADV_2500FDX_CAP:
	case MAC_PROP_EN_2500FDX_CAP:
		*(uint8_t *)pr_val = ure_has_2500fdx(sc) &&
		    !!(sc->ure_10gbt_ctrl & URE_ADV_2500TFDX);
		break;
	case MAC_PROP_ADV_5000FDX_CAP:
	case MAC_PROP_EN_5000FDX_CAP:
		*(uint8_t *)pr_val = ure_has_5000fdx(sc) &&
		    !!(sc->ure_10gbt_ctrl & URE_ADV_5000TFDX);
		break;
	default:
		return (ENOTSUP);
	}
	return (0);
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
	_NOTE(ARGUNUSED(arg, pr_name));

	switch (pr_num) {
	case MAC_PROP_DUPLEX:
	case MAC_PROP_SPEED:
	case MAC_PROP_STATUS:
		mac_prop_info_set_perm(prh, MAC_PROP_PERM_READ);
		break;
	case MAC_PROP_AUTONEG:
		mac_prop_info_set_perm(prh, MAC_PROP_PERM_READ);
		mac_prop_info_set_default_uint8(prh, 1);
		break;
	case MAC_PROP_FLOWCTRL:
		mac_prop_info_set_perm(prh, MAC_PROP_PERM_READ);
		mac_prop_info_set_default_link_flowctrl(prh,
		    LINK_FLOWCTRL_NONE);
		break;
	case MAC_PROP_MTU:
		mac_prop_info_set_perm(prh, MAC_PROP_PERM_READ);
		mac_prop_info_set_range_uint32(prh, ETHERMTU, ETHERMTU);
		break;
	case MAC_PROP_ADV_2500FDX_CAP:
	case MAC_PROP_EN_2500FDX_CAP:
	case MAC_PROP_ADV_5000FDX_CAP:
	case MAC_PROP_EN_5000FDX_CAP:
		mac_prop_info_set_perm(prh, MAC_PROP_PERM_READ);
		break;
	default:
		break;
	}
}

static boolean_t
ure_m_getcapab(void *arg, mac_capab_t cap, void *cap_data)
{
	ure_softc_t *sc = (ure_softc_t *)arg;

	switch (cap) {
	case MAC_CAPAB_HCKSUM: {
		uint32_t *flags = (uint32_t *)cap_data;
		if (!sc->ure_hcksum_en) {
			*flags = 0;
			return (B_FALSE);
		}
		*flags = HCKSUM_IPHDRCKSUM | HCKSUM_INET_PARTIAL;
		return (B_TRUE);
	}
	case MAC_CAPAB_LSO: {
		mac_capab_lso_t *cap_lso = (mac_capab_lso_t *)cap_data;
		uint32_t hdrsize, lso_max_v4, lso_max_v6;

		/*
		 * LSO requires checksum offload.  Skip the 8152
		 * (Fast Ethernet) - TSO is pointless at 100 Mbps.
		 */
		if (!sc->ure_lso_en || !sc->ure_hcksum_en ||
		    (sc->ure_flags & URE_FLAG_8152)) {
			return (B_FALSE);
		}

		hdrsize = (sc->ure_flags & URE_FLAG_8157) ?
		    sizeof (ure_txpkt_v2_t) : sizeof (ure_txpkt_t);

		/*
		 * Maximum TCP payload that fits in one USB transfer:
		 * buffer minus TX descriptor, worst-case L2 (VLAN),
		 * maximum L3, and maximum TCP headers.
		 */
		lso_max_v4 = sc->ure_txbufsz - hdrsize -
		    (sizeof (struct ether_header) + VLAN_TAGSZ) -
		    60 - 60;
		lso_max_v6 = sc->ure_txbufsz - hdrsize -
		    (sizeof (struct ether_header) + VLAN_TAGSZ) -
		    sizeof (ip6_t) - 60;

		cap_lso->lso_flags = LSO_TX_BASIC_TCP_IPV4 |
		    LSO_TX_BASIC_TCP_IPV6;
		cap_lso->lso_basic_tcp_ipv4.lso_max = lso_max_v4;
		cap_lso->lso_basic_tcp_ipv6.lso_max = lso_max_v6;
		return (B_TRUE);
	}
	default:
		return (B_FALSE);
	}
}

/*
 * TX chain kmem_cache constructor. Pre-allocates the aggregation
 * buffer so the TX hot path never calls allocb().
 */
static int
ure_tx_chain_construct(void *buf, void *arg, int kmflags)
{
	ure_tx_chain_t *chain = (ure_tx_chain_t *)buf;
	ure_softc_t *sc = (ure_softc_t *)arg;

	chain->uc_sc = sc;
	chain->uc_bufmax = sc->ure_txbufsz;
	chain->uc_buf = allocb(chain->uc_bufmax,
	    (kmflags == KM_SLEEP) ? BPRI_LO : BPRI_MED);
	if (chain->uc_buf == NULL) {
		return (-1);
	}
	chain->uc_npkts = 0;
	chain->uc_nbytes = 0;

	return (0);
}

/*
 * TX chain kmem_cache destructor. Frees the pre-allocated
 * aggregation buffer.
 */
static void
ure_tx_chain_destroy(void *buf, void *arg)
{
	_NOTE(ARGUNUSED(arg));
	ure_tx_chain_t *chain = (ure_tx_chain_t *)buf;

	if (chain->uc_buf != NULL) {
		freemsg(chain->uc_buf);
		chain->uc_buf = NULL;
	}
}

/*
 * USB pipe management
 */

static int
ure_open_pipes(ure_softc_t *sc)
{
	usb_pipe_policy_t policy;
	usb_flags_t flags;
	int ret;

	/* Bulk IN: URE_RX_LIST_CNT concurrent transfers + headroom */
	bzero(&policy, sizeof (policy));
	policy.pp_max_async_reqs = URE_RX_LIST_CNT + 2;

	flags = USB_FLAGS_SLEEP;

#if defined(USB_FLAGS_START_NEXT_FIRST)
	flags |= USB_FLAGS_START_NEXT_FIRST;
#endif

	ret = usb_pipe_xopen(sc->ure_dip,
	    &sc->ure_bulkin_xdesc, &policy,
	    flags, &sc->ure_bulkin_pipe);
	if (ret != USB_SUCCESS) {
		dev_err(sc->ure_dip, CE_WARN,
		    "failed to open bulk IN pipe: %d", ret);
		return (DDI_FAILURE);
	}

	/* Bulk OUT: URE_TX_MAX concurrent transfers + headroom */
	bzero(&policy, sizeof (policy));
	policy.pp_max_async_reqs = URE_TX_MAX + 4;

	ret = usb_pipe_xopen(sc->ure_dip,
	    &sc->ure_bulkout_xdesc, &policy,
	    flags, &sc->ure_bulkout_pipe);
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

/*
 * USB event callbacks
 */

static int
ure_disconnect_cb(dev_info_t *dip)
{
	ure_softc_t *sc;
	int instance = ddi_get_instance(dip);

	sc = ddi_get_soft_state(ure_statep, instance);
	if (sc == NULL) {
		return (DDI_SUCCESS);
	}

	mutex_enter(&sc->ure_lock);
	sc->ure_was_running = sc->ure_running;	/* save BEFORE clearing */
	sc->ure_gone = B_TRUE;
	sc->ure_running = B_FALSE;
	sc->ure_flags &= ~URE_FLAG_LINK;
	sc->ure_link_state = LINK_STATE_DOWN;
	mutex_exit(&sc->ure_lock);

	/* Discard any pending coalescing buffer */
	ure_txc_discard(sc);

	if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
		mac_link_update(sc->ure_mh, LINK_STATE_DOWN);
	}

	return (DDI_SUCCESS);
}

static int
ure_reconnect_cb(dev_info_t *dip)
{
	ure_softc_t *sc;
	int instance = ddi_get_instance(dip);
	boolean_t was_running;

	sc = ddi_get_soft_state(ure_statep, instance);
	if (sc == NULL) {
		return (DDI_SUCCESS);
	}

	if (usb_check_same_device(dip, sc->ure_lh, USB_LOG_L0, -1,
	    USB_CHK_ALL, NULL) != USB_SUCCESS) {
		mutex_enter(&sc->ure_lock);
		sc->ure_gone = B_TRUE;
		sc->ure_running = B_FALSE;
		sc->ure_flags &= ~URE_FLAG_LINK;
		sc->ure_link_state = LINK_STATE_DOWN;
		mutex_exit(&sc->ure_lock);

		if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
			mac_link_update(sc->ure_mh, LINK_STATE_DOWN);
		}

		return (DDI_SUCCESS);
	}

	mutex_enter(&sc->ure_lock);
	sc->ure_gone = B_FALSE;
	was_running = sc->ure_was_running;
	sc->ure_was_running = B_FALSE;
	mutex_exit(&sc->ure_lock);

	/* Refresh bulk pipe handles after bus reset */
	ure_close_pipes(sc);
	if (ure_open_pipes(sc) != DDI_SUCCESS) {
		dev_err(sc->ure_dip, CE_WARN,
		    "pipe open failed on reconnect");
		goto reconn_fail;
	}

	/* Re-initialise chip */
	if (ure_chip_init(sc) != 0) {
		dev_err(sc->ure_dip, CE_WARN,
		    "chip init failed on reconnect");
		mutex_enter(&sc->ure_lock);
		sc->ure_gone = B_TRUE;
		mutex_exit(&sc->ure_lock);
		return (DDI_SUCCESS);
	}

	if (was_running) {
		int error;

		/* Clear PHY power-down before NIC reset */
		if (ure_phy_powerup(sc) != 0) {
			dev_err(sc->ure_dip, CE_WARN,
			    "PHY powerup failed on reconnect");
			goto reconn_fail;
		}

		/* NIC reset - USB I/O, no lock held */
		if (sc->ure_flags & URE_FLAG_8152) {
			error = ure_rtl8152_nic_reset(sc);
		} else {
			error = ure_rtl8153_nic_reset(sc);
		}
		if (error != 0) {
			dev_err(sc->ure_dip, CE_WARN,
			    "NIC reset failed on reconnect");
			goto reconn_fail;
		}

		if (ure_ifmedia_init(sc) != 0) {
			dev_err(sc->ure_dip, CE_WARN,
			    "media init failed on reconnect");
			goto reconn_fail;
		}
		if (ure_set_rx_filter(sc) != 0) {
			dev_err(sc->ure_dip, CE_WARN,
			    "RX filter setup failed on reconnect");
			goto reconn_fail;
		}

		mutex_enter(&sc->ure_lock);
		if (sc->ure_gone) {
			mutex_exit(&sc->ure_lock);
			goto reconn_fail;
		}
		sc->ure_running = B_TRUE;
		ure_rx_start(sc);
		mutex_exit(&sc->ure_lock);

		mac_link_update(sc->ure_mh, LINK_STATE_UNKNOWN);
	}

	return (DDI_SUCCESS);

reconn_fail:
	(void) ure_phy_powerdown(sc);
	mutex_enter(&sc->ure_lock);
	sc->ure_gone = B_TRUE;
	mutex_exit(&sc->ure_lock);
	if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
		mac_link_update(sc->ure_mh, LINK_STATE_DOWN);
	}
	return (DDI_SUCCESS);
}

/*
 * Chip identification
 */

static int
ure_chip_init(ure_softc_t *sc)
{
	uint16_t ver;
	int err;

	sc->ure_chip = 0;
	sc->ure_flags = 0;
	sc->ure_ocp_base = 0;

	sc->ure_phy_read = ure_ocp_reg_read;
	sc->ure_phy_write = ure_ocp_reg_write;

	if ((err = ure_read_2(sc, URE_PLA_TCR1, URE_MCU_TYPE_PLA,
	    &ver)) != USB_SUCCESS) {
		return (err);
	}
	ver &= URE_VERSION_MASK;

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
		sc->ure_rxbufsz = URE_8156_RX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8156 (0x%04x)\n", ver);
		break;
	case 0x7030:
		sc->ure_flags = URE_FLAG_8156;
		sc->ure_txbufsz = URE_8156_TX_BUFSZ;
		sc->ure_rxbufsz = URE_8156_RX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8156 (0x%04x)\n", ver);
		break;
	case 0x7400:
		sc->ure_flags = URE_FLAG_8156B;
		sc->ure_chip |= URE_CHIP_VER_7400;
		sc->ure_txbufsz = URE_8156_TX_BUFSZ;
		sc->ure_rxbufsz = URE_8156_RX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8156B (0x%04x)\n", ver);
		break;
	case 0x7410:
		sc->ure_flags = URE_FLAG_8156B;
		sc->ure_txbufsz = URE_8156_TX_BUFSZ;
		sc->ure_rxbufsz = URE_8156_RX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8156B (0x%04x)\n", ver);
		break;
	case 0x7420:
		sc->ure_flags = URE_FLAG_8156B;
		sc->ure_chip |= URE_CHIP_VER_7420;
		sc->ure_txbufsz = URE_8156_TX_BUFSZ;
		sc->ure_rxbufsz = URE_8156_RX_BUFSZ;
		dev_err(sc->ure_dip, CE_CONT,
		    "?RTL8153D (0x%04x)\n", ver);
		break;
	default:
		/*
		 * Unknown chip version - fall through to RTL8153 init.
		 * No ure_flags chip bit is set, so the init-variant
		 * switch below will take the default (RTL8153) path.
		 */
		sc->ure_rxbufsz = URE_8153_RX_BUFSZ;
		sc->ure_txbufsz = URE_TX_BUFSZ;
		dev_err(sc->ure_dip, CE_WARN,
		    "unknown chip version 0x%04x", ver);
		break;
	}

	/*
	 * Cache the 10GBASE-T AN control register for chips that
	 * support 2.5G/5G.  This tells us what speeds the firmware
	 * has enabled for autonegotiation advertisement.
	 */
	if (ure_has_2500fdx(sc)) {
		if ((err = ure_ocp_reg_read(sc, URE_OCP_10GBT_CTRL,
		    &sc->ure_10gbt_ctrl)) != USB_SUCCESS) {
			return (err);
		}
	}

	/* Run chip-variant-specific init */
	switch (sc->ure_flags & URE_FLAG_CHIP_MASK) {
	case URE_FLAG_8152:
		if ((err = ure_rtl8152_init(sc)) != USB_SUCCESS) {
			return (err);
		}
		break;
	case URE_FLAG_8153B:
	case URE_FLAG_8156:
	case URE_FLAG_8156B:
		if ((err = ure_rtl8153b_init(sc)) != USB_SUCCESS) {
			return (err);
		}
		break;
	case URE_FLAG_8157:
		if ((err = ure_rtl8157_init(sc)) != USB_SUCCESS) {
			return (err);
		}
		break;
	default:
		/* RTL8153 base */
		if ((err = ure_rtl8153_init(sc)) != USB_SUCCESS) {
			return (err);
		}
		break;
	}

	return (USB_SUCCESS);
}

static void
ure_chip_uninit(ure_softc_t *sc __unused)
{
	/*
	 * Chip-specific detach-time teardown belongs here.  The current
	 * chip initialisation paths only program hardware state and do not
	 * allocate chip-specific software resources, so there is nothing to
	 * undo at present.
	 */
}

/*
 * Attach / Detach
 */

static void
ure_cleanup(ure_softc_t *sc)
{
	ure_link_timer_stop(sc);

	if (sc->ure_attach_seq & URE_ATTACH_USB_EVT) {
		usb_unregister_event_cbs(sc->ure_dip, &ure_events);
		sc->ure_attach_seq &= ~URE_ATTACH_USB_EVT;
	}

	if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
		(void) mac_unregister(sc->ure_mh);
		sc->ure_attach_seq &= ~URE_ATTACH_MAC_REG;
	}

	if (sc->ure_attach_seq & URE_ATTACH_CHIP_INIT) {
		ure_chip_uninit(sc);
		sc->ure_attach_seq &= ~URE_ATTACH_CHIP_INIT;
	}

	ure_close_pipes(sc);

	/* Tear down TX cache (before mutexes) */
	if (sc->ure_attach_seq & URE_ATTACH_TX_CACHE) {
		kmem_cache_destroy(sc->ure_tx_cache);
		sc->ure_tx_cache = NULL;
		sc->ure_attach_seq &= ~URE_ATTACH_TX_CACHE;
	}

	/* Free multicast address list */
	{
		ure_mcast_entry_t *me;
		while ((me = list_remove_head(&sc->ure_mcast_list)) != NULL) {
			kmem_free(me, sizeof (*me));
		}
		list_destroy(&sc->ure_mcast_list);
	}

	if (sc->ure_attach_seq & URE_ATTACH_MUTEX) {
		mutex_destroy(&sc->ure_txc_lock);
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

	if (sc->ure_attach_seq & URE_ATTACH_LOG) {
		usb_free_log_hdl(sc->ure_lh);
		sc->ure_lh = NULL;
		sc->ure_attach_seq &= ~URE_ATTACH_LOG;
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
	boolean_t was_running;
	int ret;
	int err;

	switch (cmd) {
	case DDI_ATTACH:
		break;
	case DDI_RESUME:
		sc = ddi_get_soft_state(ure_statep, instance);
		if (sc == NULL) {
			return (DDI_FAILURE);
		}

		/*
		 * Treat resume like reconnect.  Before allowing register
		 * access again, make sure CPR has not resumed with a
		 * different device on the port.
		 */
		if (usb_check_same_device(dip, sc->ure_lh, USB_LOG_L0, -1,
		    USB_CHK_ALL, NULL) != USB_SUCCESS) {
			mutex_enter(&sc->ure_lock);
			sc->ure_gone = B_TRUE;
			sc->ure_running = B_FALSE;
			sc->ure_flags &= ~URE_FLAG_LINK;
			sc->ure_link_state = LINK_STATE_DOWN;
			mutex_exit(&sc->ure_lock);

			if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
				mac_link_update(sc->ure_mh, LINK_STATE_DOWN);
			}

			return (DDI_SUCCESS);
		}

		mutex_enter(&sc->ure_lock);
		sc->ure_gone = B_FALSE;
		was_running = sc->ure_was_running;
		sc->ure_was_running = B_FALSE;
		mutex_exit(&sc->ure_lock);

		/* Refresh bulk pipe handles after suspend/resume cycle */
		ure_close_pipes(sc);
		if (ure_open_pipes(sc) != DDI_SUCCESS) {
			dev_err(dip, CE_WARN,
			    "pipe open failed on resume");
			goto resume_fail;
		}

		/* Reinitialise chip variant registers */
		if (ure_chip_init(sc) != 0) {
			dev_err(dip, CE_WARN,
			    "chip init failed on resume");
			goto resume_fail;
		}

		if (was_running) {
			int error;

			/* Clear PHY power-down before NIC reset */
			if (ure_phy_powerup(sc) != 0) {
				dev_err(dip, CE_WARN,
				    "PHY powerup failed on resume");
				goto resume_fail;
			}

			/* NIC reset - USB I/O, no lock held */
			if (sc->ure_flags & URE_FLAG_8152) {
				error = ure_rtl8152_nic_reset(sc);
			} else {
				error = ure_rtl8153_nic_reset(sc);
			}
			if (error != 0) {
				dev_err(dip, CE_WARN,
				    "NIC reset failed on resume");
				goto resume_fail;
			}

			if (ure_ifmedia_init(sc) != 0) {
				dev_err(dip, CE_WARN,
				    "media init failed on resume");
				goto resume_fail;
			}
			if (ure_set_rx_filter(sc) != 0) {
				dev_err(dip, CE_WARN,
				    "RX filter setup failed on resume");
				goto resume_fail;
			}

			mutex_enter(&sc->ure_lock);
			if (sc->ure_gone) {
				mutex_exit(&sc->ure_lock);
				goto resume_fail;
			}
			sc->ure_running = B_TRUE;
			sc->ure_link_state = LINK_STATE_UNKNOWN;
			sc->ure_link_speed = 0;
			sc->ure_link_duplex = LINK_DUPLEX_UNKNOWN;
			sc->ure_10gbt_stat = 0;
			ure_rx_start(sc);
			mutex_exit(&sc->ure_lock);

			mac_link_update(sc->ure_mh,
			    LINK_STATE_UNKNOWN);
		}

		ure_link_timer_start(sc);

		return (DDI_SUCCESS);

	resume_fail:
		(void) ure_phy_powerdown(sc);
		mutex_enter(&sc->ure_lock);
		sc->ure_gone = B_TRUE;
		mutex_exit(&sc->ure_lock);
		if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
			mac_link_update(sc->ure_mh, LINK_STATE_DOWN);
		}
		return (DDI_FAILURE);
	default:
		return (DDI_FAILURE);
	}

	if (ddi_soft_state_zalloc(ure_statep, instance) !=
	    DDI_SUCCESS) {
		return (DDI_FAILURE);
	}

	sc = ddi_get_soft_state(ure_statep, instance);
	sc->ure_dip = dip;
	sc->ure_instance = instance;
	sc->ure_attach_seq = 0;
	sc->ure_gone = B_FALSE;
	sc->ure_running = B_FALSE;
	sc->ure_link_state = LINK_STATE_UNKNOWN;
	sc->ure_ocp_base = 0;
	sc->ure_lh = usb_alloc_log_hdl(dip, "ure", &ure_errlevel,
	    &ure_errmask, &ure_instance_debug, 0);
	if (sc->ure_lh != NULL) {
		sc->ure_attach_seq |= URE_ATTACH_LOG;
	}

	/* Read driver.conf tuneables */
	sc->ure_hcksum_en = ddi_prop_get_int(DDI_DEV_T_ANY, dip,
	    DDI_PROP_DONTPASS, "checksum", 1) != 0;
	sc->ure_lso_en = ddi_prop_get_int(DDI_DEV_T_ANY, dip,
	    DDI_PROP_DONTPASS, "lso", 1) != 0;
	list_create(&sc->ure_mcast_list, sizeof (ure_mcast_entry_t),
	    offsetof(ure_mcast_entry_t, node));

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
	ret = usb_ep_xdescr_fill(USB_EP_XDESCR_CURRENT_VERSION,
	    dip, ep_data, &sc->ure_bulkin_xdesc);
	if (ret != USB_SUCCESS) {
		dev_err(dip, CE_WARN,
		    "failed to fill bulk IN xdescr: %d", ret);
		goto fail;
	}

	ep_data = usb_lookup_ep_data(dip, sc->ure_dev_data,
	    0, 0, 0, USB_EP_ATTR_BULK, USB_EP_DIR_OUT);
	if (ep_data == NULL) {
		dev_err(dip, CE_WARN,
		    "no bulk OUT endpoint found");
		goto fail;
	}
	ret = usb_ep_xdescr_fill(USB_EP_XDESCR_CURRENT_VERSION,
	    dip, ep_data, &sc->ure_bulkout_xdesc);
	if (ret != USB_SUCCESS) {
		dev_err(dip, CE_WARN,
		    "failed to fill bulk OUT xdescr: %d", ret);
		goto fail;
	}

	/* Step 4: Initialise mutexes */
	mutex_init(&sc->ure_lock, NULL, MUTEX_DRIVER,
	    sc->ure_dev_data->dev_iblock_cookie);
	mutex_init(&sc->ure_tx_lock, NULL, MUTEX_DRIVER,
	    sc->ure_dev_data->dev_iblock_cookie);
	mutex_init(&sc->ure_txc_lock, NULL, MUTEX_DRIVER,
	    sc->ure_dev_data->dev_iblock_cookie);
	sc->ure_attach_seq |= URE_ATTACH_MUTEX;

	/* Step 5: Identify and initialise chip */
	if (ure_chip_init(sc) != 0) {
		dev_err(dip, CE_WARN, "chip initialisation failed");
		goto fail;
	}
	sc->ure_attach_seq |= URE_ATTACH_CHIP_INIT;

	/* Step 5a: Initialise TX state */
	sc->ure_tx_cnt = 0;

	/* Step 5b: Create TX chain kmem_cache (needs ure_txbufsz) */
	{
		char cache_name[64];
		(void) snprintf(cache_name, sizeof (cache_name),
		    "ure_tx_chain_%d", instance);
		sc->ure_tx_cache = kmem_cache_create(cache_name,
		    sizeof (ure_tx_chain_t), 0,
		    ure_tx_chain_construct, ure_tx_chain_destroy,
		    NULL, (void *)sc, NULL, 0);
	}
	sc->ure_attach_seq |= URE_ATTACH_TX_CACHE;

	/* Step 6: Read MAC address */
	bzero(eaddr, sizeof (eaddr));
	if (sc->ure_chip & (URE_CHIP_VER_4C00 |
	    URE_CHIP_VER_4C10)) {
		err = ure_read_mem(sc, URE_PLA_IDR, URE_MCU_TYPE_PLA,
		    eaddr, sizeof (eaddr));
	} else {
		err = ure_read_mem(sc, URE_PLA_BACKUP,
		    URE_MCU_TYPE_PLA, eaddr, sizeof (eaddr));
	}
	if (err != USB_SUCCESS) {
		dev_err(dip, CE_WARN, "failed to read MAC address");
		goto fail;
	}

	bcopy(eaddr, sc->ure_dev_addr, ETHERADDRL);

	dev_err(dip, CE_CONT,
	    "?MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	    sc->ure_dev_addr[0], sc->ure_dev_addr[1],
	    sc->ure_dev_addr[2], sc->ure_dev_addr[3],
	    sc->ure_dev_addr[4], sc->ure_dev_addr[5]);

	/* Step 7: Open bulk pipes */
	if (ure_open_pipes(sc) != DDI_SUCCESS) {
		goto fail;
	}

	/* Step 8: Register with MAC framework */
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
	sc->ure_mtu = ETHERMTU;

	ret = mac_register(macp, &sc->ure_mh);
	mac_free(macp);
	macp = NULL;

	if (ret != 0) {
		dev_err(dip, CE_WARN,
		    "mac_register failed: %d", ret);
		goto fail;
	}
	sc->ure_attach_seq |= URE_ATTACH_MAC_REG;

	/* Step 9: Register USB event callbacks */
	ret = usb_register_event_cbs(dip, &ure_events, 0);
	if (ret != USB_SUCCESS) {
		dev_err(dip, CE_WARN,
		    "usb_register_event_cbs failed: %d", ret);
		goto fail;
	}
	sc->ure_attach_seq |= URE_ATTACH_USB_EVT;

	/*
	 * Report an initial link-down so that the first link-up from
	 * the polling timer is a genuine down-to-up transition.  Without
	 * this, IPv6 DAD may not trigger if the PHY was already up
	 * (e.g. across a reboot without power cycle).
	 */
	sc->ure_link_state = LINK_STATE_DOWN;
	mac_link_update(sc->ure_mh, LINK_STATE_DOWN);

	/* Step 10: Start link polling timer (1 second) */
	ure_link_timer_start(sc);

	ddi_report_dev(dip);

	return (DDI_SUCCESS);

fail:
	if (macp != NULL) {
		mac_free(macp);
	}
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
	if (sc == NULL) {
		return (DDI_FAILURE);
	}

	switch (cmd) {
	case DDI_DETACH:
		break;
	case DDI_SUSPEND:
		/*
		 * Stop data paths first, stop link polling, then
		 * quiesce the hardware and mark the device gone.
		 * ure_running = B_FALSE is sufficient to stop new
		 * RX/TX activity and make callbacks bail out.
		 * ure_gone must remain B_FALSE until after
		 * ure_reset() so that the register writes inside
		 * ure_reset() can reach the device via ure_ctl(),
		 * which returns USB_FAILURE immediately when
		 * ure_gone is true.
		 */
		mutex_enter(&sc->ure_lock);
		sc->ure_was_running = sc->ure_running;
		sc->ure_running = B_FALSE;
		sc->ure_flags &= ~URE_FLAG_LINK;
		mutex_exit(&sc->ure_lock);

		ure_link_timer_stop(sc);

		mutex_enter(&sc->ure_lock);
		sc->ure_link_state = LINK_STATE_DOWN;
		sc->ure_link_speed = 0;
		sc->ure_link_duplex = LINK_DUPLEX_UNKNOWN;
		sc->ure_10gbt_stat = 0;
		mutex_exit(&sc->ure_lock);

		if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
			mac_link_update(sc->ure_mh, LINK_STATE_DOWN);
		}

		/* Discard any pending coalescing buffer */
		ure_txc_discard(sc);

		/*
		 * Reset the chip to quiesce DMA, then drain any
		 * in-flight USB transfers.  The callbacks will see
		 * !ure_running under ure_lock and skip
		 * mac_rx/mac_tx_update.
		 */
		(void) ure_reset(sc);

		usb_pipe_reset(sc->ure_dip, sc->ure_bulkin_pipe,
		    USB_FLAGS_SLEEP, NULL, 0);
		usb_pipe_reset(sc->ure_dip, sc->ure_bulkout_pipe,
		    USB_FLAGS_SLEEP, NULL, 0);

		mutex_enter(&sc->ure_lock);
		sc->ure_gone = B_TRUE;
		mutex_exit(&sc->ure_lock);

		return (DDI_SUCCESS);
	default:
		return (DDI_FAILURE);
	}

	/*
	 * Stop the link timer before mac_disable so the timer
	 * callback cannot call mac_link_update on a disabled MAC.
	 */
	ure_link_timer_stop(sc);

	/* Attempt MAC unregister first, may fail if busy */
	if (sc->ure_attach_seq & URE_ATTACH_MAC_REG) {
		if (mac_disable(sc->ure_mh) != 0) {
			ure_link_timer_start(sc);
			return (DDI_FAILURE);
		}
	}

	ure_cleanup(sc);
	ddi_soft_state_free(ure_statep, instance);

	return (DDI_SUCCESS);
}
