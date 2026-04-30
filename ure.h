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
 * Realtek RTL8152/8153/8153B/8156/8156B/8157 USB Ethernet
 * Driver softstate and internal definitions.
 */

#ifndef	_URE_H
#define	_URE_H

#include <sys/types.h>
#include <sys/ethernet.h>
#include <sys/list.h>
#include <sys/mac_provider.h>
#include <sys/usb/usba.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Chip type flags — set during attach based on PLA_TCR1 version.
 * Only one of the URE_FLAG_81xx bits should be set at a time.
 */
#define	URE_FLAG_LINK		0x0001
#define	URE_FLAG_8152		0x0010	/* RTL8152 */
#define	URE_FLAG_8153B		0x0020	/* RTL8153B */
#define	URE_FLAG_8156		0x0040	/* RTL8156 */
#define	URE_FLAG_8156B		0x0080	/* RTL8156B */
#define	URE_FLAG_8157		0x0100	/* RTL8157 */
#define	URE_FLAG_CHIP_MASK	0x01f0

/*
 * Chip sub-version bits — finer-grained identification.
 */
#define	URE_CHIP_VER_4C00	0x0001
#define	URE_CHIP_VER_4C10	0x0002
#define	URE_CHIP_VER_5C00	0x0004
#define	URE_CHIP_VER_5C10	0x0008
#define	URE_CHIP_VER_5C20	0x0010
#define	URE_CHIP_VER_5C30	0x0020
#define	URE_CHIP_VER_6010	0x0040
#define	URE_CHIP_VER_7420	0x0080

/*
 * Attach sequence tracking — each bit indicates a resource that
 * was successfully acquired and needs cleanup on detach.
 */
#define	URE_ATTACH_USB		0x0001	/* usb_client_attach done */
#define	URE_ATTACH_DEV_DATA	0x0002	/* usb_get_dev_data done */
#define	URE_ATTACH_MUTEX	0x0004	/* mutexes initialised */
#define	URE_ATTACH_CHIP_INIT	0x0008	/* chip init complete */
#define	URE_ATTACH_MAC_ALLOC	0x0010	/* mac_alloc done */
#define	URE_ATTACH_MAC_REG	0x0020	/* mac_register done */
#define	URE_ATTACH_USB_EVT	0x0040	/* usb event cbs registered */
#define	URE_ATTACH_LINK_TIMER	0x0080	/* link poll timer running */
#define	URE_ATTACH_TX_SER	0x0100	/* TX serializer initialised */
#define	URE_ATTACH_TX_CACHE	0x0200	/* TX chain kmem_cache created */

/*
 * Endpoint indices.
 */
#define	URE_ENDPT_RX		0
#define	URE_ENDPT_TX		1
#define	URE_ENDPT_MAX		2

/*
 * Maximum concurrent TX transfers.  Each transfer carries an
 * aggregation buffer with multiple packets, so even modest counts
 * saturate the USB pipe.
 */
#define	URE_TX_MAX		32

/*
 * Multicast address tracking entry.
 */
typedef struct ure_mcast_entry {
	list_node_t		node;
	uint8_t			addr[ETHERADDRL];
} ure_mcast_entry_t;

/* Forward declaration for ure_tx_chain_t */
struct ure_softc;

/*
 * TX chain — one per in-flight USB bulk OUT transfer.
 * The aggregation buffer (uc_buf) is pre-allocated by the
 * kmem_cache constructor and reused across submits.
 */
typedef struct ure_tx_chain {
	struct ure_softc	*uc_sc;
	mblk_t			*uc_buf;	/* pre-allocated agg buffer */
	uint32_t		uc_bufmax;	/* buffer capacity */
	int			uc_npkts;	/* packets packed this submit */
	uint64_t		uc_nbytes;	/* total bytes packed */
} ure_tx_chain_t;

/*
 * Per-instance softstate.
 *
 * Lock discipline:
 *   ure_lock    — protects softstate, link state, flags, RX path,
 *                 multicast hash, chip registers.  Never held during
 *                 bulk USB I/O.
 *   ure_tx_lock — protects ure_tx_cnt (TX in-flight counter) only.
 *                 Brief hold; never held simultaneously with ure_lock.
 */
typedef struct ure_softc {
	dev_info_t		*ure_dip;
	int			ure_instance;

	/* Attach tracking */
	uint32_t		ure_attach_seq;

	/* USB state */
	usb_client_dev_data_t	*ure_dev_data;
	usb_pipe_handle_t	ure_def_pipe;
	usb_pipe_handle_t	ure_bulkin_pipe;
	usb_pipe_handle_t	ure_bulkout_pipe;
	usb_ep_xdescr_t		ure_bulkin_xdesc;
	usb_ep_xdescr_t		ure_bulkout_xdesc;

	/* MAC framework */
	mac_handle_t		ure_mh;

	/* Synchronisation */
	kmutex_t		ure_lock;	/* general lock */
	kmutex_t		ure_tx_lock;	/* TX path lock */

	/* Link state */
	link_state_t		ure_link_state;
	uint64_t		ure_link_speed;
	link_duplex_t		ure_link_duplex;
	ddi_periodic_t		ure_link_timer;

	/* Chip identification */
	uint_t			ure_flags;
	uint_t			ure_chip;

	/* Feature flags (from driver.conf) */
	boolean_t		ure_hcksum_en;	/* HW checksum offload */

	/* PHY access function pointers (per chip variant) */
	uint16_t		(*ure_phy_read)(struct ure_softc *,
				    uint16_t);
	void			(*ure_phy_write)(struct ure_softc *,
				    uint16_t, uint16_t);

	/* MAC address */
	uint8_t			ure_dev_addr[ETHERADDRL];

	/* RX/TX buffer sizes (depend on chip variant and USB speed) */
	uint32_t		ure_rxbufsz;
	uint32_t		ure_txbufsz;
	uint32_t		ure_mtu;

	/* RX state */
	uint_t			ure_rx_cnt;	/* bulk-IN xfers in flight */

	/* TX state */
	kmem_cache_t		*ure_tx_cache;	/* slab cache for tx chains */
	usb_serialization_t	ure_tx_ser;	/* serializer for pipe submit */
	uint_t			ure_tx_cnt;	/* xfers in flight (tx_lock) */
	hrtime_t		ure_tx_watchdog; /* stuck TX detection (tx_lock) */

	/* Device online/offline */
	boolean_t		ure_running;	/* mc_start called */
	boolean_t		ure_gone;	/* USB disconnect */
	boolean_t		ure_was_running; /* saved for reconnect */

	/* Statistics */
	uint64_t		ure_stat_ierrors;
	uint64_t		ure_stat_oerrors;
	uint64_t		ure_stat_rbytes;
	uint64_t		ure_stat_ipackets;
	uint64_t		ure_stat_obytes;
	uint64_t		ure_stat_opackets;
	uint64_t		ure_stat_multircv;
	uint64_t		ure_stat_brdcstrcv;
	uint64_t		ure_stat_norcvbuf;

	/* Multicast hash (64-bit, CRC32-BE >> 26) */
	boolean_t		ure_promisc;
	uint32_t		ure_mcast_hash[2];
	list_t			ure_mcast_list;

} ure_softc_t;

/*
 * Convenience macros for read-modify-write register operations.
 */
#define	URE_SETBIT_1(sc, reg, idx, x)	\
	ure_write_1((sc), (reg), (idx), \
	    ure_read_1((sc), (reg), (idx)) | (x))
#define	URE_SETBIT_2(sc, reg, idx, x)	\
	ure_write_2((sc), (reg), (idx), \
	    ure_read_2((sc), (reg), (idx)) | (x))
#define	URE_SETBIT_4(sc, reg, idx, x)	\
	ure_write_4((sc), (reg), (idx), \
	    ure_read_4((sc), (reg), (idx)) | (x))

#define	URE_CLRBIT_1(sc, reg, idx, x)	\
	ure_write_1((sc), (reg), (idx), \
	    ure_read_1((sc), (reg), (idx)) & ~(x))
#define	URE_CLRBIT_2(sc, reg, idx, x)	\
	ure_write_2((sc), (reg), (idx), \
	    ure_read_2((sc), (reg), (idx)) & ~(x))
#define	URE_CLRBIT_4(sc, reg, idx, x)	\
	ure_write_4((sc), (reg), (idx), \
	    ure_read_4((sc), (reg), (idx)) & ~(x))


#ifdef __cplusplus
}
#endif

#endif /* _URE_H */
