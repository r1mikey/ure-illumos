# illumos ure Driver - Realtek RTL8152/8153/8156/8157 USB Ethernet

## Files

| File | Description |
|------|-------------|
| `urereg.h` | Register definitions (PLA, USB, OCP, SRAM, RX/TX packet headers) |
| `ure.h` | Driver softstate (`ure_softc_t`), flags, attach sequence tracking, macros |
| `ure.c` | Main driver source: all chip variants, RX/TX, MAC callbacks, attach/detach |
| `Makefile` | Out-of-tree build (standalone, no illumos source tree required) |

## Architecture

- **Standalone driver** - does NOT use the `usbgem` framework
- Registers directly with MAC (GLDv3) and USBA
- All chip register access via USB vendor control transfers

## Supported Chips

| Chip | Version | Speed | Flags |
|------|---------|-------|-------|
| RTL8152 | 0x4c00, 0x4c10 | 10/100 | `URE_FLAG_8152` |
| RTL8153 | 0x5c00-0x5c30 | 10/100/1000 | (default) |
| RTL8153B | 0x6000, 0x6010 | 10/100/1000 | `URE_FLAG_8153B` |
| RTL8156 | 0x7020, 0x7030 | 10/100/1000/2500 | `URE_FLAG_8156` |
| RTL8156B | 0x7410, 0x7420 | 10/100/1000/2500 | `URE_FLAG_8156B` |
| RTL8157 | 0x1030 | 10/100/1000/2500/5000 | `URE_FLAG_8157` |

## Key Features

- **RX aggregation** - multiple packets per USB bulk IN, parsed from `ure_rxpkt`/`ure_rxpkt_v2` headers
- **TX aggregation** - multiple packets packed per USB bulk OUT with per-packet TX headers
- **IPv4/TCP/UDP hardware checksum offload** (RX and TX)
- **TCP segmentation offload** (TSO/LSO) for RTL8153 and RTL8157
- **All chip init sequences** - RTL8152, RTL8153, RTL8153B, RTL8156/B, RTL8157
- **FreeBSD spurious link-down workaround** - double-read BMSR (PR 252165)
- **Robust attach/detach** - `ure_attach_seq` flags track every acquired resource; `ure_cleanup()` reverses in order
- **USB disconnect/reconnect** - `usb_register_event_cbs` with `ure_gone` flag
- **60+ device IDs** - Realtek, Lenovo, Microsoft, Samsung, TP-Link, D-Link, etc.

## Building

```bash
make            # Build the ure kernel module
make clean      # Remove build artifacts
```

The default target architecture is `amd64` (i86pc/OmniOS).  Override with:

```bash
make KARCH=aarch64
```

## Installing and Testing

### First-time install

```bash
# Build
make

# Copy driver module and conf file into place
make install

# Register the driver with the system.
# For quick testing with just your adapter (Realtek VID 0x0bda, PID 0x8153):
add_drv -i '"usbbda,8153"' ure

# Or register with all known device IDs:
add_drv \
    -i '"usb0b95,8156"' \
    -i '"usb050d,0128"' \
    -i '"usb050d,0129"' \
    -i '"usb13b1,0041"' \
    -i '"usb04b4,3610"' \
    -i '"usb2001,b301"' \
    -i '"usb2001,b328"' \
    -i '"usb056e,4010"' \
    -i '"usb056e,401a"' \
    -i '"usb17ef,304f"' \
    -i '"usb17ef,3054"' \
    -i '"usb17ef,3057"' \
    -i '"usb17ef,3062"' \
    -i '"usb17ef,3069"' \
    -i '"usb17ef,7205"' \
    -i '"usb17ef,720a"' \
    -i '"usb17ef,720c"' \
    -i '"usb17ef,3082"' \
    -i '"usb17ef,3098"' \
    -i '"usb17ef,a359"' \
    -i '"usb17ef,a387"' \
    -i '"usb17ef,3049"' \
    -i '"usb043e,3068"' \
    -i '"usb043e,3091"' \
    -i '"usb045e,07ab"' \
    -i '"usb045e,07c6"' \
    -i '"usb045e,0927"' \
    -i '"usb045e,0c30"' \
    -i '"usb0955,09ff"' \
    -i '"usb2b04,0132"' \
    -i '"usb2b04,013b"' \
    -i '"usbbda,8050"' \
    -i '"usbbda,8152"' \
    -i '"usbbda,8153"' \
    -i '"usbbda,8156"' \
    -i '"usbbda,8157"' \
    -i '"usb04e8,a101"' \
    -i '"usb0930,0a13"' \
    -i '"usb2357,0601"' \
    -i '"usb2357,0602"' \
    -i '"usb2357,0603"' \
    -i '"usb20f4,e05a"' \
    -i '"usb0fce,7a03"' \
    -i '"usb14cd,8158"' \
    -i '"usb2717,ff40"' \
    ure
```

After `add_drv`, plug in the adapter (or replug it).  The driver should
attach and a `ure0` network interface will appear in `dladm show-phys`.

### Updating after a code change

```bash
# Remove the running driver (unplug device first if possible)
rem_drv ure
modunload -i 0     # try to unload; may need to repeat or specify mod id

# Rebuild and reinstall
make clean && make
make install

# Re-register
add_drv -i '"usbbda,8153"' ure

# Plug the device back in
```

### Quick test cycle (without rem_drv)

If the device is unplugged, you can often just overwrite the module and
replug:

```bash
make && cp ure /kernel/drv/aarch64/ure
# Plug device in, new module loads
```

### Removing the driver completely

```bash
rem_drv ure
make uninstall
```

### Useful diagnostic commands

```bash
# Check if driver attached
dladm show-phys
dladm show-link

# Check device tree
prtconf -D | grep ure

# Check USB device enumeration
prtconf -v | grep -A5 'usbbda,8153'

# Watch for driver messages
tail -f /var/adm/messages

# Module info
modinfo | grep ure
```

## Design Notes

The driver manages PHY state directly rather than using the illumos MII
framework.  The MII framework assumes standard MDIO bus access and tops
out at 1000BASE-T.  These chips use Realtek's OCP register interface over
USB bulk transfers (the RTL8157 adds a separate TGPHY path), and the
RTL8156/8156B/8157 operate at 2.5G/5G which the MII framework has no
support for.  MII integration would only cover the 8152/8153 and would
not add meaningful functionality beyond what the driver already provides.

### TX checksum offload caveat

For RTL8157, non-TSO TX checksum offload is rejected when the transport
header starts beyond byte 0x3ff from the start of the Ethernet frame.
This follows the smaller RTL8157 checksum-offset field used by Linux.
Ordinary IPv4/IPv6 TCP or UDP packets are far below this limit.  It can
only be reached when MAC requests partial checksum offload for a packet
with unusually large L2 plus L3 headers, for example a packet with many
or very large encapsulation or extension headers.  The current TX path
has no per-packet software checksum fallback, so such a packet is dropped
and counted as an output error rather than sent with an invalid hardware
descriptor.

TSO uses a separate transport-offset field and is not changed by this
limit.

### Quiesce

The driver does not provide a quiesce callback.  All chip register access
goes through USB control transfers, and quiesce may run after other CPUs
have been stopped and at a raised interrupt priority.  In that context a
USB request that waits for host-controller progress is not safe.  The
normal stop, detach, suspend and disconnect paths already reset or stop
the device in contexts where sleeping USB operations are valid.  A useful
quiesce implementation would need a USB path that can complete without
blocking on normal USBA callbacks, so the driver declares that no separate
quiesce action is needed instead of trying a best-effort register write.

Hardware VLAN tag insert/strip is not used.  The chips support out-of-band
VLAN tagging via the RX/TX packet header VLAN fields (FreeBSD uses this
with its M_VLANTAG mbuf mechanism), but the illumos MAC framework has no
out-of-band VLAN tag interface.  Tags must remain in-band in the Ethernet
frame.  Enabling hardware strip on RX would require re-inserting the tag
before passing to mac_rx(); enabling hardware insert on TX would require
parsing and removing the in-band tag to populate the descriptor.  Both
add complexity for no benefit.  Hardware stripping is explicitly disabled
in ure_rxvlan().

## References

- OpenBSD `if_ure.c` v1.37 (Kevin Lo, Jonathon Fletcher) - primary reference
- FreeBSD `if_ure.c` (Kevin Lo) - TX checksum L4 offset, spurious link-down fix
- illumos `usbecm.c` - MAC/USBA integration patterns
