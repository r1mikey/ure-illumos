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

## TODO / Future Work

1. **VLAN tag offload** - hardware insert/strip via RX/TX packet header VLAN fields
2. **Proper MII integration** - for 8152/8153/8153B, integrate with illumos MII framework instead of direct PHY polling

## References

- OpenBSD `if_ure.c` v1.37 (Kevin Lo, Jonathon Fletcher) - primary reference
- FreeBSD `if_ure.c` (Kevin Lo) - TX checksum L4 offset, spurious link-down fix
- illumos `usbecm.c` - MAC/USBA integration patterns
