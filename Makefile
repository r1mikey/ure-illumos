#
# Out-of-tree Makefile for the ure (Realtek RTL8152/8153/8156/8157)
# USB Ethernet driver.
#
# Usage:
#   make			Build the driver module
#   make clean			Remove build artifacts
#   make install		Copy module and conf to /kernel/drv/$(KARCH)/
#   make uninstall		Remove module and conf from /kernel/drv/$(KARCH)/
#
# The driver is built for aarch64 by default.  Override KARCH for
# other architectures (e.g., KARCH=amd64).
#
# Prerequisites:
#   - illumos kernel headers in /usr/include (from system-header package)
#   - gcc (system compiler)
#

KARCH		= amd64
MODULE		= ure

CC		= gcc
LD		= ld

#
# Compiler flags for a kernel module.
#
# -D_KERNEL:		building kernel code
# -ffreestanding:	no hosted-environment assumptions
# -mcmodel=large:	required for aarch64 kernel modules
# -mgeneral-regs-only:	no FPU in kernel context
# -fno-strict-aliasing:	illumos convention
# -fno-unit-at-a-time:	preserve function ordering
# -fno-common:		no tentative definitions
# -std=gnu17:		illumos builds at C17
#
CFLAGS_COMMON	= -D_KERNEL -D_ELF64 \
		  -ffreestanding -fno-builtin \
		  -fno-strict-aliasing -fno-unit-at-a-time \
		  -fno-common -std=gnu17 \
		  -Wall -Wextra -Werror \
		  -Wno-unused-parameter \
		  -Wno-missing-field-initializers

CFLAGS_aarch64	= -mcmodel=large -mgeneral-regs-only
CFLAGS_amd64	= -mcmodel=kernel -mno-red-zone -mno-mmx -mno-sse

CFLAGS		= $(CFLAGS_COMMON) $(CFLAGS_$(KARCH))

INCLUDES	= -I. -I/build/arm64-gate/illumos-gate/usr/src/uts/common -I/build/arm64-gate/illumos-gate/usr/src/uts/common/sys

#
# Linker flags for a loadable kernel module.
#
# -N misc/mac:		MAC framework (GLDv3)
# -N drv/ip:		IP module
# -N misc/usba:	USB Architecture framework
#
LDFLAGS		= -ztype=kmod \
		  -N misc/mac -N drv/ip -N misc/usba

SRCS		= ure.c
OBJS		= $(SRCS:.c=.o)
MODFILE		= $(MODULE)

DESTDIR		= /kernel/drv/$(KARCH)

.KEEP_STATE:

all: $(MODFILE)

$(MODFILE): $(OBJS)
	$(LD) -o $@ $(LDFLAGS) $(OBJS)

%.o: %.c urereg.h ure.h
	$(CC) $(CFLAGS) $(INCLUDES) -c -o $@ $<

clean:
	rm -f $(OBJS) $(MODFILE)

install: $(MODFILE)
	@echo "Installing $(MODULE) driver..."
	cp $(MODFILE) $(DESTDIR)/$(MODULE)
	chmod 755 $(DESTDIR)/$(MODULE)
	@echo "Done. Run 'add_drv ure' or 'update_drv ure' to register."

uninstall:
	@echo "Removing $(MODULE) driver..."
	-rem_drv $(MODULE) 2>/dev/null
	rm -f $(DESTDIR)/$(MODULE)
	@echo "Done."

.PHONY: all clean install uninstall
