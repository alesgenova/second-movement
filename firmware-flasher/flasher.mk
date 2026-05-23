# firmware-flasher build configuration. The project Makefile includes this
# BEFORE gossamer's rules.mk (whose $(eval)-generated recipes freeze CFLAGS at
# parse time, so sources/flags/linker-script changes must land first), and
# flasher-rules.mk AFTER it.

# Repair header dependency tracking. NOT flasher-specific -- this fixes the
# whole build (hoist into the Makefile if adopted project-wide) -- but the
# flasher relies on it: edits to firmware_flasher_core.h must rebuild its
# TUs. gossamer's dep flags reference $(*F) and $(@F), but its per-source
# rules are generated through $(eval), which expands them at parse time with
# no target in scope: every TU writes the same bogus ./build/.d (target
# "./build/.o"), nothing ever -includes it, and so editing a header --
# movement_config.h included -- never rebuilds anything without a make clean.
# Strip the broken tokens and use plain -MD, which derives build/<obj>.d from
# -o: exactly the names rules.mk already -includes via DEPFILES.
CFLAGS := $(filter-out -MD -MP -MT -MF $(BUILD)/.o $(BUILD)/.d,$(CFLAGS))
CFLAGS += -MD -MP

# Patch-decoder backend selection. UltraPatch (the `ultrapatch` submodule;
# encoder CLI and this decoder MUST build from the same headers -- the
# submodule pin is that single wire-contract source) is the default; build
# with FIRMWARE_FLASHER_ULTRAPATCH=0 for the legacy detools decoder. The
# decoder TU joins the flasher's load-on-demand RAM overlay -- it runs while
# NVMCTRL erases the flash it patches.
#
# Toggling the value between builds needs NO make clean: the face is rebuilt
# whenever the selected backend differs from the one recorded in
# $(BUILD)/patch-backend.flag (see flasher-rules.mk).
FIRMWARE_FLASHER_ULTRAPATCH ?= 1
PATCH_BACKEND := $(if $(filter-out 0,$(FIRMWARE_FLASHER_ULTRAPATCH)),ultrapatch,detools)
INCLUDES += -I./firmware-flasher
ifeq ($(PATCH_BACKEND),ultrapatch)
ULTRAPATCH_PATH ?= ./ultrapatch
CFLAGS += -DFIRMWARE_FLASHER_ULTRAPATCH
INCLUDES += -I$(ULTRAPATCH_PATH)/src
FLASHER_BACKEND_TU = firmware_flasher_ultrapatch
else
FLASHER_BACKEND_TU = firmware_flasher_detools
endif
SRCS += ./firmware-flasher/firmware_flasher_core.c
SRCS += ./firmware-flasher/$(FLASHER_BACKEND_TU).c

# The RAM-resident flasher TUs: the backend-agnostic core and exactly one
# decoder backend (selected above). Each has every allocated section renamed
# to .flovl.* after compilation, which routes it into the load-on-demand
# overlay placed by flasher-overlay.ld instead of the boot-resident .data
# image -- see the stamp rules in flasher-rules.mk and
# firmware_flasher_core.h for the overlay story.
FLASHER_RAM_TUS = firmware_flasher_core $(FLASHER_BACKEND_TU)
ifndef EMSCRIPTEN
# The RAM TUs' byte-copy loops must not become memcpy/memset calls (flash-
# resident, fetched mid-erase -> crash; the audit in flasher-rules.mk would
# fail the build). Whether -Os converts them is compiler-version-dependent
# (GCC 12 does, GCC 15 doesn't), so disable the loop-to-libcall pass outright.
# Global because gossamer freezes CFLAGS per-recipe at $(eval) time (see the
# note at the end of this file); the cost elsewhere is negligible at -Os.
CFLAGS += -fno-tree-loop-distribute-patterns
# Overlay placement (hardware builds only): gossamer's LDFLAGS expands
# $(LDSCRIPT) lazily, so point it -- via path traversal, gossamer prepends
# its chips/<chip>/linker/ directory -- at a generated wrapper that INCLUDEs
# the real chip script and appends flasher-overlay.ld's SECTIONS. No gossamer
# changes, and the fragment runs after the chip script's `end = .` so the
# heap base is unaffected. (The wrapper-generation rule is in
# flasher-rules.mk.)
FLASHER_CHIP_LD := $(GOSSAMER_PATH)/chips/$(CHIP)/linker/$(LDSCRIPT).ld
LDSCRIPT := ../../../../$(BUILD)/flasher-main
endif
# (Per-object CFLAGS can't work here: gossamer's generated recipes expand
# $(CFLAGS) at $(eval) time, freezing the flags. Instead the stamp rules in
# flasher-rules.mk FAIL the build if a RAM TU references any flash-resident
# helper -- jump tables, division, memcpy/memset/memmove -- which enforces
# the same property and also catches every other libc/libgcc escape.)
