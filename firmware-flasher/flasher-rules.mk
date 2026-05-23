# firmware-flasher build rules. The project Makefile includes this AFTER
# gossamer's rules.mk (so the first rule here cannot become make's default
# goal, and so these can use its `directory' order-only target); the matching
# configuration lives in flasher.mk, included before rules.mk.

# Patch-backend selection stamp: make tracks file times, not CFLAGS, so
# toggling FIRMWARE_FLASHER_ULTRAPATCH would otherwise leave a stale
# firmware_flasher_face.o (and a stale link) unless the user remembered to
# make clean. Record the selected backend in a flag file whose content -- and
# therefore mtime -- changes only when the selection changes, and make the
# face object depend on it: a toggle rebuilds exactly the face (+ relink),
# nothing else. The recipe runs every make (phony prerequisite) but rewrites
# the file only on a real change.
$(BUILD)/patch-backend.flag: FLASHER_FORCE | directory
	@if [ ! -f $@ ] || [ "$$(cat $@)" != "$(PATCH_BACKEND)" ]; then \
	    echo "$(PATCH_BACKEND)" > $@; \
	    echo "patch backend: $(PATCH_BACKEND)"; \
	fi
FLASHER_FORCE:
.PHONY: FLASHER_FORCE
$(BUILD)/firmware_flasher_face.o: $(BUILD)/patch-backend.flag
# The core classifies body frames by the compiled-in format pattern, so it is
# backend-sensitive too.
$(BUILD)/firmware_flasher_core.o: $(BUILD)/patch-backend.flag

# RAM-resident flasher TUs (FLASHER_RAM_TUS): rename every allocated section
# to .flovl.* AFTER gossamer's generated rule compiles each one, and make
# the link wait for the rename (the stamp is an extra .elf prerequisite).
# flasher-overlay.ld collects .flovl* into the load-on-demand overlay.
# Renaming twice is harmless: `*(.flovl*)` still matches a double prefix.
# Each stamp also AUDITS the object: a RAM TU referencing a flash-resident
# helper (libgcc jump tables / division, libc mem*) would be fetched mid-erase
# and crash the flasher, so it is a build error, not a code-review item.
# Hardware only: the simulator has no RAM overlay and no arm binutils
# ($(OBJCOPY) is empty under Emscripten).
ifndef EMSCRIPTEN
FLASHER_NM = $(OBJCOPY:objcopy=nm)
FLASHER_BANNED_SYMS = __gnu_thumb1_case|__aeabi_|__udivsi3|__divsi3|memcpy$$|memset$$|memmove$$
define flasher_ramfunc_rule
$$(BUILD)/$$(BIN).elf: $$(BUILD)/$(1).flovl.stamp
$$(BUILD)/$(1).flovl.stamp: $$(BUILD)/$(1).o
	@if $$(FLASHER_NM) -u $$< | grep -qE "$$(FLASHER_BANNED_SYMS)"; then \
	    echo "ERROR: RAM-resident $$< references flash helpers:"; \
	    $$(FLASHER_NM) -u $$< | grep -E "$$(FLASHER_BANNED_SYMS)"; \
	    exit 1; \
	fi
	@echo "OBJCOPY $$< [all sections -> .flovl.* overlay]"
	@$$(OBJCOPY) --prefix-alloc-sections=.flovl $$<
	@touch $$@
endef
$(foreach t,$(FLASHER_RAM_TUS),$(eval $(call flasher_ramfunc_rule,$(t))))

# Generated linker wrapper: the real chip script + the overlay fragment (see
# the FLASHER_CHIP_LD/LDSCRIPT block in flasher.mk).
$(BUILD)/flasher-main.ld: ./firmware-flasher/flasher-overlay.ld $(FLASHER_CHIP_LD) | directory
	@echo "INCLUDE $(FLASHER_CHIP_LD)" > $@
	@cat ./firmware-flasher/flasher-overlay.ld >> $@

$(BUILD)/$(BIN).elf: $(BUILD)/flasher-main.ld
endif # EMSCRIPTEN
