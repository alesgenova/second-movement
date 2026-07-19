/*
 * Firmware-flasher overlay placement. This fragment is appended to the chip
 * linker script by the Makefile (which generates build/flasher-main.ld as
 * `INCLUDE <chip>.ld` + this file, and points gossamer's --script at it) --
 * gossamer itself is not modified. Appending after the whole chip script also
 * means the chip script's trailing `end = .` (newlib's heap base) has already
 * been computed and is unaffected by this section.
 *
 * The RAM-resident flasher TUs (firmware_flasher_core.c + the selected
 * decoder backend; every allocated section renamed to .flovl.* by the
 * Makefile's objcopy step) are NOT boot-resident: they cost zero RAM until a
 * flash session begins. Their load image lives in flash (LMA below), and the
 * face's firmware_flasher_overlay_load() copies-and-verifies it to the fixed
 * VMA window here at first-block time, just before the point of no return.
 * The window sits near the top of RAM, overlapping what is normally heap
 * headroom: the loader refuses to copy unless the heap break and the live
 * stack both clear the window (see the face), and above the overlay only the
 * stack remains, guarded at runtime by a canary at __flovl_vma_end.
 *
 * The base and the stack-reserve ceiling are mirrored in
 * firmware_flasher_core.h (FLASHER_OVERLAY_BASE); keep them in lockstep.
 */
SECTIONS {
  .flovl 0x20005000 : ALIGN(4) {
    __flovl_vma_start = .;
    *(.flovl*)
    . = ALIGN(4);
    __flovl_vma_end = .;
  } AT > flash
}

__flovl_lma_start = LOADADDR(.flovl);

ASSERT(__flovl_vma_start >= _ebss,
       "flasher overlay overlaps .bss; raise FLASHER_OVERLAY_BASE")
/* >= 2.5 KiB must remain above the overlay (canary + the whole flash
 * session's stack); grow the budget by LOWERING the base in both places. */
ASSERT(__flovl_vma_end <= 0x20007600,
       "flasher overlay overruns the stack reserve; lower FLASHER_OVERLAY_BASE")
