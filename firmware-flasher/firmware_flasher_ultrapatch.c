/*
 * MIT License
 *
 * Copyright (c) 2026 Alessandro Genova
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * UltraPatch decoder TU for the firmware-flasher face.
 *
 * This is the one translation unit that includes UltraPatch's header-only
 * device decoder (patch_apply.h from the `ultrapatch` submodule, on the
 * include path in FIRMWARE_FLASHER_ULTRAPATCH builds; the submodule pin is
 * the wire contract shared with the host CLI). See
 * firmware_flasher_ultrapatch.h for the cross-TU contract with the face.
 *
 * THE RAM-TU DISCIPLINE (see firmware_flasher_core.c's banner) applies to
 * every byte of code and data this TU emits, because the decoder runs while
 * NVMCTRL erases the flash. Like the core, the whole object is captured: the
 * build renames EVERY allocated section with `objcopy
 * --prefix-alloc-sections=.flovl`, routing .text/.rodata/.data/.bss into the
 * load-on-demand flasher overlay (flasher-overlay.ld), copied to RAM by the
 * face at first-block time. Two libc escapes are closed below: memset is
 * redirected to a local RAM copy, and HAND_ROLLED_MEMMOVE selects UltraPatch's
 * private backward-copy loop instead of libc memmove. The build fails if the
 * renamed object still references any flash-resident helper.
 *
 * UltraPatch itself is .ramfunc-friendly by design: no division, no 64-bit or
 * floating-point helpers, no switch dispatch, and all working state lives in
 * the caller-owned PatchApply object (allocated by the launcher, ~6.7 KiB).
 */

#ifdef __arm__
#include "watch.h"
#endif

#if defined(FIRMWARE_FLASHER_ULTRAPATCH) && \
    ((defined(__arm__) && defined(HAS_IR_SENSOR)) || (!defined(__arm__) && !defined(__EMSCRIPTEN__)))

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "firmware_flasher_core.h"
#include "firmware_flasher_ultrapatch.h"

/* ---- libc escapes ----------------------------------------------------- *
 * rc_models.h calls memset (model init) and patch_apply_run memsets its
 * state; libc's memset lives in flash. Redirect every call in this TU to a
 * local copy. The volatile store keeps the optimizer from recognizing the
 * loop and re-emitting a libc memset call. Defined before the UltraPatch
 * includes so the macro covers them. */
static void *flasher_ultrapatch_memset(void *dst, int c, size_t n) {
    volatile uint8_t *d = (volatile uint8_t *)dst;
    while (n--) *d++ = (uint8_t)c;
    return dst;
}
#define memset flasher_ultrapatch_memset

/* UltraPatch's private backward-copy loop instead of libc memmove. */
#define HAND_ROLLED_MEMMOVE

/* ---- decoder integration hooks ---------------------------------------- */

#define PATCH_IMAGE_BASE      FLASHER_ULTRAPATCH_IMAGE_BASE
#define PATCH_IMAGE_CAPACITY  FLASHER_ULTRAPATCH_IMAGE_CAPACITY

/* patch_apply.h declares these extern hooks, but only after this point; the
 * early prototypes keep -Wmissing-prototypes happy for the definitions. */
uint8_t flash_read(uint32_t absolute_addr);
void    flash_write_page(uint32_t absolute_page_addr, const uint8_t page[256]);

/* flash_write_page has no failure channel, so a row that still failed after
 * the write layer's retries is latched as FATAL instead of being silently
 * voided: the image cannot verify anymore, and without the latch a wedged
 * NVMCTRL would burn a timeout on every remaining row -- minutes of deafness
 * for a doomed apply. The latch (a) turns the remaining writes into no-ops
 * and (b) makes the pull gate below abort the decoder at its next byte
 * request, so the session parks and answers the host's recovery within a
 * page or two. Reset at every patch_backend_run entry. */
static bool s_write_failed;

#ifndef __arm__

/* Hosted test build (see test/harness in sensor-watch-ir-tools): the test
 * driver supplies the flash backing store and translates the compile-time
 * image window to its fake-flash buffer. The default tableless CRC32_DECODE
 * is used (same zlib semantics as the DSU path below). */
extern uint8_t host_flash_read(uint32_t absolute_addr);
extern bool    host_flash_write_page(uint32_t absolute_page_addr, const uint8_t *page);
uint8_t flash_read(uint32_t absolute_addr) { return host_flash_read(absolute_addr); }
void flash_write_page(uint32_t absolute_page_addr, const uint8_t page[256]) {
    if (s_write_failed) return;
    if (!host_flash_write_page(absolute_page_addr, page)) s_write_failed = true;
}

#else /* watch hardware */

/* Reads only happen while NVMCTRL is idle (UltraPatch reads sources, scans
 * histograms, and CRCs strictly between page writes), so a plain memory-mapped
 * load is correct. */
uint8_t flash_read(uint32_t absolute_addr) {
    return *(volatile const uint8_t *)absolute_addr;
}

/* One OUTROW (= one SAM L22 row) erase+program, via the core's bounds-checked,
 * retrying primitive, with the burst cooldown (a patch frame can emit hundreds
 * of rows back-to-back; see FLASHER_BURST_WRITE_COOLDOWN). A row that still
 * failed after the retries trips the fatal latch above. */
void flash_write_page(uint32_t absolute_page_addr, const uint8_t page[256]) {
    if (s_write_failed) return;
    if (!firmware_flasher_write_row(absolute_page_addr, page, FLASHER_BURST_WRITE_COOLDOWN))
        s_write_failed = true;
}

/* CRC32_DECODE bridge: the DSU hardware engine (via the face's RAM-resident
 * firmware_flasher_crc32) when the range satisfies the DSU's alignment rules,
 * else a tableless bitwise fallback (identical zlib semantics). Our host tool
 * always ships row-aligned from/to sizes, so real patches always take the DSU
 * path; the fallback keeps a nonconforming envelope correct (it is rejected by
 * CRC content anyway) instead of faulting the DSU. */
static uint32_t flasher_ultrapatch_crc32(uint32_t start, uint32_t size);
#define CRC32_DECODE(start, size) flasher_ultrapatch_crc32((start), (size))

#endif /* __arm__ (hooks) */

#include "patch_apply.h"

_Static_assert(OUTROW == 256u,
               "UltraPatch OUTROW must equal the SAM L22 row size (wire contract)");
_Static_assert(FLASHER_ULTRAPATCH_IMAGE_BASE % 256u == 0u &&
               FLASHER_ULTRAPATCH_IMAGE_CAPACITY % 256u == 0u,
               "image window must be row-aligned");

#ifdef __arm__
static uint32_t flasher_ultrapatch_crc32(uint32_t start, uint32_t size) {
    if (((start | size) & 3u) == 0u && size != 0u)
        return firmware_flasher_crc32((const uint8_t *)start, size);
    /* Unaligned or empty range: bitwise reflected CRC-32 (zlib variant). */
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < size; i++) {
        c ^= flash_read(start + i);
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1u)));
    }
    return c ^ 0xFFFFFFFFu;
}
#endif

/* ---- public API (see firmware_flasher_ultrapatch.h) ------------------- */

uint32_t firmware_flasher_ultrapatch_state_size(void) {
    return (uint32_t)sizeof(PatchApply);
}

bool firmware_flasher_ultrapatch_run(void *state,
                                     int (*pull)(void *ctx, uint8_t *out),
                                     void *ctx) {
    /* patch_apply_run zeroes the state itself and gates on its own
     * revision-tagged source CRC before the first write and the target CRC
     * after the last, so DONE means "verified new image in flash". */
    return patch_apply_run((PatchApply *)state, pull, ctx) == PATCH_APPLY_DONE;
}

/* ---- decoder backend entry (see firmware_flasher_core.h) --------------- *
 *
 * pd->aux holds the caller-owned PatchApply state, allocated by the launcher
 * at the point of no return (NULL when it did not fit below the overlay: the
 * apply is then refused, the session parks, and a block-0 full flash recovers
 * -- there is no in-flash fallback in this format). The blob (envelope +
 * body) streams through patch_pull_byte unchanged; UltraPatch runs its own
 * source-CRC gate before the first write and target-CRC gate after the last,
 * so a true return means "verified new image", and the EXIT verify remains on
 * top of that. On a pull-source takeover abort the run returns false with the
 * core's takeover latch set, exactly like the detools backend. */
/* Pull gate: once a write has been latched fatal, stop feeding the decoder
 * at its next byte request (PATCH_PULL_END aborts the apply cleanly). Pulls
 * happen every byte or two even inside a page burst, so the abort lands
 * within roughly a page of the failure. */
static int up_pull_gate(void *ctx, uint8_t *out) {
    if (s_write_failed) return PATCH_PULL_END;
    return patch_pull_byte(ctx, out);
}

bool patch_backend_run(const firmware_flasher_patch_t *pd) {
    if (pd->aux == (uint8_t *)0) return false;   /* no state: park; full flash recovers */
    s_write_failed = false;
    return firmware_flasher_ultrapatch_run(pd->aux, up_pull_gate, (void *)0)
        && !s_write_failed;
}

#endif /* FIRMWARE_FLASHER_ULTRAPATCH && ((arm && HAS_IR_SENSOR) || hosted test build) */
