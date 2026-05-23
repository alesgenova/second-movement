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

#ifndef FIRMWARE_FLASHER_CORE_H_
#define FIRMWARE_FLASHER_CORE_H_

/*
 * Private contract between the firmware-flasher translation units:
 *
 *   firmware_flasher_face.c        flash-resident launcher (Movement face, TEST
 *                                  stage, ENTER validation, arming hand-off)
 *   firmware_flasher_core.c        RAM-resident, backend-agnostic flasher core
 *                                  (NVM/DSU/SERCOM primitives, frame parser,
 *                                  ACK path, pull byte source, dispatch,
 *                                  flasher_run)
 *   firmware_flasher_detools.c     detools crle patch decoder backend
 *   firmware_flasher_ultrapatch.c  UltraPatch patch decoder backend
 *
 * Exactly ONE backend TU is linked (Makefile selection via
 * FIRMWARE_FLASHER_ULTRAPATCH). The core and backend TUs are RAM-resident in
 * their ENTIRETY: no per-function section attributes -- the build renames
 * every allocated section with `objcopy --prefix-alloc-sections=.ramfunc`, so
 * complete capture is guaranteed by construction (code, literal pools, AND
 * .rodata). Calls out of those TUs may only target each other or RAM data:
 * libc/libgcc live in flash and are off-limits (verified by the objdump
 * relocation audit; see the core TU's banner for the full discipline).
 *
 * Any NON-ARM compile of the core and backend TUs is a "hosted test
 * build" (no custom define needed: __arm__ comes from the cross-compiler):
 * the hardware primitives are swapped for hooks provided by the test driver
 * -- see sensor-watch-ir-tools/flasher-sim -- and nothing in these TUs may
 * depend on watch headers off-ARM.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- Flash geometry (SAM L22; see instance/nvmctrl.h) ----------------- */

#define FLASHER_PAGE_SIZE      64u
#define FLASHER_ROW_SIZE       256u
#define FLASHER_PAGES_PER_ROW  (FLASHER_ROW_SIZE / FLASHER_PAGE_SIZE)   /* 4 */

/*
 * Writable region bounds, mirroring the SAM L22 linker map (saml22n18.ld):
 *
 *   [0x00000, 0x02000)  UF2 bootloader      (SACRED, never written)
 *   [0x02000, 0x3E000)  application flash    (the only region we may write)
 *   [0x3E000, 0x40000)  reserved EEPROM      (off-limits)
 *
 * Bricking the bootloader is the one failure with no recovery path (even USB
 * recovery needs it intact), so every write is bounds-checked against these.
 * Hosted test builds substitute their fake-flash window via the two globals.
 */
#if defined(__arm__) || defined(__EMSCRIPTEN__)
#define FLASHER_BOOTLOADER_END  0x2000u    /* exclusive: first writable byte */
#define FLASHER_APP_FLASH_END   0x3E000u   /* exclusive: EEPROM begins here  */
#else
extern uint32_t flasher_hosted_bootloader_end, flasher_hosted_app_flash_end;
#define FLASHER_BOOTLOADER_END  flasher_hosted_bootloader_end
#define FLASHER_APP_FLASH_END   flasher_hosted_app_flash_end
#endif

/* ---- Wire protocol (shared with the host flasher tool) ---------------- *
 *
 * Frames use the serial_frame format. These frame flag bits travel
 * host -> watch; the watch -> host ACK carries no flags (it is the bare
 * 2-byte frame id echoed back).
 */
#define IR_FLASHER_FLAG_TEST        (1u << 2)  /* link-test frame: watch ACKs, does NOT commit */
#define IR_FLASHER_FLAG_ENTER       (1u << 3)  /* enter the real flasher; payload empty (full flash), or
                                                  with FLAG_PATCH+FLAG_VERIFY = the reference descriptor */
#define IR_FLASHER_FLAG_VERIFY      (1u << 4)  /* EXIT: final whole-image CRC check; payload = descriptor */
/* Delta (patch) flash rather than a verbatim uf2-like one. On ENTER (with
 * FLAG_VERIFY): "patch session", ACK only if the reference descriptor matches
 * current flash. On a data block: this is a patch frame, not a uf2-like row.
 *
 * THE BIT IS THE FORMAT: each patch dialect has its own flag bit, and a build
 * only recognizes its compiled-in decoder's bit as "patch" -- everywhere (the
 * ENTER, the first-block hand-off, the body stream). A frame in the other
 * dialect is simply not a patch frame: ignored in silence, so a format
 * mismatch (wrong host --patch-format, or a mixed-up --start-block resume)
 * stalls at its first frame instead of feeding the wrong decoder.
 * NOTE: this makes the core backend-sensitive; the Makefile's backend-flag
 * stamp rebuilds it on a toggle. */
#define IR_FLASHER_FLAG_PATCH_DETOOLS  (1u << 5)   /* detools crle dialect (legacy value) */
#define IR_FLASHER_FLAG_PATCH_ULTRA    (1u << 1)   /* UltraPatch dialect */
#ifdef FIRMWARE_FLASHER_ULTRAPATCH
#define IR_FLASHER_FLAG_PATCH  IR_FLASHER_FLAG_PATCH_ULTRA
#else
#define IR_FLASHER_FLAG_PATCH  IR_FLASHER_FLAG_PATCH_DETOOLS
#endif

/* The ACK is a bare frame-id echo; repeating it (back to back) gives the host's
 * sliding-window matcher more chances to find an intact copy on the weak
 * watch->host return path, at the cost of a longer ACK. */
#define IR_FLASHER_ACK_COUNT_MIN    1u
#define IR_FLASHER_ACK_COUNT_MAX    4u

/* The EXIT (VERIFY) frame's payload: the image descriptor, 12 bytes LE
 * (base_addr, total_length, image_crc32). */
#define IR_FLASHER_DESCRIPTOR_SIZE  12u

/* A data block's payload: target address (4 LE) followed by one full
 * 256-byte row. The watch writes the row verbatim to the target address. */
#define IR_FLASHER_BLOCK_PAYLOAD    (4u + FLASHER_ROW_SIZE)   /* 260 */

/* Patch (delta) ENTER payload: 20 bytes LE
 *   base(4) from_size(4) ref_crc32(4) to_size(4) shift_size(4)
 * (see enter_patch_setup in the face for the full semantics). */
#define IR_FLASHER_PATCH_ENTER_SIZE  20u

/*
 * Everything the RAM-resident patch apply needs, assembled by the launcher at
 * first-block time and handed through firmware_flasher_arm_and_run ->
 * flasher_run -> patch_backend_run. In the detools build `aux` is a
 * (aux_mask+1)-byte, power-of-two sliding window of original-REF bytes; in the
 * UltraPatch build it is the decoder's caller-owned state buffer (NULL when
 * the launcher could not allocate it below the overlay: the backend then
 * refuses to run and the parked session is recovered with a block-0 full
 * flash). from/to/shift_size come from the verified ENTER.
 */
typedef struct {
    uint8_t *aux;            /* detools: REF window; ultrapatch: decoder state */
    uint32_t aux_mask;       /* window size - 1 (a power-of-two mask); 0 for ultrapatch */
    uint32_t base;           /* app base of the images */
    uint32_t from_size;      /* REF extent (source bound) */
    uint32_t to_size;        /* NEW extent (bytes to reconstruct) */
    uint32_t shift_size;     /* detools shift_size = max source lookback */
} firmware_flasher_patch_t;

/* ---- Bounds check ------------------------------------------------------ *
 *
 * Pure arithmetic with no hardware. The single source of truth for "may I
 * write here": the ENTER/EXIT descriptors and every per-row write are
 * validated through it (defense in depth). True iff [addr, addr + len) lies
 * entirely within the writable application region: does not touch the
 * bootloader below or the reserved EEPROM above, and does not wrap. len must
 * be non-zero. static inline: the flash-resident face and the RAM-resident
 * core each get their own copy in their own residency. */
static inline bool firmware_flasher_range_writable(uint32_t addr, uint32_t len) {
    if (len == 0) return false;
    /* Guard against wrap-around: addr + len must not overflow. */
    if (addr > 0xFFFFFFFFu - len) return false;
    uint32_t end = addr + len;   /* exclusive */
    if (addr < FLASHER_BOOTLOADER_END) return false;   /* touches bootloader */
    if (end  > FLASHER_APP_FLASH_END)  return false;   /* touches EEPROM/OOB */
    return true;
}

/* ---- Overlay geometry (hardware builds only) --------------------------- *
 *
 * The core + backend TUs are NOT boot-resident: their sections (renamed to
 * .flovl.* by the build) live in flash and are copied to a fixed VMA window
 * near the top of RAM by the face's loader at first-block time (see
 * flasher-overlay.ld for the full story). FLASHER_OVERLAY_BASE mirrors the
 * linker fragment's placement and is the heap ceiling for a flash session:
 * the loader refuses to copy if the heap break has crossed it, and the
 * launcher's allocations (aux window / decoder state) must fit below it.
 * A canary word just past the overlay guards against the session's stack
 * digging into the loaded code. */
#ifdef __arm__
#define FLASHER_OVERLAY_BASE    0x20005000u   /* keep in lockstep with flasher-overlay.ld */
#define FLASHER_OVERLAY_CANARY  0x5AFE57ACu

extern uint8_t __flovl_vma_start[], __flovl_vma_end[];
extern const uint8_t __flovl_lma_start[];

static inline volatile uint32_t *flasher_overlay_canary_addr(void) {
    return (volatile uint32_t *)(((uint32_t)(uintptr_t)__flovl_vma_end + 3u) & ~3u);
}
#endif

/* ---- Core exports (firmware_flasher_core.c; all RAM-resident) ---------- */

/* Erase + program one 256-byte row; bounds-checked via range_writable, addr
 * row-aligned. NVMCTRL failures/timeouts are retried a few times per row
 * (the expired NVM timeout doubles as a battery cooldown); false only when
 * the row still failed, or immediately on a bounds/alignment error.
 * `cooldown_ticks` (128 Hz RTC) is idled after a successful commit: pass
 * FLASHER_BURST_WRITE_COOLDOWN from burst-writing callers (patch backends) to
 * duty-cycle the NVM current on a coin cell, and 0 from link-paced ones. */
bool firmware_flasher_write_row(uint32_t addr, const uint8_t *data, uint8_t cooldown_ticks);

/* Post-commit rest per burst-written row, in 128 Hz RTC ticks. 2 gives each
 * row a full 7.8 ms RTC period of NVM idle, halving a sustained burst's duty
 * cycle; 0 disables the rest. 0 for now: no burst has shown real coin-cell
 * sag -- the stall once blamed on brownout was the host flasher's stale-ACK
 * livelock. */
#define FLASHER_BURST_WRITE_COOLDOWN  0u

/* DSU hardware CRC-32 (zlib variant); `data` 4-aligned, `len` a multiple of
 * 4, NVMCTRL idle. Used by the parser, the verifies, and (in ultrapatch
 * builds) the decoder TU's CRC32_DECODE hook. Also callable pre-arm from the
 * flash-resident launcher (ENTER pre-flight). */
uint32_t firmware_flasher_crc32(const uint8_t *data, uint32_t len);

/* RTC MODE0 COUNT32 ticks at 128 Hz (1024 Hz / DIV8; see rtc32.c). The core
 * times its ACK settle and hardware-wait bounds with it; the face's arming
 * step uses it to derive the settle from the TEST-stage link timings. */
#define FLASHER_RTC_HZ  128u

/* ACK timing knobs, computed and written by the launcher's arming step before
 * flasher_run takes over (see firmware_flasher_arm_and_run). */
extern uint32_t flasher_ack_settle_ticks;   /* RTC ticks to settle before each ACK */
extern uint8_t  flasher_ack_count;          /* times to repeat the id per ACK (1..4) */

/* Pull one compressed patch byte for a decoder backend. The contract
 * deliberately matches UltraPatch's PatchPull callback (write one byte to
 * *out and return FLASHER_PULL_BYTE, or FLASHER_PULL_END to abort) so either
 * backend plugs in unchanged. Serves the current body frame; at a frame
 * boundary ACKs the consumed frame and sleeps until the next-in-sequence one
 * (dup re-ACK / silence rules inside). Returns FLASHER_PULL_END exactly when
 * a full-flash takeover aborts the patch session. */
enum { FLASHER_PULL_END = 0, FLASHER_PULL_BYTE = 1 };
int patch_pull_byte(void *ctx, uint8_t *out);

/* The RAM-resident main loop (never returns); see its definition. Only the
 * face's arming step may call this, after masking IRQs and raw-configuring
 * the link. */
void flasher_run(const firmware_flasher_patch_t *patch,
                 uint16_t first_block_id,
                 uint32_t first_block_addr,
                 bool first_block_is_patch,
                 uint16_t first_block_len,
                 const uint8_t *first_block);

/* ---- Backend contract (one of the backend TUs provides this) ----------- *
 *
 * Called by flasher_run once the byte source is primed with the first body
 * frame (not yet ACKed). Pull the compressed body via patch_pull_byte(),
 * reconstruct NEW in [pd->base, pd->base + pd->to_size) with
 * firmware_flasher_write_row(), and return true once the image is fully
 * written. Return false on ANY failure -- corrupt stream, write error, or a
 * pull-source abort (takeover) -- withOUT ACKing the current frame; the
 * caller distinguishes recovery cases via its takeover latch. The final EXIT
 * CRC remains the integrity gate either way. */
bool patch_backend_run(const firmware_flasher_patch_t *pd);

#endif /* FIRMWARE_FLASHER_CORE_H_ */
