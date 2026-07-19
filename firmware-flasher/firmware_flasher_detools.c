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
/* detools crle patch decoder backend for the firmware flasher.
 *
 * One of the two backend TUs (see firmware_flasher_core.h; the other is
 * firmware_flasher_ultrapatch.c) -- the Makefile links exactly one. This TU
 * is RAM-resident in its entirety via the same whole-TU section rename as the
 * core TU; the RAM-TU discipline documented there applies verbatim here (no
 * libc/libgcc calls, no division, explicit copy loops). It consumes only the
 * core's pull byte source and row-write primitive.
 */

#ifdef __arm__
#include "watch.h"
#endif
#include "firmware_flasher_core.h"

#if (defined(__arm__) && defined(HAS_IR_SENSOR)) || (!defined(__arm__) && !defined(__EMSCRIPTEN__))

/* Patch session params, copied at patch_backend_run entry from the verified
 * ENTER's descriptor (firmware_flasher_patch_t). s_aux is the sliding source
 * window; NULL = in-flash shift fallback. Statics (not locals) so every
 * helper reaches them from one base register on the M0+. */
static uint8_t *s_aux;        /* detools: aux window; ultrapatch: decoder state     */
static uint32_t s_aux_mask;   /* window size - 1 (mask, since no division here)     */
static uint32_t s_pbase;      /* app base of the images                             */
static uint32_t s_pfrom;      /* REF extent (source bound)                          */
static uint32_t s_pto;        /* NEW extent (bytes to reconstruct)                  */
static uint32_t s_pshift;     /* detools shift_size (the patch's read-coord origin)  */

/* ===================================================================== *
 *  detools backend: crle-decompressed, in place, mirrors the verified
 *  host-side reference apply (aux path).
 *
 *  The crle-compressed patch BODY streams in over 256 B frames (the launcher
 *  hands off the first one; the rest arrive through the pull source). The
 *  segment loop pulls decompressed bytes on demand. NEW is reconstructed one
 *  row at a time into a RAM buffer, then erase+written. Source bytes (original
 *  REF) come from flash where the row hasn't been overwritten yet, else from a
 *  sliding "aux window" of the last shift_size original bytes (saved just
 *  before each row is overwritten), or -- when malloc could not size the
 *  window -- from a one-shot in-flash relocation of REF (s_aux == NULL). No
 *  row is erased twice. The window is the malloc'd buffer the launcher passed
 *  (we never malloc/free in .ramfunc).
 *
 *  Two varint encodings are in play: crle's own (unsigned, 7-bit LSB groups)
 *  for its run lengths, and detools' signed varint (6-bit first byte) for the
 *  segment diff/extra/adjustment sizes inside the DECOMPRESSED stream.
 * ===================================================================== */

static uint32_t s_pshift_phys;/* in-flash shift only: where REF is physically placed
                               * = region_end - base - from (REF flush to flash end,
                               * independent of shift_size). Read maps patch index r
                               * -> base + s_pshift_phys + r. ENTER validated
                               * shift <= this, so the (>=0) delta is implicit.       */

/* The pull source returned FLASHER_PULL_END (takeover abort): latched so every
 * decode loop unwinds promptly instead of spinning on the 0x00 filler bytes
 * patch_in_byte() returns from then on. */
static bool s_pull_end;

/* One compressed body byte for the decode chain below. After an abort it
 * returns 0x00 filler; the s_pull_end checks in the loops unwind the apply. */
static uint8_t patch_in_byte(void) {
    uint8_t b = 0;
    if (!s_pull_end && patch_pull_byte((void *)0, &b) != FLASHER_PULL_BYTE)
        s_pull_end = true;
    return b;
}

/* crle decoder state. */
enum { CRLE_NEED = 0, CRLE_SCATTER, CRLE_REPEAT };
static uint8_t  s_crle_state;
static uint32_t s_crle_left;  /* scattered/repeat bytes still to emit */
static uint8_t  s_crle_byte;  /* the repeated byte */

/* crle's own length varint: unsigned, 7-bit groups LSB-first, 0x80 = continue. */
static uint32_t crle_varint(void) {
    uint32_t v = 0, off = 0;
    uint8_t b;
    do { b = patch_in_byte(); v |= (uint32_t)(b & 0x7Fu) << off; off += 7u; } while (b & 0x80u);
    return v;
}

/* Pull one DECOMPRESSED byte (matches detools CrleDecompressor). */
static uint8_t crle_out_byte(void) {
    for (;;) {
        if (s_pull_end) return 0;               /* pull source aborted: unwind */
        if (s_crle_state == CRLE_SCATTER) {
            if (s_crle_left) { s_crle_left--; return patch_in_byte(); }  /* literal passthrough */
            s_crle_state = CRLE_NEED;
        } else if (s_crle_state == CRLE_REPEAT) {
            if (s_crle_left) { s_crle_left--; return s_crle_byte; }
            s_crle_state = CRLE_NEED;
        }
        uint8_t kind = patch_in_byte();
        uint32_t n   = crle_varint();
        if (kind == 0u) {                       /* SCATTERED: n literal bytes */
            s_crle_left = n; s_crle_state = CRLE_SCATTER;
        } else {                                /* REPEATED: one byte, n times */
            s_crle_left = n; s_crle_byte = patch_in_byte(); s_crle_state = CRLE_REPEAT;
        }
    }
}

/* detools signed varint (6-bit first byte), read from the DECOMPRESSED stream. */
static int32_t patch_size(void) {
    uint8_t b = crle_out_byte();
    bool sign = (b & 0x40u) != 0;
    uint32_t v = b & 0x3Fu, off = 6u;
    while (b & 0x80u) { b = crle_out_byte(); v |= (uint32_t)(b & 0x7Fu) << off; off += 7u; }
    return sign ? -(int32_t)v : (int32_t)v;
}

/* Read n source bytes for the reconstruction into dst. Two strategies:
 *  - aux (s_aux != NULL): read original REF at index (from_offset - shift), from
 *    flash if not yet overwritten (index >= tip = current row start), else from
 *    the sliding aux window.
 *  - in-flash shift (s_aux == NULL): REF was relocated to s_pshift_phys (flush to
 *    flash end), so REF index r sits at base + s_pshift_phys + r, still intact
 *    because the segment loop only reads ahead of the write frontier; no window.
 * Returns false on a corrupt/over-reaching reference. */
static bool patch_read_source(uint32_t from_offset, uint8_t *dst,
                                              uint32_t n, uint32_t tip) {
    if (s_aux == NULL) {
        for (uint32_t i = 0; i < n; i++) {
            uint32_t r = from_offset + i - s_pshift;          /* REF index (as in aux path) */
            if (r >= s_pfrom) return false;                   /* beyond REF (or underflow): corrupt */
            dst[i] = *((volatile uint8_t *)(s_pbase + s_pshift_phys + r));
        }
        return true;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint32_t r = from_offset + i - s_pshift;          /* original-REF index */
        if (r >= s_pfrom) return false;                   /* beyond REF: corrupt */
        if (r >= tip) {
            dst[i] = *((volatile uint8_t *)(s_pbase + r));/* still original in flash */
        } else {
            if (tip - r > s_aux_mask + 1u) return false;  /* outside the window */
            dst[i] = s_aux[r & s_aux_mask];
        }
    }
    return true;
}

/* Reconstruct one NEW segment (= one row) into `row` (mirrors the host-side reference apply). */
static bool patch_apply_segment(uint32_t to_offset, uint32_t seg_to,
                                                uint32_t from_offset, uint8_t *row) {
    if (patch_size() != 0) return false;                  /* dfpatch_size must be 0 */
    uint32_t to_pos = 0;
    while (to_pos < seg_to) {
        if (s_pull_end) return false;                     /* pull source aborted: unwind */
        int32_t dsize = patch_size();                     /* diff run */
        if (dsize < 0 || to_pos + (uint32_t)dsize > seg_to) return false;
        if (dsize > 0) {
            uint8_t src[FLASHER_ROW_SIZE];
            if (!patch_read_source(from_offset, src, (uint32_t)dsize, to_offset)) return false;
            for (int32_t j = 0; j < dsize; j++)
                row[to_pos + (uint32_t)j] = (uint8_t)(crle_out_byte() + src[j]);
            from_offset += (uint32_t)dsize;
            to_pos      += (uint32_t)dsize;
        }
        int32_t esize = patch_size();                     /* extra (literal) run */
        if (esize < 0 || to_pos + (uint32_t)esize > seg_to) return false;
        for (int32_t j = 0; j < esize; j++) row[to_pos++] = crle_out_byte();
        from_offset += (uint32_t)patch_size();            /* adjustment (signed) */
    }
    return true;
}

/* In-flash shift fallback (s_aux == NULL): relocate the from_size of REF to
 * s_pshift_phys, flush against the app-flash end, NOT by the patch's shift_size
 * (constant placement, independent of shift_size, for uniform flash wear). The
 * source then reads straight from the relocated copy at base + s_pshift_phys + r,
 * no RAM window. Copy row-by-row HIGH-TO-LOW so a destination row never clobbers a
 * source row not yet read (when the physical shift < from_size the source and
 * destination ranges overlap). Both offsets are row-aligned, so /256 and *256 are
 * constant power-of-two shifts (no division helper). ~from_rows extra
 * erase+program cycles; aligning to the end keeps NEW/REF overlap minimal. */
static bool patch_shift_ref(void) {
    uint32_t from_rows = s_pfrom / FLASHER_ROW_SIZE;
    uint8_t row[FLASHER_ROW_SIZE];
    for (uint32_t k = from_rows; k-- > 0; ) {             /* k = from_rows-1 .. 0 */
        uint32_t src = s_pbase + k * FLASHER_ROW_SIZE;
        uint32_t dst = src + s_pshift_phys;
        for (uint32_t i = 0; i < FLASHER_ROW_SIZE; i++)   /* read source row (NVM idle) */
            row[i] = *((volatile uint8_t *)(src + i));
        if (!firmware_flasher_write_row(dst, row, FLASHER_BURST_WRITE_COOLDOWN)) return false;
    }
    return true;
}

/* Drive the whole patch apply (see the backend contract above). Returns true
 * once all of NEW is reconstructed + written (the final EXIT verify still gates
 * the reboot, in the main loop). */
bool patch_backend_run(const firmware_flasher_patch_t *pd) {
    s_aux = pd->aux; s_aux_mask = pd->aux_mask;
    s_pbase = pd->base; s_pfrom = pd->from_size;
    s_pto = pd->to_size; s_pshift = pd->shift_size;
    s_pull_end = false;
    bool shift_mode = (s_aux == NULL);
    /* In-flash shift fallback: place REF flush against the app-flash end (the
     * largest valid shift; ENTER validated base+shift+from <= end, so this is
     * >= the patch's shift_size). Relocate BEFORE consuming any patch bytes
     * (flash still holds the original REF in full at this point). */
    if (shift_mode) {
        s_pshift_phys = (FLASHER_APP_FLASH_END - s_pbase) - s_pfrom;
        if (!patch_shift_ref()) return false;
    }
    s_crle_state = CRLE_NEED; s_crle_left = 0;

    uint8_t row[FLASHER_ROW_SIZE];
    const uint32_t seg = FLASHER_ROW_SIZE;
    for (uint32_t to_off = 0; to_off < s_pto; to_off += seg) {
        uint32_t from_offset = (to_off + seg > s_pshift) ? (to_off + seg) : s_pshift;
        uint32_t seg_to = (s_pto - to_off < seg) ? (s_pto - to_off) : seg;
        if (!patch_apply_segment(to_off, seg_to, from_offset, row)) return false;
        for (uint32_t k = seg_to; k < seg; k++) row[k] = 0xFFu;  /* pad partial last row (erased) */
        /* aux strategy only: save this row's ORIGINAL bytes (still in flash) into
         * the window before we overwrite them, so later rows can reference them as
         * source. The shift strategy reads from the relocated copy, so no save. */
        if (!shift_mode)
            for (uint32_t k = 0; k < seg && (to_off + k) < s_pfrom; k++)
                s_aux[(to_off + k) & s_aux_mask] = *((volatile uint8_t *)(s_pbase + to_off + k));
        if (!firmware_flasher_write_row(s_pbase + to_off, row, FLASHER_BURST_WRITE_COOLDOWN)) return false;
    }
    return true;
}

#endif /* (arm && HAS_IR_SENSOR) || hosted test build */
