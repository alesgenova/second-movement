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

#ifndef FIRMWARE_FLASHER_ULTRAPATCH_H_
#define FIRMWARE_FLASHER_ULTRAPATCH_H_

/*
 * Cross-TU contract between the firmware-flasher face and the UltraPatch
 * decoder translation unit (firmware_flasher_ultrapatch.c). Only compiled into
 * FIRMWARE_FLASHER_ULTRAPATCH builds (the default); a
 * FIRMWARE_FLASHER_ULTRAPATCH=0 build links firmware_flasher_detools.c instead
 * and neither side of this contract exists.
 *
 * The decoder TU is RAM-resident IN ITS ENTIRETY: the build post-processes it
 * with `objcopy --prefix-alloc-sections=.flovl`, which routes its .text AND
 * its .rodata (UltraPatch has const tables) into the load-on-demand flasher
 * overlay (flasher-overlay.ld), copied to RAM by the face at first-block time,
 * not at boot. That is what makes it legal to run while NVMCTRL is erasing the
 * flash it decodes into. It follows that firmware_flasher_ultrapatch_state_size()
 * may only be called after the overlay is loaded.
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * The patchable image window, compile-time constants of the decoder
 * (PATCH_IMAGE_BASE / PATCH_IMAGE_CAPACITY). They mirror the face's writable
 * region [FLASHER_BOOTLOADER_END, FLASHER_APP_FLASH_END); the face's patch
 * ENTER validation requires base_addr == FLASHER_ULTRAPATCH_IMAGE_BASE, so a patch
 * for any other geometry is refused before the point of no return.
 */
#define FLASHER_ULTRAPATCH_IMAGE_BASE      0x2000u
#define FLASHER_ULTRAPATCH_IMAGE_CAPACITY  0x3C000u

/* ---- Provided by firmware_flasher_ultrapatch.c (RAM-resident) --------- */

/* sizeof(PatchApply): how many bytes of caller-owned decoder state
 * firmware_flasher_ultrapatch_run needs. The launcher places this buffer at
 * the start of Movement's heap at the point of no return; it needs 4-byte
 * alignment. Callable from flash-resident code once the overlay is loaded. */
uint32_t firmware_flasher_ultrapatch_state_size(void);

/* Run one whole patch apply: pulls the blob (envelope + body) byte-by-byte
 * through `pull` (the face passes patch_pull_byte), reconstructs the image in
 * place, and returns true only when UltraPatch's own source and target CRC
 * gates both passed. `state` is the state_size() buffer; zeroed internally.
 * On false the image may be untouched, partial, or complete-but-unverified;
 * the caller's park/takeover/EXIT machinery decides what happens next. */
bool firmware_flasher_ultrapatch_run(void *state,
                                     int (*pull)(void *ctx, uint8_t *out),
                                     void *ctx);

/* The row-write and DSU-CRC primitives this TU builds on come from the
 * RAM-resident core (declared in firmware_flasher_core.h), as does the
 * patch_backend_run contract this TU fulfills. */

#endif /* FIRMWARE_FLASHER_ULTRAPATCH_H_ */
