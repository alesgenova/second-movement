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

#include "serial_frame.h"
#include "sam.h"     /* Must precede the _SAML22_ check; sam.h is what
                      * defines the chip macro (via saml22.h). On Emscripten,
                      * sam.h provides stubs and does not define _SAML22_. */
#include <string.h>

/* On the SAML22 target use the DSU peripheral's hardware CRC32 engine
 * (memory mode, ~1 byte/cycle). Falls back to a software bit-by-bit CRC32
 * elsewhere (simulator / Emscripten / unit tests). Both produce identical
 * results: standard CRC32 variant (init 0xFFFFFFFF, poly 0xEDB88320
 * reflected, final XOR 0xFFFFFFFF). */
#if defined(_SAML22_) && !defined(__EMSCRIPTEN__)
  #define SERIAL_FRAME_USE_DSU_CRC32 1
#endif

#define PREAMBLE_AA 0xAA
#define PREAMBLE_55 0x55

#define CRC32_INIT      0xFFFFFFFFul
#define CRC32_FINAL_XOR 0xFFFFFFFFul

#define LENGTH_MASK  0x03FFu   /* low 10 bits */
#define FLAGS_SHIFT  10u
#define FLAGS_MASK   0x3Fu     /* 6 bits */

/* CRC region offsets inside crc_buf. */
#define CRCBUF_ID_LO   0
#define CRCBUF_ID_HI   1
#define CRCBUF_LENFLAGS_LO  2
#define CRCBUF_LENFLAGS_HI  3
#define CRCBUF_PAYLOAD_OFF  4

/* State machine states. */
enum {
    HUNT_AA0,        /* matched 0 bytes; expect AA */
    HUNT_55_0,       /* matched AA;     expect 55 */
    HUNT_AA_1,       /* matched AA 55;  expect AA */
    HUNT_55_1,       /* matched AA 55 AA; expect 55 */
    ID_LO,
    ID_HI,
    LENFLAGS_LO,
    LENFLAGS_HI,
    PAYLOAD,
    CRC_BYTES,       /* collecting 4 CRC bytes, count tracked by crc_bytes_seen */
};

#ifdef SERIAL_FRAME_USE_DSU_CRC32

/* Hardware CRC32 via DSU. The buffer must be 4-byte aligned and the length
 * must be a multiple of 4; both guaranteed by the serial_frame format and
 * struct alignment.
 *
 * IMPORTANT: DSU registers are PAC-write-protected by default on SAM L22.
 * Writes without first unlocking via PAC->WRCTRL silently succeed-but-do-
 * nothing (no fault, just dropped), so DSU->CTRL never starts the CRC,
 * STATUSA.DONE never sets, and the wait loop spins forever.
 *
 * Result convention: DSU applies the final XOR with 0xFFFFFFFF internally.
 * DATA after computation IS already the standard IEEE 802.3 CRC32, so
 * we read it directly with no further XOR (matches ASF's dsu_crc32_cal). */
static uint32_t compute_crc32(const uint8_t *data, uint32_t len) {
    PAC->WRCTRL.reg = PAC_WRCTRL_PERID(ID_DSU) | PAC_WRCTRL_KEY_CLR;

    /* CRITICAL: do NOT use the DSU_ADDR_ADDR / DSU_LENGTH_LENGTH macros
     * here: they shift the value left by 2 (because the ADDR/LENGTH
     * fields live at bits 2..31). `data` is already a 4-aligned pointer
     * and `len` is already a multiple of 4, so the bottom 2 bits are
     * naturally 0 and we want the value written verbatim. Using the
     * macros would point DSU at (data << 2) for (len << 2) bytes,
     * a wildly wrong region producing a garbage CRC. */
    DSU->ADDR.reg    = (uint32_t)data;
    DSU->LENGTH.reg  = len;
    DSU->DATA.reg    = CRC32_INIT;
    DSU->STATUSA.reg = DSU_STATUSA_DONE | DSU_STATUSA_BERR;  /* clear */
    DSU->CTRL.reg    = DSU_CTRL_CRC;                         /* start */
    while (!(DSU->STATUSA.reg & DSU_STATUSA_DONE)) { /* spin */ }
    /* Per Microchip: DSU returns the NON-complemented CRC. To match the
     * standard IEEE 802.3 / zlib.crc32 variant, XOR with 0xFFFFFFFF. */
    uint32_t crc = DSU->DATA.reg ^ CRC32_FINAL_XOR;

    PAC->WRCTRL.reg = PAC_WRCTRL_PERID(ID_DSU) | PAC_WRCTRL_KEY_SET;
    return crc;
}

#else  /* software fallback */

#define CRC32_POLY 0xEDB88320ul

static uint32_t compute_crc32(const uint8_t *data, uint32_t len) {
    uint32_t crc = CRC32_INIT;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ CRC32_POLY) : (crc >> 1);
        }
    }
    return crc ^ CRC32_FINAL_XOR;
}

#endif

void serial_frame_parser_init(serial_frame_parser_t *p, serial_frame_parser_cb_t cb, void *user) {
    memset(p, 0, sizeof(*p));
    p->state = HUNT_AA0;
    p->cb    = cb;
    p->user  = user;
}

void serial_frame_parser_reset(serial_frame_parser_t *p) {
    /* Drop the in-flight frame only; stats, callback, and any delivered
     * frame are left alone. Mirror the parse-position fields cleared at the
     * start of a fresh frame so the next preamble starts from a clean slate. */
    p->state          = HUNT_AA0;
    p->payload_idx    = 0;
    p->crc_received   = 0;
    p->crc_bytes_seen = 0;
}

void serial_frame_parser_feed(serial_frame_parser_t *p, const uint8_t *bytes, size_t n) {
    for (size_t i = 0; i < n; i++) {
        uint8_t b = bytes[i];
        p->bytes_total++;

        switch (p->state) {
            case HUNT_AA0:
                if (b == PREAMBLE_AA) p->state = HUNT_55_0;
                break;

            case HUNT_55_0:
                if      (b == PREAMBLE_55) p->state = HUNT_AA_1;
                else if (b == PREAMBLE_AA) {/* still matched 1: stay */}
                else                       p->state = HUNT_AA0;
                break;

            case HUNT_AA_1:
                if (b == PREAMBLE_AA) p->state = HUNT_55_1;
                else                  p->state = HUNT_AA0;
                break;

            case HUNT_55_1:
                if (b == PREAMBLE_55) {
                    /* Full preamble matched: start a new frame. */
                    p->crc_received   = 0;
                    p->crc_bytes_seen = 0;
                    p->state          = ID_LO;
                } else if (b == PREAMBLE_AA) {
                    /* Sequence ends in AA, could start a new attempt. */
                    p->state = HUNT_55_0;
                } else {
                    p->state = HUNT_AA0;
                }
                break;

            case ID_LO:
                p->crc_buf[CRCBUF_ID_LO] = b;
                p->state = ID_HI;
                break;

            case ID_HI:
                p->crc_buf[CRCBUF_ID_HI] = b;
                p->id = (uint16_t)p->crc_buf[CRCBUF_ID_LO]
                      | ((uint16_t)p->crc_buf[CRCBUF_ID_HI] << 8);
                p->state = LENFLAGS_LO;
                break;

            case LENFLAGS_LO:
                p->crc_buf[CRCBUF_LENFLAGS_LO] = b;
                p->state = LENFLAGS_HI;
                break;

            case LENFLAGS_HI: {
                p->crc_buf[CRCBUF_LENFLAGS_HI] = b;
                uint16_t lf = (uint16_t)p->crc_buf[CRCBUF_LENFLAGS_LO]
                            | ((uint16_t)p->crc_buf[CRCBUF_LENFLAGS_HI] << 8);
                p->length = lf & LENGTH_MASK;
                p->flags  = (lf >> FLAGS_SHIFT) & FLAGS_MASK;

                if (p->length > SERIAL_FRAME_MAX_PAYLOAD) {
                    /* Bogus length: almost certainly noise that happened to
                     * match the preamble. Drop and resync. */
                    p->frames_bad_length++;
                    p->state = HUNT_AA0;
                } else {
                    /* Physical payload is the declared length rounded up to
                     * the next multiple of 4. Trailing bytes are padding
                     * (the transmitter writes zeros); they enter the CRC
                     * but are not delivered to the consumer. A zero-length
                     * payload has physical_length 0 and no payload bytes on
                     * the wire, so skip straight to the CRC, otherwise the
                     * PAYLOAD state would swallow the first CRC byte. */
                    p->physical_length = (p->length + 3u) & ~3u;
                    p->payload_idx     = 0;
                    p->state           = (p->physical_length == 0) ? CRC_BYTES : PAYLOAD;
                }
                break;
            }

            case PAYLOAD:
                /* Buffer holds physical bytes (including padding). Consumer
                 * only sees the first `length` via the callback. */
                if (p->payload_idx < SERIAL_FRAME_MAX_PAYLOAD) {
                    p->crc_buf[CRCBUF_PAYLOAD_OFF + p->payload_idx] = b;
                }
                p->payload_idx++;
                if (p->payload_idx >= p->physical_length) {
                    p->state = CRC_BYTES;
                }
                break;

            case CRC_BYTES: {
                /* Wire order is little-endian: lo byte first. */
                p->crc_received |= ((uint32_t)b) << (8u * p->crc_bytes_seen);
                p->crc_bytes_seen++;
                if (p->crc_bytes_seen >= 4) {
                    uint32_t computed = compute_crc32(
                        p->crc_buf, CRCBUF_PAYLOAD_OFF + p->physical_length);
                    if (p->crc_received == computed) {
                        p->frames_valid++;
                        p->payload_bytes_valid += p->length;
                        if (p->cb) {
                            serial_frame_t frame = {
                                .id = p->id,
                                .length  = p->length,
                                .flags   = p->flags,
                                .payload = &p->crc_buf[CRCBUF_PAYLOAD_OFF],
                            };
                            p->cb(&frame, p->user);
                        }
                    } else {
                        p->frames_bad_crc++;
                    }
                    p->state = HUNT_AA0;
                }
                break;
            }

            default:
                p->state = HUNT_AA0;
                break;
        }
    }
}

size_t serial_frame_encode(uint8_t *out, size_t out_size,
                       uint16_t id, uint16_t length, uint8_t flags,
                       const uint8_t *payload) {
    if (length > SERIAL_FRAME_MAX_PAYLOAD) return 0;
    uint16_t physical   = (uint16_t)((length + 3u) & ~3u);
    size_t   frame_size = (size_t)(4u + 4u + physical + 4u);  /* preamble + id/len+flags + padded payload + crc */
    if (out_size < frame_size) return 0;

    /* Preamble */
    out[0] = PREAMBLE_AA;
    out[1] = PREAMBLE_55;
    out[2] = PREAMBLE_AA;
    out[3] = PREAMBLE_55;

    /* id (LE) */
    out[4] = (uint8_t)(id     );
    out[5] = (uint8_t)(id >> 8);

    /* Length (10 bits) + flags (6 bits), LE */
    uint16_t lenflags = ((uint16_t)(flags & FLAGS_MASK) << FLAGS_SHIFT)
                      | (length & LENGTH_MASK);
    out[6] = (uint8_t)(lenflags     );
    out[7] = (uint8_t)(lenflags >> 8);

    /* Payload + zero padding to a 4-byte boundary. When length is 0 there is
     * no payload and physical is 0, so both the copy and the pad loop are
     * no-ops; guard the memcpy since `payload` may legitimately be NULL. */
    if (length > 0) memcpy(&out[8], payload, length);
    for (uint16_t i = length; i < physical; i++) {
        out[8 + i] = 0;
    }

    /* CRC32 over id || len+flags || padded payload (4 + physical bytes,
     * starting at offset 4, exactly the layout the parser feeds into the
     * same compute_crc32). */
    uint32_t crc = compute_crc32(&out[4], 4u + physical);
    size_t   crc_off = 8u + physical;
    out[crc_off + 0] = (uint8_t)(crc      );
    out[crc_off + 1] = (uint8_t)(crc >>  8);
    out[crc_off + 2] = (uint8_t)(crc >> 16);
    out[crc_off + 3] = (uint8_t)(crc >> 24);

    return frame_size;
}
