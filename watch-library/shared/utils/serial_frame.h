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

#pragma once

#include <stdint.h>
#include <stddef.h>

/*
 * serial_frame
 *
 * Byte-fed state machine that pulls validated frames out of a noisy byte
 * stream encoded as:
 *
 *   +----------+---------+-------------+---------+---------+
 *   | preamble | id      | len + flags | payload | crc32   |
 *   | 4 bytes  | 2 bytes | 2 bytes     | N bytes | 4 bytes |
 *   +----------+---------+-------------+---------+---------+
 *                <--------- CRC32 covers this ---------->
 *                (id + len+flags + payload, the payload padded to a
 *                 4-byte multiple, so the region is always 4-aligned;
 *                 see "Padding scheme" below)
 *
 *   Multi-byte fields (id, len+flags, crc32) are little-endian on the wire.
 *
 *   preamble:  0xAA 0x55 0xAA 0x55
 *   id:        uint16 LE, application-defined frame identifier. The transport
 *              passes it through verbatim and assigns it no meaning.
 *   len+flags: bits [9:0]  = declared payload length in bytes, range
 *                            [0, MAX_PAYLOAD] (0 = a header-only frame with
 *                            no payload bytes on the wire)
 *              bits [15:10] = flags: 6 bits, application-defined. The
 *                            transport passes them through verbatim and
 *                            assigns them no meaning.
 *   crc32:     CRC-32 standard variant (poly 0xEDB88320 reflected, init
 *              0xFFFFFFFF, final XOR 0xFFFFFFFF) over id || len+flags
 *              || padded_payload. Little-endian on wire. Matches Python's
 *              zlib.crc32 / Ethernet / PNG / gzip.
 *
 * Padding scheme: the *declared* payload length can be any value in
 * [0, MAX_PAYLOAD], but the wire carries `physical = ceil(length / 4) * 4`
 * payload bytes; the trailing (physical - length) bytes are padding (any
 * value; the transmitter typically writes zeros). The CRC32 covers all
 * physical bytes including padding. The receiver computes physical from
 * the declared length, consumes that many bytes, validates CRC, then
 * delivers only the first `length` bytes to the consumer (padding is
 * dropped). This keeps the CRC region (id + len+flags + padded payload
 * = 4 + physical bytes) 4-byte aligned, which is what lets the SAML22 DSU
 * hardware CRC32 engine compute it directly with no special-case code (the
 * engine requires a 4-aligned address and a length that is a multiple of 4).
 * On SAML22 that hardware path is what runs; elsewhere a software CRC32 with
 * identical results is used (see compute_crc32 in serial_frame.c).
 *
 * Usage: allocate one serial_frame_parser_t per stream, init it with a callback,
 * then push received bytes via serial_frame_parser_feed(); typically called from
 * a UART drain loop. On each valid frame, the callback fires synchronously
 * with a serial_frame_t * whose pointers are valid only for the duration
 * of the call. Copy if you need to retain.
 *
 * Memory: ~565 bytes per instance (mostly the 516-byte CRC buffer = id +
 * len+flags + 512-byte max padded payload). Safe to allocate on the stack
 * or as a static.
 */

#define SERIAL_FRAME_MAX_PAYLOAD 512

/* Maximum on-wire frame size for any payload up to SERIAL_FRAME_MAX_PAYLOAD:
 *   preamble(4) + id(2) + len+flags(2) + padded_payload (multiple of 4,
 *   <= SERIAL_FRAME_MAX_PAYLOAD) + crc32(4) = 12 + SERIAL_FRAME_MAX_PAYLOAD. */
#define SERIAL_FRAME_MAX_SIZE   (12 + SERIAL_FRAME_MAX_PAYLOAD)

/* The 6-bit flags field (high bits of len+flags) carries no transport-defined
 * meaning; each application assigns its own. Define your own flag constants
 * in the consumer that produces and interprets them. */

typedef struct {
    uint16_t       id;
    uint16_t       length;   /* 0..MAX_PAYLOAD; when 0, `payload` points at the
                                buffer but must not be dereferenced */
    uint8_t        flags;
    const uint8_t *payload;
} serial_frame_t;

typedef void (*serial_frame_parser_cb_t)(const serial_frame_t *frame, void *user);

typedef struct {
    /* State machine */
    uint8_t  state;
    uint16_t id;                 /* parsed id for the in-progress frame */
    uint16_t length;             /* parsed declared length (10 bits) */
    uint16_t physical_length;    /* ceil(length / 4) * 4: bytes consumed from wire */
    uint8_t  flags;              /* parsed flags (6 bits) */
    uint16_t payload_idx;        /* counts up to physical_length */
    uint32_t crc_received;
    uint8_t  crc_bytes_seen;

    /* CRC buffer: id (2) + len+flags (2) + padded payload (up to MAX).
     * 4-byte aligned because the SAML22 DSU CRC32 engine requires it. The
     * software fallback works on any alignment but uses the same layout. */
    uint8_t crc_buf[4 + SERIAL_FRAME_MAX_PAYLOAD] __attribute__((aligned(4)));

    /* Dispatch */
    serial_frame_parser_cb_t cb;
    void                    *user;

    /* Stats: how many of each outcome since init(). */
    uint32_t bytes_total;          /* every byte fed in */
    uint32_t frames_valid;
    uint32_t frames_bad_crc;
    uint32_t frames_bad_length;
    uint32_t payload_bytes_valid;  /* sum of length over all valid frames */
} serial_frame_parser_t;

/* Reset the parser to a clean HUNT state and install the dispatch callback.
 * `cb` may be NULL if the caller only wants stats. `user` is opaque, passed
 * through to the callback. Stats are zeroed. */
void serial_frame_parser_init(serial_frame_parser_t *p, serial_frame_parser_cb_t cb, void *user);

/* Feed `n` bytes into the parser. May fire the callback zero or more times
 * synchronously (once per valid frame contained in this slice). State is
 * preserved across calls; partial frames are continued. */
void serial_frame_parser_feed(serial_frame_parser_t *p, const uint8_t *bytes, size_t n);

/* Abandon any in-progress (incomplete) frame and return to the HUNT state so
 * the next preamble locks cleanly. Use to resynchronize after the byte stream
 * has stalled (e.g. an optical dropout). Without it, a frame truncated
 * mid-flight would consume bytes belonging to the following frame, losing it
 * too. Does NOT touch stats, the callback, or any frame already delivered;
 * only the in-flight parse position is reset. Safe to call in any state (a
 * no-op when already hunting). */
void serial_frame_parser_reset(serial_frame_parser_t *p);

/* Encode one frame into `out`. Returns the number of bytes written, or 0
 * on invalid input (length out of [0, SERIAL_FRAME_MAX_PAYLOAD] or out_size too
 * small to hold the resulting wire frame). `payload` may be NULL when length
 * is 0 (a header-only frame).
 *
 * `out` must be 4-byte aligned (the SAML22 DSU CRC32 engine requires it; the
 * software fallback works on any alignment but uses the same buffer layout).
 * The high 2 bits of `flags` are dropped; only the low 6 bits go on the wire. */
size_t serial_frame_encode(uint8_t *out, size_t out_size,
                       uint16_t id, uint16_t length, uint8_t flags,
                       const uint8_t *payload);
