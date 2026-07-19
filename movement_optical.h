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
#include <stdbool.h>
#include <stddef.h>
#include "watch_optical.h"   /* pulls in HAS_IR_SENSOR via the board pins.h */

#ifdef HAS_IR_SENSOR

/*
 * movement_optical
 *
 * Half-duplex, turn-taking stop-and-wait transceiver over watch_optical. Owns the
 * RX<->TX flipping, ACK settle, TX drain, RX idle-resync, and the Movement tick
 * frequency; a face just decides, per received frame, whether to respond.
 *
 * Per EVENT_TICK call tick(); on FRAMES, drain with next_frame() and respond (or
 * not) via send_ack/send_frame. Contract:
 *   - Respond synchronously inside the drain loop; not responding keeps listening.
 *   - On FRAMES you MUST drain to false — an unconsumed held frame blocks RX.
 *   - One response per turn (the first send_* flips to TX; retransmits drop).
 *   - The link owns tick frequency (poll_hz / tx_hz / 1 Hz) and SERCOM open/close;
 *     don't call those yourself while a session is live.
 *
 * settle_ticks (at tx_hz) delays a response sent from LISTENING, letting the peer
 * recover from IR self-saturation; sends from IDLE or with settle_ticks==0 go now.
 * Not usable in low-energy mode; stop() before EVENT_LOW_ENERGY_UPDATE.
 */

/* Max bare-id ACK repeats (back-to-back copies on the weak return path). */
#define MOVEMENT_OPTICAL_ACK_MAX  4u

typedef struct {
    uint32_t rx_baud;         /* host -> watch */
    uint32_t tx_baud;         /* watch -> host (ACKs / responses) */
    bool     irda;            /* IrDA pulse encoding vs NRZ (both directions) */
    bool     tx_invert;       /* CTRLA.TXINV data-bit invert (ISO 7816); see watch_optical.h */
    bool     rx_invert;       /* CTRLA.RXINV data-bit invert (ISO 7816) */
    uint8_t  poll_hz;         /* Movement tick rate while LISTENING */
    uint8_t  tx_hz;           /* Movement tick rate while SETTLING + SENDING */
    uint8_t  settle_ticks;    /* tx_hz ticks to wait before a response TX */
    uint8_t  ack_count;       /* bare-id ACK repeats, clamped to [1, ACK_MAX] */
} movement_optical_config_t;

typedef enum {
    MOVEMENT_OPTICAL_NONE = 0,  /* nothing happened this tick */
    MOVEMENT_OPTICAL_FRAMES,    /* >= 1 frame ready; drain with next_frame() */
    MOVEMENT_OPTICAL_TX_DONE,   /* a send finished draining; listening again */
} movement_optical_event_t;

/* Link state. Embed one per session (in a face's context or as a static) and
 * treat the fields as opaque; drive it only through the functions below. */
typedef struct {
    movement_optical_config_t cfg;
    uint8_t  phase;               /* internal phase enum */
    uint16_t settle_remaining;
    uint16_t rx_idle_polls;
    uint16_t rx_idle_limit;
    /* pending outbound message (queued at send_* time, issued at TX flip) */
    uint8_t  pend_kind;           /* internal: none / frame / ack */
    uint16_t pend_id;
    uint8_t  pend_flags;
    uint16_t pend_size;
    const uint8_t *pend_payload;  /* frame payload (immediate sends only) */
    uint8_t  ack_buf[2 * MOVEMENT_OPTICAL_ACK_MAX];  /* staged ACK bytes */
    uint8_t  ack_len;
} movement_optical_t;

/* Initialize with `cfg` (copied); leaves the link closed. */
void movement_optical_init(movement_optical_t *l, const movement_optical_config_t *cfg);

/* Open RX and start listening (requests poll_hz). Closes any open direction first. */
void movement_optical_listen(movement_optical_t *l);

/* Close the link and reset the tick frequency to 1 Hz. Safe anytime. */
void movement_optical_stop(movement_optical_t *l);

/* Call once per EVENT_TICK. Returns FRAMES / TX_DONE / NONE (see the banner). */
movement_optical_event_t movement_optical_tick(movement_optical_t *l);

/* Next held frame (payload valid only until the next call); false when none left. */
bool movement_optical_next_frame(movement_optical_t *l, watch_optical_frame_t *out);

/* Queue a serial frame to transmit. False if a send is in progress, size is too
 * big, or it's a deferred data-frame response (from LISTENING with settle_ticks>0,
 * unsupported — only the ACK is staged). IDLE / settle_ticks==0 sends go immediately. */
bool movement_optical_send_frame(movement_optical_t *l, const uint8_t *payload,
                                   uint16_t size, uint16_t id, uint8_t flags);

/* Queue the bare-id ACK for `id` (× ack_count). False if a send is in progress. */
bool movement_optical_send_ack(movement_optical_t *l, uint16_t id);

#endif /* HAS_IR_SENSOR */
