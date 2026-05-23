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

#include <string.h>
#include "movement_optical.h"
#include "movement.h"

#ifdef HAS_OPTICAL_LINK

/* Internal phases. */
enum {
    LINK_IDLE = 0,   /* closed */
    LINK_LISTENING,  /* RX open, polling */
    LINK_SETTLING,   /* response queued, waiting out the settle delay (RX still open) */
    LINK_SENDING,    /* TX open, draining */
};

/* Pending outbound message kind. */
enum {
    PEND_NONE = 0,
    PEND_FRAME,
    PEND_ACK,
};

/* Idle polls before resyncing the parser: max(~3 byte-times, ~0.5s), floored at 2
 * so a lone jittery poll can't discard a mid-arrival frame at high baud. */
static uint16_t link_compute_idle_limit(uint32_t baud, uint8_t tick_hz) {
    uint32_t byte_gap = (30u * (uint32_t)tick_hz + baud - 1u) / baud;   /* ceil(~3 byte-times) */
    uint32_t half_sec = ((uint32_t)tick_hz + 1u) / 2u;
    uint32_t limit    = byte_gap > half_sec ? byte_gap : half_sec;
    if (limit < 2u) limit = 2u;
    return (uint16_t)limit;
}

/* Open RX with the link config, request poll_hz, enter LISTENING. */
static void link_open_rx(movement_optical_t *l) {
    watch_optical_close();
    watch_optical_config_t cfg = {
        .baud = l->cfg.rx_baud, .irda = l->cfg.irda, .invert = l->cfg.rx_invert,
    };
    watch_optical_open(WATCH_OPTICAL_DIR_RX, &cfg);
    movement_request_tick_frequency(l->cfg.poll_hz);
    l->rx_idle_polls = 0;
    l->rx_idle_limit = link_compute_idle_limit(l->cfg.rx_baud, l->cfg.poll_hz);
    l->phase = LINK_LISTENING;
}

/* Flip to TX and issue the pending message (staged ACK bytes, or the frame). */
static void link_flip_to_tx_and_send(movement_optical_t *l) {
    watch_optical_close();
    watch_optical_config_t cfg = {
        .baud = l->cfg.tx_baud, .irda = l->cfg.irda, .invert = l->cfg.tx_invert,
    };
    watch_optical_open(WATCH_OPTICAL_DIR_TX, &cfg);
    if (l->pend_kind == PEND_ACK) {
        watch_optical_send_raw(l->ack_buf, l->ack_len);
    } else {
        watch_optical_send(l->pend_payload, l->pend_size, l->pend_id, l->pend_flags);
    }
    l->pend_kind = PEND_NONE;
    l->phase = LINK_SENDING;
}

/* Tail of send_frame / send_ack: switch to tx_hz, then settle (a response from
 * LISTENING) or transmit now. Phase already validated + message staged. */
static bool link_start_send(movement_optical_t *l) {
    movement_request_tick_frequency(l->cfg.tx_hz);
    if (l->phase == LINK_LISTENING && l->cfg.settle_ticks > 0) {
        l->settle_remaining = l->cfg.settle_ticks;
        l->phase = LINK_SETTLING;   /* stay in RX; flip at settle end */
    } else {
        link_flip_to_tx_and_send(l);
    }
    return true;
}

void movement_optical_init(movement_optical_t *l, const movement_optical_config_t *cfg) {
    memset(l, 0, sizeof(*l));
    l->cfg = *cfg;
    if (l->cfg.ack_count < 1u) l->cfg.ack_count = 1u;
    if (l->cfg.ack_count > MOVEMENT_OPTICAL_ACK_MAX) l->cfg.ack_count = MOVEMENT_OPTICAL_ACK_MAX;
    l->phase = LINK_IDLE;
}

void movement_optical_listen(movement_optical_t *l) {
    link_open_rx(l);
}

void movement_optical_stop(movement_optical_t *l) {
    watch_optical_close();
    movement_request_tick_frequency(1);
    l->phase = LINK_IDLE;
    l->pend_kind = PEND_NONE;
}

movement_optical_event_t movement_optical_tick(movement_optical_t *l) {
    switch (l->phase) {
        case LINK_SETTLING:
            if (l->settle_remaining > 0 && --l->settle_remaining == 0) {
                link_flip_to_tx_and_send(l);
            }
            return MOVEMENT_OPTICAL_NONE;

        case LINK_SENDING:
            watch_optical_poll();
            if (watch_optical_tx_idle()) {
                link_open_rx(l);
                return MOVEMENT_OPTICAL_TX_DONE;
            }
            return MOVEMENT_OPTICAL_NONE;

        case LINK_LISTENING:
            watch_optical_poll();
            if (watch_optical_rx_has_frame()) {
                l->rx_idle_polls = 0;
                return MOVEMENT_OPTICAL_FRAMES;
            }
            if (watch_optical_rx_idle()) {
                if (++l->rx_idle_polls >= l->rx_idle_limit) {
                    watch_optical_rx_reset();
                    l->rx_idle_polls = 0;
                }
            } else {
                l->rx_idle_polls = 0;
            }
            return MOVEMENT_OPTICAL_NONE;

        default:   /* LINK_IDLE */
            return MOVEMENT_OPTICAL_NONE;
    }
}

bool movement_optical_next_frame(movement_optical_t *l, watch_optical_frame_t *out) {
    if (l->phase != LINK_LISTENING) return false;
    /* Poll BEFORE receive, never after: receive() returns a pointer into the single
     * held-frame buffer that the next poll() would overwrite, so a post-receive poll
     * would corrupt the frame the caller is about to read. */
    watch_optical_poll();
    if (!watch_optical_receive(out)) return false;
    l->rx_idle_polls = 0;
    return true;
}

bool movement_optical_send_frame(movement_optical_t *l, const uint8_t *payload,
                                   uint16_t size, uint16_t id, uint8_t flags) {
    if (l->phase != LINK_LISTENING && l->phase != LINK_IDLE) return false;
    if (size > WATCH_OPTICAL_MAX_PAYLOAD) return false;
    /* A deferred data-frame response isn't staged across the settle; refuse it.
     * Immediate sends use the caller's payload only for this call. */
    if (l->phase == LINK_LISTENING && l->cfg.settle_ticks > 0) return false;
    l->pend_kind    = PEND_FRAME;
    l->pend_payload = payload;
    l->pend_size    = size;
    l->pend_id      = id;
    l->pend_flags   = flags;
    return link_start_send(l);
}

bool movement_optical_send_ack(movement_optical_t *l, uint16_t id) {
    if (l->phase != LINK_LISTENING && l->phase != LINK_IDLE) return false;
    uint8_t n = 0;
    for (uint8_t i = 0; i < l->cfg.ack_count; i++) {
        l->ack_buf[n++] = (uint8_t)(id);
        l->ack_buf[n++] = (uint8_t)(id >> 8);
    }
    l->ack_len   = n;
    l->pend_kind = PEND_ACK;
    return link_start_send(l);
}

#endif /* HAS_OPTICAL_LINK */
