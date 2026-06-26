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

#include "watch_optical.h"

#ifdef HAS_IR_SENSOR

#include <string.h>
#include "sam.h"
#include "uart2.h"
#include "watch.h"

/* SERCOM topology, hardcoded for the watch's optical link:
 *   TX: PA12 (RED LED), mux D → SERCOM3 PAD[0]
 *   RX: PA04 (IRSENSE), mux D → SERCOM0 PAD[0]
 * Both pad 0, both TXPO_0 / RXPO_0. */
#define OPTICAL_TX_SERCOM  3
#define OPTICAL_RX_SERCOM  0

typedef enum {
    STATE_CLOSED = 0,
    STATE_RX,
    STATE_TX,
} state_t;

static state_t s_state = STATE_CLOSED;

/* TX side: one wire-frame buffer, drained by poll(). */
static uint8_t  s_tx_buf[SERIAL_FRAME_MAX_SIZE] __attribute__((aligned(4)));
static uint16_t s_tx_size = 0;
static uint16_t s_tx_pos  = 0;

/* RX side: parser + single held-frame slot. If a frame completes while the
 * slot is still occupied, we drop it and bump the counter, same memory cost
 * as a one-slot queue. Caller is expected to drain via receive() each tick. */
static serial_frame_parser_t s_parser;
static struct {
    bool     occupied;
    uint16_t id;
    uint16_t size;
    uint8_t  flags;
    uint8_t  payload[WATCH_OPTICAL_MAX_PAYLOAD];
} s_held;
static uint32_t s_dropped_count;

/* Bytes fed to the parser during the most recent poll(). Drives
 * watch_optical_rx_idle(): zero-fed + empty FIFO == the line is quiet. */
static uint16_t s_rx_fed_last_poll;

static void on_frame(const serial_frame_t *f, void *user) {
    (void) user;
    if (s_held.occupied) {
        s_dropped_count++;
        return;
    }
    memcpy(s_held.payload, f->payload, f->length);
    s_held.id       = f->id;
    s_held.size     = f->length;
    s_held.flags    = f->flags;
    s_held.occupied = true;
}

/* Force the LED off and back to plain GPIO. Defense in depth so it can
 * never be driven by a stale SERCOM3 configuration while RX is active. */
static void disengage_tx_pin(void) {
    HAL_GPIO_RED_pmuxdis();
    HAL_GPIO_RED_off();
}

/* Power down the phototransistor and detach it from SERCOM. */
static void disengage_rx_pin(void) {
    HAL_GPIO_IRSENSE_pmuxdis();
    HAL_GPIO_IRSENSE_off();
    HAL_GPIO_IR_ENABLE_off();
}

bool watch_optical_open(watch_optical_dir_t dir, const watch_optical_config_t *cfg) {
    if (s_state != STATE_CLOSED) return false;
    if (cfg == NULL) return false;

    uart2_sercom_config_t sercom_cfg = {
        .baud           = cfg->baud,
        .irda           = cfg->irda,
        .run_in_standby = true,
        .invert         = cfg->invert,
    };

    if (dir == WATCH_OPTICAL_DIR_TX) {
        /* Make sure the RX pin is fully gated, even if a previous session
         * left it half-configured, the phototransistor must not be powered
         * while the LED is transmitting next to it. */
        disengage_rx_pin();

        sercom_cfg.sercom = OPTICAL_TX_SERCOM;
        /* cfg->invert -> CTRLA.TXINV, which inverts only the DATA bits (ISO 7816
         * inverse convention), NOT the idle/optical polarity (see watch_optical.h).
         * In IrDA the SIR encoder sets the line polarity itself (idle low, 0 = a
         * brief high pulse), so invert has no effect here; leave it false. NB the
         * red LED is active-LOW, so this idle-low line leaves the LED LIT at idle
         * and blinking OFF per pulse; IrDA TX is high-LED-duty (~80% on), a power
         * disadvantage; see the watch_optical.h banner. */

        HAL_GPIO_RED_pmuxen(HAL_GPIO_PMUX_SERCOM_ALT);
        HAL_GPIO_RED_drvstr(1);
        if (!uart2_open(UART2_TXPO_0, &sercom_cfg, UART2_RXPO_NONE, NULL)) {
            disengage_tx_pin();
            return false;
        }

        s_tx_size = 0;
        s_tx_pos  = 0;
        s_state   = STATE_TX;
        return true;
    }

    if (dir == WATCH_OPTICAL_DIR_RX) {
        /* Same belt-and-suspenders: park the LED before lighting up the
         * sensor next to it. */
        disengage_tx_pin();

        sercom_cfg.sercom = OPTICAL_RX_SERCOM;

        HAL_GPIO_IR_ENABLE_out();
        HAL_GPIO_IR_ENABLE_clr();       /* power on phototransistor bias */
        HAL_GPIO_IRSENSE_in();
        HAL_GPIO_IRSENSE_pmuxen(HAL_GPIO_PMUX_SERCOM_ALT);
        if (!uart2_open(UART2_TXPO_NONE, NULL, UART2_RXPO_0, &sercom_cfg)) {
            disengage_rx_pin();
            return false;
        }

        serial_frame_parser_init(&s_parser, on_frame, NULL);
        s_held.occupied    = false;
        s_dropped_count    = 0;
        s_rx_fed_last_poll = 0;
        s_state            = STATE_RX;
        return true;
    }

    return false;
}

void watch_optical_close(void) {
    if (s_state == STATE_CLOSED) return;
    uart2_close();
    if (s_state == STATE_TX) {
        disengage_tx_pin();
    } else {
        disengage_rx_pin();
    }
    s_state = STATE_CLOSED;
}

void watch_optical_poll(void) {
    if (s_state == STATE_TX) {
        if (s_tx_pos < s_tx_size) {
            size_t n = uart2_tx_write(&s_tx_buf[s_tx_pos], s_tx_size - s_tx_pos);
            s_tx_pos += (uint16_t)n;
        }
        return;
    }
    if (s_state == STATE_RX) {
        /* Feed bytes one at a time so we stop the instant a frame lands in
         * the held slot; otherwise a second frame in the same drain would
         * be silently dropped by on_frame. The per-byte overhead is
         * negligible at our baud rates (<= 9600 → <= 150 B/tick at 64 Hz). */
        uint16_t fed = 0;
        while (!s_held.occupied) {
            uint8_t b;
            if (uart2_rx_read(&b, 1) != 1) break;
            serial_frame_parser_feed(&s_parser, &b, 1);
            fed++;
        }
        s_rx_fed_last_poll = fed;
    }
}

bool watch_optical_send(const uint8_t *payload, uint16_t size,
                        uint16_t id, uint8_t flags) {
    if (s_state != STATE_TX) return false;
    /* Reject if the previous frame's wire buffer hasn't fully drained.
     * (The UART FIFO may still hold bytes from the previous frame; that's
     * fine, the new frame's bytes queue up behind them.) */
    if (s_tx_pos < s_tx_size) return false;

    size_t n = serial_frame_encode(s_tx_buf, sizeof(s_tx_buf),
                               id, size, flags, payload);
    if (n == 0) return false;
    s_tx_size = (uint16_t)n;
    s_tx_pos  = 0;
    /* Push as much as the UART FIFO will accept right away; poll() will
     * pump whatever's left over subsequent ticks. */
    size_t q  = uart2_tx_write(s_tx_buf, s_tx_size);
    s_tx_pos  = (uint16_t)q;
    return true;
}

bool watch_optical_send_raw(const uint8_t *bytes, size_t n) {
    if (s_state != STATE_TX) return false;
    if (s_tx_pos < s_tx_size) return false;   /* previous send not drained */
    if (n == 0 || n > sizeof(s_tx_buf)) return false;

    memcpy(s_tx_buf, bytes, n);
    s_tx_size = (uint16_t)n;
    s_tx_pos  = 0;
    /* Push as much as the UART FIFO will accept; poll() pumps the rest,
     * same drain path as watch_optical_send(). */
    size_t q  = uart2_tx_write(s_tx_buf, s_tx_size);
    s_tx_pos  = (uint16_t)q;
    return true;
}

bool watch_optical_tx_idle(void) {
    if (s_state != STATE_TX) return true;
    return (s_tx_pos >= s_tx_size) && (uart2_tx_bytes_pending() == 0);
}

bool watch_optical_receive(watch_optical_frame_t *out) {
    if (s_state != STATE_RX || out == NULL || !s_held.occupied) return false;
    out->id = s_held.id;
    out->size     = s_held.size;
    out->flags    = s_held.flags;
    out->payload  = s_held.payload;
    s_held.occupied = false;
    return true;
}

void watch_optical_rx_stats(watch_optical_rx_stats_t *out) {
    if (out == NULL) return;
    out->bytes_total         = s_parser.bytes_total;
    out->frames_valid        = s_parser.frames_valid;
    out->frames_bad_crc      = s_parser.frames_bad_crc;
    out->frames_bad_length   = s_parser.frames_bad_length;
    out->payload_bytes_valid = s_parser.payload_bytes_valid;
    out->frames_dropped      = s_dropped_count;
}

bool watch_optical_rx_idle(void) {
    if (s_state != STATE_RX) return false;
    return s_rx_fed_last_poll == 0 && uart2_rx_bytes_pending() == 0;
}

void watch_optical_rx_reset(void) {
    if (s_state != STATE_RX) return;
    serial_frame_parser_reset(&s_parser);
}

#endif /* HAS_IR_SENSOR */
