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
#include "movement.h"          /* pulls in HAS_IR_SENSOR via the board pins.h */
#include "serial_frame.h"

#ifdef HAS_IR_SENSOR

/*
 * watch_optical
 *
 * Half-duplex framed messaging over the watch's onboard optical link:
 *
 *   RX: phototransistor (IRSENSE) routed to SERCOM0 PAD[0] (USART RX),
 *       phototransistor bias powered via IR_ENABLE while open.
 *   TX: red LED          (RED)    routed to SERCOM3 PAD[0] (USART TX). In IrDA
 *       the SIR encoder idles the line LOW and emits a 3/16-bit HIGH pulse per
 *       0 bit; the optical polarity is fixed by the encoder, not by cfg->invert
 *       (which only inverts data bits; see below).
 *
 *       *** The red LED is active-LOW *** (verified on a scope: idle-low line =
 *       LED LIT, the brief HIGH pulses = LED OFF). So IrDA TX runs the LED LIT at
 *       idle with brief dark blinks, i.e. HIGH duty (~80% on), the OPPOSITE of
 *       IrDA's usual low-duty-pulse power profile. The link still works because
 *       the IrDA receiver is edge/position-based (it doesn't care about light vs
 *       dark polarity), but for the watch IrDA TX is a POWER DISADVANTAGE; it
 *       maximizes LED-on time instead of minimizing it. NRZ (with invert chosen
 *       so the line idles HIGH = LED off) is the low-LED-power TX option.
 *
 * Layered on:
 *   - gossamer's polled UART driver (256-byte FIFOs, ISR-driven internally,
 *     run_in_standby = true so the link stays alive between Movement ticks).
 *   - serial_frame_parser_t: preamble + id + len/flags + CRC32-validated
 *     framing. The CRC region is 4-byte aligned so we can use the SAML22
 *     DSU hardware CRC32 engine on TX (and the parser on RX).
 *
 * Lifecycle (strictly half-duplex; only one direction is active at a time):
 *
 *     CLOSED  -- watch_optical_open(DIR_RX, cfg) -->  RX_OPEN
 *     CLOSED  -- watch_optical_open(DIR_TX, cfg) -->  TX_OPEN
 *     {RX_OPEN, TX_OPEN}  -- watch_optical_close() -->  CLOSED
 *
 * open() fails if a session is already active in the other direction;
 * callers must close() first. The unused direction's hardware (LED pmux
 * or phototransistor bias) is always actively gated off, so the LED can
 * never reach the phototransistor while in RX, and vice versa.
 *
 * Per-tick driving:
 *   watch_optical_poll() is the ONLY function that moves bytes between
 *   hardware and the wire buffers. send(), receive(), and tx_idle() are
 *   pure queries with no side effects. Faces must call poll() once per
 *   EVENT_TICK while a session is open.
 *
 * Tick-rate constraint:
 *   The UART RX FIFO is 256 bytes. Faces must request a tick rate fast
 *   enough to drain it before it fills: at 9600 baud, >= 4 Hz minimum.
 *
 * Static memory: ~1.6 KB total (524 B TX wire buffer + ~540 B parser
 * internal + 524 B RX held-frame slot). All gated by HAS_IR_SENSOR.
 *
 * Not usable from low-energy mode; peripherals are gated off there.
 * Faces should close() before EVENT_LOW_ENERGY_UPDATE.
 */

#define WATCH_OPTICAL_MAX_PAYLOAD  SERIAL_FRAME_MAX_PAYLOAD

typedef enum {
    WATCH_OPTICAL_DIR_RX,
    WATCH_OPTICAL_DIR_TX,
} watch_optical_dir_t;

typedef struct {
    uint32_t baud;        /* baud rate, e.g. 9600 */
    bool     irda;        /* true = IrDA pulse encoding, false = NRZ */
    bool     invert;      /* CTRLA.TXINV (TX) / RXINV (RX). Per the SAM L22
                           * datasheet (31.6, CTRLA): this inverts ONLY the data
                           * bits ("Start, parity and stop bit(s) are unchanged").
                           * It is the ISO 7816 "inverse convention", NOT a line
                           * or idle-level inversion: it does NOT set the optical
                           * polarity or whether the LED idles on/off.
                           *
                           * In IrDA the SIR encoder fixes the line polarity by
                           * itself (idles LOW, each 0 bit = a 3/16-bit HIGH
                           * pulse), so this flag has no observable effect in
                           * IrDA, confirmed on a scope (toggling it does not
                           * change the optical duty). It only matters in NRZ /
                           * ISO 7816, where both ends must use the same setting.
                           * Note the red LED is active-LOW, so in IrDA the LED is
                           * LIT at idle and blinks OFF per pulse (see the header
                           * banner). Leave false for normal use. */
} watch_optical_config_t;

/* One frame's worth of received metadata + payload pointer.
 *
 * `payload` points into the library's internal held-frame buffer and remains
 * valid only until the NEXT call to watch_optical_receive() or
 * watch_optical_poll(); copy if you need to retain. */
typedef struct {
    uint16_t       id;
    uint16_t       size;
    uint8_t        flags;
    const uint8_t *payload;
} watch_optical_frame_t;

/* RX statistics. Cumulative since the most recent watch_optical_open(DIR_RX). */
typedef struct {
    uint32_t bytes_total;          /* every byte the parser was fed */
    uint32_t frames_valid;
    uint32_t frames_bad_crc;
    uint32_t frames_bad_length;
    uint32_t payload_bytes_valid;  /* sum of `size` over all valid frames */
    uint32_t frames_dropped;       /* frame completed while held slot full */
} watch_optical_rx_stats_t;

/* --- lifecycle --- */

/* Open one direction of the link. Configures the relevant SERCOM, routes
 * the pin pmux, and (for RX) powers the phototransistor. Returns false if
 * a session is already open, if cfg is NULL, or if the underlying UART
 * driver refuses the configuration. On false, no state changes. */
bool watch_optical_open(watch_optical_dir_t dir, const watch_optical_config_t *cfg);

/* Close whichever direction is open. Tears down the SERCOM, releases pmux,
 * powers down the phototransistor. Safe to call when nothing is open. */
void watch_optical_close(void);

/* --- per-tick pump --- */

/* Pump the active direction:
 *   TX: push any leftover wire bytes into the UART FIFO.
 *   RX: drain UART bytes through the serial-frame parser until either the FIFO
 *       empties or a frame lands in the held slot.
 *
 * Must be called once per EVENT_TICK while a session is open. */
void watch_optical_poll(void);

/* --- TX (only meaningful when DIR_TX is open) --- */

/* Encode one frame (`payload` of `size` bytes, with `id` written
 * verbatim into the 16-bit wire id field, low 6 bits of `flags`)
 * into the TX wire buffer and queue as many bytes as the UART FIFO will
 * accept; poll() pumps the rest.
 *
 * Returns false if not in DIR_TX, if the previous frame's wire buffer
 * hasn't fully drained yet (call poll() and try again), or if size is
 * out of [0, WATCH_OPTICAL_MAX_PAYLOAD]. `payload` may be NULL when size is
 * 0 (a header-only frame carrying only id + flags). On false, no state
 * changes. */
bool watch_optical_send(const uint8_t *payload, uint16_t size,
                        uint16_t id, uint8_t flags);

/* Queue `n` raw bytes verbatim onto the wire: no framing, no CRC, no
 * framing. poll() pumps them out exactly like watch_optical_send(), and
 * tx_idle() reports when they have fully drained.
 *
 * For tiny fixed messages where the receiver validates by exact value match
 * rather than by frame integrity, e.g. a bare 2-byte id acknowledgement in a
 * stop-and-wait exchange, where the peer only accepts the one id it is waiting
 * for, so any corruption simply fails to match and is retried.
 *
 * Returns false if not in DIR_TX, if the previous send's wire buffer hasn't
 * fully drained yet (call poll() and try again), or if n is 0 or larger than
 * the internal wire buffer (SERIAL_FRAME_MAX_SIZE). On false, no state changes. */
bool watch_optical_send_raw(const uint8_t *bytes, size_t n);

/* True when the TX wire buffer AND the UART hardware are fully drained,
 * i.e. nothing is left to transmit. Pure query (poll() does the pumping).
 * Returns true when not in DIR_TX. */
bool watch_optical_tx_idle(void);

/* --- RX (only meaningful when DIR_RX is open) --- */

/* If a validated frame is waiting in the held slot, populate `*out` and
 * return true; otherwise return false. The payload pointer in `*out`
 * remains valid only until the next call to watch_optical_receive() or
 * watch_optical_poll(). Pure query (poll() does the UART drain). */
bool watch_optical_receive(watch_optical_frame_t *out);

/* Copy the current RX statistics into `*out`. Zeroed at open(DIR_RX) time. */
void watch_optical_rx_stats(watch_optical_rx_stats_t *out);

/* True when the RX line made no forward progress during the most recent
 * poll(): the last poll fed zero bytes to the parser AND the UART FIFO is
 * empty. A single received byte makes it false again, so a caller counting
 * consecutive idle polls only needs a threshold above the inter-byte gap at
 * its baud, not the whole-frame duration. Pure query; false when not DIR_RX. */
bool watch_optical_rx_idle(void);

/* Discard any partial frame currently being parsed and resync to HUNT, so the
 * next preamble locks cleanly. Call when the caller has decided (by its own
 * idle-poll count) that the line has been quiet long enough that an in-flight
 * frame is dead (e.g. after a dropout, or while a peer waits to retransmit).
 * Does NOT discard a completed frame already in the held slot, and does NOT
 * touch rx_stats. No-op when not in DIR_RX. */
void watch_optical_rx_reset(void);

#endif /* HAS_IR_SENSOR */
