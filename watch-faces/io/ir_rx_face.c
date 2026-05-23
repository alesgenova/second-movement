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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ir_rx_face.h"
#include "watch.h"
#include "watch_optical.h"

#ifdef HAS_IR_SENSOR

// During receive we request a tick frequency chosen via the RATE menu. The
// UART RX FIFO inside watch_optical is 256 bytes; the tick rate must be
// high enough to drain it before it fills. Default to the framework max
// (64 Hz), which handles up to ~9600 baud comfortably.

static const uint32_t baud_options[]      = { 50, 150, 300, 600, 900, 1200, 2400, 3600, 4800, 9600 };
static const uint8_t  tick_rate_options[] = { 1, 2, 4, 8, 16, 32, 64 };
#define BAUD_OPTION_COUNT      (sizeof(baud_options)      / sizeof(baud_options[0]))
#define TICK_RATE_OPTION_COUNT (sizeof(tick_rate_options) / sizeof(tick_rate_options[0]))
#define BAUD_DEFAULT_INDEX      7  // = 3600 baud (matches firmware_flasher_face RX baud)
#define TICK_RATE_DEFAULT_INDEX 6  // = 64 Hz, the framework max

static void render_menu(const ir_rx_state_t *state);
static void render_flash(const ir_rx_state_t *state);
static void enter_flash_mode(ir_rx_state_t *state);
static void exit_flash_mode(ir_rx_state_t *state);

// Number of consecutive idle polls after which we resync the parser to HUNT.
// A poll is "idle" only when no byte arrived since the last one, so the
// threshold must clear the worst legitimate mid-frame gap before it fires. We
// take the larger of two bounds, floored at 2:
//   * ~3 byte-times (ceil(30*tick_hz/baud)): covers a slow frame trickling in
//     at low baud, where consecutive bytes are several polls apart.
//   * ~0.5s (tick_hz/2): a floor so that at high baud, where the inter-byte
//     gap is sub-tick, a single jittery idle poll (a late tick, a stream
//     hiccup) can't resync a frame that's only mid-arrival.
// The old version floored at 1, so at high baud one stray idle poll discarded
// the in-flight frame; since rx_reset() touches no stats, the byte count still
// looked right but no frame ever validated: the "bytes arrive, no valid
// frames" flakiness. Either bound still resyncs long before a sender's next
// burst, so recovery after a genuine dropout is unaffected.
static uint16_t compute_rx_idle_limit(uint32_t baud, uint8_t tick_hz) {
    uint32_t byte_gap = (30u * tick_hz + baud - 1u) / baud;  // ceil(~3 byte-times)
    uint32_t half_sec = ((uint32_t)tick_hz + 1u) / 2u;       // ~0.5s of silence
    uint32_t limit    = byte_gap > half_sec ? byte_gap : half_sec;
    if (limit < 2u) limit = 2u;
    return (uint16_t)limit;
}

void ir_rx_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void) watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(ir_rx_state_t));
        memset(*context_ptr, 0, sizeof(ir_rx_state_t));
        ir_rx_state_t *state = (ir_rx_state_t *)*context_ptr;
        state->baud_index      = BAUD_DEFAULT_INDEX;
        state->encoding        = IR_RX_ENC_NRZ;  /* NRZ default (matches ir_tx_face / the host) */
        state->tick_rate_index = TICK_RATE_DEFAULT_INDEX;
        /* RXINV inverts only the data bits (ISO 7816), not the optical polarity,
         * and has no effect in IrDA; default OFF. See watch_optical.h. */
        state->invert          = false;
    }
}

void ir_rx_face_activate(void *context) {
    ir_rx_state_t *state = (ir_rx_state_t *)context;
    // Menu selections (baud_index, encoding, menu_index) persist across activations.
    // Flash mode is per-activation: never re-enter it on re-activate.
    state->in_flash_mode = false;
}

bool ir_rx_face_loop(movement_event_t event, void *context) {
    ir_rx_state_t *state = (ir_rx_state_t *)context;

    switch (event.event_type) {
        case EVENT_ACTIVATE:
            render_menu(state);
            break;

        case EVENT_TICK:
            if (state->in_flash_mode) {
                // Drain + parse on every tick (full polling rate) but only
                // refresh the LCD once per second (subsecond == 0). High tick
                // rates are for FIFO drain timeliness, not display rate.
                watch_optical_poll();
                // We don't care about frame contents here (this face only
                // counts), but the receive() call has to happen anyway;
                // otherwise the held-frame slot stays occupied and poll()
                // can't drain more bytes through the parser.
                watch_optical_frame_t frame;
                bool got_frame = false;
                while (watch_optical_receive(&frame)) { got_frame = true; }
                // Resync on a quiet line. If a frame was truncated (a dropped
                // byte on a finicky link), the parser would otherwise consume
                // the next frame's preamble as the dead frame's body and lose
                // it too. Once the line goes idle for rx_idle_limit polls we
                // abandon the partial frame so the next preamble locks clean.
                // Any received byte (or frame) means progress, so restart the
                // count. See enter_flash_mode for how the limit is derived.
                if (got_frame) {
                    state->rx_idle_polls = 0;
                } else if (watch_optical_rx_idle()) {
                    if (++state->rx_idle_polls >= state->rx_idle_limit) {
                        watch_optical_rx_reset();
                        state->rx_idle_polls = 0;
                    }
                } else {
                    state->rx_idle_polls = 0;
                }
                if (event.subsecond == 0) {
                    render_flash(state);
                }
            }
            break;

        case EVENT_LIGHT_BUTTON_DOWN:
            // Swallow so movement_default_loop_handler doesn't turn on the LED.
            break;

        case EVENT_LIGHT_BUTTON_UP:
            if (state->in_flash_mode) {
                // Cycle through bytes / frames / payload metrics. The choice
                // persists in state so the next flash session opens on the
                // same metric.
                state->display_metric = (state->display_metric + 1) % 3;
                render_flash(state);
            } else {
                state->menu_index = (state->menu_index + 1) % IR_RX_MENU_COUNT;
                render_menu(state);
            }
            break;

        case EVENT_ALARM_BUTTON_UP:
            if (state->in_flash_mode) {
                // Stop receive and return to the menu.
                exit_flash_mode(state);
                render_menu(state);
            } else if (state->menu_index == IR_RX_MENU_BAUD) {
                state->baud_index = (state->baud_index + 1) % BAUD_OPTION_COUNT;
                render_menu(state);
            } else if (state->menu_index == IR_RX_MENU_ENCODING) {
                state->encoding = (state->encoding + 1) % IR_RX_ENC_COUNT;
                render_menu(state);
            } else if (state->menu_index == IR_RX_MENU_INVERT) {
                state->invert = !state->invert;
                render_menu(state);
            } else if (state->menu_index == IR_RX_MENU_RATE) {
                state->tick_rate_index = (state->tick_rate_index + 1) % TICK_RATE_OPTION_COUNT;
                render_menu(state);
            } else if (state->menu_index == IR_RX_MENU_FLASH) {
                // Start receive with current baud + encoding + rate.
                enter_flash_mode(state);
                render_flash(state);
            }
            break;

        case EVENT_TIMEOUT:
            break;

        case EVENT_LOW_ENERGY_UPDATE:
            // Peripherals are gated off in low-energy mode; nothing to refresh.
            break;

        default:
            return movement_default_loop_handler(event);
    }

    // RX is opened with run_in_standby=true, so bytes keep arriving while
    // the framework sleeps between events.
    return true;
}

void ir_rx_face_resign(void *context) {
    ir_rx_state_t *state = (ir_rx_state_t *)context;
    if (state->in_flash_mode) {
        exit_flash_mode(state);
    }
}

static void render_menu(const ir_rx_state_t *state) {
    char buf[8];
    watch_clear_display();
    watch_clear_indicator(WATCH_INDICATOR_ARROWS);

    switch (state->menu_index) {
        case IR_RX_MENU_BAUD:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "Baud ", "Bd");
            snprintf(buf, sizeof(buf), "%6lu", (unsigned long)baud_options[state->baud_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_RX_MENU_ENCODING:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "EncOd", "EC");
            if (state->encoding == IR_RX_ENC_IRDA) {
                watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "  IrdA", "  IrdA");
            } else {
                watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "   nrZ", "   nrZ");
            }
            break;

        case IR_RX_MENU_INVERT:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "InV  ", "In");
            watch_display_text_with_fallback(WATCH_POSITION_BOTTOM,
                state->invert ? "    On" : "   OFF",
                state->invert ? "    On" : "   OFF");
            break;

        case IR_RX_MENU_RATE:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "rAtE ", "rt");
            snprintf(buf, sizeof(buf), "%3u HZ", (unsigned)tick_rate_options[state->tick_rate_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_RX_MENU_FLASH:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "FLASH", "FL");
            watch_display_text(WATCH_POSITION_BOTTOM, " rEAdy");
            break;

        default:
            break;
    }
}

static void render_flash(const ir_rx_state_t *state) {
    char buf[8];
    watch_clear_display();
    watch_set_indicator(WATCH_INDICATOR_ARROWS);

    watch_optical_rx_stats_t stats;
    watch_optical_rx_stats(&stats);

    // Top label + bottom value chosen via the Light button:
    //   0 = raw bytes seen by the parser (incl. noise)
    //   1 = frames that passed CRC
    //   2 = total payload bytes from those valid frames
    uint32_t value;
    const char *top_long;
    const char *top_short;
    switch (state->display_metric) {
        case 1:
            value     = stats.frames_valid;
            top_long  = "FrAMS";
            top_short = "Fr";
            break;
        case 2:
            value     = stats.payload_bytes_valid;
            top_long  = "PaYLd";
            top_short = "Pd";
            break;
        case 0:
        default:
            value     = stats.bytes_total;
            top_long  = "bYtES";
            top_short = "bY";
            break;
    }

    watch_display_text_with_fallback(WATCH_POSITION_TOP, top_long, top_short);

    if (value > 999999) value = 999999;
    snprintf(buf, sizeof(buf), "%6lu", (unsigned long)value);
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
}

static void enter_flash_mode(ir_rx_state_t *state) {
    state->in_flash_mode = true;
    // display_metric is intentionally NOT reset; it persists across flash
    // sessions so the user reopens on whatever metric they last picked via
    // the Light button.
    watch_optical_config_t cfg = {
        .baud = baud_options[state->baud_index],
        .irda = (state->encoding == IR_RX_ENC_IRDA),
        .invert = state->invert,  /* data-bit invert only (ISO 7816); no effect in IrDA */
    };
    watch_optical_open(WATCH_OPTICAL_DIR_RX, &cfg);
    movement_request_tick_frequency(tick_rate_options[state->tick_rate_index]);
    state->rx_idle_polls = 0;
    state->rx_idle_limit = compute_rx_idle_limit(baud_options[state->baud_index],
                                                 tick_rate_options[state->tick_rate_index]);
}

static void exit_flash_mode(ir_rx_state_t *state) {
    state->in_flash_mode = false;
    watch_optical_close();
    movement_request_tick_frequency(1);
}

#endif // HAS_IR_SENSOR
