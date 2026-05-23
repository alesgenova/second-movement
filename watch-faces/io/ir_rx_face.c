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
#include "movement_optical.h"

#ifdef HAS_OPTICAL_LINK

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
                // Drain and discard frames (stats come from watch_optical_rx_stats);
                // the drain is what pumps RX and clears the held slot. Redraw 1/s.
                movement_optical_tick(&state->link);
                watch_optical_frame_t frame;
                while (movement_optical_next_frame(&state->link, &frame)) { /* count only */ }
                if (event.subsecond == 0) render_flash(state);
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
    // SIGNAL blinks once per second (clock parity; render_flash runs 1/s).
    if (movement_get_utc_timestamp() % 2 == 0) watch_set_indicator(WATCH_INDICATOR_SIGNAL);
    else                                       watch_clear_indicator(WATCH_INDICATOR_SIGNAL);

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
    state->in_flash_mode = true;   // display_metric persists across sessions
    // RX-only: tx_* / settle / ack are unused (we never send).
    uint32_t baud    = baud_options[state->baud_index];
    uint8_t  tick_hz = tick_rate_options[state->tick_rate_index];
    movement_optical_config_t cfg = {
        .rx_baud = baud, .tx_baud = baud,
        .irda = (state->encoding == IR_RX_ENC_IRDA),
        .tx_invert = false, .rx_invert = state->invert,
        .poll_hz = tick_hz, .tx_hz = tick_hz,
        .settle_ticks = 0, .ack_count = 1,
    };
    movement_optical_init(&state->link, &cfg);
    movement_optical_listen(&state->link);
}

static void exit_flash_mode(ir_rx_state_t *state) {
    state->in_flash_mode = false;
    movement_optical_stop(&state->link);
}

#else /* !HAS_OPTICAL_LINK */

/* Stub for boards without the IR sensor, so the face can sit in any
 * movement_config.h unconditionally: it just says so and yields. */

void ir_rx_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void) watch_face_index;
    (void) context_ptr;
}

void ir_rx_face_activate(void *context) {
    (void) context;
}

bool ir_rx_face_loop(movement_event_t event, void *context) {
    (void) context;
    if (event.event_type == EVENT_ACTIVATE) {
        watch_clear_display();
        watch_display_text_with_fallback(WATCH_POSITION_TOP, "IR", "IR");
        watch_display_text(WATCH_POSITION_BOTTOM, " no Ir");
        return true;
    }
    return movement_default_loop_handler(event);
}

void ir_rx_face_resign(void *context) {
    (void) context;
}

#endif // HAS_OPTICAL_LINK
