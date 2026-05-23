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
#include "ir_tx_face.h"
#include "watch.h"
#include "watch_optical.h"

#ifdef HAS_IR_SENSOR

#define TX_DONE_DISPLAY_SECONDS   2

/* Application-defined frame flags. The serial_frame transport assigns no meaning
 * to the 6 flag bits; this demo uses two of them to mark message boundaries. */
#define IR_TX_FLAG_LAST_FRAME   (1u << 1)
#define IR_TX_FLAG_FIRST_FRAME  (1u << 2)

/* One chunk's worth of LFSR-generated payload, regenerated per frame.
 * Static so it doesn't bloat the per-face state struct. */
static uint8_t payload_buf[WATCH_OPTICAL_MAX_PAYLOAD];

static const uint32_t baud_options[]    = { 50, 150, 300, 600, 900, 1200, 2400, 3600, 4800, 9600 };
static const uint16_t frame_size_options[] = { 8, 16, 32, 64, 128 };
static const uint16_t num_frames_options[] = { 2, 4, 8, 16, 32 };
static const uint8_t  rate_options[]    = { 1, 2, 4, 8, 16, 32, 64 };
static const uint8_t  delay_options[]   = { 0, 1, 2, 5, 10 };

#define BAUD_OPTION_COUNT        (sizeof(baud_options)        / sizeof(baud_options[0]))
#define FRAME_SIZE_OPTION_COUNT  (sizeof(frame_size_options)  / sizeof(frame_size_options[0]))
#define NUM_FRAMES_OPTION_COUNT  (sizeof(num_frames_options)  / sizeof(num_frames_options[0]))
#define RATE_OPTION_COUNT        (sizeof(rate_options)  / sizeof(rate_options[0]))
#define DELAY_OPTION_COUNT       (sizeof(delay_options) / sizeof(delay_options[0]))

static void render_menu(const ir_tx_state_t *state);
static void render_countdown(const ir_tx_state_t *state);
static void render_tx(const ir_tx_state_t *state);
static void render_done(const ir_tx_state_t *state);
static void enter_countdown(ir_tx_state_t *state);
static void enter_tx(ir_tx_state_t *state);
static void exit_tx(ir_tx_state_t *state);
static void try_send_next_frame(ir_tx_state_t *state);

void ir_tx_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void) watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(ir_tx_state_t));
        memset(*context_ptr, 0, sizeof(ir_tx_state_t));
        ir_tx_state_t *state = (ir_tx_state_t *)*context_ptr;
        state->baud_index       = 2;  /* 300 baud (matches firmware_flasher_face TX baud) */
        state->encoding         = IR_TX_ENC_NRZ;  /* NRZ default (IrDA keeps the LED lit ~80%; see watch_optical.h) */
        state->frame_size_index = 0;  /* 8 payload bytes per frame */
        state->num_frames_index = 1;  /* 4 frames */
        state->rate_index = RATE_OPTION_COUNT - 1;  /* default 64 Hz */
        /* TXINV inverts only the data bits (ISO 7816), not the optical polarity,
         * and has no effect in IrDA; default OFF. See watch_optical.h. */
        state->invert     = false;
    }
}

void ir_tx_face_activate(void *context) {
    ir_tx_state_t *state = (ir_tx_state_t *)context;
    /* Menu selections persist across activations; only the per-session
     * transmission phase resets. */
    state->phase = IR_TX_PHASE_IDLE;
}

bool ir_tx_face_loop(movement_event_t event, void *context) {
    ir_tx_state_t *state = (ir_tx_state_t *)context;

    switch (event.event_type) {
        case EVENT_ACTIVATE:
            render_menu(state);
            break;

        case EVENT_TICK:
            if (state->phase == IR_TX_PHASE_COUNTDOWN) {
                if (event.subsecond == 0) {
                    if (state->countdown_remaining > 0) {
                        state->countdown_remaining--;
                    }
                    if (state->countdown_remaining == 0) {
                        enter_tx(state);
                        render_tx(state);
                    } else {
                        render_countdown(state);
                    }
                }
            } else if (state->phase == IR_TX_PHASE_TX) {
                /* Pump bytes at the full tick rate (FIFO drain is the limit),
                 * try to queue the next frame, refresh the display once per
                 * second. */
                watch_optical_poll();
                try_send_next_frame(state);
                if (event.subsecond == 0) render_tx(state);
                if (state->frame_idx >= state->total_frames
                    && watch_optical_tx_idle()) {
                    /* All frames queued and the line is fully idle. */
                    exit_tx(state);
                    state->phase = IR_TX_PHASE_DONE;
                    state->done_seconds_remaining = TX_DONE_DISPLAY_SECONDS;
                    render_done(state);
                }
            } else if (state->phase == IR_TX_PHASE_DONE) {
                if (event.subsecond == 0) {
                    if (state->done_seconds_remaining > 0) {
                        state->done_seconds_remaining--;
                    }
                    if (state->done_seconds_remaining == 0) {
                        state->phase = IR_TX_PHASE_IDLE;
                        movement_request_tick_frequency(1);
                        render_menu(state);
                    }
                }
            }
            break;

        case EVENT_LIGHT_BUTTON_DOWN:
            /* Swallow so movement_default_loop_handler doesn't turn on the LED. */
            break;

        case EVENT_LIGHT_BUTTON_UP:
            if (state->phase == IR_TX_PHASE_IDLE) {
                state->menu_index = (state->menu_index + 1) % IR_TX_MENU_COUNT;
                render_menu(state);
            }
            break;

        case EVENT_ALARM_BUTTON_UP:
            if (state->phase == IR_TX_PHASE_TX || state->phase == IR_TX_PHASE_COUNTDOWN) {
                /* Abort. Tear down SERCOM if it was up, return to menu. */
                if (state->phase == IR_TX_PHASE_TX) exit_tx(state);
                state->phase = IR_TX_PHASE_IDLE;
                movement_request_tick_frequency(1);
                render_menu(state);
            } else if (state->phase == IR_TX_PHASE_DONE) {
                /* Skip the DONE display, return to menu immediately. */
                state->phase = IR_TX_PHASE_IDLE;
                movement_request_tick_frequency(1);
                render_menu(state);
            } else {
                /* IDLE: act on the current menu entry. */
                switch (state->menu_index) {
                    case IR_TX_MENU_BAUD:
                        state->baud_index = (state->baud_index + 1) % BAUD_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_TX_MENU_ENCODING:
                        state->encoding = (state->encoding + 1) % IR_TX_ENC_COUNT;
                        render_menu(state);
                        break;
                    case IR_TX_MENU_INVERT:
                        state->invert = !state->invert;
                        render_menu(state);
                        break;
                    case IR_TX_MENU_FRAME_SIZE:
                        state->frame_size_index = (state->frame_size_index + 1) % FRAME_SIZE_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_TX_MENU_NUM_FRAMES:
                        state->num_frames_index = (state->num_frames_index + 1) % NUM_FRAMES_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_TX_MENU_RATE:
                        state->rate_index = (state->rate_index + 1) % RATE_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_TX_MENU_DELAY:
                        state->delay_index = (state->delay_index + 1) % DELAY_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_TX_MENU_START:
                        enter_countdown(state);
                        if (state->phase == IR_TX_PHASE_TX) render_tx(state);
                        else                                render_countdown(state);
                        break;
                    default:
                        break;
                }
            }
            break;

        case EVENT_TIMEOUT:
            break;

        case EVENT_LOW_ENERGY_UPDATE:
            break;

        default:
            return movement_default_loop_handler(event);
    }

    return true;
}

void ir_tx_face_resign(void *context) {
    ir_tx_state_t *state = (ir_tx_state_t *)context;
    if (state->phase == IR_TX_PHASE_TX) {
        exit_tx(state);
    }
    movement_request_tick_frequency(1);
}

/* --- Menu rendering ---------------------------------------------------- */

static void render_menu(const ir_tx_state_t *state) {
    char buf[8];
    watch_clear_display();
    watch_clear_indicator(WATCH_INDICATOR_ARROWS);

    switch (state->menu_index) {
        case IR_TX_MENU_BAUD:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "Baud ", "Bd");
            snprintf(buf, sizeof(buf), "%6lu", (unsigned long)baud_options[state->baud_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case IR_TX_MENU_ENCODING:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "EncOd", "EC");
            if (state->encoding == IR_TX_ENC_IRDA)
                watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "  IrdA", "  IrdA");
            else
                watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "   nrZ", "   nrZ");
            break;
        case IR_TX_MENU_INVERT:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "InV  ", "In");
            watch_display_text_with_fallback(WATCH_POSITION_BOTTOM,
                state->invert ? "    On" : "   OFF",
                state->invert ? "    On" : "   OFF");
            break;
        case IR_TX_MENU_FRAME_SIZE:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "FrSiZ", "FS");
            snprintf(buf, sizeof(buf), "%5ub", frame_size_options[state->frame_size_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case IR_TX_MENU_NUM_FRAMES:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "nFrMS", "nF");
            snprintf(buf, sizeof(buf), "%6u", num_frames_options[state->num_frames_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case IR_TX_MENU_RATE:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "rAtE ", "rt");
            snprintf(buf, sizeof(buf), "%3uHZ", rate_options[state->rate_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case IR_TX_MENU_DELAY:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "dELAY", "dL");
            snprintf(buf, sizeof(buf), "%4us ", delay_options[state->delay_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        case IR_TX_MENU_START:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "Start", "St");
            watch_display_text(WATCH_POSITION_BOTTOM, " rEAdy");
            break;
        default:
            break;
    }
}

static void render_countdown(const ir_tx_state_t *state) {
    char buf[8];
    watch_clear_display();
    watch_set_indicator(WATCH_INDICATOR_ARROWS);
    watch_display_text_with_fallback(WATCH_POSITION_TOP, "Start", "St");
    snprintf(buf, sizeof(buf), " GO %2u", state->countdown_remaining);
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
}

static void render_tx(const ir_tx_state_t *state) {
    char buf[8];
    watch_clear_display();
    watch_set_indicator(WATCH_INDICATOR_ARROWS);
    watch_display_text_with_fallback(WATCH_POSITION_TOP, "Tx   ", "tx");
    /* Show payload-bytes-emitted (capped to 6 digits). */
    uint32_t v = state->payload_emitted;
    if (v > 999999) v = 999999;
    snprintf(buf, sizeof(buf), "%6lu", (unsigned long)v);
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
}

static void render_done(const ir_tx_state_t *state) {
    (void) state;
    char buf[8];
    watch_clear_display();
    watch_clear_indicator(WATCH_INDICATOR_ARROWS);
    watch_display_text_with_fallback(WATCH_POSITION_TOP, "Start", "St");
    snprintf(buf, sizeof(buf), "%6s", " donE");
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
}

/* --- Phase transitions ------------------------------------------------- */

static void enter_countdown(ir_tx_state_t *state) {
    uint8_t delay = delay_options[state->delay_index];
    state->countdown_remaining = delay;
    if (delay == 0) {
        /* Skip countdown entirely. */
        enter_tx(state);
    } else {
        state->phase = IR_TX_PHASE_COUNTDOWN;
        movement_request_tick_frequency(rate_options[state->rate_index]);
    }
}

static void enter_tx(ir_tx_state_t *state) {
    state->phase           = IR_TX_PHASE_TX;
    state->frame_idx       = 0;
    state->payload_emitted = 0;
    state->lfsr            = 0x42;

    /* Number of frames to send is configured directly. */
    state->total_frames = num_frames_options[state->num_frames_index];

    watch_optical_config_t cfg = {
        .baud = baud_options[state->baud_index],
        .irda = (state->encoding == IR_TX_ENC_IRDA),
        .invert = state->invert,   /* data-bit invert only (ISO 7816); no effect in IrDA */
    };
    watch_optical_open(WATCH_OPTICAL_DIR_TX, &cfg);

    movement_request_tick_frequency(rate_options[state->rate_index]);
}

static void exit_tx(ir_tx_state_t *state) {
    (void) state;
    watch_optical_close();
}

/* --- Frame construction ----------------------------------------------- */

static uint8_t lfsr_next(ir_tx_state_t *state) {
    uint8_t lsb = state->lfsr & 1;
    state->lfsr >>= 1;
    if (lsb) state->lfsr ^= 0xB8;  /* Galois LFSR, taps 0xB8 */
    return state->lfsr;
}

static void try_send_next_frame(ir_tx_state_t *state) {
    if (state->frame_idx >= state->total_frames) return;

    /* Every frame carries the same configured payload size. */
    uint16_t declared = frame_size_options[state->frame_size_index];

    /* Flags: FIRST on frame 0, LAST on the final frame. */
    uint8_t flags = 0;
    if (state->frame_idx == 0)                        flags |= IR_TX_FLAG_FIRST_FRAME;
    if (state->frame_idx == state->total_frames - 1)  flags |= IR_TX_FLAG_LAST_FRAME;

    /* Snapshot LFSR so we can rewind if the send is rejected; otherwise a
     * busy-rejected attempt would silently consume PRNG bytes that never
     * reach the wire and the receiver's LFSR check would diverge. */
    uint8_t lfsr_snap = state->lfsr;
    for (uint16_t i = 0; i < declared; i++) {
        payload_buf[i] = lfsr_next(state);
    }
    if (watch_optical_send(payload_buf, declared, state->frame_idx, flags)) {
        state->frame_idx++;
        state->payload_emitted += declared;
    } else {
        state->lfsr = lfsr_snap;
    }
}

#endif // HAS_IR_SENSOR
