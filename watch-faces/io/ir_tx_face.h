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

#include "movement.h"

#ifdef HAS_IR_SENSOR

/*
 * IR TX FACE
 *
 * Face to test infrared transmission using the watch onboard red LED.
 *
 * Host side: pair with `bin/test_rx.py` (it receives what this face transmits)
 * from the sensor-watch-ir-tools repo, running on an attached modem:
 *   https://github.com/alesgenova/sensor-watch-ir-tools
 * Match the host's --baud / --encoding to this face's menu settings.
 *
 */

typedef enum {
    IR_TX_MENU_BAUD = 0,
    IR_TX_MENU_ENCODING,
    IR_TX_MENU_INVERT,           /* CTRLA.TXINV: data-bit invert (ISO 7816), NOT optical polarity; no effect in IrDA */
    IR_TX_MENU_FRAME_SIZE,       /* payload bytes per frame */
    IR_TX_MENU_NUM_FRAMES,       /* number of frames to send */
    IR_TX_MENU_RATE,
    IR_TX_MENU_DELAY,
    IR_TX_MENU_START,
    IR_TX_MENU_COUNT,
} ir_tx_menu_t;

typedef enum {
    IR_TX_ENC_IRDA = 0,
    IR_TX_ENC_NRZ,
    IR_TX_ENC_COUNT,
} ir_tx_encoding_t;

typedef enum {
    IR_TX_PHASE_IDLE = 0,        /* in menu */
    IR_TX_PHASE_COUNTDOWN,       /* waiting for start delay to elapse */
    IR_TX_PHASE_TX,              /* actively pumping bytes to SERCOM3 */
    IR_TX_PHASE_DONE,            /* TX complete; brief display, then back to IDLE */
} ir_tx_phase_t;

typedef struct {
    ir_tx_menu_t menu_index;
    uint8_t baud_index;
    ir_tx_encoding_t encoding;
    bool invert;                    /* CTRLA.TXINV: data-bit invert (ISO 7816); not optical polarity, no effect in IrDA */
    uint8_t frame_size_index;       /* payload bytes per frame */
    uint8_t num_frames_index;       /* number of frames to send */
    uint8_t rate_index;
    uint8_t delay_index;

    uint8_t phase;                  /* ir_tx_phase_t */
    uint8_t countdown_remaining;    /* seconds left before TX (COUNTDOWN phase) */
    uint8_t done_seconds_remaining; /* seconds left to display DONE before returning */

    /* TX state */
    uint16_t frame_idx;             /* next frame to build (0..total_frames-1) */
    uint16_t total_frames;
    uint32_t payload_emitted;       /* total payload bytes emitted so far */

    uint8_t lfsr;                   /* PRNG state for payload byte generation */
} ir_tx_state_t;

void ir_tx_face_setup(uint8_t watch_face_index, void **context_ptr);
void ir_tx_face_activate(void *context);
bool ir_tx_face_loop(movement_event_t event, void *context);
void ir_tx_face_resign(void *context);

#define ir_tx_face ((const watch_face_t){ \
    ir_tx_face_setup, \
    ir_tx_face_activate, \
    ir_tx_face_loop, \
    ir_tx_face_resign, \
    NULL, \
})

#endif // HAS_IR_SENSOR
