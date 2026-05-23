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
 * IR RX FACE
 *
 * Face to test infrared reception using the watch onboard photo-transistor.
 *
 * Host side: pair with `bin/test_tx.py` (it transmits frames for this face to
 * receive) from the sensor-watch-ir-tools repo, running on an attached modem:
 *   https://github.com/alesgenova/sensor-watch-ir-tools
 * Match the host's --baud / --encoding to this face's menu settings.
 */

typedef enum {
    IR_RX_MENU_BAUD = 0,
    IR_RX_MENU_ENCODING,
    IR_RX_MENU_INVERT,           /* CTRLA.RXINV: data-bit invert (ISO 7816), NOT optical polarity; no effect in IrDA */
    IR_RX_MENU_RATE,
    IR_RX_MENU_FLASH,
    IR_RX_MENU_COUNT,
} ir_rx_menu_t;

typedef enum {
    IR_RX_ENC_IRDA = 0,
    IR_RX_ENC_NRZ,
    IR_RX_ENC_COUNT,
} ir_rx_encoding_t;

typedef struct {
    ir_rx_menu_t menu_index;
    uint8_t baud_index;
    ir_rx_encoding_t encoding;
    bool invert;                   /* CTRLA.RXINV: data-bit invert (ISO 7816); not optical polarity, no effect in IrDA */
    uint8_t tick_rate_index;
    bool in_flash_mode;
    uint8_t display_metric;        /* 0=bytes, 1=frames, 2=payload bytes;
                                      cycled with Light button during flash
                                      mode, persists across sessions */
    uint16_t rx_idle_polls;        /* consecutive polls with a quiet RX line */
    uint16_t rx_idle_limit;        /* reset the parser after this many idle
                                      polls; derived from baud + tick rate at
                                      flash-mode entry */
} ir_rx_state_t;

void ir_rx_face_setup(uint8_t watch_face_index, void **context_ptr);
void ir_rx_face_activate(void *context);
bool ir_rx_face_loop(movement_event_t event, void *context);
void ir_rx_face_resign(void *context);

#define ir_rx_face ((const watch_face_t){ \
    ir_rx_face_setup, \
    ir_rx_face_activate, \
    ir_rx_face_loop, \
    ir_rx_face_resign, \
    NULL, \
})

#endif // HAS_IR_SENSOR
