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

/* Emscripten / simulator: no real optical link. Pretend everything succeeds,
 * report an idle link, never deliver any frames. Faces built against this
 * API exercise their menus and state machines cleanly in the browser. */

bool watch_optical_open(watch_optical_dir_t dir, const watch_optical_config_t *cfg) {
    (void) dir;
    (void) cfg;
    return cfg != NULL;
}

void watch_optical_close(void) { }

void watch_optical_poll(void) { }

bool watch_optical_send(const uint8_t *payload, uint16_t size,
                        uint16_t id, uint8_t flags) {
    (void) payload;
    (void) id;
    (void) flags;
    return size <= WATCH_OPTICAL_MAX_PAYLOAD;
}

bool watch_optical_send_raw(const uint8_t *bytes, size_t n) {
    (void) bytes;
    return n > 0;
}

bool watch_optical_tx_idle(void) { return true; }

bool watch_optical_receive(watch_optical_frame_t *out) {
    (void) out;
    return false;
}

void watch_optical_rx_stats(watch_optical_rx_stats_t *out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
}

/* No real link: the line is always quiet and there's never a partial frame. */
bool watch_optical_rx_idle(void) { return true; }

void watch_optical_rx_reset(void) { }

#endif /* HAS_IR_SENSOR */
