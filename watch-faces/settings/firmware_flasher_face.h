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
 * FIRMWARE FLASHER FACE
 *
 * Launcher for in-place firmware flashing over the IR link.
 *
 * Host side: drive with `bin/firmware_flasher.py` (UF2 flash, full or delta)
 * from the sensor-watch-ir-tools repo, running on an attached modem:
 *   https://github.com/alesgenova/sensor-watch-ir-tools
 * Match the host's --baud / --ack-baud / --encoding to this face's menu settings.
 *
 * The flash session is bidirectional (receives firmware frames, sends
 * ACK/NAK), so RX and TX baud are configured independently. The default
 * 3600/300 split matches the ACK-based stop-and-wait protocol: fast
 * downstream firmware bytes, slow upstream acknowledgements.
 *
 * Flash mode runs in two stages:
 *
 *   1. TESTING: a link-reliability handshake driven by normal Movement
 *      ticks. The host transmits a few test frames; for each one the watch
 *      flips the (half-duplex) link to TX, sends back an ACK, and flips
 *      back to RX. The watch stays in this stage, ACKing test frames, until
 *      the host sends the ENTER frame (which moves it to FLASHING). This
 *      stage uses EVENT_TICK and the watch_optical helper module.
 *
 *   2. FLASHING: the point of no return. Entered when the host's (empty)
 *      ENTER frame is ACKed; the watch then receives the first data block and
 *      transfers control to the RAM-resident flasher, committed until reboot.
 *
 */

void firmware_flasher_face_setup(uint8_t watch_face_index, void **context_ptr);
void firmware_flasher_face_activate(void *context);
bool firmware_flasher_face_loop(movement_event_t event, void *context);
void firmware_flasher_face_resign(void *context);

#define firmware_flasher_face ((const watch_face_t){ \
    firmware_flasher_face_setup, \
    firmware_flasher_face_activate, \
    firmware_flasher_face_loop, \
    firmware_flasher_face_resign, \
    NULL, \
})

#endif // HAS_IR_SENSOR
