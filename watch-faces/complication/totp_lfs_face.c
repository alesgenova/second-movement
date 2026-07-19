/*
 * MIT License
 *
 * Copyright (c) 2022 Wesley Ellis (https://github.com/tahnok)
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

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "TOTP.h"
#include "base32.h"

#include "watch.h"
#include "filesystem.h"
#include "movement_pin_service.h"
#include "movement_optical.h"   /* IR receive; declarations gated by HAS_IR_SENSOR */

#include "totp_lfs_face.h"

#define MAX_TOTP_RECORDS 30
#define MAX_TOTP_SECRET_SIZE 128
#define TOTP_FILE "totp_uris.txt"

const char* TOTP_URI_START = "otpauth://totp/";

struct totp_record {
    char label[4];
    hmac_alg algorithm;
    uint8_t period;
    uint8_t secret_size;

    union {
        uint8_t *secret;
        struct {
            uint16_t file_secret_offset;
            uint16_t file_secret_length;
        };
    };
};

/* This is used if we're not storing all the secrets but instead
 * calculating them on demand. Avoids malloc in normal operation.
 */
static uint8_t current_secret[MAX_TOTP_SECRET_SIZE];

static struct totp_record totp_records[MAX_TOTP_RECORDS];
static uint8_t num_totp_records = 0;

static void init_totp_record(struct totp_record *totp_record) {
    totp_record->label[0] = 'A';
    totp_record->label[1] = 'A';
    totp_record->label[2] = 'A';
    totp_record->label[3] = 0;
    totp_record->algorithm = SHA1;
    totp_record->period = 30;
    totp_record->secret_size = 0;
}

static bool totp_lfs_face_read_param(struct totp_record *totp_record, char *param, char *value) {
    if (!strcmp(param, "issuer")) {
        if (value[0] == '\0') {
            printf("TOTP issuer must be a non-empty string\n");
            return false;
        }
        snprintf(totp_record->label, sizeof(totp_record->label), "%-3s", value);
    } else if (!strcmp(param, "secret")) {
        totp_record->file_secret_length = strlen(value);
        if (UNBASE32_LEN(totp_record->file_secret_length) > MAX_TOTP_SECRET_SIZE) {
            printf("TOTP secret too long: %s\n", value);
            return false;
        }
        totp_record->secret_size = base32_decode((unsigned char *)value, current_secret);
        if (totp_record->secret_size == 0) {
            printf("TOTP can't decode secret: %s\n", value);
            return false;
        }
    } else if (!strcmp(param, "digits")) {
        if (!strcmp(param, "6")) {
            printf("TOTP got %s, not 6 digits\n", value);
            return false;
        }
    } else if (!strcmp(param, "period")) {
        totp_record->period = atoi(value);
        if (totp_record->period == 0) {
            printf("TOTP invalid period %s\n", value);
            return false;
        }
    } else if (!strcmp(param, "algorithm")) {
        if (!strcmp(value, "SHA1")) {
            totp_record->algorithm = SHA1;
        }
        else if (!strcmp(value, "SHA224")) {
            totp_record->algorithm = SHA224;
        }
        else if (!strcmp(value, "SHA256")) {
            totp_record->algorithm = SHA256;
        }
        else if (!strcmp(value, "SHA384")) {
            totp_record->algorithm = SHA384;
        }
        else if (!strcmp(value, "SHA512")) {
            totp_record->algorithm = SHA512;
        }
        else {
            printf("TOTP ignored due to algorithm %s\n", value);
            return false;
        }
    }

    return true;
}

static void totp_lfs_face_read_file(char *filename) {
    // For 'format' of file, see comment at top.
    const size_t uri_start_len = strlen(TOTP_URI_START);

    if (!filesystem_file_exists(filename)) {
        printf("TOTP file error: %s\n", filename);
        return;
    }

    char line[256];
    int32_t offset = 0, old_offset = 0;
    while (old_offset = offset, filesystem_read_line(filename, line, &offset, 255) && strlen(line)) {
        if (num_totp_records == MAX_TOTP_RECORDS) {
            printf("TOTP max records: %d\n", MAX_TOTP_RECORDS);
            break;
        }

        // Check that it looks like a URI
        if (strncmp(TOTP_URI_START, line, uri_start_len)) {
            printf("TOTP invalid uri start: %s\n", line);
            continue;
        }

        // Check that we can find a '?' (to start our parameters)
        char *param;
        char *param_saveptr = NULL;
        char *params = strchr(line + uri_start_len, '?');
        if (params == NULL) {
            printf("TOTP no params: %s\n", line);
            continue;
        }

        // Process the parameters and put them in the record
        init_totp_record(&totp_records[num_totp_records]);
        bool error = false;
        param = strtok_r(params + 1, "&", &param_saveptr);
        do {
            char *param_middle = strchr(param, '=');
            *param_middle = '\0';
            if (totp_lfs_face_read_param(&totp_records[num_totp_records], param, param_middle + 1)) {
                if (!strcmp(param, "secret")) {
                    totp_records[num_totp_records].file_secret_offset = old_offset + (param_middle + 1 - line);
                }
            } else {
                error = true;
            }
        } while ((param = strtok_r(NULL, "&", &param_saveptr)));

        if (error) {
            totp_records[num_totp_records].secret_size = 0;
            continue;
        }

        // If we found a probably valid TOTP record, keep it.
        if (totp_records[num_totp_records].secret_size) {
            num_totp_records += 1;
        } else {
            printf("TOTP missing secret: %s\n", line);
        }
    }
}

void totp_lfs_face_setup(uint8_t watch_face_index, void ** context_ptr) {
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(totp_lfs_state_t));
        memset(*context_ptr, 0, sizeof(totp_lfs_state_t));
        totp_lfs_state_t *state = (totp_lfs_state_t *)*context_ptr;
        state->face_index = watch_face_index;
    }

#if !(__EMSCRIPTEN__)
    if (num_totp_records == 0) {
        totp_lfs_face_read_file(TOTP_FILE);
    }
#endif
}

static uint8_t *totp_lfs_face_get_file_secret(struct totp_record *record) {
    char buffer[BASE32_LEN(MAX_TOTP_SECRET_SIZE) + 1];
    int32_t file_secret_offset = record->file_secret_offset;

    /* TODO filesystem_read_line is quite inefficient. Consider writing a new function,
     * and keeping the file open?
     */
    if (!filesystem_read_line(TOTP_FILE, buffer, &file_secret_offset, record->file_secret_length + 1)) {
        /* Shouldn't happen at this point. Return current_secret, which is misleading but will not cause a crash. */
        printf("TOTP can't read expected secret from totp_uris.txt (failed readline)\n");
        return current_secret;
    }
    if (base32_decode((unsigned char *)buffer, current_secret) != record->secret_size) {
        printf("TOTP can't properly decode secret '%s' from totp_uris.txt; failed at offset %d; read to %ld\n", buffer, record->file_secret_offset, file_secret_offset);
    }
    return current_secret;
}

static void totp_face_set_record(totp_lfs_state_t *totp_state, int i) {
    struct totp_record *record;

    if (num_totp_records == 0 && i >= num_totp_records) {
        return;
    }

    totp_state->current_index = i;
    record = &totp_records[i];

    TOTP(
        totp_lfs_face_get_file_secret(record),
        record->secret_size,
        record->period,
        record->algorithm
    );
    totp_state->current_code = getCodeFromTimestamp(totp_state->timestamp);
    totp_state->steps = totp_state->timestamp / record->period;
}

void totp_lfs_face_activate(void *context) {
    totp_lfs_state_t *totp_state = (totp_lfs_state_t *)context;
    totp_state->steps = 0;
    totp_state->current_code = 0;

#if __EMSCRIPTEN__
    if (num_totp_records == 0) {
        // Doing this here rather than in setup makes things a bit more pleasant in the simulator, since there's no easy way to trigger
        // setup again after uploading the data.
        totp_lfs_face_read_file(TOTP_FILE);
    }
#endif

    totp_state->timestamp = movement_get_utc_timestamp();
    totp_face_set_record(totp_state, 0);
}

static void totp_face_display(totp_lfs_state_t *totp_state) {
    uint8_t index = totp_state->current_index;
    char buf[7];

    if (num_totp_records == 0) {
        watch_display_text(WATCH_POSITION_FULL, "No2F Codes");
        return;
    }

    div_t result = div(totp_state->timestamp, totp_records[index].period);
    if (result.quot != totp_state->steps) {
        totp_state->current_code = getCodeFromTimestamp(totp_state->timestamp);
        totp_state->steps = result.quot;
    }
    uint8_t valid_for = totp_records[index].period - result.rem;

    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, totp_records[index].label, totp_records[index].label);
    sprintf(buf, "%2d", valid_for);
    watch_display_text_with_fallback(WATCH_POSITION_TOP_RIGHT, buf, buf);
    sprintf(buf, "%06lu", totp_state->current_code);
    watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, buf, buf);
}

/* Delete-confirmation prompt: entry label on top, "dELETE" below. */
static void totp_face_display_delete_confirm(totp_lfs_state_t *totp_state) {
    uint8_t index = totp_state->current_index;
    watch_clear_display();
    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, totp_records[index].label, totp_records[index].label);
    watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "dELETE", " dELET");
}

/* Remove record `index`'s line from the file, located via its secret offset
 * (always inside the line). Other lines are preserved. True on success. */
static bool totp_lfs_face_delete_record(uint8_t index) {
    if (index >= num_totp_records) return false;

    int32_t size = filesystem_get_file_size(TOTP_FILE);
    if (size <= 0) return false;

    char *buf = malloc((size_t)size + 1);
    if (buf == NULL) return false;
    if (!filesystem_read_file(TOTP_FILE, buf, size)) {
        free(buf);
        return false;
    }
    buf[size] = '\0';

    // Bound the line around the secret offset; abort if it's out of range.
    int32_t marker = totp_records[index].file_secret_offset;
    if (marker < 0 || marker >= size) {
        free(buf);
        return false;
    }
    int32_t start = marker;
    while (start > 0 && buf[start - 1] != '\n') start--;
    int32_t end = marker;
    while (end < size && buf[end] != '\n') end++;
    if (end < size) end++;   // include the trailing newline

    int32_t remaining = size - end;   // splice the line out
    memmove(buf + start, buf + end, (size_t)remaining);
    int32_t new_size = start + remaining;

    bool ok;
    if (new_size == 0) {
        ok = filesystem_rm(TOTP_FILE);   // last entry removed -> drop the file
    } else {
        ok = filesystem_write_file(TOTP_FILE, buf, new_size);
    }
    free(buf);
    return ok;
}

/* Delete the selected record, reload, and re-anchor: the same index lands on the
 * next entry ("delete and advance"), clamped when the last one is removed. */
static void totp_lfs_face_delete_current(totp_lfs_state_t *totp_state) {
    if (!totp_lfs_face_delete_record(totp_state->current_index)) return;

    num_totp_records = 0;
    totp_lfs_face_read_file(TOTP_FILE);

    if (num_totp_records == 0) {
        totp_state->current_index = 0;
    } else {
        if (totp_state->current_index >= num_totp_records) {
            totp_state->current_index = num_totp_records - 1;
        }
        totp_state->timestamp = movement_get_utc_timestamp();
        totp_face_set_record(totp_state, totp_state->current_index);
    }
}

#ifdef HAS_IR_SENSOR

/* IR RECEIVE MODE: a long Alarm press listens over the optical link for serial
 * frames carrying one otpauth:// URI each, appends new ones to the TOTP file, and
 * ACKs each (bare 2-byte id echo). A repeated id is a retransmit: re-ACKed but not
 * re-added. Invalid payloads are dropped silently. Short Alarm exits. Link params
 * match the host totp-sender (2400 RX / 300 TX / NRZ). */

#define TOTP_IR_MAX_URI          255u

static const movement_optical_config_t TOTP_IR_LINK_CFG = {
    .rx_baud = 2400u, .tx_baud = 300u,
    .irda = false, .tx_invert = false, .rx_invert = false,
    .poll_hz = 8u, .tx_hz = 64u, .settle_ticks = 1u, .ack_count = 1u,
};

static movement_optical_t totp_ir_link;
static bool     totp_ir_active;
static bool     totp_ir_beep;            /* per-frame beep, toggled by Light */
static bool     totp_ir_have_last;
static uint16_t totp_ir_last_id;         /* id of the last frame added (dedup) */
static uint16_t totp_ir_added;

/* Parse `line` as an otpauth TOTP URI; true if it yields a usable secret. Mutates
 * `line` (strtok) and clobbers current_secret, like the file reader. */
static bool totp_lfs_face_validate_uri(char *line) {
    const size_t uri_start_len = strlen(TOTP_URI_START);
    if (strncmp(TOTP_URI_START, line, uri_start_len)) return false;

    char *params = strchr(line + uri_start_len, '?');
    if (params == NULL) return false;

    struct totp_record record;
    init_totp_record(&record);

    bool error = false;
    char *param_saveptr = NULL;
    char *param = strtok_r(params + 1, "&", &param_saveptr);
    if (param == NULL) return false;
    do {
        char *param_middle = strchr(param, '=');
        if (param_middle == NULL) { error = true; break; }
        *param_middle = '\0';
        if (!totp_lfs_face_read_param(&record, param, param_middle + 1)) {
            error = true;
            break;
        }
    } while ((param = strtok_r(NULL, "&", &param_saveptr)));

    if (error) return false;
    return record.secret_size != 0;
}

/* Append one URI line to the file. Records are reparsed once, on exit. */
static void totp_lfs_face_append_uri(const char *uri, uint16_t len) {
    char line[TOTP_IR_MAX_URI + 2];
    memcpy(line, uri, len);
    line[len] = '\n';
    filesystem_append_file(TOTP_FILE, line, len + 1);
}

/* SIGNAL blinks once per second (clock parity) as a receiving heartbeat. */
static void totp_ir_apply_signal_blink(void) {
    if (movement_get_utc_timestamp() % 2 == 0) watch_set_indicator(WATCH_INDICATOR_SIGNAL);
    else                                       watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
}

static void totp_ir_render(void) {
    char buf[7];
    watch_clear_display();
    totp_ir_apply_signal_blink();
    if (totp_ir_beep) watch_set_indicator(WATCH_INDICATOR_BELL);   // solid = beep on
    else              watch_clear_indicator(WATCH_INDICATOR_BELL);
    watch_display_text_with_fallback(WATCH_POSITION_TOP, "IR rc", "IR");
    snprintf(buf, sizeof(buf), "%6u", (unsigned)totp_ir_added);
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
}

static void totp_ir_enter(void) {
    totp_ir_have_last = false;
    totp_ir_last_id   = 0;
    totp_ir_added     = 0;
    totp_ir_active    = true;
    totp_ir_beep      = true;
    movement_optical_init(&totp_ir_link, &TOTP_IR_LINK_CFG);
    movement_optical_listen(&totp_ir_link);
    totp_ir_render();
}

static void totp_ir_exit(void) {
    movement_optical_stop(&totp_ir_link);
    watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
    watch_clear_indicator(WATCH_INDICATOR_BELL);
    totp_ir_active = false;
    if (totp_ir_added) {   // reparse the file once, now the appends are done
        num_totp_records = 0;
        totp_lfs_face_read_file(TOTP_FILE);
    }
}

/* Validate the frame's URI, appending if new. True = ACK it (valid, new or a
 * retransmit); false = malformed, drop without ACK. */
static bool totp_ir_accept_frame(const watch_optical_frame_t *frame) {
    if (frame->size == 0 || frame->size > TOTP_IR_MAX_URI) return false;

    char uri[TOTP_IR_MAX_URI + 1];
    memcpy(uri, frame->payload, frame->size);
    uri[frame->size] = '\0';

    char scratch[TOTP_IR_MAX_URI + 1];   // validation mutates; keep uri intact
    memcpy(scratch, uri, (size_t)frame->size + 1);
    if (!totp_lfs_face_validate_uri(scratch)) return false;

    bool duplicate = totp_ir_have_last && frame->id == totp_ir_last_id;
    if (!duplicate) {
        totp_lfs_face_append_uri(uri, frame->size);
        totp_ir_have_last = true;
        totp_ir_last_id   = frame->id;
        totp_ir_added++;
    }
    return true;
}

static void totp_ir_tick(void) {
    if (movement_optical_tick(&totp_ir_link) != MOVEMENT_OPTICAL_FRAMES) return;

    watch_optical_frame_t f;
    while (movement_optical_next_frame(&totp_ir_link, &f)) {
        if (totp_ir_beep) movement_play_note(BUZZER_NOTE_C6, 15);
        if (totp_ir_accept_frame(&f)) {
            movement_optical_send_ack(&totp_ir_link, f.id);
            totp_ir_render();
            break;   // one ACK per turn; the send flips us out of listening
        }
    }
}

#endif /* HAS_IR_SENSOR */

bool totp_lfs_face_loop(movement_event_t event, void *context) {

    totp_lfs_state_t *totp_state = (totp_lfs_state_t *)context;

    if (movement_pin_service_is_locked()) {
        return movement_pin_service_loop(event, totp_state->face_index, "totp", "2f");
    }

#ifdef HAS_IR_SENSOR
    // Receive mode runs its own loop. Face-leaving events fall through to the
    // default handler; the resulting resign tears down the link. Only the two
    // paths that skip resign tear down inline: the in-face Alarm exit and
    // low-energy (movement doesn't resign before deep sleep).
    if (totp_ir_active) {
        switch (event.event_type) {
            case EVENT_TICK:
                if (event.subsecond == 0) totp_ir_apply_signal_blink();
                totp_ir_tick();
                break;
            case EVENT_ALARM_BUTTON_UP:   // exit receive, stay in the face
                totp_ir_exit();
                totp_state->timestamp = movement_get_utc_timestamp();
                if (num_totp_records) totp_face_set_record(totp_state, totp_state->current_index);
                totp_face_display(totp_state);
                break;
            case EVENT_LIGHT_BUTTON_UP:    // toggle the per-frame beep
                totp_ir_beep = !totp_ir_beep;
                totp_ir_render();
                break;
            case EVENT_LIGHT_BUTTON_DOWN:  // swallow: don't let the default light the LED
                break;
            case EVENT_LOW_ENERGY_UPDATE:  // no resign before sleep: tear down here
                totp_ir_exit();
                return movement_default_loop_handler(event);
            default:
                return movement_default_loop_handler(event);
        }
        return true;
    }
#endif

    // Delete-confirmation prompt: Light cancels, Alarm confirms, else inert.
    if (totp_state->confirming_delete) {
        switch (event.event_type) {
            case EVENT_TICK:
                totp_state->timestamp++;
                break;
            case EVENT_LIGHT_BUTTON_UP:   // cancel
                totp_state->confirming_delete = false;
                totp_face_display(totp_state);
                break;
            case EVENT_ALARM_BUTTON_UP:   // confirm
                totp_state->confirming_delete = false;
                totp_lfs_face_delete_current(totp_state);
                totp_face_display(totp_state);
                break;
            case EVENT_TIMEOUT:
                movement_move_to_page(0);   // resign clears confirming_delete
                break;
            case EVENT_LIGHT_BUTTON_DOWN:   // swallow: no LED during the prompt
                break;
            case EVENT_LOW_ENERGY_UPDATE:   // no resign before sleep
                totp_state->confirming_delete = false;
                return movement_default_loop_handler(event);
            default:
                return movement_default_loop_handler(event);
        }
        return true;
    }

    switch (event.event_type) {
        case EVENT_TICK:
            totp_state->timestamp++;
            totp_face_display(totp_state);
            break;
        case EVENT_ACTIVATE:
            totp_face_display(totp_state);
            break;
        case EVENT_TIMEOUT:
            movement_move_to_page(0);
            break;
        case EVENT_ALARM_BUTTON_UP:
            if (num_totp_records) {
                totp_face_set_record(totp_state, (totp_state->current_index + 1) % num_totp_records);
                totp_face_display(totp_state);
            }
            break;
        case EVENT_ALARM_LONG_PRESS:
#ifdef HAS_IR_SENSOR
            // Long Alarm press enters IR receive mode to import TOTP entries.
            totp_ir_enter();
#endif
            break;
        case EVENT_LIGHT_BUTTON_UP:
            if (num_totp_records) {
                totp_face_set_record(totp_state, (totp_state->current_index + num_totp_records - 1) % num_totp_records);
                totp_face_display(totp_state);
            }
            break;
        case EVENT_ALARM_BUTTON_DOWN:
        case EVENT_LIGHT_BUTTON_DOWN:
            break;
        case EVENT_LIGHT_LONG_PRESS:
            movement_illuminate_led();
            break;
        case EVENT_LIGHT_REALLY_LONG_PRESS:
            // Really-long Light press on a selected entry asks to delete it.
            if (num_totp_records) {
                totp_state->confirming_delete = true;
                totp_face_display_delete_confirm(totp_state);
            }
            break;
        default:
            movement_default_loop_handler(event);
            break;
    }

    return true;
}

void totp_lfs_face_resign(void *context) {
    totp_lfs_state_t *totp_state = (totp_lfs_state_t *)context;
    totp_state->confirming_delete = false;
#ifdef HAS_IR_SENSOR
    if (totp_ir_active) totp_ir_exit();
#endif
}
