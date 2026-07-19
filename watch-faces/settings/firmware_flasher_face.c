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
/* newlib heap-break query, used to check an aux window fits BEFORE malloc (its
 * _sbrk has no limit check, so an oversized malloc corrupts the heap instead of
 * failing). Declared here to avoid pulling <unistd.h>; only _sbrk is linked. */
extern void *_sbrk(int incr);
#include "firmware_flasher_face.h"
#include "watch.h"
#include "movement_optical.h"
#include "serial_frame.h"
#include "firmware_flasher_core.h"

#ifdef HAS_IR_SENSOR

/* =====================================================================
 *  FLASHING PROTOCOL: HIGH-LEVEL OVERVIEW
 *
 *  Half-duplex IR link, host -> watch data, watch -> host ACKs. Every
 *  frame is a serial_frame {preamble, id, len/flags, payload,
 *  CRC32}; an ACK is just the frame's 2-byte id echoed back (no framing,
 *  no CRC, repeated N times for redundancy on the weak return path). The
 *  host blocks waiting for the id it expects, so a corrupted ACK simply
 *  fails to match and the host retransmits. The flag bits (TEST / ENTER /
 *  VERIFY / PATCH) are defined below.
 *
 *  The session runs as a sequence of stages. The golden rule for the data
 *  stages is STOP-AND-WAIT: the watch ACKs a frame ONLY after it has fully
 *  acted on it (written the row / consumed the patch bytes / verified the
 *  image). So an ACK the host receives is proof of durable progress, and a
 *  lost ACK just means the host resends a frame the watch already handled.
 *
 *  1. TEST  (frame flag = TEST)
 *     A link-reliability warm-up. For each TEST frame the watch flips to
 *     TX, ACKs it, flips back to RX, and STAYS in this stage. Nothing is
 *     committed. It measures whether the round-trip is good enough to
 *     flash, and it stays here until an ENTER frame arrives.
 *
 *  2. ENTER  (flag = ENTER; the point we choose full vs. patch)
 *     - Plain ENTER (empty payload): a FULL (uf2-like) flash. ACK + advance.
 *     - Patch ENTER (ENTER|PATCH|VERIFY, 20-byte reference descriptor): a
 *       DELTA flash. The watch CRCs its CURRENT flash over the reference
 *       range and ACKs ONLY if it matches: proof the patch targets the
 *       right base image. On mismatch it does NOTHING (no ACK); the host
 *       retries or gives up. A repeated ENTER (our ACK was lost) is just
 *       re-ACKed; re-verifying is harmless because nothing has been
 *       written yet. ENTER is the last reversible moment.
 *
 *  3. FIRST DATA BLOCK  ->  POINT OF NO RETURN
 *     Once ENTER is ACKed the watch waits for the first data block. It does
 *     NOT ACK that block from here; it copies it aside and hands control
 *     to the RAM-resident flasher (arm_and_run), which reconfigures the
 *     link raw and never returns. From here the firmware is being
 *     overwritten; the only exits are a verified image (reboot) or staying
 *     parked awaiting a retry.
 *
 *  4. DATA STREAM  (in the RAM flasher)
 *     Both kinds obey the same stop-and-wait policy:
 *         * next-in-sequence  -> act, ACK, advance
 *         * duplicate of last -> re-ACK only (our ACK was lost), no re-act
 *         * out-of-sequence / wrong kind -> do NOTHING (no ACK)
 *         * act (write) failed -> do NOTHING (no ACK) so the host resends
 *     - FULL: the main receive loop acts on each block {addr, 256-byte row}:
 *       erase+write the row, then ACK (id 0 also restarts a full flash from
 *       the top).
 *     - PATCH: the compressed delta body streams in; the decoder PULLS its
 *       bytes from the frame stream (patch_pull_byte), and a frame is ACKed
 *       the moment its last byte is consumed -- a pull decoder only asks for
 *       more input after applying what it read, so the ACK still certifies
 *       durable progress. The same dup/order rules apply inside the source.
 *     Either way a NON-patch id-0 block arriving mid-PATCH is a takeover: the
 *     watch abandons the half-finished patch and restarts as a full flash from
 *     block 0 (the only way to restart a patch, which applies in place).
 *
 *  5. EXIT / VERIFY  (flag = VERIFY, payload = 12-byte image descriptor)
 *     The terminator. The watch DSU-CRCs the freshly written region against
 *     the descriptor. Match -> ACK (echo id) and reboot into the new image.
 *     Mismatch -> do NOTHING (no ACK): the host prompts the user to resend,
 *     and the watch stays in the flasher rather than rebooting into a brick.
 *
 *  So, per frame, the watch does exactly one of three things: ACT-then-ACK
 *  (commit a block / verify the image), ACK-only (TEST frame, or a
 *  duplicate/repeat whose work is already done), or NOTHING (a reference
 *  mismatch, an out-of-sequence or wrong-kind frame, or a failed write).
 *  Silence is the signal that makes the host retransmit.
 * ===================================================================== */

/* ===================================================================== *
 *  PRIVATE PROTOCOL, GEOMETRY, AND TYPES                                *
 *                                                                       *
 *  None of this is part of the face's public interface, so it lives     *
 *  here rather than in the header; movement_faces.h only ever sees the  *
 *  face entry points. The host flasher tool mirrors the wire values by  *
 *  hand; keep them in lockstep.                                         *
 * ===================================================================== */

/* Geometry, wire constants, and the patch descriptor type are shared with
 * the RAM-resident core and the decoder backends; they live in
 * firmware_flasher_core.h. */

/* ---- Host-detectable beacon ------------------------------------------- *
 *
 * The host flasher scans the assembled image for this before flashing: its
 * absence means the image cannot receive IR flashes (flashing it would be
 * the LAST IR flash the watch accepts), and its patch_flag tells the host
 * which patch dialect the build speaks, so --patch-format is auto-selected
 * from the reference image. It lives inside the real (HAS_IR_SENSOR) face
 * on purpose: a non-IR build must FAIL the check. The magic (8 bytes,
 * including the NUL) is the eternal search anchor and never changes;
 * beacon_version gates the layout of what follows.
 *
 * The face classifies patch frames through beacon.patch_flag (below), so the
 * beacon is genuinely referenced -- the linker keeps it exactly when the
 * functioning face is present, and it cannot disagree with the watch's
 * behavior. The RAM core keeps the IR_FLASHER_FLAG_PATCH compile-time
 * constant instead: overlay code must not read flash-resident data mid-erase
 * (both come from the same macro, so they agree by construction). */
typedef struct __attribute__((packed)) {
    char    magic[8];        /* "SWFLSHR" + NUL: eternal search anchor */
    uint8_t beacon_version;  /* layout version of the fields below (1) */
    uint8_t patch_flag;      /* IR_FLASHER_FLAG_PATCH: this build's patch dialect bit */
} firmware_flasher_beacon_t;

static const firmware_flasher_beacon_t firmware_flasher_beacon = {
    .magic = "SWFLSHR", .beacon_version = 1, .patch_flag = IR_FLASHER_FLAG_PATCH,
};

/* Read the dialect through a volatile view: a plain read of a static const
 * field constant-folds to an immediate, the reference disappears, and
 * --gc-sections silently drops the beacon from the image (empirically
 * verified). The volatile access cannot be folded, so this is what actually
 * keeps the beacon linked. */
static inline uint8_t firmware_flasher_patch_dialect(void) {
    return ((volatile const firmware_flasher_beacon_t *)&firmware_flasher_beacon)->patch_flag;
}


/* ---- Launcher state -------------------------------------------------- */

typedef enum {
    IR_FLASHER_MENU_RX_BAUD = 0,
    IR_FLASHER_MENU_TX_BAUD,
    IR_FLASHER_MENU_ENCODING,
    IR_FLASHER_MENU_TX_INVERT,   /* CTRLA.TXINV: data-bit invert (ISO 7816), NOT optical polarity; no effect in IrDA */
    IR_FLASHER_MENU_RX_INVERT,   /* CTRLA.RXINV: data-bit invert (ISO 7816), NOT optical polarity; no effect in IrDA */
    IR_FLASHER_MENU_POLL_RATE,
    IR_FLASHER_MENU_ACK_RATE,    /* tick rate used while settling + draining the ACK */
    IR_FLASHER_MENU_ACK_SETTLE,  /* ACK-rate ticks to wait before sending the ACK */
    IR_FLASHER_MENU_ACK_COUNT,   /* how many times to repeat the id in each ACK (1-4) */
    IR_FLASHER_MENU_FLASH,
    IR_FLASHER_MENU_COUNT,
} firmware_flasher_menu_t;

typedef enum {
    IR_FLASHER_ENC_IRDA = 0,
    IR_FLASHER_ENC_NRZ,
    IR_FLASHER_ENC_COUNT,
} firmware_flasher_encoding_t;

typedef enum {
    IR_FLASHER_PHASE_MENU = 0,      /* navigating the parameter menu */
    /* Link-active app phases. The comms sub-states (listening, settling, ACK
     * draining) live in the movement_optical; these are only the flasher's own
     * stages. pending_ack_advances marks that the ACK now draining was for an
     * ENTER frame, so the link's TX_DONE advances TEST -> WAIT_BLOCK. */
    IR_FLASHER_PHASE_TEST,          /* ACK test frames (stay); ENTER -> ACK + advance */
    IR_FLASHER_PHASE_WAIT_BLOCK,    /* ENTER ACKed; re-ACK a repeated ENTER, or hand off the first block */
} firmware_flasher_phase_t;

typedef struct {
    firmware_flasher_menu_t menu_index;
    bool settings_unlocked;      /* false = only the flash page is shown; toggled by a Light long-press */
    uint8_t rx_baud_index;
    uint8_t tx_baud_index;
    firmware_flasher_encoding_t encoding;
    bool tx_invert;              /* CTRLA.TXINV: data-bit invert (ISO 7816); not optical polarity, no effect in IrDA */
    bool rx_invert;              /* CTRLA.RXINV: data-bit invert (ISO 7816); not optical polarity, no effect in IrDA */
    uint8_t poll_rate_index;
    uint8_t ack_rate_index;      /* tick rate for the ACK settle + drain */
    uint8_t ack_settle_index;    /* number of ACK-rate ticks to wait before ACK */
    uint8_t ack_count;           /* times to repeat the id per ACK (1..4) */
    bool beep_on_frame;          /* play a short beep on each test-phase frame; toggled by Light */

    /* Flash-mode runtime state (valid while phase != MENU). */
    firmware_flasher_phase_t phase;
    movement_optical_t link;    /* optical link session (direction flip, settle, ACK, idle-resync) */
    uint16_t test_frames_acked;   /* test frames round-tripped so far */
    uint16_t last_frame_id;       /* id of the most recent frame received (shown in the test view) */
    bool     pending_ack_advances;/* the ACK now draining is for an ENTER: advance to WAIT_BLOCK on TX_DONE */

    /* Real-flash stage (point-of-no-return hand-off into the RAM flasher). The
     * whole-image descriptor is NOT held here; it travels in the EXIT frame and
     * is parsed by the RAM flasher; the launcher only needs the first block. */
    bool     session_is_patch;    /* the accepted ENTER was a VERIFIED patch ENTER:
                                     this authorizes the flasher to apply patch blocks */
    /* Patch header fields parsed + stored from the patch ENTER (valid while
     * session_is_patch). Acted on only at first-block time (aux malloc + handoff). */
    uint32_t patch_base;          /* app base of the images */
    uint32_t patch_from_size;     /* REF extent */
    uint32_t patch_to_size;       /* NEW extent */
    uint32_t patch_shift_size;    /* detools shift_size = aux window size */
    uint16_t first_block_id;      /* id of the first data block (ACKed by the flasher) */
    uint32_t first_block_addr;    /* its target address */
    uint8_t  first_block[FLASHER_ROW_SIZE] __attribute__((aligned(4)));  /* its 256-byte row */
} firmware_flasher_state_t;

/* ---------------------------------------------------------------------- *
 *  The face code below hands off to the RAM-resident flasher core (its own
 *  translation unit; entry points declared in firmware_flasher_core.h).
 *  Only the arming hand-off itself is defined further down, and it is
 *  hardware-only.
 * ---------------------------------------------------------------------- */
#ifndef __EMSCRIPTEN__
#ifdef FIRMWARE_FLASHER_ULTRAPATCH
#include "firmware_flasher_ultrapatch.h"   /* state-size probe for the aux malloc */
#endif
static bool     firmware_flasher_overlay_load(void);
static void     firmware_flasher_arm_and_run(uint32_t rx_baud, uint32_t tx_baud, bool irda,
                                             bool tx_invert, bool rx_invert,
                                             uint8_t poll_hz, uint8_t ack_hz, uint8_t ack_settle_ticks,
                                             uint8_t ack_count,
                                             const firmware_flasher_patch_t *patch,
                                             uint16_t first_block_id, uint32_t first_block_addr,
                                             bool first_block_is_patch, uint16_t first_block_len,
                                             const uint8_t *first_block);
#endif

/* ===================================================================== *
 *  Launcher face                                                        *
 * ===================================================================== */

static const uint32_t baud_options[]      = { 50, 150, 300, 600, 900, 1200, 2400, 3600, 4200, 4800, 9600 };
static const uint8_t  poll_rate_options[] = { 1, 2, 4, 8, 16, 32, 64 };
// ACK-rate ticks to wait before transmitting the ACK, giving the peer receiver's
// phototransistor time to recover from the IR self-saturation our frame caused.
static const uint8_t  ack_settle_options[] = { 0, 1, 2, 4, 8, 16, 32 };
#define BAUD_OPTION_COUNT       (sizeof(baud_options)       / sizeof(baud_options[0]))
#define POLL_RATE_OPTION_COUNT  (sizeof(poll_rate_options)  / sizeof(poll_rate_options[0]))
#define ACK_SETTLE_OPTION_COUNT (sizeof(ack_settle_options) / sizeof(ack_settle_options[0]))

// Index of each option's default value in its lookup table.
// rx 3600 / tx 300 / NRZ, RX 8 Hz, ACK 64 Hz, 4-tick ACK settle. NRZ over IrDA
// because IrDA keeps the active-low LED lit ~80% of the time (see watch_optical.h).
#define RX_BAUD_DEFAULT_INDEX    7   // = 3600 baud
#define TX_BAUD_DEFAULT_INDEX    2   // =  300 baud
#define POLL_RATE_DEFAULT_INDEX  3   // =    8 Hz
#define ACK_RATE_DEFAULT_INDEX   (POLL_RATE_OPTION_COUNT - 1)  // = 64 Hz
#define ACK_SETTLE_DEFAULT_INDEX 3   // = 4 ticks

static void render_menu(const firmware_flasher_state_t *state);
static void render_test(const firmware_flasher_state_t *state);
static void render_flash_wait(const firmware_flasher_state_t *state);
static void render_signal_heartbeat(void);

static void enter_test(firmware_flasher_state_t *state);
static void abort_flash_mode(firmware_flasher_state_t *state);
static bool handle_test_frame(firmware_flasher_state_t *state, const watch_optical_frame_t *frame);
static void handle_enter_frame(firmware_flasher_state_t *state, const watch_optical_frame_t *frame);
static bool enter_patch_setup(firmware_flasher_state_t *state, const watch_optical_frame_t *frame);

/* Any phase with the optical link open / a session in progress (test handshake,
 * or ENTERed and waiting for the first block): used to tear down cleanly on
 * resign / low-energy, and as the Alarm-aborts-everything guard. */
static bool is_link_active(const firmware_flasher_state_t *state) {
    return state->phase == IR_FLASHER_PHASE_TEST ||
           state->phase == IR_FLASHER_PHASE_WAIT_BLOCK;
}

// Reset every link parameter to the known-good default found by hardware
// testing (see RX_BAUD_DEFAULT_INDEX et al). Used at first setup and again when
// the settings are re-locked from the menu.
static void set_defaults(firmware_flasher_state_t *state) {
    state->rx_baud_index    = RX_BAUD_DEFAULT_INDEX;
    state->tx_baud_index    = TX_BAUD_DEFAULT_INDEX;
    state->encoding         = IR_FLASHER_ENC_NRZ;
    state->poll_rate_index  = POLL_RATE_DEFAULT_INDEX;
    state->ack_rate_index   = ACK_RATE_DEFAULT_INDEX;
    state->ack_settle_index = ACK_SETTLE_DEFAULT_INDEX;
    state->ack_count        = IR_FLASHER_ACK_COUNT_MIN;   // 1 = no redundancy
    // Data-bit invert (ISO 7816), not optical polarity; no effect in IrDA. Default
    // OFF: correct for IrDA, and for NRZ both ends just need to agree.
    state->tx_invert        = false;
    state->rx_invert        = false;
}

void firmware_flasher_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void) watch_face_index;
    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(firmware_flasher_state_t));
        memset(*context_ptr, 0, sizeof(firmware_flasher_state_t));
        firmware_flasher_state_t *state = (firmware_flasher_state_t *)*context_ptr;
        set_defaults(state);
        // Settings start locked: only the flash page is shown until the user
        // long-presses Light to reveal the parameter pages.
        state->settings_unlocked = false;
        state->menu_index        = IR_FLASHER_MENU_FLASH;
        state->beep_on_frame     = true;   // audible per-frame feedback during the test phase
    }
}

void firmware_flasher_face_activate(void *context) {
    firmware_flasher_state_t *state = (firmware_flasher_state_t *)context;
    // Menu selections persist across activations; flash mode is always
    // (re)entered from the menu, never resumed on activate.
    state->phase = IR_FLASHER_PHASE_MENU;
}

bool firmware_flasher_face_loop(movement_event_t event, void *context) {
    firmware_flasher_state_t *state = (firmware_flasher_state_t *)context;

    switch (event.event_type) {
        case EVENT_ACTIVATE:
            render_menu(state);
            break;

        case EVENT_TICK:
            if (is_link_active(state)) {
                if (event.subsecond == 0) render_signal_heartbeat();   // once/sec link-alive blink
                switch (movement_optical_tick(&state->link)) {
                    case MOVEMENT_OPTICAL_FRAMES: {
                        watch_optical_frame_t frame;
                        while (movement_optical_next_frame(&state->link, &frame)) {
                            if (handle_test_frame(state, &frame)) break;
                        }
                        break;
                    }
                    case MOVEMENT_OPTICAL_TX_DONE:
                        // ACK drained. An ENTER ACK advances to wait-for-block; a
                        // test-frame ACK is just counted.
                        if (state->pending_ack_advances) {
                            state->pending_ack_advances = false;
                            state->phase = IR_FLASHER_PHASE_WAIT_BLOCK;
                            render_flash_wait(state);
                        } else {
                            state->test_frames_acked++;
                            render_test(state);
                        }
                        break;
                    default:
                        break;
                }
            }
            break;

        case EVENT_LIGHT_BUTTON_DOWN:
            // Swallow so movement_default_loop_handler doesn't turn on the LED.
            break;

        case EVENT_LIGHT_BUTTON_UP:
            // In the menu, Light cycles the pages (only once settings are
            // unlocked; while locked the flash page is the only page). During the
            // test phase, Light toggles the per-frame beep. Otherwise inert.
            if (state->phase == IR_FLASHER_PHASE_MENU) {
                if (state->settings_unlocked) {
                    state->menu_index = (state->menu_index + 1) % IR_FLASHER_MENU_COUNT;
                    render_menu(state);
                }
            } else if (state->phase == IR_FLASHER_PHASE_TEST) {
                state->beep_on_frame = !state->beep_on_frame;
                render_test(state);
            }
            break;

        case EVENT_LIGHT_LONG_PRESS:
            // A long Light press toggles the settings lock, but only from the
            // menu (the link must not be active). Unlocking reveals the parameter
            // pages and jumps to the first (baud) page; re-locking resets every
            // option to its default and returns to the flash page.
            if (state->phase == IR_FLASHER_PHASE_MENU) {
                if (!state->settings_unlocked) {
                    state->settings_unlocked = true;
                    state->menu_index = IR_FLASHER_MENU_RX_BAUD;
                } else {
                    set_defaults(state);
                    state->settings_unlocked = false;
                    state->menu_index = IR_FLASHER_MENU_FLASH;
                }
                render_menu(state);
            }
            break;

        case EVENT_ALARM_BUTTON_UP:
            if (is_link_active(state)) {
                // Abort any in-progress session (the test stage, or after ENTER
                // while waiting for the first block) and return to the menu. The
                // host's ENTER frame (not a button) starts the real flash, so
                // Alarm here is purely the abort/back-out, valid right up until the
                // first block hands off to the RAM flasher.
                abort_flash_mode(state);
                render_menu(state);
            } else {
                // Menu: act on the current entry.
                switch (state->menu_index) {
                    case IR_FLASHER_MENU_RX_BAUD:
                        state->rx_baud_index = (state->rx_baud_index + 1) % BAUD_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_FLASHER_MENU_TX_BAUD:
                        state->tx_baud_index = (state->tx_baud_index + 1) % BAUD_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_FLASHER_MENU_ENCODING:
                        state->encoding = (state->encoding + 1) % IR_FLASHER_ENC_COUNT;
                        render_menu(state);
                        break;
                    case IR_FLASHER_MENU_TX_INVERT:
                        state->tx_invert = !state->tx_invert;
                        render_menu(state);
                        break;
                    case IR_FLASHER_MENU_RX_INVERT:
                        state->rx_invert = !state->rx_invert;
                        render_menu(state);
                        break;
                    case IR_FLASHER_MENU_POLL_RATE:
                        state->poll_rate_index = (state->poll_rate_index + 1) % POLL_RATE_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_FLASHER_MENU_ACK_RATE:
                        state->ack_rate_index = (state->ack_rate_index + 1) % POLL_RATE_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_FLASHER_MENU_ACK_SETTLE:
                        state->ack_settle_index = (state->ack_settle_index + 1) % ACK_SETTLE_OPTION_COUNT;
                        render_menu(state);
                        break;
                    case IR_FLASHER_MENU_ACK_COUNT:
                        state->ack_count = (state->ack_count >= IR_FLASHER_ACK_COUNT_MAX)
                                         ? IR_FLASHER_ACK_COUNT_MIN
                                         : (uint8_t)(state->ack_count + 1);
                        render_menu(state);
                        break;
                    case IR_FLASHER_MENU_FLASH:
                        enter_test(state);
                        render_test(state);
                        break;
                    default:
                        break;
                }
            }
            break;

        case EVENT_TIMEOUT:
            break;

        case EVENT_LOW_ENERGY_UPDATE:
            // Peripherals are gated off in low-energy mode; tear down any open
            // session so we don't leave the link half-configured.
            if (is_link_active(state)) {
                abort_flash_mode(state);
                render_menu(state);
            }
            break;

        default:
            return movement_default_loop_handler(event);
    }

    // RX/TX are opened with run_in_standby=true, so the link stays alive while
    // the framework sleeps between ticks.
    return true;
}

void firmware_flasher_face_resign(void *context) {
    firmware_flasher_state_t *state = (firmware_flasher_state_t *)context;
    if (is_link_active(state)) {
        abort_flash_mode(state);
    }
}

/* --- Phase transitions ------------------------------------------------- */

// Link config from the menu (ACK rate -> tx_hz, ACK settle -> settle_ticks).
static movement_optical_config_t make_link_config(const firmware_flasher_state_t *state) {
    movement_optical_config_t cfg = {
        .rx_baud      = baud_options[state->rx_baud_index],
        .tx_baud      = baud_options[state->tx_baud_index],
        .irda         = (state->encoding == IR_FLASHER_ENC_IRDA),
        .tx_invert    = state->tx_invert,
        .rx_invert    = state->rx_invert,
        .poll_hz      = poll_rate_options[state->poll_rate_index],
        .tx_hz        = poll_rate_options[state->ack_rate_index],
        .settle_ticks = ack_settle_options[state->ack_settle_index],
        .ack_count    = state->ack_count,
    };
    return cfg;
}

// Open the link in RX and stay in TEST (ACKing test frames) until ENTER arrives.
static void enter_test(firmware_flasher_state_t *state) {
    state->test_frames_acked    = 0;
    state->session_is_patch     = false;   // set true only by a verified patch ENTER
    state->pending_ack_advances = false;
    movement_optical_config_t cfg = make_link_config(state);
    movement_optical_init(&state->link, &cfg);
    movement_optical_listen(&state->link);
    state->phase = IR_FLASHER_PHASE_TEST;
}

static void abort_flash_mode(firmware_flasher_state_t *state) {
    movement_optical_stop(&state->link);
    state->phase = IR_FLASHER_PHASE_MENU;
}

/* Dispatch one received frame in the TEST / WAIT_BLOCK stages (from the
 * next_frame drain loop). Returns true when it responded (an ACK ends the turn)
 * or handed off to the RAM flasher; false for an unrelated frame (keep draining). */
static bool handle_test_frame(firmware_flasher_state_t *state,
                              const watch_optical_frame_t *frame) {
    state->last_frame_id = frame->id;

    if (state->phase == IR_FLASHER_PHASE_TEST) {
        // Per-frame beep (Light toggles). Async; safe on receipt since the buzzer
        // teardown no longer disturbs the RED pin (see watch_tcc.c).
        if (state->beep_on_frame) movement_play_note(BUZZER_NOTE_C6, 15);
        if (frame->flags & IR_FLASHER_FLAG_TEST) {   // ACK + stay in TEST
            movement_optical_send_ack(&state->link, frame->id);
            state->pending_ack_advances = false;
            render_test(state);
            return true;
        }
        if (frame->flags & IR_FLASHER_FLAG_ENTER) {
            handle_enter_frame(state, frame);   // ACKs (advances) iff valid
            return true;
        }
        return false;
    }

    // WAIT_BLOCK: a repeated ENTER is re-ACKed; the first data block is NOT ACKed
    // but copied aside and handed to the RAM flasher (which never returns).
    bool first_is_patch = (frame->flags & firmware_flasher_patch_dialect()) != 0;
    if (frame->flags & IR_FLASHER_FLAG_ENTER) {
        handle_enter_frame(state, frame);
        return true;
    }
    if (state->session_is_patch && first_is_patch &&
        frame->size > 0 && frame->size <= FLASHER_ROW_SIZE) {
        // First patch body chunk (patch mode authorized by the verified ENTER).
#ifndef __EMSCRIPTEN__
        // POINT OF NO RETURN: load + verify the RAM overlay (flasher core +
        // decoder) FIRST -- the state-size probe below already runs overlay
        // code. Refusal here is the loader's ~impossible fail-safes only.
        if (!firmware_flasher_overlay_load()) return true;   // swallowed, no ACK
#ifdef FIRMWARE_FLASHER_ULTRAPATCH
        // `aux` carries the UltraPatch decoder state (PatchApply), not a source
        // window. CRITICAL: probe the break first — newlib's _sbrk has no limit
        // check, so an oversized malloc corrupts the heap and still returns
        // non-NULL; and the overlay base is a hard ceiling (an allocation past
        // it would overlap the just-loaded flasher). If it can't be allocated,
        // still arm: the backend refuses to run without state, the session
        // parks, and the host recovers with a block-0 full flash over IR --
        // no refusal state, Movement is already forfeit.
        uint32_t need = firmware_flasher_ultrapatch_state_size();
        uint8_t *aux = NULL;
        uintptr_t brk = (uintptr_t)_sbrk(0);  // current heap top
        if (brk >= 0x20000000u && brk < FLASHER_OVERLAY_BASE &&
            (uint32_t)(FLASHER_OVERLAY_BASE - brk) >= need + 512u /*allocator margin*/) {
            aux = malloc(need);   // guaranteed to fit
        }
        firmware_flasher_patch_t pd = {
            .aux = aux, .aux_mask = 0u,
#else
        // Aux window = smallest pow2 >= shift_size (indexed by mask in the RAM
        // core); NULL => in-flash shift fallback. CRITICAL: don't just malloc —
        // newlib's _sbrk has no limit check, so an oversized request corrupts
        // the heap and still returns non-NULL. Probe the break and require it
        // to fit below the overlay base (the heap ceiling; the loaded flasher
        // sits right above it).
        uint32_t pow2 = 1u;
        while (pow2 < state->patch_shift_size) pow2 <<= 1;
        uint8_t *aux = NULL;
        uintptr_t brk = (uintptr_t)_sbrk(0);  // current heap top
        if (brk >= 0x20000000u && brk < FLASHER_OVERLAY_BASE &&
            (uint32_t)(FLASHER_OVERLAY_BASE - brk) >= pow2 + 512u /*allocator margin*/) {
            aux = malloc(pow2);   // guaranteed to fit
        }
        firmware_flasher_patch_t pd = {
            .aux = aux, .aux_mask = aux ? (pow2 - 1u) : 0u,
#endif
            .base = state->patch_base, .from_size = state->patch_from_size,
            .to_size = state->patch_to_size, .shift_size = state->patch_shift_size,
        };
        state->first_block_id = frame->id;
        memcpy(state->first_block, frame->payload, frame->size);
        // POINT OF NO RETURN: never returns; the RAM flasher closes the link.
        firmware_flasher_arm_and_run(
            baud_options[state->rx_baud_index],
            baud_options[state->tx_baud_index],
            state->encoding == IR_FLASHER_ENC_IRDA,
            state->tx_invert, state->rx_invert,
            poll_rate_options[state->poll_rate_index],
            poll_rate_options[state->ack_rate_index],
            ack_settle_options[state->ack_settle_index],
            state->ack_count,
            &pd, frame->id, 0u, true, frame->size, state->first_block);
#endif
        return true;   // unreachable on hardware; patch never reaches here on sim
    }
    if (!first_is_patch && frame->size == IR_FLASHER_BLOCK_PAYLOAD) {
        // First uf2-like block (also the patch -> uf2 block-0 fallback).
        state->first_block_id   = frame->id;
        state->first_block_addr = (uint32_t)frame->payload[0]
                                | ((uint32_t)frame->payload[1] << 8)
                                | ((uint32_t)frame->payload[2] << 16)
                                | ((uint32_t)frame->payload[3] << 24);
        memcpy(state->first_block, frame->payload + 4, FLASHER_ROW_SIZE);
#ifndef __EMSCRIPTEN__
        // POINT OF NO RETURN: load + verify the RAM overlay (refusal is the
        // loader's ~impossible fail-safes only), then arm. Never returns.
        if (!firmware_flasher_overlay_load()) return true;
        // patch = NULL => uf2 session.
        firmware_flasher_arm_and_run(
            baud_options[state->rx_baud_index],
            baud_options[state->tx_baud_index],
            state->encoding == IR_FLASHER_ENC_IRDA,
            state->tx_invert, state->rx_invert,
            poll_rate_options[state->poll_rate_index],
            poll_rate_options[state->ack_rate_index],
            ack_settle_options[state->ack_settle_index],
            state->ack_count,
            NULL, state->first_block_id, state->first_block_addr,
            false, FLASHER_ROW_SIZE, state->first_block);
#endif
        abort_flash_mode(state);   // sim only (no flashing); return to the menu
        render_menu(state);
        return true;
    }
    return false;
}

/* Parse + validate a patch ENTER's 20-byte header {base, from_size, ref_crc32,
 * to_size, shift_size}, LE (IR_FLASHER_PATCH_ENTER_SIZE). Returns true (and
 * stores the apply params in `state`) iff the watch's CURRENT flash over
 * [base, base+from_size) CRCs to ref_crc32, i.e. the patch applies to the right
 * base image. The read-back CRC needs the DSU, so this is hardware-only; on the
 * simulator (flashing unsupported) it always refuses. Nothing is acted on here
 * beyond reading flash; the aux allocation / apply happen at first-block time. */
static bool enter_patch_setup(firmware_flasher_state_t *state,
                              const watch_optical_frame_t *frame) {
#ifndef __EMSCRIPTEN__
    if (frame->size != IR_FLASHER_PATCH_ENTER_SIZE) return false;
    const uint8_t *p = frame->payload;
    uint32_t base  = (uint32_t)p[0]  | ((uint32_t)p[1]  << 8) | ((uint32_t)p[2]  << 16) | ((uint32_t)p[3]  << 24);
    uint32_t from  = (uint32_t)p[4]  | ((uint32_t)p[5]  << 8) | ((uint32_t)p[6]  << 16) | ((uint32_t)p[7]  << 24);
    uint32_t ref   = (uint32_t)p[8]  | ((uint32_t)p[9]  << 8) | ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);
    uint32_t to    = (uint32_t)p[12] | ((uint32_t)p[13] << 8) | ((uint32_t)p[14] << 16) | ((uint32_t)p[15] << 24);
    uint32_t shift = (uint32_t)p[16] | ((uint32_t)p[17] << 8) | ((uint32_t)p[18] << 16) | ((uint32_t)p[19] << 24);
#ifdef FIRMWARE_FLASHER_ULTRAPATCH
    /* (The dialect gate happened upstream: only an ENTER carrying OUR patch
     * bit reaches here.) The decoder's image window is compile-time, so base
     * must equal it exactly; shift_size is not a concept in this format and
     * ships as 0. from/to stay row-aligned (the host pads both images), which
     * also satisfies the DSU. */
    if (base != FLASHER_ULTRAPATCH_IMAGE_BASE) return false;
    if (from  == 0 || (from  % FLASHER_ROW_SIZE) != 0) return false;   /* row-aligned (host pads) */
    if (to    == 0 || (to    % FLASHER_ROW_SIZE) != 0) return false;
    if (shift != 0) return false;
    if (!firmware_flasher_range_writable(base, from)) return false;    /* REF region in app flash */
    if (!firmware_flasher_range_writable(base, to))   return false;    /* NEW region in app flash */
#else
    if ((base & 3u) != 0) return false;                                /* DSU needs 4-aligned addr */
    if (from  == 0 || (from  % FLASHER_ROW_SIZE) != 0) return false;   /* row-aligned (host pads) */
    if (to    == 0 || (to    % FLASHER_ROW_SIZE) != 0) return false;
    if (shift == 0 || (shift % FLASHER_ROW_SIZE) != 0) return false;
    if (!firmware_flasher_range_writable(base, from)) return false;    /* REF region in app flash */
    if (!firmware_flasher_range_writable(base, to))   return false;    /* NEW region in app flash */
    /* The shifted REF must also fit: base+shift+from <= app-flash end. This both
     * bounds the aux window and guarantees the in-flash shift fallback (which
     * relocates REF up to the region end) has room, i.e. shift <= max shift. */
    if (!firmware_flasher_range_writable(base + shift, from)) return false;
#endif
    /* Pre-flight reference CRC via the flash-resident serial_frame DSU code:
     * the RAM core (and its own CRC routine) has not been loaded yet -- the
     * overlay is only copied in at first-block time. Same engine, same
     * convention. */
    if (serial_frame_crc32((const uint8_t *)base, from) != ref) return false;
    state->patch_base       = base;
    state->patch_from_size  = from;
    state->patch_to_size    = to;
    state->patch_shift_size = shift;
    return true;
#else
    (void) state; (void) frame;
    return false;
#endif
}

/* ACK an ENTER and advance to the wait-for-block stage. A patch ENTER is ACKed
 * only if its header verifies (enter_patch_setup); a mismatch is dropped, no ACK.
 * A verified patch ENTER — not any block flag — is what enables patch mode. */
static void handle_enter_frame(firmware_flasher_state_t *state,
                               const watch_optical_frame_t *frame) {
    bool is_patch = (frame->flags & firmware_flasher_patch_dialect()) != 0;
    /* An ENTER carrying VERIFY or a payload is a patch ENTER in SOME dialect
     * (a full-flash ENTER is empty and bare); if it is not OURS, silence --
     * never ACK it as a full-flash ENTER. */
    if (!is_patch && ((frame->flags & IR_FLASHER_FLAG_VERIFY) || frame->size != 0)) return;
    if (is_patch && !enter_patch_setup(state, frame)) return;
    state->session_is_patch = is_patch;
    movement_optical_send_ack(&state->link, frame->id);
    state->pending_ack_advances = true;
}

/* --- Rendering --------------------------------------------------------- */

static void render_menu(const firmware_flasher_state_t *state) {
    char buf[8];
    watch_clear_display();   // also clears every indicator (SIGNAL/BELL/ARROWS)
    watch_clear_indicator(WATCH_INDICATOR_ARROWS);

    switch (state->menu_index) {
        case IR_FLASHER_MENU_RX_BAUD:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "rbAUd", "rb");
            snprintf(buf, sizeof(buf), "%6lu", (unsigned long)baud_options[state->rx_baud_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_FLASHER_MENU_TX_BAUD:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "tbAUd", "tb");
            snprintf(buf, sizeof(buf), "%6lu", (unsigned long)baud_options[state->tx_baud_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_FLASHER_MENU_ENCODING:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "EncOd", "EC");
            if (state->encoding == IR_FLASHER_ENC_IRDA) {
                watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "  IrdA", "  IrdA");
            } else {
                watch_display_text_with_fallback(WATCH_POSITION_BOTTOM, "   nrZ", "   nrZ");
            }
            break;

        case IR_FLASHER_MENU_TX_INVERT:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "tInV ", "tI");
            watch_display_text_with_fallback(WATCH_POSITION_BOTTOM,
                state->tx_invert ? "    On" : "   OFF",
                state->tx_invert ? "    On" : "   OFF");
            break;

        case IR_FLASHER_MENU_RX_INVERT:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "rInV ", "rI");
            watch_display_text_with_fallback(WATCH_POSITION_BOTTOM,
                state->rx_invert ? "    On" : "   OFF",
                state->rx_invert ? "    On" : "   OFF");
            break;

        case IR_FLASHER_MENU_POLL_RATE:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "PoLL ", "Po");
            snprintf(buf, sizeof(buf), "%3u HZ", (unsigned)poll_rate_options[state->poll_rate_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_FLASHER_MENU_ACK_RATE:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "AcrAt", "Ar");
            snprintf(buf, sizeof(buf), "%3u HZ", (unsigned)poll_rate_options[state->ack_rate_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_FLASHER_MENU_ACK_SETTLE:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "SEtLE", "SE");
            snprintf(buf, sizeof(buf), "%5ut", (unsigned)ack_settle_options[state->ack_settle_index]);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_FLASHER_MENU_ACK_COUNT:
            watch_display_text_with_fallback(WATCH_POSITION_TOP, "Acnt ", "An");
            snprintf(buf, sizeof(buf), "%5ux", (unsigned)state->ack_count);
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;

        case IR_FLASHER_MENU_FLASH:
            watch_display_text(WATCH_POSITION_TOP, "IR");
            watch_display_text(WATCH_POSITION_BOTTOM, " FlASH");
            break;

        default:
            break;
    }
}

// SIGNAL blinks once per second (clock parity) as a link-alive heartbeat.
static void render_signal_heartbeat(void) {
    if (movement_get_utc_timestamp() % 2 == 0) watch_set_indicator(WATCH_INDICATOR_SIGNAL);
    else                                       watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
}

static void render_test(const firmware_flasher_state_t *state) {
    char buf[8];
    watch_clear_display();
    watch_set_indicator(WATCH_INDICATOR_ARROWS);
    render_signal_heartbeat();
    if (state->beep_on_frame) watch_set_indicator(WATCH_INDICATOR_BELL);   // solid = beep on
    // "tESt" (custom) / "FLAS" (classic) + last-received id.
    watch_display_text_with_fallback(WATCH_POSITION_TOP, "tESt ", "FLAS");
    snprintf(buf, sizeof(buf), "%6u", (unsigned)state->last_frame_id);
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
}

static void render_flash_wait(const firmware_flasher_state_t *state) {
    (void) state;
    watch_clear_display();
    watch_set_indicator(WATCH_INDICATOR_ARROWS);
    render_signal_heartbeat();
    // ENTER received; waiting for the first data block.
    watch_display_text_with_fallback(WATCH_POSITION_TOP, "WA1T", "WA1T");
}
/* ===================================================================== *
 *  The RAM-resident flasher core, the frame parser, and the patch       *
 *  decoder backends live in their own fully-RAM translation units:      *
 *  firmware_flasher_core.c and firmware_flasher_detools.c /             *
 *  firmware_flasher_ultrapatch.c (see firmware_flasher_core.h for the   *
 *  contract). Only the flash-resident arming hand-off remains here.     *
 * ===================================================================== */
#ifndef __EMSCRIPTEN__

#include "sam.h"
#include "uart2.h"

/* watch_optical_close() (from watch_optical.h, included above) tears down the
 * Movement-side optical session before the arming step reconfigures the link
 * raw. */

/* ---- Overlay loader (flash-resident; runs AT the point of no return) ----
 *
 * The RAM flasher (core + decoder backend) is not boot-resident: its load
 * image sits in flash and is copied here, at first-block time, to the fixed
 * VMA window placed by flasher-overlay.ld near the top of RAM.
 *
 * Receiving the first data block after a verified ENTER IS the point of no
 * return: the session is committed, Movement is forfeit, and every recovery
 * path from here on runs over the IR link inside the flasher (park, block-0
 * full-flash takeover, EXIT verify) -- never by returning to Movement. So
 * the copy proceeds even though the window overlaps what is normally heap
 * headroom: any heap contents above the overlay base are dead weight now,
 * and if state the hand-off still reads from the heap got smashed (possible
 * only if Movement's heap had grown past the overlay base, ~12 KiB), the
 * resulting garbage fails frame CRCs or the decode and the session parks --
 * still recoverable over IR.
 *
 * Only two ~impossible conditions refuse (swallow the block, no ACK),
 * because PROCEEDING would be strictly worse than the host retrying:
 *   - the live stack pointer not clearing the overlay end: the copy would
 *     overwrite the very frames executing it and crash mid-copy;
 *   - the post-copy byte-for-byte verify failing: arming a corrupted
 *     flash-writing engine could endanger the bootloader.
 *
 * The copy also plants a canary word just past the overlay; the core checks
 * it after the decoder backend runs, so a session stack that dug into the
 * loaded code parks the session instead of trusting its output (the EXIT
 * CRC remains the final gate either way). */
static bool firmware_flasher_overlay_load(void) {
    uint32_t n = (uint32_t)(__flovl_vma_end - __flovl_vma_start);
    if (__get_MSP() < (uint32_t)(uintptr_t)__flovl_vma_end + 1024u /*session stack*/)
        return false;
    memcpy(__flovl_vma_start, __flovl_lma_start, n);
    if (memcmp(__flovl_vma_start, __flovl_lma_start, n) != 0) return false;
    *flasher_overlay_canary_addr() = FLASHER_OVERLAY_CANARY;
    return true;
}

/* ---- Arming hand-off (flash-resident; runs once before any erase) ---- *
 *
 * THE POINT OF NO RETURN. Arms the system and runs the RAM-resident flasher;
 * NEVER RETURNS (it reboots via NVIC_SystemReset on a verified image, or stays
 * in its loop awaiting a retry; control reaches Movement again only across a
 * reset). The launcher must already have, over the proven watch_optical
 * transport, received + ACKed the (empty) ENTER frame and received the first
 * data block (id/addr/data) WITHOUT ACKing it; `first_block` points at
 * FLASHER_ROW_SIZE bytes in RAM. This call configures the raw SERCOM link
 * (rx_baud/tx_baud/irda, tx_invert/rx_invert), masks interrupts, parks the RTC
 * + NVM cache, selects STANDBY, and hands control to flasher_run.
 *
 * `patch` is NULL for a uf2-like session, else the assembled patch descriptor:
 * the verified ENTER's authority for patch mode, NOT any block flag. poll_hz /
 * ack_hz / ack_settle_ticks are the launcher's TEST-stage link timings, from
 * which the per-ACK settle is derived (see below). */
static void firmware_flasher_arm_and_run(uint32_t rx_baud, uint32_t tx_baud, bool irda,
                                  bool tx_invert, bool rx_invert,
                                  uint8_t poll_hz, uint8_t ack_hz, uint8_t ack_settle_ticks,
                                  uint8_t ack_count,
                                  const firmware_flasher_patch_t *patch,
                                  uint16_t first_block_id, uint32_t first_block_addr,
                                  bool first_block_is_patch, uint16_t first_block_len,
                                  const uint8_t *first_block) {
    flasher_ack_count = (ack_count < IR_FLASHER_ACK_COUNT_MIN) ? IR_FLASHER_ACK_COUNT_MIN
                : (ack_count > IR_FLASHER_ACK_COUNT_MAX) ? IR_FLASHER_ACK_COUNT_MAX
                : ack_count;
    /* Per-ACK settle, in RTC ticks = MIDDLE of the TEST stage's effective-settle
     * window expressed directly in ticks. The window in seconds is
     * [settle/ack_hz, settle/ack_hz + 1/poll_hz]; its midpoint x 128 Hz is:
     *     ack_settle_ticks*128/ack_hz  +  (128/2)/poll_hz
     * i.e. the configured settle plus half the poll-latency window that the TEST
     * stage incurred but the flasher (no poll latency) does not. */
    flasher_ack_settle_ticks = ((uint32_t)ack_settle_ticks * FLASHER_RTC_HZ) / ack_hz
                       + (FLASHER_RTC_HZ / 2u) / poll_hz;

    /* Mask all interrupt dispatch first: from here, no ISR (all in flash, about
     * to be erased) may ever run again. NVIC pending bits still wake WFI. */
    __disable_irq();

    /* Drop the Movement-side optical session and bring up BOTH directions raw:
     * RX on SERCOM0, TX on SERCOM3. uart2_open also acquires the STANDBY clock
     * chain (OSC16M + GCLK0 RUNSTDBY) and enables SERCOM0 RXC, our WFI wake. */
    watch_optical_close();

    /* Leave the RED LED (TX) pin disengaged while listening so it can't leak into our
     * phototransistor and corrupt RX (see send_ack); it is engaged only during a TX. */
    HAL_GPIO_RED_pmuxdis();
    HAL_GPIO_RED_off();                        /* LED off (Hi-Z) */
    HAL_GPIO_IR_ENABLE_out();
    HAL_GPIO_IR_ENABLE_clr();                 /* phototransistor bias ON */
    HAL_GPIO_IRSENSE_in();
    HAL_GPIO_IRSENSE_pmuxen(HAL_GPIO_PMUX_SERCOM_ALT);

    /* tx_invert/rx_invert -> CTRLA.TXINV/RXINV: data-bit invert (ISO 7816), not
     * optical polarity, no effect in IrDA. Different SERCOMs, so RX/TX baud differ. */
    uart2_sercom_config_t txc = {
        .sercom = 3, .baud = tx_baud, .irda = irda, .invert = tx_invert, .run_in_standby = true,
    };
    uart2_sercom_config_t rxc = {
        .sercom = 0, .baud = rx_baud, .irda = irda, .invert = rx_invert, .run_in_standby = true,
    };
    uart2_open(UART2_TXPO_0, &txc, UART2_RXPO_0, &rxc);

    /* The end-of-image read-back verify must see true flash, not stale cache. */
    NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_CACHEDIS;

    /* Keep the RTC counting (it times the ACK settle) but silence its
     * interrupts so they cannot wake the flasher's WFI. Movement never resumes. */
    RTC->MODE0.INTENCLR.reg = RTC_MODE0_INTENCLR_MASK;

    /* STANDBY is selected by SLEEPCFG on the SAM L22 (NOT SCB SLEEPDEEP; see
     * system_saml22.c, which never sets it). Wait for the write to take, per the
     * SLEEPCFG bridge-latency note. */
    PM->SLEEPCFG.bit.SLEEPMODE = PM_SLEEPCFG_SLEEPMODE_STANDBY_Val;
    while (PM->SLEEPCFG.bit.SLEEPMODE != PM_SLEEPCFG_SLEEPMODE_STANDBY_Val) { }

    NVIC_ClearPendingIRQ(SERCOM0_IRQn);

    /* Into RAM. Never returns. */
    flasher_run(patch, first_block_id, first_block_addr,
                first_block_is_patch, first_block_len, first_block);
    for (;;) { }   /* unreachable, but make the no-return contract explicit */
}

#endif /* !__EMSCRIPTEN__ */

#else /* !HAS_IR_SENSOR */

/* Stub for boards without the IR sensor, so the face can sit in any
 * movement_config.h unconditionally: it just says so and yields. */

void firmware_flasher_face_setup(uint8_t watch_face_index, void **context_ptr) {
    (void) watch_face_index;
    (void) context_ptr;
}

void firmware_flasher_face_activate(void *context) {
    (void) context;
}

bool firmware_flasher_face_loop(movement_event_t event, void *context) {
    (void) context;
    if (event.event_type == EVENT_ACTIVATE) {
        watch_clear_display();
        watch_display_text_with_fallback(WATCH_POSITION_TOP, "IR", "IR");
        watch_display_text(WATCH_POSITION_BOTTOM, " no Ir");
        return true;
    }
    return movement_default_loop_handler(event);
}

void firmware_flasher_face_resign(void *context) {
    (void) context;
}

#endif // HAS_IR_SENSOR
