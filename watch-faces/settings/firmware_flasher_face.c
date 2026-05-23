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
#include "watch_optical.h"

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
 *     - FULL: each block is {addr, 256-byte row}. The watch erases+writes
 *       the row, THEN ACKs. Ordering by id (stop-and-wait):
 *         * next-in-sequence  -> write, ACK, advance
 *         * duplicate of last -> re-ACK only (our ACK was lost), no rewrite
 *         * id 0 (restart)    -> write, ACK, reset sequence
 *         * out-of-sequence / wrong kind -> do NOTHING (no ACK)
 *         * write failed      -> do NOTHING (no ACK) so the host resends
 *     - PATCH: the crle-compressed delta body streams in; each body frame
 *       is ACKed as its bytes are consumed and NEW is reconstructed a row
 *       at a time. Same dup/ordering discipline.
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

/* Mark a function RAM-resident. noinline so it can never be folded into a
 * flash-resident caller, which would defeat the whole point. On the emscripten
 * simulator (no NVMCTRL/DSU, flashing unsupported) it is an ordinary
 * flash-resident stub, so the section attribute is dropped. */
#ifndef __EMSCRIPTEN__
  #define FLASHER_RAMFUNC __attribute__((section(".ramfunc"), noinline))
#else
  #define FLASHER_RAMFUNC
#endif

/*
 * Flash geometry (SAM L22, from instance/nvmctrl.h): a page is 64 bytes, a
 * row is 256 bytes = 4 pages. Erase granularity is a row; write granularity
 * is a page. One IR firmware block maps to exactly one row.
 */
#define FLASHER_PAGE_SIZE      64u
#define FLASHER_ROW_SIZE       256u
#define FLASHER_PAGES_PER_ROW  (FLASHER_ROW_SIZE / FLASHER_PAGE_SIZE)   /* 4 */

/*
 * Writable region bounds, mirroring the SAM L22 linker map (saml22n18.ld):
 *
 *   [0x00000, 0x02000)  UF2 bootloader      (SACRED, never written)
 *   [0x02000, 0x3E000)  application flash    (the only region we may write)
 *   [0x3E000, 0x40000)  reserved EEPROM      (off-limits)
 *
 * Bricking the bootloader is the one failure with no recovery path (even USB
 * recovery needs it intact), so every write is bounds-checked against these.
 */
#define FLASHER_BOOTLOADER_END  0x2000u    /* exclusive: first writable byte */
#define FLASHER_APP_FLASH_END   0x3E000u   /* exclusive: EEPROM begins here  */

/* ---- Wire protocol (shared with the host flasher tool) -------------- *
 *
 * Frames use the serial_frame format. These frame flag bits travel
 * host -> watch; the watch -> host ACK carries no flags (it is the bare
 * 2-byte frame id echoed back).
 * (Bit 1 is reserved/unused: EXIT terminates the session and the watch stays in
 * TEST until ENTER, so no "last frame" flag is needed for either stage.)
 */
#define IR_FLASHER_FLAG_TEST        (1u << 2)  /* link-test frame: watch ACKs, does NOT commit */
#define IR_FLASHER_FLAG_ENTER       (1u << 3)  /* enter the real flasher; payload empty (full flash), or
                                                  with FLAG_PATCH+FLAG_VERIFY = the reference descriptor */
#define IR_FLASHER_FLAG_VERIFY      (1u << 4)  /* EXIT: final whole-image CRC check; payload = descriptor */
#define IR_FLASHER_FLAG_PATCH       (1u << 5)  /* delta (patch) flash rather than a verbatim uf2-like one.
                                                  On ENTER (with FLAG_VERIFY): "patch session", ACK only if
                                                  the reference descriptor matches current flash. On a data
                                                  block: this is a patch frame, not a uf2-like row. */

/* The ACK is a bare frame-id echo; repeating it (back to back) gives the host's
 * sliding-window matcher more chances to find an intact copy on the weak
 * watch->host return path, at the cost of a longer ACK. Shared by the launcher
 * (TEST-stage ACK) and the RAM flasher (block ACKs). */
#define IR_FLASHER_ACK_COUNT_MIN    1u
#define IR_FLASHER_ACK_COUNT_MAX    4u

/* The EXIT (VERIFY) frame's payload: the image descriptor, 12 bytes little-endian
 * (base_addr, total_length, image_crc32; see firmware_flasher_descriptor_t).
 * ENTER carries no payload; the descriptor travels with the final EXIT request. */
#define IR_FLASHER_DESCRIPTOR_SIZE  12u

/* A data block's payload: target address (4 LE) followed by one full
 * 256-byte row. The watch writes the row verbatim to the target address. */
#define IR_FLASHER_BLOCK_PAYLOAD    (4u + FLASHER_ROW_SIZE)   /* 260 */

/*
 * Whole-image descriptor carried by the EXIT (VERIFY) frame. The post-write
 * verify CRCs exactly [base_addr, base_addr + total_length); total_length is a
 * multiple of FLASHER_ROW_SIZE (so it is also 4-aligned for the DSU), and the
 * host computes image_crc32 over the same region (the concatenation of every
 * written row, padding included). The flasher revalidates these bounds before
 * handing the range to the DSU (a bad pointer can bus-error the CRC engine).
 */
typedef struct {
    uint32_t base_addr;
    uint32_t total_length;   /* bytes; a multiple of FLASHER_ROW_SIZE */
    uint32_t image_crc32;
} firmware_flasher_descriptor_t;

/*
 * Patch (delta) ENTER payload: the detools in-place header fields the watch
 * needs, plus the reference descriptor for the pre-flight verify. 20 bytes LE:
 *   base(4) from_size(4) ref_crc32(4) to_size(4) shift_size(4)
 * The first 12 bytes are exactly the reference descriptor {base, from_size,
 * ref_crc32}; the watch ACKs the ENTER only if its current flash over
 * [base, base+from_size) CRCs to ref_crc32. segment_size is assumed = the erase
 * row and memory_size is unused (aux path only), so neither is carried.
 */
#define IR_FLASHER_PATCH_ENTER_SIZE  20u

/*
 * Everything the RAM-resident aux patch apply needs, assembled by the launcher
 * at first-block time (after it has malloc'd the aux window) and handed to
 * firmware_flasher_arm_and_run. `aux` is a (aux_mask+1)-byte, power-of-two
 * sliding window of original-REF bytes (power-of-two so indexing is a mask, not
 * a division, which is forbidden in .ramfunc). from/to/shift_size come from the ENTER.
 */
typedef struct {
    uint8_t *aux;            /* sliding window of original REF bytes */
    uint32_t aux_mask;       /* window size - 1 (a power-of-two mask) */
    uint32_t base;           /* app base of the images */
    uint32_t from_size;      /* REF extent (source bound) */
    uint32_t to_size;        /* NEW extent (bytes to reconstruct) */
    uint32_t shift_size;     /* detools shift_size = max source lookback */
} firmware_flasher_patch_t;

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
    IR_FLASHER_PHASE_TEST_RX,       /* link open RX: test frames (ACK + stay) or ENTER (ACK + advance) */
    IR_FLASHER_PHASE_TEST_SETTLE,   /* frame received; waiting before sending ACK */
    IR_FLASHER_PHASE_TEST_TX,       /* link open TX, draining the ACK */
    /* Real-flash stage. The TEST_SETTLE / TEST_TX phases above are reused to
     * settle + drain the ENTER frame's ACK; pending_ack_advances distinguishes
     * that from a test-frame ACK. After the ENTER ACK drains we wait for the
     * first data block (re-ACKing a repeated ENTER if our ACK was lost), then
     * jump (irreversibly) into the RAM-resident flasher. */
    IR_FLASHER_PHASE_FLASH_WAIT_BLOCK,  /* ENTER ACKed; RX open, waiting for first block */
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

    /* Flash-mode runtime state (valid while phase != MENU). */
    firmware_flasher_phase_t phase;
    uint16_t test_frames_acked;   /* test frames round-tripped so far */
    uint16_t last_frame_id;       /* id of the most recent frame received (shown in the test view) */
    uint16_t pending_ack_id;      /* id of the frame currently being ACKed */
    bool     pending_ack_advances;/* the frame being ACKed is ENTER: advance to flash after the ACK */
    uint16_t settle_remaining;    /* ACK-rate ticks left before sending the ACK */
    uint16_t rx_idle_polls;       /* consecutive idle polls while RX is open */
    uint16_t rx_idle_limit;       /* reset the parser after this many (set on RX) */

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
 *  The face code below hands off to the RAM-resident flasher core, which
 *  is defined further down (after the face). Forward-declare the handful
 *  of core entry points the face calls; all are hardware-only.
 * ---------------------------------------------------------------------- */
#ifndef __EMSCRIPTEN__
static bool     firmware_flasher_range_writable(uint32_t addr, uint32_t len);
static uint32_t firmware_flasher_crc32(const uint8_t *data, uint32_t len);
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

static const uint32_t baud_options[]      = { 50, 150, 300, 600, 900, 1200, 2400, 3600, 4800, 9600 };
static const uint8_t  poll_rate_options[] = { 1, 2, 4, 8, 16, 32, 64 };
// ACK-rate ticks to wait before transmitting the ACK, giving the peer receiver's
// phototransistor time to recover from the IR self-saturation our frame caused.
static const uint8_t  ack_settle_options[] = { 0, 1, 2, 4, 8, 16, 32 };
#define BAUD_OPTION_COUNT       (sizeof(baud_options)       / sizeof(baud_options[0]))
#define POLL_RATE_OPTION_COUNT  (sizeof(poll_rate_options)  / sizeof(poll_rate_options[0]))
#define ACK_SETTLE_OPTION_COUNT (sizeof(ack_settle_options) / sizeof(ack_settle_options[0]))

// Index of each option's default value in its lookup table.
// rx 3600 / tx 300 / NRZ, RX 8 Hz, ACK 64 Hz, 4-tick ACK settle. NRZ over IrDA
// because IrDA keeps the active-low LED lit ~80% of the time (see watch_optical.h);
// the hardware-tested known-good was 3600 / 150 / IrDA.
#define RX_BAUD_DEFAULT_INDEX    7   // = 3600 baud
#define TX_BAUD_DEFAULT_INDEX    2   // =  300 baud
#define POLL_RATE_DEFAULT_INDEX  3   // =    8 Hz
#define ACK_RATE_DEFAULT_INDEX   (POLL_RATE_OPTION_COUNT - 1)  // = 64 Hz
#define ACK_SETTLE_DEFAULT_INDEX 3   // = 4 ticks

// Number of consecutive idle polls before we resync the RX parser. We reset
// only after the line has been quiet for ~0.5s, NOT on the first idle poll. At
// the data baud a frame's bytes arrive far faster than one poll period, so a
// frame never produces an idle poll mid-arrival in theory, but a stray idle
// poll from real-world jitter (a late tick, a stream hiccup) would, with a
// threshold of 1, discard a frame mid-flight. Requiring several idle polls adds
// margin while still resyncing well within the multi-second gap before the host
// retransmits a dropped frame. Floored at 2 so it's never the eager single poll.
static uint16_t compute_rx_idle_limit(uint8_t poll_hz) {
    uint16_t limit = (uint16_t)((poll_hz + 1u) / 2u);   // ~0.5s of silence
    if (limit < 2u) limit = 2u;
    return limit;
}

static void render_menu(const firmware_flasher_state_t *state);
static void render_test(const firmware_flasher_state_t *state);
static void render_flash_wait(const firmware_flasher_state_t *state);

static void enter_test(firmware_flasher_state_t *state);
static void enter_test_rx(firmware_flasher_state_t *state);
static void enter_test_tx(firmware_flasher_state_t *state);
static void enter_flash_rx(firmware_flasher_state_t *state, firmware_flasher_phase_t phase);
static void begin_ack(firmware_flasher_state_t *state, uint16_t id, bool advances);
static void abort_flash_mode(firmware_flasher_state_t *state);
static bool poll_rx_frame(firmware_flasher_state_t *state, watch_optical_frame_t *frame);
static void handle_enter_frame(firmware_flasher_state_t *state, const watch_optical_frame_t *frame);
static bool enter_patch_setup(firmware_flasher_state_t *state, const watch_optical_frame_t *frame);

/* Any phase with the optical link open / a session in progress (test handshake,
 * or auto-armed and waiting for ENTER / the first block): used to tear down
 * cleanly on resign / low-energy, and as the Alarm-aborts-everything guard. */
static bool is_link_active(const firmware_flasher_state_t *state) {
    return state->phase == IR_FLASHER_PHASE_TEST_RX ||
           state->phase == IR_FLASHER_PHASE_TEST_SETTLE ||
           state->phase == IR_FLASHER_PHASE_TEST_TX ||
           state->phase == IR_FLASHER_PHASE_FLASH_WAIT_BLOCK;
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
    // TXINV/RXINV invert only the DATA bits (ISO 7816), not the optical
    // polarity, and have no effect in IrDA (the SIR encoder owns polarity).
    // Default both OFF (no data inversion): the correct choice for IrDA, and
    // for NRZ both ends just need to agree. See watch_optical.h.
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
            if (state->phase == IR_FLASHER_PHASE_TEST_RX) {
                // Listening during the test stage. Two frames take part:
                //  - a TEST frame: flip to TX, ACK it, and keep listening (stay
                //    in the test stage). Only frames marked TEST are ACKed here,
                //    so a real payload frame can't be mistaken for one.
                //  - the (empty) ENTER frame: ACK it and advance to the real-flash
                //    stage. The watch stays in TEST until ENTER arrives; there is
                //    no separate auto-arm step.
                watch_optical_frame_t frame;
                if (poll_rx_frame(state, &frame)) {
                    state->last_frame_id = frame.id;
                    if (frame.flags & IR_FLASHER_FLAG_TEST) {
                        begin_ack(state, frame.id, false);
                        render_test(state);
                    } else if (frame.flags & IR_FLASHER_FLAG_ENTER) {
                        handle_enter_frame(state, &frame);
                    }
                }
            } else if (state->phase == IR_FLASHER_PHASE_FLASH_WAIT_BLOCK) {
                // ENTER has been ACKed; waiting for the first real data block. Two
                // things can arrive:
                //  - a repeated ENTER: the host didn't get our ACK. Re-ACK (the
                //    TEST_TX drain routes us back here), stay waiting. ENTER is
                //    empty now, so there is nothing to validate.
                //  - the first data block (address + one 256-byte row, no special
                //    flags): DO NOT ACK it. Copy it aside and hand control to the
                //    RAM-resident flasher, which writes it, ACKs it from RAM, and
                //    receives the rest. That call never returns.
                watch_optical_frame_t frame;
                if (poll_rx_frame(state, &frame)) {
                    bool first_is_patch = (frame.flags & IR_FLASHER_FLAG_PATCH) != 0;
                    if (frame.flags & IR_FLASHER_FLAG_ENTER) {
                        handle_enter_frame(state, &frame);
                    } else if (state->session_is_patch && first_is_patch &&
                               frame.size > 0 && frame.size <= FLASHER_ROW_SIZE) {
                        // First PATCH body frame (a chunk of the crle-compressed
                        // patch body). NOW (the last reversible moment) allocate
                        // the aux window and drop into the RAM flasher. Patch mode is
                        // authorized by the verified ENTER (session_is_patch), never
                        // a block flag.
#ifndef __EMSCRIPTEN__
                        // Aux window: smallest power of two >= shift_size (so the
                        // .ramfunc indexes it with a mask, not a division). If the
                        // heap can't give us one (tight RAM, or a large shift_size
                        // such as the host's max-shift option), aux stays NULL and
                        // the flasher falls back to the in-flash shift strategy: it
                        // relocates REF in flash instead of buffering it in RAM
                        // (~2x flash writes, no window). Either way we hand off; the
                        // patch is applied; malloc never strands the session.
                        uint32_t pow2 = 1u;
                        while (pow2 < state->patch_shift_size) pow2 <<= 1;
                        // CRITICAL: do NOT just malloc() and check the result.
                        // newlib's _sbrk has no limit check, so an oversized request
                        // (e.g. the max-shift option's ~256 KiB pow2) does NOT fail
                        // cleanly: it bumps the heap break PAST the top of RAM and
                        // leaves the allocator corrupt for every later allocation,
                        // while still returning non-NULL. So decide whether it fits
                        // BEFORE allocating: probe the current heap break and require
                        // the window + stack headroom to fit under the RAM top. Too
                        // big => aux stays NULL => in-flash shift fallback, and malloc
                        // is never even called, so the heap is never disturbed.
                        uint8_t *aux = NULL;
                        uintptr_t brk = (uintptr_t)_sbrk(0);  // current heap top
                        if (brk >= 0x20000000u && brk < 0x20008000u &&
                            (uint32_t)(0x20008000u - brk) >= pow2 + 4096u /*stack reserve*/) {
                            aux = malloc(pow2);   // guaranteed to fit: cannot corrupt
                        }
                        firmware_flasher_patch_t pd = {
                            .aux = aux, .aux_mask = aux ? (pow2 - 1u) : 0u,
                            .base = state->patch_base, .from_size = state->patch_from_size,
                            .to_size = state->patch_to_size, .shift_size = state->patch_shift_size,
                        };
                        state->first_block_id = frame.id;
                        memcpy(state->first_block, frame.payload, frame.size);
                        // POINT OF NO RETURN: never returns (reboots on done).
                        firmware_flasher_arm_and_run(
                            baud_options[state->rx_baud_index],
                            baud_options[state->tx_baud_index],
                            state->encoding == IR_FLASHER_ENC_IRDA,
                            state->tx_invert, state->rx_invert,
                            poll_rate_options[state->poll_rate_index],
                            poll_rate_options[state->ack_rate_index],
                            ack_settle_options[state->ack_settle_index],
                            state->ack_count,
                            &pd, frame.id, 0u, true, frame.size, state->first_block);
#endif
                    } else if (!first_is_patch && frame.size == IR_FLASHER_BLOCK_PAYLOAD) {
                        // First uf2-like data block (full flash, including a patch ->
                        // uf2 block-0 fallback). DO NOT ACK it. Copy it aside and
                        // hand control to the RAM flasher (writes + ACKs from RAM).
                        state->first_block_id   = frame.id;
                        state->first_block_addr = (uint32_t)frame.payload[0]
                                                | ((uint32_t)frame.payload[1] << 8)
                                                | ((uint32_t)frame.payload[2] << 16)
                                                | ((uint32_t)frame.payload[3] << 24);
                        memcpy(state->first_block, frame.payload + 4, FLASHER_ROW_SIZE);
#ifndef __EMSCRIPTEN__
                        // POINT OF NO RETURN: never returns. patch = NULL => uf2
                        // session; first_block_len is the full row.
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
                        // Unreachable on hardware; flashing is unsupported on the
                        // simulator, so just return to the menu there.
                        abort_flash_mode(state);
                        render_menu(state);
                    }
                    // else: not a valid first block for this session -> ignore.
                }
            } else if (state->phase == IR_FLASHER_PHASE_TEST_SETTLE) {
                // Counting down the ACK settle delay (shared by the test-frame
                // and ENTER-frame ACKs). The core sleeps in STANDBY between these
                // (ACK-rate) ticks; when it elapses we transmit.
                if (--state->settle_remaining == 0) {
                    enter_test_tx(state);
                }
            } else if (state->phase == IR_FLASHER_PHASE_TEST_TX) {
                // Draining the ACK at the ACK tick rate. The core sleeps in
                // STANDBY between ticks while the SERCOM clocks the ACK out; we
                // wake to check progress and flip back to RX the moment it's idle.
                watch_optical_poll();
                if (watch_optical_tx_idle()) {
                    if (state->pending_ack_advances) {
                        // An ENTER-frame ACK just drained (fresh, or a re-ACK of a
                        // repeated one). (Re)enter the wait-for-first-block state.
                        enter_flash_rx(state, IR_FLASHER_PHASE_FLASH_WAIT_BLOCK);
                        render_flash_wait(state);
                    } else {
                        // A test-frame ACK drained: count it and keep listening.
                        state->test_frames_acked++;
                        enter_test_rx(state);
                        render_test(state);
                    }
                }
            }
            break;

        case EVENT_LIGHT_BUTTON_DOWN:
            // Swallow so movement_default_loop_handler doesn't turn on the LED.
            break;

        case EVENT_LIGHT_BUTTON_UP:
            // Light cycles the menu pages, but only once the settings are
            // unlocked; while locked the flash page is the only page, so a short
            // press is a no-op. During flash mode the Light button is inert.
            if (state->phase == IR_FLASHER_PHASE_MENU && state->settings_unlocked) {
                state->menu_index = (state->menu_index + 1) % IR_FLASHER_MENU_COUNT;
                render_menu(state);
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

static watch_optical_config_t make_config(const firmware_flasher_state_t *state, bool tx) {
    watch_optical_config_t cfg = {
        .baud = baud_options[tx ? state->tx_baud_index : state->rx_baud_index],
        .irda = (state->encoding == IR_FLASHER_ENC_IRDA),
        .invert = tx ? state->tx_invert : state->rx_invert,
    };
    return cfg;
}

static void enter_test(firmware_flasher_state_t *state) {
    // Begin the link-reliability handshake. enter_test_rx sets the poll rate.
    // The watch stays in the test stage (ACKing test frames) until the host's
    // ENTER frame advances it to the real-flash stage.
    state->test_frames_acked = 0;
    state->session_is_patch  = false;   // set true only by a verified patch ENTER
    enter_test_rx(state);
}

// Flip the half-duplex link to RX and enter `phase`. close() first: open()
// refuses to switch directions while the other is still active. RX runs at the
// configured poll rate, fast enough to drain the UART FIFO at the chosen baud.
// Shared by the test handshake and both real-flash RX waits.
static void enter_flash_rx(firmware_flasher_state_t *state, firmware_flasher_phase_t phase) {
    watch_optical_close();
    watch_optical_config_t cfg = make_config(state, false);
    watch_optical_open(WATCH_OPTICAL_DIR_RX, &cfg);
    movement_request_tick_frequency(poll_rate_options[state->poll_rate_index]);
    state->rx_idle_polls = 0;
    state->rx_idle_limit = compute_rx_idle_limit(poll_rate_options[state->poll_rate_index]);
    state->phase = phase;
}

static void enter_test_rx(firmware_flasher_state_t *state) {
    enter_flash_rx(state, IR_FLASHER_PHASE_TEST_RX);
}

// Queue an ACK for `id` and drive it out: switch to the ACK tick rate, then
// either fire immediately or hold off for the configured settle (so the peer's
// phototransistor recovers from the self-saturation our incoming frame caused).
// Shared by the test-frame ACK and the ENTER-frame ACK; `advances` is true for an
// ENTER frame, telling the TEST_TX drain to move on to the real-flash stage.
static void begin_ack(firmware_flasher_state_t *state, uint16_t id, bool advances) {
    state->pending_ack_id       = id;
    state->pending_ack_advances = advances;
    movement_request_tick_frequency(poll_rate_options[state->ack_rate_index]);
    state->settle_remaining     = ack_settle_options[state->ack_settle_index];
    if (state->settle_remaining == 0) {
        enter_test_tx(state);
    } else {
        state->phase = IR_FLASHER_PHASE_TEST_SETTLE;
    }
}

static void enter_test_tx(firmware_flasher_state_t *state) {
    // Flip the link to TX and queue the ACK: the bare 2-byte id (LE) of the
    // frame we received, no framing or CRC. The host is blocked waiting for
    // exactly this id, so any corruption in flight simply fails to match and is
    // retried; the id echo is self-validating in the safe direction.
    //
    // The tick rate is already the configured ACK rate (set when the frame was
    // received, and held through any settle). The TEST_TX drain runs at it so we
    // notice tx_idle (and flip back to RX) within ~one tick of the ACK clearing
    // the wire, not a full configured-poll period later, which would risk the
    // host's next frame arriving before we're listening. The core STANDBYs
    // between ticks (the SERCOM transmits in standby), so this costs wake-ups,
    // not a busy-spin: the right trade on coin-cell power.
    watch_optical_close();
    watch_optical_config_t cfg = make_config(state, true);
    watch_optical_open(WATCH_OPTICAL_DIR_TX, &cfg);
    // Send the 2-byte id ack_count times back to back. Repeats give the host's
    // sliding-window matcher more chances to find an intact copy on the weak
    // return path; it accepts the first valid copy, so extra copies are free of
    // false positives.
    uint8_t ack[2 * IR_FLASHER_ACK_COUNT_MAX];
    uint8_t n = 0;
    for (uint8_t i = 0; i < state->ack_count; i++) {
        ack[n++] = (uint8_t)(state->pending_ack_id     );
        ack[n++] = (uint8_t)(state->pending_ack_id >> 8);
    }
    watch_optical_send_raw(ack, n);
    state->phase = IR_FLASHER_PHASE_TEST_TX;
}

static void abort_flash_mode(firmware_flasher_state_t *state) {
    watch_optical_close();
    movement_request_tick_frequency(1);
    state->phase = IR_FLASHER_PHASE_MENU;
}

/* Poll the RX link once and report whether a complete frame is ready in
 * `frame`. Folds in the idle-resync bookkeeping shared by every RX phase: a
 * delivered frame or fed-but-incomplete bytes count as progress; only after
 * rx_idle_limit consecutive fully-quiet polls (~0.5s) do we abandon a half-
 * parsed frame and resync to HUNT, so a live frame is never discarded on a lone
 * jittery idle poll. See compute_rx_idle_limit. */
static bool poll_rx_frame(firmware_flasher_state_t *state, watch_optical_frame_t *frame) {
    watch_optical_poll();
    if (watch_optical_receive(frame)) {
        state->rx_idle_polls = 0;
        return true;
    }
    if (watch_optical_rx_idle()) {
        if (++state->rx_idle_polls >= state->rx_idle_limit) {
            watch_optical_rx_reset();
            state->rx_idle_polls = 0;
        }
    } else {
        state->rx_idle_polls = 0;
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
    if (firmware_flasher_crc32((const uint8_t *)base, from) != ref) return false;
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

/* Handle an ENTER frame from a RX-waiting phase. A plain ENTER (empty payload) is
 * ACKed and advances to the real-flash stage. A patch ENTER (FLAG_PATCH) is ACKed
 * only if its header verifies (enter_patch_setup, which also stores the params); a
 * mismatch is dropped silently so the host can retry or give up. Shared by the
 * test phase (first ENTER) and FLASH_WAIT_BLOCK (a repeated ENTER whose ACK was
 * lost; re-verify is harmless since no flashing has begun). */
static void handle_enter_frame(firmware_flasher_state_t *state,
                               const watch_optical_frame_t *frame) {
    bool is_patch = (frame->flags & IR_FLASHER_FLAG_PATCH) != 0;
    if (is_patch && !enter_patch_setup(state, frame)) {
        return;   /* patch header invalid / reference mismatch -> no ACK */
    }
    // Authorize patch blocks ONLY via a verified patch ENTER: this, not any
    // block flag, is what puts the flasher into patch mode.
    state->session_is_patch = is_patch;
    begin_ack(state, frame->id, true);
    render_flash_wait(state);
}

/* --- Rendering --------------------------------------------------------- */

static void render_menu(const firmware_flasher_state_t *state) {
    char buf[8];
    watch_clear_display();
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

static void render_test(const firmware_flasher_state_t *state) {
    char buf[8];
    watch_clear_display();
    watch_set_indicator(WATCH_INDICATOR_ARROWS);
    // "tESt" (custom) / "FLAS" (classic) + the id of the last frame received.
    watch_display_text_with_fallback(WATCH_POSITION_TOP, "tESt ", "FLAS");
    snprintf(buf, sizeof(buf), "%6u", (unsigned)state->last_frame_id);
    watch_display_text(WATCH_POSITION_BOTTOM, buf);
}

static void render_flash_wait(const firmware_flasher_state_t *state) {
    (void) state;
    watch_clear_display();
    watch_set_indicator(WATCH_INDICATOR_ARROWS);
    // ENTER received: in the real-flash stage, waiting for the first data block.
    watch_display_text_with_fallback(WATCH_POSITION_TOP, "WA1T", "WA1T");
}

/* ===================================================================== *
 *  RAM-RESIDENT FLASHER CORE                                            *
 *                                                                       *
 *  Every function marked FLASHER_RAMFUNC is RAM-resident on the SAM L22, *
 *  which the startup copies from flash into RAM at boot (piggybacking on *
 *  the .data copy loop; see saml22n18.ld and startup_saml22.c). Once     *
 *  running, the RAM copy executes, so the flash rows holding the         *
 *  original firmware can be erased and overwritten underneath it.        *
 *                                                                       *
 *  --- THE .ramfunc DISCIPLINE (read before touching the core) ---      *
 *                                                                       *
 *  While an NVMCTRL erase or page-write is in flight, the flash array    *
 *  CANNOT be read: any instruction fetch or data load from flash stalls  *
 *  or faults. So every FLASHER_RAMFUNC function, and everything it       *
 *  touches, must live in RAM:                                           *
 *    - No calls to flash-resident functions (Movement, watch_optical,    *
 *      libc). In particular DO NOT call memcpy/memset; the compiler's    *
 *      helpers live in flash. Copy with explicit loops.                 *
 *    - No reads of .rodata (const tables, string literals). The CRC      *
 *      routine is deliberately table-free.                              *
 *    - Immediate constants and per-function literal pools are fine: the  *
 *      compiler places a function's literal pool in its own section,     *
 *      i.e. in RAM next to it.                                          *
 *    - Avoid 64-bit math and division (they pull in flash-resident       *
 *      helpers); use shifts.                                            *
 *  Reading flash is allowed only when NVMCTRL is idle (no command        *
 *  pending): that is how firmware_flasher_crc32() can checksum the       *
 *  freshly written image at the end of a session.                       *
 *                                                                       *
 *  Everything below drives NVMCTRL / the DSU / raw SERCOM and so is the  *
 *  hardware-only path: it compiles to nothing on the emscripten         *
 *  simulator (no fallback: the launcher's flash hand-off is likewise     *
 *  gated out there).                                                    *
 * ===================================================================== */
#ifndef __EMSCRIPTEN__

#include "sam.h"
#include "uart2.h"

/* watch_optical_close() (from watch_optical.h, included above) tears down the
 * Movement-side optical session before the arming step reconfigures the link
 * raw. */

/* ---- Bounds check ---------------------------------------------------- *
 *
 * Pure arithmetic with no hardware. The single source of truth for "may I write
 * here": both the per-image descriptor and every per-row write are validated
 * through it (defense in depth). True iff [addr, addr + len) lies entirely
 * within the writable application region: does not touch the bootloader below
 * or the reserved EEPROM above, and does not wrap. len must be non-zero. */
FLASHER_RAMFUNC static bool firmware_flasher_range_writable(uint32_t addr, uint32_t len) {
    if (len == 0) return false;
    /* Guard against wrap-around: addr + len must not overflow. */
    if (addr > 0xFFFFFFFFu - len) return false;
    uint32_t end = addr + len;   /* exclusive */
    if (addr < FLASHER_BOOTLOADER_END) return false;   /* touches bootloader */
    if (end  > FLASHER_APP_FLASH_END)  return false;   /* touches EEPROM/OOB */
    return true;
}

/* =====================================================================
 *  !!! DRY RUN: BENCH LINK TEST ONLY. MUST BE 0 FOR REAL FLASHING. !!!
 *
 *  Set to 1 to exercise the ENTIRE optical protocol on silicon (RX, parse,
 *  stop-and-wait dedup, ACK (with settle + sensor blanking), and the VERIFY
 *  round-trip + reboot) while touching NO flash at all: firmware_flasher_write_row
 *  becomes a no-op that reports success, and VERIFY is treated as passing so the
 *  watch ACKs everything and cleanly reboots into the UNCHANGED current firmware.
 *  Nothing is erased or written. Set back to 0 before any real flash.
 * ===================================================================== */
#define FLASHER_DRY_RUN 0

/* The flash array is memory-mapped at FLASH_ADDR (0x0) and written a 16-bit
 * half-word at a time. Writes here land in the NVMCTRL page buffer; they only
 * reach the array when the WP command is issued. */
#define NVM_MEMORY ((volatile uint16_t *)FLASH_ADDR)

/* NVMCTRL.ADDR is a half-word (16-bit) address, so a byte address is halved
 * before it is loaded (same convention as watch_storage.c). */
#define NVM_HALFWORD_ADDR(byte_addr)  ((byte_addr) >> 1)

/* ---- NVMCTRL command primitives ------------------------------------- */
/* Compiled out under FLASHER_DRY_RUN (nothing calls them, and nothing else may
 * issue an erase/write): the only code in this file that modifies the array. */
#if !FLASHER_DRY_RUN

/* Spin until NVMCTRL finishes the pending command, then clear and report its
 * error state. Touches only the NVMCTRL peripheral registers (APB, not the
 * flash array), so it is safe to spin here while an erase/write completes.
 * Returns true if the command completed cleanly, false if the controller
 * flagged a programming, lock-region, or invalid-command error. */
FLASHER_RAMFUNC static bool nvm_wait_ready(void) {
    while (!NVMCTRL->INTFLAG.bit.READY) {
        /* ~6 ms for a row erase, ~1 ms for a page write */
    }
    uint16_t status = NVMCTRL->STATUS.reg;
    /* Write-1-to-clear the sticky status bits so the next command starts
     * from a known state. */
    NVMCTRL->STATUS.reg = NVMCTRL_STATUS_MASK;
    return (status & (NVMCTRL_STATUS_NVME      /* programming/erase error   */
                    | NVMCTRL_STATUS_LOCKE     /* locked-region violation   */
                    | NVMCTRL_STATUS_PROGE))   /* invalid command           */
           == 0;
}

/* Load ADDR and issue `cmd` (we OR in the execution key here) at the given
 * byte address, then wait for the command to retire. Returns
 * nvm_wait_ready()'s verdict. */
FLASHER_RAMFUNC static bool nvm_command(uint32_t byte_addr, uint32_t cmd) {
    NVMCTRL->ADDR.reg  = NVM_HALFWORD_ADDR(byte_addr);
    NVMCTRL->CTRLA.reg = cmd | NVMCTRL_CTRLA_CMDEX_KEY;
    return nvm_wait_ready();
}

/* Erase one 256-byte row. Caller guarantees row alignment and writability. */
FLASHER_RAMFUNC static bool nvm_erase_row(uint32_t row_addr) {
    if (!nvm_wait_ready()) return false;
    return nvm_command(row_addr, NVMCTRL_CTRLA_CMD_ER);
}

/* Write one 64-byte page from `src` (page-aligned region) to `page_addr`.
 * Clears the page buffer, fills it half-word by half-word from RAM, then
 * commits with WP. CTRLB.MANW is 1 at reset (manual write), so filling the
 * buffer does not auto-trigger a write; the explicit WP does. */
FLASHER_RAMFUNC static bool nvm_write_page(uint32_t page_addr, const uint8_t *src) {
    if (!nvm_wait_ready()) return false;

    /* Page Buffer Clear: start from an all-ones buffer. */
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_PBC | NVMCTRL_CTRLA_CMDEX_KEY;
    if (!nvm_wait_ready()) return false;

    /* Fill the page buffer. Writing to the mapped flash address loads the
     * buffer (no array access yet). Combine bytes into half-words explicitly
     * (no memcpy: it lives in flash). */
    volatile uint16_t *dst = &NVM_MEMORY[NVM_HALFWORD_ADDR(page_addr)];
    for (uint32_t i = 0; i < FLASHER_PAGE_SIZE; i += 2) {
        uint16_t hw = (uint16_t)src[i] | ((uint16_t)src[i + 1] << 8);
        dst[i >> 1] = hw;
    }

    /* Commit the buffer to the array. */
    return nvm_command(page_addr, NVMCTRL_CTRLA_CMD_WP);
}

#endif /* !FLASHER_DRY_RUN */

/* ---- Public API (hardware) ------------------------------------------ */

/*
 * Commit one 256-byte row: bounds-check, erase the row, write its four pages.
 * `addr` must be row-aligned (a multiple of FLASHER_ROW_SIZE) and the row must
 * be writable per firmware_flasher_range_writable(); otherwise nothing is
 * written and the function returns false. `data` points at exactly
 * FLASHER_ROW_SIZE bytes (4-byte aligned). Returns false if NVMCTRL reports a
 * programming error, in which case the caller must NOT acknowledge the block
 * (the host will retransmit).
 */
FLASHER_RAMFUNC static bool firmware_flasher_write_row(uint32_t addr, const uint8_t *data) {
    /* Defense in depth: bounds-check every row write even though the image
     * descriptor was already validated up front. */
    if (!firmware_flasher_range_writable(addr, FLASHER_ROW_SIZE)) return false;
    if ((addr & (FLASHER_ROW_SIZE - 1u)) != 0) return false;   /* row-aligned */

#if FLASHER_DRY_RUN
    /* Bench link test: report success without touching flash. */
    (void)data;
    return true;
#else
    if (!nvm_erase_row(addr)) return false;

    for (uint32_t p = 0; p < FLASHER_PAGES_PER_ROW; p++) {
        uint32_t page_addr = addr + p * FLASHER_PAGE_SIZE;
        if (!nvm_write_page(page_addr, &data[p * FLASHER_PAGE_SIZE])) return false;
    }
    return true;
#endif
}

/*
 * CRC-32 (zlib/Ethernet variant: poly 0xEDB88320 reflected, init 0xFFFFFFFF,
 * final XOR 0xFFFFFFFF) over `len` bytes starting at `data`, via the SAM L22 DSU
 * hardware engine in memory mode, the same engine and convention as
 * serial_frame.c. Reused for both the per-row frame check and the
 * end-of-session whole-image verify. The DSU requires `data` 4-byte aligned and
 * `len` a multiple of 4 (both call sites satisfy this); `data` may point into
 * the flash array (the whole-image verify) provided NVMCTRL is idle.
 */
FLASHER_RAMFUNC static uint32_t firmware_flasher_crc32(const uint8_t *data, uint32_t len) {
    /* DSU hardware CRC32, memory mode: register sequence replicated from
     * serial_frame.c's compute_crc32 (we can't call it: it lives in flash).
     * The two gotchas it documents apply verbatim:
     *   1. DSU registers are PAC-write-protected by default; unlock first or
     *      the writes are silently dropped and CTRL never starts.
     *   2. Write ADDR/LENGTH verbatim, NOT via the DSU_ADDR_ADDR /
     *      DSU_LENGTH_LENGTH macros, which shift left by 2 (the fields start
     *      at bit 2), which would point the engine at a wildly wrong region. */
    PAC->WRCTRL.reg = PAC_WRCTRL_PERID(ID_DSU) | PAC_WRCTRL_KEY_CLR;

    DSU->ADDR.reg    = (uint32_t)data;
    DSU->LENGTH.reg  = len;
    DSU->DATA.reg    = 0xFFFFFFFFu;                          /* CRC32 init */
    DSU->STATUSA.reg = DSU_STATUSA_DONE | DSU_STATUSA_BERR;  /* clear */
    DSU->CTRL.reg    = DSU_CTRL_CRC;                         /* start */
    while (!(DSU->STATUSA.reg & DSU_STATUSA_DONE)) { /* spin */ }
    /* DSU returns the non-complemented CRC; XOR with 0xFFFFFFFF to match the
     * standard IEEE 802.3 / zlib.crc32 variant. */
    uint32_t crc = DSU->DATA.reg ^ 0xFFFFFFFFu;

    PAC->WRCTRL.reg = PAC_WRCTRL_PERID(ID_DSU) | PAC_WRCTRL_KEY_SET;
    return crc;
}

/* ===================================================================== *
 *  RAM-resident flasher: receive/commit loop + arming hand-off          *
 * ===================================================================== */

/* SERCOM topology, must match watch_optical.c:
 *   RX: SERCOM0 / IRSENSE (PA04, RXPO 0); TX: SERCOM3 / RED LED (PA12, TXPO 0). */
#define FLASHER_RX_SERCOM   SERCOM0
#define FLASHER_TX_SERCOM   SERCOM3
#define FLASHER_RX_IRQ      SERCOM0_IRQn

/* RTC MODE0 COUNT32 ticks at 128 Hz (1024 Hz / DIV8; see rtc32.c), used to
 * time the ACK settle below. */
#define FLASHER_RTC_HZ                128u

/* Settle (in RTC ticks) before each ACK so the host's phototransistor recovers
 * from the saturation its own LED caused while sending the frame (the weak,
 * asymmetric ACK path). NOT hardcoded: computed in arm_and_run directly in RTC
 * ticks from the TEST-stage link timings (the middle of that stage's validated
 * effective-settle window); see the formula there.
 *
 * Why it must be this large: the TEST stage ran RX at the poll rate, so its ACK
 * left the configured settle PLUS up to one poll period of Movement latency. The
 * flasher processes each byte immediately (no poll latency), so it has to wait
 * that whole window itself; otherwise the ACK arrives while the host is still
 * recovering and it mis-reads the pulses (e.g. 02 00 -> 7f/a0/ff). */
static uint32_t s_ack_settle_ticks;
static uint8_t  s_ack_count = 1;   /* times to repeat the id per ACK (1..4) */

/* Note there is deliberately NO stall timeout: if the host walks away the
 * flasher stays put in STANDBY (slow cell drain) rather than rebooting into a
 * half-written, almost-certainly-unbootable image. Staying in the flasher keeps
 * the only non-USB recovery path open: the user can resume flashing over the
 * link. The flash is committed once entered; reboot happens only on a verified
 * image. */

/* ---- Raw polled SERCOM + RTC (all .ramfunc) ------------------------- */

FLASHER_RAMFUNC static uint32_t rtc_count(void) {
    /* COUNTSYNC is left enabled by rtc_init(), so COUNT is continuously
     * synchronized, so a plain read is all the coarse stall timer needs. */
    return RTC->MODE0.COUNT.reg;
}

FLASHER_RAMFUNC static void busy_wait_ticks(uint32_t ticks) {
    uint32_t t0 = rtc_count();
    while ((rtc_count() - t0) < ticks) { /* spin; the RTC keeps running */ }
}

FLASHER_RAMFUNC static void wfi_standby(void) {
    /* SLEEPCFG was set to STANDBY during arming, so WFI enters STANDBY. A
     * pending SERCOM0 RXC (or any pending NVIC bit) wakes us; PRIMASK=1 keeps
     * the ISR from dispatching, so we simply resume here and re-poll. */
    __DSB();
    __WFI();
}

/* Read one RX byte if the SERCOM has one. Clears the sticky error flags and the
 * NVIC pending bit (so the next WFI sleeps until the next byte). */
FLASHER_RAMFUNC static bool rx_read_byte(uint8_t *out) {
    Sercom *S = FLASHER_RX_SERCOM;
    if (!(S->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC)) return false;
    uint16_t status = S->USART.STATUS.reg;
    *out = (uint8_t)S->USART.DATA.reg;
    S->USART.STATUS.reg = status;            /* W1C FERR/BUFOVF/PERR */
    NVIC_ClearPendingIRQ(FLASHER_RX_IRQ);
    return true;
}

/* Discard whatever is sitting in the RX path; used right after an ACK, where
 * our own LED may have leaked into the (re-powered) sensor. */
FLASHER_RAMFUNC static void rx_flush(void) {
    Sercom *S = FLASHER_RX_SERCOM;
    while (S->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) {
        uint16_t status = S->USART.STATUS.reg;
        (void)S->USART.DATA.reg;
        S->USART.STATUS.reg = status;
    }
    NVIC_ClearPendingIRQ(FLASHER_RX_IRQ);
}

/* Polled TX of `n` bytes, then wait for the line to fully drain (TXC). */
FLASHER_RAMFUNC static void tx_write(const uint8_t *buf, uint32_t n) {
    Sercom *S = FLASHER_TX_SERCOM;
    for (uint32_t i = 0; i < n; i++) {
        while (!(S->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE)) { }
        S->USART.DATA.reg = buf[i];
    }
    while (!(S->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC)) { }
}

/* ---- Minimal serial-frame parser (.ramfunc, mirrors serial_frame.c) -- *
 *
 * The flasher needs its own parser because serial_frame.c lives in flash
 * being erased. It is deliberately tiny: it only ever sees two frame shapes,
 * a data block (IR_FLASHER_BLOCK_PAYLOAD bytes) and an EXIT/VERIFY request
 * (IR_FLASHER_DESCRIPTOR_SIZE-byte payload), and validates each with the DSU
 * CRC. (It never sees ENTER: the launcher absorbs ENTER + its repeats and only
 * hands off once a real data block arrives.) crcbuf holds the CRC region
 * (id + len/flags at [0..3], payload at [4..]); the delivered payload is
 * crcbuf[4..4+out_len). */
enum { PS_HUNT = 0, PS_HEADER, PS_PAYLOAD, PS_CRC };

static struct {
    uint8_t  state;
    uint8_t  pre;         /* preamble bytes matched so far (0..3) */
    uint8_t  hidx;        /* header bytes collected (0..4) */
    uint16_t physical;    /* payload bytes on the wire (declared rounded up to 4) */
    uint16_t pidx;        /* payload bytes collected */
    uint8_t  cidx;        /* CRC bytes collected (0..4) */
    uint32_t crc_recv;    /* CRC read off the wire, little-endian */
    uint16_t out_id;      /* delivered: frame id */
    uint8_t  out_flags;   /* delivered: flag bits */
    uint16_t out_len;     /* delivered: declared payload length */
    uint8_t  crcbuf[4 + IR_FLASHER_BLOCK_PAYLOAD] __attribute__((aligned(4)));
} P;

FLASHER_RAMFUNC static void parser_reset(void) {
    P.state = PS_HUNT;
    P.pre   = 0;
}

/* Feed one byte; returns true once a complete, CRC-valid frame is in P.
 *
 * Written as an if/else ladder, NOT a switch: a dense switch on P.state would
 * make the compiler emit a libgcc jump-table helper (__gnu_thumb1_case_*),
 * which lives in flash and would be fetched mid-erase. Keep it branch-only. */
FLASHER_RAMFUNC static bool parser_feed(uint8_t b) {
    if (P.state == PS_HUNT) {
        /* preamble = AA 55 AA 55 */
        uint8_t expect = (P.pre & 1u) ? 0x55u : 0xAAu;
        if (b == expect) {
            if (++P.pre == 4) { P.pre = 0; P.state = PS_HEADER; P.hidx = 0; }
        } else {
            P.pre = (b == 0xAAu) ? 1u : 0u;   /* allow an immediate restart */
        }
        return false;
    }
    if (P.state == PS_HEADER) {
        P.crcbuf[P.hidx++] = b;
        if (P.hidx == 4) {
            uint16_t lenflags = (uint16_t)P.crcbuf[2] | ((uint16_t)P.crcbuf[3] << 8);
            uint16_t declared = lenflags & 0x3FFu;
            P.out_id    = (uint16_t)P.crcbuf[0] | ((uint16_t)P.crcbuf[1] << 8);
            P.out_flags = (uint8_t)((lenflags >> 10) & 0x3Fu);
            P.out_len   = declared;
            if (declared > IR_FLASHER_BLOCK_PAYLOAD) {   /* impossible frame: resync */
                parser_reset();
                return false;
            }
            P.physical = (uint16_t)((declared + 3u) & ~3u);
            P.pidx = 0; P.cidx = 0; P.crc_recv = 0;
            P.state = (P.physical == 0) ? PS_CRC : PS_PAYLOAD;
        }
        return false;
    }
    if (P.state == PS_PAYLOAD) {
        P.crcbuf[4 + P.pidx] = b;
        if (++P.pidx == P.physical) { P.state = PS_CRC; P.cidx = 0; }
        return false;
    }
    /* PS_CRC (and any stray state) */
    P.crc_recv |= (uint32_t)b << (8u * P.cidx);
    if (++P.cidx == 4) {
        uint32_t calc = firmware_flasher_crc32(P.crcbuf, 4u + P.physical);
        parser_reset();
        return (calc == P.crc_recv);
    }
    return false;
}

/* ---- ACK + frame handling (.ramfunc) -------------------------------- */

/* Send the bare 2-byte frame-id ACK (the entire watch -> host vocabulary).
 * Settle first, then: blind our own sensor, engage the RED LED on SERCOM3 TX,
 * transmit, disengage the LED again (so it can't leak into RX while we listen),
 * re-arm the sensor, drop any leaked bytes, and resync the parser.
 *
 * Engaging the TX pin only for the transmit (not continuously) keeps the LED
 * from leaking into our own sensor while we listen. The red LED is active-LOW,
 * so in IrDA the SIR-idle-low line leaves the LED LIT at idle, so it MUST be
 * disengaged before RX or it floods the sensor (IrDA is the worse case, not the
 * safe one). In NRZ the idle (mark = high) is LED-off; we disengage in both
 * cases anyway as belt-and-suspenders, mirroring watch_optical. */
FLASHER_RAMFUNC static void send_ack(uint16_t id) {
    busy_wait_ticks(s_ack_settle_ticks);
    /* Repeat the 2-byte id s_ack_count times so the host's sliding-window matcher
     * has more chances to find an intact copy on the weak return path. */
    uint8_t ack[2 * IR_FLASHER_ACK_COUNT_MAX];
    uint32_t n = 0;
    for (uint8_t i = 0; i < s_ack_count; i++) {
        ack[n++] = (uint8_t)id;
        ack[n++] = (uint8_t)(id >> 8);
    }

    HAL_GPIO_IR_ENABLE_set();                         /* sensor bias OFF (blind to our LED) */
    HAL_GPIO_RED_drvstr(1);
    HAL_GPIO_RED_pmuxen(HAL_GPIO_PMUX_SERCOM_ALT);    /* engage LED on SERCOM3 TX */

    tx_write(ack, n);

    HAL_GPIO_RED_pmuxdis();                           /* disengage... */
    HAL_GPIO_RED_off();                               /* ...LED off (Hi-Z) */
    HAL_GPIO_IR_ENABLE_clr();                         /* sensor bias ON again */
    rx_flush();
    parser_reset();
}

/* Id of the last block actually committed, or -1 before the first one. Drives
 * stop-and-wait dedup/ordering so a resent block (whose ACK we lost) is re-ACKed
 * without redoing the write, and a uf2-like restart (id 0) reflashes. */
static int32_t s_last_id;

/* Current data mode: true = applying a patch, false = writing verbatim uf2-like
 * rows. Seeded in flasher_run from whether a (verified) patch descriptor was
 * handed in, NOT from any block flag. In the uf2 path it can flip patch -> uf2
 * (never back) on a uf2-like block 0; see commit_block. */
static bool s_patch_mode;

/* Apply one data block under the stop-and-wait ordering + mode rules. `is_patch`
 * is the block's FLAG_PATCH.
 *
 *   - uf2-like (non-patch) block 0, while patching : abandon the patch and
 *       restart as a FULL flash (a patch is applied in place and can't be
 *       re-run from the start; the only restart is a verbatim reflash). This
 *       switches the mode to uf2-like permanently.
 *   - block kind must match the current mode             : else ignore (silent).
 *   - id == last_id                                      : dup (our ACK was lost) -> re-ACK only.
 *   - id == last_id + 1                                  : the next block -> commit, ACK, advance.
 *   - id == 0 in uf2-like mode (restart)                 : commit, ACK, last_id = 0.
 *   - anything else (incl. a patch restart)              : ignore (silent).
 *
 * The very first block folds into the "next" case (s_last_id starts at -1).
 * A commit that fails leaves last_id untouched and sends no ACK, so the host
 * retransmits. The per-row target is bounds-checked inside write_row (absolute
 * bootloader/EEPROM guard); the EXIT CRC is the whole-image integrity gate.
 *
 * NOTE: the live patch apply runs entirely in patch_apply (it consumes the body
 * stream directly), so commit_block only ever handles uf2-like rows or a stray
 * patch frame arriving after the apply. It writes uf2 rows; a stray patch frame is
 * ignored (only a duplicate of the last id is re-ACKed); it must never apply. */
FLASHER_RAMFUNC static void commit_block(uint16_t id, bool is_patch,
                                         uint32_t addr, const uint8_t *row) {
    /* uf2-like block 0 received while patching: drop to a full reflash. */
    if (id == 0 && !is_patch && s_patch_mode) {
        s_patch_mode = false;
        s_last_id = -1;
    }
    if (is_patch != s_patch_mode) {
        return;                       /* wrong kind for this mode: ignore */
    }
    if (s_last_id >= 0 && id == (uint16_t)s_last_id) {
        send_ack(id);                 /* duplicate of the last committed block */
        return;
    }
    /* A non-duplicate PATCH frame here can only be a stray AFTER patch_apply has
     * run (the live patch stream is consumed inside patch_apply, never here). We
     * must NOT "apply" it: ACKing it would mask a failed apply by waving the host
     * through to an EXIT-CRC hang. Ignore it; the host stays honestly stuck on the
     * failing frame, and a uf2 block-0 (handled above) still recovers via reflash. */
    if (is_patch) {
        return;
    }
    /* uf2-like row from here on (is_patch == s_patch_mode == false). */
    bool is_next    = (id == (uint16_t)(s_last_id + 1));
    bool is_restart = (id == 0);      /* a full flash may restart from block 0 */
    if (!is_next && !is_restart) {
        return;                       /* out-of-sequence: ignore */
    }
    if (firmware_flasher_write_row(addr, row)) {
        s_last_id = (int32_t)id;
        send_ack(id);
    }
    /* else: commit failed -> stay silent, last_id unchanged -> retransmit */
}

/* Act on the frame currently held in P. Returns true only when an EXIT frame
 * confirmed the whole image (the caller then reboots into the new firmware). */
FLASHER_RAMFUNC static bool handle_frame(void) {
    if (P.out_flags & IR_FLASHER_FLAG_VERIFY) {
        /* EXIT: the 12-byte payload is the image descriptor {base, total_length,
         * image_crc32}. Validate the range BEFORE the DSU touches it: a bad,
         * unaligned, or out-of-bounds pointer can bus-error the CRC engine and
         * wedge this loop. A malformed EXIT is ignored (no ACK -> host retries). */
        if (P.out_len != IR_FLASHER_DESCRIPTOR_SIZE) return false;
        uint32_t base = (uint32_t)P.crcbuf[4]  | ((uint32_t)P.crcbuf[5]  << 8)
                      | ((uint32_t)P.crcbuf[6]  << 16) | ((uint32_t)P.crcbuf[7]  << 24);
        uint32_t len  = (uint32_t)P.crcbuf[8]  | ((uint32_t)P.crcbuf[9]  << 8)
                      | ((uint32_t)P.crcbuf[10] << 16) | ((uint32_t)P.crcbuf[11] << 24);
        uint32_t want = (uint32_t)P.crcbuf[12] | ((uint32_t)P.crcbuf[13] << 8)
                      | ((uint32_t)P.crcbuf[14] << 16) | ((uint32_t)P.crcbuf[15] << 24);
        if ((base & 3u) != 0) return false;                       /* DSU needs 4-aligned addr */
        if ((len & (FLASHER_ROW_SIZE - 1u)) != 0) return false;   /* row- (=> 4-) aligned */
        if (!firmware_flasher_range_writable(base, len)) return false;  /* nonzero + in app flash */
#if FLASHER_DRY_RUN
        /* Bench link test: nothing was written, so the real CRC can't match.
         * Treat EXIT as passing to exercise the round-trip + reboot (into the
         * unchanged current firmware). */
        (void)want;
        send_ack(P.out_id);
        return true;
#else
        if (firmware_flasher_crc32((const uint8_t *)base, len) == want) {
            send_ack(P.out_id);   /* echo the id => "passed"; caller reboots */
            return true;
        }
        /* Mismatch: stay silent. The host gets no echo, prompts the user, and on
         * a resend we overwrite toward a valid image rather than rebooting into a
         * brick. The watch stays in the flasher either way. */
        return false;
#endif
    }

    if (P.out_flags & IR_FLASHER_FLAG_PATCH) {
        /* A patch frame seen by this loop is a re-sent LAST body frame whose ACK
         * was lost (the live patch stream is consumed inside patch_apply, not here):
         * commit_block re-ACKs it as a duplicate (id == s_last_id) and ignores any
         * other patch frame. It has no addr/row, so we pass 0/NULL. */
        commit_block(P.out_id, true, 0, (const uint8_t *)0);
    } else if (P.out_len == IR_FLASHER_BLOCK_PAYLOAD) {
        /* A uf2-like data block: a full flash, or the patch->uf2 block-0 fallback
         * (commit_block flips s_patch_mode on a non-patch block 0). */
        uint32_t addr = (uint32_t)P.crcbuf[4]
                      | ((uint32_t)P.crcbuf[5] << 8)
                      | ((uint32_t)P.crcbuf[6] << 16)
                      | ((uint32_t)P.crcbuf[7] << 24);
        commit_block(P.out_id, false, addr, &P.crcbuf[8]);
    }
    return false;
}

/* ===================================================================== *
 *  Patch (delta) apply: crle-decompressed, in place, mirrors the verified
 *  host-side reference apply (aux path). UNTESTED ON HARDWARE.
 *
 *  The crle-compressed patch BODY streams in over 256 B frames (the launcher
 *  hands off the first one; the rest arrive here). The segment loop pulls
 *  decompressed bytes on demand: a frame is ACKed the moment its bytes are
 *  consumed, so stop-and-wait still holds. NEW is reconstructed one row at a
 *  time into a RAM buffer, then erase+written. Source bytes (original REF) come
 *  from flash where the row hasn't been overwritten yet, else from a sliding
 *  "aux window" of the last shift_size original bytes (saved just before each row
 *  is overwritten). No in-flash shift; no row is erased twice. The window is the
 *  malloc'd buffer the launcher passed (we never malloc/free in .ramfunc).
 *
 *  Two varint encodings are in play: crle's own (unsigned, 7-bit LSB groups) for
 *  its lengths, and detools' signed varint (6-bit first byte) for the segment
 *  diff/extra/adjustment sizes inside the DECOMPRESSED stream.
 * ===================================================================== */

/* Patch session params, copied from the descriptor in flasher_run. */
static uint8_t *s_aux;        /* aux window (power-of-two bytes); NULL = in-flash shift */
static uint32_t s_aux_mask;   /* window size - 1 (mask, since no division here)     */
static uint32_t s_pbase;      /* app base of the images                             */
static uint32_t s_pfrom;      /* REF extent (source bound)                          */
static uint32_t s_pto;        /* NEW extent (bytes to reconstruct)                  */
static uint32_t s_pshift;     /* detools shift_size (the patch's read-coord origin)  */
static uint32_t s_pshift_phys;/* in-flash shift only: where REF is physically placed
                               * = region_end - base - from (REF flush to flash end,
                               * independent of shift_size). Read maps patch index r
                               * -> base + s_pshift_phys + r. ENTER validated
                               * shift <= this, so the (>=0) delta is implicit.       */

/* Current compressed body frame, fed to the crle decoder. */
static uint8_t  s_body[FLASHER_ROW_SIZE];
static uint16_t s_body_len, s_body_pos;
static uint16_t s_body_id;    /* frame id of the current body frame */

/* crle decoder state. */
enum { CRLE_NEED = 0, CRLE_SCATTER, CRLE_REPEAT };
static uint8_t  s_crle_state;
static uint32_t s_crle_left;  /* scattered/repeat bytes still to emit */
static uint8_t  s_crle_byte;  /* the repeated byte */

/* Pull the next compressed byte; at a frame boundary, ACK the consumed frame and
 * receive the next in-sequence body frame (re-ACKing a duplicate whose ACK the
 * host missed). A non-patch / out-of-sequence frame mid-apply is ignored (v1: no
 * mid-apply fallback; a failed patch is recovered after the apply, see below). */
FLASHER_RAMFUNC static uint8_t patch_in_byte(void) {
    if (s_body_pos >= s_body_len) {
        send_ack(s_body_id);                       /* current frame consumed */
        for (;;) {
            uint8_t b;
            if (!rx_read_byte(&b)) { wfi_standby(); continue; }
            if (!parser_feed(b)) continue;
            bool is_patch = (P.out_flags & IR_FLASHER_FLAG_PATCH) != 0;
            if (is_patch && P.out_id == s_body_id) { send_ack(s_body_id); continue; }  /* dup */
            if (is_patch && P.out_id == (uint16_t)(s_body_id + 1)) {
                s_body_id  = P.out_id;
                s_body_len = P.out_len;
                for (uint16_t k = 0; k < s_body_len; k++) s_body[k] = P.crcbuf[4 + k];
                s_body_pos = 0;
                break;
            }
            /* else: ignore, keep waiting */
        }
    }
    return s_body[s_body_pos++];
}

/* crle's own length varint: unsigned, 7-bit groups LSB-first, 0x80 = continue. */
FLASHER_RAMFUNC static uint32_t crle_varint(void) {
    uint32_t v = 0, off = 0;
    uint8_t b;
    do { b = patch_in_byte(); v |= (uint32_t)(b & 0x7Fu) << off; off += 7u; } while (b & 0x80u);
    return v;
}

/* Pull one DECOMPRESSED byte (matches detools CrleDecompressor). */
FLASHER_RAMFUNC static uint8_t crle_out_byte(void) {
    for (;;) {
        if (s_crle_state == CRLE_SCATTER) {
            if (s_crle_left) { s_crle_left--; return patch_in_byte(); }  /* literal passthrough */
            s_crle_state = CRLE_NEED;
        } else if (s_crle_state == CRLE_REPEAT) {
            if (s_crle_left) { s_crle_left--; return s_crle_byte; }
            s_crle_state = CRLE_NEED;
        }
        uint8_t kind = patch_in_byte();
        uint32_t n   = crle_varint();
        if (kind == 0u) {                       /* SCATTERED: n literal bytes */
            s_crle_left = n; s_crle_state = CRLE_SCATTER;
        } else {                                /* REPEATED: one byte, n times */
            s_crle_left = n; s_crle_byte = patch_in_byte(); s_crle_state = CRLE_REPEAT;
        }
    }
}

/* detools signed varint (6-bit first byte), read from the DECOMPRESSED stream. */
FLASHER_RAMFUNC static int32_t patch_size(void) {
    uint8_t b = crle_out_byte();
    bool sign = (b & 0x40u) != 0;
    uint32_t v = b & 0x3Fu, off = 6u;
    while (b & 0x80u) { b = crle_out_byte(); v |= (uint32_t)(b & 0x7Fu) << off; off += 7u; }
    return sign ? -(int32_t)v : (int32_t)v;
}

/* Read n source bytes for the reconstruction into dst. Two strategies:
 *  - aux (s_aux != NULL): read original REF at index (from_offset - shift), from
 *    flash if not yet overwritten (index >= tip = current row start), else from
 *    the sliding aux window.
 *  - in-flash shift (s_aux == NULL): REF was relocated to s_pshift_phys (flush to
 *    flash end), so REF index r sits at base + s_pshift_phys + r, still intact
 *    because the segment loop only reads ahead of the write frontier; no window.
 * Returns false on a corrupt/over-reaching reference. */
FLASHER_RAMFUNC static bool patch_read_source(uint32_t from_offset, uint8_t *dst,
                                              uint32_t n, uint32_t tip) {
    if (s_aux == NULL) {
        for (uint32_t i = 0; i < n; i++) {
            uint32_t r = from_offset + i - s_pshift;          /* REF index (as in aux path) */
            if (r >= s_pfrom) return false;                   /* beyond REF (or underflow): corrupt */
            dst[i] = *((volatile uint8_t *)(s_pbase + s_pshift_phys + r));
        }
        return true;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint32_t r = from_offset + i - s_pshift;          /* original-REF index */
        if (r >= s_pfrom) return false;                   /* beyond REF: corrupt */
        if (r >= tip) {
            dst[i] = *((volatile uint8_t *)(s_pbase + r));/* still original in flash */
        } else {
            if (tip - r > s_aux_mask + 1u) return false;  /* outside the window */
            dst[i] = s_aux[r & s_aux_mask];
        }
    }
    return true;
}

/* Reconstruct one NEW segment (= one row) into `row` (mirrors the host-side reference apply). */
FLASHER_RAMFUNC static bool patch_apply_segment(uint32_t to_offset, uint32_t seg_to,
                                                uint32_t from_offset, uint8_t *row) {
    if (patch_size() != 0) return false;                  /* dfpatch_size must be 0 */
    uint32_t to_pos = 0;
    while (to_pos < seg_to) {
        int32_t dsize = patch_size();                     /* diff run */
        if (dsize < 0 || to_pos + (uint32_t)dsize > seg_to) return false;
        if (dsize > 0) {
            uint8_t src[FLASHER_ROW_SIZE];
            if (!patch_read_source(from_offset, src, (uint32_t)dsize, to_offset)) return false;
            for (int32_t j = 0; j < dsize; j++)
                row[to_pos + (uint32_t)j] = (uint8_t)(crle_out_byte() + src[j]);
            from_offset += (uint32_t)dsize;
            to_pos      += (uint32_t)dsize;
        }
        int32_t esize = patch_size();                     /* extra (literal) run */
        if (esize < 0 || to_pos + (uint32_t)esize > seg_to) return false;
        for (int32_t j = 0; j < esize; j++) row[to_pos++] = crle_out_byte();
        from_offset += (uint32_t)patch_size();            /* adjustment (signed) */
    }
    return true;
}

/* In-flash shift fallback (s_aux == NULL): relocate the from_size of REF to
 * s_pshift_phys, flush against the app-flash end, NOT by the patch's shift_size
 * (constant placement, independent of shift_size, for uniform flash wear). The
 * source then reads straight from the relocated copy at base + s_pshift_phys + r,
 * no RAM window. Copy row-by-row HIGH-TO-LOW so a destination row never clobbers a
 * source row not yet read (when the physical shift < from_size the source and
 * destination ranges overlap). Both offsets are row-aligned, so /256 and *256 are
 * constant power-of-two shifts (no division helper). ~from_rows extra
 * erase+program cycles; aligning to the end keeps NEW/REF overlap minimal. */
FLASHER_RAMFUNC static bool patch_shift_ref(void) {
    uint32_t from_rows = s_pfrom / FLASHER_ROW_SIZE;
    uint8_t row[FLASHER_ROW_SIZE];
    for (uint32_t k = from_rows; k-- > 0; ) {             /* k = from_rows-1 .. 0 */
        uint32_t src = s_pbase + k * FLASHER_ROW_SIZE;
        uint32_t dst = src + s_pshift_phys;
        for (uint32_t i = 0; i < FLASHER_ROW_SIZE; i++)   /* read source row (NVM idle) */
            row[i] = *((volatile uint8_t *)(src + i));
        if (!firmware_flasher_write_row(dst, row)) return false;
    }
    return true;
}

/* Drive the whole patch apply. Returns true once all of NEW is reconstructed +
 * written (the final EXIT verify still gates the reboot, in the loop below). */
FLASHER_RAMFUNC static bool patch_apply(const uint8_t *first_block,
                                        uint16_t first_id, uint16_t first_len) {
    bool shift_mode = (s_aux == NULL);
    /* In-flash shift fallback: place REF flush against the app-flash end (the
     * largest valid shift; ENTER validated base+shift+from <= end, so this is
     * >= the patch's shift_size). Relocate BEFORE consuming any patch bytes
     * (flash still holds the original REF in full at this point). */
    if (shift_mode) {
        s_pshift_phys = (FLASHER_APP_FLASH_END - s_pbase) - s_pfrom;
        if (!patch_shift_ref()) return false;
    }

    s_body_id = first_id;
    s_body_len = first_len;
    for (uint16_t k = 0; k < first_len; k++) s_body[k] = first_block[k];
    s_body_pos = 0;
    s_crle_state = CRLE_NEED; s_crle_left = 0;

    uint8_t row[FLASHER_ROW_SIZE];
    const uint32_t seg = FLASHER_ROW_SIZE;
    for (uint32_t to_off = 0; to_off < s_pto; to_off += seg) {
        uint32_t from_offset = (to_off + seg > s_pshift) ? (to_off + seg) : s_pshift;
        uint32_t seg_to = (s_pto - to_off < seg) ? (s_pto - to_off) : seg;
        if (!patch_apply_segment(to_off, seg_to, from_offset, row)) return false;
        for (uint32_t k = seg_to; k < seg; k++) row[k] = 0xFFu;  /* pad partial last row (erased) */
        /* aux strategy only: save this row's ORIGINAL bytes (still in flash) into
         * the window before we overwrite them, so later rows can reference them as
         * source. The shift strategy reads from the relocated copy, so no save. */
        if (!shift_mode)
            for (uint32_t k = 0; k < seg && (to_off + k) < s_pfrom; k++)
                s_aux[(to_off + k) & s_aux_mask] = *((volatile uint8_t *)(s_pbase + to_off + k));
        if (!firmware_flasher_write_row(s_pbase + to_off, row)) return false;
    }
    send_ack(s_body_id);   /* ACK the final body frame (consumed, not yet ACKed) */
    return true;
}

/* The RAM-resident main loop. For a uf2 session, commits the first block then
 * receives/commits the rest; for a patch session, runs the crle aux apply, then
 * (either way) waits for the EXIT verify. NEVER RETURNS: there is intentionally
 * no stall timeout (see the settle note above): an abandoned flash stays parked
 * here, recoverable over the link, rather than rebooting into a half-written
 * image. */
FLASHER_RAMFUNC static void flasher_run(const firmware_flasher_patch_t *patch,
                                        uint16_t first_block_id,
                                        uint32_t first_block_addr,
                                        bool first_block_is_patch,
                                        uint16_t first_block_len,
                                        const uint8_t *first_block) {
    s_last_id = -1;
    /* Patch mode is authorized by the verified ENTER (patch != NULL), NOT by any
     * block flag, so a stray patch frame can't apply against an unverified image. */
    s_patch_mode = (patch != NULL);
    parser_reset();

    if (s_patch_mode) {
        s_aux = patch->aux; s_aux_mask = patch->aux_mask;
        s_pbase = patch->base; s_pfrom = patch->from_size;
        s_pto = patch->to_size; s_pshift = patch->shift_size;
        if (patch_apply(first_block, first_block_id, first_block_len)) {
            s_last_id = (int32_t)s_body_id;   /* so a re-sent last body frame re-ACKs */
        }
        /* On failure the flash is half-patched; we stay parked (no reboot) and a
         * uf2 block-0 fallback (handled below via commit_block) can still recover
         * with a full reflash. Reset the parser for the clean EXIT-wait loop. */
        parser_reset();
    } else {
        /* uf2 session: commit the first block (received but not yet ACKed). The
         * uf2 first block is always a full row, so first_block_len is unused here. */
        commit_block(first_block_id, first_block_is_patch, first_block_addr, first_block);
    }

    for (;;) {
        uint8_t b;
        if (rx_read_byte(&b)) {
            if (parser_feed(b) && handle_frame()) {
                /* send_ack() already drained the ACK to TXC, so the host has
                 * the bits; reboot into the freshly verified firmware. */
                NVIC_SystemReset();
            }
        } else {
            wfi_standby();
        }
    }
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
    s_ack_count = (ack_count < IR_FLASHER_ACK_COUNT_MIN) ? IR_FLASHER_ACK_COUNT_MIN
                : (ack_count > IR_FLASHER_ACK_COUNT_MAX) ? IR_FLASHER_ACK_COUNT_MAX
                : ack_count;
    /* Per-ACK settle, in RTC ticks = MIDDLE of the TEST stage's effective-settle
     * window expressed directly in ticks. The window in seconds is
     * [settle/ack_hz, settle/ack_hz + 1/poll_hz]; its midpoint x 128 Hz is:
     *     ack_settle_ticks*128/ack_hz  +  (128/2)/poll_hz
     * i.e. the configured settle plus half the poll-latency window that the TEST
     * stage incurred but the flasher (no poll latency) does not. */
    s_ack_settle_ticks = ((uint32_t)ack_settle_ticks * FLASHER_RTC_HZ) / ack_hz
                       + (FLASHER_RTC_HZ / 2u) / poll_hz;

    /* Mask all interrupt dispatch first: from here, no ISR (all in flash, about
     * to be erased) may ever run again. NVIC pending bits still wake WFI. */
    __disable_irq();

    /* Drop the Movement-side optical session and bring up BOTH directions raw:
     * RX on SERCOM0, TX on SERCOM3. uart2_open also acquires the STANDBY clock
     * chain (OSC16M + GCLK0 RUNSTDBY) and enables SERCOM0 RXC, our WFI wake. */
    watch_optical_close();

    /* Leave the RED LED (TX) pin DISENGAGED while listening: a lit LED leaks into
     * our own phototransistor and corrupts RX. In IrDA the SIR encoder idles the
     * line LOW (LED off), so an engaged pin would actually be harmless there, but
     * in NRZ the idle line is the mark level (HIGH = LED on), so we disengage in
     * both cases as belt-and-suspenders, exactly what watch_optical does. send_ack()
     * engages the pin only for the brief moment it transmits, then disengages it. */
    HAL_GPIO_RED_pmuxdis();
    HAL_GPIO_RED_off();                        /* LED off (Hi-Z) */
    HAL_GPIO_IR_ENABLE_out();
    HAL_GPIO_IR_ENABLE_clr();                 /* phototransistor bias ON */
    HAL_GPIO_IRSENSE_in();
    HAL_GPIO_IRSENSE_pmuxen(HAL_GPIO_PMUX_SERCOM_ALT);

    /* tx_invert/rx_invert -> CTRLA.TXINV/RXINV: these invert only the DATA bits
     * (ISO 7816 inverse convention), NOT the optical polarity, and have no effect
     * in IrDA (the SIR encoder owns the polarity); see watch_optical.h. They're
     * passed through from the launcher's menu but should be left false for normal
     * IrDA use. Different SERCOMs, so the two baud rates may differ (fast data
     * in, slow ACK out). */
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

#endif // HAS_IR_SENSOR
