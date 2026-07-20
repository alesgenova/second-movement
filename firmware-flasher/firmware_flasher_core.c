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
/* ===================================================================== *
 *  RAM-RESIDENT FLASHER CORE (backend-agnostic translation unit)        *
 *                                                                       *
 *  Everything in this TU runs from RAM on the SAM L22. There are NO     *
 *  per-function section attributes: the build renames EVERY allocated   *
 *  section of this object to .flovl.* (objcopy, see the Makefile), so   *
 *  code, literal pools, .rodata AND statics all land in the flasher     *
 *  overlay (flasher-overlay.ld) -- NOT boot-resident, but copied in on  *
 *  demand by the face's firmware_flasher_overlay_load() at first-block  *
 *  time. Once running, the RAM copy executes, so the flash rows holding *
 *  the original firmware can be erased and overwritten underneath it.   *
 *                                                                       *
 *  --- THE RAM-TU DISCIPLINE (read before touching this file) ---       *
 *                                                                       *
 *  While an NVMCTRL erase or page-write is in flight, the flash array   *
 *  CANNOT be read: any instruction fetch or data load from flash stalls *
 *  or faults. Whole-TU capture makes .rodata and jump tables legal here *
 *  (they live in RAM too; -fno-jump-tables stays on out of caution),    *
 *  but everything OUTSIDE this TU is still off-limits mid-erase:        *
 *    - No calls to flash-resident functions (Movement, watch_optical,   *
 *      libc, libgcc). In particular DO NOT call memcpy/memset, and      *
 *      avoid 64-bit math and division (their helpers live in flash).    *
 *      Copy with explicit loops; use shifts.                            *
 *    - Calls into the linked backend TU are fine (same treatment).      *
 *  Verify with the objdump relocation audit that no reloc escapes to a  *
 *  flash symbol. Reading flash is allowed only when NVMCTRL is idle:    *
 *  that is how firmware_flasher_crc32() can checksum the freshly        *
 *  written image at the end of a session.                               *
 *                                                                       *
 *  Any non-ARM compile of this TU is a hosted test build (see           *
 *  sensor-watch-ir-tools/flasher-sim): the hardware primitives are     *
 *  swapped for test-driver hooks; the protocol logic is identical.      *
 * ===================================================================== */

#ifdef __arm__
#include "watch.h"
#endif
#include "firmware_flasher_core.h"

#if (defined(__arm__) && defined(HAS_IR_SENSOR)) || (!defined(__arm__) && !defined(__EMSCRIPTEN__))

#ifdef __arm__

#include "sam.h"
#include "uart2.h"

/* The flash array is memory-mapped at FLASH_ADDR (0x0) and written a 16-bit
 * half-word at a time. Writes here land in the NVMCTRL page buffer; they only
 * reach the array when the WP command is issued. */
#define NVM_MEMORY ((volatile uint16_t *)FLASH_ADDR)

/* NVMCTRL.ADDR is a half-word (16-bit) address, so a byte address is halved
 * before it is loaded (same convention as watch_storage.c). */
#define NVM_HALFWORD_ADDR(byte_addr)  ((byte_addr) >> 1)

/* Bound for every blocking hardware wait in the flash/verify/ACK path, so a
 * brownout-wedged peripheral (NVMCTRL never READY, DSU never DONE, SERCOM never
 * draining) can't trap the CPU and leave the flasher deaf to the id-0 recovery: on
 * timeout the op reports failure and control returns to the receive loop. ~1 s at
 * the 128 Hz RTC (see FLASHER_RTC_HZ), vast margin over a ~6 ms erase / ~1 ms write;
 * the 32 kHz RTC keeps counting through a flash-current brownout, so it still fires. */
#define FLASHER_HW_TIMEOUT_TICKS  128u   /* ~1 s at the 128 Hz RTC MODE0 counter */

/* NVMCTRL gets a much tighter bound of its own: a legitimate row erase is
 * ~6 ms and a page program ~2.5 ms, so 8 ticks (55-62 ms) is still many times
 * any real completion. Under battery sag NVMCTRL can wedge with READY never
 * asserting; bailing out quickly lets the row-write retry loop use the expired
 * wait as a cooldown instead of burning a full second per attempt. The SERCOM
 * and DSU waits keep the long bound above (a 300-baud ACK drain alone is
 * ~270 ms). */
#define FLASHER_NVM_TIMEOUT_TICKS  8u    /* 55-62 ms at 128 Hz */

/* Rewrite attempts per row after a failed/timed-out erase+program pass. Each
 * failed pass already idled through the NVM timeout (a built-in cooldown for
 * a sagging cell), so retries follow immediately; a row that still fails after
 * these is treated as fatal by the caller. */
#define FLASHER_ROW_WRITE_RETRIES  3u

/* Read the free-running 32-bit RTC counter (COUNTSYNC is left enabled, so a
 * plain read is coherent). Used only to bound the spins below. */
#define FLASHER_RTC_NOW()  (RTC->MODE0.COUNT.reg)

/* ---- NVMCTRL command primitives ------------------------------------- */
/* The only code in this file that modifies the flash array. */

/* Spin until NVMCTRL finishes the pending command, then clear and report its
 * error state. Touches only the NVMCTRL peripheral registers (APB, not the
 * flash array), so it is safe to spin here while an erase/write completes.
 * Returns true if the command completed cleanly, false if the controller
 * flagged a programming, lock-region, or invalid-command error. */
static bool nvm_wait_ready(void) {
    uint32_t t0 = FLASHER_RTC_NOW();
    while (!NVMCTRL->INTFLAG.bit.READY) {
        /* ~6 ms for a row erase, ~1 ms for a page write -- but BOUNDED: a
         * brownout-wedged controller that never asserts READY must not trap the
         * CPU here, or the flasher can never honour an id-0 recovery. On timeout
         * report failure; write_row retries the row, and a row that keeps
         * failing is fatal to its session (no ACK / backend abort). */
        if ((uint32_t)(FLASHER_RTC_NOW() - t0) > FLASHER_NVM_TIMEOUT_TICKS) return false;
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
static bool nvm_command(uint32_t byte_addr, uint32_t cmd) {
    NVMCTRL->ADDR.reg  = NVM_HALFWORD_ADDR(byte_addr);
    NVMCTRL->CTRLA.reg = cmd | NVMCTRL_CTRLA_CMDEX_KEY;
    return nvm_wait_ready();
}

/* Erase one 256-byte row. Caller guarantees row alignment and writability. */
static bool nvm_erase_row(uint32_t row_addr) {
    if (!nvm_wait_ready()) return false;
    return nvm_command(row_addr, NVMCTRL_CTRLA_CMD_ER);
}

/* Write one 64-byte page from `src` (page-aligned region) to `page_addr`.
 * Clears the page buffer, fills it half-word by half-word from RAM, then
 * commits with WP. CTRLB.MANW is 1 at reset (manual write), so filling the
 * buffer does not auto-trigger a write; the explicit WP does. */
static bool nvm_write_page(uint32_t page_addr, const uint8_t *src) {
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

/* ---- Public API (hardware) ------------------------------------------ */

/* Forward decl: defined with the raw SERCOM/RTC helpers below. */
static void busy_wait_ticks(uint32_t ticks);

/* One complete erase+program pass over a row. False on any NVMCTRL error or
 * timeout; the row's content is then UNDEFINED (a retry must redo the whole
 * pass). */
static bool nvm_row_pass(uint32_t addr, const uint8_t *data) {
    if (!nvm_erase_row(addr)) return false;
    for (uint32_t p = 0; p < FLASHER_PAGES_PER_ROW; p++) {
        uint32_t page_addr = addr + p * FLASHER_PAGE_SIZE;
        if (!nvm_write_page(page_addr, &data[p * FLASHER_PAGE_SIZE])) return false;
    }
    return true;
}

/*
 * Commit one 256-byte row: bounds-check, erase the row, write its four pages,
 * retrying the whole pass up to FLASHER_ROW_WRITE_RETRIES times on an NVMCTRL
 * error or timeout (a failed pass already idled through the NVM timeout, which
 * doubles as a cooldown for a sagging battery; the head-of-pass wait also lets
 * a still-pending command drain before the redo). `addr` must be row-aligned
 * and writable per firmware_flasher_range_writable(); otherwise nothing is
 * written and the function returns false immediately (deterministic caller
 * error, never retried). `data` points at exactly FLASHER_ROW_SIZE bytes.
 *
 * `cooldown_ticks` (128 Hz RTC) is idled after a successful commit, so
 * burst-writing callers (the patch backends, which can emit hundreds of rows
 * back-to-back from a few input bytes) duty-cycle the NVM current and give a
 * coin cell time to recover between rows. Link-paced callers (full flash: one
 * row per received frame) pass 0.
 *
 * Returns false only when the row still failed after every retry; the caller
 * must NOT acknowledge the block (full flash: the host retransmits; patch:
 * the backend aborts the apply).
 */
bool firmware_flasher_write_row(uint32_t addr, const uint8_t *data, uint8_t cooldown_ticks) {
    /* Defense in depth: bounds-check every row write even though the image
     * descriptor was already validated up front. */
    if (!firmware_flasher_range_writable(addr, FLASHER_ROW_SIZE)) return false;
    if ((addr & (FLASHER_ROW_SIZE - 1u)) != 0) return false;   /* row-aligned */

    for (uint32_t attempt = 0; attempt <= FLASHER_ROW_WRITE_RETRIES; attempt++) {
        if (nvm_row_pass(addr, data)) {
            if (cooldown_ticks) busy_wait_ticks(cooldown_ticks);
            return true;
        }
    }
    return false;
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
uint32_t firmware_flasher_crc32(const uint8_t *data, uint32_t len) {
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
    uint32_t t0 = FLASHER_RTC_NOW();
    while (!(DSU->STATUSA.reg & DSU_STATUSA_DONE)) {
        /* Bounded: a brownout-wedged DSU that never signals DONE must not trap
         * the CPU. On timeout fall through with a CRC that cannot match a real
         * image, so VERIFY fails, no reboot, and the receive loop stays alive. */
        if ((uint32_t)(FLASHER_RTC_NOW() - t0) > FLASHER_HW_TIMEOUT_TICKS) break;
    }
    /* DSU returns the non-complemented CRC; XOR with 0xFFFFFFFF to match the
     * standard IEEE 802.3 / zlib.crc32 variant. A timed-out (never-DONE) engine
     * yields 0, which will not match the descriptor's image_crc32. */
    uint32_t crc = (DSU->STATUSA.reg & DSU_STATUSA_DONE) ? (DSU->DATA.reg ^ 0xFFFFFFFFu) : 0u;

    PAC->WRCTRL.reg = PAC_WRCTRL_PERID(ID_DSU) | PAC_WRCTRL_KEY_SET;
    return crc;
}

/* ===================================================================== *
 *  RAM-resident flasher: receive/commit loop + arming hand-off          *
 * ===================================================================== */

/* SERCOM topology, must match watch_optical.c:
 *   RX: SERCOM3 / IRSENSE (PB01, RXPO 3); TX: SERCOM0 / white LED (PB22, TXPO 2). */
#define FLASHER_RX_SERCOM   SERCOM3
#define FLASHER_TX_SERCOM   SERCOM0
#define FLASHER_RX_IRQ      SERCOM3_IRQn

/* RTC MODE0 COUNT32 ticks at 128 Hz (1024 Hz / DIV8; see rtc32.c), used to
 * time the ACK settle below. */
/* FLASHER_RTC_HZ lives in firmware_flasher_core.h (the face's arming step
 * derives the ACK settle from it too). */

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
uint32_t flasher_ack_settle_ticks;
uint8_t  flasher_ack_count = 1;   /* times to repeat the id per ACK (1..4) */

/* Note there is deliberately NO stall timeout: if the host walks away the
 * flasher stays put in STANDBY (slow cell drain) rather than rebooting into a
 * half-written, almost-certainly-unbootable image. Staying in the flasher keeps
 * the only non-USB recovery path open: the user can resume flashing over the
 * link. The flash is committed once entered; reboot happens only on a verified
 * image. */

/* ---- Raw polled SERCOM + RTC (all .ramfunc) ------------------------- */

static uint32_t rtc_count(void) {
    /* COUNTSYNC is left enabled by rtc_init(), so COUNT is continuously
     * synchronized, so a plain read is all the coarse stall timer needs. */
    return RTC->MODE0.COUNT.reg;
}

static void busy_wait_ticks(uint32_t ticks) {
    uint32_t t0 = rtc_count();
    while ((rtc_count() - t0) < ticks) { /* spin; the RTC keeps running */ }
}

static void wfi_standby(void) {
    /* SLEEPCFG was set to STANDBY during arming, so WFI enters STANDBY. A
     * pending SERCOM3 RXC (or any pending NVIC bit) wakes us; PRIMASK=1 keeps
     * the ISR from dispatching, so we simply resume here and re-poll. */
    __DSB();
    __WFI();
}

/* Read one RX byte if the SERCOM has one. Clears the sticky error flags and the
 * NVIC pending bit (so the next WFI sleeps until the next byte). */
static bool rx_read_byte(uint8_t *out) {
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
static void rx_flush(void) {
    Sercom *S = FLASHER_RX_SERCOM;
    uint32_t t0 = FLASHER_RTC_NOW();
    while (S->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) {
        uint16_t status = S->USART.STATUS.reg;
        (void)S->USART.DATA.reg;
        S->USART.STATUS.reg = status;
        /* Bounded: a wedged SERCOM stuck asserting RXC must not trap us here.
         * Leftover bytes are harmless -- parser_reset() follows every flush. */
        if ((uint32_t)(FLASHER_RTC_NOW() - t0) > FLASHER_HW_TIMEOUT_TICKS) break;
    }
    NVIC_ClearPendingIRQ(FLASHER_RX_IRQ);
}

/* Polled TX of `n` bytes, then wait for the line to fully drain (TXC). Every wait
 * is bounded (see FLASHER_HW_TIMEOUT_TICKS): a brownout-wedged SERCOM that never
 * asserts DRE/TXC must not trap the CPU in an ACK -- a dropped ACK just makes the
 * host retransmit, whereas a hang would kill the id-0 recovery. */
static void tx_write(const uint8_t *buf, uint32_t n) {
    Sercom *S = FLASHER_TX_SERCOM;
    uint32_t t0 = FLASHER_RTC_NOW();
    for (uint32_t i = 0; i < n; i++) {
        while (!(S->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE))
            if ((uint32_t)(FLASHER_RTC_NOW() - t0) > FLASHER_HW_TIMEOUT_TICKS) return;
        S->USART.DATA.reg = buf[i];
    }
    while (!(S->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC))
        if ((uint32_t)(FLASHER_RTC_NOW() - t0) > FLASHER_HW_TIMEOUT_TICKS) return;
}

#else /* hosted (non-ARM) test build: the test driver provides the hardware layer */

void send_ack(uint16_t id);
bool rx_read_byte(uint8_t *out);
void wfi_standby(void);
void host_reboot(void);
#define NVIC_SystemReset() host_reboot()

#endif /* __arm__ */

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

static void parser_reset(void) {
    P.state = PS_HUNT;
    P.pre   = 0;
}

/* Feed one byte; returns true once a complete, CRC-valid frame is in P.
 *
 * Written as an if/else ladder, NOT a switch: a dense switch on P.state would
 * make the compiler emit a libgcc jump-table helper (__gnu_thumb1_case_*),
 * which lives in flash and would be fetched mid-erase. Keep it branch-only. */
static bool parser_feed(uint8_t b) {
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

#ifdef __arm__

/* ---- ACK + frame handling (.ramfunc) -------------------------------- */

/* Send the bare 2-byte frame-id ACK (the entire watch -> host vocabulary).
 * Settle first, then: blind our own sensor, engage the LED on SERCOM0 TX,
 * transmit, disengage the LED again (so it can't leak into RX while we listen),
 * re-arm the sensor, drop any leaked bytes, and resync the parser.
 *
 * Engaging the TX pin only for the transmit (not continuously) keeps the active-low
 * RED LED from leaking into our own sensor: in IrDA the SIR-idle-low line leaves it
 * LIT at idle, so it must be disengaged before RX (we disengage for NRZ too). */
static void send_ack(uint16_t id) {
    busy_wait_ticks(flasher_ack_settle_ticks);
    /* Repeat the 2-byte id flasher_ack_count times so the host's sliding-window matcher
     * has more chances to find an intact copy on the weak return path. */
    uint8_t ack[2 * IR_FLASHER_ACK_COUNT_MAX];
    uint32_t n = 0;
    for (uint8_t i = 0; i < flasher_ack_count; i++) {
        ack[n++] = (uint8_t)id;
        ack[n++] = (uint8_t)(id >> 8);
    }

    HAL_GPIO_IR_ENABLE_set();                         /* sensor bias OFF (blind to our LED) */
    HAL_GPIO_RED_drvstr(1);
    HAL_GPIO_RED_pmuxen(HAL_GPIO_PMUX_SERCOM);        /* engage LED on SERCOM0 TX */

    tx_write(ack, n);

    HAL_GPIO_RED_pmuxdis();                           /* disengage... */
    HAL_GPIO_RED_off();                               /* ...LED off (Hi-Z) */
    HAL_GPIO_IR_ENABLE_clr();                         /* sensor bias ON again */
    rx_flush();
    parser_reset();
}

#endif /* __arm__ */

/* ---- Session state (shared by both modes) ---------------------------- */

/* Id of the last block actually committed (uf2 row written, or the final patch
 * body frame of a completed apply), or -1 before the first one. Drives
 * stop-and-wait dedup/ordering so a resent block (whose ACK we lost) is re-ACKed
 * without redoing the write, and a uf2-like restart (id 0) reflashes. */
static int32_t s_last_id;

/* Current data mode: true = applying a patch, false = writing verbatim uf2-like
 * rows. Seeded in flasher_run from whether a (verified) patch descriptor was
 * handed in, NOT from any block flag. It can flip patch -> uf2 (never back) on a
 * uf2-like block 0: mid-apply via the pull source's takeover abort, afterwards
 * via commit_block. */
static bool s_patch_mode;

/* ===================================================================== *
 *  Patch byte source (pull; shared by every decoder backend)
 *
 *  The patch decoder PULLS its compressed input one byte at a time through
 *  patch_pull_byte(), whose contract deliberately matches UltraPatch's
 *  PatchPull callback (int (*)(void *ctx, uint8_t *out); write one byte and
 *  return FLASHER_PULL_BYTE, or FLASHER_PULL_END to abort the decode), so
 *  either decoder backend plugs into the same source unchanged.
 *
 *  The source serves the current body frame (s_body); at a frame boundary it
 *  ACKs the frame it just exhausted -- every byte of it has been consumed AND
 *  acted on, because a pull decoder only asks for more input after applying
 *  what it already read, so stop-and-wait's durable-progress promise holds --
 *  then sleeps in the raw RX loop until the next-in-sequence body frame
 *  arrives. The stop-and-wait dedup/order policy is applied right here:
 *      * duplicate of the current frame -> re-ACK only (our ACK was lost)
 *      * next-in-sequence patch frame   -> becomes the current frame
 *      * NON-patch id-0 data block      -> TAKEOVER: a fresh full flash is
 *        starting over this half-finished patch (the only way to restart a
 *        patch, which applies in place). The source parks the frame in P,
 *        latches s_takeover, and returns FLASHER_PULL_END; the decoder
 *        unwinds and flasher_run restarts in full-flash mode on that block.
 *      * anything else (out-of-sequence, wrong kind, premature EXIT) ->
 *        do NOTHING (no ACK), keep listening; silence makes the host resend.
 * ===================================================================== */

/* Pull-source verdicts (FLASHER_PULL_END / _BYTE) come from the shared
 * header; they match UltraPatch's PATCH_PULL_END / _BYTE. */

/* Current compressed body frame, fed to the decoder backend. */
static uint8_t  s_body[FLASHER_ROW_SIZE];
static uint16_t s_body_len, s_body_pos;
static uint16_t s_body_id;    /* frame id of the current body frame */

/* The pull source aborted with a full-flash takeover: the uf2 block-0 frame is
 * parked in P for flasher_run to act on once the decoder has unwound. */
static bool s_takeover;

int patch_pull_byte(void *ctx, uint8_t *out) {
    (void) ctx;
    while (s_body_pos >= s_body_len) {
        send_ack(s_body_id);                       /* current frame consumed */
        for (;;) {
            uint8_t b;
            if (!rx_read_byte(&b)) { wfi_standby(); continue; }
            if (!parser_feed(b)) continue;
            if (P.out_flags & IR_FLASHER_FLAG_VERIFY) continue;  /* premature EXIT: ignore */
            bool is_patch = (P.out_flags & IR_FLASHER_FLAG_PATCH) != 0;
            if (!is_patch) {
                if (P.out_id == 0 && P.out_len == IR_FLASHER_BLOCK_PAYLOAD) {
                    s_takeover = true;             /* full-flash takeover; frame parked in P */
                    return FLASHER_PULL_END;
                }
                continue;                          /* other non-patch frame: ignore */
            }
            if (P.out_id == s_body_id) { send_ack(s_body_id); continue; }  /* dup */
            if (P.out_id != (uint16_t)(s_body_id + 1)) continue;  /* out-of-sequence: ignore */
            if (P.out_len > FLASHER_ROW_SIZE) continue;  /* oversized for a body frame: ignore */
            s_body_id  = P.out_id;
            s_body_len = P.out_len;
            for (uint16_t k = 0; k < s_body_len; k++) s_body[k] = P.crcbuf[4 + k];
            s_body_pos = 0;
            break;   /* the while re-checks: a zero-length frame just gets ACKed */
        }
    }
    *out = s_body[s_body_pos++];
    return FLASHER_PULL_BYTE;
}

/* ===================================================================== *
 *  Frame dispatch + main receive loop
 * ===================================================================== */

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
 * NOTE: the live patch apply runs entirely in patch_backend_run (it consumes the
 * body stream through the pull source), so commit_block only ever handles
 * uf2-like rows or a stray patch frame arriving after the apply. It writes uf2
 * rows; a stray patch frame is ignored (only a duplicate of the last id is
 * re-ACKed); it must never apply. */
static void commit_block(uint16_t id, bool is_patch,
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
    /* A non-duplicate PATCH frame here can only be a stray AFTER the apply has
     * run (the live patch stream is consumed through the pull source, never
     * here). We must NOT "apply" it: ACKing it would mask a failed apply by
     * waving the host through to an EXIT-CRC hang. Ignore it; the host stays
     * honestly stuck on the failing frame, and a uf2 block-0 (handled above)
     * still recovers via reflash. */
    if (is_patch) {
        return;
    }
    /* uf2-like row from here on (is_patch == s_patch_mode == false). */
    bool is_next    = (id == (uint16_t)(s_last_id + 1));
    bool is_restart = (id == 0);      /* a full flash may restart from block 0 */
    if (!is_next && !is_restart) {
        return;                       /* out-of-sequence: ignore */
    }
    if (firmware_flasher_write_row(addr, row, 0u /* link-paced */)) {
        s_last_id = (int32_t)id;
        send_ack(id);
    }
    /* else: commit failed -> stay silent, last_id unchanged -> retransmit */
}

/* Act on the frame currently held in P. Returns true only when an EXIT frame
 * confirmed the whole image (the caller then reboots into the new firmware). */
static bool handle_frame(void) {
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
        if (firmware_flasher_crc32((const uint8_t *)base, len) == want) {
            send_ack(P.out_id);   /* echo the id => "passed"; caller reboots */
            return true;
        }
        /* Mismatch: stay silent. The host gets no echo, prompts the user, and on
         * a resend we overwrite toward a valid image rather than rebooting into a
         * brick. The watch stays in the flasher either way. */
        return false;
    }

    if (P.out_flags & IR_FLASHER_FLAG_PATCH) {
        /* A patch frame seen by this loop is a re-sent LAST body frame whose ACK
         * was lost (the live patch stream is consumed through the pull source,
         * not here): commit_block re-ACKs it as a duplicate (id == s_last_id) and
         * ignores any other patch frame. It has no addr/row, so we pass 0/NULL. */
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

/* Overlay stack canary: firmware_flasher_overlay_load() plants a word just
 * past the loaded overlay; if the session's stack ever dug that deep, the
 * loaded code itself may be corrupt, so the deepest call chain (the decoder
 * backend) is not trusted on a dead canary and the session parks instead.
 * Best-effort: a real overrun has likely crashed already, and the EXIT CRC
 * remains the authoritative image gate. Hosted test builds have no overlay. */
#ifdef __arm__
static bool flasher_overlay_canary_ok(void) {
    return *flasher_overlay_canary_addr() == FLASHER_OVERLAY_CANARY;
}
#else
static bool flasher_overlay_canary_ok(void) { return true; }   /* no overlay off-ARM */
#endif

/* The RAM-resident main loop. For a uf2 session, commits the first block then
 * receives/commits the rest; for a patch session, primes the pull source with
 * the handed-off first body frame and runs the decoder backend to completion,
 * then (either way) waits for the EXIT verify. NEVER RETURNS: there is
 * intentionally no stall timeout (see the settle note above): an abandoned
 * flash stays parked here, recoverable over the link, rather than rebooting
 * into a half-written image. */
void flasher_run(const firmware_flasher_patch_t *patch,
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
        /* Prime the pull source with the first body frame (received by the
         * launcher but NOT yet ACKed; the source ACKs it once consumed). */
        s_body_id  = first_block_id;
        s_body_len = first_block_len;
        for (uint16_t k = 0; k < first_block_len; k++) s_body[k] = first_block[k];
        s_body_pos = 0;
        s_takeover = false;
        if (patch_backend_run(patch) && flasher_overlay_canary_ok()) {
            send_ack(s_body_id);   /* ACK the final body frame (consumed, not yet ACKed) */
            s_last_id = (int32_t)s_body_id;   /* so a re-sent last body frame re-ACKs */
        } else if (s_takeover) {
            /* A fresh full flash is starting over this half-finished patch: the
             * pull source unwound the decoder and parked the uf2 block-0 frame
             * in P. Route it through handle_frame like any received frame;
             * commit_block flips the mode and commits it as full block 0. */
            handle_frame();
        }
        /* On any other failure the flash is half-patched; we stay parked (no
         * reboot, no ACK) and a uf2 block-0 fallback (via commit_block below)
         * can still recover with a full reflash. Reset the parser for the clean
         * EXIT-wait loop. */
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

#endif /* (arm && HAS_IR_SENSOR) || hosted test build */
