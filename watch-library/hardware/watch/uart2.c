/*
    Bits cribbed from Alex Taradov's BSD-3 licensed UART peripheral,
    Copyright (c) 2017-2023, Alex Taradov <alex@taradov.com>. All rights reserved.
    Full text available at: https://opensource.org/licenses/BSD-3
    Other stuff is MIT-licensed by me, Joey Castillo.
    Copyright 2023 Joey Castillo for Oddly Specific Objects.
    Published under the standard MIT License.
    Full text available at: https://opensource.org/licenses/MIT
*/

#include "pins.h"
#include "sam.h"
#include "system.h"
#include "sercom.h"
#include "uart2.h"

#ifndef SERCOM_USART_CTRLA_MODE_USART_INT_CLK
#define SERCOM_USART_CTRLA_MODE_USART_INT_CLK SERCOM_USART_CTRLA_MODE(1)
#endif

#define UART_BUF_SIZE 256

typedef struct {
    int     wr;
    int     rd;
    uint8_t data[UART_BUF_SIZE];
} fifo_t;

static volatile fifo_t rx_fifo;
static volatile fifo_t tx_fifo;

/* Current owners of the single RX / TX channel. -1 = closed. When both are
 * non-negative and equal, the link is same-SERCOM duplex. */
static volatile int8_t rx_sercom = -1;
static volatile int8_t tx_sercom = -1;

/* False until the first byte has been queued on the current TX session.
 * Gates the TXC hardware check in uart2_tx_bytes_pending: INTFLAG.TXC reads
 * 0 after enable until a frame fully shifts out, so without this gate
 * uart2_tx_bytes_pending would falsely report 1 byte pending on a freshly
 * opened (and never-written) channel. */
static volatile bool tx_started = false;

/* Tracks whether the global STANDBY clock chain (OSC16M + GCLK0) is
 * currently held. Set by chain_acquire when uart2_open detects any direction
 * needs standby; cleared by chain_release in uart2_close. Per-SERCOM
 * CTRLA.RUNSTDBY is set directly by configure_sercom (it's enable-protected
 * but the SERCOM is disabled during configure, so the write sticks). */
static bool chain_held = false;

/* ---- FIFO ----------------------------------------------------------- */

static bool fifo_push(volatile fifo_t *fifo, uint8_t value) {
    int next_wr = (fifo->wr + 1) % UART_BUF_SIZE;
    if (next_wr == fifo->rd) return false;
    fifo->data[fifo->wr] = value;
    fifo->wr = next_wr;
    return true;
}

static bool fifo_pop(volatile fifo_t *fifo, uint8_t *value) {
    if (fifo->rd == fifo->wr) return false;
    *value = fifo->data[fifo->rd];
    fifo->rd = (fifo->rd + 1) % UART_BUF_SIZE;
    return true;
}

static size_t fifo_count(volatile const fifo_t *fifo) {
    return (size_t)((fifo->wr - fifo->rd + UART_BUF_SIZE) % UART_BUF_SIZE);
}

static void fifo_reset(volatile fifo_t *fifo) {
    fifo->wr = 0;
    fifo->rd = 0;
}

/* ---- Standby clock chain (OSC16M + GCLK0) hold ----------------------
 *
 * On SAM L21 / L22, the SERCOM is fed by OSC16M → GCLK0. Both upstream
 * stages gate off during STANDBY by default and must have RUNSTDBY set
 * for the SERCOM to keep clocking. If you add support for a new chip
 * here, mirror its define in BOTH guards below — a silent no-op on the
 * target chip means RX bytes vanish during STANDBY and TX never drains.
 *
 * Because uart2_open rejects re-open while a session is active, there is
 * at most one open session at a time, so a simple bool replaces what was
 * previously a refcount.
 */

static void chain_acquire(void) {
    if (chain_held) return;
#if defined(_SAML21_) || defined(_SAML22_)
    OSCCTRL->OSC16MCTRL.bit.RUNSTDBY = 1;
    GCLK->GENCTRL[0].reg |= GCLK_GENCTRL_RUNSTDBY;
    while (GCLK->SYNCBUSY.reg & (1ul << GCLK_SYNCBUSY_GENCTRL0_Pos)) {}
#endif
    chain_held = true;
}

static void chain_release(void) {
    if (!chain_held) return;
#if defined(_SAML21_) || defined(_SAML22_)
    GCLK->GENCTRL[0].reg &= ~GCLK_GENCTRL_RUNSTDBY;
    while (GCLK->SYNCBUSY.reg & (1ul << GCLK_SYNCBUSY_GENCTRL0_Pos)) {}
    OSCCTRL->OSC16MCTRL.bit.RUNSTDBY = 0;
#endif
    chain_held = false;
}

/* ---- Configure SERCOM USART for one or both directions -------------- */

/* `txpo == UART2_TXPO_NONE` disables TX; `rxpo == UART2_RXPO_NONE` disables
 * RX. Resets the SERCOM via SWRST, so any prior configuration on the same
 * SERCOM is lost. Leaves the SERCOM disabled so caller can program any
 * remaining enable-protected bits (e.g. TXINV / RXINV) before enabling. */
static void configure_sercom(uint8_t sercom, uart2_txpo_t txpo, uart2_rxpo_t rxpo,
                             uint32_t baud, bool irda, bool run_in_standby) {
    Sercom *S = SERCOM_Peripherals[sercom].sercom;

    _sercom_clock_setup(sercom);

    S->USART.CTRLA.bit.ENABLE = 0;
    while (S->USART.SYNCBUSY.bit.ENABLE) {}
    S->USART.CTRLA.bit.SWRST = 1;
    while (S->USART.SYNCBUSY.bit.SWRST || S->USART.SYNCBUSY.bit.ENABLE) {}

    bool tx_enable = (txpo != UART2_TXPO_NONE);
    bool rx_enable = (rxpo != UART2_RXPO_NONE);
    if (!tx_enable) txpo = 0;
    if (!rx_enable) rxpo = 0;

    S->USART.CTRLA.reg = SERCOM_USART_CTRLA_FORM(0)
                       | SERCOM_USART_CTRLA_MODE_USART_INT_CLK
                       | SERCOM_USART_CTRLA_DORD
                       | SERCOM_USART_CTRLA_SAMPR(0)
                       | SERCOM_USART_CTRLA_TXPO(txpo)
                       | SERCOM_USART_CTRLA_RXPO(rxpo);

    S->USART.CTRLB.reg = (tx_enable ? SERCOM_USART_CTRLB_TXEN : 0)
                       | (rx_enable ? SERCOM_USART_CTRLB_RXEN : 0)
                       | (irda      ? SERCOM_USART_CTRLB_ENC  : 0);
    while (S->USART.SYNCBUSY.bit.CTRLB) {}

    uint32_t cpu_speed = get_cpu_frequency();
    uint64_t br = (uint64_t)65536 * (cpu_speed - 16 * baud) / cpu_speed;
    S->USART.BAUD.bit.BAUD = (uint16_t)(br + 1);

    /* CTRLA.RUNSTDBY is enable-protected; SERCOM is still disabled here so
     * the write sticks. The upstream clock chain is acquired/released once
     * per session by uart2_open / uart2_close, not per-SERCOM. */
    if (run_in_standby) {
        S->USART.CTRLA.bit.RUNSTDBY = 1;
    }
}

/* ---- Validation ----------------------------------------------------- */

static bool validate_config(uart2_txpo_t txpo, const uart2_sercom_config_t *tx,
                            uart2_rxpo_t rxpo, const uart2_sercom_config_t *rx) {
    if (!rx && !tx) return false;

    if (rx) {
        if (rx->sercom >= 6) return false;
        if (rxpo == UART2_RXPO_NONE) return false;
    }
    if (tx) {
        if (tx->sercom >= 6) return false;
        if (txpo == UART2_TXPO_NONE) return false;
    }

    /* Same-SERCOM duplex: shared register fields must agree. invert is
     * per-direction in hardware (separate TXINV and RXINV bits) so it can
     * differ. Same-pointer case trivially satisfies the check. */
    if (rx && tx && rx->sercom == tx->sercom && rx != tx) {
        if (rx->baud != tx->baud) return false;
        if (rx->irda != tx->irda) return false;
        if (rx->run_in_standby != tx->run_in_standby) return false;
    }

    return true;
}

/* ---- Open / close --------------------------------------------------- */

bool uart2_open(uart2_txpo_t txpo, const uart2_sercom_config_t *tx,
               uart2_rxpo_t rxpo, const uart2_sercom_config_t *rx) {
    /* Reject re-open while a session is active; caller must uart2_close
     * first. This keeps state and standby ownership unambiguous. */
    if (rx_sercom >= 0 || tx_sercom >= 0) return false;

    if (!validate_config(txpo, tx, rxpo, rx)) return false;

    bool duplex = (rx && tx && rx->sercom == tx->sercom);

    if (duplex) {
        uint8_t sercom = rx->sercom;
        configure_sercom(sercom, txpo, rxpo, rx->baud, rx->irda, rx->run_in_standby);

        Sercom *S = SERCOM_Peripherals[sercom].sercom;
        uint32_t ctrla_or = 0;
        if (rx->invert) ctrla_or |= SERCOM_USART_CTRLA_RXINV;
        if (tx->invert) ctrla_or |= SERCOM_USART_CTRLA_TXINV;
        if (ctrla_or) S->USART.CTRLA.reg |= ctrla_or;

        S->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;

        NVIC_ClearPendingIRQ(SERCOM_Peripherals[sercom].interrupt_line);
        NVIC_EnableIRQ(SERCOM_Peripherals[sercom].interrupt_line);

        _sercom_enable(sercom);
        rx_sercom = (int8_t)sercom;
        tx_sercom = (int8_t)sercom;
    } else {
        if (rx) {
            uint8_t sercom = rx->sercom;
            configure_sercom(sercom, UART2_TXPO_NONE, rxpo, rx->baud, rx->irda, rx->run_in_standby);

            Sercom *S = SERCOM_Peripherals[sercom].sercom;
            if (rx->invert) S->USART.CTRLA.reg |= SERCOM_USART_CTRLA_RXINV;

            S->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;

            NVIC_ClearPendingIRQ(SERCOM_Peripherals[sercom].interrupt_line);
            NVIC_EnableIRQ(SERCOM_Peripherals[sercom].interrupt_line);

            _sercom_enable(sercom);
            rx_sercom = (int8_t)sercom;
        }

        if (tx) {
            uint8_t sercom = tx->sercom;
            configure_sercom(sercom, txpo, UART2_RXPO_NONE, tx->baud, tx->irda, tx->run_in_standby);

            Sercom *S = SERCOM_Peripherals[sercom].sercom;
            if (tx->invert) S->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXINV;

            /* DRE INTENSET is enabled lazily by uart2_tx_write when there's
             * something to send; opening leaves it masked so the ISR doesn't
             * fire on an empty FIFO. */

            NVIC_ClearPendingIRQ(SERCOM_Peripherals[sercom].interrupt_line);
            NVIC_EnableIRQ(SERCOM_Peripherals[sercom].interrupt_line);

            _sercom_enable(sercom);
            tx_sercom = (int8_t)sercom;
        }
    }

    /* Acquire the STANDBY clock chain once if any direction needs it.
     * Released as a single step by uart2_close. */
    if ((rx && rx->run_in_standby) || (tx && tx->run_in_standby)) {
        chain_acquire();
    }

    return true;
}

static void teardown_sercom(uint8_t sercom) {
    Sercom *S = SERCOM_Peripherals[sercom].sercom;
    S->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_RXC | SERCOM_USART_INTENCLR_DRE;
    _sercom_disable(sercom);
    NVIC_DisableIRQ(SERCOM_Peripherals[sercom].interrupt_line);
}

void uart2_close(void) {
    bool duplex = (rx_sercom >= 0 && rx_sercom == tx_sercom);

    if (rx_sercom >= 0) {
        teardown_sercom((uint8_t)rx_sercom);
    }
    if (tx_sercom >= 0 && !duplex) {
        teardown_sercom((uint8_t)tx_sercom);
    }

    chain_release();

    rx_sercom = -1;
    tx_sercom = -1;
    tx_started = false;
    fifo_reset(&rx_fifo);
    fifo_reset(&tx_fifo);
}

/* ---- RX read -------------------------------------------------------- */

size_t uart2_rx_bytes_pending(void) {
    return fifo_count(&rx_fifo);
}

size_t uart2_rx_read(void *buf, size_t max) {
    if (rx_sercom < 0) return 0;
    uint8_t *out = (uint8_t *)buf;
    IRQn_Type line = SERCOM_Peripherals[(uint8_t)rx_sercom].interrupt_line;

    NVIC_DisableIRQ(line);
    size_t copied = 0;
    while (copied < max && fifo_pop(&rx_fifo, &out[copied])) copied++;
    NVIC_EnableIRQ(line);
    return copied;
}

/* ---- TX write ------------------------------------------------------- */

size_t uart2_tx_write(const void *buf, size_t len) {
    if (tx_sercom < 0) return 0;
    const uint8_t *in = (const uint8_t *)buf;
    uint8_t sercom = (uint8_t)tx_sercom;
    Sercom *S = SERCOM_Peripherals[sercom].sercom;
    IRQn_Type line = SERCOM_Peripherals[sercom].interrupt_line;

    NVIC_DisableIRQ(line);

    size_t queued;
    for (queued = 0; queued < len; queued++) {
        if (!fifo_push(&tx_fifo, in[queued])) break;
    }
    if (queued > 0) {
        tx_started = true;
        S->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
    }

    NVIC_EnableIRQ(line);
    return queued;
}

size_t uart2_tx_bytes_pending(void) {
    if (tx_sercom < 0 || !tx_started) return 0;
    Sercom *S = SERCOM_Peripherals[(uint8_t)tx_sercom].sercom;
    uint16_t flags = S->USART.INTFLAG.reg;
    size_t hw = ((flags & SERCOM_USART_INTFLAG_DRE) ? 0u : 1u)
              + ((flags & SERCOM_USART_INTFLAG_TXC) ? 0u : 1u);
    return fifo_count(&tx_fifo) + hw;
}

/* ---- IRQ ------------------------------------------------------------ */

void uart2_irq_handler(uint8_t sercom) {
    Sercom *S = SERCOM_Peripherals[sercom].sercom;
    uint16_t flags = S->USART.INTFLAG.reg;

    if (flags & SERCOM_USART_INTFLAG_RXC) {
        uint16_t status = S->USART.STATUS.reg;
        uint8_t byte = (uint8_t)S->USART.DATA.reg;
        S->USART.STATUS.reg = status;
        (void)fifo_push(&rx_fifo, byte);  /* dropped on overflow */
    }

    if (flags & SERCOM_USART_INTFLAG_DRE) {
        uint8_t byte;
        if (fifo_pop(&tx_fifo, &byte)) {
            S->USART.DATA.reg = byte;
        } else {
            S->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
        }
    }
}

/* ---- Default IRQ handlers ------------------------------------------- *
 *
 * These are STRONG definitions on purpose. The startup file declares
 * `irq_handler_sercomN` as `__attribute__((weak, alias("irq_handler_dummy")))`,
 * where `irq_handler_dummy` is `while(1);`. If we defined the handlers here
 * weakly as well, GNU ld would have two weak symbols and pick the first one
 * in link order — typically the startup file's, leaving the SERCOM IRQ
 * pointing at the infinite-loop dummy. The CPU would hang the instant a
 * byte arrived. Strong here overrides the startup's weak alias deterministically.
 *
 * If another driver ever needs to own one of these SERCOM IRQ handlers
 * (e.g. SPI/I2C in IRQ mode), the resolution is to route through
 * uart2_irq_handler from that driver's strong definition, not to fight
 * the linker with another weak.
 */
void irq_handler_sercom0(void);
void irq_handler_sercom1(void);
void irq_handler_sercom2(void);
void irq_handler_sercom3(void);
void irq_handler_sercom4(void);
void irq_handler_sercom5(void);

void irq_handler_sercom0(void) { uart2_irq_handler(0); }
void irq_handler_sercom1(void) { uart2_irq_handler(1); }
void irq_handler_sercom2(void) { uart2_irq_handler(2); }
void irq_handler_sercom3(void) { uart2_irq_handler(3); }
void irq_handler_sercom4(void) { uart2_irq_handler(4); }
void irq_handler_sercom5(void) { uart2_irq_handler(5); }
