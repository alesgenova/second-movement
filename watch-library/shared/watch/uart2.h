////< @file uart2.h
/*
 * MIT License
 *
 * Copyright (c) 2024 Joey Castillo
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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @addtogroup uart UART Peripheral
 * @brief Polled, FIFO-buffered UART driver with optional STANDBY operation.
 * @details A single open call configures one or both directions, which may
 *          live on the same SERCOM (full duplex) or on different SERCOMs.
 *          Each direction has a fixed 256-byte software FIFO.
 *
 *          The consumer API is fully polled — no user code runs in ISR
 *          context. An internal ISR moves bytes between the SERCOM and the
 *          FIFO; with `run_in_standby = true` it continues to run during
 *          STANDBY, briefly waking the CPU to drain/fill and returning to
 *          STANDBY.
 *
 *          Consumers must poll fast enough to drain RX before the FIFO
 *          overflows. At 9600 baud the FIFO fills in ~267 ms, so a ≥ 4 Hz
 *          tick is required at that baud. `uart2_tx_write` never blocks: it
 *          reports how many bytes it could queue and the caller buffers any
 *          remainder. Pin pmux configuration is the caller's responsibility;
 *          the driver only touches the SERCOM.
 * @{
 */

/// @brief UART transmit pinout options.
typedef enum {
    UART2_TXPO_0 = 0,
    UART2_TXPO_2,
    UART2_TXPO_0_FLOW_CONTROL,
    UART2_TXPO_NONE = 0xff
} uart2_txpo_t;

/// @brief UART receive pinout options.
typedef enum {
    UART2_RXPO_0 = 0,
    UART2_RXPO_1,
    UART2_RXPO_2,
    UART2_RXPO_3,
    UART2_RXPO_NONE = 0xff
} uart2_rxpo_t;

/**
 * @brief Per-direction SERCOM configuration for one side of a UART link.
 * @details One instance describes the settings for a single SERCOM operating
 *          in UART mode. The same struct shape is reused for both RX and TX
 *          sides — pads (rxpo/txpo) are passed separately to `uart2_open`
 *          because they have direction-specific enum types.
 *
 *          For same-SERCOM duplex, callers may pass the same pointer for
 *          both `rx` and `tx` arguments — pointer equality is treated as an
 *          explicit "duplex on one config" signal and skips the per-field
 *          cross-check.
 */
typedef struct {
    uint8_t  sercom;            ///< SERCOM number (0..5).
    uint32_t baud;              ///< Desired baud rate.
    bool     irda;              ///< Enable IrDA encoding/decoding (CTRLB.ENC).
    bool     invert;            ///< Invert the line (CTRLA.RXINV for RX side, CTRLA.TXINV for TX side).
    bool     run_in_standby;    ///< Keep the FIFO clocked during MCU STANDBY.
} uart2_sercom_config_t;

/**
 * @brief Opens a UART link with one or both directions enabled.
 * @details Pass NULL for `rx` to leave RX disabled, NULL for `tx` to leave
 *          TX disabled. At least one direction must be non-NULL. When a
 *          direction is enabled, its pad selector must not be `*_NONE`.
 *
 *          When `rx` and `tx` are both non-NULL with the same `sercom`
 *          value (same-SERCOM duplex), the shared register fields must
 *          agree: `baud`, `irda`, and `run_in_standby` must match between
 *          the two. The `invert` flag is per-direction in hardware (TXINV
 *          and RXINV are separate CTRLA bits) and may differ. Passing the
 *          same pointer for both `rx` and `tx` satisfies the cross-check
 *          trivially.
 *
 *          The call fails if a UART link is already open — `uart2_close()`
 *          must be called explicitly before reconfiguring.
 *
 *          Pin pmux configuration is the caller's responsibility; the
 *          driver only touches the SERCOM.
 * @param txpo Pad selection for TX (ignored if `tx` is NULL).
 * @param tx   TX configuration, or NULL to disable TX.
 * @param rxpo Pad selection for RX (ignored if `rx` is NULL).
 * @param rx   RX configuration, or NULL to disable RX.
 * @return true on success; false if the configuration is invalid (no
 *         direction enabled, sercom out of range, pad inconsistent with
 *         pointer, or same-SERCOM duplex with mismatched shared params).
 *         On false return, no state has changed.
 */
bool uart2_open(uart2_txpo_t txpo, const uart2_sercom_config_t *tx,
               uart2_rxpo_t rxpo, const uart2_sercom_config_t *rx);

/**
 * @brief Closes the UART link, tearing down both directions if both are open.
 * @details Disables the SERCOM(s), releases the standby clock chain hold(s)
 *          taken at open time, and resets the FIFOs. Unread RX bytes and
 *          unsent TX bytes are discarded. Callers needing the TX line
 *          flushed should poll `uart2_tx_bytes_pending() == 0` before close.
 *          Safe to call when nothing is open.
 */
void uart2_close(void);

/// @brief Returns the number of bytes currently waiting in the RX FIFO.
size_t uart2_rx_bytes_pending(void);

/**
 * @brief Drains up to `max` bytes from the RX FIFO into `buf`.
 * @return The number of bytes actually copied.
 */
size_t uart2_rx_read(void *buf, size_t max);

/**
 * @brief Queues up to `len` bytes for transmission.
 * @details Never blocks. Returns the number of bytes actually queued; if
 *          less than `len`, the caller buffers the remainder and retries on
 *          a later tick.
 */
size_t uart2_tx_write(const void *buf, size_t len);

/**
 * @brief Returns the number of bytes the caller has handed off that have
 *        not yet fully transmitted.
 * @details Counts bytes still in the TX FIFO, in the SERCOM DATA register,
 *          and being clocked out of the shift register. Returns 0 once the
 *          line is fully idle — use `uart2_tx_bytes_pending() == 0` as the
 *          "transmission complete" check. Also returns 0 before the first
 *          byte has been queued in the current session (so the count
 *          reflects "what the caller is still waiting on").
 */
size_t uart2_tx_bytes_pending(void);

/**
 * @brief Internal IRQ entry point.
 * @details Called from the `irq_handler_sercom0..5` defaults provided by
 *          `uart.c`. Exposed so that a driver overriding one of those
 *          handlers can still drive the FIFO.
 */
void uart2_irq_handler(uint8_t sercom);

/** @} */
