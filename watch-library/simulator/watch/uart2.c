/*
    Bits cribbed from Alex Taradov's BSD-3 licensed UART peripheral,
    Copyright (c) 2017-2023, Alex Taradov <alex@taradov.com>. All rights reserved.
    Full text available at: https://opensource.org/licenses/BSD-3
    Other stuff is MIT-licensed by me, Joey Castillo.
    Copyright 2023 Joey Castillo for Oddly Specific Objects.
    Published under the standard MIT License.
    Full text available at: https://opensource.org/licenses/MIT
*/

#include "uart2.h"

bool uart2_open(uart2_txpo_t txpo, const uart2_sercom_config_t *tx,
               uart2_rxpo_t rxpo, const uart2_sercom_config_t *rx) {
    (void)txpo; (void)tx; (void)rxpo; (void)rx;
    return true;
}
void uart2_close(void) {}
size_t uart2_rx_bytes_pending(void) { return 0; }
size_t uart2_rx_read(void *buf, size_t max) { (void)buf; (void)max; return 0; }
size_t uart2_tx_write(const void *buf, size_t len) { (void)buf; return len; }
size_t uart2_tx_bytes_pending(void) { return 0; }

void uart2_irq_handler(uint8_t sercom) { (void)sercom; }
