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

/*
 * watch_optical_config.h
 *
 * Board-specific wiring of the optical (IR / LED) half-duplex serial link used
 * by watch_optical, movement_optical and the firmware flasher. This is the ONE
 * place that says which LED, which phototransistor, and which SERCOM pads a
 * given board uses; every consumer derives what it needs from the macros below.
 *
 * gossamer's per-board pins.h names the pins (HAL_GPIO_PIN(RED, ...), etc.) but
 * knows nothing about which SERCOM / pad / pin-mux the link rides on, and we do
 * not control that repository. So the mapping lives here, keyed on the board
 * name that the Makefile passes in as -DBOARD_$(BOARD).
 *
 * To add a board: add an #elif defined(BOARD_<name>) block that defines
 * HAS_OPTICAL_LINK plus the WATCH_OPTICAL_* knobs. Boards without the hardware
 * simply have no block; HAS_OPTICAL_LINK stays undefined and every optical
 * consumer compiles down to its stub.
 *
 * Knobs (all required when HAS_OPTICAL_LINK is defined, except RX_ENABLE_*):
 *   WATCH_OPTICAL_TX_SERCOM      SERCOM index driving the LED (USART TX)
 *   WATCH_OPTICAL_TX_PAD         SERCOM pad the LED pin lands on (0 or 2 on SAM L22)
 *   WATCH_OPTICAL_TX_PIN         pin name as declared in pins.h (e.g. RED)
 *   WATCH_OPTICAL_TX_PMUX        HAL_GPIO_PMUX_SERCOM or HAL_GPIO_PMUX_SERCOM_ALT
 *   WATCH_OPTICAL_RX_SERCOM      SERCOM index reading the phototransistor (USART RX)
 *   WATCH_OPTICAL_RX_PAD         SERCOM pad the sensor pin lands on (0..3)
 *   WATCH_OPTICAL_RX_PIN         pin name as declared in pins.h (e.g. IRSENSE)
 *   WATCH_OPTICAL_RX_PMUX        HAL_GPIO_PMUX_SERCOM or HAL_GPIO_PMUX_SERCOM_ALT
 *   WATCH_OPTICAL_RX_ENABLE_PIN  (optional) pin gating the phototransistor bias
 *   WATCH_OPTICAL_RX_ENABLE_ACTIVE_LOW  1 if driving it low powers the bias ON
 *
 * NB: HAS_OPTICAL_LINK is deliberately distinct from gossamer's HAS_IR_SENSOR.
 * The latter means "the phototransistor exists and can be read via the ADC"
 * (light_sensor_face); this one means "TX + RX are wired to SERCOMs and the
 * serial link is usable". A board may have one without the other.
 */

/* ===================================================================== *
 *  Per-board wiring                                                     *
 * ===================================================================== */

#if defined(BOARD_sensorwatch_pro)
/* Sensor Watch Pro:
 *   TX: PA12 (RED LED), mux D -> SERCOM3 PAD[0]
 *   RX: PA04 (IRSENSE), mux D -> SERCOM0 PAD[0]
 *   Phototransistor bias enable on PB22 (IR_ENABLE), active-low. */
#define HAS_OPTICAL_LINK
#define WATCH_OPTICAL_TX_SERCOM              3
#define WATCH_OPTICAL_TX_PAD                 0
#define WATCH_OPTICAL_TX_PIN                 RED
#define WATCH_OPTICAL_TX_PMUX                HAL_GPIO_PMUX_SERCOM_ALT
#define WATCH_OPTICAL_RX_SERCOM              0
#define WATCH_OPTICAL_RX_PAD                 0
#define WATCH_OPTICAL_RX_PIN                 IRSENSE
#define WATCH_OPTICAL_RX_PMUX                HAL_GPIO_PMUX_SERCOM_ALT
#define WATCH_OPTICAL_RX_ENABLE_PIN          IR_ENABLE
#define WATCH_OPTICAL_RX_ENABLE_ACTIVE_LOW   1

/* New boards go above this line. */
#endif

/* ===================================================================== *
 *  Consumer API (derived from the knobs above; no per-board content)    *
 * ===================================================================== *
 *
 * Consumers use ONLY these names (plus the raw WATCH_OPTICAL_{TX,RX}_SERCOM
 * indices for uart2's `.sercom` field):
 *
 *   WATCH_OPTICAL_TX_SERCOM_INST      Sercom *            e.g. SERCOM3
 *   WATCH_OPTICAL_RX_SERCOM_INST      Sercom *            e.g. SERCOM0
 *   WATCH_OPTICAL_RX_SERCOM_IRQ       IRQn_Type           e.g. SERCOM0_IRQn
 *   WATCH_OPTICAL_TX_TXPO             uart2 TX pad-out    e.g. UART2_TXPO_0
 *   WATCH_OPTICAL_RX_RXPO             uart2 RX pad-in     e.g. UART2_RXPO_0
 *   WATCH_OPTICAL_TX_PIN_pmuxen()     route LED pin to its SERCOM (mux baked in)
 *   WATCH_OPTICAL_TX_PIN_pmuxdis()    back to plain GPIO
 *   WATCH_OPTICAL_TX_PIN_drvstr(v)    drive strength
 *   WATCH_OPTICAL_TX_PIN_off()        Hi-Z
 *   WATCH_OPTICAL_RX_PIN_pmuxen()     route sensor pin to its SERCOM (mux baked in)
 *   WATCH_OPTICAL_RX_PIN_pmuxdis()    back to plain GPIO
 *   WATCH_OPTICAL_RX_PIN_in()         input
 *   WATCH_OPTICAL_RX_PIN_off()        Hi-Z
 *   WATCH_OPTICAL_RX_bias_on()        power the phototransistor bias (drives the
 *                                     enable pin as an output at its active level)
 *   WATCH_OPTICAL_RX_bias_off()       inactive level; a single register write, the
 *                                     flasher toggles it around every ACK
 *   WATCH_OPTICAL_RX_bias_release()   Hi-Z the enable pin (session teardown)
 *
 * The three bias_* calls are no-ops on a board with no enable pin.
 *
 * WOC_* macros are internal glue: never use them outside this file.
 */

#ifdef HAS_OPTICAL_LINK

/* Internal. Two-level token pasting so the knobs expand before being glued:
 *   WOC_JOIN(SERCOM, WATCH_OPTICAL_TX_SERCOM)           -> SERCOM3
 *   WOC_JOIN3(SERCOM, WATCH_OPTICAL_RX_SERCOM, _IRQn)   -> SERCOM0_IRQn
 *   WOC_PIN(WATCH_OPTICAL_TX_PIN, _off)                 -> HAL_GPIO_RED_off
 * (JOIN3 exists because SERCOM0 etc. are themselves macros: nesting two JOINs
 * would expand SERCOM0 to its pointer cast before _IRQn could be pasted on.) */
#define WOC_JOIN_(a, b)        a##b
#define WOC_JOIN(a, b)         WOC_JOIN_(a, b)
#define WOC_JOIN3_(a, b, c)    a##b##c
#define WOC_JOIN3(a, b, c)     WOC_JOIN3_(a, b, c)
#define WOC_PIN(pin, op)       WOC_JOIN3(HAL_GPIO_, pin, op)

#define WATCH_OPTICAL_TX_SERCOM_INST   WOC_JOIN(SERCOM, WATCH_OPTICAL_TX_SERCOM)
#define WATCH_OPTICAL_RX_SERCOM_INST   WOC_JOIN(SERCOM, WATCH_OPTICAL_RX_SERCOM)
#define WATCH_OPTICAL_RX_SERCOM_IRQ    WOC_JOIN3(SERCOM, WATCH_OPTICAL_RX_SERCOM, _IRQn)

/* A pad the SERCOM cannot use for that direction (e.g. TX on pad 1) has no
 * UART2_TXPO_* enumerator and fails to compile here, which is the point. */
#define WATCH_OPTICAL_TX_TXPO          WOC_JOIN(UART2_TXPO_, WATCH_OPTICAL_TX_PAD)
#define WATCH_OPTICAL_RX_RXPO          WOC_JOIN(UART2_RXPO_, WATCH_OPTICAL_RX_PAD)

#define WATCH_OPTICAL_TX_PIN_pmuxen()  WOC_PIN(WATCH_OPTICAL_TX_PIN, _pmuxen)(WATCH_OPTICAL_TX_PMUX)
#define WATCH_OPTICAL_TX_PIN_pmuxdis() WOC_PIN(WATCH_OPTICAL_TX_PIN, _pmuxdis)()
#define WATCH_OPTICAL_TX_PIN_drvstr(v) WOC_PIN(WATCH_OPTICAL_TX_PIN, _drvstr)(v)
#define WATCH_OPTICAL_TX_PIN_off()     WOC_PIN(WATCH_OPTICAL_TX_PIN, _off)()

#define WATCH_OPTICAL_RX_PIN_pmuxen()  WOC_PIN(WATCH_OPTICAL_RX_PIN, _pmuxen)(WATCH_OPTICAL_RX_PMUX)
#define WATCH_OPTICAL_RX_PIN_pmuxdis() WOC_PIN(WATCH_OPTICAL_RX_PIN, _pmuxdis)()
#define WATCH_OPTICAL_RX_PIN_in()      WOC_PIN(WATCH_OPTICAL_RX_PIN, _in)()
#define WATCH_OPTICAL_RX_PIN_off()     WOC_PIN(WATCH_OPTICAL_RX_PIN, _off)()

#ifdef WATCH_OPTICAL_RX_ENABLE_PIN
#  if WATCH_OPTICAL_RX_ENABLE_ACTIVE_LOW
#    define WOC_BIAS_ACTIVE   _clr     /* internal: hal_gpio op that turns the bias ON */
#    define WOC_BIAS_INACTIVE _set     /* internal: ... and OFF */
#  else
#    define WOC_BIAS_ACTIVE   _set
#    define WOC_BIAS_INACTIVE _clr
#  endif
#  define WATCH_OPTICAL_RX_bias_on()      do { WOC_PIN(WATCH_OPTICAL_RX_ENABLE_PIN, _out)(); \
                                               WOC_PIN(WATCH_OPTICAL_RX_ENABLE_PIN, WOC_BIAS_ACTIVE)(); } while (0)
#  define WATCH_OPTICAL_RX_bias_off()     WOC_PIN(WATCH_OPTICAL_RX_ENABLE_PIN, WOC_BIAS_INACTIVE)()
#  define WATCH_OPTICAL_RX_bias_release() WOC_PIN(WATCH_OPTICAL_RX_ENABLE_PIN, _off)()
#else
#  define WATCH_OPTICAL_RX_bias_on()      ((void)0)
#  define WATCH_OPTICAL_RX_bias_off()     ((void)0)
#  define WATCH_OPTICAL_RX_bias_release() ((void)0)
#endif

#endif /* HAS_OPTICAL_LINK */
