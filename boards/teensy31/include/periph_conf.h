/*
 * Copyright (C) 2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_teensy31
 * @{
 *
 * @file
 * @name        Peripheral MCU configuration for the Teensy3.1 & 3.2
 *
 * @author      Loïc Dauphin <loic.dauphin@inria.fr>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @name Clock system configuration
 * @{
 */
static const clock_config_t clock_config = {
    // safe clock dividers for this CPU
    .clkdiv1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) |
               SIM_CLKDIV1_OUTDIV3(1) | SIM_CLKDIV1_OUTDIV4(2),
    // Select default clocking mode
    .default_mode = KINETIS_MCG_MODE_PEE,
    // The crystal connected to OSC0 is 8 MHz
    .erc_range = KINETIS_MCG_ERC_RANGE_HIGH,
    .fcrdiv = 0, // Fast IRC divide by 1 => 4 MHz
    .oscsel = 0, // Use OSC0 for external clock
    .clc = 0, // Use external load caps on board
    .fll_frdiv = 0b011, // Divide by 256
    .fll_factor_fei = KINETIS_MCG_FLL_FACTOR_1920, // FLL freq = 60 MHz ?
    .fll_factor_fee = KINETIS_MCG_FLL_FACTOR_1920, // FLL freq = 60 MHz
    .pll_prdiv = 0b00011, // Divide by 4 => PLL input freq = 2 MHz
    .pll_vdiv = 0b00000, // Multiply by 30 => PLL output freq = 60 MHz
    .enable_oscillator = true, // Enable oscillator, EXTAL0 is connected to a crystal
    .select_fast_irc = true, // Use fast IRC when in FBI mode
    .enable_mcgirclk = false, // We don't need the internal reference clock while running in PEE mode
};
#define CLOCK_CORECLOCK              (60000000ul)
#define CLOCK_BUSCLOCK               (CLOCK_CORECLOCK / 1)
/** @} */

/**
 * @name Timer configuration
 * @{
 */
#define PIT_NUMOF               (2U)
#define PIT_CONFIG {                 \
        {                            \
            .prescaler_ch = 0,       \
            .count_ch = 1,           \
        },                           \
        {                            \
            .prescaler_ch = 2,       \
            .count_ch = 3,           \
        },                           \
    }
#define LPTMR_NUMOF             (0U)
#define LPTMR_CONFIG { \
    }
#define TIMER_NUMOF             ((PIT_NUMOF) + (LPTMR_NUMOF))

#define PIT_BASECLOCK           (CLOCK_BUSCLOCK)
#define PIT_ISR_0               isr_pit1
#define PIT_ISR_1               isr_pit3
/** @} */

/**
 * @name UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev    = UART0,
        .freq   = CLOCK_CORECLOCK,
        .pin_rx = GPIO_PIN(PORT_B, 16), /* TEENSY PIN 0 */
        .pin_tx = GPIO_PIN(PORT_B, 17), /* TEENSY PIN 1 */
        .pcr_rx = PORT_PCR_MUX(3),
        .pcr_tx = PORT_PCR_MUX(3),
        .irqn   = UART0_RX_TX_IRQn,
        .scgc_addr = &SIM->SCGC4,
        .scgc_bit = SIM_SCGC4_UART0_SHIFT,
        .mode   = UART_MODE_8N1,
        .type   = KINETIS_UART,
    },
    {
        .dev    = UART1,
        .freq   = CLOCK_CORECLOCK,
        .pin_rx = GPIO_PIN(PORT_C, 3), /* TEENSY PIN 9 */
        .pin_tx = GPIO_PIN(PORT_C, 4), /* TEENSY PIN 10 */
        .pcr_rx = PORT_PCR_MUX(3),
        .pcr_tx = PORT_PCR_MUX(3),
        .irqn   = UART1_RX_TX_IRQn,
        .scgc_addr = &SIM->SCGC4,
        .scgc_bit = SIM_SCGC4_UART1_SHIFT,
        .mode   = UART_MODE_8N1,
        .type   = KINETIS_UART,
    },
};

#define UART_0_ISR          (isr_uart0_rx_tx)
#define UART_1_ISR          (isr_uart1_rx_tx)

#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
/** @} */

/**
 * @name    PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .ftm        = FTM0,
        .chan       = {
            { .pin = GPIO_PIN(PORT_C, 1), .af = 4, .ftm_chan = 0 }, /* TEENSY PIN 22 */
            { .pin = GPIO_PIN(PORT_C, 2), .af = 4, .ftm_chan = 1 }, /* TEENSY PIN 23 */
            { .pin = GPIO_UNDEF,          .af = 0, .ftm_chan = 0 },
            { .pin = GPIO_UNDEF,          .af = 0, .ftm_chan = 0 }
        },
        .chan_numof = 2,
        .ftm_num    = 0
    }
};

#define PWM_NUMOF           (sizeof(pwm_config) / sizeof(pwm_config[0]))
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
