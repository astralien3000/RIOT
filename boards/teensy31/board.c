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
 * @brief       Board specific implementations for the Teensy3.1 & 3.2 boards
 *
 * @author      Loïc Dauphin <loic.dauphin@inria.fr>
 *
 * @}
 */

#include <stddef.h>
#include <stdio.h>

#include "board.h"
#include "cpu.h"
#include "periph/gpio.h"

#include "WProgram.h"
#include "usb_dev.h"
void ResetHandler(void);

void board_init(void)
{
    /* initialize the boards LEDs */
    gpio_init(LED0_PIN, GPIO_OUT);
    gpio_set(LED0_PIN);

    /* initialize the CPU */
    cpu_init();
    //ResetHandler();

    SIM_SCGC5 = 0x00043F82;		// clocks active to all GPIO
    SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);

    // USB uses PLL clock, trace is CPU clock, CLKOUT=OSCERCLK0
    SIM_SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(6);

    // initialize the SysTick counter
    SYST_RVR = (F_CPU / 1000) - 1;
    SYST_CVR = 0;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
    SCB_SHPR3 = 0x20200000;  // Systick = priority 32

    usb_init();
}

extern volatile uint32_t systick_millis_count;
void isr_systick(void)
{
    systick_millis_count++;
}
