/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_samr21-xpro
 * @{
 *
 * @file
 * @brief       Board specific implementations for the Atmel SAM R21 Xplained
 *              Pro board
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"

void board_init(void)
{
    /* initialize the on-board LED */
    gpio_init(LED0_PIN, GPIO_OUT);

    NVIC_SetPriority(EIC_IRQn, 0);

    NVIC_SetPriority(TC3_IRQn, 1);
    NVIC_SetPriority(TC4_IRQn, 1);

    NVIC_SetPriority(PM_IRQn, 2);
    NVIC_SetPriority(SYSCTRL_IRQn, 2);
    NVIC_SetPriority(WDT_IRQn, 2);
    NVIC_SetPriority(RTC_IRQn, 2);
    NVIC_SetPriority(NVMCTRL_IRQn, 2);
    NVIC_SetPriority(DMAC_IRQn, 2);
    NVIC_SetPriority(USB_IRQn, 2);
    NVIC_SetPriority(EVSYS_IRQn, 2);
    NVIC_SetPriority(SERCOM0_IRQn, 2);
    NVIC_SetPriority(SERCOM1_IRQn, 2);
    NVIC_SetPriority(SERCOM2_IRQn, 2);
    NVIC_SetPriority(SERCOM3_IRQn, 2);
    NVIC_SetPriority(SERCOM4_IRQn, 2);
    NVIC_SetPriority(SERCOM5_IRQn, 2);
    NVIC_SetPriority(TCC0_IRQn, 2);
    NVIC_SetPriority(TCC1_IRQn, 2);
    NVIC_SetPriority(TCC2_IRQn, 2);
    NVIC_SetPriority(TC5_IRQn, 2);
    NVIC_SetPriority(ADC_IRQn, 2);
    NVIC_SetPriority(AC_IRQn, 2);
    NVIC_SetPriority(PTC_IRQn, 2);

    /* initialize the CPU */
    cpu_init();
}
