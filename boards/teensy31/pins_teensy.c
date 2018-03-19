/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "core_pins.h"
#include "pins_arduino.h"

#if defined(KINETISK)
#define GPIO_BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define GPIO_BITBAND_PTR(reg, bit) ((uint32_t *)GPIO_BITBAND_ADDR((reg), (bit)))
//#define GPIO_SET_BIT(reg, bit) (*GPIO_BITBAND_PTR((reg), (bit)) = 1)
//#define GPIO_CLR_BIT(reg, bit) (*GPIO_BITBAND_PTR((reg), (bit)) = 0)
const struct digital_pin_bitband_and_config_table_struct digital_pin_to_info_PGM[] = {
	{GPIO_BITBAND_PTR(CORE_PIN0_PORTREG, CORE_PIN0_BIT), &CORE_PIN0_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN1_PORTREG, CORE_PIN1_BIT), &CORE_PIN1_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN2_PORTREG, CORE_PIN2_BIT), &CORE_PIN2_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN3_PORTREG, CORE_PIN3_BIT), &CORE_PIN3_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN4_PORTREG, CORE_PIN4_BIT), &CORE_PIN4_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN5_PORTREG, CORE_PIN5_BIT), &CORE_PIN5_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN6_PORTREG, CORE_PIN6_BIT), &CORE_PIN6_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN7_PORTREG, CORE_PIN7_BIT), &CORE_PIN7_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN8_PORTREG, CORE_PIN8_BIT), &CORE_PIN8_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN9_PORTREG, CORE_PIN9_BIT), &CORE_PIN9_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN10_PORTREG, CORE_PIN10_BIT), &CORE_PIN10_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN11_PORTREG, CORE_PIN11_BIT), &CORE_PIN11_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN12_PORTREG, CORE_PIN12_BIT), &CORE_PIN12_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN13_PORTREG, CORE_PIN13_BIT), &CORE_PIN13_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN14_PORTREG, CORE_PIN14_BIT), &CORE_PIN14_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN15_PORTREG, CORE_PIN15_BIT), &CORE_PIN15_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN16_PORTREG, CORE_PIN16_BIT), &CORE_PIN16_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN17_PORTREG, CORE_PIN17_BIT), &CORE_PIN17_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN18_PORTREG, CORE_PIN18_BIT), &CORE_PIN18_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN19_PORTREG, CORE_PIN19_BIT), &CORE_PIN19_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN20_PORTREG, CORE_PIN20_BIT), &CORE_PIN20_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN21_PORTREG, CORE_PIN21_BIT), &CORE_PIN21_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN22_PORTREG, CORE_PIN22_BIT), &CORE_PIN22_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN23_PORTREG, CORE_PIN23_BIT), &CORE_PIN23_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN24_PORTREG, CORE_PIN24_BIT), &CORE_PIN24_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN25_PORTREG, CORE_PIN25_BIT), &CORE_PIN25_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN26_PORTREG, CORE_PIN26_BIT), &CORE_PIN26_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN27_PORTREG, CORE_PIN27_BIT), &CORE_PIN27_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN28_PORTREG, CORE_PIN28_BIT), &CORE_PIN28_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN29_PORTREG, CORE_PIN29_BIT), &CORE_PIN29_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN30_PORTREG, CORE_PIN30_BIT), &CORE_PIN30_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN31_PORTREG, CORE_PIN31_BIT), &CORE_PIN31_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN32_PORTREG, CORE_PIN32_BIT), &CORE_PIN32_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN33_PORTREG, CORE_PIN33_BIT), &CORE_PIN33_CONFIG},
#ifdef CORE_PIN34_PORTREG
	{GPIO_BITBAND_PTR(CORE_PIN34_PORTREG, CORE_PIN34_BIT), &CORE_PIN34_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN35_PORTREG, CORE_PIN35_BIT), &CORE_PIN35_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN36_PORTREG, CORE_PIN36_BIT), &CORE_PIN36_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN37_PORTREG, CORE_PIN37_BIT), &CORE_PIN37_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN38_PORTREG, CORE_PIN38_BIT), &CORE_PIN38_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN39_PORTREG, CORE_PIN39_BIT), &CORE_PIN39_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN40_PORTREG, CORE_PIN40_BIT), &CORE_PIN40_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN41_PORTREG, CORE_PIN41_BIT), &CORE_PIN41_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN42_PORTREG, CORE_PIN42_BIT), &CORE_PIN42_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN43_PORTREG, CORE_PIN43_BIT), &CORE_PIN43_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN44_PORTREG, CORE_PIN44_BIT), &CORE_PIN44_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN45_PORTREG, CORE_PIN45_BIT), &CORE_PIN45_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN46_PORTREG, CORE_PIN46_BIT), &CORE_PIN46_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN47_PORTREG, CORE_PIN47_BIT), &CORE_PIN47_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN48_PORTREG, CORE_PIN48_BIT), &CORE_PIN48_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN49_PORTREG, CORE_PIN49_BIT), &CORE_PIN49_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN50_PORTREG, CORE_PIN50_BIT), &CORE_PIN50_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN51_PORTREG, CORE_PIN51_BIT), &CORE_PIN51_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN52_PORTREG, CORE_PIN52_BIT), &CORE_PIN52_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN53_PORTREG, CORE_PIN53_BIT), &CORE_PIN53_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN54_PORTREG, CORE_PIN54_BIT), &CORE_PIN54_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN55_PORTREG, CORE_PIN55_BIT), &CORE_PIN55_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN56_PORTREG, CORE_PIN56_BIT), &CORE_PIN56_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN57_PORTREG, CORE_PIN57_BIT), &CORE_PIN57_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN58_PORTREG, CORE_PIN58_BIT), &CORE_PIN58_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN59_PORTREG, CORE_PIN59_BIT), &CORE_PIN59_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN60_PORTREG, CORE_PIN60_BIT), &CORE_PIN60_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN61_PORTREG, CORE_PIN61_BIT), &CORE_PIN61_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN62_PORTREG, CORE_PIN62_BIT), &CORE_PIN62_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN63_PORTREG, CORE_PIN63_BIT), &CORE_PIN63_CONFIG},
#endif
};

#endif

extern void usb_init(void);

//void init_pins(void)
__attribute__((noinline))
void _init_Teensyduino_internal_(void)
{
	delay(50);
	usb_init();
	delay(350);
}

// TODO: startup code needs to initialize all pins to GPIO mode, input by default

void digitalWrite(uint8_t pin, uint8_t val)
{
	if (pin >= CORE_NUM_DIGITAL) return;
#ifdef KINETISK
	if (*portModeRegister(pin)) {
		if (val) {
			*portSetRegister(pin) = 1;
		} else {
			*portClearRegister(pin) = 1;
		}
#else
	if (*portModeRegister(pin) & digitalPinToBitMask(pin)) {
		if (val) {
			*portSetRegister(pin) = digitalPinToBitMask(pin);
		} else {
			*portClearRegister(pin) = digitalPinToBitMask(pin);
		}
#endif
	} else {
		volatile uint32_t *config = portConfigRegister(pin);
		if (val) {
			// TODO use bitband for atomic read-mod-write
			*config |= (PORT_PCR_PE | PORT_PCR_PS);
			//*config = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;
		} else {
			// TODO use bitband for atomic read-mod-write
			*config &= ~(PORT_PCR_PE);
			//*config = PORT_PCR_MUX(1);
		}
	}

}

void pinMode(uint8_t pin, uint8_t mode)
{
	volatile uint32_t *config;

	if (pin >= CORE_NUM_DIGITAL) return;
	config = portConfigRegister(pin);

	if (mode == OUTPUT || mode == OUTPUT_OPENDRAIN) {
#ifdef KINETISK
		*portModeRegister(pin) = 1;
#else
		*portModeRegister(pin) |= digitalPinToBitMask(pin); // TODO: atomic
#endif
		*config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		if (mode == OUTPUT_OPENDRAIN) {
		    *config |= PORT_PCR_ODE;
		} else {
		    *config &= ~PORT_PCR_ODE;
                }
	} else {
#ifdef KINETISK
		*portModeRegister(pin) = 0;
#else
		*portModeRegister(pin) &= ~digitalPinToBitMask(pin);
#endif
		if (mode == INPUT) {
			*config = PORT_PCR_MUX(1);
		} else if (mode == INPUT_PULLUP) {
			*config = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS;
		} else if (mode == INPUT_PULLDOWN) {
			*config = PORT_PCR_MUX(1) | PORT_PCR_PE;
		} else { // INPUT_DISABLE
			*config = 0;
		}
	}
}

// the systick interrupt is supposed to increment this at 1 kHz rate
volatile uint32_t systick_millis_count = 0;

//uint32_t systick_current, systick_count, systick_istatus;  // testing only

uint32_t micros(void)
{
	uint32_t count, current, istatus;

	__disable_irq();
	current = SYST_CVR;
	count = systick_millis_count;
	istatus = SCB_ICSR;	// bit 26 indicates if systick exception pending
	__enable_irq();
	 //systick_current = current;
	 //systick_count = count;
	 //systick_istatus = istatus & SCB_ICSR_PENDSTSET ? 1 : 0;
	if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) count++;
	current = ((F_CPU / 1000) - 1) - current;
#if defined(KINETISL) && F_CPU == 48000000
	return count * 1000 + ((current * (uint32_t)87381) >> 22);
#elif defined(KINETISL) && F_CPU == 24000000
	return count * 1000 + ((current * (uint32_t)174763) >> 22);
#endif
	return count * 1000 + current / (F_CPU / 1000000);
}

void delay(uint32_t ms)
{
	uint32_t start = micros();

	if (ms > 0) {
		while (1) {
			while ((micros() - start) >= 1000) {
				ms--;
				if (ms == 0) return;
				start += 1000;
			}
            //yield();
		}
	}
}
