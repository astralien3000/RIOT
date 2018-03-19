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

#include "kinetis.h"
#include "core_pins.h" // testing only
#include <errno.h>


// Flash Security Setting. On Teensy 3.2, you can lock the MK20 chip to prevent
// anyone from reading your code.  You CAN still reprogram your Teensy while
// security is set, but the bootloader will be unable to respond to auto-reboot
// requests from Arduino. Pressing the program button will cause a full chip
// erase to gain access, because the bootloader chip is locked out.  Normally,
// erase occurs when uploading begins, so if you press the Program button
// accidentally, simply power cycling will run your program again.  When
// security is locked, any Program button press causes immediate full erase.
// Special care must be used with the Program button, because it must be made
// accessible to initiate reprogramming, but it must not be accidentally
// pressed when Teensy Loader is not being used to reprogram.  To set lock the
// security change this to 0xDC.  Teensy 3.0 and 3.1 do not support security lock.
#define FSEC 0xDE

// Flash Options
#define FOPT 0xF9


extern unsigned long _stext;
extern unsigned long _etext;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _estack;
//extern void __init_array_start(void);
//extern void __init_array_end(void);



extern int main (void);
void ResetHandler(void);
void _init_Teensyduino_internal_(void) __attribute__((noinline));
void __libc_init_array(void);


void fault_isr(void)
{
	while (1) {
		// keep polling some communication while in fault
		// mode, so we don't completely die.
		if (SIM_SCGC4 & SIM_SCGC4_USBOTG) usb_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART0) uart0_status_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART1) uart1_status_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART2) uart2_status_isr();
	}
}

void unused_isr(void)
{
	fault_isr();
}

extern volatile uint32_t systick_millis_count;
void isr_systick(void)
{
    systick_millis_count++;
}

static void startup_default_early_hook(void) {
#if defined(KINETISK)
	WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;
#elif defined(KINETISL)
	SIM_COPC = 0;  // disable the watchdog
#endif
}

void ResetHandler(void)
{
	uint32_t *src = &_etext;
	uint32_t *dest = &_sdata;
	unsigned int i;
#if F_CPU <= 2000000
    volatile int n;
#endif
    //volatile int count;

    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");

    // programs using the watchdog timer or needing to initialize hardware as
    // early as possible can implement startup_early_hook()
    startup_default_early_hook();

    // enable clocks to always-used peripherals
    SIM_SCGC5 = 0x00043F82;		// clocks active to all GPIO

	// TODO: do this while the PLL is waiting to lock....
	while (dest < &_edata) *dest++ = *src++;
	dest = &_sbss;
	while (dest < &_ebss) *dest++ = 0;

    // enable capacitors for crystal
    OSC0_CR = OSC_SC8P | OSC_SC2P | OSC_ERCLKEN;

    // enable osc, 8-32 MHz range, low power mode
	MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS;
	// switch to crystal as clock source, FLL input = 16 MHz / 512
	MCG_C1 =  MCG_C1_CLKS(2) | MCG_C1_FRDIV(4);
	// wait for crystal oscillator to begin
	while ((MCG_S & MCG_S_OSCINIT0) == 0) ;
	// wait for FLL to use oscillator
	while ((MCG_S & MCG_S_IREFST) != 0) ;
	// wait for MCGOUT to use oscillator
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)) ;

	// now in FBE mode
	//  C1[CLKS] bits are written to 10
	//  C1[IREFS] bit is written to 0
	//  C1[FRDIV] must be written to divide xtal to 31.25-39 kHz
	//  C6[PLLS] bit is written to 0
	//  C2[LP] is written to 0

    // if we need faster than the crystal, turn on the PLL
	MCG_C5 = MCG_C5_PRDIV0(3);		 // config PLL input for 16 MHz Crystal / 4 = 4 MHz

	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(0); // config PLL for 96 MHz output

	// wait for PLL to start using xtal as its input
	while (!(MCG_S & MCG_S_PLLST)) ;
	// wait for PLL to lock
	while (!(MCG_S & MCG_S_LOCK0)) ;
	// now we're in PBE mode

    // now program the clock dividers
	// config divisors: 48 MHz core, 48 MHz bus, 24 MHz flash, USB = 96 / 2
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV3(1) |  SIM_CLKDIV1_OUTDIV4(3);
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);

    // switch to PLL as clock source, FLL input = 16 MHz / 512
    MCG_C1 = MCG_C1_CLKS(0) | MCG_C1_FRDIV(4);
    // wait for PLL clock to be used
    while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3)) ;
    // now we're in PEE mode
    // USB uses PLL clock, trace is CPU clock, CLKOUT=OSCERCLK0
    SIM_SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(6);

    // initialize the SysTick counter
    SYST_RVR = (F_CPU / 1000) - 1;
    SYST_CVR = 0;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
    SCB_SHPR3 = 0x20200000;  // Systick = priority 32

	//init_pins();
	__enable_irq();

	_init_Teensyduino_internal_();

	__libc_init_array();

    main();
	while (1) ;
}

char *__brkval = (char *)&_ebss;

#ifndef STACK_MARGIN
#if defined(__MKL26Z64__)
#define STACK_MARGIN  512
#elif defined(__MK20DX128__)
#define STACK_MARGIN  1024
#elif defined(__MK20DX256__)
#define STACK_MARGIN  4096
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define STACK_MARGIN  8192
#endif
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

void * _sbrk(int incr)
{
	char *prev, *stack;

	prev = __brkval;
	if (incr != 0) {
		__asm__ volatile("mov %0, sp" : "=r" (stack) ::);
		if (prev + incr >= stack - STACK_MARGIN) {
			errno = ENOMEM;
			return (void *)-1;
		}
		__brkval = prev + incr;
	}
	return prev;
}

__attribute__((weak)) 
int _read(int file, char *ptr, int len)
{
	return 0;
}

__attribute__((weak)) 
int _close(int fd)
{
	return -1;
}

#include <sys/stat.h>

__attribute__((weak)) 
int _fstat(int fd, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

__attribute__((weak)) 
int _isatty(int fd)
{
	return 1;
}

__attribute__((weak)) 
int _lseek(int fd, long long offset, int whence)
{
	return -1;
}

__attribute__((weak)) 
void _exit(int status)
{
	while (1);
}

__attribute__((weak)) 
void __cxa_pure_virtual()
{
	while (1);
}

__attribute__((weak)) 
int __cxa_guard_acquire (char *g) 
{
	return !(*g);
}

__attribute__((weak)) 
void __cxa_guard_release(char *g)
{
	*g = 1;
}

__attribute__((weak))
void abort(void)
{
	while (1) ;
}

#pragma GCC diagnostic pop

int nvic_execution_priority(void)
{
	uint32_t priority=256;
	uint32_t primask, faultmask, basepri, ipsr;

	// full algorithm in ARM DDI0403D, page B1-639
	// this isn't quite complete, but hopefully good enough
	__asm__ volatile("mrs %0, faultmask\n" : "=r" (faultmask)::);
	if (faultmask) return -1;
	__asm__ volatile("mrs %0, primask\n" : "=r" (primask)::);
	if (primask) return 0;
	__asm__ volatile("mrs %0, ipsr\n" : "=r" (ipsr)::);
	if (ipsr) {
		if (ipsr < 16) priority = 0; // could be non-zero
		else priority = NVIC_GET_PRIORITY(ipsr - 16);
	}
	__asm__ volatile("mrs %0, basepri\n" : "=r" (basepri)::);
	if (basepri > 0 && basepri < priority) priority = basepri;
	return priority;
}


#if defined(HAS_KINETIS_HSRUN) && F_CPU > 120000000
int kinetis_hsrun_disable(void)
{
	if (SMC_PMSTAT == SMC_PMSTAT_HSRUN) {
		// First, reduce the CPU clock speed, but do not change
		// the peripheral speed (F_BUS).  Serial1 & Serial2 baud
		// rates will be impacted, but most other peripherals
		// will continue functioning at the same speed.
		#if F_CPU == 240000000 && F_BUS == 60000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 3, 1, 7); // ok
		#elif F_CPU == 240000000 && F_BUS == 80000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 8); // ok
		#elif F_CPU == 240000000 && F_BUS == 120000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 7); // ok
		#elif F_CPU == 216000000 && F_BUS == 54000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 3, 1, 7); // ok
		#elif F_CPU == 216000000 && F_BUS == 72000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 8); // ok
		#elif F_CPU == 216000000 && F_BUS == 108000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 7); // ok
		#elif F_CPU == 192000000 && F_BUS == 48000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 3, 1, 7); // ok
		#elif F_CPU == 192000000 && F_BUS == 64000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 8); // ok
		#elif F_CPU == 192000000 && F_BUS == 96000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 7); // ok
		#elif F_CPU == 180000000 && F_BUS == 60000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 8); // ok
		#elif F_CPU == 180000000 && F_BUS == 90000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 7); // ok
		#elif F_CPU == 168000000 && F_BUS == 56000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 5); // ok
		#elif F_CPU == 144000000 && F_BUS == 48000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 5); // ok
		#elif F_CPU == 144000000 && F_BUS == 72000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 5); // ok
		#else
			return 0;
		#endif
		// Then turn off HSRUN mode
		SMC_PMCTRL = SMC_PMCTRL_RUNM(0);
		while (SMC_PMSTAT == SMC_PMSTAT_HSRUN) ; // wait
		return 1;
	}
	return 0;
}

int kinetis_hsrun_enable(void)
{
	if (SMC_PMSTAT == SMC_PMSTAT_RUN) {
		// Turn HSRUN mode on
		SMC_PMCTRL = SMC_PMCTRL_RUNM(3);
		while (SMC_PMSTAT != SMC_PMSTAT_HSRUN) {;} // wait
		// Then configure clock for full speed
		#if F_CPU == 240000000 && F_BUS == 60000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 3, 0, 7);
		#elif F_CPU == 240000000 && F_BUS == 80000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 7);
		#elif F_CPU == 240000000 && F_BUS == 120000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 7);
		#elif F_CPU == 216000000 && F_BUS == 54000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 3, 0, 7);
		#elif F_CPU == 216000000 && F_BUS == 72000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 7);
		#elif F_CPU == 216000000 && F_BUS == 108000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 7);
		#elif F_CPU == 192000000 && F_BUS == 48000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 3, 0, 6);
		#elif F_CPU == 192000000 && F_BUS == 64000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 6);
		#elif F_CPU == 192000000 && F_BUS == 96000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 6);
		#elif F_CPU == 180000000 && F_BUS == 60000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 6);
		#elif F_CPU == 180000000 && F_BUS == 90000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 6);
		#elif F_CPU == 168000000 && F_BUS == 56000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 5);
		#elif F_CPU == 144000000 && F_BUS == 48000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 4);
		#elif F_CPU == 144000000 && F_BUS == 72000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 4);
		#else
			return 0;
		#endif
		return 1;
	}
	return 0;
}
#endif // HAS_KINETIS_HSRUN && F_CPU > 120000000

