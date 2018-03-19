#include <usb_serial.h>
#include "quad_decode.h"

#include <stdio.h>
#include <string.h>

extern "C" int main(void)
{
    QuadDecode<1> enc1;
    QuadDecode<2> enc2;

    enc1.setup();
    enc1.start();
    enc2.setup();
    enc2.start();

    while (1) {
        int p1 = enc1.calcPosn();
        int p2 = enc2.calcPosn();
        char buff[128];
        snprintf(buff, 128, "[%i, %i]\n", p1, p2);
        usb_serial_write(buff, strlen(buff));
        for(unsigned long long i = 0 ; i < 10000000 ; i++) __asm("nop");
    }
}

