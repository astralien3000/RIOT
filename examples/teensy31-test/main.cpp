#include <usb_serial.h>

#include "xtimer.h"

extern "C" int main(void)
{
    while (1) {
        usb_serial_write("hello\n", sizeof("hello\n"));
        xtimer_usleep(500000);
    }
}

