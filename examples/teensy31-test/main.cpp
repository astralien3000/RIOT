#include <stdio.h>

#include <quad_decode.h>
#include <usb_serial.h>
#include <usb_dev.h>

int main(void)
{
    while(1) {
        usb_serial_write("Hello World!\n", sizeof("Hello World!\n"));
    }

    return 0;
}
