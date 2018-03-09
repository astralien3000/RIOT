#include <stdio.h>

#include <quad_decode.h>
#include <usb_serial.h>

int main(void)
{
    QuadDecode<1> enc;

    usb_serial_write("Hello World!\n", sizeof("Hello World!\n"));

    return 0;
}
