#include <stdio.h>

#include <quad_decode.h>
#include <usb_serial.h>

__attribute__((noinline)) void _init_Teensyduino_internal_(void);

int main(void)
{
    _init_Teensyduino_internal_();

    Serial.begin(115200);

    while(1) {
        Serial.println("RIOT!");
    }

    return 0;
}
