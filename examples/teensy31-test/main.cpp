#include "usb_serial.h"
#include "periph/qdec.h"
#include "xtimer.h"

#include <stdio.h>
#include <string.h>

extern "C" int main(void)
{
    //qdec_init(1, QDEC_X4, NULL, NULL);
    qdec_init(2, QDEC_X4, NULL, NULL);

    while (1) {
        int p1 = 0;//qdec_read(1);
        int p2 = qdec_read(2);
        char buff[128];
        snprintf(buff, 128, "[%i, %i]\n", p1, p2);
        usb_serial_write(buff, strlen(buff));
        //for(unsigned long long i = 0 ; i < 10000000 ; i++) __asm("nop");
        xtimer_sleep(1);
    }
}

