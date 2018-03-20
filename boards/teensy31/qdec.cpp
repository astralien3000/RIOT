#include "periph/qdec.h"

#include "quad_decode.h"

QuadDecode<1> enc1;
QuadDecode<2> enc2;

int32_t qdec_init(qdec_t dev, qdec_mode_t mode, qdec_cb_t cb, void *arg) {
    if(dev == 1) {
        enc1.setup();
        enc1.start();
    }
    else if(dev == 2) {
        enc2.setup();
        enc2.start();
    }

    return 0;
}

int32_t qdec_read(qdec_t dev) {
    if(dev == 1) {
        return enc1.calcPosn();
    }
    else if(dev == 2) {
        return enc2.calcPosn();
    }

    return 0;
}

int32_t qdec_read_and_reset(qdec_t dev) {
    int ret = 0;

    if(dev == 1) {
        ret = enc1.calcPosn();
        enc1.zeroFTM();
    }
    else if(dev == 2) {
        ret = enc2.calcPosn();
        enc1.zeroFTM();
    }

    return ret;
}

void qdec_start(qdec_t dev) {
    if(dev == 1) {
        enc1.start();
    }
    else if(dev == 2) {
        enc2.start();
    }
}

void qdec_stop(qdec_t qdec) {
    // ??
}
