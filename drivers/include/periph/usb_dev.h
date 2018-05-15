/*
 * Copyright (C) 2018 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_periph_usb_dev USB_DEV
 * @ingroup     drivers_periph
 * @brief       Low-level USB_DEV peripheral driver
 *
 * TODO
 *
 * @{
 *
 * @file
 * @brief       Low-level USB_DEV peripheral driver interface definition
 *
 * @author      Loïc Dauphin <loic.dauphin@inria.fr>
 */

#ifndef PERIPH_USB_DEV_H
#define PERIPH_USB_DEV_H

#include <stddef.h>
#include <stdint.h>
#include <limits.h>

#include "periph_cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Define default USB_DEV_EP type identifier
 */
#ifndef HAVE_USB_DEV_EP_T
typedef unsigned int usb_dev_ep_t;
#endif

/**
 * @brief   Signature for receive interrupt callback
 *
 * @param[in] arg           context to the callback (optional)
 * @param[in] data          the byte that was received
 */
typedef void(*usb_dev_rx_cb_t)(void *arg, uint8_t data);

/**
 * @brief   Interrupt context for a USB_DEV device
 */
#ifndef HAVE_USB_DEV_ISR_CTX_T
typedef struct {
    usb_dev_rx_cb_t rx_cb;     /**< data received interrupt callback */
    void *arg;              /**< argument to both callback routines */
} usb_dev_isr_ctx_t;
#endif

/**
 * @brief   Possible USB_DEV return values
 */
enum {
    USB_DEV_OK         =  0,   /**< everything in order */
    USB_DEV_ERROR      = -1,   /**< internal error */
};

/**
 * @brief   Initialize the USB in device mode
 *
 * TODO
 */
int usb_dev_init(void);

/**
 * @brief   Initialize a given USB device endpoint
 *
 * TODO
 */
int usb_dev_ep_init(usb_dev_ep_t ep);

/**
 * @brief   Write data from the given buffer to the specified USB_DEV endpoint
 *
 * TODO
 */
size_t usb_dev_ep_write(usb_dev_ep_t ep, const uint8_t *data, size_t len);

/**
 * @brief   Read data to the given buffer from the specified USB_DEV endpoint
 *
 * TODO
 */
size_t usb_dev_ep_read(usb_dev_ep_t ep, uint8_t *data, size_t len);

/**
 * @brief   Power on the USB_DEV device
 */
void usb_dev_poweron(void);

/**
 * @brief Power off the USB_DEV device
 */
void usb_dev_poweroff(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_USB_DEV_H */
/** @} */
