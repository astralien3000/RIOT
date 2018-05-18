/*
 * Copyright (C) 2015 Phytec Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @addtogroup  usb_dev_stack
 * @{
 *
 * @file
 * @brief       USB device driver interface definitions.
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 */

#ifndef USB_DEV_DRIVER_H
#define USB_DEV_DRIVER_H

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include "thread.h"
#include "usb_dev_config.h"
#include "usb_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USB_DEV_CTRL_EP              0

typedef enum {
    USB_DEV_STRING_IDX_MFR = 1,
    USB_DEV_STRING_IDX_PRODUCT,
    USB_DEV_STRING_IDX_SN,
    USB_DEV_STRING_IDX_CIF,
    USB_DEV_STRING_IDX_DIF,
} usb_dev_string_idx_t;

typedef enum {
    USB_DEV_STAGE_IDLE = 0,
    USB_DEV_STAGE_STATUS_IN,
    USB_DEV_STAGE_DATA_IN,
    USB_DEV_STAGE_DATA_OUT,
    USB_DEV_STAGE_STALL,
} usb_dev_stage_t;

typedef uint32_t usb_ep_t;

typedef enum {
    USB_DEV_EVENT_SETUP = 0,
    USB_DEV_EVENT_OUT,
    USB_DEV_EVENT_IN,
    USB_DEV_EVENT_REQUEST_CLASS_SETUP,
    USB_DEV_EVENT_REQUEST_CLASS_OUT,
    USB_DEV_EVENT_STALL,
    USB_DEV_EVENT_RESET,
    USB_DEV_EVENT_WAKEUP,
    USB_DEV_EVENT_SUSPEND,
    USB_DEV_EVENT_RESUME,
    USB_DEV_EVENT_SOF,
    USB_DEV_EVENT_ERROR,
} usb_dev_event_t;

/**
 * @brief   Definition of basic usb device options.
 */
typedef enum {
    USB_DEV_SET_CONNECT,             /**< Connect the USB device */
    USB_DEV_SET_DISCONNECT,          /**< Disconnect the USB device */
    USB_DEV_SET_ADDRESS,             /**< Set USB device address */
    USB_DEV_OPT_EP_RESET,            /**< Reset USB device endpoint */
    USB_DEV_SET_EP_ENABLE,           /**< Enable an endpoint */
    USB_DEV_SET_EP_DISABLE,          /**< Disable an endpoint */
    USB_DEV_SET_EP_STALL,            /**< Set STALL for an endpoint */
    USB_DEV_SET_EP_CLR_STALL,        /**< Clear STALL for an endpoint */
} usb_dev_cmd_t;

/**
 * @brief   Definition of the usb device type
 *
 * @see     struct usb_dev_t
 *
 * @note    Forward definition to use in @ref usb_dev_ops_t
 */
typedef struct usb_dev_t usb_dev_t;

/**
 * @brief   usb device API definition.
 *
 * @details This is a set of functions that must be implemented by any driver
 *           for a usb device.
 */
typedef struct {
    usb_dev_t *dev;
    /**
     * @brief Initialize a usb device.
     *
     * @return  0 on success
     */
    int (*init)(void);

    /**
     * @brief Configure a given usb device endpoint
     *
     * @param[in] ep                the usb device endpoint number
     * @param[in] size              the enpoint packet size
     * @return  a error
     */
    int (*configure_ep)(usb_ep_t ep, size_t size);

    /**
     * @brief Set Toggle Bit
     *
     * @param[in] ep                the usb device endpoint number
     * @return  a error
     */
    int (*set_ep_toggle_bit)(usb_ep_t ep);

    /**
     * @brief Clear Toggle Bit
     *
     * @param[in] ep                the usb device endpoint number
     * @return  a error
     */
    int (*clr_ep_toggle_bit)(usb_ep_t ep);

    /**
     * @brief Write data to a given usb device endpoint buffer
     *
     * @param[in] ep                the usb endpoint
     * @param[in] data              the data to write
     * @param[in] data_len          the length of *data* in byte
     * @return  a error
     */
    int (*write_ep)(usb_ep_t ep, uint8_t * data, size_t data_len);

    /**
     * @brief Read data from a given usb device endpoint buffer
     *
     * @param[in] ep                the usb endpoint
     * @param[in] data              the data to read
     * @param[in] data_len          the length of *data* in byte
     * @return  a error
     */
    int (*read_ep)(usb_ep_t ep, uint8_t *data, size_t size);

    /**
     * @brief   Set an option value for a given usb device.
     *
     * @param[in] opt           the option type
     * @param[in] value         the value to set
     * @return  a error
     */
    int (*ioctl)(usb_dev_cmd_t cmd, uint32_t arg);

} usb_dev_ops_t;

typedef int (* usb_dev_iface_init_t)(usb_dev_t *dev);
typedef usb_dev_ops_t *(* usb_dev_driver_init_t)(usb_dev_t *dev);

/**
 * @brief   Definition of the usb device type
 *
 */
struct usb_dev_t {
    usb_dev_ops_t *driver;      /**< The driver for this device */
    void (*irq_ep_event)(usb_dev_t *dev, usb_ep_t ep, uint16_t event_type);
    void (*irq_bc_event)(usb_dev_t *dev, uint16_t event_type);

    uint8_t *string_descriptor;
    usb_device_descriptor_t *device_descriptor;

    uint8_t *sdescr_tab[USB_DEV_DESCRIPTOR_IDX_MAX];
    uint8_t *cfgdescr_tab[USB_DEV_DESCRIPTOR_IDX_MAX];

    kernel_pid_t ep_in_pid[USB_DEV_MAX_ENDPOINT_IN];
    kernel_pid_t ep_out_pid[USB_DEV_MAX_ENDPOINT_OUT];

    usb_device_request_t setup_pkt;

    uint8_t *ep0_data_ptr;
    uint16_t ep0_data_cnt_tx;
    uint16_t ep0_data_cnt_rx;
    uint8_t ep0_buf[USB_DEV_EP0_MAX_PACKET];
    uint8_t zero_pkt;

    uint8_t address;
    uint16_t status;
    uint8_t configuration;
    uint32_t ep_configured;
    uint32_t ep_halted;
    uint32_t ep_stalled;
    uint8_t n_interfaces;

    uint8_t alt_setting[USB_DEV_MAX_INTERFACES];

    usb_dev_stage_t stage;
};

int usb_dev_register_iface(usb_dev_iface_init_t init);
int usb_dev_register_driver(usb_dev_driver_init_t init);
int usb_dev_add_string_descriptor(usb_dev_t *dev, uint8_t index, char *string);
int usb_dev_add_cfg_descriptor(usb_dev_t *dev, uint8_t *descriptor);
int usb_dev_remove(usb_dev_t *dev);
uint32_t ep_configured(usb_dev_t *dev, usb_ep_t ep);

#ifdef __cplusplus
}
#endif

#endif /* USB_DEV_DRIVER_H */
/**
 * @}
 */
