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
 * @brief       USB device stack configuration.
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 */

#ifndef USB_DEV_CONFIG_H
#define USB_DEV_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB_DEV_IDVENDOR                 0xDEAD
#define USB_DEV_IDPRODUCT                0xBEEF
#define USB_DEV_BCDDEVICE                0x0100
#define USB_DEV_STRDESC_LANGID           0x0409

#define USB_DEV_MAX_USER_ENDPOINTS       4

#define USB_DEV_MAX_ENDPOINTS            (USB_DEV_MAX_USER_ENDPOINTS + 1)

#define USB_DEV_MAX_ENDPOINT_IN          (USB_DEV_MAX_ENDPOINTS)
#define USB_DEV_MAX_ENDPOINT_OUT         (USB_DEV_MAX_ENDPOINTS)
#define USB_DEV_MAX_ENDPOINT_NUM         (USB_DEV_MAX_ENDPOINT_IN + USB_DEV_MAX_ENDPOINT_OUT)

#define USB_DEV_MAX_INTERFACES           8

#define USB_DEV_EP0_MAX_PACKET           8

#define USB_DEV_DESCRIPTOR_IDX_MAX       16

#ifdef __cplusplus
}
#endif

#endif
/**
 * @}
 */
