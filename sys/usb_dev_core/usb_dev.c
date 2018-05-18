/*
 * Copyright (C) 2015 Phytec Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @addtogroup  usb_dev
 * @{
 *
 * @file
 * @brief       USB device stack.
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 */


#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "periph/usb_dev.h"
#include "usb_dev.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define STATUS_MASK_OUT(ep)             (1 << ENDPOINT_NUM(ep))
#define STATUS_MASK_IN(ep)              (1 << (16 + ENDPOINT_NUM(ep)))

#define STATUS_MASK_EP0_OUT             STATUS_MASK_OUT(0)
#define STATUS_MASK_EP0_IN              STATUS_MASK_IN(0)

typedef int (* usb_dev_srq_t)(usb_dev_t *dev);

usb_device_descriptor_t usb_dev_def_descriptor = {
  .bLength = sizeof(usb_device_descriptor_t),
  .bDescriptorType = DESCRIPTOR_TYPE_DEVICE,
  .bcdUSB = 0x0110,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = USB_DEV_EP0_MAX_PACKET,
  .idVendor = USB_DEV_IDVENDOR,
  .idProduct = USB_DEV_IDPRODUCT,
  .bcdDevice = USB_DEV_BCDDEVICE,
  .iManufacturer = 0,
  .iProduct = 0,
  .iSerialNumber = 0,
  .bNumConfigurations = 0x01,
};

usb_string_descriptor_t usb_dev_def_string_descriptor = {
  .bLength = sizeof(usb_string_descriptor_t),
  .bDescriptorType = DESCRIPTOR_TYPE_STRING,
  .bString = USB_DEV_STRDESC_LANGID,
};

//inline static void usb_dev_broadcast_event(usb_dev_t *dev, msg_t *msg);

inline static void usb_dev_broadcast_class(usb_dev_t *dev, msg_t *msg);

static void usb_dev_data_in_prepare(usb_dev_t *dev, size_t l);

static inline usb_ep_t srq_get_ep_addr(usb_dev_t *dev)
{
  return (usb_ep_t)ENDPOINT_ADDR(dev->setup_pkt.wIndexL);
}

uint32_t ep_configured(usb_dev_t *dev, usb_ep_t ep)
{
  if (ep & ENDPOINT_IN_MASK) {
    return dev->ep_configured & STATUS_MASK_IN(ep);
  }
  return dev->ep_configured & STATUS_MASK_OUT(ep);
}

static inline void ep_set_configured(usb_dev_t *dev, usb_ep_t ep)
{
  if (ep & ENDPOINT_IN_MASK) {
    dev->ep_configured |= STATUS_MASK_IN(ep);
  }
  else {
    dev->ep_configured |= STATUS_MASK_OUT(ep);
  }
}

static inline void ep_clear_configured(usb_dev_t *dev, usb_ep_t ep)
{
  if (ep & ENDPOINT_IN_MASK) {
    dev->ep_configured &= ~STATUS_MASK_IN(ep);
  }
  else {
    dev->ep_configured &= ~STATUS_MASK_OUT(ep);
  }
}

static inline uint32_t ep_halted(usb_dev_t *dev, usb_ep_t ep)
{
  if (ep & ENDPOINT_IN_MASK) {
    return dev->ep_halted & STATUS_MASK_IN(ep);
  }
  return dev->ep_halted & STATUS_MASK_OUT(ep);
}

static inline void ep_set_halted(usb_dev_t *dev, usb_ep_t ep)
{
  if (ep & ENDPOINT_IN_MASK) {
    dev->ep_halted |= STATUS_MASK_IN(ep);
  }
  else {
    dev->ep_halted |= STATUS_MASK_OUT(ep);
  }
}

static inline void ep_clear_halted(usb_dev_t *dev, usb_ep_t ep)
{
  if (ep & ENDPOINT_IN_MASK) {
    dev->ep_halted &= ~STATUS_MASK_IN(ep);
  }
  else {
    dev->ep_halted &= ~STATUS_MASK_OUT(ep);
  }
}

static inline uint32_t ep_stalled(usb_dev_t *dev, usb_ep_t ep)
{
  if (ep & ENDPOINT_IN_MASK) {
    return dev->ep_stalled & STATUS_MASK_IN(ep);
  }
  return dev->ep_stalled & STATUS_MASK_OUT(ep);
}

static inline void ep_set_stalled(usb_dev_t *dev, usb_ep_t ep)
{
  if (ep & ENDPOINT_IN_MASK) {
    dev->ep_stalled |= STATUS_MASK_IN(ep);
  }
  else {
    dev->ep_stalled |= STATUS_MASK_OUT(ep);
  }
}

static inline void ep_clear_stalled(usb_dev_t *dev, usb_ep_t ep)
{
  if (ep & ENDPOINT_IN_MASK) {
    dev->ep_stalled &= ~STATUS_MASK_IN(ep);
  }
  else {
    dev->ep_stalled &= ~STATUS_MASK_OUT(ep);
  }
}

static inline void reset_ep_status(usb_dev_t *dev)
{
  usb_dev_ops_t *driver = dev->driver;
  dev->ep_configured  = STATUS_MASK_EP0_IN | STATUS_MASK_EP0_OUT;
  dev->ep_halted  = 0;
  dev->ep_stalled = 0;
  for (uint8_t n = 1; n < 16; n++) {
    if (ep_configured(dev, ENDPOINT_ADDR_OUT(n))) {
      driver->ioctl(USB_DEV_SET_EP_DISABLE, ENDPOINT_ADDR_OUT(n));
    }

    if (ep_configured(dev, ENDPOINT_ADDR_IN(n))) {
      driver->ioctl(USB_DEV_SET_EP_DISABLE, ENDPOINT_ADDR_IN(n));
    }
  }
}

static inline void set_ep_configuration(usb_dev_t *dev, usb_ep_descriptor_t *ep_dsc)
{
  usb_dev_ops_t *driver = dev->driver;
  usb_ep_t ep = ENDPOINT_ADDR(ep_dsc->bEndpointAddress);
  size_t size = ep_dsc->wMaxPacketSize;
  usb_dev_ep_type_t type = USB_DEV_EP_TYPE_CTRL;

  if(ep_dsc->bmAttributes & ENDPOINT_TRANSFER_TYPE_BULK) {
    type = USB_DEV_EP_TYPE_BULK;
  }
  else if(ep_dsc->bmAttributes & ENDPOINT_TRANSFER_TYPE_INTERRUPT) {
    type = USB_DEV_EP_TYPE_INTR;
  }
  else if(ep_dsc->bmAttributes & ENDPOINT_TRANSFER_TYPE_ISOCHRONOUS) {
    type = USB_DEV_EP_TYPE_ISOC;
  }

  usb_dev_ep_open(ep, size, type);
  driver->ioctl(USB_DEV_SET_EP_ENABLE, ep);
  driver->ioctl(USB_DEV_OPT_EP_RESET, ep);
  ep_set_configured(dev, ep);
}

inline static int usb_dev_get_string_descriptor(usb_dev_t *dev, uint8_t index, uint8_t **buf)
{
  if (index >= USB_DEV_DESCRIPTOR_IDX_MAX) {
    *buf = NULL;
    return 0;
  }

  uint8_t *sdescr = dev->sdescr_tab[index];
  if (sdescr == NULL) {
    *buf = NULL;
    return 0;
  }

  *buf = sdescr;
  return ((usb_string_descriptor_t *)sdescr)->bLength;
}

inline static int usb_dev_get_cfg_descriptor(usb_dev_t *dev, uint8_t index, uint8_t **buf)
{
  uint8_t *ptr = (uint8_t *)dev->cfgdescr_tab[index];

  for (uint8_t n = 0; n != index; n++) {
    if (((usb_cfg_descriptor_t *)ptr)->bLength != 0) {
      ptr += ((usb_cfg_descriptor_t *)ptr)->wTotalLength;
    }
  }

  *buf = ptr;
  return ((usb_cfg_descriptor_t *)ptr)->wTotalLength;
}

/**
 * @brief Get Status Device
 * This request returns status for the device.
 * @note usb_20.pdf, 9.4.5
 * The device should pass back the status of Self Powered (D0) and
 * Remote Wakeup (D1) bits during Data In stage.
 *
 * wLength : 2
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_get_status_dev(usb_dev_t *dev)
{
  dev->ep0_data_ptr = (uint8_t *)&dev->status;
  return USB_DEV_STAGE_DATA_IN;
}

/**
 * @brief Get Status Interface
 * This request returns status for a interface.
 * @note usb_20.pdf, 9.4.5
 * The device should pass back the zero value during Data In stage
 * or Request Error if specified interface does not exist.
 *
 * wLength : 2
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_get_status_if(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;
  dev->ep0_data_ptr = dev->ep0_buf;
  uint16_t *val = (uint16_t*)dev->ep0_buf;

  if ((dev->configuration == 0) && (dev->address == 0)) {
    /* Device is in Default state */
    return USB_DEV_STAGE_STALL;
  }

  if ((dev->configuration == 0) && (dev->address)) {
    /* Device is in Address state */
    return USB_DEV_STAGE_STALL;
  }

  if (setup_pkt->wIndexL >= dev->n_interfaces) {
    /* Index too high */
    return USB_DEV_STAGE_STALL;
  }

  *val = 0;
  return USB_DEV_STAGE_DATA_IN;
}

/**
 * @brief Get Status Endpoing
 * This request returns status for a endpoint.
 * @note usb_20.pdf, 9.4.5
 * The device should pass back the status of Halt (D0) bit
 * during Data In stage. The Halt bit is set to one, if the
 * the endpoint is halted.
 *
 * wLength : 2
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_get_status_ep(usb_dev_t *dev)
{
  usb_ep_t ep = srq_get_ep_addr(dev);
  dev->ep0_data_ptr = dev->ep0_buf;
  uint16_t *val = (uint16_t*)dev->ep0_buf;

  if ((dev->configuration == 0) && (dev->address == 0)) {
    /* Device is in Default state */
    return USB_DEV_STAGE_STALL;
  }

  if ((dev->configuration == 0) && (dev->address)) {
    /* Device is in Address state */
    if (ENDPOINT_NUM(ep) != 0) {
      return USB_DEV_STAGE_STALL;
    }
  }

  if (!ep_configured(dev, ep)) {
    /* Unconfigured Endpoint */
    return USB_DEV_STAGE_STALL;
  }

  *val = ep_halted(dev, ep) ? 1 : 0;
  return USB_DEV_STAGE_DATA_IN;
}

/**
 * @brief Set Feature Device
 * This request is used to set the Remote Wakeup feature for
 * the device.
 * @note usb_20.pdf, 9.4.9
 * The device should pass back a STALL during Status stage
 * if the feature cannot be set or not exist.
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_set_feature_dev(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if (setup_pkt->wValue == FEATURE_DEVICE_REMOTE_WAKEUP) {
    dev->status |=  DEVICE_STATUS_REMOTE_WAKEUP;
    return USB_DEV_STAGE_STATUS_IN;
  }

  return USB_DEV_STAGE_STALL;
}

/**
 * @brief Set Feature Endpoint
 * This request is used to set the Halt feature for
 * a endpoing.
 * @note usb_20.pdf, 9.4.9
 * The device should pass back a STALL during Status stage
 * if the feature cannot be set.
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_set_feature_ep(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;
  usb_dev_ops_t *driver = dev->driver;
  usb_ep_t ep = srq_get_ep_addr(dev);

  if (dev->configuration == 0) {
    return USB_DEV_STAGE_STALL;
  }

  if (ENDPOINT_NUM(ep) == 0) {
    return USB_DEV_STAGE_STALL;
  }

  if (!ep_configured(dev, ep)) {
    return USB_DEV_STAGE_STALL;
  }

  if (setup_pkt->wValue != FEATURE_ENDPOINT_HALT) {
    return USB_DEV_STAGE_STALL;
  }

  driver->ioctl(USB_DEV_SET_EP_STALL, ep);
  ep_set_halted(dev, ep);
  return USB_DEV_STAGE_STATUS_IN;
}

/**
 * @brief Clear Feature Device
 * This request is used to clear the Remote Wakeup feature for
 * the device.
 * @note usb_20.pdf, 9.4.1
 * The device should pass back a STALL during Status stage
 * if the feature cannot be clear or not exist.
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_clear_feature_dev(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if (setup_pkt->wValue == FEATURE_DEVICE_REMOTE_WAKEUP) {
    dev->status &= ~DEVICE_STATUS_REMOTE_WAKEUP;
    return USB_DEV_STAGE_STATUS_IN;
  }

  return USB_DEV_STAGE_STALL;
}

/**
 * @brief Clear Feature Endpoint
 * This request is used to clear the Halt feature for
 * a endpoing.
 * @note usb_20.pdf, 9.4.9
 * The device should pass back a STALL during Status stage
 * if the feature cannot be clear. The endpoing should
 * reinitialized the data toggle bit to DATA0.
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_clear_feature_ep(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;
  usb_dev_ops_t *driver = dev->driver;
  usb_ep_t ep = srq_get_ep_addr(dev);

  if (dev->configuration == 0) {
    return USB_DEV_STAGE_STALL;
  }

  if (ENDPOINT_NUM(ep) == 0) {
    return USB_DEV_STAGE_STALL;
  }

  if (!ep_configured(dev, ep)) {
    return USB_DEV_STAGE_STALL;
  }

  if (setup_pkt->wValue != FEATURE_ENDPOINT_HALT) {
    return USB_DEV_STAGE_STALL;
  }

  if (ep_stalled(dev, ep)) {
    /* The enpoint has been stalled by the hardware driver. */
    return USB_DEV_STAGE_STALL;
  }

  driver->ioctl(USB_DEV_SET_EP_CLR_STALL, ep);
  ep_clear_halted(dev, ep);
  return USB_DEV_STAGE_STATUS_IN;
}

/**
 * @brief Set Address
 * This request is used to set the address of the device.
 * @note usb_20.pdf, 9.4.6
 * The device should not change the device address until
 * the Status stage is completed succesfully.
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_set_address_dev(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if ((dev->configuration) && (dev->address)) {
    /* Device is in Configured state */
    return USB_DEV_STAGE_STALL;
  }

  /* Device is in Default or Address state */
  dev->address = setup_pkt->wValueL;
  return USB_DEV_STAGE_STATUS_IN;
}

/**
 * @brief Get Descriptor Device
 * This request returns the device descriptor.
 * @note usb_20.pdf, 9.4.3
 *
 * wValueH field specifies the descriptor type and
 * wValueL field the descriptor index.
 * wLength field specifies the number of bytes to
 * pass back during the Data In stage
 *
 * The request is valid for all device states.
 *
 * wLength : Descriptor Length
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_get_descriptor_dev(usb_dev_t *dev)
{
  uint8_t n = 0;
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  switch (setup_pkt->wValueH) {
    case DESCRIPTOR_TYPE_DEVICE:
      dev->ep0_data_ptr = (uint8_t*)dev->device_descriptor;
      n = sizeof(usb_device_descriptor_t);
      break;

    case DESCRIPTOR_TYPE_CONFIGURATION:
      n = usb_dev_get_cfg_descriptor(dev, setup_pkt->wValueL, &dev->ep0_data_ptr);
      break;

    case DESCRIPTOR_TYPE_STRING:
      n = usb_dev_get_string_descriptor(dev, setup_pkt->wValueL, &dev->ep0_data_ptr);
      break;

    case DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
    default:
      return USB_DEV_STAGE_STALL;
  }

  if (n == 0) {
    return USB_DEV_STAGE_STALL;
  }

  usb_dev_data_in_prepare(dev, n);
  return USB_DEV_STAGE_DATA_IN;
}

/**
 * @brief Get Configuration Device
 * @note usb_20.pdf, 9.4.2
 * The device should pass back the current configuration
 * value during Data In stage.
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  usb_dev_stage_t
 */
static int srq_get_config_dev(usb_dev_t *dev)
{
  dev->ep0_data_ptr = &dev->configuration;
  return USB_DEV_STAGE_DATA_IN;
}

/**
 * @brief Set Configuration Device
 * This request sets the device configuration.
 * @note usb_20.pdf, 9.4.7
 *
 * wValueL field specifies the configuration value.
 *
 * The device should pass back a STALL during Status stage
 * if the configuration value is invalid.
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  0 on success
 */
static int srq_set_config_dev(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;
  uint8_t *dsc_p;
  uint8_t alternate = 0;

  if (setup_pkt->wValueL == 0) {
    /* Device enters or remains in the Address state. */
    dev->configuration = 0;
    reset_ep_status(dev);
    return USB_DEV_STAGE_STATUS_IN;
  }

  usb_dev_get_cfg_descriptor(dev, 0, &dsc_p);

  while (((usb_generic_descriptor_t *)dsc_p)->bLength) {

    usb_cfg_descriptor_t *cfg_dsc = (usb_cfg_descriptor_t *)dsc_p;
    usb_if_descriptor_t *iface_dsc = (usb_if_descriptor_t *)dsc_p;
    usb_ep_descriptor_t *ep_dsc = (usb_ep_descriptor_t *)dsc_p;

    if (cfg_dsc->bDescriptorType == DESCRIPTOR_TYPE_CONFIGURATION) {
      if (cfg_dsc->bConfigurationValue != setup_pkt->wValueL) {
        dsc_p += cfg_dsc->wTotalLength;
        continue;
      }
      dev->configuration = setup_pkt->wValueL;
      dev->n_interfaces = cfg_dsc->bNumInterfaces;
      dev->status = 0;
      reset_ep_status(dev);
      if (cfg_dsc->bmAttributes & CONFIG_SELF_POWERED) {
        dev->status |= DEVICE_STATUS_SELF_POWERED;
      }
    }

    if (iface_dsc->bDescriptorType == DESCRIPTOR_TYPE_INTERFACE) {
      alternate = iface_dsc->bAlternateSetting;
    }

    if ((ep_dsc->bDescriptorType == DESCRIPTOR_TYPE_ENDPOINT) && (alternate == 0)) {
      set_ep_configuration(dev, ep_dsc);
    }
    dsc_p += ((usb_generic_descriptor_t *)dsc_p)->bLength;
  }

  if (dev->configuration == setup_pkt->wValueL) {
    return USB_DEV_STAGE_STATUS_IN;
  }

  return USB_DEV_STAGE_STALL;
}

/**
 * @brief Get Interface
 * @note usb_20.pdf, 9.4.4
 * The device should pass back the selected alternate setting
 * for the specified interface during Data In stage.
 *
 * wIndex field specifies the interface.
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  0 on success
 */
static int srq_get_interface_if(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if (dev->configuration == 0) {
    return USB_DEV_STAGE_STALL;
  }

  if (setup_pkt->wIndexL >= dev->n_interfaces) {
    return USB_DEV_STAGE_STALL;
  }

  dev->ep0_data_ptr = dev->alt_setting + setup_pkt->wIndexL;
  return USB_DEV_STAGE_DATA_IN;
}


/**
 * @brief Set Interface
 * This request sets the interface alternate setting.
 * @note usb_20.pdf, 9.4.10
 *
 * wValue field specifies the alternate setting.
 * wIndex field specifies the interface.
 *
 * The device should pass back a STALL during Status stage
 * if the device only supports a default setting.
 *
 * @param[out] dev          usb device stack descriptor
 *
 * @return                  0 on success
 */
static int srq_set_interface_if(usb_dev_t *dev)
{
  usb_dev_ops_t *driver = dev->driver;
  usb_device_request_t *setup_pkt = &dev->setup_pkt;
  uint8_t *dsc_p;
  uint8_t alternate = 0;
  uint8_t interface = 0;
  int retval = USB_DEV_STAGE_STALL;

  if (dev->configuration == 0) {
    return retval;
  }

  if (setup_pkt->wIndexL > USB_DEV_MAX_INTERFACES) {
    return retval;
  }

  usb_dev_get_cfg_descriptor(dev, 0, &dsc_p);

  while (((usb_generic_descriptor_t *)dsc_p)->bLength) {

    usb_cfg_descriptor_t *cfg_dsc = (usb_cfg_descriptor_t *)dsc_p;
    usb_if_descriptor_t *iface_dsc = (usb_if_descriptor_t *)dsc_p;
    usb_ep_descriptor_t *ep_dsc = (usb_ep_descriptor_t *)dsc_p;

    if (cfg_dsc->bDescriptorType == DESCRIPTOR_TYPE_CONFIGURATION) {
      if (cfg_dsc->bConfigurationValue != dev->configuration) {
        dsc_p += cfg_dsc->wTotalLength;
        continue;
      }
    }

    if (iface_dsc->bDescriptorType == DESCRIPTOR_TYPE_INTERFACE) {
      interface = iface_dsc->bInterfaceNumber;
      alternate = iface_dsc->bAlternateSetting;
    }

    if (ep_dsc->bDescriptorType == DESCRIPTOR_TYPE_ENDPOINT) {
      if (interface == setup_pkt->wIndexL) {
        if (alternate == setup_pkt->wValueL) {
          if (ENDPOINT_NUM(ep_dsc->bEndpointAddress) != 0) {
            if (ep_configured(dev, ep_dsc->bEndpointAddress)) {
              ep_clear_halted(dev, ep_dsc->bEndpointAddress);
              ep_clear_stalled(dev, ep_dsc->bEndpointAddress);
              driver->ioctl(USB_DEV_SET_EP_DISABLE, ep_dsc->bEndpointAddress);
              ep_clear_configured(dev, ep_dsc->bEndpointAddress);
            }

            set_ep_configuration(dev, ep_dsc);
            dev->alt_setting[interface] = alternate;
            retval = USB_DEV_STAGE_STATUS_IN;;
          }
        }
      }
    }
    dsc_p += ((usb_generic_descriptor_t *)dsc_p)->bLength;
  }

  return retval;
}

static int srq_error(usb_dev_t *dev)
{
  (void)dev;
  return USB_DEV_STAGE_STALL;
}

/* TODO: lookup table can be downsized to 12 x 3 
 *
 * lookup table over bRequest + Recipient
 *     ((4 bit for Request Codes) << 2) | (2 bit for Recipient)
 */
usb_dev_srq_t srq_lt[64] = {
  /*  Device      ,      Interface      ,     Endpoing      ,     Other  */
  srq_get_status_dev, srq_get_status_if, srq_get_status_ep, srq_error,
  srq_clear_feature_dev, srq_error, srq_clear_feature_ep, srq_error,
  srq_error, srq_error, srq_error, srq_error, /* reserved */
  srq_set_feature_dev, srq_error, srq_set_feature_ep, srq_error,
  srq_error, srq_error, srq_error, srq_error, /* reserved */
  srq_set_address_dev, srq_error, srq_error, srq_error,
  srq_get_descriptor_dev, srq_error, srq_error, srq_error,
  srq_error, srq_error, srq_error, srq_error, /* not supported */
  srq_get_config_dev, srq_error, srq_error, srq_error,
  srq_set_config_dev, srq_error, srq_error, srq_error,
  srq_error, srq_get_interface_if, srq_error, srq_error,
  srq_error, srq_set_interface_if, srq_error, srq_error,
  srq_error, srq_error, srq_error, srq_error, /* not supported */
  srq_error, srq_error, srq_error, srq_error, /* reserved */
  srq_error, srq_error, srq_error, srq_error, /* reserved */
  srq_error, srq_error, srq_error, srq_error, /* reserved */
};

static inline int usb_dev_ep0_setup(usb_dev_t *dev)
{
  msg_t msg;
  usb_dev_ops_t *driver = dev->driver;
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  usb_dev_ep_read(0, (uint8_t *)setup_pkt, USB_DEV_EP0_MAX_PACKET);

  if (REQUEST_DIR(setup_pkt->bmRequestType) == REQUEST_DEVICE_TO_HOST) {
    dev->ep0_data_cnt_tx = setup_pkt->wLength;
  }
  else {
    dev->ep0_data_cnt_rx = setup_pkt->wLength;
  }

  driver->set_ep_toggle_bit(ENDPOINT_ADDR_IN(0));

  uint8_t srq_idx = ((setup_pkt->bRequest & 0xf) << 2)
                    | (REQUEST_RECIPIENT(setup_pkt->bmRequestType) & 0x3);

  switch (REQUEST_TYPE(setup_pkt->bmRequestType)) {

    case REQUEST_STANDARD:
      return srq_lt[srq_idx](dev);

    case REQUEST_CLASS:
      dev->ep0_data_ptr = dev->ep0_buf;
      msg.type = USB_DEV_EVENT_REQUEST_CLASS_SETUP;
      usb_dev_broadcast_class(dev, &msg);
      return msg.content.value;

    default:
      break;
  }

  return USB_DEV_STAGE_STALL;
}

static inline int usb_dev_ep0_out(usb_dev_t *dev)
{
  msg_t msg;
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if (REQUEST_DIR(setup_pkt->bmRequestType) == REQUEST_DEVICE_TO_HOST) {
    /* 3-Stage-Control-Transfer, receive ZDP */
    usb_dev_ep_read(0, dev->ep0_buf, USB_DEV_EP0_MAX_PACKET);
    return USB_DEV_STAGE_IDLE;
  }

  /* REQUEST_HOST_TO_DEVICE */
  if (dev->ep0_data_cnt_rx) {
    /* Data Out stage, receive data */
    size_t n = usb_dev_ep_read(0, dev->ep0_data_ptr, USB_DEV_EP0_MAX_PACKET);
    dev->ep0_data_cnt_rx -= n;
    dev->ep0_data_ptr += n;

    if (dev->ep0_data_cnt_rx != 0) {
      /* Data Out stage, wait for the next packet */
      return USB_DEV_STAGE_IDLE;
    }

    /* Host to device transfer complete */
    switch (REQUEST_TYPE(setup_pkt->bmRequestType)) {

      case REQUEST_STANDARD:
        return USB_DEV_STAGE_STALL;

      case REQUEST_CLASS:
        msg.type = USB_DEV_EVENT_REQUEST_CLASS_OUT;
        usb_dev_broadcast_class(dev, &msg);
        return msg.content.value;

      default:
        return USB_DEV_STAGE_STALL;
    }
  }
  return USB_DEV_STAGE_IDLE;
}

static void usb_dev_data_in_prepare(usb_dev_t *dev, size_t l)
{
  if (dev->ep0_data_cnt_tx > l) {
    dev->ep0_data_cnt_tx = l;
  }

  /*
     * Send ZLP at the end of transmission if the length is equal to or
     * a multiple of a maximum length.
     */
  if (!(dev->ep0_data_cnt_tx % USB_DEV_EP0_MAX_PACKET)) {
    dev->zero_pkt = 1;
  }
}

#if 0
static void usb_dev_data_in_stage(usb_dev_t *dev)
{
  usb_dev_ops_t *driver = dev->driver;
  size_t n;

  if (dev->ep0_data_cnt_tx > USB_DEV_EP0_MAX_PACKET) {
    n = usb_dev_ep_write(ENDPOINT_ADDR_IN(0), dev->ep0_data_ptr, USB_DEV_EP0_MAX_PACKET);
  }
  else if (dev->ep0_data_cnt_tx == 0) {
    dev->zero_pkt = 0;
    n = usb_dev_ep_write(ENDPOINT_ADDR_IN(0), dev->ep0_data_ptr, 0);
  }
  else {
    n = usb_dev_ep_write(ENDPOINT_ADDR_IN(0), dev->ep0_data_ptr, dev->ep0_data_cnt_tx);
  }

  dev->ep0_data_cnt_tx -= n;
  dev->ep0_data_ptr += n;
}
#endif

static inline int usb_dev_ep0_in(usb_dev_t *dev)
{
  usb_dev_ops_t *driver = dev->driver;
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if (REQUEST_DIR(setup_pkt->bmRequestType) == REQUEST_DEVICE_TO_HOST) {
    if (dev->ep0_data_cnt_tx || dev->zero_pkt) {
      return USB_DEV_STAGE_DATA_IN;
    }
  }
  else if (dev->configuration == 0) {
    driver->ioctl(USB_DEV_SET_ADDRESS, (uint32_t)dev->address);
  }
  return USB_DEV_STAGE_IDLE;
}

static inline void usb_dev_status_in_stage(usb_dev_t *dev)
{
  (void)dev;
  usb_dev_ep_write(ENDPOINT_ADDR_IN(0), NULL, 0);
}

static inline void usb_dev_ep0_stall_in(usb_dev_t *dev)
{
  usb_dev_ops_t *driver = dev->driver;
  driver->ioctl(USB_DEV_SET_EP_STALL, 0x80);
  dev->ep0_data_cnt_tx = 0;
}

static inline void usb_dev_ep0_stall_out(usb_dev_t *dev)
{
  usb_dev_ops_t *driver = dev->driver;
  driver->ioctl(USB_DEV_SET_EP_STALL, 0);
  dev->ep0_data_cnt_rx = 0;
}

static inline void usb_dev_ep0_stall(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if (REQUEST_DIR(setup_pkt->bmRequestType) == REQUEST_DEVICE_TO_HOST) {
    usb_dev_ep0_stall_in(dev);
  }
  else if (setup_pkt->wLength) {
    usb_dev_ep0_stall_out(dev);
  }
}

#if 0
static void *usb_dev_ep0(void *arg)
{
  //msg_t msg;
  msg_t msg, msg_q[4];
  msg_init_queue(msg_q, 4);
  usb_dev_t *dev = arg;

  while (true) {
    msg_receive(&msg);

    switch (msg.type) {
      case USB_DEV_EVENT_SETUP:
        dev->stage = usb_dev_ep0_setup(dev);
        break;

      case USB_DEV_EVENT_OUT:
        dev->stage = usb_dev_ep0_out(dev);
        break;

      case USB_DEV_EVENT_IN:
        dev->stage = usb_dev_ep0_in(dev);
        break;

      case USB_DEV_EVENT_RESET:
        dev->status  = 0;
        dev->address = 0;
        dev->configuration = 0;
        dev->stage = USB_DEV_STAGE_IDLE;
        reset_ep_status(dev);
        break;

      case USB_DEV_EVENT_ERROR:
        /* TODO */
        break;

      case USB_DEV_EVENT_SUSPEND:
        /* TODO */
        break;

      case USB_DEV_EVENT_RESUME:
        /* TODO */
        break;

      case USB_DEV_EVENT_SOF:
      default:
        continue;
    }

    if (dev->stage == USB_DEV_STAGE_DATA_IN) {
      usb_dev_data_in_stage(dev);
    }
    else if (dev->stage == USB_DEV_STAGE_STATUS_IN) {
      usb_dev_status_in_stage(dev);
    }
    else if (dev->stage == USB_DEV_STAGE_STALL) {
      usb_dev_ep0_stall(dev);
    }
  }

  return NULL;
}
#endif

inline static void usb_dev_broadcast_class(usb_dev_t *dev, msg_t *msg)
{
  kernel_pid_t tmp = KERNEL_PID_UNDEF;

  tmp = dev->ep_in_pid[1];
  if (tmp != dev->ep_out_pid[0]) {
    msg_send_receive(msg, msg, tmp);
  }

  /*
    for (uint8_t i = 1; i < 4; i++) {
        tmp = dev->ep_out_pid[i];
        if (tmp != dev->monitor_pid) {
      msg_send_receive(msg, msg, tmp);
  }
    }
    */
}

void usb_dev_irq_ep_event(usb_dev_t *dev, usb_ep_t ep, uint16_t type)
{
  kernel_pid_t tmp = KERNEL_PID_UNDEF;
  msg_t msg;
  msg.type = type;
  msg.content.value = ep;

  if (ep & ENDPOINT_IN_MASK) {
    tmp = dev->ep_in_pid[ep & 0x0f];
  }
  else {
    tmp = dev->ep_out_pid[ep & 0x0f];
  }
  msg_send_int(&msg, tmp);
}

void usb_dev_irq_bc_event(usb_dev_t *dev, uint16_t type)
{
  kernel_pid_t tmp = KERNEL_PID_UNDEF;
  msg_t msg;
  msg.type = type;
  msg.content.value = 0;

  for (uint8_t i = 1; i < USB_DEV_MAX_ENDPOINT_OUT; i++) {
    tmp = dev->ep_out_pid[i];
    if (tmp != dev->ep_out_pid[0]) {
      msg_send_int(&msg, tmp);
    }
  }
}

int usb_dev_add_string_descriptor (usb_dev_t *dev, uint8_t index, char *string)
{
  uint8_t *sdescr = NULL;
  uint8_t n = 0;

  if (!index) {
    return ENOTEMPTY;
  }

  unsigned int l = strlen(string) * 2;
  l = l > (UINT8_MAX - 2) ? (UINT8_MAX - 2): l;

  if (!l) {
    dev->sdescr_tab[index] = NULL;
    return ENODATA;
  }

  sdescr = malloc(l + 2);
  if (sdescr == NULL) {
    return ENOMEM;
  }

  sdescr[n++] = (uint8_t)l + 2;
  sdescr[n++] = DESCRIPTOR_TYPE_STRING;

  for (unsigned int j = 0; j < l / 2; j++) {
    sdescr[n++] = string[j];
    sdescr[n++] = 0;
  }

  dev->sdescr_tab[index] = sdescr;

  if (index == USB_DEV_STRING_IDX_MFR) {
    dev->device_descriptor->iManufacturer = USB_DEV_STRING_IDX_MFR;
  }
  else if (index == USB_DEV_STRING_IDX_PRODUCT) {
    dev->device_descriptor->iProduct = USB_DEV_STRING_IDX_PRODUCT;
  }
  else if (index == USB_DEV_STRING_IDX_SN) {
    dev->device_descriptor->iSerialNumber = USB_DEV_STRING_IDX_SN;
  }
  return 0;
}

int usb_dev_add_cfg_descriptor(usb_dev_t *dev, uint8_t *descriptor)
{
  for (int j = 0; j < USB_DEV_DESCRIPTOR_IDX_MAX; j++) {
    if (dev->cfgdescr_tab[j] == NULL) {
      dev->cfgdescr_tab[j] = descriptor;
      return 0;
    }
  }
  return -1;
}

static usb_dev_iface_init_t iface_init[4] = { NULL, NULL, NULL, NULL, };
static usb_dev_driver_init_t usb_driver_init = NULL;

int usb_dev_register_iface(usb_dev_iface_init_t init)
{
  iface_init[0] = init;
  return 0;
}

int usb_dev_register_driver(usb_dev_driver_init_t init)
{
  usb_driver_init = init;
  return 0;
}

//static char usb_dev_stack_ep0[512];
usb_dev_t dev;

int usb_dev_init(void)
{
  dev.driver = usb_driver_init(&dev);
  if (dev.driver == NULL) {
    return 0;
  }
  dev.irq_ep_event = usb_dev_irq_ep_event;
  dev.irq_bc_event = usb_dev_irq_bc_event;

  /*
  dev.ep_out_pid[0] = thread_create(usb_dev_stack_ep0,
                                    sizeof(usb_dev_stack_ep0),
                                    1,//THREAD_PRIORITY_MAIN,
                                    CREATE_STACKTEST | CREATE_WOUT_YIELD,
                                    usb_dev_ep0, &dev,
                                    "usb-ep0");
                                    */
  dev.ep_in_pid[0] = dev.ep_out_pid[0];

  for (uint8_t i = 1; i < USB_DEV_MAX_ENDPOINT_IN; i++) {
    dev.ep_in_pid[i] = dev.ep_out_pid[0];
  }
  for (uint8_t i = 1; i < USB_DEV_MAX_ENDPOINT_OUT; i++) {
    dev.ep_out_pid[i] = dev.ep_out_pid[0];
  }

  dev.device_descriptor = &usb_dev_def_descriptor;
  memset(dev.cfgdescr_tab, (int)NULL, USB_DEV_DESCRIPTOR_IDX_MAX);
  dev.sdescr_tab[0] = (uint8_t*)&usb_dev_def_string_descriptor;
  usb_dev_add_string_descriptor(&dev, USB_DEV_STRING_IDX_MFR, "JOHN");
  usb_dev_add_string_descriptor(&dev, USB_DEV_STRING_IDX_PRODUCT, "USB-TEST");
  usb_dev_add_string_descriptor(&dev, USB_DEV_STRING_IDX_SN, "0123");

  if (iface_init[0]) {
    iface_init[0](&dev);
  }

  dev.driver->ioctl(USB_DEV_SET_CONNECT, 1);

  return 0;
}

int usb_dev_remove(usb_dev_t *dev)
{
  for (int j = 1; j < USB_DEV_DESCRIPTOR_IDX_MAX; j++) {
    if (dev->cfgdescr_tab[j] != NULL) {
      free(dev->cfgdescr_tab[j]);
    }
  }
  return 0;
}

//module_init(usbdev_init);
