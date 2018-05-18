/*
 * Copyright (C) 2015 Phytec Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     usb_dev_cdcacm
 * @{
 *
 * @file
 * @brief       CDC ACM Class
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 *
 * @}
 */
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "ringbuffer.h"
#include "thread.h"
#include "board.h"
#include "mutex.h"

#include "usb_types.h"
#include "usb_dev.h"

#define UART_ACM_0_EN                   1
#include "periph/uart.h"

#define USB_DEV_ACM_EP_PACKET_SIZE       64
#define USB_DEV_ACM_EP_INTIN             1
#define USB_DEV_ACM_POLL_IVAL            32
#define USB_DEV_ACM_EP_BULKIN            2
#define USB_DEV_ACM_EP_BULKOUT           2

#ifndef USB_DEV_MAX_POWER
#define USB_DEV_MAX_POWER                200
#endif

#if (USB_DEV_ACM_EP_INTIN >= USB_DEV_MAX_ENDPOINT_IN)
#error "USB_DEV_ACM_EP_INTIN exceeds the maximum number of IN endpoints."
#endif

#if (USB_DEV_ACM_EP_BULKIN >= USB_DEV_MAX_ENDPOINT_IN)
#error "USB_DEV_ACM_EP_BULKIN exceeds the maximum number of IN endpoints."
#endif

#if (USB_DEV_ACM_EP_BULKOUT >= USB_DEV_MAX_ENDPOINT_OUT)
#error "USB_DEV_ACM_EP_BULKOUT exceeds the maximum number of OUT endpoints."
#endif

void usb_dev_acm_evt_in(usb_dev_t *dev);

#ifndef CDCACM_BUFSIZE
#define CDCACM_BUFSIZE                  128
#endif

static mutex_t tx_rb_lock = MUTEX_INIT;
static char rx_buffer[CDCACM_BUFSIZE];
static char tx_buffer[CDCACM_BUFSIZE];
ringbuffer_t cdcacm_rx_rb = RINGBUFFER_INIT(rx_buffer);
ringbuffer_t cdcacm_tx_rb = RINGBUFFER_INIT(tx_buffer);

uint8_t rec_buffer[USB_DEV_ACM_EP_PACKET_SIZE];
uint8_t send_buffer[USB_DEV_ACM_EP_PACKET_SIZE];

static uart_isr_ctx_t ucb_config;

typedef struct __attribute__((packed))
{
  usb_cfg_descriptor_t cfg;

  usb_if_descriptor_t if0;
  cdc_header_descriptor_t if0_header;
  cdc_cm_descriptor_t if0_cm;
  cdc_acm_descriptor_t if0_acm;
  cdc_union_descriptor_t if0_union;
  usb_ep_descriptor_t if0_ep_int_in;

  usb_if_descriptor_t if1;
  usb_ep_descriptor_t if1_ep_bulk_out;
  usb_ep_descriptor_t if1_ep_bulk_in;

  usb_generic_descriptor_t terminator;
}
usb_cdc_acm_config_t;

usb_cdc_acm_config_t usb_cdc_acm_config = {
  .cfg = {
    .bLength = sizeof(usb_cfg_descriptor_t),
    .bDescriptorType = DESCRIPTOR_TYPE_CONFIGURATION,
    .wTotalLength = sizeof(usb_cdc_acm_config_t) - sizeof(usb_generic_descriptor_t),
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = CONFIG_BUS_POWERED,
    .bMaxPower = CONFIG_POWER_MA(USB_DEV_MAX_POWER),
  },

  .if0 = {
    .bLength = sizeof(usb_if_descriptor_t),
    .bDescriptorType = DESCRIPTOR_TYPE_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = COMMUNICATION_INTERFACE_CLASS,
    .bInterfaceSubClass = ABSTRACT_CONTROL_MODEL,
    .bInterfaceProtocol = 1, /* TODO: fix AT-commands (v.25ter) */
    .iInterface = USB_DEV_STRING_IDX_CIF,
  },

  .if0_header = {
    .bFunctionLength = 5,
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = HEADER_FUNC_DESCRIPTOR,
    .bcdCDC = CDC_SRN_1_20,
  },

  .if0_cm = {
    .bFunctionLength = 5,
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = CALL_MANAGEMENT_FUNC_DESCRIPTOR,
    .bmCapabilities = 3,
    .bDataInterface = 2,
  },

  .if0_acm = {
    .bFunctionLength = 4,
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = ABSTRACT_CONTROL_MANAGEMENT_FUNC_DESCRIPTOR,
    .bmCapabilities = 0,    /* TODO: add set/get line coding ... */
  },

  .if0_union = {
    .bFunctionLength = 5,
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = UNION_FUNC_DESCRIPTOR,
    .bControlInterface = 0,
    .bSubordinateInterface0 = 1,
  },

  .if0_ep_int_in = {
    .bLength = sizeof(usb_ep_descriptor_t),
    .bDescriptorType = DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = ENDPOINT_ADDR_IN(USB_DEV_ACM_EP_INTIN),
    .bmAttributes = ENDPOINT_TRANSFER_TYPE_INTERRUPT,
    .wMaxPacketSize = USB_DEV_ACM_EP_PACKET_SIZE,
    .bInterval = USB_DEV_ACM_POLL_IVAL,
  },

  .if1 = {
    .bLength = sizeof(usb_if_descriptor_t),
    .bDescriptorType = DESCRIPTOR_TYPE_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = DATA_INTERFACE_CLASS,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = USB_DEV_STRING_IDX_DIF,
  },

  .if1_ep_bulk_out = {
    .bLength = sizeof(usb_ep_descriptor_t),
    .bDescriptorType = DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = ENDPOINT_ADDR_OUT(USB_DEV_ACM_EP_BULKOUT),
    .bmAttributes = ENDPOINT_TRANSFER_TYPE_BULK,
    .wMaxPacketSize = USB_DEV_ACM_EP_PACKET_SIZE,
    .bInterval = 0x00,
  },

  .if1_ep_bulk_in = {
    .bLength = sizeof(usb_ep_descriptor_t),
    .bDescriptorType = DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = ENDPOINT_ADDR_IN(USB_DEV_ACM_EP_BULKIN),
    .bmAttributes = ENDPOINT_TRANSFER_TYPE_BULK,
    .wMaxPacketSize = USB_DEV_ACM_EP_PACKET_SIZE,
    .bInterval = 0x00,
  },

  .terminator = {
    .bLength = 0,
    .bDescriptorType = 0,
  }
};

inline static int usb_dev_cdc_wrong_iface(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if (setup_pkt->wIndexL == usb_cdc_acm_config.if0.bInterfaceNumber) {
    return 0;
  }
  if (setup_pkt->wIndexL == usb_cdc_acm_config.if1.bInterfaceNumber) {
    return 0;
  }
  return -1;
};

inline static int usb_dev_cdc_request_class_if(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if (usb_dev_cdc_wrong_iface(dev)) {
    return -1;
  }

  switch (setup_pkt->bRequest) {
    case SET_LINE_CODING: //0x20 (32)
      //dev->ep0_data_ptr = dev->ep0_buf;
      return USB_DEV_STAGE_STATUS_IN;

    case SET_CONTROL_LINE_STATE: //0x22 (34)
      return USB_DEV_STAGE_STATUS_IN;

    case SET_COMM_FEATURE:
    case SET_LINE_PARMS:
    case GET_COMM_FEATURE:
    case CLEAR_COMM_FEATURE:
    case GET_LINE_CODING:
    case SEND_BREAK:
    default:
      break;
  }

  return USB_DEV_STAGE_STALL;
}

inline static int usb_dev_cdc_out_class_if(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  if (usb_dev_cdc_wrong_iface(dev)) {
    return -1;
  }

  switch (setup_pkt->bRequest) {
    case SET_LINE_CODING:
      return USB_DEV_STAGE_STATUS_IN;

    case SET_COMM_FEATURE:
    default:
      break;
  }

  return USB_DEV_STAGE_STALL;
}

inline static void usb_dev_acm_reset(void)
{
  ringbuffer_init(&cdcacm_tx_rb, tx_buffer, CDCACM_BUFSIZE);
  ringbuffer_init(&cdcacm_rx_rb, rx_buffer, CDCACM_BUFSIZE);
  /* TODO: reset data_send_zlp .... */
  return;
}

/* TODO */
/*
static int usb_dev_acm_notify_host(usb_dev_t *dev, uint16_t stat)
{
    usb_cdc_notification_t notification;
    usb_dev_ops_t *driver = dev->driver;

    if (!dev->configuration) {
        return 1;
    }

    notification.bmRequestType = REQUEST_TO_HOST(REQUEST_CLASS, REQUEST_TO_INTERFACE);
    notification.bNotification = SERIAL_STATE;
    notification.wValue = 0;
    notification.wIndex = 0;
    notification.wLength = 2;
    notification.Data = stat;

    driver->write_ep(ENDPOINT_ADDR_IN(USB_DEV_ACM_EP_INTIN),
                     (uint8_t*)&notification, sizeof(notification));
    return 0;

}
*/

static volatile uint32_t acm_error_log;
static volatile uint32_t start_tx;

inline static void usb_dev_acm_sof_event(usb_dev_t *dev)
{
  //LED_B_TOGGLE;
  if (mutex_trylock(&tx_rb_lock)) {
    acm_error_log = cdcacm_tx_rb.avail;
    //if (!ringbuffer_empty(&cdcacm_tx_rb)) {
    if (cdcacm_tx_rb.avail) {
      start_tx = 42;
      mutex_unlock(&tx_rb_lock);
      usb_dev_acm_evt_in(dev);
      return;
    }
    start_tx = 0;
    mutex_unlock(&tx_rb_lock);
  }
}

void usb_dev_acm_evt_out(usb_dev_t *dev)
{
  usb_dev_ops_t *driver = dev->driver;

  size_t l = driver->read_ep(ENDPOINT_ADDR_OUT(USB_DEV_ACM_EP_BULKOUT),
                             rec_buffer, USB_DEV_ACM_EP_PACKET_SIZE);

  ringbuffer_add(&cdcacm_rx_rb, (char*)rec_buffer, l);

  if (ucb_config.rx_cb != NULL) {
    int retval = ringbuffer_get_one(&cdcacm_rx_rb);
    while (retval != -1) {
      ucb_config.rx_cb(ucb_config.arg, (char)retval);
      retval = ringbuffer_get_one(&cdcacm_rx_rb);
    }
  }
}

void usb_dev_acm_evt_in(usb_dev_t *dev)
{
  usb_dev_ops_t *driver = dev->driver;
  static int32_t data_send_zlp = 0;

  if (!start_tx) {
    return;
  }

  if (!mutex_trylock(&tx_rb_lock)) {
    return;
  }

  size_t len_to_send = ringbuffer_get(&cdcacm_tx_rb, (char*)send_buffer,
                                      USB_DEV_ACM_EP_PACKET_SIZE);

  if (len_to_send == 0) {
    start_tx = 0;
    if (data_send_zlp == 0) {
      mutex_unlock(&tx_rb_lock);
      return;
    }
    data_send_zlp = 0;
  }

  size_t len_sent = driver->write_ep(ENDPOINT_ADDR_IN(USB_DEV_ACM_EP_BULKIN),
                                     send_buffer, len_to_send);


  unsigned retval = ringbuffer_empty(&cdcacm_tx_rb);
  if (retval && (len_sent == USB_DEV_ACM_EP_PACKET_SIZE)) {
    data_send_zlp = 1;
  }
  else {
    data_send_zlp = 0;
  }
  mutex_unlock(&tx_rb_lock);
}

static inline int usb_dev_setup_request_class(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  switch (REQUEST_RECIPIENT(setup_pkt->bmRequestType)) {

    case REQUEST_TO_INTERFACE:
      return usb_dev_cdc_request_class_if(dev);

    case REQUEST_TO_DEVICE:
    case REQUEST_TO_ENDPOINT:
    default:
      break;
  }
  return USB_DEV_STAGE_STALL;
}

static inline int usb_dev_request_class_out(usb_dev_t *dev)
{
  usb_device_request_t *setup_pkt = &dev->setup_pkt;

  switch (REQUEST_RECIPIENT(setup_pkt->bmRequestType)) {

    case REQUEST_TO_INTERFACE:
      return usb_dev_cdc_out_class_if(dev);

    case REQUEST_TO_DEVICE:
    case REQUEST_TO_ENDPOINT:
    default:
      break;
  }
  return USB_DEV_STAGE_STALL;
}

#if 0
static void *usb_dev_cdc_acm(void *arg)
{
  msg_t msg, reply, msg_q[8];
  msg_init_queue(msg_q, 8);
  usb_dev_t *dev = arg;

  while (true) {
    msg_receive(&msg);
    reply.type = msg.type;
    usb_ep_t ep = msg.content.value;

    switch (msg.type) {
      case USB_DEV_EVENT_OUT:
        usb_dev_acm_evt_out(dev);
        break;

      case USB_DEV_EVENT_IN:
        if (ep == ENDPOINT_ADDR_IN(USB_DEV_ACM_EP_BULKIN)) {
          usb_dev_acm_evt_in(dev);
        }
        break;

      case USB_DEV_EVENT_SOF:
        if (ep_configured(dev, ENDPOINT_ADDR_IN(USB_DEV_ACM_EP_BULKIN))) {
          usb_dev_acm_sof_event(dev);
        }
        break;

      case USB_DEV_EVENT_REQUEST_CLASS_SETUP:
        reply.content.value = usb_dev_setup_request_class(dev);
        msg_reply(&msg, &reply);
        break;

      case USB_DEV_EVENT_REQUEST_CLASS_OUT:
        reply.content.value = usb_dev_request_class_out(dev);
        msg_reply(&msg, &reply);
        break;

      case USB_DEV_EVENT_RESET:
        usb_dev_acm_reset();
        break;

      default:
        break;
    }
  }

  return NULL;
}
#endif

#if 0
int acm_uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
  usb_dev_acm_reset();

  ucb_config.rx_cb = rx_cb;
  ucb_config.arg = arg;

  return 0;
}

void acm_uart_write(uart_t uart, const uint8_t *data, size_t len)
{
  mutex_lock(&tx_rb_lock);
  ringbuffer_add(&cdcacm_tx_rb, (char*)data, len);
  mutex_unlock(&tx_rb_lock);
}

uartdev_ops_t acm_uart_ops = {
  .dev = UART_ACM_0,
  .uart_init = acm_uart_init,
  .uart_write = acm_uart_write,
  .uart_poweron = NULL,
  .uart_poweroff = NULL,
};

int usb_dev_acm_uart_init (void)
{
  uartdev_register_driver(&acm_uart_ops);
  return 0;
}
#endif
//static char usb_dev_stack_acm[512];

int usb_dev_acm_init(usb_dev_t *dev)
{
  usb_dev_add_string_descriptor(dev, USB_DEV_STRING_IDX_CIF, "USB-CDC");
  usb_dev_add_string_descriptor(dev, USB_DEV_STRING_IDX_DIF, "CDC-ACM");
  usb_dev_add_cfg_descriptor(dev, (uint8_t*)(&usb_cdc_acm_config));

  dev->device_descriptor->bDeviceClass = COMMUNICATION_DEVICE_CLASS;
  dev->device_descriptor->bDeviceSubClass = 0;
  dev->device_descriptor->bDeviceProtocol = 0;

  usb_dev_acm_reset();

  kernel_pid_t tmp = KERNEL_PID_UNDEF;
  /*
    kernel_pid_t tmp = thread_create(usb_dev_stack_acm,
                                sizeof(usb_dev_stack_acm),
                                THREAD_PRIORITY_MAIN / 2,
                                CREATE_STACKTEST | CREATE_WOUT_YIELD,
                                usb_dev_cdc_acm, dev,
                                "cdcacm");
                                */

  dev->ep_in_pid[USB_DEV_ACM_EP_INTIN] = tmp;
  dev->ep_in_pid[USB_DEV_ACM_EP_BULKIN] = tmp;
  dev->ep_out_pid[USB_DEV_ACM_EP_BULKOUT] = tmp;
  return 0;
}

int usb_dev_acm_register(void)
{
  usb_dev_register_iface(usb_dev_acm_init);
  return 0;
}

//submod_init(usbdev_acm_register);
//driver_init(usbdev_acm_uart_init);
