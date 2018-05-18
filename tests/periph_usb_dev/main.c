/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Manual test application for UART peripheral drivers
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "periph/usb_dev.h"

void usb_dev_reset_cb(void) {
  printf("usb_dev_reset_cb\n");
  usb_dev_ep_open(0x00, 64, USB_DEV_EP_TYPE_CTRL);
  usb_dev_ep_open(0x80, 64, USB_DEV_EP_TYPE_CTRL);
  printf("usb_dev_reset_cb\n");
}

int main(void)
{
  printf("TEST\n");
  usb_dev_init();
  printf("TEST\n");
  while(1);
  return 0;
}
