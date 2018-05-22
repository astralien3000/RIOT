/*
 * Copyright (C) 2018 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_usb_dev
 * @{
 *
 * @file
 * @brief       Low-level USB_DEV driver implementation
 *
 * @author      Loïc Dauphin <loic.dauphin@inria.fr>
 *
 * @}
 */

#include "cpu.h"
#include "periph/gpio.h"
#include "periph/usb_dev.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define USBx ((USB_OTG_GlobalTypeDef*)USB_OTG_FS_PERIPH_BASE)

#define ENABLE 1
#define DISABLE 0

#define USBx_PCGCCTL    *(__IO uint32_t *)((uint32_t)USBx + USB_OTG_PCGCCTL_BASE)
#define USBx_HPRT0      *(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE)

#define USBx_DEVICE     ((USB_OTG_DeviceTypeDef *)((uint32_t )USBx + USB_OTG_DEVICE_BASE))
#define USBx_INEP(i)    ((USB_OTG_INEndpointTypeDef *)((uint32_t)USBx + USB_OTG_IN_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USBx_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USBx + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USBx_DFIFO(i) *(__IO uint32_t *)((uint32_t)USBx + USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE)

#define DCFG_FRAME_INTERVAL_80                 0
#define DCFG_FRAME_INTERVAL_85                 1
#define DCFG_FRAME_INTERVAL_90                 2
#define DCFG_FRAME_INTERVAL_95                 3

#define USB_OTG_ULPI_PHY     1
#define USB_OTG_EMBEDDED_PHY 2

#define USB_OTG_SPEED_HIGH                     0
#define USB_OTG_SPEED_HIGH_IN_FULL             1
#define USB_OTG_SPEED_LOW                      2
#define USB_OTG_SPEED_FULL                     3

#define DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ     (0 << 1)
#define DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ     (1 << 1)
#define DSTS_ENUMSPD_LS_PHY_6MHZ               (2 << 1)
#define DSTS_ENUMSPD_FS_PHY_48MHZ              (3 << 1)

#define EP_TYPE_CTRL                           0
#define EP_TYPE_ISOC                           1
#define EP_TYPE_BULK                           2
#define EP_TYPE_INTR                           3
#define EP_TYPE_MSK                            3

#define STS_GOUT_NAK                           1
#define STS_DATA_UPDT                          2
#define STS_XFER_COMP                          3
#define STS_SETUP_COMP                         4
#define STS_SETUP_UPDT                         6

#define CFG_PHY_ITFACE USB_OTG_EMBEDDED_PHY
#define CFG_USE_EXTERNAL_VBUS 1
#define CFG_DMA_ENABLE DISABLE
#define CFG_VBUS_SENSING_ENABLE DISABLE
#define CFG_SPEED USB_OTG_SPEED_HIGH
#define CFG_SOF_ENABLE DISABLE
#define CFG_DEV_ENDPOINTS 4
#define CFG_USE_DEDICATED_EP1 DISABLE

typedef enum
{
  USB_OTG_DEVICE_MODE  = 0,
  USB_OTG_HOST_MODE    = 1,
  USB_OTG_DRD_MODE     = 2

}USB_OTG_ModeTypeDef;

static inline void _usb_set_device_mode(void)
{
  USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
  USBx->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
  //HAL_Delay(50);
  //return HAL_OK;
}


static inline void _usb_set_dev_speed(uint8_t speed)
{
  USBx_DEVICE->DCFG |= speed;
  //return HAL_OK;
}

static inline void _usb_flush_tx_fifo(uint32_t num)
{
  uint32_t count = 0;

  USBx->GRSTCTL = ( USB_OTG_GRSTCTL_TXFFLSH |(uint32_t)( num << 5 ));

  do
  {
    if (++count > 200000)
    {
      //return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

  //return HAL_OK;
}

static inline uint8_t _usb_get_dev_speed(void)
{
  uint8_t speed = 0;

  if((USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD) == DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ)
  {
    speed = USB_OTG_SPEED_HIGH;
  }
  else if (((USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD) == DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ)||
           ((USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD) == DSTS_ENUMSPD_FS_PHY_48MHZ))
  {
    speed = USB_OTG_SPEED_FULL;
  }
  else if((USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD) == DSTS_ENUMSPD_LS_PHY_6MHZ)
  {
    speed = USB_OTG_SPEED_LOW;
  }

  return speed;
}

static inline void _usb_flush_rx_fifo(void)
{
  uint32_t count = 0;

  USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

  do
  {
    if (++count > 200000)
    {
      //return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

  //return HAL_OK;
}

static inline void _usb_dev_connect(void)
{
  USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS ;
  for(volatile int i = 500000 ; i ; i--);
  //HAL_Delay(3);
  //return HAL_OK;
}


static inline void _usb_dev_disconnect(void)
{
  USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS ;
  for(volatile int i = 500000 ; i ; i--);
  //HAL_Delay(3);
  //return HAL_OK;
}

static inline void _usb_enable_global_int(void)
{
  USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
  //return HAL_OK;
}


static inline void _usb_core_reset(void)
{
  uint32_t count = 0;

  /* Wait for AHB master IDLE state. */
  DEBUG("Wait for AHB master IDLE state\n");
  do
  {
    if (++count > 200000)
    {
      //return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0);

  /* Core Soft Reset */
  DEBUG("Core Soft Reset\n");
  count = 0;
  USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

  do
  {
    if (++count > 200000)
    {
      //return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);

  //return HAL_OK;
}

static inline void _usb_core_init(void) {
  if (CFG_PHY_ITFACE == USB_OTG_ULPI_PHY)
  {
    USBx->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);

    /* Init The ULPI Interface */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

    /* Select vbus source */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);
    if(CFG_USE_EXTERNAL_VBUS == 1)
    {
      USBx->GUSBCFG |= USB_OTG_GUSBCFG_ULPIEVBUSD;
    }

    /* Reset after a PHY select  */
    _usb_core_reset();
  }
  else /* FS interface (embedded Phy) */
  {

    /* Select FS Embedded PHY */
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

    /* Reset after a PHY select and set Host mode */
    _usb_core_reset();

    /* Deactivate the power down*/
    USBx->GCCFG = USB_OTG_GCCFG_PWRDWN;
  }

  if(CFG_DMA_ENABLE == ENABLE)
  {
    USBx->GAHBCFG |= (USB_OTG_GAHBCFG_HBSTLEN_1 | USB_OTG_GAHBCFG_HBSTLEN_2);
    USBx->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;
  }
}

static inline void _usb_dev_init(void)
{
  uint32_t i = 0;

  /*Activate VBUS Sensing B */
  USBx->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;

  if (CFG_VBUS_SENSING_ENABLE == 0)
  {
    USBx->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
  }

  /* Restart the Phy Clock */
  USBx_PCGCCTL = 0;

  /* Device mode configuration */
  USBx_DEVICE->DCFG |= DCFG_FRAME_INTERVAL_80;

  if(CFG_PHY_ITFACE == USB_OTG_ULPI_PHY)
  {
    if(CFG_SPEED == USB_OTG_SPEED_HIGH)
    {
      /* Set High speed phy */
      _usb_set_dev_speed(USB_OTG_SPEED_HIGH);
    }
    else
    {
      /* set High speed phy in Full speed mode */
      _usb_set_dev_speed(USB_OTG_SPEED_HIGH_IN_FULL);
    }
  }
  else
  {
    /* Set Full speed phy */
    _usb_set_dev_speed(USB_OTG_SPEED_FULL);
  }

  /* Flush the FIFOs */
  _usb_flush_tx_fifo(0x10); /* all Tx FIFOs */
  _usb_flush_rx_fifo();


  /* Clear all pending Device Interrupts */
  USBx_DEVICE->DIEPMSK = 0;
  USBx_DEVICE->DOEPMSK = 0;
  USBx_DEVICE->DAINT = 0xFFFFFFFF;
  USBx_DEVICE->DAINTMSK = 0;

  for (i = 0 ; i < CFG_DEV_ENDPOINTS ; i++)
  {
    if ((USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
    {
      USBx_INEP(i)->DIEPCTL = (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);
    }
    else
    {
      USBx_INEP(i)->DIEPCTL = 0;
    }

    USBx_INEP(i)->DIEPTSIZ = 0;
    USBx_INEP(i)->DIEPINT  = 0xFF;
  }

  for (i = 0 ; i < CFG_DEV_ENDPOINTS ; i++)
  {
    if ((USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
    {
      USBx_OUTEP(i)->DOEPCTL = (USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK);
    }
    else
    {
      USBx_OUTEP(i)->DOEPCTL = 0;
    }

    USBx_OUTEP(i)->DOEPTSIZ = 0;
    USBx_OUTEP(i)->DOEPINT  = 0xFF;
  }

  USBx_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

  if (CFG_DMA_ENABLE == 1)
  {
    /*Set threshold parameters */
    USBx_DEVICE->DTHRCTL = (USB_OTG_DTHRCTL_TXTHRLEN_6 | USB_OTG_DTHRCTL_RXTHRLEN_6);
    USBx_DEVICE->DTHRCTL |= (USB_OTG_DTHRCTL_RXTHREN | USB_OTG_DTHRCTL_ISOTHREN | USB_OTG_DTHRCTL_NONISOTHREN);

    i= USBx_DEVICE->DTHRCTL;
  }

  /* Disable all interrupts. */
  USBx->GINTMSK = 0;

  /* Clear any pending interrupts */
  USBx->GINTSTS = 0xBFFFFFFF;

  /* Enable the common interrupts */
  if (CFG_DMA_ENABLE == DISABLE)
  {
    USBx->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
  }

  /* Enable interrupts matching to the Device mode ONLY */
  USBx->GINTMSK |= (USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |\
                    USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |\
                    USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM|\
                    USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM);

  if(CFG_SOF_ENABLE)
  {
    USBx->GINTMSK |= USB_OTG_GINTMSK_SOFM;
  }

  if (CFG_VBUS_SENSING_ENABLE == ENABLE)
  {
    USBx->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);
  }

  //return HAL_OK;
}

void _usb_pcd_msp_init(void) {
  /* Configure USB FS GPIOs */
  //__GPIOA_CLK_ENABLE();

  /* Configure DM DP Pins */
  //GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  //GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  gpio_init(GPIO_PIN(PORT_A, 11), GPIO_OD);
  gpio_init_af(GPIO_PIN(PORT_A, 11), GPIO_AF10);

  gpio_init(GPIO_PIN(PORT_A, 12), GPIO_OD);
  gpio_init_af(GPIO_PIN(PORT_A, 12), GPIO_AF10);

  /* Configure VBUS Pin */
  //GPIO_InitStruct.Pin = GPIO_PIN_9;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  gpio_init(GPIO_PIN(PORT_A, 9), GPIO_IN);

  /* This for ID line debug */
  //  GPIO_InitStruct.Pin = GPIO_PIN_10;
  //  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  //  GPIO_InitStruct.Pull = GPIO_PULLUP;
  //  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  //  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  //  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  gpio_init(GPIO_PIN(PORT_A, 10), GPIO_OD_PU);
  gpio_init_af(GPIO_PIN(PORT_A, 10), GPIO_AF10);

  /* Enable USB FS Clocks */
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  /* Set USBFS Interrupt to the lowest priority */
  NVIC_SetPriority(OTG_FS_IRQn, 3);

  /* Enable USBFS Interrupt */
  NVIC_EnableIRQ(OTG_FS_IRQn);
}

int usb_dev_init(void) {
  // HAL_PCD_MspInit
  DEBUG("HAL_PCD_MspInit\n");
  _usb_pcd_msp_init();

  // USB_CoreInit
  DEBUG("USB_CoreInit\n");
  _usb_core_init();

  // USB_SetCurrentMode USB_OTG_DEVICE_MODE
  DEBUG("USB_SetCurrentMode\n");
  _usb_set_device_mode();

  // Init endpoints structures
  DEBUG("Init endpoints structures\n");

  // USB_DevInit
  DEBUG("USB_DevInit\n");
  _usb_dev_init();

  // USB_DevDisconnect
  DEBUG("USB_DevDisconnect\n");
  _usb_dev_disconnect();

  DEBUG("USB_Start\n");
  _usb_dev_connect();
  _usb_enable_global_int();

  return USB_DEV_OK;
}

#define PRINTREG(reg) do { \
  for(int i = 32 ; i ; i--) { \
  DEBUG(((reg >> i) & 1)?"1":" "); \
  } \
  DEBUG(" : "#reg"\n"); \
  } while(0)

static inline void _usb_ep0_out_start(uint8_t *psetup)
{
  USBx_OUTEP(0)->DOEPTSIZ = 0;
  USBx_OUTEP(0)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19)) ;
  USBx_OUTEP(0)->DOEPTSIZ |= (3 * 8);
  USBx_OUTEP(0)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;

  if (CFG_DMA_ENABLE == 1)
  {
    USBx_OUTEP(0)->DOEPDMA = (uint32_t)psetup;
    /* EP enable */
    USBx_OUTEP(0)->DOEPCTL = 0x80008000;
  }

  //return HAL_OK;
}

static inline void _usb_activate_setup(void)
{
  /* Set the MPS of the IN EP based on the enumeration speed */
  USBx_INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;

  if((USBx_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD) == DSTS_ENUMSPD_LS_PHY_6MHZ)
  {
    USBx_INEP(0)->DIEPCTL |= 3;
  }
  USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;

  //return HAL_OK;
}

extern void usb_dev_reset_cb(void);

#define PACKED __attribute__((packed))
#define PACKED_STRUCT struct PACKED
#define PACKED_UNION union PACKED

typedef PACKED_STRUCT {
  PACKED_UNION {
    PACKED_STRUCT {
      uint8_t   bmRequest;
      uint8_t   bRequest;
      uint16_t  wValue;
      uint16_t  wIndex;
      uint16_t  wLength;
    };
    PACKED_STRUCT {
      uint32_t u32Raw[2];
    };
  };
} usbd_setup_req_t;

void USB_ReadPacket(uint8_t *dest, uint16_t len)
{
  uint32_t i=0;
  uint32_t count32b = (len + 3) / 4;

  usbd_setup_req_t req;

  for ( i = 0; i < count32b; i++, dest += 4 )
  {
    //*(__packed uint32_t *)dest = USBx_DFIFO(0);
    req.u32Raw[i] = USBx_DFIFO(0);
  }

  DEBUG("bmRequest = %lu\n", (uint32_t)req.bmRequest);
  DEBUG("bRequest = %lu\n", (uint32_t)req.bRequest);
  DEBUG("wValue = %lu\n", (uint32_t)req.wValue);
  DEBUG("wIndex = %lu\n", (uint32_t)req.wIndex);
  DEBUG("wLength = %lu\n", (uint32_t)req.wLength);
}

uint32_t USB_ReadDevAllOutEpInterrupt (void)
{
  uint32_t v;
  v  = USBx_DEVICE->DAINT;
  v &= USBx_DEVICE->DAINTMSK;
  return ((v & 0xffff0000) >> 16);
}

uint32_t USB_ReadDevOutEPInterrupt (uint8_t epnum)
{
  uint32_t v;
  v  = USBx_OUTEP(epnum)->DOEPINT;
  v &= USBx_DEVICE->DOEPMSK;
  return v;
}

void HAL_PCD_DataOutStageCallback(uint8_t epnum)
{
  DEBUG("HAL_PCD_DataOutStageCallback %lu\n", (uint32_t)epnum);
}

void HAL_PCD_SetupStageCallback(void)
{
  DEBUG("HAL_PCD_SetupStageCallback\n");
}

void isr_otg_fs(void) {
  DEBUG("isr_otg_fs\n");
  PRINTREG(USBx->GINTMSK);
  PRINTREG(USBx->GINTSTS);
  PRINTREG(USB_OTG_GINTSTS_MMIS);
  PRINTREG(USB_OTG_GINTSTS_OEPINT);
  PRINTREG(USB_OTG_GINTSTS_IEPINT);
  PRINTREG(USB_OTG_GINTSTS_WKUINT);
  PRINTREG(USB_OTG_GINTSTS_USBSUSP);
  PRINTREG(USB_OTG_GINTSTS_USBRST);
  PRINTREG(USB_OTG_GINTSTS_ENUMDNE);
  PRINTREG(USB_OTG_GINTSTS_RXFLVL);
  PRINTREG(USB_OTG_GINTSTS_SOF);
  PRINTREG(USB_OTG_GINTSTS_IISOIXFR);
  PRINTREG(USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
  PRINTREG(USB_OTG_GINTSTS_SRQINT);
  PRINTREG(USB_OTG_GINTSTS_OTGINT);

  /* Handle Reset Interrupt */
  if(USBx->GINTSTS & USB_OTG_GINTSTS_USBRST)
  {
    USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
    _usb_flush_tx_fifo(0);

    for (int i = 0 ; i < CFG_DEV_ENDPOINTS ; i++)
    {
      USBx_INEP(i)->DIEPINT = 0xFF;
      USBx_OUTEP(i)->DOEPINT = 0xFF;
    }
    USBx_DEVICE->DAINT = 0xFFFFFFFF;
    USBx_DEVICE->DAINTMSK |= 0x10001;

    if(CFG_USE_DEDICATED_EP1)
    {
      USBx_DEVICE->DOUTEP1MSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM);
      USBx_DEVICE->DINEP1MSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);
    }
    else
    {
      USBx_DEVICE->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM);
      USBx_DEVICE->DIEPMSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);
    }

    /* Set Default Address to 0 */
    USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

    /* setup EP0 to receive SETUP packets */
    //_usb_ep0_out_start((uint8_t *)hpcd->Setup);
    _usb_ep0_out_start((uint8_t *)0);

    /* Clear Flag */
    //__HAL_PCD_CLEAR_FLAG(USBx, USB_OTG_GINTSTS_USBRST);
    USBx->GINTSTS = USB_OTG_GINTSTS_USBRST;
  }

  /* Handle Enumeration done Interrupt */
  if(USBx->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
  {
    _usb_activate_setup();
    USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;

    if (_usb_get_dev_speed() == USB_OTG_SPEED_HIGH)
    {
      //hpcd->Init.speed            = USB_OTG_SPEED_HIGH;
      //hpcd->Init.ep0_mps          = USB_OTG_HS_MAX_PACKET_SIZE ;
      USBx->GUSBCFG |= (USB_OTG_GUSBCFG_TRDT_0 | USB_OTG_GUSBCFG_TRDT_3);
    }
    else
    {
      //hpcd->Init.speed            = USB_OTG_SPEED_FULL;
      //hpcd->Init.ep0_mps          = USB_OTG_FS_MAX_PACKET_SIZE ;
      USBx->GUSBCFG |= (USB_OTG_GUSBCFG_TRDT_0 | USB_OTG_GUSBCFG_TRDT_2);
    }

    //HAL_PCD_ResetCallback(hpcd);
    usb_dev_reset_cb();

    /* Clear Flag */
    //__HAL_PCD_CLEAR_FLAG(USBx, USB_OTG_GINTSTS_ENUMDNE);
    USBx->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
  }

  if(USBx->GINTSTS & USB_OTG_GINTSTS_USBSUSP)
  {
    if((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS)
    {
      //HAL_PCD_SuspendCallback(hpcd);
    }
    USBx->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
  }

  /* Handle RxQLevel Interrupt */
  if(USBx->GINTSTS & USB_OTG_GINTSTS_RXFLVL)
  {
    DEBUG("RxQLvl\n");
    USBx->GINTMSK &= ~USB_OTG_GINTSTS_RXFLVL;
    const uint32_t temp = USBx->GRXSTSP;
    const uint32_t ep_num = temp & USB_OTG_GRXSTSP_EPNUM;
    //USB_OTG_EPTypeDef* ep = &hpcd->OUT_ep[temp & USB_OTG_GRXSTSP_EPNUM];
    DEBUG("EP%lu\n", ep_num);

    if(((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_DATA_UPDT)
    {
      if((temp & USB_OTG_GRXSTSP_BCNT) != 0)
      {
        DEBUG("DATA\n");
        //USB_ReadPacket(USBx, ep->xfer_buff, (temp & USB_OTG_GRXSTSP_BCNT) >> 4);
        USB_ReadPacket(NULL, (temp & USB_OTG_GRXSTSP_BCNT) >> 4);
        //ep->xfer_buff += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
        //ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
      }
    }
    else if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) ==  STS_SETUP_UPDT)
    {
      DEBUG("SETUP\n");
      //USB_ReadPacket(USBx, (uint8_t *)hpcd->Setup, 8);
      USB_ReadPacket(NULL, 8);
      //ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
    }
    USBx->GINTMSK |= USB_OTG_GINTSTS_RXFLVL;
  }

  if(USBx->GINTSTS & USB_OTG_GINTSTS_OEPINT)
  {
    DEBUG("OEPINT\n");
    uint32_t epnum = 0;

    /* Read in the device interrupt bits */
    uint32_t ep_intr = USB_ReadDevAllOutEpInterrupt();

    while ( ep_intr )
    {
      if (ep_intr & 0x1)
      {
        uint32_t epint = USB_ReadDevOutEPInterrupt(epnum);

        if(( epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC)
        {
          USBx_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_XFRC;

          if(CFG_DMA_ENABLE == 1)
          {
            //hpcd->OUT_ep[epnum].xfer_count = hpcd->OUT_ep[epnum].maxpacket- (USBx_OUTEP(epnum)->DOEPTSIZ & USB_OTG_DOEPTSIZ_XFRSIZ);
            //hpcd->OUT_ep[epnum].xfer_buff += hpcd->OUT_ep[epnum].maxpacket;
          }

          HAL_PCD_DataOutStageCallback(epnum);

          if(CFG_DMA_ENABLE == 1)
          {
#if 0
            if((epnum == 0) && (hpcd->OUT_ep[epnum].xfer_len == 0))
            {
               /* this is ZLP, so prepare EP0 for next setup */
              USB_EP0_OutStart(hpcd->Instance, 1, (uint8_t *)hpcd->Setup);
            }
#endif
          }
        }

        if(( epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP)
        {
          /* Inform the upper layer that a setup packet is available */
          HAL_PCD_SetupStageCallback();
          USBx_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_STUP;
        }

        if(( epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS)
        {
          USBx_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_OTEPDIS;
        }
      }
      epnum++;
      ep_intr >>= 1;
    }
  }

  //while(1);
}

void isr_otg_fs_wkup(void) {
  DEBUG("isr_otg_fs_wkup\n");
  while(1);
}

static inline void _usb_activate_ep(uint8_t addr, uint32_t mps, uint8_t type)
{
  const uint8_t num   = addr & 0x7F;
  const uint8_t is_in = addr & 0x80;

  if (is_in)
  {
   USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & ((1 << (num)));

    if (((USBx_INEP(num)->DIEPCTL) & USB_OTG_DIEPCTL_USBAEP) == 0)
    {
      USBx_INEP(num)->DIEPCTL |= ((mps & USB_OTG_DIEPCTL_MPSIZ ) | (type << 18 ) | ((num) << 22 ) | (USB_OTG_DIEPCTL_SD0PID_SEVNFRM) | (USB_OTG_DIEPCTL_USBAEP));
    }

  }
  else
  {
     USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((1 << (num)) << 16);

    if (((USBx_OUTEP(num)->DOEPCTL) & USB_OTG_DOEPCTL_USBAEP) == 0)
    {
      USBx_OUTEP(num)->DOEPCTL |= ((mps & USB_OTG_DOEPCTL_MPSIZ ) | (type << 18 ) | (USB_OTG_DIEPCTL_SD0PID_SEVNFRM)| (USB_OTG_DOEPCTL_USBAEP));
    }
  }
  //return HAL_OK;
}

int usb_dev_ep_open(usb_dev_ep_t ep, uint32_t mps, usb_dev_ep_type_t type) {
  //  HAL_StatusTypeDef  ret = HAL_OK;
  //  USB_OTG_EPTypeDef *ep;

  //  if ((ep_addr & 0x80) == 0x80)
  //  {
  //    ep = &hpcd->IN_ep[ep_addr & 0x7F];
  //  }
  //  else
  //  {
  //    ep = &hpcd->OUT_ep[ep_addr & 0x7F];
  //  }
  //  ep->num   = ep_addr & 0x7F;

  //  ep->is_in = (0x80 & ep_addr) != 0;
  //  ep->maxpacket = ep_mps;
  //  ep->type = ep_type;
  //  if (ep->is_in)
  //  {
  //    /* Assign a Tx FIFO */
  //    ep->tx_fifo_num = ep->num;
  //  }
  //  /* Set initial data PID. */
  //  if (ep_type == EP_TYPE_BULK )
  //  {
  //    ep->data_pid_start = 0;
  //  }

  //  __HAL_LOCK(hpcd);
  _usb_activate_ep(ep, mps, type);
  //  __HAL_UNLOCK(hpcd);
  //  return ret;
  return USB_DEV_OK;
}

