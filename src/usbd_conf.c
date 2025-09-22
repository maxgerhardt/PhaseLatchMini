#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_def.h"
#include "usbd_cdc.h"
#include "stm32f1xx_hal.h"

PCD_HandleTypeDef hpcd_USB_FS;

void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle) {
  if(pcdHandle->Instance==USB) {
    __HAL_RCC_USB_CLK_ENABLE();
    // USB uses PA11 (DM) PA12 (DP) - default alt config after reset is fine
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* pcdHandle) {
  if(pcdHandle->Instance==USB) {
    __HAL_RCC_USB_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  }
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

volatile uint32_t ll_datain_count = 0; // counts HAL_PCD_DataInStageCallback for any IN EP
volatile uint32_t ep1_in_irqs = 0;     // counts EP1 IN completions specifically
volatile uint8_t ep1_busy_flag = 0;    // cleared on EP1 IN complete
#include "main.h" // for LED pin (if defined)

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev) {
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = 0x40; // restore EP0 MPS 64 bytes (standard for FS)
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.pData = pdev;
  pdev->pData = &hpcd_USB_FS;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
    return USBD_FAIL;
  }
  // BTABLE at 0x00, start endpoint buffers at 0x40 on 32-byte boundaries
  // EP0 OUT (0x00) RX, EP0 IN (0x80) TX, EP1 OUT (0x01), EP1 IN (0x81), EP2 IN (0x82)
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00, PCD_SNG_BUF, 0x40);   // EP0 OUT
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80, PCD_SNG_BUF, 0x80);   // EP0 IN
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x01, PCD_SNG_BUF, 0xC0);   // CDC OUT
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x81, PCD_SNG_BUF, 0x100);  // CDC IN (default)
  HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x82, PCD_SNG_BUF, 0x140);  // CDC CMD (INT IN)
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev) {
  HAL_PCD_DeInit(pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) {
  HAL_PCD_Start(pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev) {
  HAL_PCD_Stop(pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
  HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  PCD_HandleTypeDef *hpcd = pdev->pData;
  if ((ep_addr & 0x80) == 0x80)
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  else
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr) {
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK;
}


uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
}
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
  if(epnum == 1) {
    ll_datain_count++; ep1_in_irqs++; ep1_busy_flag = 0;
    // Force-clear CDC TxState and increment tx_completes if class data present
    USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*)hpcd->pData;
    if(pdev && pdev->pClassData) {
      USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)pdev->pClassData;
      if(hcdc->TxState) hcdc->TxState = 0; // unblock pipeline
      extern void cdc_force_tx_complete_hook(void);
      cdc_force_tx_complete_hook();
      #if !defined(THROUGHPUT_BASELINE) || (THROUGHPUT_BASELINE==0)
        #if !defined(ADC_SMOKE) || (ADC_SMOKE==0)
          // Auto-schedule a tiny follow-up packet to keep host polling (disabled in baseline/smoke)
          static uint8_t auto_ping[8] = { 'A','U','T','O',0,0,'\r','\n'};
          auto_ping[4] = (uint8_t)(ll_datain_count & 0xFF);
          auto_ping[5] = (uint8_t)(ep1_in_irqs & 0xFF);
          if(hcdc->TxState==0) {
            USBD_LL_Transmit(pdev, 0x81, auto_ping, sizeof(auto_ping));
            hcdc->TxState = 1; // mark busy until callback
          }
        #endif
      #endif
      // Burst feeder for IQ streaming: immediately schedule next IQ packet if available
      #if defined(ENABLE_IQ) && (ENABLE_IQ==1)
        extern volatile uint8_t ep1_busy_flag; // already cleared
        extern volatile iq_chunk_t q[8];
        extern volatile uint8_t q_head; extern volatile uint8_t q_tail;
        extern USBD_HandleTypeDef hUsbDeviceFS;
        extern volatile uint32_t feed_isr_pkts; extern volatile uint32_t feed_chain_max; 
        extern volatile uint32_t feed_chain_current; 
        // Limit chain depth per ISR to avoid starving main loop (e.g., up to 4 packets)
  uint32_t chain = 0; const uint32_t chain_limit = 12; // increased from 4 to 12 for higher burst depth
        while(ep1_busy_flag==0 && q_tail != q_head && chain < chain_limit) {
          iq_chunk_t chunk = q[q_tail];
          uint32_t send_bytes = chunk.words * 4U; if(send_bytes > 64) send_bytes = 64;
          ep1_busy_flag = 1;
          USBD_LL_Transmit(&hUsbDeviceFS, 0x81, (uint8_t*)chunk.ptr, send_bytes);
          chunk.ptr += send_bytes / 4U; chunk.words -= send_bytes / 4U;
          if(chunk.words == 0) { q_tail = (q_tail + 1) & 7; } else { q[q_tail] = chunk; }
          feed_isr_pkts++; chain++;
        }
        feed_chain_current = chain;
        if(chain > feed_chain_max) feed_chain_max = chain;
      #endif
    }
  }
  USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) {
  USBD_LL_SOF(hpcd->pData);
}
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL; USBD_LL_Reset(hpcd->pData); USBD_LL_SetSpeed(hpcd->pData, speed);
}
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_Suspend(hpcd->pData); }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_Resume(hpcd->pData); }
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_DevConnected(hpcd->pData); }
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_DevDisconnected(hpcd->pData); }

// Static alloc not required; using default malloc/free macros from usbd_conf.h

// Low-level transmit and receive wrappers required by USB Device Core
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
  HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}
