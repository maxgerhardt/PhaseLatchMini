// Minimal CDC test driver only
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "usbd_def.h"
#include "usbd_cdc.h"
#include "main.h"
#include "usbd_conf.h"
#include "stm32f1xx.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

static uint8_t UserRxBufferFS[64];
static uint8_t UserTxBufferFS[64];

// Diagnostics counters
static volatile uint32_t tx_attempts=0, tx_busy_returns=0, tx_completes=0, tx_watchdog_clears=0;
static volatile uint32_t tiny_packets_sent=0;
static volatile uint32_t auto_attempts=0;      // background scheduled attempts
static volatile uint32_t force_attempts=0;     // low-level USBD_LL_Transmit attempts
static volatile uint8_t stream_on = 1; // force streaming by default for debug
static volatile uint8_t dtr_set = 1;   // treat as always set (remove gating for debug)
// Fallback class data block (used if pClassData never populated by stack)
static USBD_CDC_HandleTypeDef cdc_fallback;
static uint8_t cdc_fallback_active = 0;
static volatile uint32_t txstate_observed = 0;
static volatile uint32_t direct_attempts = 0;
static volatile uint32_t frame_attempts = 0;
static volatile uint32_t frame_ok = 0;
static volatile uint32_t frame_busy = 0;
static volatile uint32_t loop_heart = 0; // increments each CDC_Minimal_Task invocation
extern volatile uint32_t ll_datain_count; // from usbd_conf.c
static uint32_t last_stat_ms=0;
// Forward decl for manual DataIn hook
void USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

// Raw low-level transmit probe (bypasses CDC state machine) for debugging
static void raw_in_ep_probe(void) {
  // Disabled: never inject probe bytes into the IN stream. Keep CDC strictly binary.
  (void)0;
}

static int8_t CDC_Init_FS(void) {
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  // Initial ASCII banner intentionally removed to avoid any non-binary bytes
  // being emitted on the CDC IN endpoint. Keep the stream strictly binary.
  // If core didn't allocate class data yet, point it to our static block
  if(hUsbDeviceFS.pClassData == NULL) {
    memset(&cdc_fallback,0,sizeof(cdc_fallback));
    hUsbDeviceFS.pClassData = &cdc_fallback;
    cdc_fallback_active = 1;
  }
  return (USBD_OK);
}
static int8_t CDC_DeInit_FS(void) { return (USBD_OK); }
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length) {
  (void)pbuf; (void)length; (void)cmd; return (USBD_OK);
} 

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) {
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  tx_attempts++;
  if(!hcdc) { tx_busy_returns++; return USBD_BUSY; }
  if(hcdc->TxState) { tx_busy_returns++; return USBD_BUSY; }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
  // Pure streaming mode: ignore all incoming data and emit NO textual responses.
  // This guarantees the bulk IN stream remains strictly packed IQ words without ASCII headers.
  (void)Buf; (void)Len; USBD_CDC_ReceivePacket(&hUsbDeviceFS); return (USBD_OK);
}

// Attempt to ensure we catch completion regardless of actual weak symbol naming.
// Some ST stacks use USBD_CDC_DataIn, others route via class callback table.
void USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  // Call default core handler if present (not accessible here unless weak linked)
  if(pdev->pClassData) {
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)pdev->pClassData;
    hcdc->TxState = 0U;
  }
  tx_completes++;
  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
}

// Helper to let main query tx_completes without tight coupling
uint32_t CDC_Debug_TxCompletes(void){ return tx_completes; }
uint32_t CDC_Debug_TxAttempts(void){ return tx_attempts; }

// Polling task: aggressive fill with 64B frames; every 500ms emit STAT line
void CDC_Minimal_Task(void) {
  // Fully disabled in pure IQ streaming build to avoid ANY non-sample bytes.
  return;
}

// Unconditional background poke: increments auto_attempts every call, tries a direct CDC_Transmit_FS
void CDC_Background_Poke(void) { /* disabled */ }

// Stubs kept for link compatibility
uint8_t CDC_Stream_Active(void){return 0;}
uint32_t CDC_Stream_Drops(void){return 0;}
void CDC_Debug_ForceStart(void){}
void CDC_Stream_Poll(void){}
void CDC_Debug_SendSerialState(void){}



// Hook invoked from low-level DataIn callback (usbd_conf.c) to ensure tx_completes advances
// even if the normal USBD_CDC_DataIn path is bypassed or not wired.
void cdc_force_tx_complete_hook(void) { tx_completes++; }


