/* Raw USB vendor class with simple queued streaming support.
 * Provides bulk IN (0x81) and OUT (0x01). A small ring buffer of pending
 * application data blocks is packetised into 64-byte USB transfers.
 *
 * This file is only active when USE_RAW==1. Otherwise it compiles to an
 * empty translation unit; stub inline functions live in the header. */

#if !(defined(USE_RAW) && (USE_RAW==1))
/* Raw mode disabled */
#else
#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_def.h"
#include "usbd_raw.h"
#include "usbd_conf.h"

/* Forward declarations */
static uint8_t USBD_RAW_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_RAW_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_RAW_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_RAW_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_RAW_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *USBD_RAW_GetFSCfgDesc(uint16_t *length);

USBD_ClassTypeDef USBD_RAW = {
  USBD_RAW_Init,
  USBD_RAW_DeInit,
  USBD_RAW_Setup,
  NULL,
  NULL,
  USBD_RAW_DataIn,
  USBD_RAW_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_RAW_GetFSCfgDesc,
};

/* Total length = 9 (config) + 9 (interface) + 7 (EP OUT) + 7 (EP IN) = 32 (0x20) */
__ALIGN_BEGIN static uint8_t USBD_RAW_CfgDesc[] __ALIGN_END = {
  /* Configuration Descriptor */
  0x09, USB_DESC_TYPE_CONFIGURATION, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
  /* Interface Descriptor (bInterfaceClass=0xFF vendor) */
  0x09, USB_DESC_TYPE_INTERFACE, 0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x00,
  /* Endpoint OUT (Bulk, wMaxPacketSize=64) */
  0x07, USB_DESC_TYPE_ENDPOINT, 0x01, 0x02, 0x40, 0x00, 0x00,
  /* Endpoint IN  (Bulk, wMaxPacketSize=64) */
  0x07, USB_DESC_TYPE_ENDPOINT, 0x81, 0x02, 0x40, 0x00, 0x00,
};

static uint8_t raw_out_buf[64];

/* Stream queue: holds pointers to buffers supplied by producer (e.g. ADC DMA half/full) */
typedef struct { const uint8_t *ptr; uint32_t len; uint32_t off; } raw_seg_t;
#define RAW_Q_DEPTH 8
static volatile raw_seg_t raw_q[RAW_Q_DEPTH];
static volatile uint8_t raw_q_head = 0; /* next write */
static volatile uint8_t raw_q_tail = 0; /* next send */
volatile uint32_t usbd_raw_stream_queued_bytes = 0;
volatile uint32_t usbd_raw_stream_drops = 0;

static inline uint8_t raw_q_next(uint8_t i){ return (uint8_t)((i+1U) & (RAW_Q_DEPTH-1U)); }
static int raw_q_full(void){ return raw_q_next(raw_q_head) == raw_q_tail; }
static int raw_q_empty(void){ return raw_q_head == raw_q_tail; }

/* Called by application (interrupt context allowed) to enqueue a contiguous buffer.
 * Buffer must remain valid until fully transmitted (e.g. DMA circular region). */
void usbd_raw_stream_submit(const uint8_t *data, uint32_t length) {
  if(length==0) return;
  uint32_t primask = __get_PRIMASK(); __disable_irq();
  if(raw_q_full()) { usbd_raw_stream_drops++; if(!primask) __enable_irq(); return; }
  raw_q[raw_q_head].ptr = data;
  raw_q[raw_q_head].len = length;
  raw_q[raw_q_head].off = 0;
  raw_q_head = raw_q_next(raw_q_head);
  usbd_raw_stream_queued_bytes += length;
  if(!primask) __enable_irq();
}

/* Internal: attempt to start a transfer if endpoint idle */
static void raw_try_tx(USBD_HandleTypeDef *pdev) {
  if(pdev->dev_state != USBD_STATE_CONFIGURED) return;
  if(raw_q_empty()) return;
  /* Check low-level endpoint busy (Tx FIFO) via core handle state: rely on driver pacing
   * We opportunistically call USBD_LL_Transmit; if stack returns USBD_BUSY we abort. */
  const raw_seg_t *seg = (const raw_seg_t*)&raw_q[raw_q_tail];
  uint32_t remaining = seg->len - seg->off;
  uint16_t pkt = (remaining > 64U)? 64U : (uint16_t)remaining;
  if(pkt==0) return; /* should not happen */
  if(USBD_LL_Transmit(pdev, 0x81, (uint8_t*)(seg->ptr + seg->off), pkt) == USBD_OK) {
    /* Advance offset only when DataIn callback fires (to avoid double send on early re-entry) */
  }
}

/* Public task (call from main loop) to ensure at least one transfer is in flight */
void usbd_raw_stream_task(void) {
  extern USBD_HandleTypeDef hUsbDeviceFS; /* defined in main.c */
  raw_try_tx(&hUsbDeviceFS);
}

/* Configuration state flag visible to main.c */
volatile uint8_t usbd_raw_configured = 0;

static uint8_t USBD_RAW_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
  (void)cfgidx;
  /* Mark configured for main loop */
  usbd_raw_configured = 1;
  USBD_LL_OpenEP(pdev, 0x01, USBD_EP_TYPE_BULK, 0x40);
  USBD_LL_OpenEP(pdev, 0x81, USBD_EP_TYPE_BULK, 0x40);
  USBD_LL_PrepareReceive(pdev, 0x01, raw_out_buf, sizeof(raw_out_buf));
  /* Kick any pending queue */
  raw_try_tx(pdev);
  return USBD_OK;
}

static uint8_t USBD_RAW_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
  (void)cfgidx;
  usbd_raw_configured = 0;
  USBD_LL_CloseEP(pdev, 0x01);
  USBD_LL_CloseEP(pdev, 0x81);
  return USBD_OK;
}

static uint8_t USBD_RAW_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
  (void)pdev; (void)req; return USBD_OK;
}

static uint8_t USBD_RAW_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  if((epnum & 0x7F) == 0x01) { /* EP1 IN complete */
    if(!raw_q_empty()) {
      /* Advance segment */
      raw_seg_t *seg = (raw_seg_t*)&raw_q[raw_q_tail];
      uint32_t remaining = seg->len - seg->off;
      uint32_t step = (remaining > 64U)? 64U : remaining;
      seg->off += step;
      remaining -= step;
      if(remaining == 0) {
        /* Pop */
        raw_q_tail = raw_q_next(raw_q_tail);
      }
    }
    /* Start next if available */
    raw_try_tx(pdev);
  }
  return USBD_OK;
}

static uint8_t USBD_RAW_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  (void)epnum;
  USBD_LL_PrepareReceive(pdev, 0x01, raw_out_buf, sizeof(raw_out_buf));
  return USBD_OK;
}

static uint8_t *USBD_RAW_GetFSCfgDesc(uint16_t *length) {
  *length = sizeof(USBD_RAW_CfgDesc);
  return USBD_RAW_CfgDesc;
}

#endif /* USE_RAW active */
