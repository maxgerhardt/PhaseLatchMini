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
#if ENABLE_IQ
  // Disabled in ADC streaming mode to avoid inserting ASCII into binary stream
  return;
#endif
  extern USBD_HandleTypeDef hUsbDeviceFS;
  if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*)hUsbDeviceFS.pData;
  if(!hpcd) return;
  // Only fire if CDC reports idle to avoid overwriting active transfer
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if(hcdc && hcdc->TxState) return;
  static uint8_t rawbuf[16];
  static uint32_t rseq=0;
  for(int i=0;i<16;i++) rawbuf[i] = (uint8_t)(0xA0 + ((rseq+i)&0x1F));
  rseq++;
  // Use core API (still goes through HAL_PCD_EP_Transmit) but with small size patterns
  USBD_LL_Transmit(&hUsbDeviceFS, CDC_IN_EP, rawbuf, 16);
}

static int8_t CDC_Init_FS(void) {
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  // For pure ADC streaming builds (ENABLE_IQ) suppress initial ASCII banner to keep
  // the bulk stream strictly binary (helps host alignment logic). Retain banner
  // for non-ADC diagnostic / ramp environments so a terminal shows life immediately.
#if !ENABLE_IQ
  static const uint8_t init_msg[] = "BOOT\r\n";
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)init_msg, sizeof(init_msg)-1);
  USBD_CDC_TransmitPacket(&hUsbDeviceFS);
#endif
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
  if(*Len) {
    // search buffer for commands anywhere
    for(uint32_t i=0;i+4<*Len;i++) {
      if((Buf[i]=='S'||Buf[i]=='s') && (Buf[i+1]=='T'||Buf[i+1]=='t') && (Buf[i+2]=='A'||Buf[i+2]=='a') && (Buf[i+3]=='R'||Buf[i+3]=='r') && (Buf[i+4]=='T'||Buf[i+4]=='t')) {
        stream_on = 1; 
        char ack[160];
        USBD_CDC_HandleTypeDef *hcdc2 = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData; uint8_t txs2 = hcdc2? hcdc2->TxState:0; uint16_t ep1r = USB->EP1R;
        int n = snprintf(ack, sizeof(ack), "OK START TA=%lu TB=%lu TC=%lu WD=%lu FA=%lu FO=%lu FB=%lu TXS=%u EP1R=%04X LH=%lu\r\n",
          (unsigned long)tx_attempts,(unsigned long)tx_busy_returns,(unsigned long)tx_completes,(unsigned long)tx_watchdog_clears,(unsigned long)frame_attempts,(unsigned long)frame_ok,(unsigned long)frame_busy,(unsigned)txs2,ep1r,(unsigned long)loop_heart);
        CDC_Transmit_FS((uint8_t*)ack, (uint16_t)n); break;
      }
    }
    for(uint32_t i=0;i+3<*Len;i++) {
      if((Buf[i]=='S'||Buf[i]=='s') && (Buf[i+1]=='T'||Buf[i+1]=='t') && (Buf[i+2]=='O'||Buf[i+2]=='o') && (Buf[i+3]=='P'||Buf[i+3]=='p')) {
        stream_on = 0; const char *ack="OK STOP\r\n"; CDC_Transmit_FS((uint8_t*)ack, (uint16_t)strlen(ack)); break;
      }
    }
    for(uint32_t i=0;i+4<*Len;i++) {
      if((Buf[i]=='S'||Buf[i]=='s') && (Buf[i+1]=='T'||Buf[i+1]=='t') && (Buf[i+2]=='A'||Buf[i+2]=='a') && (Buf[i+3]=='T'||Buf[i+3]=='t') && (Buf[i+4]=='S'||Buf[i+4]=='s')) {
          extern USBD_HandleTypeDef hUsbDeviceFS;
          uint8_t ds = hUsbDeviceFS.dev_state;
          void *pcd = hUsbDeviceFS.pClassData;
          uint16_t ep1r = USB->EP1R;
          extern volatile uint32_t ep1_in_irqs; extern volatile uint8_t ep1_busy_flag;
          char line[280]; int n = snprintf(line, sizeof(line), "STAT TA=%lu TB=%lu TC=%lu WD=%lu P=%lu TS=%lu DIR=%lu TXS=%u STR=%u FA=%lu FO=%lu FB=%lu AA=%lu RA=%lu LH=%lu E1=%lu EB=%u FBF=%u DS=%u EP1R=%04X PCD=%p\r\n",
            (unsigned long)tx_attempts,(unsigned long)tx_busy_returns,(unsigned long)tx_completes,(unsigned long)tx_watchdog_clears,(unsigned long)tiny_packets_sent,(unsigned long)txstate_observed,(unsigned long)direct_attempts,(unsigned long)((USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData?((USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData)->TxState:0),stream_on, (unsigned long)frame_attempts, (unsigned long)frame_ok, (unsigned long)frame_busy, (unsigned long)auto_attempts, (unsigned long)force_attempts,(unsigned long)loop_heart,(unsigned long)ep1_in_irqs, (unsigned)ep1_busy_flag, (unsigned)cdc_fallback_active, (unsigned)ds, ep1r, pcd);
        CDC_Transmit_FS((uint8_t*)line, (uint16_t)n); break;
      }
    }
    // 'F' feed stats (loop vs ISR packets)
#if ENABLE_IQ
    for(uint32_t i=0;i<*Len;i++) {
      if(Buf[i]=='F' || Buf[i]=='f') {
        extern volatile uint32_t feed_loop_pkts, feed_isr_pkts, feed_busy_skips, feed_chain_max;
        char fline[128]; int fn = snprintf(fline,sizeof(fline),"FEED L=%lu I=%lu SK=%lu CHMAX=%lu\r\n",
          (unsigned long)feed_loop_pkts,(unsigned long)feed_isr_pkts,(unsigned long)feed_busy_skips,(unsigned long)feed_chain_max);
        CDC_Transmit_FS((uint8_t*)fline,(uint16_t)fn); break;
      }
    }
#endif
    // Single-letter 'A' command anywhere in buffer -> ADC status snapshot (when ENABLE_IQ build flag used)
#if ENABLE_IQ
    for(uint32_t i=0;i<*Len;i++) {
      if(Buf[i]=='A' || Buf[i]=='a') {
        extern volatile uint32_t iq_dma_half_count, iq_dma_full_count, adc_drops;
        extern volatile uint32_t iq_last_words[4];
        char aline[160];
        int an = snprintf(aline,sizeof(aline),"ADCSTAT H=%lu F=%lu DROP=%lu W0=%08lX W1=%08lX W2=%08lX W3=%08lX\r\n",
          (unsigned long)iq_dma_half_count,(unsigned long)iq_dma_full_count,(unsigned long)adc_drops,
          (unsigned long)iq_last_words[0],(unsigned long)iq_last_words[1],(unsigned long)iq_last_words[2],(unsigned long)iq_last_words[3]);
        CDC_Transmit_FS((uint8_t*)aline,(uint16_t)an);
        break;
      }
    }
#endif
  }
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
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
#if (defined(THROUGHPUT_BASELINE) && (THROUGHPUT_BASELINE==1)) || ENABLE_IQ
  // In baseline or ADC streaming mode, suppress debug micro-packets entirely.
  return;
#endif
  static uint32_t last_attempt=0; static uint8_t banner_sent=0; static uint32_t last_force_raw=0; 
  uint32_t now = HAL_GetTick();
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if(!hcdc) return;
  if(!banner_sent && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) { const char *b = "MINCFG AUTOSTREAM\r\n"; banner_sent=1; CDC_Transmit_FS((uint8_t*)b, (uint16_t)strlen(b));
    char init[96]; USBD_CDC_HandleTypeDef *hcdc2=(USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData; uint8_t txs2 = hcdc2? hcdc2->TxState:0; uint16_t ep1r = USB->EP1R; 
    int n2=snprintf(init,sizeof(init),"STAT EARLY TA=%lu TB=%lu TC=%lu WD=%lu P=%lu EP1R=%04X TXS=%u\r\n",(unsigned long)tx_attempts,(unsigned long)tx_busy_returns,(unsigned long)tx_completes,(unsigned long)tx_watchdog_clears,(unsigned long)tiny_packets_sent,ep1r,txs2);
    CDC_Transmit_FS((uint8_t*)init,(uint16_t)n2);
  }
  // Watchdog: clear stuck TxState (>5ms) to keep pipeline probing
  if(hcdc->TxState && (now - last_attempt) > 5) { hcdc->TxState = 0; tx_watchdog_clears++; }
  // Periodic stats every 500ms
  if(now - last_stat_ms > 500) {
    last_stat_ms = now;
    char line[96];
    uint16_t ep1r = USB->EP1R; uint8_t txs = 0; if(hcdc) txs = hcdc->TxState; txstate_observed += txs; 
  int n = snprintf(line, sizeof(line), "STAT TA=%lu TB=%lu TC=%lu WD=%lu P=%lu DI=%lu EP1R=%04X TXS=%u FA=%lu FO=%lu FB=%lu LH=%lu\r\n",
                     (unsigned long)tx_attempts, (unsigned long)tx_busy_returns,
                     (unsigned long)tx_completes, (unsigned long)tx_watchdog_clears,
                     (unsigned long)tiny_packets_sent, (unsigned long)ll_datain_count,
           ep1r, txs, (unsigned long)frame_attempts, (unsigned long)frame_ok, (unsigned long)frame_busy, (unsigned long)loop_heart);
    CDC_Transmit_FS((uint8_t*)line, (uint16_t)n);
  // Also push a raw probe packet every stats interval
  raw_in_ep_probe();
  }
  // Rapid transmit loop: fill when free with 64B payload
  if(1) { // force attempts irrespective of stream_on
    loop_heart++;
    // Always record an attempt tick to verify loop runs
    frame_attempts++;
    static uint8_t frame[8]; static uint32_t seq=0;
    // If TxState stuck > 5ms clear (already in watchdog earlier), then attempt
    if(!hcdc->TxState) {
      last_attempt = now;
      frame[0]= (uint8_t)(seq);
      frame[1]= (uint8_t)(seq>>8);
      frame[2]= 0xA5; frame[3]=0x5A;
      frame[4]= 0xF0; frame[5]= 0x0F; frame[6]= (uint8_t)(seq^0xAA); frame[7]= (uint8_t)(seq^0x55);
      uint8_t r = CDC_Transmit_FS(frame, 8);
      if(r == USBD_OK) { seq++; tiny_packets_sent++; frame_ok++; }
      else if(r == USBD_BUSY) { frame_busy++; }
    }
  }
#if !ENABLE_IQ
  // Raw kick & beacon only in non-ADC modes
  if(now - last_force_raw > 400) { last_force_raw = now; static uint8_t rawk[8] = {'R','A','W','K',0,0,'\r','\n'}; rawk[4]=(uint8_t)(auto_attempts & 0xFF); USBD_LL_Transmit(&hUsbDeviceFS, CDC_IN_EP, rawk, sizeof(rawk)); }
  static uint32_t last_beacon=0; if(now - last_beacon > 300) {
    last_beacon = now; static uint8_t beacon[12] = { 'B','E','A','C','O','N','\r','\n',0,0,0,0 }; beacon[8]=(uint8_t)(loop_heart & 0xFF); beacon[9]=(uint8_t)((loop_heart>>8)&0xFF); beacon[10]=(uint8_t)(tx_attempts & 0xFF); beacon[11]=(uint8_t)(tx_completes & 0xFF); USBD_LL_Transmit(&hUsbDeviceFS, CDC_IN_EP, beacon, sizeof(beacon)); }
#endif
}

// Unconditional background poke: increments auto_attempts every call, tries a direct CDC_Transmit_FS
void CDC_Background_Poke(void) {
#if (defined(THROUGHPUT_BASELINE) && (THROUGHPUT_BASELINE==1)) || ENABLE_IQ
  return;
#endif
  extern USBD_HandleTypeDef hUsbDeviceFS;
  if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  auto_attempts++;
  static uint8_t poke[8] = { 'P','O','K','E',0,0,0,'\n'};
  poke[4] = (uint8_t)(auto_attempts & 0xFF);
  poke[5] = (uint8_t)(tx_attempts & 0xFF);
  if(hcdc && hcdc->TxState==0) {
    CDC_Transmit_FS(poke, sizeof(poke));
  }
  // Also try a raw low-level transmit every 4th poke to see if packets ever schedule
  if((auto_attempts & 3U)==0) {
    force_attempts++;
    static uint8_t rawp[8] = { 'R','A','W','T',0,0,0,'\r'};
    rawp[4] = (uint8_t)(force_attempts & 0xFF);
    rawp[5] = (uint8_t)(auto_attempts & 0xFF);
    USBD_LL_Transmit(&hUsbDeviceFS, CDC_IN_EP, rawp, sizeof(rawp));
  }
}

// Stubs kept for link compatibility
uint8_t CDC_Stream_Active(void){return 0;}
uint32_t CDC_Stream_Drops(void){return 0;}
void CDC_Debug_ForceStart(void){}
void CDC_Stream_Poll(void){}
void CDC_Debug_SendSerialState(void){}



// Hook invoked from low-level DataIn callback (usbd_conf.c) to ensure tx_completes advances
// even if the normal USBD_CDC_DataIn path is bypassed or not wired.
void cdc_force_tx_complete_hook(void) { tx_completes++; }


