// DMA debug LED toggle for ADC activity
#include "main.h"
void dma_led_toggle(void) {
  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
}
// Cleaned main.c simplifying debug/ASTATS logic and removing duplicated / broken blocks
#include "main.h"
// Minimal build focuses solely on verifying bulk IN endpoint reliability.
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#if defined(USE_RAW) && (USE_RAW==1)
#include "usbd_raw.h"
#endif
#include "usbd_def.h"
#include <string.h>
#include <stdio.h>

// Diagnostics toggle: set to 1 to enable EARLYTICK / TICK reporting
#ifndef USB_TICK_DIAG
#define USB_TICK_DIAG 0
#endif

#if USB_TICK_DIAG
static uint32_t latched_tick0 = 0, latched_tick1 = 0; // captured post-config
static uint8_t tick_captured = 0;
static uint8_t early_report_done = 0;
#endif

USBD_HandleTypeDef hUsbDeviceFS;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_DEVICE_Init(void);

#if ENABLE_IQ
#include "iq_adc.h"
#endif

volatile iq_chunk_t q[8];
volatile uint8_t q_head=0, q_tail=0; // ring indices (mask with &7)
volatile uint32_t adc_drops=0;        // overflow drops (exported)
// USB feed instrumentation (packets scheduled from main loop vs ISR burst, skips)
#if ENABLE_IQ
volatile uint32_t feed_loop_pkts=0;
volatile uint32_t feed_isr_pkts=0;
volatile uint32_t feed_busy_skips=0;
volatile uint32_t feed_chain_max=0; // longest continuous ISR scheduling chain
volatile uint32_t feed_chain_current=0;
#endif

#if ENABLE_IQ
static void iq_cb(uint32_t *data, uint32_t count, uint8_t index) {
  (void)index;
  uint8_t next = (q_head + 1) & 7;
  if(next == q_tail) { adc_drops++; return; }
  q[q_head].ptr = data;
  q[q_head].words = count; // 32-bit words
  q_head = next;
#if defined(USE_RAW) && (USE_RAW==1)
  extern void usbd_raw_stream_submit(const uint8_t *data, uint32_t length);
  usbd_raw_stream_submit((const uint8_t*)data, count*4U);
#endif
}
#endif

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  // Set LED low after GPIO init
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
  // Optional early SysTick probe
  // Diagnostics capturing deferred until after USB configured to avoid interfering with enumeration
  MX_USB_DEVICE_Init();
  #if ENABLE_IQ
    iq_init(iq_cb);
    iq_start();
  #endif
  static uint8_t ramp_buf[64];
  #if !ENABLE_IQ
    for(int i=0;i<64;i++) ramp_buf[i]=(uint8_t)i;
  #endif
  uint32_t ramp_seq=0;
  static volatile uint32_t main_loop_heart = 0;
  while(1) {
    main_loop_heart++;
    uint32_t now = HAL_GetTick();
    // Always provide a slow LED blink independent of diagnostics (300ms)
    static uint32_t last_led = 0;
    // Fallback: if SysTick not advancing (now unchanged), use a spin counter to still show life
    static uint32_t spin_cnt = 0;
    if(now == last_led) {
      if(++spin_cnt > 500000) { // crude ~delay based on loop iterations
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        spin_cnt = 0;
      }
    } else if(now - last_led >= 300) {
      HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
      last_led = now;
      spin_cnt = 0;
    }
    extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
    extern USBD_HandleTypeDef hUsbDeviceFS;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    #if !ENABLE_IQ
      if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hcdc && hcdc->TxState==0) {
        ramp_buf[0]=(uint8_t)ramp_seq;
        ramp_buf[1]=(uint8_t)(ramp_seq>>8);
        ramp_buf[2]=(uint8_t)(ramp_seq>>16);
        ramp_buf[3]=(uint8_t)(ramp_seq>>24);
        if(CDC_Transmit_FS(ramp_buf, sizeof(ramp_buf))==USBD_OK) {
          ramp_seq++;
        }
      }
    #endif
  // Forced low-level transmit spam removed (can be reintroduced if needed)
    // Invoke CDC diagnostic high-rate filler to attempt transfers and show stats
    #if !defined(THROUGHPUT_BASELINE) || (THROUGHPUT_BASELINE==0)
      CDC_Minimal_Task();
      // Background poke disabled in throughput baseline to avoid 8B packet spam
      CDC_Background_Poke();
    #endif
    // Heartbeat & one-time early tick report after configuration
    // Single EARLYTICK report shortly after configuration when diagnostics enabled
#if USB_TICK_DIAG
    if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED) {
      static uint32_t config_time0 = 0;
      if(config_time0==0) config_time0 = now; // first time we see configured
      // After 500ms in configured state, capture ticks and send single report
      if(!tick_captured && (now - config_time0) > 500U) {
        latched_tick0 = HAL_GetTick();
        for(volatile uint32_t spin=0; spin<10000; ++spin) { __NOP(); }
        latched_tick1 = HAL_GetTick();
        tick_captured = 1;
      }
      if(tick_captured && !early_report_done) {
        early_report_done = 1;
        // Capture key SysTick / core registers for diagnostics
        uint32_t ctrl = SysTick->CTRL;
        uint32_t load = SysTick->LOAD;
        uint32_t val  = SysTick->VAL;
        uint32_t icsr = SCB->ICSR;
        uint32_t prim = __get_PRIMASK();
        char et[160];
        int en = snprintf(et,sizeof(et),
          "TICKCHK %lu %lu DT=%lu CTRL=%08lX LOAD=%08lX VAL=%08lX ICSR=%08lX PRIM=%lu MLH=%lu\r\n",
          (unsigned long)latched_tick0,(unsigned long)latched_tick1,
          (unsigned long)(latched_tick1 - latched_tick0),
          (unsigned long)ctrl,(unsigned long)load,(unsigned long)val,(unsigned long)icsr,(unsigned long)prim,(unsigned long)main_loop_heart);
        CDC_Transmit_FS((uint8_t*)et,(uint16_t)en);
      }
    }
#endif
    // Autonomous low-level EP1 feeder: send 32B frame of ADC samples whenever EP1 not busy
  extern volatile uint8_t ep1_busy_flag;
    if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED) {
      // Drain as many queued IQ chunks as USB will accept (one in-flight at a time)
      while(ep1_busy_flag==0 && q_tail != q_head) {
        iq_chunk_t chunk = q[q_tail];
        uint32_t send_bytes = chunk.words * 4U;
        if(send_bytes > 64) send_bytes = 64; // full FS packet size
        ep1_busy_flag = 1;
        USBD_LL_Transmit(&hUsbDeviceFS, CDC_IN_EP, (uint8_t*)chunk.ptr, send_bytes);
        // Adjust chunk
        chunk.ptr += send_bytes / 4U;
        chunk.words -= send_bytes / 4U;
        if(chunk.words == 0) { q_tail = (q_tail + 1) & 7; }
        else { q[q_tail] = chunk; }
        #if ENABLE_IQ
          feed_loop_pkts++;
        #endif
        // Break after scheduling one packet (must wait for IRQ to clear ep1_busy_flag)
        break;
      }
      #if ENABLE_IQ
        if(ep1_busy_flag) {
          // counted as busy skip if queue has more but we're limited by in-flight packet
          if(q_tail != q_head) feed_busy_skips++; 
        }
      #endif
    }
  #if defined(ADC_SMOKE) && (ADC_SMOKE==1)
    // Periodic ADC smoke status (every ~500ms) textual only
    if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED) {
      static uint32_t last_adcstat=0; if(now - last_adcstat > 500) {
        last_adcstat = now;
        extern volatile uint32_t iq_dma_half_count, iq_dma_full_count, adc_drops; // adc_drops already declared
        extern volatile uint32_t iq_last_words[4];
          uint32_t h = iq_dma_half_count; uint32_t f = iq_dma_full_count;
          // If no progress yet, include key register diagnostics to help debug
          if((h|f)==0) {
            uint32_t cr1 = ADC1->CR1; uint32_t cr2 = ADC1->CR2; uint32_t sr = ADC1->SR; uint32_t dma_cndtr = DMA1_Channel1->CNDTR; uint32_t dma_ccr = DMA1_Channel1->CCR; uint32_t tim_cr1 = TIM3->CR1; uint32_t tim_cnt = TIM3->CNT; uint32_t tim_arr = TIM3->ARR;
            char line[192];
            int n = snprintf(line,sizeof(line),"ADCSTAT H=%lu F=%lu DROP=%lu W0=%08lX W1=%08lX W2=%08lX W3=%08lX SR=%08lX CR1=%08lX CR2=%08lX CNDTR=%lu CCR=%08lX TCR1=%04lX TCNT=%lu TARR=%lu\r\n",
              (unsigned long)h,(unsigned long)f,(unsigned long)adc_drops,
              (unsigned long)iq_last_words[0],(unsigned long)iq_last_words[1],(unsigned long)iq_last_words[2],(unsigned long)iq_last_words[3],
              (unsigned long)sr,(unsigned long)cr1,(unsigned long)cr2,(unsigned long)dma_cndtr,(unsigned long)dma_ccr,(unsigned long)tim_cr1,(unsigned long)tim_cnt,(unsigned long)tim_arr);
            CDC_Transmit_FS((uint8_t*)line,(uint16_t)n);
          } else {
        char line[128];
          int n = snprintf(line,sizeof(line),"ADCSTAT H=%lu F=%lu DROP=%lu W0=%08lX W1=%08lX W2=%08lX W3=%08lX\r\n",
            (unsigned long)h,(unsigned long)f,(unsigned long)adc_drops,
            (unsigned long)iq_last_words[0],(unsigned long)iq_last_words[1],(unsigned long)iq_last_words[2],(unsigned long)iq_last_words[3]);
          CDC_Transmit_FS((uint8_t*)line,(uint16_t)n);
          }
      }
    }
  #endif
  // Occasionally emit a MAINSTAT to show main loop progressing
  // static uint32_t last_mainstat=0; if((now - last_mainstat) > 700) { last_mainstat = now; char ms[64]; int n=snprintf(ms,sizeof(ms),"MAINSTAT MLH=%lu\r\n",(unsigned long)main_loop_heart); CDC_Transmit_FS((uint8_t*)ms,(uint16_t)n); }
    // Core state beacon (raw transmit bypass) every 500ms
    // Core beacon disabled for throughput test
    // Raw IN spammer: every ~10ms try an 8B packet direct, ignoring CDC state
    // Raw ramp spammer disabled: replaced by queued IQ streaming
  /* Raw streaming task not used in CDC fallback */
    // (CDC manual init fallback removed in raw mode)
  }
}

static void MX_USB_DEVICE_Init(void) {
  USBD_Init(&hUsbDeviceFS, &FS_Desc, USBD_SPEED_FULL);
#if defined(USE_RAW) && (USE_RAW==1)
  extern USBD_ClassTypeDef USBD_RAW;
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_RAW);
#else
  extern USBD_ClassTypeDef USBD_CDC;
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
  USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
#endif
  USBD_Start(&hUsbDeviceFS);
  // Safety re-open endpoints if class init failed to do so
  extern USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef*, uint8_t, uint8_t, uint16_t);
  USBD_LL_OpenEP(&hUsbDeviceFS, 0x81, USBD_EP_TYPE_BULK, 64);
  USBD_LL_OpenEP(&hUsbDeviceFS, 0x01, USBD_EP_TYPE_BULK, 64);
  USBD_LL_OpenEP(&hUsbDeviceFS, 0x82, USBD_EP_TYPE_INTR, 8);
}

static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
}

static void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 8MHz * 9 = 72MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // 36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

  __HAL_RCC_USB_CLK_ENABLE();
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    HAL_Delay(100);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  (void)file; (void)line;
}
#endif
