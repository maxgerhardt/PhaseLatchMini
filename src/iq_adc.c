#include "iq_adc.h"
#include "main.h"
#if defined(ENABLE_IQ) && (ENABLE_IQ==1)
#include "usbd_core.h"
#include "usbd_def.h"
#include "usbd_cdc.h"
#endif

/* Buffer: each 32-bit entry holds I (lower 16) and Q (upper 16) */
static volatile uint32_t iq_buffer[IQ_DMA_LENGTH];
volatile uint32_t iq_dma_half_count = 0;
volatile uint32_t iq_dma_full_count = 0;
volatile uint32_t iq_last_words[4] = {0,0,0,0}; // snapshot for smoke test
static iq_callback_t user_cb = 0;

static ADC_HandleTypeDef hadc1; // master
static ADC_HandleTypeDef hadc2; // slave
static DMA_HandleTypeDef hdma_adc1;
static TIM_HandleTypeDef htim3;

static void adc_gpio_init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1; // PA0, PA1
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void timer_trigger_init(void) {
  __HAL_RCC_TIM3_CLK_ENABLE();
  uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
  // If APB1 prescaler !=1 timer clock doubles (TIMx clocking on APB1 domain quirk)
  if(((RCC->CFGR & RCC_CFGR_PPRE1)>>8) > 0) timer_clk *= 2U;

  uint32_t target = IQ_SAMPLE_RATE_HZ; // desired samples per second

  // Strategy: iterate possible prescalers to find an ARR within 16-bit and
  // frequency closest to target. Limit search to keep it quick.
  uint32_t best_psc = 0, best_arr = 0, best_diff = 0xFFFFFFFF, best_rate = 0;
  for(uint32_t psc = 0; psc < 0xFFFF; ++psc) {
    uint32_t tclk_div = timer_clk / (psc + 1U);
    if(tclk_div < target) break; // further prescalers only lower tclk_div
    uint32_t arr = (tclk_div / target);
    if(arr == 0) continue;
    if(arr > 0) arr -= 1U; // ARR is zero-based
    if(arr > 0xFFFF) continue;
    uint32_t achieved = tclk_div / (arr + 1U);
    uint32_t diff = (achieved > target) ? (achieved - target) : (target - achieved);
    if(diff < best_diff) {
      best_diff = diff; best_psc = psc; best_arr = arr; best_rate = achieved;
      if(diff == 0) break;
    }
  }
  // Fallback: if not found (shouldn't happen) default to 1MHz base logic
  if(best_diff == 0xFFFFFFFF) {
    best_psc = (timer_clk / 1000000U) - 1U;
    best_arr = (1000000U / target) - 1U;
    best_rate = (timer_clk / (best_psc + 1U)) / (best_arr + 1U);
  }

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = best_psc;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = best_arr;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim3);

  // TRGO on update
  TIM_MasterConfigTypeDef mcfg = {0};
  mcfg.MasterOutputTrigger = TIM_TRGO_UPDATE;
  mcfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &mcfg);

  // Optionally store or log achieved rate for diagnostics (user can watch in debugger)
  (void)best_rate; // suppress unused warning if not inspected
}

static void adc_dual_init(void) {
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();

  // ADC clock prescaler: PCLK2 divided by 6 -> <=14MHz
  __HAL_RCC_AFIO_CLK_ENABLE();
  MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6);

  hadc1.Instance = ADC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 0;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.NbrOfConversion = 1;
  // F1 HAL ADC_InitTypeDef does not contain Mode; dual mode is configured via registers directly
  HAL_ADC_Init(&hadc1);

  hadc2.Instance = ADC2;
  hadc2.Init = hadc1.Init;
  HAL_ADC_Init(&hadc2);

  // Configure channels
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_0; // PA0
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  sConfig.Channel = ADC_CHANNEL_1; // PA1
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  // Enable dual regular simultaneous mode: set DUAL[4:0]=00100 in ADC1->CR1
  MODIFY_REG(ADC1->CR1, 0x1F << 16, 0x4 << 16);

  // Calibration sequence
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);
}

static void dma_init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; // dual-mode packs into 32-bit
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_adc1);
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void iq_init(iq_callback_t cb) {
  user_cb = cb;
  adc_gpio_init();
  timer_trigger_init();
  adc_dual_init();
  dma_init();
}

void iq_start(void) {
  // Enable DMA interrupts for half/transfer complete
  __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_HT);
  __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TC);
  // Ensure DMA bit enabled in ADC1 (F1 requires explicit set)
  ADC1->CR2 |= ADC_CR2_DMA;

  // Start slave then master
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc1); // ensure master actually enabled before DMA start

  // Try HAL helper; if it fails to configure CNDTR (remains 0) we fallback
  (void)HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)iq_buffer, IQ_DMA_LENGTH);

  if (DMA1_Channel1->CNDTR == 0) {
    // Manual DMA setup fallback for dual regular simultaneous mode
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)iq_buffer;
    DMA1_Channel1->CNDTR = IQ_DMA_LENGTH; // number of 32-bit words
    // Configure: memory increment, peripheral/mem size 32-bit, circular, high priority
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_CIRC | DMA_CCR_PL_1;
    // Enable half/transfer complete interrupts
    DMA1_Channel1->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
  }

  // Start timer trigger after DMA armed
  HAL_TIM_Base_Start(&htim3);

#if defined(ADC_SMOKE) && (ADC_SMOKE==1)
  // In smoke mode also issue a software start in case external trigger path misconfigured
  // Enable external trigger if not already
  ADC1->CR2 |= ADC_CR2_EXTTRIG;
  // Also try a SWSTART to kick first conversion; dual regular simultaneous: start master is sufficient
  ADC1->CR2 |= ADC_CR2_SWSTART;
#endif
}

void iq_stop(void) {
  HAL_TIM_Base_Stop(&htim3);
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_Stop(&hadc2);
  HAL_DMA_Abort(&hdma_adc1);
}

void DMA1_Channel1_IRQHandler(void) {
  if(__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_HT1)) {
    __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_HT1);
    iq_dma_half_count++;
    iq_last_words[0] = iq_buffer[0];
    iq_last_words[1] = iq_buffer[1];
    // Toggle LED to indicate DMA half complete
    extern void dma_led_toggle(void); dma_led_toggle();
    if(user_cb) user_cb((uint32_t*)iq_buffer, IQ_DMA_LENGTH/2U, 0);
    // DMA kick: schedule first packet immediately if USB idle
#if defined(ENABLE_IQ) && (ENABLE_IQ==1)
  extern volatile uint8_t ep1_busy_flag; extern USBD_HandleTypeDef hUsbDeviceFS;
  extern volatile iq_chunk_t q[8]; extern volatile uint8_t q_tail, q_head;
  extern volatile uint32_t feed_isr_pkts; // reuse ISR counter
    static volatile uint32_t dma_kick_count = 0; // local static for reference (not exposed yet)
    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && ep1_busy_flag==0 && q_tail != q_head) {
      iq_chunk_t chunk = q[q_tail];
      uint32_t send_bytes = chunk.words * 4U; if(send_bytes > 64) send_bytes = 64;
      ep1_busy_flag = 1;
      USBD_LL_Transmit(&hUsbDeviceFS, 0x81, (uint8_t*)chunk.ptr, send_bytes);
      chunk.ptr += send_bytes/4U; chunk.words -= send_bytes/4U;
      if(chunk.words==0) { q_tail = (q_tail + 1) & 7; } else { q[q_tail] = chunk; }
      feed_isr_pkts++; dma_kick_count++;
    }
#endif
  }
  if(__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_TC1)) {
    __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TC1);
    iq_dma_full_count++;
    iq_last_words[2] = iq_buffer[IQ_DMA_LENGTH/2U];
    iq_last_words[3] = iq_buffer[IQ_DMA_LENGTH/2U + 1U];
    // Toggle LED to indicate DMA full complete
    extern void dma_led_toggle(void); dma_led_toggle();
    if(user_cb) user_cb((uint32_t*)&iq_buffer[IQ_DMA_LENGTH/2U], IQ_DMA_LENGTH/2U, 1);
    // DMA kick for second half
#if defined(ENABLE_IQ) && (ENABLE_IQ==1)
  extern volatile uint8_t ep1_busy_flag; extern USBD_HandleTypeDef hUsbDeviceFS;
  extern volatile iq_chunk_t q[8]; extern volatile uint8_t q_tail, q_head;
  extern volatile uint32_t feed_isr_pkts;
    static volatile uint32_t dma_kick_count2 = 0;
    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && ep1_busy_flag==0 && q_tail != q_head) {
      iq_chunk_t chunk = q[q_tail];
      uint32_t send_bytes = chunk.words * 4U; if(send_bytes > 64) send_bytes = 64;
      ep1_busy_flag = 1;
      USBD_LL_Transmit(&hUsbDeviceFS, 0x81, (uint8_t*)chunk.ptr, send_bytes);
      chunk.ptr += send_bytes/4U; chunk.words -= send_bytes/4U;
      if(chunk.words==0) { q_tail = (q_tail + 1) & 7; } else { q[q_tail] = chunk; }
      feed_isr_pkts++; dma_kick_count2++;
    }
#endif
  }
}
