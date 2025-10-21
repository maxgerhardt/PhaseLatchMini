// Minimal IQ streaming firmware: sends only raw ADC-packed words over CDC, no text.
#include "main.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#if ENABLE_IQ
#include "iq_adc.h"
#endif
#include <string.h>

USBD_HandleTypeDef hUsbDeviceFS;
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_DEVICE_Init(void);
static void Quiet_Unused_Pins(void);

// Shared counters / stubs referenced by other modules (always provided)
volatile uint32_t feed_isr_pkts=0, feed_chain_max=0, feed_chain_current=0;
void dma_led_toggle(void){ HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN); }
#if ENABLE_IQ
volatile iq_chunk_t q[8];
volatile uint8_t q_head=0, q_tail=0;
volatile uint32_t adc_drops=0;
volatile uint32_t feed_loop_pkts=0, feed_busy_skips=0;
static void iq_cb(uint32_t *data, uint32_t count, uint8_t index){(void)index;uint8_t next=(q_head+1)&7; if(next==q_tail){adc_drops++;return;} q[q_head].ptr=data; q[q_head].words=count; q_head=next;}
#endif

int main(void){
  HAL_Init(); SystemClock_Config(); Quiet_Unused_Pins(); MX_GPIO_Init(); MX_USB_DEVICE_Init();
#if ENABLE_IQ
  iq_init(iq_cb);
  // Defer ADC start until USB configured (word boundary stability). Timeout fallback to avoid deadlock.
  uint32_t wait_start = HAL_GetTick();
  while(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
    if(HAL_GetTick() - wait_start > 2000) break; // 2s fallback
  }
  iq_start();
#endif
  static uint32_t last_led=0, spin=0; extern volatile uint8_t ep1_busy_flag; while(1){
    uint32_t now=HAL_GetTick();
    if(now==last_led){ if(++spin>500000){ HAL_GPIO_TogglePin(LED_GPIO_PORT,LED_PIN); spin=0; } }
    else if(now-last_led>=300){ HAL_GPIO_TogglePin(LED_GPIO_PORT,LED_PIN); last_led=now; spin=0; }
    if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED){
#if ENABLE_IQ
      while(ep1_busy_flag==0){
        if(q_tail==q_head) break; // no data
        iq_chunk_t c=q[q_tail];
        uint32_t bytes=c.words*4U; if(bytes>64) bytes=64; // 64-byte USB FS packet
        ep1_busy_flag=1;
        USBD_LL_Transmit(&hUsbDeviceFS, CDC_IN_EP,(uint8_t*)c.ptr, bytes);
        c.ptr+=bytes/4U; c.words-=bytes/4U;
        if(c.words==0) q_tail=(q_tail+1)&7; else q[q_tail]=c;
        feed_loop_pkts++;
        break;
      }
      if(ep1_busy_flag && q_tail!=q_head) feed_busy_skips++;
#endif
    }
  }
}

static void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct={0}; RCC_ClkInitTypeDef RCC_ClkInitStruct={0};
  RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState=RCC_HSE_ON; RCC_OscInitStruct.HSEPredivValue=RCC_HSE_PREDIV_DIV1; RCC_OscInitStruct.HSIState=RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON; RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSE; RCC_OscInitStruct.PLL.PLLMUL=RCC_PLL_MUL9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct)!=HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; RCC_ClkInitStruct.AHBCLKDivider=RCC_SYSCLK_DIV1; RCC_ClkInitStruct.APB1CLKDivider=RCC_HCLK_DIV2; RCC_ClkInitStruct.APB2CLKDivider=RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!=HAL_OK) Error_Handler(); __HAL_RCC_USB_CLK_ENABLE();
}

static void MX_GPIO_Init(void){
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct={0};
  GPIO_InitStruct.Pin=LED_PIN; GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(LED_GPIO_PORT,&GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
}

static void Quiet_Unused_Pins(void){ /* Optionally configure unused pins as analog (implementation omitted for brevity) */ }

static void MX_USB_DEVICE_Init(void){
  /* Full-speed device init (speed param = 0). */
  USBD_Init(&hUsbDeviceFS, &FS_Desc, 0); USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC); USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS); USBD_Start(&hUsbDeviceFS);
}

void Error_Handler(void){ __disable_irq(); while(1){ HAL_GPIO_TogglePin(LED_GPIO_PORT,LED_PIN); HAL_Delay(100);} }

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){ (void)file; (void)line; }
#endif
