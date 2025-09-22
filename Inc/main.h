#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define LED_GPIO_PORT GPIOB
#define LED_PIN GPIO_PIN_12

void Error_Handler(void);

// Exported ADC ring overflow counter
extern volatile uint32_t adc_drops;

// IQ chunk descriptor (32-bit words pointer + remaining word count)
typedef struct { uint32_t *ptr; uint32_t words; } iq_chunk_t;

#if defined(ENABLE_IQ) && (ENABLE_IQ==1)
// Expose ring buffer and indices for ISR burst feeder
extern volatile iq_chunk_t q[8];
extern volatile uint8_t q_head, q_tail;
// Feed instrumentation counters
extern volatile uint32_t feed_loop_pkts;
extern volatile uint32_t feed_isr_pkts;
extern volatile uint32_t feed_busy_skips;
extern volatile uint32_t feed_chain_max;
extern volatile uint32_t feed_chain_current;
#endif

#endif
