#ifndef IQ_ADC_H
#define IQ_ADC_H
#include <stdint.h>
#include "stm32f1xx_hal.h"

// Raised from 100000 to ~166.7 kS/s (timer base 1MHz, ARR=5 -> 1e6/(5+1) = 166666.6)
#define IQ_SAMPLE_RATE_HZ 166667U
#define IQ_BUFFER_PAIRS   1024U
#define IQ_DMA_LENGTH     (IQ_BUFFER_PAIRS*2U)

typedef void (*iq_callback_t)(uint32_t *data, uint32_t count, uint8_t index);

void iq_init(iq_callback_t cb);
void iq_start(void);
void iq_stop(void);

extern volatile uint32_t iq_dma_half_count;
extern volatile uint32_t iq_dma_full_count;

static inline uint32_t iq_half_count(void) { return iq_dma_half_count; }
static inline uint32_t iq_full_count(void) { return iq_dma_full_count; }

#endif
