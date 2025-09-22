#pragma once
#include "usbd_core.h"
/* Raw vendor bulk class (IN 0x81, OUT 0x01) */
extern USBD_ClassTypeDef USBD_RAW;

/* Simple streaming API used by ADC DMA callback to enqueue large buffers
 * which are then packetised into 64-byte bulk transfers on EP 0x81.
 * Data is sent losslessly in order (bulk guarantees reliability). */
void usbd_raw_stream_submit(const uint8_t *data, uint32_t length);
void usbd_raw_stream_task(void); /* call in main loop to kick initial transfer */

extern volatile uint32_t usbd_raw_stream_queued_bytes; /* cumulative bytes accepted */
extern volatile uint32_t usbd_raw_stream_drops;        /* buffers dropped due to full queue */

#ifndef USE_RAW
/* Provide harmless inline stubs if raw mode excluded */
static inline void usbd_raw_stream_submit(const uint8_t *data, uint32_t length) { (void)data; (void)length; }
static inline void usbd_raw_stream_task(void) {}
#endif

