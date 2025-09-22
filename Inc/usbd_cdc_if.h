#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H
#include "usbd_cdc.h"

#ifndef CDC_IN_EP
#define CDC_IN_EP 0x81U
#endif

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
uint8_t CDC_Stream_Active(void);
uint32_t CDC_Stream_Drops(void);
void CDC_Debug_ForceStart(void);
#ifdef MINIMAL_CDC
void CDC_Minimal_Task(void); // periodic small packet sender
void CDC_Background_Poke(void); // new: unconditional background attempt & counters
uint32_t CDC_Debug_TxCompletes(void);
uint32_t CDC_Debug_TxAttempts(void);
#endif

#endif
