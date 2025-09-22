// Interrupt handlers for STM32F1 (minimal set needed for this project)
#include "stm32f1xx_hal.h"

// Provide SysTick handler (was missing) so HAL_GetTick advances.
void SysTick_Handler(void) {
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}
