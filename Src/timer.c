#include "stm32f1xx_hal.h"

#include "timer.h"
#include "usbd_cdc_if.h"

volatile struct pps_capture_t pps_capture;

uint32_t get_counters() {
  uint16_t tim2_before, tim1, tim2_after;

  // TODO: disable interrupts?
  tim2_before = __HAL_TIM_GET_COUNTER(&htim2);
  tim1 = __HAL_TIM_GET_COUNTER(&htim1);
  tim2_after = __HAL_TIM_GET_COUNTER(&htim2);

  if(tim2_before != tim2_after) {
    if(tim1 > 60000) { // allow for ~5000 cycles between before/after (is this enough for interrupts?)
      tim2_after = tim2_before;
    }
  }

  return ((uint32_t)tim2_after) << 16 | tim1;
}

void timer_start() {
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  pps_capture.irq_milli = HAL_GetTick();
  pps_capture.irq_time = get_counters();
  pps_capture.cap_tim1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
  CDC_Transmit_serial_state(CDC_SERIAL_STATE_DCD);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
