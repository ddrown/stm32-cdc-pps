#include "stm32f1xx_hal.h"

#include "timer.h"
#include "usbd_cdc_if.h"

volatile struct pps_capture_t pps_capture;

uint32_t get_counters() {
  uint16_t tim2_before, tim3, tim2_after;

  // TODO: disable interrupts?
  tim2_before = __HAL_TIM_GET_COUNTER(&htim2);
  tim3 = __HAL_TIM_GET_COUNTER(&htim3);
  tim2_after = __HAL_TIM_GET_COUNTER(&htim2);

  if(tim2_before != tim2_after) {
    if(tim3 > 60000) { // allow for ~5000 cycles between before/after (is this enough for interrupts?)
      tim2_after = tim2_before;
    }
  }

  return ((uint32_t)tim2_after) << 16 | tim3;
}

void timer_start() {
  pps_capture.ch2_events = 0;
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  uint32_t irq_milli, irq_time;

  irq_milli = HAL_GetTick();
  irq_time = get_counters();
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    pps_capture.cap_tim3 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
    pps_capture.irq_milli = irq_milli;
    pps_capture.irq_time = irq_time;
    CDC_Transmit_serial_state(CDC_SERIAL_STATE_DCD);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  } else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    pps_capture.ch2_cap_tim3 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
    pps_capture.ch2_irq_time = irq_time;
    pps_capture.ch2_events++;
  }
}
