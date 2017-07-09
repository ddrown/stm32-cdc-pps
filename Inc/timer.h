#ifndef TIMER_H
#define TIMER_H

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

uint32_t get_counters();
void timer_start();

extern volatile struct pps_capture_t {
  uint32_t irq_milli;
  uint32_t irq_time;
  uint16_t cap_tim1;
  uint32_t sent_milli;
  uint32_t sent_time;
} pps_capture;

#endif
