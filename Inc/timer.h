#ifndef TIMER_H
#define TIMER_H

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

uint32_t get_counters();
void timer_start();

extern volatile struct pps_capture_t {
  uint32_t irq_milli;
  uint32_t irq_time;
  uint16_t cap_tim3;
  uint32_t sent_milli;
  uint32_t sent_time;
  uint32_t ch2_irq_time;
  uint16_t ch2_cap_tim3;
  uint8_t ch2_events;
} pps_capture;

#endif
