#include "stm32f1xx_hal.h"

#include <string.h>
#include <stdlib.h>

#include "uart.h"
#include "usbd_cdc_if.h"

// aim for a 1ms buffer
#define BUFFERLEN 12
#define BUFFERS 2
uint8_t uartData[BUFFERS][BUFFERLEN];
volatile uint32_t last_uartData;
uint8_t active_uartData = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  CDC_Transmit_FS(uartData[active_uartData], BUFFERLEN);
  active_uartData = (active_uartData + 1) % BUFFERS;
  HAL_UART_Receive_IT(huart, uartData[active_uartData], BUFFERLEN);
  last_uartData = HAL_GetTick();
}

void start_rx_uart() {
  HAL_UART_Receive_IT(&UART_NAME, uartData[active_uartData], BUFFERLEN);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, (uint8_t *)UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

uint8_t flush_uart() {
  if(UART_NAME.RxXferCount == BUFFERLEN)
    return 0;

  CDC_Transmit_FS(uartData[active_uartData], BUFFERLEN - UART_NAME.RxXferCount);
  active_uartData = (active_uartData + 1) % BUFFERS;

  // races with interrupt
  __disable_irq();
  UART_NAME.pRxBuffPtr = uartData[active_uartData];
  UART_NAME.RxXferSize = BUFFERLEN;
  UART_NAME.RxXferCount = BUFFERLEN;
  __enable_irq();

  return 1;
}
