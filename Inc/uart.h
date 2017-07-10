#ifndef UART_H
#define UART_H

extern UART_HandleTypeDef huart1;
#define UART_NAME huart1
extern volatile uint32_t last_uartData;

void start_rx_uart();
uint8_t flush_uart();

#endif // UART_H
