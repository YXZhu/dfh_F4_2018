#ifndef __UART_H
#define __UART_H

#define RECEIVELEN 1024
#define USART_DMA_SENDING 1//发送未完成
#define USART_DMA_SENDOVER 0//发送完成

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
typedef struct
{
uint8_t receive_flag:1;//空闲接收标记
uint8_t dmaSend_flag:1;//发送完成标记
uint16_t rx_len;//接收长度
uint8_t usartDMA_rxBuf[RECEIVELEN];//DMA接收缓存
}USART_RECEIVETYPE;

extern USART_RECEIVETYPE UsartType1;

void Usart1SendData_DMA(uint8_t *pdata, uint16_t Length);
void UsartReceive_IDLE(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void print_usart2(char *format, ...);

#endif
