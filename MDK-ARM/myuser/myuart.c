#include "myuart.h"
#include <string.h>
#include "servoctrl.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern osThreadId mpuHandle;
extern osThreadId ServoCtrlHandle;

uint8_t uart3_rx_buffer[30];
uint8_t uart2_rx_buffer[16];
extern uint8_t rec[sizeof(recresu_t)];
extern uint8_t mpurebuf[8];
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	 {
		//if(mpuHandle != NULL) osSignalSet(mpuHandle,0x02);
	 }
	 if(huart->Instance == USART3)
	 {
		 //osSignalSet(ServoCtrlHandle,0x08);
	 }
	
}
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
	 if(huart->Instance == USART2)
	 {
		// if(mpuHandle != NULL) osSignalSet(mpuHandle,0x01);
		 //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	 }
	 if(huart->Instance == USART3)
	 {
		 //if(ServoCtrlHandle != NULL) osSignalSet(ServoCtrlHandle,0x08);
	 }
	 
 }
 
  /* 接收不定长数据 */
 void  USARTReceive_IDLE(UART_HandleTypeDef *huart)
{
	
	if((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_AbortReceive_IT(huart);
		if(huart->Instance == USART2)
		{
			memcpy(mpurebuf,uart2_rx_buffer,sizeof(mpurebuf));
			if(mpuHandle != NULL) osSignalSet(mpuHandle,0x01);
			HAL_UART_Receive_DMA(huart,uart2_rx_buffer,30);
		}
		else if(huart->Instance == USART3)
		{
			//HAL_UART_AbortReceive_IT(huart);
			memcpy(rec,uart3_rx_buffer,sizeof(rec));
			if(ServoCtrlHandle != NULL) osSignalSet(ServoCtrlHandle,0x08);
			HAL_UART_Receive_DMA(huart,uart3_rx_buffer,30);

		}
	
	}
	else return;
}


 
