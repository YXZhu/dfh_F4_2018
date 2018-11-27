#include "mpu.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "uart.h"
#include  <math.h> 

extern UART_HandleTypeDef huart2;
extern osThreadId mpuHandle;
extern osThreadId moto_jzHandle;
extern osThreadId ServoCtrlHandle;
uint8_t mpurebuf[8] = {0};
int16_t YAW,PITCH,ROLL;

void mputask(void const * argument)
{
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;
	xLastWakeTime = xTaskGetTickCount();
	YAW = 0;
	PITCH = 0;
	ROLL = 0;
	HAL_UART_Receive_DMA(&huart2, mpurebuf,8);
    osEvent event;	
	for(;;)
	{
		event = osSignalWait(0x02,80);
		if(event.status == osEventSignal)
		{
			//s:
			if(event.value.signals&0x01)
			{
//				 if(!(mpurebuf[0]==0xaa))//如果帧头错误，清缓存
//				 {	
//					 HAL_UART_AbortReceive(&huart2);
//					//HAL_UART_DMAStop(&huart2);
//					HAL_UART_Receive_DMA(&huart2, mpurebuf,8); 
//					mpurebuf[0]=0;
//					// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15,GPIO_PIN_SET);
//				 }
//				 else
//				 {
					 if(mpurebuf[7]==0x55)
					{
					 YAW=(mpurebuf[1]<<8|mpurebuf[2]);//YAW，PITCH,ROLL为真实值的100倍
						if(moto_jzHandle != NULL) osSignalSet(moto_jzHandle,0x01); // 转弯服务
						HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
					 //PITCH=(mpurebuf[3]<<8|mpurebuf[4]);
					// ROLL=(mpurebuf[5]<<8|mpurebuf[6]);
					 //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15,GPIO_PIN_RESET);
					}
				//}
			}
			if(event.value.signals&0x02)
			{
				 if(!(mpurebuf[0]==0xaa))//如果帧头错误，清缓存
				 {	
					 HAL_UART_AbortReceive(&huart2);
					//HAL_UART_DMAStop(&huart2);
					HAL_UART_Receive_DMA(&huart2, mpurebuf,8); 
					mpurebuf[0]=0;
					// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15,GPIO_PIN_SET);
				 }
			 }
		}
		else if(event.status == osEventTimeout)
		{
			 HAL_UART_AbortReceive(&huart2);
					//HAL_UART_DMAStop(&huart2);
			HAL_UART_Receive_DMA(&huart2, mpurebuf,8); 
			HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
			//goto s;
		}

//		ab[3]=ab[1]/sqrt((ab[0]*ab[0]+ab[2]*ab[2]));
//                        ab[3]=atan(ab[3]);
	//print_usart2("%d,%d,%d\n\r",x,y,z);
		//print_usart2("%d,%d,%d,%d\n\r",ab[0],ab[1],ab[2],ab[3]);
	//print_usart2("%d,%d,%d\n\r",YAW);
		
		//osDelayUntil(&xLastWakeTime, xFrequency);
	 }
 }
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	 {
		if(mpuHandle != NULL) osSignalSet(mpuHandle,0x02);
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
		 if(mpuHandle != NULL) osSignalSet(mpuHandle,0x01);
		 //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	 }
	 if(huart->Instance == USART3)
	 {
		 if(ServoCtrlHandle != NULL) osSignalSet(ServoCtrlHandle,0x08);
	 }
	 
 }
