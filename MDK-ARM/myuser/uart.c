#include "uart.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdarg.h>
#include <string.h>
USART_RECEIVETYPE UsartType1; 
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;




static int inHandlerMode (void)
{
	  return __get_IPSR() != 0;
}

void print_usart2(char *format, ...)
{
	 char buf[64];

	 if(inHandlerMode() != 0)
	  taskDISABLE_INTERRUPTS();
	 else
	 {
		while(HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX)
			osThreadYield();
	 }

	 va_list ap;
	 va_start(ap, format);
	 if(vsprintf(buf, format, ap) > 0)
	 {
		 HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), 100);
	 }
	 va_end(ap);

	 if(inHandlerMode() != 0)
		taskENABLE_INTERRUPTS();
}

