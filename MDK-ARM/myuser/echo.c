#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "echo.h"
#include "dwt_stm32_delay.h"
#include "mymath.h"

//unsigned char SW1,SW2,SW3,SW4,SW5,SW6;
//unsigned char SW1;
CCMRAM float ED1,ED2,ED3,ED4,ED5,ED6;
uint32_t EDjl1,EDjl2,EDjl3,EDjl4,EDjl5,EDjl6; // ≥¨…˘≤®æ‡¿Î
//unsigned char temp1,temp2,temp3,temp4,temp5,temp6;	
uint32_t echo1[2] = {0}, echo2[2] = {0}, echo3[2] = {0}, echo4[2] = {0},echo5[2] = {0},echo6[2] = {0};

CCMRAM KalmanFilterValue_t Echo1Kalman,Echo2Kalman,Echo3Kalman,Echo4Kalman,Echo5Kalman,Echo6Kalman;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

osThreadId Echo_1Handle;
osThreadId Echo_2Handle;
osThreadId Echo_3Handle;
osThreadId Echo_4Handle;
osThreadId Echo_5Handle;
osThreadId Echo_6Handle;

void Echo_1task(void const * argument);
void Echo_2task(void const * argument);
void Echo_3task(void const * argument);
void Echo_4task(void const * argument);
void Echo_5task(void const * argument);
void Echo_6task(void const * argument);

void UDelayUS (unsigned int ulCount)
{
	unsigned int i;
	for ( i = 0; i < ulCount; i ++ )
	{
		uint8_t uc = 12;     //…Ë÷√÷µŒ™12£¨¥Û‘º—”1Œ¢√Î  	      
		while ( uc -- );     //—”1Œ¢√Î	
	}	
}

#define EchoTime 15
void EchoFreertosInit()
//void Echo_Init(void)
{

	EDjl1 = 0;
	EDjl2 = 0;
	EDjl3 = 0;
	EDjl4 = 0;
	EDjl5 = 0;
	EDjl6 = 0;
	ED1 = 0;
	ED2 = 0;
	ED3 = 0;
	ED4 = 0;
	ED5 = 0;
	ED6 = 0;
	/* ø®∂˚¬¸¬À≤® */
	
	Echo1Kalman.ProcessNiose_Q = 0.15f;
	Echo1Kalman.MeasureNoise_R = 1.1f; //1.1
	
	Echo2Kalman.ProcessNiose_Q = 0.15f;
	Echo2Kalman.MeasureNoise_R = 1.1f;
	
	Echo3Kalman.ProcessNiose_Q = 0.15f;
	Echo3Kalman.MeasureNoise_R = 1.1f;
	
	Echo4Kalman.ProcessNiose_Q = 0.15f;
	Echo4Kalman.MeasureNoise_R = 1.1f;
	
	Echo5Kalman.ProcessNiose_Q = 0.15f;
	Echo5Kalman.MeasureNoise_R = 1.1f;
	
	Echo6Kalman.ProcessNiose_Q = 0.15f; //0.09
	Echo6Kalman.MeasureNoise_R = 1.1f;
	
	osThreadDef(Echo_1, Echo_1task, osPriorityRealtime, 0, 128);
	Echo_1Handle = osThreadCreate(osThread(Echo_1), NULL);

	/* definition and creation of Echo_2 */
	osThreadDef(Echo_2, Echo_2task, osPriorityRealtime, 0, 128);
	Echo_2Handle = osThreadCreate(osThread(Echo_2), NULL);

	/* definition and creation of Echo_3 */
	osThreadDef(Echo_3, Echo_3task, osPriorityRealtime, 0, 128);
	Echo_3Handle = osThreadCreate(osThread(Echo_3), NULL);

	/* definition and creation of Echo_4 */
	osThreadDef(Echo_4, Echo_4task, osPriorityRealtime, 0, 128);
	Echo_4Handle = osThreadCreate(osThread(Echo_4), NULL);

	osThreadDef(Echo_5,Echo_5task,osPriorityRealtime, 0, 128);
	Echo_5Handle = osThreadCreate(osThread(Echo_5), NULL);

	osThreadDef(Echo_6, Echo_6task, osPriorityRealtime, 0, 128);
	Echo_6Handle = osThreadCreate(osThread(Echo_6), NULL);
  
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{	
	  switch((uint32_t)htim->Instance)
	  {
		  case (uint32_t)TIM1:
		  {
			  switch(htim->Channel)
			  {
				  case HAL_TIM_ACTIVE_CHANNEL_1:
				  {
					 if(HAL_GPIO_ReadPin(Echo_1_GPIO_Port,Echo_1_Pin))
					 {
						 echo1[0] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
					 }
					 else
					 {
						 echo1[1] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
						 osSignalSet(Echo_1Handle,0x01);
					 }
				 }
				  break;
				  case HAL_TIM_ACTIVE_CHANNEL_2:
				  {
					 if(HAL_GPIO_ReadPin(Echo_2_GPIO_Port,Echo_2_Pin))
					 {
						 echo2[0] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
						 //temp2 = 0;
					 }
					 else
					 {
						 echo2[1] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
						 //temp2 = 1;
						 osSignalSet(Echo_2Handle,0x01);
					 }
				 }
				  break;
				  case HAL_TIM_ACTIVE_CHANNEL_3:
				  {
					 if(HAL_GPIO_ReadPin(Echo_3_GPIO_Port,Echo_3_Pin))
					 {
						 echo3[0] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
						 //temp3 = 0;
					 }
					 else
					 {
						 echo3[1] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
						 //temp3 = 1;
						 osSignalSet(Echo_3Handle,0x01);
					 }
				 }
				  break;	
				  case HAL_TIM_ACTIVE_CHANNEL_4:
				  {
					 if(HAL_GPIO_ReadPin(Echo_4_GPIO_Port,Echo_4_Pin))
					 {
						 echo4[0] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
						 //temp4 = 0;
					 }
					 else
					 {
						 echo4[1] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
						 //temp4 = 1;
						 osSignalSet(Echo_4Handle,0x01);
					 }
				 }
				  break;	
				  default:break;
			 }
		 }
		 break;
		 case (uint32_t)TIM2:
		 {
			 switch(htim->Channel)
			 {
				  case HAL_TIM_ACTIVE_CHANNEL_3:
				  {
					 if(HAL_GPIO_ReadPin(Echo_5_GPIO_Port,Echo_5_Pin))
					 {
						 echo5[0] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
						 //temp5 = 0;
					 }
					 else
					 {
						 echo5[1] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
						 //temp5 = 1;
						 osSignalSet(Echo_5Handle,0x01);
					 }
				 }
				  break;	
				  case HAL_TIM_ACTIVE_CHANNEL_4:
				  {
					 if(HAL_GPIO_ReadPin(Echo_6_GPIO_Port,Echo_6_Pin))
					 {
						 echo6[0] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
						 //temp6 = 0;
					 }
					 else
					 {
						 echo6[1] =  HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
						 //temp6 = 1;
						 osSignalSet(Echo_6Handle,0x01);
					 }
				 }
				  break;	
				  default:break;
			 }
		 }
		 break;
		 default:break;
	 }
} 

//uint16_t ED1t,ED2t,ED3t,ED4t,ED5t,ED6t;
extern osThreadId main_1Handle;
extern uint8_t moto_control1;

void Echo_1task(void const * argument)
{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = EchoTime;
	//xLastWakeTime = xTaskGetTickCount();
	osEvent event;
	//uint32_t echo2time;
  for(;;)
  {
	  event = osSignalWait(0x0f,EchoTime);
	  if(event.status == osEventSignal)
	  {
		  if(event.value.signals&0x01)
		  {
				if(echo1[0]<echo1[1]) ED1 = echo1[1] - echo1[0];
				else ED1 = (50000-echo1[0]) + echo1[1];			   
				ED1*=0.17f;
			   EDjl1 = KalmanFilter(&Echo1Kalman,ED1);
//				if(((ED1 - ED1t) > 100) || ((ED1t - ED1) > 100)) EDjl1 = ED1t;
//				else	EDjl1 = ED1;
//				ED1t = ED1;
				//SW1 = 1;	
		  }
		  
	  }
	  else if(event.status == osEventTimeout)
	  {
		   HAL_GPIO_WritePin(Trig_1_GPIO_Port,Trig_1_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
			DWT_Delay_us(15);		
			HAL_GPIO_WritePin(Trig_1_GPIO_Port,Trig_1_Pin,GPIO_PIN_RESET);		  
	  }
	 
  }
}
void Echo_2task(void const * argument)
{
	osEvent event;
	for(;;)
	{
		event = osSignalWait(0x02,EchoTime);
		if(event.status == osEventSignal)
		{
		  if(event.value.signals&0x01)
		  {
			  if(echo2[0]<echo2[1]) ED2 =  echo2[1] - echo2[0];
			  else ED2 = (50000-echo2[0]) + echo2[1];
			  ED2*=0.17f;
			  EDjl2 = KalmanFilter(&Echo2Kalman,ED2);
//			  if(((ED2 - ED2t) > 100) || ((ED2t - ED2) > 100)) EDjl2 = ED2t;
//			  else	EDjl2 = ED2;
//			  ED2t = ED2;
		  }
	  }
		else if(event.status == osEventTimeout)
	  {
			HAL_GPIO_WritePin(Trig_2_GPIO_Port,Trig_2_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
			DWT_Delay_us(15);
			HAL_GPIO_WritePin(Trig_2_GPIO_Port,Trig_2_Pin,GPIO_PIN_RESET);//Trig_1 = 0;			
		}
	}
}

void Echo_3task(void const * argument)
{
	osEvent event;
	 for(;;)
	 {
		event = osSignalWait(0x02,EchoTime);
		if(event.status == osEventSignal)
		{
			if(event.value.signals&0x01)
		  {
			  if(echo3[0]<echo3[1]) ED3 = echo3[1] - echo3[0];
			  else ED3 = (50000-echo3[0]) + echo3[1];				 
				ED3*=0.17f;
			  EDjl3 = KalmanFilter(&Echo3Kalman,ED3);
		  }
		}
		else if(event.status == osEventTimeout)
		{
			HAL_GPIO_WritePin(Trig_3_GPIO_Port,Trig_3_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
			DWT_Delay_us(15);
			HAL_GPIO_WritePin(Trig_3_GPIO_Port,Trig_3_Pin,GPIO_PIN_RESET);//Trig_1 = 0;	
		}
	 }
}
void Echo_4task(void const * argument)
{
	osEvent event;
	for(;;)
	{
		event = osSignalWait(0x02,EchoTime);
		if(event.status == osEventSignal)
		{
			if(event.value.signals&0x01)
		  {
			  if(echo4[0]<echo4[1]) ED4 = echo4[1] - echo4[0];
			  else ED4 = (50000-echo4[0]) + echo4[1];				 
				ED4*=0.17f;
				EDjl4 = KalmanFilter(&Echo4Kalman,ED4);
		  }
		}
		else if(event.status == osEventTimeout)
		{
			HAL_GPIO_WritePin(Trig_4_GPIO_Port,Trig_4_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
			DWT_Delay_us(15);
			HAL_GPIO_WritePin(Trig_4_GPIO_Port,Trig_4_Pin,GPIO_PIN_RESET);//Trig_1 = 0;	
		}
	}
}

void Echo_5task(void const * argument)
{
	osEvent event;
	for(;;)
	{
		event = osSignalWait(0x02,EchoTime);
		if(event.status == osEventSignal)
		{
			if(event.value.signals&0x01)
		  {
			  if(echo5[0]<echo5[1]) ED5 = echo5[1] - echo5[0];
			  else ED5 = (50000-echo5[0]) + echo5[1];				 
			  ED5*=0.17f;
			  EDjl5 = KalmanFilter(&Echo5Kalman,ED5);
		  }
		}
		else if(event.status == osEventTimeout)
		{
			HAL_GPIO_WritePin(Trig_5_GPIO_Port,Trig_5_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
			DWT_Delay_us(15);
			HAL_GPIO_WritePin(Trig_5_GPIO_Port,Trig_5_Pin,GPIO_PIN_RESET);//Trig_1 = 0;	
		}
	}
}
void Echo_6task(void const * argument)
{
	osEvent event;
	for(;;)
	{
		event = osSignalWait(0x02,EchoTime);
		if(event.status == osEventSignal)
		{
			if(event.value.signals&0x01)
		  {
			  if(echo6[0]<echo6[1]) ED6 = echo6[1] - echo6[0];
			  else ED6 = (50000-echo6[0]) + echo6[1];				 
			  ED6*=0.17f;
			  EDjl6 = KalmanFilter(&Echo6Kalman,ED6);
		  }
		}
		else if(event.status == osEventTimeout)
		{
			HAL_GPIO_WritePin(Trig_6_GPIO_Port,Trig_6_Pin,GPIO_PIN_SET);//Trig_1 = 1;			
			DWT_Delay_us(15);
			HAL_GPIO_WritePin(Trig_6_GPIO_Port,Trig_6_Pin,GPIO_PIN_RESET);//Trig_1 = 0;	
		}
	}
}
