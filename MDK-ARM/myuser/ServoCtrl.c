#include "servoctrl.h"
#include <string.h>
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3;
#define S1_OUT TIM_CHANNEL_1
#define S2_OUT TIM_CHANNEL_2
#define S3_OUT TIM_CHANNEL_3
#define Cam_OUT TIM_CHANNEL_4
#define SxTime htim3

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) ) //��С����
	
ServoCtrl_t ServoCtrlS1;
ServoCtrl_t ServoCtrlS2;
ServoCtrl_t ServoCtrlS3;
ServoCtrl_t ServoCtrlCam; // ����ͷ
//uint32_t timer;
//enum 
//{
//	ServoS1 =0,
//	ServoS2,
//	ServoS3
//};
recresu_t recresu;
uint8_t rec[sizeof(recresu_t)];
osThreadId ServoCtrlHandle;

uint32_t Angle2Pulse(uint8_t angle,uint8_t exchange); 
uint8_t AngleRun(ServoCtrl_t *ServoCtr);
void ServoCtrlTask(void const * argument);
void readResu(void);
void catsome(void);
void ServoCtrlFreertosInit(void) // �� ��s1 120 160 s2 50 s3 0 ���� s1 120 170 s2 55 s3 18 ���� s1 100 130 s2 42 s3 18
{
	/* ��ʼ�����*/
	ServoCtrlS1.MinAngleSet = 0;
	ServoCtrlS1.MaxAngleSet = 180;
	ServoCtrlS1.ServoAngleSet = 0; // 115 ƽֱ     0 ����
	ServoCtrlS1.ServoAngleNow = 180;
	ServoCtrlS1.ServoSpeed = 360;
	
	ServoCtrlS2.MinAngleSet = 18;
	ServoCtrlS2.MaxAngleSet = 70; //50
	ServoCtrlS2.ServoAngleSet = 18; //18 //18 ���
	ServoCtrlS2.ServoAngleNow = 18;
	ServoCtrlS2.ServoSpeed = 360;
	
	ServoCtrlS3.MinAngleSet = 15;
	ServoCtrlS3.MaxAngleSet = 180;
	ServoCtrlS3.ServoAngleSet = 95; // ǰ������ ˳ʱ�� Ϊ15 ��ʱ��Ϊ 180
	ServoCtrlS3.ServoAngleNow = 95;
	ServoCtrlS3.ServoSpeed = 360;
	
	ServoCtrlCam.MinAngleSet = 0;
	ServoCtrlCam.MaxAngleSet = 180;
	ServoCtrlCam.ServoAngleSet = 90;  //35 160  // ǰ������ ˳ʱ�� Ϊ15 ��ʱ��Ϊ 180
	ServoCtrlCam.ServoAngleNow = 95;
	ServoCtrlCam.ServoSpeed = 180;	
	
  osThreadDef(ServoCtrlTask, ServoCtrlTask, osPriorityNormal, 0, 128);
  ServoCtrlHandle = osThreadCreate(osThread(ServoCtrlTask), NULL);
  
}
uint32_t plues = 1500;
uint8_t KenAngle = 90;
void ServoCtrlTask(void const * argument)
{
	osEvent event;
	uint32_t timeZero;
	uint32_t timerun;
	//uint8_t sendKen[2];
	uint8_t catSet; //�Ƿ��ժ��־
	memset(&recresu,0,sizeof(recresu));
	memset(&rec,0,sizeof(rec));
	osDelay(500);
	HAL_TIM_PWM_Start_IT(&SxTime,S1_OUT);
	HAL_TIM_PWM_Start_IT(&SxTime,S2_OUT);
	HAL_TIM_PWM_Start_IT(&SxTime,S3_OUT);
	HAL_TIM_PWM_Start_IT(&SxTime,Cam_OUT);
	//ServoCtrlS1.TimeLast = osKernelSysTick();
	//ServoCtrlS2.TimeLast = osKernelSysTick();
	//ServoCtrlS3.TimeLast = osKernelSysTick();
	//sendKen[0] = 0xAD;
	__HAL_TIM_SET_COMPARE(&SxTime,S1_OUT,Angle2Pulse(AngleRun(&ServoCtrlS1),0));
	__HAL_TIM_SET_COMPARE(&SxTime,S2_OUT,Angle2Pulse(AngleRun(&ServoCtrlS2),0));
	__HAL_TIM_SET_COMPARE(&SxTime,S3_OUT,Angle2Pulse(AngleRun(&ServoCtrlS3),0));
	
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE); //�������ڿ����ж� ������������
   HAL_UART_Receive_DMA(&huart3,rec,sizeof(rec));
	timeZero = osKernelSysTick();
	if(!HAL_GPIO_ReadPin(Startnum_GPIO_Port,Startnum_Pin)) catSet = 0;
	else catSet = 1;
	//timerun = timeZero; // ��¼��ǰʱ��
  for(;;)
  {  
	  event = osSignalWait(0xff,40);
	  if(event.status == osEventSignal)
	  {
		  if(event.value.signals&0x01)
			 // __HAL_TIM_SET_COMPARE(&SxTime,S1_OUT,plues);
				__HAL_TIM_SET_COMPARE(&SxTime,S1_OUT,Angle2Pulse(AngleRun(&ServoCtrlS1),0));
		  if(event.value.signals&0x02)
				__HAL_TIM_SET_COMPARE(&SxTime,S2_OUT,Angle2Pulse(AngleRun(&ServoCtrlS2),0));
		  if(event.value.signals&0x04)
				__HAL_TIM_SET_COMPARE(&SxTime,S3_OUT,Angle2Pulse(AngleRun(&ServoCtrlS3),0));
		  if(event.value.signals&0x10)
				__HAL_TIM_SET_COMPARE(&SxTime,Cam_OUT,Angle2Pulse(AngleRun(&ServoCtrlCam),0));
		  if(event.value.signals&0x08)
		  {
			  readResu();
			  timeZero = osKernelSysTick(); 
			  
		  }
	  }
	  else if(event.status == osEventTimeout)
	  {
		  
	  }
	  if(catSet == 1) catsome(); // ������ժ
	  else setServo(&ServoCtrlS2,100,60); // ̧�߲�ժ�� 
	  if(osKernelSysTick() - timeZero > 200) // 200ms ����
	  {
		 
		  //sendKen[0] = sendKen[1] = KenAngle;		 
		  timeZero = osKernelSysTick();
		  memset(&recresu,0,sizeof(recresu));
	  }
    //osDelay(5);
  }
}


// ��ժ����
extern int32_t setSPEED; // �����ٶ�
extern uint8_t angleJS; //��¼ת��Ĵ��� ���ɸ��ģ���
extern uint8_t moto_control1; //���õ��״̬ ���ɸ��� 4 ��ת 5 ��ת
//resu.x 320 ~ 0 ���� resu.y 240 ~ 0 �ϵ���
void catsome()
{
	static uint8_t state;
	if(moto_control1 == 4) setServo(&ServoCtrlS3,100,180);
	else if(moto_control1 == 5) setServo(&ServoCtrlS3,100,15);
	else
	{
		switch(angleJS)
		{
			case 0:
				setServo(&ServoCtrlS2,100,60);
			break;
			case 1:
				if(state == 0)
				{
					state = 1;
					setServo(&ServoCtrlS3,100,95);
					setServo(&ServoCtrlS2,100,18);
					setServo(&ServoCtrlCam,180,60);
				}
				else
				{
					if(recresu.bg != 0)
					{
						if(recresu.x > 150 && recresu.w >= 50) // ��Ϊ�ǻ���
						{						
							setServo(&ServoCtrlS3,100,180);
							setServo(&ServoCtrlS1,180,120);
						}
					}
				}
			break;
		}
	}
	
}
	
void readResu()
{
	if(rec[0] == 0xaa&& rec[sizeof(rec)-1] == 0xed)
	{
		memcpy(&recresu,rec,sizeof(rec));
	}
	else
	{
		rec[0] = 0;
	   rec[sizeof(rec)-1] = 0;
		recresu.bg = 0;
		recresu.ed = 0;
	}
	HAL_UART_Receive_DMA(&huart3,rec,sizeof(rec));	
}

// ���ö���ٶ� �Ƕ�
void setServo(ServoCtrl_t *ServoCtr,float Speed,uint8_t Angle) // ��ÿ��	
{
	ServoCtr->ServoSpeed = Speed;
	ServoCtr->ServoAngleSet = Angle;
}

//void motionrun(uint16_t *data,
/* angle �Ƕ� exchange ���÷�����Ϊ0�� */
uint32_t Angle2Pulse(uint8_t angle,uint8_t exchange)
{
	#define MaxPulse 2500.f
	#define MinPulse 600.f
	if(angle > 180 ) return 0;
	if(exchange)
		return  (uint32_t)((180 - angle)*((MaxPulse - MinPulse)/180.f)+MinPulse);
	else
		return (uint32_t)(angle*((MaxPulse - MinPulse)/180.f)+MinPulse);
}

/* �Ƕȿ��� �ٶȿ��� */
uint8_t AngleRun(ServoCtrl_t *ServoCtr)
//uint8_t AngleRun(uint8_t angleSet,uint8_t angleSpeed,uint8_t *nowangle,uint32_t *time)
{
	float temp;
	
	if(ServoCtr->ServoAngleSet > ServoCtr->MaxAngleSet) // �ж��ǲ����ڸö�����÷�Χ��
	{
		ServoCtr->ServoAngleSet = ServoCtr->MaxAngleSet;
	}
	else if(ServoCtr->ServoAngleSet < ServoCtr->MinAngleSet)
	{
		ServoCtr->ServoAngleSet = ServoCtr->MinAngleSet;
	}
	
	if(ServoCtr->ServoAngleSet == ServoCtr->ServoAngleNow)
	{
		ServoCtr->TimeLast = osKernelSysTick(); // ���浱ǰʱ��
		return ServoCtr->ServoAngleSet;
	}
	else
	{
//		if(ServoCtr->TimeLast == 0) // δ��ȡʱ��
//		{
//			ServoCtr->TimeLast = osKernelSysTick();
//			return ServoCtr->ServoAngleNow;
//		}
		
		temp = osKernelSysTick() - ServoCtr->TimeLast; // ��ȡ��ǰʱ��֮��
		temp = 1000.f / temp; //  ���㵱ǰ������ֵ
		ServoCtr->TimeLast = osKernelSysTick(); // ���浱ǰʱ��
		//if(temp == 0) return 255; // �����ٶȹ���
		temp = (float)ServoCtr->ServoSpeed / temp;
		
		if(ServoCtr->ServoAngleSet > ServoCtr->ServoAngleNow)
		{	
			temp = ServoCtr->ServoAngleNow + temp;	
			if(temp >= ServoCtr->ServoAngleSet) 
				ServoCtr->ServoAngleNow = ServoCtr->ServoAngleSet;	
			else
				ServoCtr->ServoAngleNow = temp;
		}
		else
		{
			temp = ServoCtr->ServoAngleNow - temp;
			if(temp <= ServoCtr->ServoAngleSet) 
				ServoCtr->ServoAngleNow = ServoCtr->ServoAngleSet;	
			else
				ServoCtr->ServoAngleNow = temp;
		}
		return ServoCtr->ServoAngleNow;
	}
}

// pwm �����ж� ʹ����˶������� 
 void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
 {
	 if(htim->Instance == TIM3)
	 {
		 if(ServoCtrlHandle == NULL) return;
		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		 {
			 osSignalSet(ServoCtrlHandle,0x01); //ServoCtrlS1
		 }
		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		 {
			  osSignalSet(ServoCtrlHandle,0x02); //ServoCtrlS2
		 }
		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		 {
			  osSignalSet(ServoCtrlHandle,0x04); //ServoCtrlS3
		 }
		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		 {
			  osSignalSet(ServoCtrlHandle,0x10); //ServoCtrlS3
			 
		 }
	 }
	 
 }
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	 if(ServoCtrlHandle == NULL) return;
	 if(GPIO_Pin == Right_Int_Pin)
	 {
		 osSignalSet(ServoCtrlHandle,0x20);
	 }
	 if(GPIO_Pin == LEFT_Int_Pin)
	 {
		 osSignalSet(ServoCtrlHandle,0x40);
	 }
	 
	 
 }
 
 /**************************************/
 
 

