
#include "moto.h"
#include "echo.h"
#include "pidcontoller.h"
#include "math.h"
#include "uart.h"
#include "arm_math.h"
#include <stdio.h>
//#include <math.h>

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) ) //大小限制
#define ABS(x)   ((x)>=0?(x):-(x))
// 车宽25cm 左右传感器相距23cm 跑道40cm 8.5cm
//double angle;
extern uint16_t EDjl1,EDjl2,EDjl3,EDjl4,EDjl5,EDjl6;
extern int16_t YAW;
extern int32_t setSPEED,speed;
//extern int16_t SPEEDA,SPEEDB;
int32_t SPDA,SPDB; //25ms 测的速度计数；
uint16_t SA=0,SB=0,Timec=0;
int32_t setSPA,setSPB; //设置目标电机速度值

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim4;
CCMRAM float SPDA_CM,SPDB_CM;
CCMRAM float disA_CM,disB_CM;
CCMRAM arm_pid_instance_f32 SPDA_PID,SPDB_PID;


#define TurnSpeed 30 // 转弯速度
#define MiniDis 75
#define MaxDis  100

#define disHold 85 //毫米

CCMRAM int16_t angle_temp1,angle_temp2,angle_temp3,angle_temp4,angle_time;
//SF 0 左旋 SF 1 右旋
int16_t moto_angle(unsigned char SF,int16_t setangle,unsigned char a)
{  
	if(angle_temp1 == 0)
	{
		//angle_temp2 = YAW;
		//setSPA = TurnSpeed;
      //setSPB = TurnSpeed;
		angle_temp1 = 1;
		angle_time = 0;
		if(SF ==1)
		{
			angle_temp2 = YAW - setangle;
			moto_right(2,TurnSpeed);
			//moto_right(0);
		}
		else
		{
			angle_temp2 = YAW + setangle;
			moto_left(2,TurnSpeed);
		}
		if(angle_temp2>18000) angle_temp2 = -36000 + angle_temp2;
		if(angle_temp2<-18000) angle_temp2 = 36000 + angle_temp2;
		angle_temp3 = angle_temp2 + 150;
		if(angle_temp3>18000) angle_temp3 = -36000 + angle_temp3;
		angle_temp4 = angle_temp2 - 150;
		if(angle_temp4<-18000) angle_temp4 = angle_temp4 + 36000;
	}
	if(SF == 1)
	{
		if(angle_time>10) moto_right(a,TurnSpeed);
		else
		{
			angle_time++;
			//moto_right(3);
		}
		if(angle_temp3<angle_temp4)
		{
			if(YAW>=angle_temp4)
			{
				
//				moto_left(a);
//				osDelay(3);
				moto_stop();
				
				angle_temp1 = 0;
				return HAL_OK;
			}
		}
		else
		{			
			if(YAW<angle_temp3)
			{
				if(YAW>=angle_temp4)
				{
//					moto_left(a);
//					osDelay (3);
					moto_stop();
					angle_temp1 = 0;
					return HAL_OK;
				}
				//else moto_left(1);
			}	
	   }		

	}
	else
	{
		if(angle_time>10) moto_left(a,TurnSpeed);
		else
		{
			angle_time++;
			//moto_left(3);
		}
		if(YAW>angle_temp4)
		{
			if(YAW<=angle_temp3)
			{
//				moto_right(a);
//				osDelay (3);
				moto_stop();	
				angle_temp1 = 0;
				//arm_pid_reset_f32(&SPDA_PID);
				//arm_pid_reset_f32(&SPDB_PID);
				return HAL_OK;
			}
		  // else moto_right(1);
		}
		//else moto_left(1);
	}	
	return HAL_BUSY;
}

void moto_front(void)
{
	//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin|N4_Pin,GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin|N3_Pin,GPIO_PIN_SET);
}
void moto_back(void)
{
	//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin|N4_Pin,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin|N3_Pin,GPIO_PIN_RESET);
}



void moto_right(unsigned char a,int32_t sp)
{	
//	setSPA = TurnSpeed;
//   setSPB = TurnSpeed;
   switch(a)
	{
		case 0:
		{
			setSPA = sp;
			setSPB = 0;
		   //HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
	      //HAL_GPIO_WritePin(N4_GPIO_Port,N2_Pin|N3_Pin|N4_Pin,GPIO_PIN_SET);
		}
		break;
		case 1:
		{
			setSPB = -sp;
			setSPA = sp;
		    //HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin|N3_Pin,GPIO_PIN_RESET);
	      // HAL_GPIO_WritePin(N4_GPIO_Port,N2_Pin|N4_Pin,GPIO_PIN_SET);
		}
      break;
		case 2:
		{
			setSPB = -sp;
			setSPA = 0;
			//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin|N2_Pin|N4_Pin,GPIO_PIN_SET);
		   //HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
		}
		break;
		default:break;
	}
	
}

void moto_left(unsigned char a,int32_t sp)
{	
//   setSPA = TurnSpeed;
//   setSPB = TurnSpeed;
	switch(a)
	{
		case 0:
		{
			setSPB = sp;
			setSPA = 0;
			//HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin|N1_Pin|N3_Pin,GPIO_PIN_SET);
	      //HAL_GPIO_WritePin(N3_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
		}
		break;
		case 1:
		{
			setSPB = sp;
			setSPA = -sp;
			//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin|N3_Pin,GPIO_PIN_SET);
			//HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin|N2_Pin,GPIO_PIN_RESET);
		}
		break;
		case 2:
		{
			setSPB = 0;
			setSPA = -sp;
			//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin|N4_Pin|N3_Pin,GPIO_PIN_SET);
			//HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
		}
		break;
		default:break;
	}
}

void moto_stop(void)
{
	setSPA = 0;
	setSPB = 0;
	//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin|N2_Pin|N3_Pin|N4_Pin,GPIO_PIN_RESET);
}

uint8_t moto_control1;

//设置机器行走固定距离，A走多少，B走多少，速度
//读取值 设定值
// direction 机器方向
int16_t ReadDisA = 0,ReadDisB = 0;
void moto_frontDis(int16_t setDisA,int16_t setDisB,int32_t speed1)
{
	int16_t RA,RB;
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 5;
	xLastWakeTime = xTaskGetTickCount();
	ReadDisA = 0;
	ReadDisB = 0;
	disA_CM = 0;
	disB_CM = 0;
	__HAL_TIM_SET_COUNTER(&htim8,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	//setSPB = setSPA = speed1;
	while(1)
	{
		
		osDelayUntil(&xLastWakeTime, xFrequency);
		RA = disA_CM;
		RB = disB_CM;
		if(RA<setDisA)
		{
			setSPA = speed1;
			//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
		}
		else
		{
			setSPA = 0;
			//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin|N2_Pin,GPIO_PIN_RESET);
			//if(ReadDisB<=-setDisB) break ;
		}
		if(RB<setDisB)
		{
			setSPB = speed1;
			//HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
		}
		else
		{
			setSPB = 0;
			//HAL_GPIO_WritePin(N1_GPIO_Port,N3_Pin|N4_Pin,GPIO_PIN_RESET);
			//if(ReadDisA<=-setDisA) break ;
		}
		if((RB>=setDisB) && (RA>=setDisA)) 
		{
			//__HAL_TIM_SET_COUNTER(&htim8,0);
			//__HAL_TIM_SET_COUNTER(&htim4,0);
			break ;
		}
		
	}
}
void moto_backDis(int16_t setDisA,int16_t setDisB,int32_t speed1)
{
	int16_t RA,RB;
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;
	xLastWakeTime = xTaskGetTickCount();
	ReadDisA = 0;
	ReadDisB = 0;
	disA_CM = 0;
	disB_CM = 0;
	__HAL_TIM_SET_COUNTER(&htim8,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	//setSPB = setSPA = speed1;
	while(1)
	{
		osDelayUntil(&xLastWakeTime, xFrequency);
		RA = disA_CM;
		RB = disB_CM;
		if(RA>-setDisA)
		{
			setSPA = -speed1;
			//HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
		}
		else
		{
			setSPA = 0;
			//HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin|N2_Pin,GPIO_PIN_RESET);
			//if(ReadDisB<=-setDisB) break ;
		}
		if(RB>-setDisB)
		{
			setSPB = -speed1;
			//HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
		}
		else
		{
			setSPB = 0;
			//HAL_GPIO_WritePin(N1_GPIO_Port,N3_Pin|N4_Pin,GPIO_PIN_RESET);
			//if(ReadDisA<=-setDisA) break ;
		}
		if((RB<=-setDisB)&&(RA<=-setDisA)) 
		{
			//__HAL_TIM_SET_COUNTER(&htim8,0);
			//__HAL_TIM_SET_COUNTER(&htim4,0);
			break;
		}
		
	}
}
extern osThreadId main_1Handle;
extern osThreadId moto_jzHandle;
extern osThreadId mpuHandle;
extern osThreadId moto_controlHandle;
extern osThreadId bzHandle;
extern osThreadId Echo_1Handle;

extern int16_t LowSpeed; // 低速时速度
#define SlowDownDis  300// 开始减速距离 80cm
CCMRAM arm_pid_instance_f32 EdjlXCtrlPid,EdjlYCtrlPid; // X 水平方向距离 固定某一距离 Y 垂直方向 平行轨道
uint8_t Edjldebug;
CCMRAM int32_t outEdjlX,outEdjlY,inputError;
//int16_t YAWCtrl,YawError;
void moto_jztask(void const * argument)
{
	uint32_t jzspeed;
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 5;
	const TickType_t xFrequency1 = 5;
	const TickType_t xFrequency2 = 120;
	
	moto_control1 = 0;
	speed = 0;
	EdjlXCtrlPid.Kp = 0.21f;
	EdjlXCtrlPid.Ki = 0.f;
	EdjlXCtrlPid.Kd = 0.08f;
	
	EdjlYCtrlPid.Kp = 0.5f;
	EdjlYCtrlPid.Ki = 0.f;
	EdjlYCtrlPid.Kd = 0.3f;	
	//osDelay(200);
	while(YAW == 0) osDelay(10);
	//YAWCtrl = YAW + 18000;
	arm_pid_init_f32(&EdjlXCtrlPid,1);
	arm_pid_init_f32(&EdjlYCtrlPid,1);
	
	osEvent event;
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		osDelayUntil(&xLastWakeTime, xFrequency);
		//if(EDjl2>EDjl3) 
		switch(moto_control1)
		{
			case 0:
			{
				arm_pid_reset_f32(&EdjlXCtrlPid);
				arm_pid_reset_f32(&EdjlYCtrlPid);
			}
			break;
			
			case 1:
			{
				if(speed > 0) // 提前预减速 当speed大于LowSpeed时 不受逻辑影响 
				{
					if(speed == LowSpeed)
					{
							if(EDjl1 < 400) //  根据距离减速
								jzspeed = (speed - LowSpeed)*((EDjl1-SlowDownDis)/100)+LowSpeed;
							else jzspeed = LowSpeed;
					}
					else jzspeed = speed;
				}
				else
					jzspeed = 0;
				
				if(Edjldebug)
				{
					Edjldebug = 0;
					arm_pid_init_f32(&EdjlXCtrlPid,1);
					arm_pid_init_f32(&EdjlYCtrlPid,1);
				}
//				YawError = YAW + 18000 - YAWCtrl;
//				if(YawError > 18000) YawError = 36000 - YawError;	
				if(EDjl3 < 120 && EDjl2 < 120)
				{
					
					inputError = disHold - (EDjl3 + EDjl2)/2;
					outEdjlX = arm_pid_f32(&EdjlXCtrlPid,LIMIT(inputError,-55,55)); //输入限幅
					inputError = EDjl3 - EDjl2;
					//if(ABS(inputError) < 5) YAWCtrl = YAW + 18000;
					outEdjlY = arm_pid_f32(&EdjlYCtrlPid,LIMIT(inputError,-20,20));

					outEdjlY = LIMIT(outEdjlY,-30,30);
					outEdjlX = LIMIT(outEdjlX,-38,38);
					
					setSPA = jzspeed + outEdjlY + outEdjlX;
					setSPB = jzspeed - outEdjlY - outEdjlX;
				}
				else
				{
					arm_pid_reset_f32(&EdjlXCtrlPid);
					arm_pid_reset_f32(&EdjlYCtrlPid);
				
					if(EDjl2<MiniDis)
					{
							//moto_front();
							setSPA = speed;
							setSPB = (speed/1.33f);
					}
					else if(EDjl2>MaxDis)
					{	
						 //moto_front();
						 setSPA = (speed/1.33f);
						 setSPB = speed;			
					}
					else
					{
						if(EDjl3<MiniDis)
						{
							setSPB = speed;
							setSPA = 0;
							//moto_left(0);
						}
						else if(EDjl3>MaxDis)
						{
							setSPA = speed;
							setSPB = 0;
							//moto_right(0);
						}
						else
						{
							setSPA = speed;
							setSPB = speed;
							//moto_front();
						}
					 }
				}
			 }
			break;
			case 2:
			{
				if(Edjldebug)
				{
					Edjldebug = 0;
					arm_pid_init_f32(&EdjlXCtrlPid,1);
					arm_pid_init_f32(&EdjlYCtrlPid,1);
				}
//				YawError = YAW + 18000 - YAWCtrl;
//				if(YawError > 18000) YawError = 36000 - YawError;	
				if(EDjl3 < 120 && EDjl2 < 120)
				{
					
					inputError =  disHold - (EDjl3 + EDjl2)/2;
					outEdjlX = arm_pid_f32(&EdjlXCtrlPid,LIMIT(inputError,-55,55)); //输入限幅
					inputError = EDjl3 - EDjl2;
					//if(ABS(inputError) < 5) YAWCtrl = YAW + 18000;
					outEdjlY = arm_pid_f32(&EdjlYCtrlPid,LIMIT(inputError,-20,20));

					outEdjlY = LIMIT(outEdjlY,-30,30);
					outEdjlX = LIMIT(outEdjlX,-38,38);
					
					setSPA = -speed + outEdjlY - outEdjlX;
					setSPB = -speed - outEdjlY + outEdjlX;
				}
				else
				{
					arm_pid_reset_f32(&EdjlXCtrlPid);
					arm_pid_reset_f32(&EdjlYCtrlPid);
					if(EDjl3<MiniDis)
					{
						//if(EDjl2<175)
						//{
							//moto_back();
							setSPA = -speed;
							setSPB = -(speed/1.33f);
						//}
					}
					else if(EDjl3>MaxDis)
					{	
							//moto_back();
							setSPA = -(speed/1.33f);
							setSPB = -speed;			
					}
					else
					{
						if(EDjl2<MiniDis)
						{
							setSPB = -speed;
							setSPA = 0;
							//moto_right(2);
						}
						else if(EDjl2>MaxDis)
						{
							setSPB = 0;
							setSPA = -speed;
							//moto_left(2);
						}
						else
						{
							setSPA = -speed;
							setSPB = -speed;
							//moto_back();
						}
					}
				}
			}
			break;
			case 4:  
			{//右
//				vTaskSuspend(Echo_1Handle);
//				vTaskResume(mpuHandle);
//			   vTaskSuspend(bzHandle);
				moto_stop();
				osDelay(120);
			  // osDelayUntil(&xLastWakeTime, xFrequency2);
           // vTaskSuspend(moto_controlHandle);
			   //TIM9->CCR1 = 5000;
				//TIM9->CCR2 = 5000;
            
					/*
					while(moto_angle(1,9000,1) != HAL_OK)
					{
						//osSignalWait(0x02,40);
						//if(event.status == osEventTimeout)
						//	break;
						osDelayUntil(&xLastWakeTime,xFrequency1);
						//break;
					}*/
					while(1)
					{
						osSignalWait(0x02,40);
						if(event.status == osEventTimeout)
						{
							moto_stop();
							//break;
						}
						if(moto_angle(1,9000,1) == HAL_OK) break;
					}
						moto_control1 = 0;
					   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15,GPIO_PIN_SET);
					arm_pid_reset_f32(&EdjlXCtrlPid);
						arm_pid_reset_f32(&EdjlYCtrlPid);
									arm_pid_reset_f32(&SPDA_PID);
				     arm_pid_reset_f32(&SPDB_PID);
						//osDelay(1000);
						///vTaskResume(moto_controlHandle);
						//vTaskSuspend(mpuHandle);
//						vTaskResume(Echo_1Handle);
						//osDelayUntil(&xLastWakeTime, xFrequency2);
					   osDelay(120);
//						vTaskResume(bzHandle);
						vTaskResume(main_1Handle);
				
			}
			break;
			case 3:  
			{//左  
				//vTaskSuspend(Echo_1Handle);
				//vTaskResume(mpuHandle);
				//vTaskSuspend(bzHandle);
				moto_stop();
				osDelay(120);
			   //osDelayUntil(&xLastWakeTime, xFrequency2);     			
					/*while(moto_angle(0,9000,1) != HAL_OK)
					{
						osDelayUntil(&xLastWakeTime,xFrequency1);
	
					   //break;	
					}*/
					while(1)
					{
						osSignalWait(0x02,40);
						if(event.status == osEventTimeout)
						{
							moto_stop();
							//break;
						}
						if(moto_angle(0,9000,1) == HAL_OK) break;
					}
					   moto_control1 = 0;
					
					   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15,GPIO_PIN_SET);
								arm_pid_reset_f32(&EdjlXCtrlPid);
						arm_pid_reset_f32(&EdjlYCtrlPid);
									arm_pid_reset_f32(&SPDA_PID);
				     arm_pid_reset_f32(&SPDB_PID);
						//osDelay(200);
						//vTaskResume(moto_controlHandle);
						//vTaskSuspend(mpuHandle);
						//vTaskResume(Echo_1Handle);
						//osDelayUntil(&xLastWakeTime, xFrequency2);
						osDelay(120);
						//vTaskResume(bzHandle);
						vTaskResume(main_1Handle);
				
			}
			break;
			case 5: //前进
			{
				//moto_front();
				setSPB = setSPA = speed;
			}
			break;
			case 6: //后退
			{
				//moto_back();
				setSPB = setSPA = speed;
			}
			case 7: // Test
			{  moto_control1 = 0;
				moto_backDis(4,4,30);
				
			}break;
			case 8: // Test
			{   moto_control1 = 0;
				moto_frontDis(4,4,30);
				
			}break;
			default:break;
		}
		//osDelay(10);
	}
		
}

extern uint8_t angleJS;
void bztask(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1;
	const TickType_t xFrequency1 = 5;
	const TickType_t xFrequency2 = 10;
	const TickType_t xFrequency3 = 1000;
	const TickType_t xFrequency4 = 2;
	xLastWakeTime = xTaskGetTickCount();
	uint8_t swdj_1,bzbjs_1,swjs_1,swjs_3;
	swjs_1 = 0;
	swjs_3 = 0;
	bzbjs_1 = 0;
	swdj_1 = 1;
//	setSPEED = 8000;
//	speed = setSPEED;
//	angleJS = 6;
	HAL_GPIO_WritePin(ENA2_GPIO_Port,ENA2_Pin|ENB2_Pin,GPIO_PIN_RESET);
	for(;;)
	{
//		
//		if(HAL_GPIO_ReadPin(HW_1_GPIO_Port,HW_1_Pin)==GPIO_PIN_RESET)
//		{
//		HAL_GPIO_WritePin(ENA2_GPIO_Port,ENA2_Pin|ENB2_Pin,GPIO_PIN_SET);
//		for (uint16_t j=0; j<2; j++) 
//	  {
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|D_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin|B_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency1);		  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,D_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin|C_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency1);			  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,D_Pin|C_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency1);		  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency1);			  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|C_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency1);		  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|C_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency1);	
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|A_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency1);	
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|C_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency1);	
//	  }
//	  for (uint16_t j=0; j<43-2; j++) //0.703125 * 42
//	  {
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|D_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin|B_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency);			  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,D_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin|C_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency);			  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,D_Pin|C_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency);			  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency);			  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|C_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency);			  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|C_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|A_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency);			  
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin,GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|C_Pin|D_Pin,GPIO_PIN_RESET);
//		  osDelayUntil(&xLastWakeTime, xFrequency);
//	  }
//	  HAL_GPIO_WritePin(ENA2_GPIO_Port,ENA2_Pin|ENB2_Pin,GPIO_PIN_RESET);
//     }
		 if(HAL_GPIO_ReadPin(HW_1_GPIO_Port,HW_1_Pin) == 1)
		{
			osDelayUntil(&xLastWakeTime, xFrequency4);
			if(HAL_GPIO_ReadPin(HW_1_GPIO_Port,HW_1_Pin) == 1)
			{
				 if(swdj_1 == 0)
				 {
					swdj_1 = 1;
					 bzbjs_1++;
					 swjs_1 = 0;
					 swjs_3 = 0; 	 
				 }
							
			}
			if(swjs_3>20) bzbjs_1 = 0;
			else swjs_3++;	
		}
		else
		{
			swdj_1 = 0;			
			if(swjs_1>80) bzbjs_1 = 0;
			else swjs_1++;				
		}
     if(angleJS >= 1)
	  {
		  while(angleJS >= 10)
		  {
			  osDelayUntil(&xLastWakeTime, xFrequency3);
		  }
			if(bzbjs_1 == 3)
			{
				bzbjs_1 = 0;
				vTaskSuspend(main_1Handle);
				vTaskSuspend(moto_jzHandle);
				vTaskSuspend(moto_controlHandle);
				moto_stop();			
				HAL_GPIO_WritePin(ENA2_GPIO_Port,ENA2_Pin|ENB2_Pin,GPIO_PIN_SET);
				for (uint16_t j=0; j<2; j++) 
			  {
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|D_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin|B_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency1);		  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,D_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin|C_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency1);			  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,D_Pin|C_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency1);		  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency1);			  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|C_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency1);		  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|C_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency1);	
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|A_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency1);	
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|C_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency1);	
			  }
			  for (uint16_t j=0; j<43-2; j++) //0.703125 * 42
			  {
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|D_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin|B_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency);			  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,D_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin|C_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency);			  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,D_Pin|C_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency);			  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|B_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency);			  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|C_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency);			  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin|C_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|A_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,C_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency);			  
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,A_Pin,GPIO_PIN_SET);
				  HAL_GPIO_WritePin(ENA2_GPIO_Port,B_Pin|C_Pin|D_Pin,GPIO_PIN_RESET);
				  osDelayUntil(&xLastWakeTime, xFrequency);
			  }
			  HAL_GPIO_WritePin(ENA2_GPIO_Port,ENA2_Pin|ENB2_Pin,GPIO_PIN_RESET);
			  
            setSPEED = 9999;
			   speed = setSPEED;
				vTaskResume(main_1Handle);
			   vTaskResume(moto_jzHandle);				
				vTaskResume(moto_controlHandle);
				osDelayUntil(&xLastWakeTime, xFrequency3);
			}
			switch(bzbjs_1)
			{
				case 0 :
				    setSPEED = 9999;
				    speed = setSPEED;
				break;
				case 1 :
			      setSPEED = 7000;
				   speed = setSPEED;
				break;
				case 2:
					setSPEED = 5000;
				   speed = setSPEED;
				break;
				case 3:
					setSPEED = 0;
				   speed = setSPEED;
				break;
				default:break;
			}
		
		
	}
   	osDelayUntil(&xLastWakeTime, xFrequency2);
	}
}





#define wheelPerimer (17.907078125f) // 轮子周长
#define wheelPixwork (0.020348952414f) // 每个脉冲走多少
#define wheelMaxSpeed (80) // 最高速度 80cm/s
CCMRAM float outpida,outpidb;
uint8_t debuga= 0,debugb;
/* A轮PID控制函数 */
static uint32_t SpeedACtrl(arm_pid_instance_f32 *PID, int32_t setspeedA ,float spd)
{
	//int8_t px = ABS(setspeedA);
	//static int32_t setspeedA_old;
	static uint32_t time;
	static float outpida_t;
	static int32_t outpidint,outpidintold; // 整形判断
	arm_abs_f32(&outpida,&outpida_t,1);
	if(outpida_t > 800 /*|| (setspeedA_old/setspeedA)<0*/) // 判断电机正反转 参数变化过大 参数复位
	{
		//setspeedA_old = setspeedA;
		outpida = 0;
		arm_pid_reset_f32(PID);
		return 0;
	}
	//else if(px > px_o && px - px_o >20) px = 0.5*px; // 设置参数变化过大 调节设置参数
	//else setspeedA_old = setspeedA;
	outpida = arm_pid_f32(PID,(LIMIT(setspeedA,-80,80) - spd)); ////动态参数 *(-0.01875f*px+2.6875f) //
	
	
	outpidint = outpida;
	if(debuga == 1) // 关闭电机输出
	{
		//debug = 0;
		//cmd.send.SpeedA = 0;
	}
	else if(debuga == 2)  // 复位PID算法
	{
		arm_pid_init_f32(PID, 1);
		debuga = 0;
	}
	else
	{
		//if(!setspeedA) cmd.send.SpeedA = 0;
			if(outpidint < 0 && outpidintold == 0) 
			{
				outpidintold = 1;
				time = osKernelSysTick();
				HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
				//else if(outpidint
				//arm_abs_f32(&outpida,&outpida,1);
				//cmd.send.SpeedA = LIMIT(outpida,-100,10);
			}
			
			else if(outpidint > 0 && outpidintold == 0)
			{
				outpidintold = 2;
				time = osKernelSysTick();
				HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
				//cmd.send.SpeedA = LIMIT(outpida,-10,100);
			}
			else 
			{		
					if(osKernelSysTick() > time + 120) outpidintold = 0; // 大于120ms 允许操作
					if(outpidint == 0 || (outpidintold == 1 && outpidint > 0 ) || (outpidintold == 2 && outpidint < 0 ))  // 反转时刹车120ms
					{
						//if(outpidint == 0) outpidintold = 0;
						if(outpidint != 0)arm_pid_reset_f32(PID); // 复位pid
						HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_RESET);
					}
			}
		
		//outpidintold = outpidint;
		//arm_abs_f32(&outpida,&outpida,1);
		return LIMIT((uint32_t)outpida_t,0,100);
	}
	return 0;
}
/* B轮PID控制函数 */
static uint32_t SpeedBCtrl(arm_pid_instance_f32 *PID, int8_t setspeedB ,float spd)
{
	//int8_t px = ABS(setspeedB);
	//static int8_t setspeedB_old;
	static uint32_t time;
	float outpidb_t;
	static int32_t outpidint,outpidintold; // 整形判断
	
	arm_abs_f32(&outpidb,&outpidb_t,1);//|| setspeedA == 0
	
	if(/*(setspeedB_old/setspeedB) < 0  ||*/ outpidb_t > 800) // 判断电机正反转 参数变化过大 参数复位
	{
		//setspeedB_old = setspeedB;
		outpidb = 0;
		arm_pid_reset_f32(PID);
		return 0;
	}
	//else setspeedB_old = setspeedB;
	outpidb = arm_pid_f32(PID,(LIMIT(setspeedB,-80,80) - spd)); ////动态参数 *(-0.01875f*px+2.6875f) //
	outpidint = outpidb;
	if(debugb == 1) // 关闭电机输出
	{
		//debug = 0;
		//cmd.send.SpeedA = 0;
	}
	else if(debugb == 2)  // 复位PID算法
	{
		arm_pid_init_f32(PID, 1);
		debugb = 0;
	}
	else
	{
		if(outpidint < 0 && outpidintold == 0) 
		{
			outpidintold = 1;
			time = osKernelSysTick();
			HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
		}
		
		else if(outpidint> 0 && outpidintold == 0)
		{
			outpidintold = 2;
			time = osKernelSysTick();
			HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
			//cmd.send.SpeedA = LIMIT(outpida,-10,100);
		}
		else 
		{
			if(osKernelSysTick() > time + 120) outpidintold = 0; // 大于120ms 允许操作
			if(outpidint == 0 || (outpidintold == 1 && outpidint > 0 ) || (outpidintold == 2 && outpidint < 0 ))  // 反转时刹车120ms
			{
				//if(outpidint == 0) outpidintold = 0;
				if(outpidint != 0) arm_pid_reset_f32(PID); // 复位pid
				HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_RESET);
			}

		}
		///arm_abs_f32(&outpida,&outpida,1);
		return LIMIT((uint32_t)outpidb_t,0,100);
	}
	return 0;
}
void moto_controltask(void const * argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;
	
	int32_t SPA_temp,SPB_temp;
	uint32_t SPDA_t,SPDB_t;
	int32_t spdt;
	HAL_GPIO_WritePin(N1_GPIO_Port,N1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N2_GPIO_Port,N2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N3_GPIO_Port,N3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(N4_GPIO_Port,N4_Pin,GPIO_PIN_SET);
	
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	
	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,50);
	__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,50);
	//while(1);
	//moto_front();
	setSPA = 0;
	setSPB = 0;
	SPDA = 0;
	SPDB = 0;
	SPDA_t = __HAL_TIM_GET_COUNTER(&htim8);
	SPDB_t = __HAL_TIM_GET_COUNTER(&htim4);
	/* PID 参数初始化 */ 
	SPDA_PID.Kp = 1.05f;
	SPDA_PID.Ki = 0.265f;
	SPDA_PID.Kd = 0.83f;
	
	SPDB_PID.Kp = 1.05f;
	SPDB_PID.Ki = 0.245f;
	SPDB_PID.Kd = 0.83f;
	
	arm_pid_init_f32(&SPDA_PID,1);
	arm_pid_init_f32(&SPDB_PID,1);
	osDelay(200);
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		//轮子一圈880个脉冲 轮子直径 5.7cm 周长 17.907078125CM 一个脉冲走 0.020348952414CM
		
		osDelayUntil(&xLastWakeTime, xFrequency);
		ReadDisA = __HAL_TIM_GET_COUNTER(&htim8); //记录路程
		ReadDisB = __HAL_TIM_GET_COUNTER(&htim4); //
		
		spdt = __HAL_TIM_GET_COUNTER(&htim8) - SPDA_t;
		if(spdt > 10000) // 反转
			SPDA = spdt - 65535;
		else if(spdt < -10000) //正转
			SPDA = 65535 + spdt;
		else
			SPDA = spdt;
		
		spdt = __HAL_TIM_GET_COUNTER(&htim4) - SPDB_t;
		if(spdt > 10000) // 反转
			SPDB = spdt - 65535;
		else if(spdt < -10000) //正转
			SPDB = 65535 + spdt;
		else
			SPDB = spdt;		
		
		SPDA_t = __HAL_TIM_GET_COUNTER(&htim8); //保存当前计数
		SPDB_t = __HAL_TIM_GET_COUNTER(&htim4);
		
		disA_CM = ReadDisA*wheelPixwork;
		disB_CM = ReadDisB*wheelPixwork;
		
		SPDA_CM = SPDA*wheelPixwork*(1000.f/xFrequency); // 转厘米 // 单位cm/s 0 ~ 85 cm/s
		SPDB_CM = SPDB*wheelPixwork*(1000.f/xFrequency);
		
		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_1,SpeedACtrl(&SPDA_PID,setSPA,SPDA_CM));
		__HAL_TIM_SetCompare(&htim9,TIM_CHANNEL_2,SpeedBCtrl(&SPDB_PID,setSPB,SPDB_CM));		
	}
}
