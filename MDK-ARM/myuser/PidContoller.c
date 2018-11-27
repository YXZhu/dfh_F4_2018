#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "pidcontoller.h"



//����ʽPID�㷨
void PID_AbsoluteMode(PID_AbsoluteType* PID)
{
 if(PID->kp      < 0)    PID->kp      = -PID->kp;
 if(PID->ki      < 0)    PID->ki      = -PID->ki;
 if(PID->kd      < 0)    PID->kd      = -PID->kd;
 if(PID->errILim < 0)    PID->errILim = -PID->errILim;

 PID->errP = PID->errNow;  //��ȡ���ڵ�������kp����

 PID->errI += PID->errNow; //�����֣�����ki����

 if(PID->errILim != 0)	   //΢�����޺�����
 {
  if(     PID->errI >  PID->errILim)    PID->errI =  PID->errILim;
  else if(PID->errI < -PID->errILim)    PID->errI = -PID->errILim;
 }
 
 PID->errD = PID->errNow - PID->errOld;//���΢�֣�����kd����

 PID->errOld = PID->errNow;	//�������ڵ����
 
 PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//�������ʽPID���

}


/*******************************************************************************************************/



//����ʽPID�㷨
void PID_IncrementMode(PID_IncrementType* PID)
{
 float dErrP, dErrI, dErrD;
 
 if(PID->kp < 0)    PID->kp = -PID->kp;
 if(PID->ki < 0)	PID->ki = -PID->ki;
 if(PID->kd < 0)    PID->kd = -PID->kd;

 dErrP = PID->errNow - PID->errOld1;

 dErrI = PID->errNow;

 dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;

 PID->errOld2 = PID->errOld1; //�������΢��
 PID->errOld1 = PID->errNow;  //һ�����΢��

 /*����ʽPID����*/
 PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
 
 if(PID->kp == 0 && PID->ki == 0 && PID->kd == 0)   PID->ctrOut = 0;

 else PID->ctrOut += PID->dCtrOut;
}


/*****************************************����ٶȻ��ŷ�***********************************************/

//�ٶ�
PID_AbsoluteType PID_ControlA;//����PID�㷨�Ľṹ��
PID_AbsoluteType PID_ControlB;//����PID�㷨�Ľṹ��

//�������
PID_AbsoluteType PID_ControlJL1;
PID_AbsoluteType PID_ControlJL2;

int32_t User_PidSpeedControlA(int32_t SpeedTag,int32_t spnow)
{

   PID_ControlA.errNow = SpeedTag - spnow; //���㲢д���ٶ����
   	
   PID_ControlA.kp      = 2.22;             //д�����ϵ��Ϊ15
   PID_ControlA.ki      = 0.785;              //д�����ϵ��Ϊ5
   PID_ControlA.kd      = 1.196;              //д��΢��ϵ��Ϊ5
   PID_ControlA.errILim = 1000;           //д������������Ϊ1000 ����Ϊ-1000

   PID_AbsoluteMode(&PID_ControlA);       //ִ�о���ʽPID�㷨
	
   return  PID_ControlA.ctrOut;         //��ȡ����ֵ

}
//extern uint8_t aa[2];
int32_t User_PidSpeedControlB(int32_t SpeedTag,int32_t spnow)
{
//	PID_ControlB.kp      = 2.22;             //д�����ϵ��Ϊ15 2.58
//   PID_ControlB.ki      = 0.785;              //д�����ϵ��Ϊ5
//   PID_ControlB.kd      = 1.196; 
//  PID_ControlB.kp      = 2.78;             //д�����ϵ��Ϊ15

//   PID_ControlB.ki      = 1.26;              //д�����ϵ��Ϊ5
//   PID_ControlB.kd      = 0.5;              //д��΢��ϵ��Ϊ5
//  PID_ControlB.errILim = 1000; 
   PID_ControlB.errNow = SpeedTag - spnow; //���㲢д���ٶ����
   	
   PID_ControlB.kp      = 2.22;             //д�����ϵ��Ϊ15 2.58
   PID_ControlB.ki      = 0.785;              //д�����ϵ��Ϊ5
   PID_ControlB.kd      = 1.196;              //д��΢��ϵ��Ϊ5
   PID_ControlB.errILim = 1000;           //д������������Ϊ1000 ����Ϊ-1000

   PID_AbsoluteMode(&PID_ControlB);       //ִ�о���ʽPID�㷨
	
   return  PID_ControlB.ctrOut;         //��ȡ����ֵ

}

int32_t User_PidSpeedControlJL1(int32_t SpeedTag,int32_t spnow)
{

   PID_ControlJL1.errNow = SpeedTag - spnow; //���㲢д���ٶ����
   	
   PID_ControlJL1.kp      = 10;             //д�����ϵ��Ϊ15
   PID_ControlJL1.ki      = 5;              //д�����ϵ��Ϊ5
   PID_ControlJL1.kd      = 0;              //д��΢��ϵ��Ϊ5
   PID_ControlJL1.errILim = 2;           //д������������Ϊ1000 ����Ϊ-1000

   PID_AbsoluteMode(&PID_ControlJL1);       //ִ�о���ʽPID�㷨
	
   return  PID_ControlJL1.ctrOut;         //��ȡ����ֵ

}

int32_t User_PidSpeedControlJL2(int32_t SpeedTag,int32_t spnow)
{

   PID_ControlJL2.errNow = SpeedTag - spnow; //���㲢д���ٶ����
   	
   PID_ControlJL2.kp      = 10;             //д�����ϵ��Ϊ15
   PID_ControlJL2.ki      = 5;              //д�����ϵ��Ϊ5
   PID_ControlJL2.kd      = 0;              //д��΢��ϵ��Ϊ5
   PID_ControlJL2.errILim = 2;           //д������������Ϊ1000 ����Ϊ-1000

   PID_AbsoluteMode(&PID_ControlJL2);       //ִ�о���ʽPID�㷨
	
   return  PID_ControlJL2.ctrOut;         //��ȡ����ֵ

}


