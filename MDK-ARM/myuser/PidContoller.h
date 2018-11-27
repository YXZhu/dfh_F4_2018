#ifndef __FUNTION_H
#define __FUNTION_H

typedef enum {Speed = 0, Position = !Speed} PidControlMode;//ѡ���ٶ�ģʽ��λ��ģʽ

/*����ʽPID�㷨���ӿڲ����ṹ����*/
typedef struct 
{
 /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 float kp;     //����ϵ��
 float ki;     //����ϵ��
 float kd;     //΢��ϵ��
 float errILim;//����������
 
 float errNow;//��ǰ�����
 float ctrOut;//���������
 
 /*PID�㷨�ڲ���������ֵ�����޸�*/
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;


/*����ʽPID�㷨���ӿڲ����ṹ����*/
typedef struct 
{
 /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 float kp;     //����ϵ��
 float ki;     //����ϵ��
 float kd;     //΢��ϵ��
 
 float errNow; //��ǰ�����
 float dCtrOut;//�����������
 float  ctrOut;//�������
 
 /*PID�㷨�ڲ���������ֵ�����޸�*/
 float errOld1;
 float errOld2;
 
}PID_IncrementType;


extern int32_t User_PidSpeedControlA(int32_t SpeedTag,int32_t spnow);
extern int32_t User_PidSpeedControlB(int32_t SpeedTag,int32_t spnow);
int32_t User_PidSpeedControlJL2(int32_t SpeedTag,int32_t spnow);
int32_t User_PidSpeedControlJL1(int32_t SpeedTag,int32_t spnow);
//extern void User_PidPositionControl(s32 Position,s32 SpeedTag);

#endif







