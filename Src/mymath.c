#include "mymath.h"

/*-------------------------------------------------------------------------------------------------------------*/
/*       
		һ�׿������˲�
        Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
        R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��       
*/

float KalmanFilter(KalmanFilterValue_t * KalmanFilterValue,const float ResrcData)
//float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
        //float R = MeasureNoise_R;
        //float Q = ProcessNiose_Q;

        //static float x_last;

        float x_mid = KalmanFilterValue->x_last;
        float x_now;

        //static float p_last;

        float p_mid ;
        float p_now;
        float kg;       

        x_mid=KalmanFilterValue->x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=KalmanFilterValue->p_last+KalmanFilterValue->ProcessNiose_Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
        kg=p_mid/(p_mid+KalmanFilterValue->MeasureNoise_R); //kgΪkalman filter��RΪ����
        x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
               
        p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       

        KalmanFilterValue->p_last = p_now; //����covarianceֵ
        KalmanFilterValue->x_last = x_now; //����ϵͳ״ֵ̬

        return x_now;               
}

/*-------------------------------------------------------------------------------------------------------------*/

