#include "mymath.h"

/*-------------------------------------------------------------------------------------------------------------*/
/*       
		一阶卡尔曼滤波
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好       
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
        p_mid=KalmanFilterValue->p_last+KalmanFilterValue->ProcessNiose_Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
        kg=p_mid/(p_mid+KalmanFilterValue->MeasureNoise_R); //kg为kalman filter，R为噪声
        x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
               
        p_now=(1-kg)*p_mid;//最优值对应的covariance       

        KalmanFilterValue->p_last = p_now; //更新covariance值
        KalmanFilterValue->x_last = x_now; //更新系统状态值

        return x_now;               
}

/*-------------------------------------------------------------------------------------------------------------*/

