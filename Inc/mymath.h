#ifndef __mymath_H
#define __mymath_H

typedef struct
{
	float x_last;
	float p_last;
	float ProcessNiose_Q;
	float MeasureNoise_R;
}KalmanFilterValue_t;

float KalmanFilter(KalmanFilterValue_t *KalmanFilterValue,const float ResrcData);

#endif
