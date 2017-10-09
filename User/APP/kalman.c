#include "kalman.h"

KalmanStruct l_kalmanParam;
KalmanStruct r_kalmanParam;

void KalmanInit(KalmanStruct* kalmanFilter,float init_x,float init_p,float predict_q,float newMeasured_q)
{
	kalmanFilter->x = init_x;
	kalmanFilter->p = init_p;
	kalmanFilter->A = 1;
	kalmanFilter->H = 1;
	kalmanFilter->q = predict_q;
	kalmanFilter->r = newMeasured_q;
}

float KalmanFilter(KalmanStruct* kalmanFilter,float newMeasured)
{
	/*predict*/
	kalmanFilter->x = kalmanFilter->A * kalmanFilter->x;
	kalmanFilter->p = kalmanFilter->A * kalmanFilter->A*kalmanFilter->p + kalmanFilter->q;
	
	/*correct*/
	kalmanFilter->gain = kalmanFilter->p * kalmanFilter->H /(kalmanFilter->p*kalmanFilter->H*kalmanFilter->H + kalmanFilter->r);
//	kalmanFilter->x = kalmanFilter->x + kalmanFilter->gain*(newMeasured - kalmanFilter->H*kalmanFilter->x);
	kalmanFilter->x = kalmanFilter->gain*(newMeasured - kalmanFilter->H*kalmanFilter->x); //only delat x
	kalmanFilter->p = (1 - kalmanFilter->gain*kalmanFilter->H)*kalmanFilter->p;
	
	return kalmanFilter->x;
}

