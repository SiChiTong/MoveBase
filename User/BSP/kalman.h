#ifndef _KALMAN_H
#define _KALMAN_H

typedef struct
{
	float x;//系统状态量
	float A;//x(n) = A*x(n-1)+u(n), u(n)~N(0,q)
	float H;//z(n) = H*x(n) + w(n),w(n)~N(0,r)
	float q;//预测过程噪声协方差
	float r;//测量过程噪声协方差
	float p;//估计误差协方差
	float gain;//卡尔曼增益
}KalmanStruct;

extern KalmanStruct l_kalmanParam;
extern KalmanStruct r_kalmanParam;

void KalmanInit(KalmanStruct* kalmanFilter,float init_x,float init_p,float predict_q,float newMeasured_q);
float KalmanFilter(KalmanStruct* kalmanFilter,float newMeasured);

#endif

