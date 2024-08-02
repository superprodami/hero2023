#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

//#include "protect_task.h"
#include "arm_math.h"
#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//浮点矩阵转置
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64
#define mat_scale   arm_mat_scale_f32


typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, B, H, AT, HT, KT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4], KT_data[4];
  float A_data[4];
	float B_data[2];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;


void   kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2, float signal3);
float Kalman_Filter_IMU(const float newGyro,const float newAngle);

//
typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
	  float B;
    float Q;
    float R;
    float H;
}extKalman_t;

void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float KalmanFilter(extKalman_t* p,float dat);
//

#endif
