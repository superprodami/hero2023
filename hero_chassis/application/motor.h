#ifndef bsp_motor_h
#define bsp_motor_h

#include "main.h"
#include "pid.h"
#include "type.h"

extern Motortype Chassis_Motor[4];

extern Motortype Ammunition_Motor;
extern Motortype Ammunition_DM_Motor;

//extern Motortype mocalun_l;
//extern Motortype mocalun_r;

extern Motortype Gimbal_MotorImuYaw;
extern Motortype Gimbal_MotorEncoderYaw;

extern Motortype Gimbal_MotorPitch;
void Motor_Init(Motortype *motor, int ID, const float pid1[3], PID_MODE mode1, const float outmax1, const float imax1, float I_Separation1, float Dead_Zone1, float gama1, int angle_max1, int angle_min1,
                const float pid2[3], PID_MODE mode2,const float outmax2, const float imax2, float I_Separation2, float Dead_Zone2, float gama2, int angle_max2, int angle_min2);
void Motor_Init2(Motortype *motor, int ID, const float pid1[3], PID_MODE mode1, const float outmax1, const float imax1, float I_Separation1, float Dead_Zone1, float gama1, int angle_max1, int angle_min1,
                 const float pid2[3], PID_MODE mode2,const float outmax2, const float imax2, float I_Separation2, float Dead_Zone2, float gama2, int angle_max2, int angle_min2);
void Motor_Init_DM(Motortype *motor, int ID, const float pid1[3], PID_MODE mode1, const float outmax1, const float imax1, float I_Separation1, float Dead_Zone1, float gama1, int angle_max1, int angle_min1,
                const float pid2[3], PID_MODE mode2,const float outmax2, const float imax2, float I_Separation2, float Dead_Zone2, float gama2, int angle_max2, int angle_min2);

#endif
