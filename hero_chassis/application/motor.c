/*
 * Motor_Task.c
 *
 *  Created on: Nov 25, 2019
 *      Author: Tongw
 */
#include "motor.h"

Motortype Chassis_Motor[4];

Motortype Ammunition_Motor;
Motortype Ammunition_DM_Motor;
//Motortype mocalun_l;
//Motortype mocalun_r;

Motortype Gimbal_MotorImuYaw;
Motortype Gimbal_MotorEncoderYaw;

Motortype Gimbal_MotorPitch;




void Motor_Init(Motortype *motor, int ID, const float pid1[3], PID_MODE mode1, const float outmax1, const float imax1, float I_Separation1, float Dead_Zone1, float gama1, int angle_max1, int angle_min1,
                const float pid2[3], PID_MODE mode2,const float outmax2, const float imax2, float I_Separation2, float Dead_Zone2, float gama2, int angle_max2, int angle_min2)
{
    motor->ID = ID;
    motor->motor_value = &moto_CAN[ID - 1];

    pid_init(&(motor->Motor_PID_Position));
    pid_init(&(motor->Motor_PID_Speed));
    //max_out    max_iout   I_Separation   Dead_Zone     gama    angle_max   angle_min
    (motor->Motor_PID_Position).f_param_init(&(motor->Motor_PID_Position), mode1, pid1, outmax1, imax1, I_Separation1, Dead_Zone1, gama1, angle_max1, angle_min1 );
    (motor->Motor_PID_Speed).f_param_init(&(motor->Motor_PID_Speed),       mode2, pid2, outmax2, imax2, I_Separation2, Dead_Zone2, gama2, angle_max2, angle_min2 );
}

void Motor_Init2(Motortype *motor, int ID, const float pid1[3], PID_MODE mode1, const float outmax1, const float imax1, float I_Separation1, float Dead_Zone1, float gama1, int angle_max1, int angle_min1,
                 const float pid2[3], PID_MODE mode2,const float outmax2, const float imax2, float I_Separation2, float Dead_Zone2, float gama2, int angle_max2, int angle_min2)
{
    motor->ID = ID;
    motor->motor_value = &moto_CAN2[ID - 1];

    pid_init(&(motor->Motor_PID_Position));
    pid_init(&(motor->Motor_PID_Speed));
    //max_out    max_iout   I_Separation   Dead_Zone  gama    angle_max    angle_min
    (motor->Motor_PID_Position).f_param_init(&(motor->Motor_PID_Position), mode1, pid1, outmax1, imax1, I_Separation1, Dead_Zone1, gama1, angle_max1, angle_min1 );
    (motor->Motor_PID_Speed).f_param_init(&(motor->Motor_PID_Speed),       mode2, pid2, outmax2, imax2, I_Separation2, Dead_Zone2, gama2, angle_max2, angle_min2 );
}

void Motor_Init_DM(Motortype *motor, int ID, const float pid1[3], PID_MODE mode1, const float outmax1, const float imax1, float I_Separation1, float Dead_Zone1, float gama1, int angle_max1, int angle_min1,
                const float pid2[3], PID_MODE mode2,const float outmax2, const float imax2, float I_Separation2, float Dead_Zone2, float gama2, int angle_max2, int angle_min2)
{
    motor->ID = ID;
    motor->motor_value = &moto_CAN_DM[ID - 1];
    pid_init(&(motor->Motor_PID_Position));
    pid_init(&(motor->Motor_PID_Speed));
    //max_out    max_iout   I_Separation   Dead_Zone     gama    angle_max   angle_min
    (motor->Motor_PID_Position).f_param_init(&(motor->Motor_PID_Position), mode1, pid1, outmax1, imax1, I_Separation1, Dead_Zone1, gama1, angle_max1, angle_min1 );
    (motor->Motor_PID_Speed).f_param_init(&(motor->Motor_PID_Speed),       mode2, pid2, outmax2, imax2, I_Separation2, Dead_Zone2, gama2, angle_max2, angle_min2 );
}
