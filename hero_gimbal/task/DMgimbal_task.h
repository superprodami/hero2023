#ifndef _DMGIMBAL_TASK_H
#define _DMGIMBAL_TASK_H
#include "main.h"

fp32 Gimbal_DM_MotorPitch_Position_pid[3];
fp32 Gimbal_DM_MotorPitch_Speed_pid[3];
fp32 Gimbal_DM_Pitch_imu_pid[3];



#define NOW  0
#define LAST 1

typedef enum
{
    USEENCODER,
    USEIMU
} GimbalModeType;

GimbalModeType PitchGimbalMode = USEIMU;
static void RemoteControlGimbal(void);

/********************键盘模式****************************/
static void GIMBAL_Mode_Choose(void);  ///云台键盘模式选择,按键响应/
static void GIMBAL_Key_Ctrl(void);     ///键盘控制云台模式




/*****************************云台位置PID控制***********************************/
static void GIMBAL_InitArgument(void);  //云台参数初始化

/***********云台键盘模式各类模式小函数*******************/
static void GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl(void); //底盘跟随云台函数
static void GIMBAL_AUTO_Mode_Ctrl(void);  ///自瞄控制函数
static void GIMBAL_Double_Loop_Out(void);  //云台电机输出
static void kalman_filter_change_realtime(void);


#endif
