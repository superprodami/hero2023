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

/********************����ģʽ****************************/
static void GIMBAL_Mode_Choose(void);  ///��̨����ģʽѡ��,������Ӧ/
static void GIMBAL_Key_Ctrl(void);     ///���̿�����̨ģʽ




/*****************************��̨λ��PID����***********************************/
static void GIMBAL_InitArgument(void);  //��̨������ʼ��

/***********��̨����ģʽ����ģʽС����*******************/
static void GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl(void); //���̸�����̨����
static void GIMBAL_AUTO_Mode_Ctrl(void);  ///������ƺ���
static void GIMBAL_Double_Loop_Out(void);  //��̨������
static void kalman_filter_change_realtime(void);


#endif
