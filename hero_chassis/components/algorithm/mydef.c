#ifndef __MYDEF

#define __MYDEF

#include "mydef.h"
//main

int16_t see,testfbd,testset,set,fellowfdb,fellowset,shootspeedfdb,shootspeedset;
double shootangleset,shootanglefdb;
fp32 abstotalangleinit;

TEST_RX_MSG CAN2_RXTEST;

volatile SYSTEMVALUE SystemValue = Starting;  //����״̬
volatile EnvironmentModeType EnvironmentMode = NOMAL;  //��������
volatile ControlModeType ControlMode = KEYBOARD; //REMOTE;//KEYBOARD;  //���Ʒ�ʽ

uint32_t control_judge_flag = 0;
uint8_t state_judge = 0;
volatile bool imu_init_finish_flag = 0;

// chassis
volatile eChassisAction actChassis = CHASSIS_NORMAL; //Ĭ�ϵ��̲�������̨����
volatile eChassisAction KeyboardactChassis = CAHSSIS_ZERO ;
eChassisAction actChassis_last = CHASSIS_NORMAL;


//gimbal
volatile Gimbal_Current_Follow gimbal_follow;
volatile eGimbalAction actGimbal = GIMBAL_NORMAL;       ////��̨����ģʽ///
Gimbal_Hanging_Status gimbal_hanging;
float IMU_angle[3] = {0.0f, 0.0f, 0.0f};
//shoot
eShootState ShootState = UNSTART;
Mocalun_Status mocalun_status;
int shoot_speed_adjust = 0;
flag_t Flag_status;//����flag ״̬λ



#endif
