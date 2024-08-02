#ifndef __MYDEF

#define __MYDEF

#include "mydef.h"
//main

int16_t see,testfbd,testset,set,fellowfdb,fellowset,shootspeedfdb,shootspeedset;
double shootangleset,shootanglefdb;
fp32 abstotalangleinit;

TEST_RX_MSG CAN2_RXTEST;

volatile SYSTEMVALUE SystemValue = Starting;  //程序状态
volatile EnvironmentModeType EnvironmentMode = NOMAL;  //所处环境
volatile ControlModeType ControlMode = KEYBOARD; //REMOTE;//KEYBOARD;  //控制方式

uint32_t control_judge_flag = 0;
uint8_t state_judge = 0;
volatile bool imu_init_finish_flag = 0;

// chassis
volatile eChassisAction actChassis = CHASSIS_NORMAL; //默认底盘不跟随云台行走
volatile eChassisAction KeyboardactChassis = CAHSSIS_ZERO ;
eChassisAction actChassis_last = CHASSIS_NORMAL;


//gimbal
volatile Gimbal_Current_Follow gimbal_follow;
volatile eGimbalAction actGimbal = GIMBAL_NORMAL;       ////云台操作模式///
Gimbal_Hanging_Status gimbal_hanging;
float IMU_angle[3] = {0.0f, 0.0f, 0.0f};
//shoot
eShootState ShootState = UNSTART;
Mocalun_Status mocalun_status;
int shoot_speed_adjust = 0;
flag_t Flag_status;//各种flag 状态位



#endif
