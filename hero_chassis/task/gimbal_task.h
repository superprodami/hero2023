#ifndef GIMBALTASKH
#define GIMBALTASKH
#include "main.h"
#include "struct_typedef.h"

//#define RADIO 72.012f
//#define YAW 0
//#define PITCH 1

//#define MECH 0
//#define GYRO 1

#define NOW  0
#define LAST 1

#define KP 0
#define KI 1
#define KD 2
#define OUTER 0
#define INNER 1

//extern float IMU_angle[3];

//extern uint8_t auto_mode;
/* 云台操作模式:

   普通             	NORMAL
   调头180°             AROUND
   打符             	BUFF
   补弹,pitch水平   	LEVEL
   机械模式pitch抬头	HIGH
   快速扭头90°          TURN
*/
//typedef enum
//{
//    GIMBAL_NORMAL            = 0,//正常模式,进行模式选择
//    GIMBAL_TURN_RIGHT        = 1,//右转90°调头
//    GIMBAL_CHASSIS_FOLLOW    = 2,//底盘跟随云台
//    GIMBAL_LEVEL             = 3,//弹仓开启,云台水平
//    GIMBAL_MANUAL            = 4,//手动打符模式
//    GIMBAL_SM_BUFF           = 5,//小符
//    GIMBAL_TURN_LEFT         = 7,//左转90°扭头
//    GIMBAL_AUTO              = 8,//自瞄
//    GIMBAL_BASE              = 9,//桥头吊射基地
//    GIMBAL_BUFF              = 10,//打符模式,大
//    GIMBAL_GYROSCOPE         = 11,//小陀螺
//} eGimbalAction;
//extern volatile eGimbalAction  actGimbal;

//typedef struct  //视觉目标速度测量
//{
//    int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
//    int freq;
//    int last_time;//上次受到目标角度的时间
//    float last_position;//上个目标角度
//    float speed;//速度
//    float last_speed;//上次速度
//    float processed_speed;//速度计算结果
//} speed_calc_data_t;

typedef enum
{
    USEENCODER,
    USEIMU
} GimbalModeType;

//typedef enum
//{
//    GIMBAL_HEAD,
//	GIMBAL_TAIL,
//} Gimbal_Current_Follow;


//typedef enum
//{
//    GIMBAL_HORIZON,  //水平
//	GIMBAL_VERTICAL,  //竖直
//} Gimbal_Hanging_Status;
//extern Gimbal_Hanging_Status gimbal_hanging;

//extern volatile eGimbalAction  actGimbal;
extern GimbalModeType YawGimbalMode;
extern GimbalModeType PitchGimbalMode;
//extern Gimbal_Current_Follow gimbal_follow;

static void Gimbal_Open_Init(void);
static void RemoteControlGimbal(void);

/********************键盘模式****************************/
static void GetEnvironmentGimbalMode(void);	//设置车辆所处环境
static void GIMBAL_Mode_Choose(void);  ///云台键盘模式选择,按键响应/
static void GIMBAL_Key_Ctrl(void);     ///键盘控制云台模式

/*****************************云台位置PID控制***********************************/
static void GIMBAL_InitArgument(void);  //云台参数初始化

/***********云台键盘模式各类模式小函数*******************/
static void GIMBAL_NO_CHASSIS_FOLLOW_Mode_Ctrl(void);
static void GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl(void); //底盘跟随云台函数
static void GIMBAL_GYROSCOPE_Mode_Ctrl(void);   //小陀螺模式
static void GIMBAL_LEVEL_Mode_Ctrl(void);  ///补弹模式/
static void GIMBAL_AUTO_Mode_Ctrl(void);  ///自瞄控制函数
//			void GIMBAL_BASE_Mode_Ctrl(void);  ///桥头吊射模式
/******************************************************************/
//static void GIMBAL_State_Change(GimbalModeType Type);
extern void GIMBAL_State_Change(GimbalModeType Type);
static void GIMBAL_Double_Loop_Out(void);  //云台电机输出
static void kalman_filter_change_realtime(void);
extern float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro
extern void PID_Change(fp32 *Kpid_Angle ,fp32 *Kpid_speed);
#endif



