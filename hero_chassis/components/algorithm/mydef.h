#ifndef _MYDEF
#define _MYDEF
//#include "type.h"
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"


extern int16_t see,testfbd,testset,set,fellowfdb,fellowset,shootspeedfdb,shootspeedset;
extern double shootangleset,shootanglefdb;

extern fp32 abstotalangleinit;

typedef uint8_t 	u8;
typedef uint16_t 	u16;
typedef uint32_t 	u32;

typedef int8_t 		s8;
typedef int16_t 	s16;
typedef int32_t		s32;

typedef volatile uint8_t 	vu8;
typedef volatile uint16_t 	vu16;
typedef volatile uint32_t 	vu32;

typedef volatile int8_t 	vs8;
typedef volatile int16_t 	vs16;
typedef volatile int32_t	vs32;

//main
typedef enum
{
	NOMAL,		//普通模式
	CLIMBING,	//爬坡模式
}EnvironmentModeType;
typedef enum
{
	KEYBOARD,
	REMOTE,
	UNUSUAL,
}ControlModeType;
typedef enum
{
    Starting = 0,
    Running = 1,
} SYSTEMVALUE;
extern volatile ControlModeType ControlMode;
extern volatile EnvironmentModeType EnvironmentMode;
extern volatile SYSTEMVALUE SystemValue;	//程序状态
extern uint32_t control_judge_flag;
extern uint8_t state_judge;
extern volatile bool imu_init_finish_flag;

//chassis
typedef enum
{
    CHASSIS_FOLLOW_GIMBAL = 1,	//底盘跟随云盘行走
    CHASSIS_GYROSCOPE     = 2,  //小陀螺模式
    CHASSIS_NORMAL        = 3,  //底盘不跟随云台行走
    CHASSIS_CORGI         = 4,  //扭屁股模式
    CHASSIS_ROSHAN        = 5,  //打符模式
    CHASSIS_SLOW          = 6,  //补弹低速模式
    CHASSIS_SZUPUP        = 7,  //爬坡模式
    CHASSIS_MISS          = 8,  //自动闪避模式
    CHASSIS_FORTYFIVE     = 9,  //45°模式
	  CAHSSIS_ZERO          = 10,
	  
} eChassisAction;
extern volatile eChassisAction actChassis;
extern volatile eChassisAction KeyboardactChassis;
extern eChassisAction actChassis_last;

//gimbal
typedef enum
{
    GIMBAL_HEAD,
	GIMBAL_TAIL,
} Gimbal_Current_Follow;
extern volatile Gimbal_Current_Follow gimbal_follow;
typedef enum
{
    GIMBAL_NORMAL            = 0,//正常模式,进行模式选择
    GIMBAL_TURN_RIGHT        = 1,//右转90°调头
    GIMBAL_CHASSIS_FOLLOW    = 2,//底盘跟随云台
    GIMBAL_LEVEL             = 3,//弹仓开启,云台水平
    GIMBAL_MANUAL            = 4,//手动打符模式
    GIMBAL_SM_BUFF           = 5,//小符
    GIMBAL_TURN_LEFT         = 7,//左转90°扭头
    GIMBAL_AUTO              = 8,//自瞄
    GIMBAL_BASE              = 9,//桥头吊射基地
    GIMBAL_BUFF              = 10,//打符模式,大
    GIMBAL_GYROSCOPE         = 11,//小陀螺
} eGimbalAction;
extern volatile eGimbalAction  actGimbal;
typedef enum
{
    GIMBAL_HORIZON,  //水平
	GIMBAL_VERTICAL,  //竖直
} Gimbal_Hanging_Status;
extern Gimbal_Hanging_Status gimbal_hanging;
extern float IMU_angle[3];

//shoot
typedef enum  //摩擦轮状态
{
    UNSTART = 0,
    START = 1,
    FINISH = 2,
} eShootState;
extern eShootState ShootState;
typedef enum  //发射状态
{
    mNORMAL = 0,
    mUNUSUAL = 1,
} Mocalun_Status;
extern Mocalun_Status mocalun_status;
extern int shoot_speed_adjust;
//
typedef struct PidTypeDef
{
	  uint32_t flag_Slop;
		float Slop;

	
	
    float Dead_Zone; //误差死区阈值
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set; //设定值
    float fdb; //反馈值

    float out;
    float lastout;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
    int angle_max;
    int angle_min;	//角度相邻值 如在一个圆内，0°和360°相邻，则max=360，min=0
    //			在一个电机内 0和8192相邻，则max=8192，min=0
    float I_Separation; //积分分离阈值
    float gama;			//微分先行滤波系数
    float lastdout;		//上一次微分输出

    void ( *f_param_init)(struct PidTypeDef *pid,  //PID参数初始化
                          uint8_t mode,
                          const float PID[3],
                          float max_out,
                          float max_iout,
                          float I_Separation,
                          float Dead_Zone,
                          float gama,
                          int angle_max,
                          int angle_min
                         );

    float (*f_cal_pid)(struct PidTypeDef *pid, const float ref, const float set);   //pid计算
    void (*f_reset_pid)(struct PidTypeDef	*pid, float PID[3]);

} PidTypeDef;

typedef struct
{
    int16_t	 	speed_rpm;
    int16_t   target_speed_rpm;
    float  	  real_current;
    int16_t  	given_current;
    uint8_t  	hall;
		int32_t   main_angle;
    int16_t 	angle;				//abs angle range:[0,8191]
    float     target_angle;		//目标角度
    uint16_t 	last_angle;			//abs angle range:[0,8191]
    uint16_t	offset_angle;
    int16_t	  round_cnt;
    float  		total_angle;
    u8			  buf_idx;
    u16			  angle_buf[5];
    u16			  fited_angle;
    u32			  msg_cnt;

	  u16	  		id;
		u16 dm_error;
    float  		velocity,   //达妙电机 速度，位置，转矩
							position,
							torque,
							target_velocity,
							target_position;

    int16_t    	speed_dp10ms;		//机械角度/10ms   机械角度范围[0,8191]
    //转每10ms=speed_dp10ms/8192

		struct
		{
			uint16_t RX_Frequent;
			uint16_t RX_add;
		}RX_MSG;
} moto_measure_t;

typedef struct
{
	uint16_t RX_Frequent;
	uint16_t RX_add;
}TEST_RX_MSG;
extern TEST_RX_MSG CAN2_RXTEST;



typedef struct
{
    int ID;
    moto_measure_t *motor_value;
    PidTypeDef Motor_PID_Position;
    PidTypeDef Motor_PID_Speed;
	  

} Motortype;
typedef enum
{
    PID_POSITION = 0,//位置式PID
    PID_DELTA, //增量式PID
} PID_MODE;

//
typedef struct
{
//  uint8_t flag;
	bool shoot_single_flag;
  bool shoot_triple_flag;
	bool shoot_normal_flag;
	
	bool stuck_flag;
	bool heat_flag;

	bool rc_shoot_flag;  //遥控器单发打弹
	bool rc_wheel_flag;  //同上
		
  uint16_t shoot_cnt;
  uint32_t shoot_left_time;
  uint8_t shoot_single_finish_flag;
  uint32_t shoot_single_time;
  uint8_t Ammunition_flag; //遥控器模式使用，在开关不在相应位置上时代替开关发挥作用
  uint8_t moca_flag;       //作用同上
  bool protect_flag_sutck; //防卡弹模式保护，跳过模式选择 防止防卡弹放松被打断
  bool protect_flag_heat;  //作用同上
  
  bool FLAG_Remote;
  bool follow_flag_remote;
  bool FLAG_Key;
  bool follow_flag_key;
  bool chassis_follow_flag;
	
  bool Chassis_Switch_C;
  uint8_t Chassis_Key_C_Change;  
  bool Chassis_Switch_F;
  uint8_t Chassis_Key_F_Change;
  bool Chassis_Switch_X;
  uint8_t Chassis_Key_X_Change;
  bool Chassis_Switch_G;
  uint8_t Chassis_Key_G_Change;
  bool Chassis_Switch_Q;
  uint8_t Chassis_Key_Q_Change;
  bool Chassis_Switch_E;
  uint8_t Chassis_Key_E_Change;
//  bool Chassis_Switch_B;
//  uint8_t Chassis_Key_B_Change;
  bool Chassis_Switch_R;
  uint8_t Chassis_Key_R_Change;
  bool Chassis_Switch_Z;
  uint8_t Chassis_Key_Z_Change;
  bool Chassis_Switch_Shift;
  uint8_t Chassis_Key_Shift_Change;
//  
  bool Gimbal_Switch_Ctrl;
  uint8_t Gimbal_Key_Ctrl_Change;
//  
//  bool Gimbal_Switch_V;
//  uint8_t Gimbal_Key_V_Change;
  
} flag_t;//各种flag 状态位

extern flag_t Flag_status;//各种flag 状态位
//

//
#define    TIME_STAMP_1MS        1
#define    TIME_STAMP_2MS        2
#define    TIME_STAMP_4MS        4
#define    TIME_STAMP_10MS      10
#define    TIME_STAMP_20MS      20
#define    TIME_STAMP_30MS      30
#define    TIME_STAMP_40MS      40
#define    TIME_STAMP_50MS      50
#define    TIME_STAMP_60MS      60
#define    TIME_STAMP_80MS      80
#define    TIME_STAMP_100MS    100
#define    TIME_STAMP_150MS    150
#define    TIME_STAMP_200MS    200
#define    TIME_STAMP_250MS    250
#define    TIME_STAMP_300MS    300
#define    TIME_STAMP_400MS    400
#define    TIME_STAMP_500MS    500
#define    TIME_STAMP_600MS    600
#define    TIME_STAMP_1000MS  1000
#define    TIME_STAMP_2000MS  2000
#define    TIME_STAMP_10S    10000

#define    FALSE    0
#define    TRUE     1

#define RADIO 72.012f
#define YAW 0
#define PITCH 1

#define MECH 0
#define GYRO 1
//

/***********************底盘信息****************************************/
#define CHASSIS_DECELE_RATIO  (3591.0f/187.0f)		//减速比  670*715*450
#define LENGTH_A 218         //mm
#define LENGTH_B 232         //mm
#define WHEEL_PERIMETER 152  //mm 直径
/***********************YAW轴云台编码器的特定值******************/
#define GIMBAL_YAW_ENCODER_MIDDLE1 2615//5380//6423//3977//3062//1210//1320//1374		//底盘和云台朝向相同1，指向＋y
#define GIMBAL_YAW_ENCODER_MIDDLE2 6750//1284//2318//8073//7158//5308//5408		//底盘和云台朝向相同2，指向-y
#define GIMBAL_YAW_ENCODER_NINETY1 566//3332//4300//1925//1010//7357//7607		//底盘和云台朝向90°，指向+90°
#define GIMBAL_YAW_ENCODER_NINETY2 4653//7428//228//6025//5110//3259//3382		//底盘和云台朝向90°，指向-90°
#define GIMBAL_YAW_ENCODER_FORTYFIVE1 249	//底盘和云台朝向45°1，指向45°
#define GIMBAL_YAW_ENCODER_FORTYFIVE2 6490	//底盘和云台朝向45°2，指向135°
#define GIMBAL_YAW_ENCODER_FORTYFIVE3 4208	//底盘和云台朝向45°3，指向-135°
#define GIMBAL_YAW_ENCODER_FORTYFIVE4 2231	//底盘和云台朝向45°4，指向-45°

/**********各个模式下各个电机的限制电流大小***************/
//普通非跟随云台底盘限流
#define NOMOAL_CHASSIS_MAX1 20000
#define NOMOAL_CHASSIS_MAX2 20000
#define NOMOAL_CHASSIS_MAX3 20000
#define NOMOAL_CHASSIS_MAX4 20000
//爬坡非跟随云台底盘限流
#define CLIMBING_CHASSIS_MAX1 30000
#define CLIMBING_CHASSIS_MAX2 30000
#define CLIMBING_CHASSIS_MAX3 30000
#define CLIMBING_CHASSIS_MAX4 30000
//普通跟随云台底盘限流
#define NOMAL_FOLLOW_CHASSIS_MAX1 20000
#define NOMAL_FOLLOW_CHASSIS_MAX2 20000
#define NOMAL_FOLLOW_CHASSIS_MAX3 20000
#define NOMAL_FOLLOW_CHASSIS_MAX4 20000
//爬坡跟随云台底盘限流
#define CLIMBING_FOLLOW_CHASSIS_MAX1 30000
#define CLIMBING_FOLLOW_CHASSIS_MAX2 30000
#define CLIMBING_FOLLOW_CHASSIS_MAX3 30000
#define CLIMBING_FOLLOW_CHASSIS_MAX4 30000
//普通小陀螺/扭屁股限流
#define NOMAL_GYRO_CHASSIS_MAX1 30000
#define NOMAL_GYRO_CHASSIS_MAX2 30000
#define NOMAL_GYRO_CHASSIS_MAX3 30000
#define NOMAL_GYRO_CHASSIS_MAX4 30000
//爬坡小陀螺/扭屁股限流
#define CLIMBING_GYRO_CHASSIS_MAX1 30000
#define CLIMBING_GYRO_CHASSIS_MAX2 30000
#define CLIMBING_GYRO_CHASSIS_MAX3 30000
#define CLIMBING_GYRO_CHASSIS_MAX4 30000


/**********************************云台信息****************************************/
/***********************Pitch轴、YAW轴云台编码器限位****************************/
#define GIMBAL_PITCH_ENCODER_MAX 4050    //up
#define GIMBAL_PITCH_ENCODER_MIDDLE 3400
#define GIMBAL_PITCH_ENCODER_MIN 3080    //down
#define GIMBAL_YAW_ENCODER_MAX 6170       //right
#define GIMBAL_YAW_ENCODER_MIDDLE 1374
#define GIMBAL_YAW_ENCODER_MIN 2020        //left

/********************遥控器/键盘参数****************************/
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX 300.0f	//底盘跟随云台模式灵敏度vx  越大灵敏度越小
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY 300.0f	//底盘跟随云台模式灵敏度vy  越大灵敏度越小


#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX 300.0f	//底盘不跟随云台模式灵敏度vx  越大灵敏度越小
#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY 300.0f	//底盘不跟随云台模式灵敏度vy  越大灵敏度越小


#define SENSITIVITY_REMOTE_GIMBAL_YAW 200.0f		//云台灵敏度yaw轴，越大灵敏度越小
#define SENSITIVITY_REMOTE_GIMBAL_PITCH 200.0f		//云台灵敏度pitch轴，越大灵敏度越小

#define SENSITIVITY_REMOTE_GIMBAL_YAW_IMU 1140.0f		//云台灵敏度yaw轴，越大灵敏度越小
#define SENSITIVITY_REMOTE_GIMBAL_PITCH_IMU 1140.0f		//云台灵敏度pitch轴，越大灵敏度越小

/***********************视觉灵敏度*************************************/

#define SENSITIVITY_VISION_GIMBAL_YAW_ENCODER 700.0f		//云台灵敏度yaw轴，越大灵敏度越小
#define SENSITIVITY_VISION_GIMBAL_PITCH_ENCODER 700.0f		//云台灵敏度pitch轴，越大灵敏度越小

/***************************拨盘信息****************************************/
/******************拨盘电机限流****************/
#define   REVOLVER_PID_POSITION_OUTMAX1       5000
#define   REVOLVER_PID_POSITION_IMAX1         2000
#define   REVOLVER_PID_SPEED_OUTMAX2    31000
#define   REVOLVER_PID_SPEED_IMAX2      15000
/******************拨盘硬件尺寸******************/
//#define   REVOL_SPEED_RATIO   2160       //电机轴一秒转一圈,2160转子转速,60*36,乘射频再除以拨盘格数就可得相应射频下的转速
//#define 	REVOL_SPEED_GRID      4			//拨盘格数
//#define  	AN_BULLET         (24576.0f)		//单个子弹电机位置增加值(这个值得测鸭)

#endif
