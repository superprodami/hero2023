#ifndef _bspvision_h
#define _bspvision_h
#include "main.h"
#define VISION_HUART huart8
#define VISION_MAX_LEN 50
#define VISION_BUFLEN 18
#define VISION_SEND 4
typedef struct{
union
	{
		uint8_t buff[4];
		float value;
	}vision_yaw_value;
	union																																																					\
    {
        uint8_t buff[4];
		float value;
    }normal_yaw_value;
	union
	{
		uint8_t buff[4];
		float value;
	}vision_pitch_value;

	union																																																					\
    {
        uint8_t buff[4];
		float value;
        float value_offset;
    } vision_distance;

	uint8_t   GYROSCOPE_target;
	uint8_t   lock_target;
	uint8_t	  shoot_target;
	uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是
	uint8_t   center_flag;
}VisionValueType;

extern VisionValueType VisionValue;

typedef enum
{
	aNORMAL,  //普通模式
	aHANGING,  //吊射模式
}Vision_Mode;
extern Vision_Mode vision_mode;

extern uint8_t vision_buf[];
extern uint8_t Vision_Send_Buff[16];
extern uint32_t vision_time;

typedef struct
{
    unsigned char dataReset;
    float yaw[2];         //相对于root的yaw，pitch角度
    float pitch[2];       //0：为初始数据    1：状态更新方程之后的数据
    float yawE[2];         //相对于root的yaw，pitch角度
    float pitchE[2];       //0：为初始数据    1：状态更新方程之后的数据
    float yawRate[2];     //yaw，pitch的角速度
    float pitchRate[2];   //0：为初始数据    1：状态更新方程之后的数据
    float yawAcc[2];     //yaw，pitch的加速度
    float pitchAcc[2];   //0：为初始数据    1：状态更新方程之后的数据
    float yawJerk[2];     //yaw，pitch的急动度
    float pitchJerk[2];   //0：为初始数据    1：状态更新方程之后的数据
    float yawX;
    float pitchX;           //状态外推数据
    float yawRateX;
    float pitchRateX;
    float yawAccX;
    float pitchAccX;
    float yawJerkX;
    float pitchJerkX;
    float yawOut;        //最终预判位置
    float pitchOut;
    /*new*/
    float distanceX;
    float distanceRateX;
    float distanceAccX;
    float distance[2];
    float distanceRate[2];
    float distanceAcc[2];

    float angle;
    float angleOut;
    float carRate;
    float flyTime;
} followStruct;					//自瞄运算数据

typedef struct{
	
	uint16_t time1;//记录时间
	uint32_t time2;//计数初始系统时间
	float Small_gyro_near;//对init的timemax、quit_value以及装甲板间距随距离拉近而放大
	int8_t last_dir;//上一次误差正负
	uint16_t is_small_gyro;//旋转计数
	uint8_t small_gyro_aimed;//反小陀螺标志位
	float Frequency_gyro;//装甲板频率——打弹频率
	
}SmallGyro_out_type;//无需自行赋值的变量

typedef struct{
	uint8_t timeout_flag;
	uint8_t	valueout_flag;
}SmallGyro_debugType;//调试观察反小陀螺退出方式，时间太短或偏移太大

typedef struct{
	
	uint16_t small_gyro_timemax;//旋转一个装甲板需要的时间，超过这个值就退出反小陀螺，即设定小陀螺的最高速度
	float distance_max;//超出这个值放大倍率就为1
	float near_max;//从远到近放大倍率的最大值
	uint8_t is_count_max;//判定小陀螺旋转次数进入反小陀螺的最大值
	float quit_value;//yaw轴达到这个值就强行退出小陀螺模式
	extKalman_t frequency_gyro_kalman;
	
}SmallGyro_init_type;//需要自行设定的变量

typedef struct{
	SmallGyro_init_type	SmallGyro_init;
	SmallGyro_out_type	SmallGyro_out;
	SmallGyro_debugType SmallGyro_debug;
}SmallGyroType;

void Small_gyro_init(SmallGyroType *small_gyro,uint16_t small_gyro_timemax,float distance_max,float near_max,uint8_t is_count_max,float quit_value,float T_Q,float T_R);
void SMALL_Gyro_Judge(SmallGyroType *small_gyro,VisionValueType *visionvalue);
extern SmallGyroType Small_Gyro;

extern void get_vision_info(uint8_t temp[]);
#endif


