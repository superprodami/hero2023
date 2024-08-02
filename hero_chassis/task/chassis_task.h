#ifndef CHASSIStaskH
#define CHASSIStaskH
#include "main.h"

typedef struct
{
    float vx;
    float vy;
    float vw;
} Chassis_Speed;

typedef struct
{
    uint8_t W;
    uint8_t S;
    uint8_t A;
    uint8_t D;
    uint8_t JS;
    float IP;
} KEYflag;




static void Chassis_InitArgument(void);
//static void SetChassisMotorMaxCurrent(const int16_t max1, const int16_t max2, const int16_t max3, const int16_t max4);
static void RemoteModeChoose(void);
static void RemoteControlChassis(void);
static void KeyboardControlChassis(void);
static void GetEnvironmentChassisMode(void);
static void mecanum_calc(Chassis_Speed *speed, int16_t *out_speed);
static void Mecanum_Set_Motor_Speed(int16_t *out_speed, Motortype *Motor );
static void Absolute_Cal(Chassis_Speed *absolute_speed, float angle );
static float FindMinAnglePNY(void);
static float Find_Y_AnglePNY(void);
//static void CHASSIS_MISS_Mode_Ctrl(void);
/*****************键盘模式*************************/
static void CHASSIS_InitArgument(void);  //键盘底盘参数初始化//
static void Chassis_Mode_Choose( void);   ////底盘键盘模式选择,按键响应//
static void CHAS_Key_Ctrl( void);    ////键盘控制底盘移动/

static void Chassis_Mouse_Move_Calculate( void);   /////鼠标控制底盘旋转,键盘QEC控制快速转圈//
static void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp_inc, int16_t sMoveRamp_dec );
static float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec ); //键盘模式辅助函数
static void Chassis_open_init(void);

/*****************底盘功率*************************/
static void Chassis_Power_Limit( void);
void chassis_power_control();
void get_chassis_power_and_buffer(float *chassis_power,float *chassis_power_buffer);
void get_chassis_power_and_buffer(float *chassis_power,float *chassis_power_buffer);

static void CHASSIS_Single_Loop_Out( void); //底盘电机输出
#endif



