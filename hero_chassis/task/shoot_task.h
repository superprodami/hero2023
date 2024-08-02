#ifndef SHOOTTASKH
#define SHOOTTASKH
#include "main.h"

#define Speed_limit  16 

//extern int shoot_speed_adjust;
typedef enum
{
    SHOOT_NORMAL       =  0,//射击模式选择,默认不动
    SHOOT_SINGLE       =  1,//单发
    SHOOT_TRIPLE       =  2,//三连发
    SHOOT_HIGHTF_LOWS  =  3,//高射频低射速
    SHOOT_MIDF_HIGHTS  =  4,//中射频高射速
    SHOOT_BUFF         =  5,//打符模式
    STOP               =  6,//反转
    RELAX              =  7,
	  SHOOT_SINGLE_DM       =  8,
} eShootAction;

eShootAction actShoot = SHOOT_NORMAL;
eShootAction actShoot_last = SHOOT_NORMAL;

bool single_shoot_flag = 1;
float main_angle=0;



/************卡弹************/
#define Stuck_Revol_PIDTerm   20     //25000      //PID输出大于这个数则认为有可能卡弹
#define Stuck_Speed_Low       0.01   //70         //测量速度低于这个数,则认为有可能卡弹

#define Stuck_SpeedPID_Time   800        //速度连续 ms过小,PID连续  ms过大
#define Stuck_Relax_Time   250       //放松时间,时间越长倒得越多
extern eShootAction actShoot;

static void RemoteShoot(void);
static void Limit_something(void);
void Judge_Speed(void);
static void SendJudgeMsg(void);
/*******************键盘模式******************************/
static void REVOLVER_Key_Ctrl(void);  //拨盘的键盘选择模式

/******拨盘键盘模式各类模式小函数*******/
static void SHOOT_Mode_Choose(void);
static void SHOOT_SINGLE_Ctrl(void);
static void SHOOT_SINGLE_Ctrl_DM(void);
static void SHOOT_TRIPLE_Ctrl(void);
static void DM_SHOOT_SINGLE_Ctrl(void);



/****卡弹处理*****/
static void REVOL_SpeedStuck(void);
//void REVOL_PositStuck(void);

/******射频热量限制******/
static bool Revolver_Heat_Limit(void);

/*************************************/
static void REVOLVER_InitArgument(void);   //拨盘参数初始化
static void REVOLVER_Rest(void);            //发弹清零,哪怕鼠标继续点击也不给发弹

#endif

