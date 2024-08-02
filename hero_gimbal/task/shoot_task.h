#ifndef SHOOTTASKH
#define SHOOTTASKH
#include "main.h"

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
} eShootAction;
extern eShootAction actShoot;

//typedef enum  //发射状态
//{
//    UNSTART = 0,
//    START = 1,
//    FINISH = 2,
//} eShootState;
//extern eShootState ShootState;


//typedef enum  //摩擦轮状态
//{
//    mNORMAL = 0,
//    mUNUSUAL = 1,
//} Mocalun_Status;
//extern Mocalun_Status mocalun_status;

static void RemoteShoot(void);
static uint16_t Judge_Speed(void);
//extern heat_measure_t heat_judge;
//extern shoot_measure_t shoot_judge;
static void Unusual_Mode_Ctrl(void);
/*******************键盘模式******************************/
static void REVOLVER_Key_Ctrl(void);  //拨盘的键盘选择模式

/******拨盘键盘模式各类模式小函数*******/
static void SHOOT_Mode_Choose(void);

/*************************************/
static void REVOLVER_InitArgument(void);   //拨盘参数初始化
static void REVOLVER_Rest(void);            //发弹清零,哪怕鼠标继续点击也不给发弹

#endif

