#ifndef SHOOTTASKH
#define SHOOTTASKH
#include "main.h"

#define Speed_limit  16 

//extern int shoot_speed_adjust;
typedef enum
{
    SHOOT_NORMAL       =  0,//���ģʽѡ��,Ĭ�ϲ���
    SHOOT_SINGLE       =  1,//����
    SHOOT_TRIPLE       =  2,//������
    SHOOT_HIGHTF_LOWS  =  3,//����Ƶ������
    SHOOT_MIDF_HIGHTS  =  4,//����Ƶ������
    SHOOT_BUFF         =  5,//���ģʽ
    STOP               =  6,//��ת
    RELAX              =  7,
	  SHOOT_SINGLE_DM       =  8,
} eShootAction;

eShootAction actShoot = SHOOT_NORMAL;
eShootAction actShoot_last = SHOOT_NORMAL;

bool single_shoot_flag = 1;
float main_angle=0;



/************����************/
#define Stuck_Revol_PIDTerm   20     //25000      //PID����������������Ϊ�п��ܿ���
#define Stuck_Speed_Low       0.01   //70         //�����ٶȵ��������,����Ϊ�п��ܿ���

#define Stuck_SpeedPID_Time   800        //�ٶ����� ms��С,PID����  ms����
#define Stuck_Relax_Time   250       //����ʱ��,ʱ��Խ������Խ��
extern eShootAction actShoot;

static void RemoteShoot(void);
static void Limit_something(void);
void Judge_Speed(void);
static void SendJudgeMsg(void);
/*******************����ģʽ******************************/
static void REVOLVER_Key_Ctrl(void);  //���̵ļ���ѡ��ģʽ

/******���̼���ģʽ����ģʽС����*******/
static void SHOOT_Mode_Choose(void);
static void SHOOT_SINGLE_Ctrl(void);
static void SHOOT_SINGLE_Ctrl_DM(void);
static void SHOOT_TRIPLE_Ctrl(void);
static void DM_SHOOT_SINGLE_Ctrl(void);



/****��������*****/
static void REVOL_SpeedStuck(void);
//void REVOL_PositStuck(void);

/******��Ƶ��������******/
static bool Revolver_Heat_Limit(void);

/*************************************/
static void REVOLVER_InitArgument(void);   //���̲�����ʼ��
static void REVOLVER_Rest(void);            //��������,�������������Ҳ��������

#endif

