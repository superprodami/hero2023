#ifndef SHOOTTASKH
#define SHOOTTASKH
#include "main.h"

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
} eShootAction;
extern eShootAction actShoot;

//typedef enum  //����״̬
//{
//    UNSTART = 0,
//    START = 1,
//    FINISH = 2,
//} eShootState;
//extern eShootState ShootState;


//typedef enum  //Ħ����״̬
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
/*******************����ģʽ******************************/
static void REVOLVER_Key_Ctrl(void);  //���̵ļ���ѡ��ģʽ

/******���̼���ģʽ����ģʽС����*******/
static void SHOOT_Mode_Choose(void);

/*************************************/
static void REVOLVER_InitArgument(void);   //���̲�����ʼ��
static void REVOLVER_Rest(void);            //��������,�������������Ҳ��������

#endif

