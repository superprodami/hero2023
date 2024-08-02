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
/* ��̨����ģʽ:

   ��ͨ             	NORMAL
   ��ͷ180��             AROUND
   ���             	BUFF
   ����,pitchˮƽ   	LEVEL
   ��еģʽpitcḩͷ	HIGH
   ����Ťͷ90��          TURN
*/
//typedef enum
//{
//    GIMBAL_NORMAL            = 0,//����ģʽ,����ģʽѡ��
//    GIMBAL_TURN_RIGHT        = 1,//��ת90���ͷ
//    GIMBAL_CHASSIS_FOLLOW    = 2,//���̸�����̨
//    GIMBAL_LEVEL             = 3,//���ֿ���,��̨ˮƽ
//    GIMBAL_MANUAL            = 4,//�ֶ����ģʽ
//    GIMBAL_SM_BUFF           = 5,//С��
//    GIMBAL_TURN_LEFT         = 7,//��ת90��Ťͷ
//    GIMBAL_AUTO              = 8,//����
//    GIMBAL_BASE              = 9,//��ͷ�������
//    GIMBAL_BUFF              = 10,//���ģʽ,��
//    GIMBAL_GYROSCOPE         = 11,//С����
//} eGimbalAction;
//extern volatile eGimbalAction  actGimbal;

//typedef struct  //�Ӿ�Ŀ���ٶȲ���
//{
//    int delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
//    int freq;
//    int last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
//    float last_position;//�ϸ�Ŀ��Ƕ�
//    float speed;//�ٶ�
//    float last_speed;//�ϴ��ٶ�
//    float processed_speed;//�ٶȼ�����
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
//    GIMBAL_HORIZON,  //ˮƽ
//	GIMBAL_VERTICAL,  //��ֱ
//} Gimbal_Hanging_Status;
//extern Gimbal_Hanging_Status gimbal_hanging;

//extern volatile eGimbalAction  actGimbal;
extern GimbalModeType YawGimbalMode;
extern GimbalModeType PitchGimbalMode;
//extern Gimbal_Current_Follow gimbal_follow;

static void Gimbal_Open_Init(void);
static void RemoteControlGimbal(void);

/********************����ģʽ****************************/
static void GetEnvironmentGimbalMode(void);	//���ó�����������
static void GIMBAL_Mode_Choose(void);  ///��̨����ģʽѡ��,������Ӧ/
static void GIMBAL_Key_Ctrl(void);     ///���̿�����̨ģʽ

/*****************************��̨λ��PID����***********************************/
static void GIMBAL_InitArgument(void);  //��̨������ʼ��

/***********��̨����ģʽ����ģʽС����*******************/
static void GIMBAL_NO_CHASSIS_FOLLOW_Mode_Ctrl(void);
static void GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl(void); //���̸�����̨����
static void GIMBAL_GYROSCOPE_Mode_Ctrl(void);   //С����ģʽ
static void GIMBAL_LEVEL_Mode_Ctrl(void);  ///����ģʽ/
static void GIMBAL_AUTO_Mode_Ctrl(void);  ///������ƺ���
//			void GIMBAL_BASE_Mode_Ctrl(void);  ///��ͷ����ģʽ
/******************************************************************/
//static void GIMBAL_State_Change(GimbalModeType Type);
extern void GIMBAL_State_Change(GimbalModeType Type);
static void GIMBAL_Double_Loop_Out(void);  //��̨������
static void kalman_filter_change_realtime(void);
extern float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro
extern void PID_Change(fp32 *Kpid_Angle ,fp32 *Kpid_speed);
#endif



