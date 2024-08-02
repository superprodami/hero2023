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
	NOMAL,		//��ͨģʽ
	CLIMBING,	//����ģʽ
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
extern volatile SYSTEMVALUE SystemValue;	//����״̬
extern uint32_t control_judge_flag;
extern uint8_t state_judge;
extern volatile bool imu_init_finish_flag;

//chassis
typedef enum
{
    CHASSIS_FOLLOW_GIMBAL = 1,	//���̸�����������
    CHASSIS_GYROSCOPE     = 2,  //С����ģʽ
    CHASSIS_NORMAL        = 3,  //���̲�������̨����
    CHASSIS_CORGI         = 4,  //Ťƨ��ģʽ
    CHASSIS_ROSHAN        = 5,  //���ģʽ
    CHASSIS_SLOW          = 6,  //��������ģʽ
    CHASSIS_SZUPUP        = 7,  //����ģʽ
    CHASSIS_MISS          = 8,  //�Զ�����ģʽ
    CHASSIS_FORTYFIVE     = 9,  //45��ģʽ
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
    GIMBAL_NORMAL            = 0,//����ģʽ,����ģʽѡ��
    GIMBAL_TURN_RIGHT        = 1,//��ת90���ͷ
    GIMBAL_CHASSIS_FOLLOW    = 2,//���̸�����̨
    GIMBAL_LEVEL             = 3,//���ֿ���,��̨ˮƽ
    GIMBAL_MANUAL            = 4,//�ֶ����ģʽ
    GIMBAL_SM_BUFF           = 5,//С��
    GIMBAL_TURN_LEFT         = 7,//��ת90��Ťͷ
    GIMBAL_AUTO              = 8,//����
    GIMBAL_BASE              = 9,//��ͷ�������
    GIMBAL_BUFF              = 10,//���ģʽ,��
    GIMBAL_GYROSCOPE         = 11,//С����
} eGimbalAction;
extern volatile eGimbalAction  actGimbal;
typedef enum
{
    GIMBAL_HORIZON,  //ˮƽ
	GIMBAL_VERTICAL,  //��ֱ
} Gimbal_Hanging_Status;
extern Gimbal_Hanging_Status gimbal_hanging;
extern float IMU_angle[3];

//shoot
typedef enum  //Ħ����״̬
{
    UNSTART = 0,
    START = 1,
    FINISH = 2,
} eShootState;
extern eShootState ShootState;
typedef enum  //����״̬
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

	
	
    float Dead_Zone; //���������ֵ
    uint8_t mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set; //�趨ֵ
    float fdb; //����ֵ

    float out;
    float lastout;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�
    int angle_max;
    int angle_min;	//�Ƕ�����ֵ ����һ��Բ�ڣ�0���360�����ڣ���max=360��min=0
    //			��һ������� 0��8192���ڣ���max=8192��min=0
    float I_Separation; //���ַ�����ֵ
    float gama;			//΢�������˲�ϵ��
    float lastdout;		//��һ��΢�����

    void ( *f_param_init)(struct PidTypeDef *pid,  //PID������ʼ��
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

    float (*f_cal_pid)(struct PidTypeDef *pid, const float ref, const float set);   //pid����
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
    float     target_angle;		//Ŀ��Ƕ�
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
    float  		velocity,   //������ �ٶȣ�λ�ã�ת��
							position,
							torque,
							target_velocity,
							target_position;

    int16_t    	speed_dp10ms;		//��е�Ƕ�/10ms   ��е�Ƕȷ�Χ[0,8191]
    //תÿ10ms=speed_dp10ms/8192

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
    PID_POSITION = 0,//λ��ʽPID
    PID_DELTA, //����ʽPID
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

	bool rc_shoot_flag;  //ң����������
	bool rc_wheel_flag;  //ͬ��
		
  uint16_t shoot_cnt;
  uint32_t shoot_left_time;
  uint8_t shoot_single_finish_flag;
  uint32_t shoot_single_time;
  uint8_t Ammunition_flag; //ң����ģʽʹ�ã��ڿ��ز�����Ӧλ����ʱ���濪�ط�������
  uint8_t moca_flag;       //����ͬ��
  bool protect_flag_sutck; //������ģʽ����������ģʽѡ�� ��ֹ���������ɱ����
  bool protect_flag_heat;  //����ͬ��
  
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
  
} flag_t;//����flag ״̬λ

extern flag_t Flag_status;//����flag ״̬λ
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

/***********************������Ϣ****************************************/
#define CHASSIS_DECELE_RATIO  (3591.0f/187.0f)		//���ٱ�  670*715*450
#define LENGTH_A 218         //mm
#define LENGTH_B 232         //mm
#define WHEEL_PERIMETER 152  //mm ֱ��
/***********************YAW����̨���������ض�ֵ******************/
#define GIMBAL_YAW_ENCODER_MIDDLE1 2615//5380//6423//3977//3062//1210//1320//1374		//���̺���̨������ͬ1��ָ��y
#define GIMBAL_YAW_ENCODER_MIDDLE2 6750//1284//2318//8073//7158//5308//5408		//���̺���̨������ͬ2��ָ��-y
#define GIMBAL_YAW_ENCODER_NINETY1 566//3332//4300//1925//1010//7357//7607		//���̺���̨����90�㣬ָ��+90��
#define GIMBAL_YAW_ENCODER_NINETY2 4653//7428//228//6025//5110//3259//3382		//���̺���̨����90�㣬ָ��-90��
#define GIMBAL_YAW_ENCODER_FORTYFIVE1 249	//���̺���̨����45��1��ָ��45��
#define GIMBAL_YAW_ENCODER_FORTYFIVE2 6490	//���̺���̨����45��2��ָ��135��
#define GIMBAL_YAW_ENCODER_FORTYFIVE3 4208	//���̺���̨����45��3��ָ��-135��
#define GIMBAL_YAW_ENCODER_FORTYFIVE4 2231	//���̺���̨����45��4��ָ��-45��

/**********����ģʽ�¸�����������Ƶ�����С***************/
//��ͨ�Ǹ�����̨��������
#define NOMOAL_CHASSIS_MAX1 20000
#define NOMOAL_CHASSIS_MAX2 20000
#define NOMOAL_CHASSIS_MAX3 20000
#define NOMOAL_CHASSIS_MAX4 20000
//���·Ǹ�����̨��������
#define CLIMBING_CHASSIS_MAX1 30000
#define CLIMBING_CHASSIS_MAX2 30000
#define CLIMBING_CHASSIS_MAX3 30000
#define CLIMBING_CHASSIS_MAX4 30000
//��ͨ������̨��������
#define NOMAL_FOLLOW_CHASSIS_MAX1 20000
#define NOMAL_FOLLOW_CHASSIS_MAX2 20000
#define NOMAL_FOLLOW_CHASSIS_MAX3 20000
#define NOMAL_FOLLOW_CHASSIS_MAX4 20000
//���¸�����̨��������
#define CLIMBING_FOLLOW_CHASSIS_MAX1 30000
#define CLIMBING_FOLLOW_CHASSIS_MAX2 30000
#define CLIMBING_FOLLOW_CHASSIS_MAX3 30000
#define CLIMBING_FOLLOW_CHASSIS_MAX4 30000
//��ͨС����/Ťƨ������
#define NOMAL_GYRO_CHASSIS_MAX1 30000
#define NOMAL_GYRO_CHASSIS_MAX2 30000
#define NOMAL_GYRO_CHASSIS_MAX3 30000
#define NOMAL_GYRO_CHASSIS_MAX4 30000
//����С����/Ťƨ������
#define CLIMBING_GYRO_CHASSIS_MAX1 30000
#define CLIMBING_GYRO_CHASSIS_MAX2 30000
#define CLIMBING_GYRO_CHASSIS_MAX3 30000
#define CLIMBING_GYRO_CHASSIS_MAX4 30000


/**********************************��̨��Ϣ****************************************/
/***********************Pitch�ᡢYAW����̨��������λ****************************/
#define GIMBAL_PITCH_ENCODER_MAX 4050    //up
#define GIMBAL_PITCH_ENCODER_MIDDLE 3400
#define GIMBAL_PITCH_ENCODER_MIN 3080    //down
#define GIMBAL_YAW_ENCODER_MAX 6170       //right
#define GIMBAL_YAW_ENCODER_MIDDLE 1374
#define GIMBAL_YAW_ENCODER_MIN 2020        //left

/********************ң����/���̲���****************************/
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX 300.0f	//���̸�����̨ģʽ������vx  Խ��������ԽС
#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY 300.0f	//���̸�����̨ģʽ������vy  Խ��������ԽС


#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX 300.0f	//���̲�������̨ģʽ������vx  Խ��������ԽС
#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY 300.0f	//���̲�������̨ģʽ������vy  Խ��������ԽС


#define SENSITIVITY_REMOTE_GIMBAL_YAW 200.0f		//��̨������yaw�ᣬԽ��������ԽС
#define SENSITIVITY_REMOTE_GIMBAL_PITCH 200.0f		//��̨������pitch�ᣬԽ��������ԽС

#define SENSITIVITY_REMOTE_GIMBAL_YAW_IMU 1140.0f		//��̨������yaw�ᣬԽ��������ԽС
#define SENSITIVITY_REMOTE_GIMBAL_PITCH_IMU 1140.0f		//��̨������pitch�ᣬԽ��������ԽС

/***********************�Ӿ�������*************************************/

#define SENSITIVITY_VISION_GIMBAL_YAW_ENCODER 700.0f		//��̨������yaw�ᣬԽ��������ԽС
#define SENSITIVITY_VISION_GIMBAL_PITCH_ENCODER 700.0f		//��̨������pitch�ᣬԽ��������ԽС

/***************************������Ϣ****************************************/
/******************���̵������****************/
#define   REVOLVER_PID_POSITION_OUTMAX1       5000
#define   REVOLVER_PID_POSITION_IMAX1         2000
#define   REVOLVER_PID_SPEED_OUTMAX2    31000
#define   REVOLVER_PID_SPEED_IMAX2      15000
/******************����Ӳ���ߴ�******************/
//#define   REVOL_SPEED_RATIO   2160       //�����һ��תһȦ,2160ת��ת��,60*36,����Ƶ�ٳ��Բ��̸����Ϳɵ���Ӧ��Ƶ�µ�ת��
//#define 	REVOL_SPEED_GRID      4			//���̸���
//#define  	AN_BULLET         (24576.0f)		//�����ӵ����λ������ֵ(���ֵ�ò�Ѽ)

#endif
