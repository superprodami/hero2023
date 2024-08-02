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
	uint8_t	  identify_target;	//��Ұ���Ƿ���Ŀ��/�Ƿ�ʶ����Ŀ��   0��  1��
	uint8_t   center_flag;
}VisionValueType;

extern VisionValueType VisionValue;

typedef enum
{
	aNORMAL,  //��ͨģʽ
	aHANGING,  //����ģʽ
}Vision_Mode;
extern Vision_Mode vision_mode;

extern uint8_t vision_buf[];
extern uint8_t Vision_Send_Buff[16];
extern uint32_t vision_time;

typedef struct
{
    unsigned char dataReset;
    float yaw[2];         //�����root��yaw��pitch�Ƕ�
    float pitch[2];       //0��Ϊ��ʼ����    1��״̬���·���֮�������
    float yawE[2];         //�����root��yaw��pitch�Ƕ�
    float pitchE[2];       //0��Ϊ��ʼ����    1��״̬���·���֮�������
    float yawRate[2];     //yaw��pitch�Ľ��ٶ�
    float pitchRate[2];   //0��Ϊ��ʼ����    1��״̬���·���֮�������
    float yawAcc[2];     //yaw��pitch�ļ��ٶ�
    float pitchAcc[2];   //0��Ϊ��ʼ����    1��״̬���·���֮�������
    float yawJerk[2];     //yaw��pitch�ļ�����
    float pitchJerk[2];   //0��Ϊ��ʼ����    1��״̬���·���֮�������
    float yawX;
    float pitchX;           //״̬��������
    float yawRateX;
    float pitchRateX;
    float yawAccX;
    float pitchAccX;
    float yawJerkX;
    float pitchJerkX;
    float yawOut;        //����Ԥ��λ��
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
} followStruct;					//������������

typedef struct{
	
	uint16_t time1;//��¼ʱ��
	uint32_t time2;//������ʼϵͳʱ��
	float Small_gyro_near;//��init��timemax��quit_value�Լ�װ�װ���������������Ŵ�
	int8_t last_dir;//��һ���������
	uint16_t is_small_gyro;//��ת����
	uint8_t small_gyro_aimed;//��С���ݱ�־λ
	float Frequency_gyro;//װ�װ�Ƶ�ʡ�����Ƶ��
	
}SmallGyro_out_type;//�������и�ֵ�ı���

typedef struct{
	uint8_t timeout_flag;
	uint8_t	valueout_flag;
}SmallGyro_debugType;//���Թ۲췴С�����˳���ʽ��ʱ��̫�̻�ƫ��̫��

typedef struct{
	
	uint16_t small_gyro_timemax;//��תһ��װ�װ���Ҫ��ʱ�䣬�������ֵ���˳���С���ݣ����趨С���ݵ�����ٶ�
	float distance_max;//�������ֵ�Ŵ��ʾ�Ϊ1
	float near_max;//��Զ�����Ŵ��ʵ����ֵ
	uint8_t is_count_max;//�ж�С������ת�������뷴С���ݵ����ֵ
	float quit_value;//yaw��ﵽ���ֵ��ǿ���˳�С����ģʽ
	extKalman_t frequency_gyro_kalman;
	
}SmallGyro_init_type;//��Ҫ�����趨�ı���

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


