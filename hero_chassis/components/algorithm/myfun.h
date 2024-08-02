#ifndef MYSYSTEM_H
#define MYSYSTEM_H
//#include "main.h"
#include "mydef.h"
#include "motor.h"


#define	_PI			3.14159265f
#define _2PI		6.28318531f
#define _PI_2		1.57079633f


typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�������
    fp32 min_value;    //�޷���Сֵ
    fp32 max_value;    //�޷����ֵ
    fp32 frame_period; //ʱ����
} ramp_function_source_t;

typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�˲����������
    fp32 num[1];       //�˲�����
    fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
//���ٿ���
extern fp32 invSqrt(fp32 num);

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//��������
extern void abs_limit(fp32 *num, fp32 Limit);
//�жϷ���λ
extern fp32 sign(fp32 value);
//��������
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern fp32 theta_format(fp32 Ang);
//����ת��  
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x, float x_min, float x_max, int bits);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

void LimitValue_16(int16_t* VALUE,int16_t MAX,int16_t MIN);
void LimitValue_u16(uint16_t* VALUE,uint16_t MAX,uint16_t MIN);
float constrain_float(float amt, float low, float high);
float RampInc_float( float *buffer, float now, float ramp );
void AngleLoop (float* angle ,float max);
void AngleLoop_f (float* angle ,float max);
float AngleLoop_DM(float *target_angle, float *angle); 
float RAMP_float( float final, float now, float ramp );
void LimitValue_f(float* VALUE,float MAX,float MIN);
float abs_float(float a);
float queue_sum(int n,float*yaw_smooth,float*yaw_smooth1,float item1);

void soft_rest(void);
void Stop_All(void);

#endif
