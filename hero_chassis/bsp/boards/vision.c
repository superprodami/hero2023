#include "vision.h"
#include "string.h"
//#include "type.h"
#include "math.h"
VisionValueType VisionValue;
Vision_Mode vision_mode;

uint8_t vision_buf[VISION_BUFLEN];
uint8_t Vision_Send_Buff[16];
uint16_t identify[30],identify_sum;
uint16_t identify_flag;
float yaw_direct=0;
float yaw_lastdirect=0;
float yaw_value_last=0;
volatile int8_t yaw_vision_flag=1;
volatile int8_t yaw_vision_value=0;
bool vfire=0;
volatile uint8_t v0='0';
volatile uint8_t v1='0';
uint16_t vfnum=0;
uint32_t vision_time = 0;

SmallGyroType Small_Gyro;

void Small_gyro_init(SmallGyroType *small_gyro,uint16_t small_gyro_timemax,float distance_max,float near_max,uint8_t is_count_max,float quit_value,float T_Q,float T_R)
{
	small_gyro->SmallGyro_init.small_gyro_timemax=small_gyro_timemax;
	small_gyro->SmallGyro_init.distance_max=distance_max;
	small_gyro->SmallGyro_init.near_max=near_max;
	small_gyro->SmallGyro_init.is_count_max=is_count_max;
	small_gyro->SmallGyro_init.quit_value=quit_value;
	small_gyro->SmallGyro_init.frequency_gyro_kalman.X_last = 0.f;
	
	small_gyro->SmallGyro_init.frequency_gyro_kalman.P_last = 0;
	small_gyro->SmallGyro_init.frequency_gyro_kalman.Q = T_Q;
	small_gyro->SmallGyro_init.frequency_gyro_kalman.R = T_R;
	small_gyro->SmallGyro_init.frequency_gyro_kalman.A = 1;
	small_gyro->SmallGyro_init.frequency_gyro_kalman.B = 0;
	small_gyro->SmallGyro_init.frequency_gyro_kalman.H = 1;
	small_gyro->SmallGyro_init.frequency_gyro_kalman.X_mid = 0.f;
	
	small_gyro->SmallGyro_out.is_small_gyro = 0;
	small_gyro->SmallGyro_out.last_dir = -1;
	small_gyro->SmallGyro_out.time1 = 0;
	small_gyro->SmallGyro_out.Small_gyro_near = 1;
	small_gyro->SmallGyro_out.Frequency_gyro = 0;
	small_gyro->SmallGyro_out.small_gyro_aimed = 0;
}
void SMALL_Gyro_Judge(SmallGyroType *small_gyro,VisionValueType *visionvalue)
{
		//根据距离判断倍率
		if(visionvalue->vision_distance.value < small_gyro->SmallGyro_init.distance_max && visionvalue->vision_distance.value > 0.1f)
		{
			small_gyro->SmallGyro_out.Small_gyro_near=(small_gyro->SmallGyro_init.distance_max-visionvalue->vision_distance.value)
																								*(small_gyro->SmallGyro_init.near_max - 1) / small_gyro->SmallGyro_init.distance_max + 1;
		}
		else 
			small_gyro->SmallGyro_out.Small_gyro_near = 1;
		
		
		if( small_gyro->SmallGyro_out.last_dir*visionvalue->vision_yaw_value.value < 0
			&& fabs(visionvalue->vision_yaw_value.value) > 0.1f *small_gyro->SmallGyro_out.Small_gyro_near)
		{
			if(small_gyro->SmallGyro_out.time1 > 0)
				small_gyro->SmallGyro_out.Frequency_gyro = KalmanFilter(&small_gyro->SmallGyro_init.frequency_gyro_kalman,500.f/small_gyro->SmallGyro_out.time1);
			if(small_gyro->SmallGyro_out.Frequency_gyro > 9)
				small_gyro->SmallGyro_out.Frequency_gyro = 9;
			small_gyro->SmallGyro_out.time1 = 0;
			small_gyro->SmallGyro_out.last_dir*= -1;
			small_gyro->SmallGyro_out.is_small_gyro++;
			small_gyro->SmallGyro_out.time2=xTaskGetTickCount();
		}
		else
		{
			small_gyro->SmallGyro_out.time1 = xTaskGetTickCount() - small_gyro->SmallGyro_out.time2;//通过系统时钟计算频率
		}

		if(fabs(visionvalue->normal_yaw_value.value) > small_gyro->SmallGyro_init.quit_value / small_gyro->SmallGyro_out.Small_gyro_near)
		{//normal_yaw_value为无预测的视觉偏移量，此处是为了防止预测值的突增
			small_gyro->SmallGyro_out.time1 = 0;
			small_gyro->SmallGyro_out.is_small_gyro = 0;
			small_gyro->SmallGyro_debug.timeout_flag = 0;
			small_gyro->SmallGyro_debug.valueout_flag = 1;
		}
		if(small_gyro->SmallGyro_out.time1 > small_gyro->SmallGyro_init.small_gyro_timemax * small_gyro->SmallGyro_out.Small_gyro_near)
		{
			small_gyro->SmallGyro_out.time1 = 0;
			small_gyro->SmallGyro_out.is_small_gyro = 0;
			small_gyro->SmallGyro_debug.timeout_flag = 1;
			small_gyro->SmallGyro_debug.valueout_flag = 0;
		}

		if(small_gyro->SmallGyro_out.is_small_gyro > small_gyro->SmallGyro_init.is_count_max)
		{
			small_gyro->SmallGyro_out.small_gyro_aimed = 1;
		}
		else
		{
			small_gyro->SmallGyro_out.small_gyro_aimed = 0;
		}
}

void get_vision_info(uint8_t temp[])
{
	if(temp[0] == 0xa0)
	{
		vision_time++;
		memcpy(&(VisionValue.vision_yaw_value.buff[0]),&temp[1],4);
		VisionValue.center_flag = temp[5];
		VisionValue.identify_target = temp[6];
	}
	else if(temp[0] == 0xa1)
	{
		vision_time++;
		memcpy(&(VisionValue.vision_distance.buff[0]),&temp[1],4);	
	}
	else if(temp[0] == 0xa2)
	{
		vision_time++;
		memcpy(&(VisionValue.normal_yaw_value.buff[0]),&temp[1],4);	
	}

}
