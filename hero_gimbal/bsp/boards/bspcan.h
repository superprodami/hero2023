#ifndef __BSP_CAN
#define __BSP_CAN


#include "stm32f4xx_hal.h"
#include "can.h"
#include "main.h"
#include "type.h"
//#include "mydef.h"

typedef enum
{
    CAN_Moto1_ID = 0x201,
    CAN_Moto2_ID = 0x202,
    CAN_Moto3_ID = 0x203,
    CAN_Moto4_ID = 0x204,

    CAN_Moto5_ID = 0x205,
    CAN_Moto6_ID = 0x206,
    CAN_Moto7_ID = 0x207,
    CAN_Moto8_ID = 0x208,

    //6020电机接收报文stdID：0x2FF,设置电调ID为5-7。反馈报文为：5：0x209 6:0x20A 7:0x20B
    CAN_Moto9_ID = 0x209,
    CAN_Moto10_ID = 0x20A,
    CAN_Moto11_ID = 0x20B,
	
		CAN_DM_Moto1_ID = 0x002,   //达妙拨弹盘
	  CAN_DM_Moto2_ID = 0x003,   //达妙pitch轴

} CAN_Message_ID;

typedef enum
{
  heat_id = 0x320,//发送热量相关标志位
	dbus_ch_id,
	dbus_mouse_id,
	dbus_key_id,
	shoot_speed_id

} receive_id;

typedef struct
{
    union
    {
        uint8_t buff[4];
        float value;
		
    } chassis_yaw_value;
	
} angle_measure_t;

/*CAN发送或是接收的ID*/

#define set_friction1_voltage __HAL_TIM_SetCompare
#define set_friction2_voltage __HAL_TIM_SetCompare

/*接收到的云台电机的参数结构体*/
typedef struct
{
    union
    {
        uint8_t buff[4];
		float value;
    } gimbal_yaw_angle;
	
    union
    {
        uint8_t buff[4];
		float value;
    } gimbal_yaw_current;
	
    union
    {
        uint8_t buff[4];
		float value;
    } ammu_current;

} GimbalValueType;

extern GimbalValueType gimbal_value;

typedef struct
{
    union
    {
        uint8_t buff[4];
        float value;
    } joule_residue;
    union
    {
        uint8_t buff[4];
        float value;
    } power_limit;
	
	union
    {
        uint8_t buff[4];
        float value;
    } cap_joule_residue;
	
	union
    {
        uint8_t buff[4];
        float value;
    } speed_42mm;
	
	union
    {
        uint8_t buff[4];
        float value;
    } speed_42mm_vision;
	
	uint8_t color_judge;

} JudgeValueType;

extern JudgeValueType JudgeValue;

/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t  moto_CAN[11];
extern moto_measure_t moto_CAN2[11];
extern moto_measure_t moto_CAN_DM[3];

extern heat_measure_t heat_judge;
extern shoot_measure_t shoot_judge;
extern uint32_t control_judge_flag;

void my_can_filter_init(CAN_HandleTypeDef* hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id);
void get_moto_measure(moto_measure_t *ptr,uint8_t temp[]);
void get_dm_moto_measure(moto_measure_t *ptr, uint8_t temp[]);

//void get_moto_measure_RMD(moto_measure_t *ptr,uint8_t temp[]);

void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
void set_moto1234_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void set_moto5678_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
extern void set_moto_pitch_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);

extern void set_angle(CAN_HandleTypeDef* hcan, float yaw_angle, bool flag);
extern void get_shoot_info(shoot_measure_t* _heat,uint8_t temp[]);
//extern void set_yaw_ammu_current(CAN_HandleTypeDef* hcan, float yaw_current, float ammu_current);
extern void get_total_angle(moto_measure_t *p);
	
#endif
