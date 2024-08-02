#ifndef __CAP_H
#define __CAP_H

#include "main.h"


#define CAP_SWITCH_CLOSE     0x01
#define CAP_SWITCH_OPEN      0x02
#define CAP_SWITCH_SLOPE     0x03

#define CAP_STATUS_WORK      0xaa
#define CAP_STATUS_OFFWORK   0xbb
#define CAP_STATUS_UNUSUAL   0xcc

#define CAP_STATUS_FLAG      0x01

#define CAP_NOMAL  0x01
#define CAP_SLOPE   0x02

extern uint8_t robot_status; 
extern uint8_t cap_switch;

typedef struct{
	uint8_t cap_status;
	uint8_t switch_status;
	float cap_joule_residue;
	uint8_t cap_mode;
	uint32_t cap_step;
}CapInfo;

extern CapInfo cap_info;
extern JudgeValueType JudgeValue;
extern float cap_percent;

extern void Send_cap_msg(void);
extern void Cap_callback_handler(uint8_t *buff);

extern void set_cap0(CAN_HandleTypeDef* hcan, s8 robot_status, s8 cap_switch, float joule_residue);
extern void set_cap1(CAN_HandleTypeDef* hcan, s8 robot_status, s8 cap_switch, float power_limit);
extern void cap_init(void);

extern void get_cap_info(uint8_t temp[]);

#endif

