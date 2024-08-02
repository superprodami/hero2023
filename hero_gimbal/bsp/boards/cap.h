#ifndef __CAP_H
#define __CAP_H

#include "main.h"

#define CAP_MAX_LEN 100
#define CAP_BUFLEN 14

extern uint8_t Capuse;
typedef struct 
{
	uint8_t Frame_Header1 ;
	uint8_t Frame_Header2 ;
	uint8_t Frame_Len ;
	uint8_t Frame_Verify;
	//功能位
	uint8_t Voltage_Input;
	uint8_t Current_Input;
	uint8_t Voltage_Output;
	uint8_t Current_Output;
	uint8_t Voltage_Cap_Input;
	uint8_t Current_Cap_Input;
	uint8_t power_input;//输入功率
	uint8_t power_cap;//超级电容充电功率
	uint8_t power_output;//输出功率
	uint8_t power_set;
	uint8_t cap_flag;
	uint8_t cap_full_flag;
}frames;



typedef struct 
{
	float Voltage_Input;
	float Current_Input;
	float Voltage_Output;
	float Current_Output;
	float Voltage_Cap_Input;
	float Current_Cap_Input;
	float power_input;
	float power_cap;
	float power_output;
	float power_energy;
	double power_cap_percentage;
	uint8_t power_set;
	uint8_t cap_flag;
	uint8_t cap_full_flag;
}power;


extern power Power;
extern frames Frames;//数据帧 
extern uint8_t cap_mode;
typedef union
{
	uint8_t uc[4];
	float f;
}Float4Byte;

extern uint8_t Cap_rxbuffer[13];
extern uint8_t Cap_txbuffer[3];


extern uint8_t Capuse;
extern uint8_t cap_buf[];


void Frames_init(frames *vframes);
void Cap_buffer_analysis(uint8_t *buff);
void Package_Frame(uint8_t _cap_mode);
void Send_cap_msg(uint8_t _cap_mode);
void Cap_callback_handler(uint8_t *buff);

#endif

