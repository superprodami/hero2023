#ifndef _UI_TASK_H_
#define _UI_TASK_H_


#include "main.h"
#include "SQ_judge.h"
#include "fifo.h"

void Info_Transmit_task(void const * argument);
extern int pitch_show;

/* ����ϵͳ����˫������ */
extern uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* ����ϵͳ�������ݶ��� */
extern fifo_s_t Referee_FIFO;
extern uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol�������ṹ�� */
extern unpack_data_t Referee_Unpack_OBJ;
extern void MY_UI_task(void const * argument);














#endif


















//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#ifndef _INFO_TRANSMIT_TASK_H
//#define _INFO_TRANSMIT_TASK_H

//#include "struct_typedef.h"

//extern void Info_Transmit_task(void const * argument);
//void Info_Transmit_task_Init(void);
//void judge_on(void);
//void Info_Transmit_task_Fun(void);
//extern void Judge_Speed(void);

//extern int pitch_show;
//#endif



