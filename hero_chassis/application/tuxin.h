#ifndef __tuxin_h
#define __tuxin_h

#include "SQ_judge.h"

extern uint8_t Judge_Self_ID;
extern uint8_t Tx_buff[5][50];
extern uint8_t Tx_buff_seven[9][200];
extern uint8_t infantry_sight[9][200];
extern uint32_t infantry_sight_data[7][12];

extern uint32_t shoot[7][12];

extern uint32_t Change_arrow0[7][12];
extern uint32_t Change_arrow1[7][12];

extern uint32_t Change_capacitance[7][12];
extern uint32_t Data_P[7][12];
extern uint32_t Data_S[7][12];
extern uint32_t Data_S_OP[7][12];
extern uint32_t Data_S_CE[7][12];
extern uint32_t Data_P_OP[7][12];
extern uint32_t Data_P_CE[7][12];
extern uint32_t UI_number;
extern uint32_t Change_Number[7][12];
typedef enum
{
    add = 1,								//����
    modify = 2,							//�޸�
    Delete = 3 						  //ɾ��
} operate_type;


typedef enum
{
    straight_line = 0,				//ֱ��
    rectangle,								//����
    circle,										//��Բ
    oval,											//��Բ
    arc,											//Բ��
    floating,									//������
    integer,									//����
    string										//�ַ�E
} graphic_type;

typedef enum
{
    red_blue = 0,				//��E����?
    yellow,						//��ɫ
    green,							//��ɫ
    orange,							//��ɫ
    zihong,							//�Ϻ�E
    pink,								//��ɫ
    qing,								//��ɫ
    black,							//��ɫ
    white								//��ɫ
} color_type;

//��Ӧ�ͻ���ɾ��ͼ��
typedef __packed struct
{
    uint8_t operate_type;			//ͼ�β�ׁE
    uint8_t layer;						//ͼ����
} ext_client_custom_graphic_delete_t;


//ͼ������
typedef __packed struct
{
    uint8_t graphic_name[3];		//ͼ��ÁE
    uint32_t operate_type: 3;		//ͼ�β�����
    uint32_t graphic_type: 3;		//ͼ������
    uint32_t layer: 4;						//ͼ����
    uint32_t color: 4;						//��ɫ
    uint32_t start_angle: 9;			//��ʼ�Ƕ�
    uint32_t end_angle: 9;				//��ֹ�Ƕ�
    uint32_t width: 10;					//�������
    uint32_t start_x: 11;				//���x����E
    uint32_t start_y: 11;				//���y����E
    uint32_t radius: 10;					//�����С��뾶
    uint32_t end_x: 11;					//�յ�x����E
    uint32_t end_y: 11;					//�յ�y����E
} graphic_data_struct_t;



//�ͻ��˻���һ��ͼ�� ���ݵ����� ID��0x0101
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

//�ͻ��˻�������ͼ��	���ݵ����� ID��0x0102
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

//�ͻ��˻������ͼ��  ���ݵ����� ID��0x0103
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

//�ͻ��˻����߸�ͼ��  ���ݵ����� ID��0x0104
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//�ͻ��˻����ַ�E	���ݵ����� ID��0x0110
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

typedef __packed struct
{
    ext_client_custom_graphic_delete_t client_custom_graphic_delete; 	//�ͻ���ͼ��ɾ��
    ext_client_custom_graphic_single_t show_single;										//�ͻ��˻���һ��ͼ��
    ext_client_custom_graphic_double_t show_double;										//�ͻ��˻�������ͼ��
    ext_client_custom_graphic_five_t   show_five;											//�ͻ��˻������ͼ��
    ext_client_custom_graphic_seven_t  show_seven;                    //�ͻ��˻����߸�ͼ��
    ext_client_custom_character_t      show_char;											//�ͻ��˻����ַ�E
} operate_data_t;

typedef struct
{
    frame_header_t frame_header;																				//֡ͷ
    uint16_t cmd_id;																										//ÁE�ށE
    ext_student_interactive_header_data_t student_interactive_header; 	//���ݶ�ͷ�ṹ
    operate_data_t operate_data;																				//������������
    uint16_t frame_tail;																								//֡β��16λcrcУ�飩
} judge_show_data_t;

typedef struct
{
    frame_header_t frame_header;
    uint16_t cmd_id;
    ext_client_custom_graphic_delete_t  ext_client_custom_graphic_delete;
} graphic_data_delete_t;

void draw_a_line(uint8_t (*txbuff)[50],uint8_t i,int Start_x, int Start_y,int End_x, int End_y,uint8_t Operate_type, UART_HandleTypeDef UART);
void draw_a_cricle(uint8_t (*txbuff)[50],uint8_t i,int x, int y,uint8_t Operate_type, UART_HandleTypeDef UART);
void draw_a_string(uint8_t (*txbuff)[50],uint8_t i,uint8_t *str,uint8_t len, uint16_t start_x,uint16_t start_y, uint8_t Operate_type, UART_HandleTypeDef UART);
void show_str(uint8_t str[],uint8_t len,uint8_t layer, uint16_t start_x,uint16_t start_y,color_type color, operate_type operate, UART_HandleTypeDef UART);

void Delete_All(uint8_t (*txbuff)[50],uint8_t i, UART_HandleTypeDef UART);
void draw_seven_line(uint8_t (*txbuff)[200], uint8_t i, UART_HandleTypeDef UART, uint32_t (*Data)[12]);

void draw_seven_cricle(uint8_t (*txbuff)[200],uint8_t i,UART_HandleTypeDef UART,uint32_t (*Data)[12]);
void Line_of_sight(UART_HandleTypeDef UART);
void shoot_show(UART_HandleTypeDef UART);
#endif

