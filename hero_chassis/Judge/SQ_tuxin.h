#ifndef __SQ_tuxin_h
#define __SQ_tuxin_h

#include "SQ_judge.h"

/* UI��������cmdID */
#define UI_DataID_Delete   0x100 //�ͻ���ɾ��ͼ��
#define UI_DataID_Draw1    0x101 //�ͻ��˻���1��ͼ��
#define UI_DataID_Draw2    0x102 //�ͻ��˻���2��ͼ��
#define UI_DataID_Draw5    0x103 //�ͻ��˻���5��ͼ��
#define UI_DataID_Draw7    0x104 //�ͻ��˻���7��ͼ��
#define UI_DataID_DrawChar 0x110 //�ͻ��˻����ַ�ͼ��

/* UIɾ������ */
#define UI_Delete_Invalid 0 //�ղ���
#define UI_Delete_Layer   1 //ɾ��ͼ��
#define UI_Delete_All     2 //ɾ������

/* UIͼ�β��� */
#define UI_Graph_invalid 0 //�ղ���
#define UI_Graph_Add     1 //����ͼ��
#define UI_Graph_Change  2 //�޸�ͼ��
#define UI_Graph_Delete  3 //ɾ��ͼ��

/* UIͼ������ */
#define UI_Graph_Line      0 //ֱ��
#define UI_Graph_Rectangle 1 //����
#define UI_Graph_Circle    2 //��Բ
#define UI_Graph_Ellipse   3 //��Բ
#define UI_Graph_Arc       4 //Բ��
#define UI_Graph_Float     5 //������
#define UI_Graph_Int       6 //����
#define UI_Graph_String    7 //�ַ���

/* UIͼ����ɫ */
#define UI_Color_Main   0 //������ɫ
#define UI_Color_Yellow 1 //��ɫ
#define UI_Color_Green  2 //��ɫ
#define UI_Color_Orange 3 //��ɫ
#define UI_Color_Purple 4 //��ɫ
#define UI_Color_Pink   5 //��ɫ
#define UI_Color_Cyan   6 //��ɫ
#define UI_Color_Black  7 //��ɫ
#define UI_Color_White  8 //��ɫ






/* �Զ������UI�ṹ�� -------------------------------------------------------*/
typedef __packed struct  //����UI UIͼ������
{
	uint8_t  graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_data;

typedef __packed struct  //����UI UI�ַ�������
{
	uint8_t  string_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t null;
	uint8_t stringdata[30];
} string_data;

typedef __packed struct  //����UI UIɾ��ͼ������
{
	uint8_t operate_tpye;
	uint8_t layer;
} delete_data_struct_t;

typedef __packed struct //����UI ����1��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data Graphic[1];
	uint16_t CRC16;
} UI_Graph1_t;

typedef __packed struct //����UI ����2��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data Graphic[2];
	uint16_t CRC16;
} UI_Graph2_t;

typedef __packed struct //����UI ����5��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data Graphic[5];
	uint16_t CRC16;
} UI_Graph5_t;

typedef __packed struct //����UI ����7��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data Graphic[7];
	uint16_t CRC16;
} UI_Graph7_t;

typedef __packed struct //����UI ����1�ַ��������ṹ��
{ 
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	string_data String;
	uint16_t CRC16;
} UI_String_t;

typedef __packed struct  //����UI UIɾ��ͼ�������ṹ��
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	delete_data_struct_t Delete;
	uint16_t CRC16;
} UI_Delete_t;


/* ����UIר�ýṹ�� */
extern UI_Graph1_t UI_Graph1;
extern UI_Graph2_t UI_Graph2;
extern UI_Graph5_t UI_Graph5;
extern UI_Graph7_t UI_Graph7;
extern UI_String_t UI_String;
extern UI_Delete_t UI_Delete;


void UI_Draw_Line(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	                char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
									uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
									uint16_t               Width,        //�߿�
									uint16_t               StartX,       //��ʼ����X
									uint16_t               StartY,       //��ʼ����Y
									uint16_t               EndX,         //��ֹ����X
									uint16_t               EndY);        //��ֹ����Y
void UI_Draw_Rectangle(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	                     char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									     uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
							     	   uint16_t               Width,        //�߿�
							     		 uint16_t               StartX,       //��ʼ����X
							     		 uint16_t               StartY,       //��ʼ����Y
							     		 uint16_t               EndX,         //��ֹ����X
							     		 uint16_t               EndY);        //��ֹ����Y
void UI_Draw_Circle(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	                  char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									  uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									  uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										uint16_t               Width,        //�߿�
										uint16_t               CenterX,      //Բ������X
							      uint16_t               CenterY,      //Բ������Y
										uint16_t               Radius);      //�뾶
void UI_Draw_Ellipse(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	                   char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										 uint16_t               Width,        //�߿�
										 uint16_t               CenterX,      //Բ������X
							       uint16_t               CenterY,      //Բ������Y
										 uint16_t               XHalfAxis,    //X���᳤
										 uint16_t               YHalfAxis);   //Y���᳤
void UI_Draw_Arc(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	               char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							   uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
								 uint16_t               StartAngle,   //��ʼ�Ƕ� [0,360]
								 uint16_t               EndAngle,     //��ֹ�Ƕ� [0,360]
								 uint16_t               Width,        //�߿�
								 uint16_t               CenterX,      //Բ������X
							   uint16_t               CenterY,      //Բ������Y
								 uint16_t               XHalfAxis,    //X���᳤
								 uint16_t               YHalfAxis);   //Y���᳤
void UI_Draw_Float(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	                 char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
									 uint16_t               NumberSize,   //�����С
									 uint16_t               Significant,  //��Чλ��
									 uint16_t               Width,        //�߿�
							     uint16_t               StartX,       //��ʼ����X
							     uint16_t               StartY,       //��ʼ����Y
									 float                  FloatData);   //��������
void UI_Draw_Int(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	               char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							   uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
								 uint16_t               NumberSize,   //�����С
								 uint16_t               Width,        //�߿�
							   uint16_t               StartX,       //��ʼ����X
							   uint16_t               StartY,       //��ʼ����Y
								 int32_t                IntData);     //��������
void UI_Draw_String(string_data *String,        //UIͼ�����ݽṹ��ָ��
	                  char                  StringName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							      uint8_t               StringOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								    uint8_t               Layer,         //UIͼ��ͼ�� [0,9]
							      uint8_t               Color,         //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										uint16_t              CharSize,      //�����С
									  uint16_t              StringLength,  //�ַ�������
									  uint16_t              Width,         //�߿�
							      uint16_t              StartX,        //��ʼ����X
							      uint16_t              StartY,        //��ʼ����Y
										char                 *StringData);   //�ַ�������

void UI_PushUp_Graphs(uint8_t Counter, void *Graphs, uint8_t RobotID);
void UI_PushUp_String(UI_String_t *String, uint8_t RobotID);
void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID);

#endif

