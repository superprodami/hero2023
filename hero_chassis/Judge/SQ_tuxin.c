/**
  ******************************************************************************
  * @file    tuxin.c
  * @brief   ͼ�����滭ͼ������ʾ״̬��������׼��
  * @author  CHY
  ******************************************************************************
  * @attention
  *
  * 2021.3 ��ƴ�����ɡ�
  * 2021.4.11 ��ʱ����ʾ�����ַ���ֻ����ʾͼ�Ρ��Ѿ�ʵ����ͼ�θ��ݳ���ĳЩ״̬���ж�̬�ı仯��
  * ��ʱ����֪��Ϊʲô������ô������¡�
  *
  *
  *
  *
  ******************************************************************************
  */
#include "SQ_judge.h"
#include "SQ_tuxin.h"
#include "CRCs.h"
#include "string.h"
#include "usart.h"


/* ����UIר�ýṹ�� */
UI_Graph1_t UI_Graph1;
UI_Graph2_t UI_Graph2;
UI_Graph5_t UI_Graph5;
UI_Graph7_t UI_Graph7;
UI_String_t UI_String;
UI_Delete_t UI_Delete;



/*==============================================================================
              ##### UI����ͼ�λ��ƺ��� #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ֱ�� UI_Draw_Line
      (+) ���ƾ��� UI_Draw_Rectangle
      (+) ������Բ UI_Draw_Circle
      (+) ������Բ UI_Draw_Ellipse
      (+) ����Բ�� UI_Draw_Arc
      (+) ����С�� UI_Draw_Float
      (+) �������� UI_Draw_Int
      (+) �����ַ� UI_Draw_String
*/



void UI_Draw_Line(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	                char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
									uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
									uint16_t               Width,        //�߿�
									uint16_t               StartX,       //��ʼ����X
									uint16_t               StartY,       //��ʼ����Y
									uint16_t               EndX,         //��ֹ����X
									uint16_t               EndY)         //��ֹ����Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Line;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
}

void UI_Draw_Rectangle(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	                     char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									     uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
							     	   uint16_t               Width,        //�߿�
							     		 uint16_t               StartX,       //��ʼ����X
							     		 uint16_t               StartY,       //��ʼ����Y
							     		 uint16_t               EndX,         //��ֹ����X
							     		 uint16_t               EndY)         //��ֹ����Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Rectangle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
}

void UI_Draw_Circle(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	                  char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									  uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									  uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										uint16_t               Width,        //�߿�
										uint16_t               CenterX,      //Բ������X
							      uint16_t               CenterY,      //Բ������Y
										uint16_t               Radius)       //�뾶
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Circle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->radius          = Radius;
}

void UI_Draw_Ellipse(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	                   char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										 uint16_t               Width,        //�߿�
										 uint16_t               CenterX,      //Բ������X
							       uint16_t               CenterY,      //Բ������Y
										 uint16_t               XHalfAxis,    //X���᳤
										 uint16_t               YHalfAxis)    //Y���᳤
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Ellipse;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

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
								 uint16_t               YHalfAxis)    //Y���᳤
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Arc;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = StartAngle;
	Graph->end_angle       = EndAngle;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

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
									 float                  FloatData)    //��������
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Float;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = NumberSize;
	Graph->end_angle       = Significant;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	int32_t IntData = FloatData * 1000;
	Graph->radius          = (IntData & 0x000003ff) >>  0;
	Graph->end_x           = (IntData & 0x001ffc00) >> 10;
	Graph->end_y           = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_Int(graphic_data *Graph,        //UIͼ�����ݽṹ��ָ��
	               char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							   uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
								 uint16_t               NumberSize,   //�����С
								 uint16_t               Width,        //�߿�
							   uint16_t               StartX,       //��ʼ����X
							   uint16_t               StartY,       //��ʼ����Y
								 int32_t                IntData)      //��������
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Int;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = NumberSize;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->radius          = (IntData & 0x000003ff) >>  0;
	Graph->end_x           = (IntData & 0x001ffc00) >> 10;
	Graph->end_y           = (IntData & 0xffe00000) >> 21;
}

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
										char                 *StringData)    //�ַ�������
{
	String->string_name[0] = StringName[0];
	String->string_name[1] = StringName[1];
	String->string_name[2] = StringName[2];
	String->operate_tpye   = StringOperate;
	String->graphic_tpye   = UI_Graph_String;
	String->layer          = Layer;
	String->color          = Color;
	String->start_angle    = CharSize;
	String->end_angle      = StringLength;
	String->width          = Width;
	String->start_x        = StartX;
	String->start_y        = StartY;
	for(int i = 0; i < StringLength; i ++) String->stringdata[i] = *StringData ++;
}

/*==============================================================================
              ##### UI����ͼ�����ͺ��� #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ͼ�� UI_PushUp_Graphs
			(+) �����ַ� UI_PushUp_String
			(+) ɾ��ͼ�� UI_PushUp_Delete
*/
void UI_PushUp_Graphs(uint8_t Counter /* 1,2,5,7 */, void *Graphs /* ��Counter��һ�µ�UI_Graphx�ṹ��ͷָ�� */, uint8_t RobotID)
{
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; //����ֻ��һ������ͼ��
	
	/* ��� frame_header */
	Graph->Referee_Transmit_Header.SOF  = HEADER_SOF;
	     if(Counter == 1) Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
	else if(Counter == 2) Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
	else if(Counter == 5) Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
	else if(Counter == 7) Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
	Graph->Referee_Transmit_Header.seq  = Graph->Referee_Transmit_Header.seq + 1;
	Graph->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Graph->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	     if(Counter == 1) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
	else if(Counter == 2) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
	else if(Counter == 5) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
	else if(Counter == 7) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;
	Graph->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	Graph->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	     if(Counter == 1)
	{
		UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
		Graph1->CRC16 = CRC16_Calculate((uint8_t *)Graph1, sizeof(UI_Graph1_t) - 2);
	}
	else if(Counter == 2)
	{
		UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
		Graph2->CRC16 = CRC16_Calculate((uint8_t *)Graph2, sizeof(UI_Graph2_t) - 2);
	}
	else if(Counter == 5)
	{
		UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
		Graph5->CRC16 = CRC16_Calculate((uint8_t *)Graph5, sizeof(UI_Graph5_t) - 2);
	}
	else if(Counter == 7)
	{
		UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
		Graph7->CRC16 = CRC16_Calculate((uint8_t *)Graph7, sizeof(UI_Graph7_t) - 2);
	}
	
	/* ʹ�ô���PushUp������ϵͳ */
	     if(Counter == 1) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph1_t));
	else if(Counter == 2) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph2_t));
	else if(Counter == 5) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph5_t));
	else if(Counter == 7) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph7_t));
}





void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
	/* ��� frame_header */
	String->Referee_Transmit_Header.SOF  = HEADER_SOF;
	String->Referee_Transmit_Header.data_length = 6 + 45;
	String->Referee_Transmit_Header.seq  = String->Referee_Transmit_Header.seq + 1;
	String->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&String->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
	String->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	String->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	String->CRC16 = CRC16_Calculate((uint8_t *)String, sizeof(UI_String_t) - 2);
	
	/* ʹ�ô���PushUp������ϵͳ */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)String, sizeof(UI_String_t));
}

void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
	/* ��� frame_header */
	Delete->Referee_Transmit_Header.SOF  = HEADER_SOF;
	Delete->Referee_Transmit_Header.data_length = 6 + 2;
	Delete->Referee_Transmit_Header.seq  = Delete->Referee_Transmit_Header.seq + 1;
	Delete->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Delete->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
	Delete->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	Delete->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	Delete->CRC16 = CRC16_Calculate((uint8_t *)Delete, sizeof(UI_Delete_t) - 2);
	
	/* ʹ�ô���PushUp������ϵͳ */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Delete, sizeof(UI_Delete_t));
}





