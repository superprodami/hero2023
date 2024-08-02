/**
  ******************************************************************************
  * @file    tuxin.c
  * @brief   图传界面画图用于显示状态、辅助瞄准。
  * @author  CHY
  ******************************************************************************
  * @attention
  *
  * 2021.3 设计大致完成。
  * 2021.4.11 暂时还显示不了字符，只能显示图形。已经实现了图形根据车的某些状态进行动态的变化。
  * 暂时还不知道为什么会有这么诡异的事。
  *
  *
  *
  *
  ******************************************************************************
  */
#include "SQ_judge.h"
#include "tuxin.h"
#include "crc.h"
#include "string.h"

judge_show_data_t show_data = {0};
uint32_t UI_number;
uint8_t Tx_buff[5][50] = {0};
uint8_t Tx_buff_seven[9][200] = {0};
uint8_t infantry_sight[9][200] = {0};

//绘图段显示数据
void draw_a_cricle(uint8_t (*txbuff)[50], uint8_t i, int x, int y, uint8_t Operate_type, UART_HandleTypeDef UART)
{
    uint32_t length = 0;

    //按要求帖齑帧头
    length = sizeof(show_data.frame_header) + sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_single.grapic_data_struct) + sizeof(show_data.cmd_id) + sizeof(show_data.frame_tail);
    show_data.frame_header.SOF = Judge_Data_SOF;
    show_data.frame_header.Data_Length = sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_single.grapic_data_struct);
    show_data.frame_header.Seq = 0;
    memcpy(txbuff + i, &show_data.frame_header, LEN_HEADER);
    Append_CRC8_Check_Sum(*(txbuff + i), LEN_HEADER);
    //发送时低八位在前
    show_data.cmd_id = 0x0301;
    memcpy(txbuff[i] + CMD_ID, &show_data.cmd_id, LEN_CMDID);
    show_data.student_interactive_header.data_cmd_id = client_custom_graphic_single_id;
    show_data.student_interactive_header.sender_ID = Judge_Self_ID;
    show_data.student_interactive_header.receiver_ID = determine_ID();
    memcpy(txbuff[i] + STU_HEADER, &show_data.student_interactive_header, LEN_STU_HEAD);
    show_data.operate_data.show_single.grapic_data_struct.graphic_name[0] = 'r';
    show_data.operate_data.show_single.grapic_data_struct.graphic_name[1] = 'o';
    show_data.operate_data.show_single.grapic_data_struct.graphic_name[2] = 'd';
    show_data.operate_data.show_single.grapic_data_struct.operate_type = Operate_type;//图形操作
    show_data.operate_data.show_single.grapic_data_struct.graphic_type = circle;
    show_data.operate_data.show_single.grapic_data_struct.layer = i;//图层
    show_data.operate_data.show_single.grapic_data_struct.color = 1;
    show_data.operate_data.show_single.grapic_data_struct.width = 5;//线条宽度
    show_data.operate_data.show_single.grapic_data_struct.start_x = x;
    show_data.operate_data.show_single.grapic_data_struct.start_y =	y;
    show_data.operate_data.show_single.grapic_data_struct.radius = 30;//半径

    memcpy(txbuff[i] + STU_DATA, &show_data.operate_data.show_single.grapic_data_struct, LEN_SINGLE_GRAPH);
    Append_CRC16_Check_Sum(*(txbuff + i), length);
    HAL_UART_Transmit(&UART, *(txbuff + i), length,5);
    memset(txbuff + i, 0, 50);
}

void Delete_All(uint8_t (*txbuff)[50],uint8_t i, UART_HandleTypeDef UART)
{
    uint32_t length = 0;
    //按要求帖齑帧头
    length = sizeof(show_data.frame_header) + sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_single.grapic_data_struct) + sizeof(show_data.cmd_id) + sizeof(show_data.frame_tail);
    show_data.frame_header.SOF = Judge_Data_SOF;
    show_data.frame_header.Data_Length = sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_single.grapic_data_struct);
    show_data.frame_header.Seq = 0;
    memcpy(txbuff + i, &show_data.frame_header, LEN_HEADER);
    Append_CRC8_Check_Sum(*(txbuff + i), LEN_HEADER);
    //发送时低八位在前
    show_data.cmd_id = 0x0301;
    memcpy(txbuff[i] + CMD_ID, &show_data.cmd_id, LEN_CMDID);
    show_data.student_interactive_header.data_cmd_id = client_custom_graphic_delete_id;
    show_data.student_interactive_header.sender_ID = Judge_Self_ID;
    show_data.student_interactive_header.receiver_ID = determine_ID();
    memcpy(txbuff[i] + STU_HEADER, &show_data.student_interactive_header, LEN_STU_HEAD);

    show_data.operate_data.client_custom_graphic_delete.operate_type = 2;
    show_data.operate_data.client_custom_graphic_delete.layer=9;

    memcpy(txbuff[i] + STU_DATA, &show_data.operate_data.client_custom_graphic_delete, 2);
    Append_CRC16_Check_Sum(*(txbuff + i), length);
    HAL_UART_Transmit(&UART, *(txbuff + i), length,50);
    memset(txbuff + i, 0, 50);
}

void draw_a_line(uint8_t (*txbuff)[50], uint8_t i, int Start_x, int Start_y, int End_x, int End_y, uint8_t Operate_type, UART_HandleTypeDef UART)
{
    uint32_t length = 0;
    //按要求帖齑帧头
    length = sizeof(show_data.frame_header) + sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_single.grapic_data_struct) + sizeof(show_data.cmd_id) + sizeof(show_data.frame_tail);
    show_data.frame_header.SOF = Judge_Data_SOF;
    show_data.frame_header.Data_Length = sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_single.grapic_data_struct);
    show_data.frame_header.Seq = 0;
    memcpy(txbuff + i, &show_data.frame_header, LEN_HEADER);
    Append_CRC8_Check_Sum(*(txbuff + i), LEN_HEADER);
    //发送时低八位在前
    show_data.cmd_id = 0x0301;
    memcpy(txbuff[i] + CMD_ID, &show_data.cmd_id, LEN_CMDID);
    show_data.student_interactive_header.data_cmd_id = client_custom_graphic_single_id;
    show_data.student_interactive_header.sender_ID = Judge_Self_ID;
    show_data.student_interactive_header.receiver_ID = determine_ID();
    memcpy(txbuff[i] + STU_HEADER, &show_data.student_interactive_header, LEN_STU_HEAD);
    show_data.operate_data.show_single.grapic_data_struct.graphic_name[0] = 'r';
    show_data.operate_data.show_single.grapic_data_struct.graphic_name[1] = 'o';
    show_data.operate_data.show_single.grapic_data_struct.graphic_name[2] = 'd';
    show_data.operate_data.show_single.grapic_data_struct.operate_type = Operate_type;//图形操讈E
    show_data.operate_data.show_single.grapic_data_struct.graphic_type = straight_line;
    show_data.operate_data.show_single.grapic_data_struct.layer = i;//图瞾E
    show_data.operate_data.show_single.grapic_data_struct.color = 1;//1黄色
    show_data.operate_data.show_single.grapic_data_struct.width = 5;//线条窥胰
    show_data.operate_data.show_single.grapic_data_struct.start_x = Start_x;
    show_data.operate_data.show_single.grapic_data_struct.start_y =	Start_y;
    show_data.operate_data.show_single.grapic_data_struct.end_x = End_x;//10;//皝E?
    show_data.operate_data.show_single.grapic_data_struct.end_y = End_y;//10;//

    memcpy(txbuff[i] + STU_DATA, &show_data.operate_data.show_single.grapic_data_struct, LEN_SINGLE_GRAPH);
    Append_CRC16_Check_Sum(*(txbuff + i), length);
    HAL_UART_Transmit(&UART, *(txbuff + i), length,5);
    memset(txbuff + i, 0, 50);
}



void show_str(uint8_t str[],uint8_t len,uint8_t layer, uint16_t start_x,uint16_t start_y,color_type color, operate_type operate, UART_HandleTypeDef UART)
{
	uint32_t length = 0;
	uint8_t txbuff[200] = {0};
		//按要求填写帧头
	length = sizeof(show_data.frame_header) + sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_single.grapic_data_struct) + 30 + sizeof(show_data.cmd_id) + sizeof(show_data.frame_tail);
	show_data.frame_header.SOF = Judge_Data_SOF;
	show_data.frame_header.Data_Length = sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_char);
	show_data.frame_header.Seq = 0;
	memcpy(txbuff,&show_data.frame_header,LEN_HEADER);
	Append_CRC8_Check_Sum(txbuff,LEN_HEADER);
	
	show_data.cmd_id = 0x0301;
	memcpy(txbuff + CMD_ID,&show_data.cmd_id,LEN_CMDID);
	show_data.student_interactive_header.data_cmd_id = client_custom_character_id;
  show_data.student_interactive_header.sender_ID = Judge_Self_ID;//当前机器人id;
  show_data.student_interactive_header.receiver_ID = determine_ID();//该机器人对应的客户端id
	memcpy(txbuff + STU_HEADER,&show_data.student_interactive_header,LEN_STU_HEAD);
	show_data.operate_data.show_char.grapic_data_struct.graphic_name[0] = str[0];
	show_data.operate_data.show_char.grapic_data_struct.graphic_name[1] = str[1];
	show_data.operate_data.show_char.grapic_data_struct.graphic_name[2] = str[2];
	show_data.operate_data.show_char.grapic_data_struct.operate_type = operate;
	show_data.operate_data.show_char.grapic_data_struct.graphic_type = string;
	show_data.operate_data.show_char.grapic_data_struct.layer = layer;
	show_data.operate_data.show_char.grapic_data_struct.color = color;
	show_data.operate_data.show_char.grapic_data_struct.start_angle = 20;	//字体大小
	show_data.operate_data.show_char.grapic_data_struct.end_angle = len;	//字体长度
	show_data.operate_data.show_char.grapic_data_struct.width = 3;
	show_data.operate_data.show_char.grapic_data_struct.start_x = start_x;
	show_data.operate_data.show_char.grapic_data_struct.start_y = start_y;
	
	for(int i = 0; i < len; i++)
	{
		show_data.operate_data.show_char.data[i] = str[i];
	}
	
	memcpy(txbuff + STU_DATA ,&show_data.operate_data.show_char.grapic_data_struct, LEN_SINGLE_GRAPH + 30);
	Append_CRC16_Check_Sum(txbuff,length);
	HAL_UART_Transmit(&UART,txbuff,length,0xffff);	
	memset(txbuff,0,sizeof(txbuff));
	for(int i = 0; i < len; i++)
	{
		show_data.operate_data.show_char.data[i] = ' ';
	}
}

void draw_seven_line(uint8_t (*txbuff)[200], uint8_t i, UART_HandleTypeDef UART, uint32_t (*Data)[12])
{
    uint32_t length = 0;
    //按要求帖齑帧头
    length = sizeof(show_data.frame_header) + sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_seven.grapic_data_struct) + sizeof(show_data.cmd_id) + sizeof(show_data.frame_tail);
    show_data.frame_header.SOF = Judge_Data_SOF;
    show_data.frame_header.Data_Length = sizeof(show_data.student_interactive_header) + sizeof(show_data.operate_data.show_seven.grapic_data_struct);
    show_data.frame_header.Seq = 0;
    memcpy(txbuff + i, &show_data.frame_header, LEN_HEADER);
    Append_CRC8_Check_Sum(*(txbuff + i), LEN_HEADER);
    //发送时低八位在前
    show_data.cmd_id = 0x0301;
    memcpy(txbuff[i] + CMD_ID, &show_data.cmd_id, LEN_CMDID);
    show_data.student_interactive_header.data_cmd_id = client_custom_graphic_seven_id;
    show_data.student_interactive_header.sender_ID = Judge_Self_ID;
    show_data.student_interactive_header.receiver_ID = determine_ID();
    memcpy(txbuff[i] + STU_HEADER, &show_data.student_interactive_header, LEN_STU_HEAD);
    for(int j = 0; j < 7; j++)
    {
        show_data.operate_data.show_seven.grapic_data_struct[j].graphic_name[0] = *(Data[j]+7);
        show_data.operate_data.show_seven.grapic_data_struct[j].graphic_name[1] = *(Data[j]+8);
        show_data.operate_data.show_seven.grapic_data_struct[j].graphic_name[2] = j;
        show_data.operate_data.show_seven.grapic_data_struct[j].operate_type = *(Data[j]+0);//Operate_type;//图形操讈E
        show_data.operate_data.show_seven.grapic_data_struct[j].graphic_type = *(Data[j]+1);//;//图形类型
        show_data.operate_data.show_seven.grapic_data_struct[j].layer = *(Data[j]+2);//5;//图瞾E
        show_data.operate_data.show_seven.grapic_data_struct[j].color = *(Data[j]+3);//1;//颜色
        show_data.operate_data.show_seven.grapic_data_struct[j].start_angle = *(Data[j]+4);//10;//
        show_data.operate_data.show_seven.grapic_data_struct[j].end_angle = *(Data[j]+5);//10;//
        show_data.operate_data.show_seven.grapic_data_struct[j].width = *(Data[j]+6);//10;//线条宽度
        show_data.operate_data.show_seven.grapic_data_struct[j].start_x = *(Data[j]+7);//x;
        show_data.operate_data.show_seven.grapic_data_struct[j].start_y =	*(Data[j]+8);//y;
        show_data.operate_data.show_seven.grapic_data_struct[j].radius = *(Data[j]+9);//10;//半径
        show_data.operate_data.show_seven.grapic_data_struct[j].end_x = *(Data[j]+10);//10;//结束x
        show_data.operate_data.show_seven.grapic_data_struct[j].end_y = *(Data[j]+11);//10;//皝E?
    }

    //show_single
    memcpy(txbuff[i] + STU_DATA, &show_data.operate_data.show_seven.grapic_data_struct, LEN_SEVEN_GRAPH);
    Append_CRC16_Check_Sum(*(txbuff + i), length);
    HAL_UART_Transmit(&UART, *(txbuff + i), length,50);
    memset(txbuff + i, 0, 200);
}
//operate_type graphic_type layer color start_angle end_angle width start_x start_y radius end_x end_y
uint32_t Change_Number[7][12]={//数值
	{add,integer,5,yellow,50,0,5,280,830,0,0,0},
  {add,integer,5,yellow,50,0,5,1650,830,0,0,0}
};
uint32_t Change_capacitance[7][12]={//电容
												{add,straight_line,3,yellow,0,0,40,100,800,10,120,800},
												{add,straight_line,3,yellow,0,0,40,130,800,10,150,800},
												{add,straight_line,3,yellow,0,0,40,160,800,10,180,800},
												{add,straight_line,3,yellow,0,0,40,190,800,10,210,800},
												{add,straight_line,3,yellow,0,0,40,220,800,10,240,800},
												{add,rectangle,3,yellow,0,0,5,90,830,10,250,770},
};
uint32_t Data_S[7][12]={//S:
												{add,straight_line,8,yellow,0,0,5,100,700,10,70,700},
												{add,straight_line,8,yellow,0,0,5,70,700,10,70,670},
												{add,straight_line,8,yellow,0,0,5,70,670,10,100,670},
												{add,straight_line,8,yellow,0,0,5,100,670,10,100,640},
												{add,straight_line,8,yellow,0,0,5,100,640,10,70,640},
												{add,circle,8,yellow,0,0,5,130,685,5,0,0},
												{add,circle,8,yellow,0,0,5,130,655,5,0,0}
};
uint32_t Data_P[7][12]={//P:
												{add,straight_line,8,yellow,0,0,5,70,590,10,100,590},
												{add,straight_line,8,yellow,0,0,5,100,590,10,100,560},
												{add,straight_line,8,yellow,0,0,5,100,560,10,70,560},
												{add,straight_line,8,yellow,0,0,5,70,590,10,70,530},
												{add,circle,8,yellow,0,0,5,130,575,5,0,0},
												{add,circle,8,yellow,0,0,5,130,545,5,0,0}
};
uint32_t Data_S_OP[7][12]={//OP
												{add,rectangle,7,yellow,0,0,5,160,700,10,190,640},
												{add,straight_line,7,yellow,0,0,5,220,700,10,250,700},
												{add,straight_line,7,yellow,0,0,5,250,700,10,250,670},
												{add,straight_line,7,yellow,0,0,5,250,670,10,220,670},
												{add,straight_line,7,yellow,0,0,5,220,700,10,220,640}
};
uint32_t Data_S_CE[7][12]={//CE
												{add,straight_line,7,yellow,0,0,5,160,700,10,190,700},
												{add,straight_line,7,yellow,0,0,5,160,700,10,160,640},
												{add,straight_line,7,yellow,0,0,5,160,640,10,190,640},
												{add,straight_line,7,yellow,0,0,5,220,700,10,250,700},
												{add,straight_line,7,yellow,0,0,5,220,700,10,220,640},
												{add,straight_line,7,yellow,0,0,5,220,670,10,250,670},
												{add,straight_line,7,yellow,0,0,5,220,640,10,250,640}
};
uint32_t Data_P_OP[7][12]={//OP
												{add,rectangle,6,yellow,0,0,5,160,590,10,190,530},
												{add,straight_line,6,yellow,0,0,5,220,590,10,250,590},
												{add,straight_line,6,yellow,0,0,5,250,590,10,250,560},
												{add,straight_line,6,yellow,0,0,5,250,560,10,220,560},
												{add,straight_line,6,yellow,0,0,5,220,590,10,220,530}
};
uint32_t Data_P_CE[7][12]={//CE
												{add,straight_line,6,yellow,0,0,5,160,590,10,190,590},
												{add,straight_line,6,yellow,0,0,5,160,590,10,160,530},
												{add,straight_line,6,yellow,0,0,5,160,530,10,190,530},
												{add,straight_line,6,yellow,0,0,5,220,590,10,250,590},
												{add,straight_line,6,yellow,0,0,5,220,590,10,220,530},
												{add,straight_line,6,yellow,0,0,5,220,560,10,250,560},
												{add,straight_line,6,yellow,0,0,5,220,530,10,250,530}
};
uint32_t Change_arrow0[7][12]= {{add,straight_line,2,yellow,0,0,4,1600,730,10,1600,940},
                               {add,straight_line,2,yellow,0,0,4,1530,800,10,1670,800}//上
};//指针
uint32_t Change_arrow1[7][12]= {{add,straight_line,2,yellow,0,0,20,1500,800,10,1500,800}
																																												//上
};

uint32_t infantry_sight_data[7][12] = {{add,straight_line,2,yellow,0,0,4,1550,750,10,1700,900},
                                      {add,straight_line,2,yellow,0,0,4,1530,800,10,1670,800},
									  {add,straight_line,2,yellow,0,0,4,1530,800,10,1670,800},
};
uint32_t shoot[7][12]  = {
		{add,straight_line,0,white,0,0,1,974,520,0,974,100},//瞄准线
        {add,straight_line,0,yellow,0,0,1,949,420,0,999,420},//敌方吊射点打基地
		{add,straight_line,0,yellow,0,0,1,949,380,0,999,380},//环高打前哨站
		{add,straight_line,0,yellow,0,0,1,949,325,0,999,325},//狙击点打前哨站
        {add,straight_line,0,green ,0,0,1,949,350,0,999,350},//吊射基地
				{add,straight_line,0,green ,0,0,1,949,400,0,999,400},//吊射基地
    };
void Line_of_sight(UART_HandleTypeDef UART)
{
    uint32_t Data[7][12]= {{add,straight_line,0,white,0,0,4,960,520,0,960,100},//瞄准线
        {add,straight_line,0,white,0,0,4,940,500,0,980,500},
        {add,straight_line,0,white,0,0,4,940,460,0,980,460},
        {add,straight_line,0,yellow,0,0,4,940,420,0,980,420},
		{add,straight_line,0,yellow,0,0,4,940,420,0,980,420},
        {add,straight_line,0,yellow,0,0,4,950,230,0,970,230}

    };
    draw_seven_line(Tx_buff_seven,0,UART,Data);
		draw_seven_line(Tx_buff_seven,2,UART,Change_arrow1);
		draw_seven_line(Tx_buff_seven,5,UART,Change_Number);
		draw_seven_line(Tx_buff_seven,3,UART,Change_capacitance);//电容
		draw_seven_line(Tx_buff_seven,4,UART,Data_P);
		draw_seven_line(Tx_buff_seven,4,UART,Data_S);
}

