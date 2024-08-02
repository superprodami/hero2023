#include "Info_Transmit_task.h"
#include "SQ_tuxin.h"
#include "cmsis_os.h"
#include "SQ_judge.h"
#include "vision.h"



int pitch_show = 0;



uint8_t Robot_ID_Current = Robot_ID_Red_Hero;

/* 中央标尺高度变量 */
uint16_t y01 = 555;
uint16_t x01 = 975;
uint16_t with01 = 80;

uint16_t y02 = 430;
uint16_t x02 = 975;
uint16_t with02 = 40;
//uint16_t y03 = 280;
//uint16_t y04 = 230;

/* 连杆坐标左上角原点*/
uint16_t xE = 100;
uint16_t yE = 750;
/* 连杆缩放倍率*/
float multi = 500.f;


/* 底盘角度中心点坐标*/
uint16_t xChassis = 120;
uint16_t yChassis = 550;

/*速度挡位提示左下角坐标*/
uint16_t xSpeed = 1500;
uint16_t ySpeed = 500;
uint16_t LongSpeed = 200;


void Info_Transmit_task(void const * argument)
{
	/* 动态UI控制变量 */
	uint16_t UI_PushUp_Counter = 261;
	float    Capacitance_X;
	
	/* 裁判系统初始化 */
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		
		
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
    
		Robot_ID_Current = Game_Robot_State.robot_id;
		
		/* UI更新 */
		UI_PushUp_Counter++;
		if(UI_PushUp_Counter % 321 == 0) //添加 小陀螺预警线 枪管指向
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  630,   30,  780,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[1], "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  780,  100,  930,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[2], "103", UI_Graph_Add, 1, UI_Color_Yellow, 2,  990,  100, 1140,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[3], "104", UI_Graph_Add, 1, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
			UI_Draw_Line(&UI_Graph7.Graphic[4], "105", UI_Graph_Add, 1, UI_Color_Yellow, 5,  959,  100,  960,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[5], "200", UI_Graph_Add, 1, UI_Color_Pink,   6,  xChassis,   yChassis,   xChassis, yChassis+50); //枪管方向
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		else if(UI_PushUp_Counter % 321 == 4) //刷新 小陀螺预警线 枪管指向
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "101", UI_Graph_Change, 1, UI_Color_Yellow, 2,  630,   30,  780,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[1], "102", UI_Graph_Change, 1, UI_Color_Yellow, 2,  780,  100,  930,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[2], "103", UI_Graph_Change, 1, UI_Color_Yellow, 2,  990,  100, 1140,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[3], "104", UI_Graph_Change, 1, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
			UI_Draw_Line(&UI_Graph7.Graphic[4], "105", UI_Graph_Change, 1, UI_Color_Yellow, 5,  959,  100,  960,  100);
			UI_Draw_Line(&UI_Graph7.Graphic[5], "200", UI_Graph_Change, 1, UI_Color_Pink,   6,  xChassis,   yChassis,   xChassis, yChassis+50); //枪管方向
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		 
		float angle_error;
//		if(UI_PushUp_Counter % 341 == 0) //添加连杆位置 
//		{
//			uint8_t colour;
//			if(fabsf(Chassis_MSG.state_vector.Dip_Angle) >0.3f)//超过15度认为倒下，此时连杆显示橙色
//			{
//				colour = UI_Color_Orange;
//			}
//			else
//			{
//				colour = UI_Color_Green;
//			}
//			if(DJM_MSG[0].angle <= HEAD_ENCODER)angle_error = HEAD_ENCODER - DJM_MSG[0].angle;
//			else angle_error = 8192 - (DJM_MSG[0].angle - HEAD_ENCODER);
//			angle_error = angle_error/8192.f*360.f;
//			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, colour, 3,  xE                                  ,   yE                              ,   xE+L5*multi                         ,   yE-0*multi                       ); //最上方横线 EA
//			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, colour, 3,  xE                                  ,   yE                              ,   xE+(-Chassis_MSG.L_Rod.Xd+L5)*multi ,   yE-(Chassis_MSG.L_Rod.Yd)*multi  ); //左上方连杆 ED
//			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, colour, 3,  xE+(-Chassis_MSG.L_Rod.Xd+L5)*multi ,   yE-(Chassis_MSG.L_Rod.Yd)*multi ,   xE+(-Chassis_MSG.L_Rod.X+L5)*multi  ,   yE-(Chassis_MSG.L_Rod.Y)*multi   ); //左下方连杆 DC
//			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, colour, 3,  xE+L5*multi                         ,   yE-0*multi                      ,   xE+(-Chassis_MSG.L_Rod.Xb+L5)*multi ,   yE-(Chassis_MSG.L_Rod.Yb)*multi  ); //右上方连杆 AB
//			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, colour, 3,  xE+(-Chassis_MSG.L_Rod.Xb+L5)*multi ,   yE-(Chassis_MSG.L_Rod.Yb)*multi ,   xE+(-Chassis_MSG.L_Rod.X+L5)*multi  ,   yE-(Chassis_MSG.L_Rod.Y)*multi   ); //右下方连杆 BC
//			UI_Draw_Arc( &UI_Graph7.Graphic[5], "201", UI_Graph_Add, 0, UI_Color_Green, (60  + (uint16_t)angle_error)%360, (120  + (uint16_t)angle_error)%360, 3, xChassis,   yChassis,   30, 30);        //底盘半圆
//			UI_Draw_Arc( &UI_Graph7.Graphic[6], "202", UI_Graph_Add, 0, UI_Color_Green, (240 + (uint16_t)angle_error)%360, (300 + (uint16_t)angle_error)%360, 3, xChassis,   yChassis,   30, 30);        //底盘半圆
//			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
//			continue;
//		}
		
		if(UI_PushUp_Counter % 301 == 0) //添加中央标尺1
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add,    0, UI_Color_Green, 1,  x01-50-with01,   y01,     x01-50,          y01); //第一行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add,    0, UI_Color_Green, 1,  x01-10,          y01,     x01+10,          y01); //第一行十字横
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add,    0, UI_Color_Green, 1,  x01+50,          y01,     x01+50+with01,   y01); //第一行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add,    0, UI_Color_Green, 1,  x01,             y01-10,  x01,             y01+10); //第一行十字竖
			
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add,    0, UI_Color_Green, 1,  x02-50-with02,   y02,     x02-50,          y02); //第二行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add,    0, UI_Color_Green, 2,  x02,             y02,     x02,             y02); //第二行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add,    0, UI_Color_Green, 1,  x02+50,          y02,     x02+50+with02,   y02); //第二行右横线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		else if(UI_PushUp_Counter % 301 == 10) //动态UI绘制 速度提示
		{
//			UI_Draw_Rectangle(&UI_Graph5.Graphic[0], "400", UI_Graph_Add, 0, UI_Color_Green, 1,xSpeed,ySpeed,xSpeed+LongSpeed, ySpeed+14);
//			if(Up_Data.if_DCG_Open == 1)//开弹舱盖便秘模式
//			{
//				UI_Draw_Line(&UI_Graph5.Graphic[1], "401", UI_Graph_Add, 0, UI_Color_Green, 14,  xSpeed,   ySpeed+7,  xSpeed+LongSpeed*0.2, ySpeed+7 );
//				float add = fabsf(Chassis_MSG.state_vector.fdb_v)/(Weak_Control.max_speed)*LongSpeed*0.2f;
//				UI_Draw_Line(&UI_Graph5.Graphic[2], "402", UI_Graph_Add, 0, UI_Color_Orange, 14,  xSpeed,   ySpeed+7,  xSpeed+add, ySpeed+7 );

//			}
//			else if(Chassis_MSG.state_vector.speed_gear == 2)
//			{
//				UI_Draw_Line(&UI_Graph5.Graphic[1], "401", UI_Graph_Add, 0, UI_Color_Green, 14,  xSpeed,   ySpeed+7,  xSpeed+LongSpeed*0.47f, ySpeed+7 );
//				float add = fabsf(Chassis_MSG.state_vector.fdb_v)/(Weak_Control.max_speed)*LongSpeed*0.47f;
//				UI_Draw_Line(&UI_Graph5.Graphic[2], "402", UI_Graph_Add, 0, UI_Color_Orange, 14,  xSpeed,   ySpeed+7,  xSpeed+add, ySpeed+7 );
//			}
//			else if(Chassis_MSG.state_vector.speed_gear == 1)
//			{
//				UI_Draw_Line(&UI_Graph5.Graphic[1], "401", UI_Graph_Add, 0, UI_Color_Green, 14,  xSpeed,   ySpeed+7,  xSpeed+LongSpeed*0.73f, ySpeed+7 );
//				float add = fabsf(Chassis_MSG.state_vector.fdb_v)/(Weak_Control.max_speed)*LongSpeed*0.73f;
//				UI_Draw_Line(&UI_Graph5.Graphic[2], "402", UI_Graph_Add, 0, UI_Color_Orange, 14,  xSpeed,   ySpeed+7,  xSpeed+add, ySpeed+7 );
//			}
//			else if(Chassis_MSG.state_vector.speed_gear == 0)
//			{
//				UI_Draw_Line(&UI_Graph5.Graphic[1], "401", UI_Graph_Add, 0, UI_Color_Green, 14,  xSpeed,   ySpeed+7,  xSpeed+LongSpeed, ySpeed+7 );
//				float add = fabsf(Chassis_MSG.state_vector.fdb_v)/(Weak_Control.max_speed)*LongSpeed;
//				UI_Draw_Line(&UI_Graph5.Graphic[2], "402", UI_Graph_Add, 0, UI_Color_Orange, 14,  xSpeed,   ySpeed+7,  xSpeed+add, ySpeed+7 );
//			}
//			
//			UI_Draw_Rectangle(&UI_Graph5.Graphic[3], "403", UI_Graph_Add, 0, UI_Color_Green, 1,xSpeed+LongSpeed*0.47f,ySpeed,xSpeed+LongSpeed*0.47f, ySpeed+14);
//			UI_Draw_Rectangle(&UI_Graph5.Graphic[4], "404", UI_Graph_Add, 0, UI_Color_Green, 1,xSpeed+LongSpeed*0.73f,ySpeed,xSpeed+LongSpeed*0.73f, ySpeed+14);
//			
//			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
//			continue;
		}
		else if(UI_PushUp_Counter % 301 == 15)//视觉频率
		{
//			UI_Draw_String(&UI_String.String, "500", UI_Graph_Add, 0, UI_Color_Green,  18, 7, 2,  1400, 400, "Vision "); //
//			UI_PushUp_String(&UI_String, Robot_ID_Current);
			continue;
		}
		else if(UI_PushUp_Counter % 301 == 30)//视觉频率
		{
//			UI_Draw_String(&UI_String.String, "500", UI_Graph_Change, 0, UI_Color_Green,  18, 7, 2,  1400, 400, "Vision "); //
//			UI_PushUp_String(&UI_String, Robot_ID_Current);
			continue;
		}
		else if(UI_PushUp_Counter % 301 == 40)
		{
//			UI_Draw_Int(&UI_Graph2.Graphic[0], "501", UI_Graph_Add,0,UI_Color_Green,20,1,1600,400,VisionValue.RX_MSG.RX_Frequent);
//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
      continue;
		}

		
		if(UI_PushUp_Counter % 21 == 0 && actChassis == CHASSIS_FOLLOW_GIMBAL) //动态UI绘制 字符和数字
		{
				UI_Draw_String(&UI_String.String, "300", UI_Graph_Add,    2, UI_Color_Pink,  44, 8, 8,  200, 550, "FOLLOW"); //弹舱盖是否开启
				UI_PushUp_String(&UI_String, Robot_ID_Current);
		}

		if(UI_PushUp_Counter % 21 == 2) //动态UI绘制 字符和数字
		{
			if(actChassis == CHASSIS_FOLLOW_GIMBAL)
			{
				UI_Draw_String(&UI_String.String, "300", UI_Graph_Change,    2, UI_Color_Pink,  44, 8, 8,  200, 550, "FOLLOW"); //弹舱盖是否开启
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
			else
			{
				UI_Draw_String(&UI_String.String, "300", UI_Graph_Delete,    2, UI_Color_Pink,  44, 8, 8,  200, 550, "NORMAL"); //弹舱盖是否开启
				UI_PushUp_String(&UI_String, Robot_ID_Current);
			}
		}
		
		if(UI_PushUp_Counter % 10 == 2) //动态UI绘制 连杆位置 底盘位置
		{
//			uint8_t colour;
////			if(fabsf(Chassis_MSG.state_vector.Dip_Angle) >0.3f)//超过15度认为倒下，此时连杆显示橙色
////			{
////				colour = UI_Color_Orange;
////			}
////			else
////			{
//				colour = UI_Color_Green;
////			}
//			if(DJM_MSG[0].angle <= HEAD_ENCODER)angle_error = HEAD_ENCODER - DJM_MSG[0].angle;
//			else angle_error = 8192 - (DJM_MSG[0].angle - HEAD_ENCODER);
//			angle_error = angle_error/8192.f*360.f;
//			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Change, 0, colour, 3,  xE                                  ,   yE                              ,   xE+L5*multi                         ,   yE-0*multi                       ); //最上方横线 EA
//			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Change, 0, colour, 3,  xE                                  ,   yE                              ,   xE+(-Chassis_MSG.L_Rod.Xd+L5)*multi ,   yE-(Chassis_MSG.L_Rod.Yd)*multi  ); //左上方连杆 ED
//			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Change, 0, colour, 3,  xE+(-Chassis_MSG.L_Rod.Xd+L5)*multi ,   yE-(Chassis_MSG.L_Rod.Yd)*multi ,   xE+(-Chassis_MSG.L_Rod.X+L5)*multi  ,   yE-(Chassis_MSG.L_Rod.Y)*multi   ); //左下方连杆 DC
//			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Change, 0, colour, 3,  xE+L5*multi                         ,   yE-0*multi                      ,   xE+(-Chassis_MSG.L_Rod.Xb+L5)*multi ,   yE-(Chassis_MSG.L_Rod.Yb)*multi  ); //右上方连杆 AB
//			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Change, 0, colour, 3,  xE+(-Chassis_MSG.L_Rod.Xb+L5)*multi ,   yE-(Chassis_MSG.L_Rod.Yb)*multi ,   xE+(-Chassis_MSG.L_Rod.X+L5)*multi  ,   yE-(Chassis_MSG.L_Rod.Y)*multi   ); //右下方连杆 BC
//			UI_Draw_Arc( &UI_Graph7.Graphic[5], "201", UI_Graph_Change, 0, UI_Color_Green, (60  + (uint16_t)angle_error)%360, (120 + (uint16_t)angle_error)%360, 3, xChassis,   yChassis,   30, 30);        //底盘半圆
//			UI_Draw_Arc( &UI_Graph7.Graphic[6], "202", UI_Graph_Change, 0, UI_Color_Green, (240 + (uint16_t)angle_error)%360, (300 + (uint16_t)angle_error)%360, 3, xChassis,   yChassis,   30, 30);        //底盘半圆
//			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
//			continue;
		}
		else if(UI_PushUp_Counter % 10 == 4) //动态UI绘制 速度提示
		{
//			if(Up_Data.if_DCG_Open == 1)//开弹舱盖便秘模式
//			{
//				UI_Draw_Line(&UI_Graph2.Graphic[0], "401", UI_Graph_Change, 0, UI_Color_Green, 14,  xSpeed,   ySpeed+7,  xSpeed+LongSpeed*0.2, ySpeed+7 );
//				float add = fabsf(Chassis_MSG.state_vector.fdb_v)/(Weak_Control.max_speed)*LongSpeed*0.2f;
//				UI_Draw_Line(&UI_Graph2.Graphic[1], "402", UI_Graph_Change, 0, UI_Color_Orange, 14,  xSpeed,   ySpeed+7,  xSpeed+add, ySpeed+7 );

//			}
//			else if(Chassis_MSG.state_vector.speed_gear == 2)
//			{
//				UI_Draw_Line(&UI_Graph2.Graphic[0], "401", UI_Graph_Change, 0, UI_Color_Green, 14,  xSpeed,   ySpeed+7,  xSpeed+LongSpeed*0.47f, ySpeed+7 );
//				float add = fabsf(Chassis_MSG.state_vector.fdb_v)/(Weak_Control.max_speed)*LongSpeed*0.47f;
//				UI_Draw_Line(&UI_Graph2.Graphic[1], "402", UI_Graph_Change, 0, UI_Color_Orange, 14,  xSpeed,   ySpeed+7,  xSpeed+add, ySpeed+7 );
//			}
//			else if(Chassis_MSG.state_vector.speed_gear == 1)
//			{
//				UI_Draw_Line(&UI_Graph2.Graphic[0], "401", UI_Graph_Change, 0, UI_Color_Green, 14,  xSpeed,   ySpeed+7,  xSpeed+LongSpeed*0.73f, ySpeed+7 );
//				float add = fabsf(Chassis_MSG.state_vector.fdb_v)/(Weak_Control.max_speed)*LongSpeed*0.73f;
//				UI_Draw_Line(&UI_Graph2.Graphic[1], "402", UI_Graph_Change, 0, UI_Color_Orange, 14,  xSpeed,   ySpeed+7,  xSpeed+add, ySpeed+7 );
//			}
//			else if(Chassis_MSG.state_vector.speed_gear == 0)
//			{
//				UI_Draw_Line(&UI_Graph2.Graphic[0], "401", UI_Graph_Change, 0, UI_Color_Green, 14,  xSpeed,   ySpeed+7,  xSpeed+LongSpeed, ySpeed+7 );
//				float add = fabsf(Chassis_MSG.state_vector.fdb_v)/(Weak_Control.max_speed)*LongSpeed;
//				UI_Draw_Line(&UI_Graph2.Graphic[1], "402", UI_Graph_Change, 0, UI_Color_Orange, 14,  xSpeed,   ySpeed+7,  xSpeed+add, ySpeed+7 );
//			}
//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
//			continue;
		}
		else if(UI_PushUp_Counter % 10 == 6) //刷新中央标尺1
		{
			uint8_t colour = UI_Color_Green;
//			if(Up_Data.if_aim_sometihing == 1)
//			{
				colour = UI_Color_Pink;
//			}
			
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Change, 0, colour, 1,  x01-50-with01,   y01,     x01-50,          y01); //第一行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Change, 0, colour, 1,  x01-10,          y01,     x01+10,          y01); //第一行十字横
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Change, 0, colour, 1,  x01+50,          y01,     x01+50+with01,   y01); //第一行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Change, 0, colour, 1,  x01,             y01-30,  x01,             y01+10); //第一行十字竖
			
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Change, 0, colour, 1,  x02-50-with02,   y02,     x02-50,          y02); //第二行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Change, 0, colour, 2,  x02,             y02-5,   x02,             y02+5); //第二行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Change, 0, colour, 1,  x02+50,          y02,     x02+50+with02,   y02); //第二行右横线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		else if(UI_PushUp_Counter % 10 == 8)
		{
//			UI_Draw_Int(&UI_Graph2.Graphic[0], "501", UI_Graph_Change,0,UI_Color_Green,20,1,1600,400,VisionValue.RX_MSG.RX_Frequent);
//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
//			continue;
		}


	}

}







