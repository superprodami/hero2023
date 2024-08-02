///**
//  ******************************************************************************
//  * @file    Info_Transmit_task.c
//  * @brief   客户端自定义UI任务。要格外注意串口占用，任务堆栈大小的问题，二者都
//  * 很有可能导致死机。
//  ******************************************************************************
//  * @attention
//  *
//  * 2021.4.13 基本实现了画固定线条，形状按实时状态改变的功能。因为未知原因暂时还是
//  * 显示不出字符，所以用不同颜色、形状的图形表示车的不同状态。
//  *
//  *
//  *
//  ******************************************************************************
//  */
//#include "Info_Transmit_task.h"
////#include "main.h"
//#include "tuxin.h"
////#include "param.h"
//#include "arm_math.h"
////#include "motor.h"

//u8 FLAG = 0;
////u8 FLAGGG = 0;
//bool S_FLAG = 0;
//bool B_FLAG = 0;
//void Info_Transmit_task(void const *argument)
//{
////    for(int i=0; i<20; i++)
////    {
////      draw_seven_line(Tx_buff_seven,6,huart1,Data_P_CE);
////      draw_seven_line(Tx_buff_seven,7,huart1,Data_S_CE);
//      Line_of_sight(huart1);
//			Change_Number[0][0]=modify;
//      Change_Number[1][0]=modify;
//			Change_arrow1[0][0]=modify;

////    }
////    Change_Number[0][0]=modify;
//	
//    while(1)
//    {
//        if(abs(mocalun_l.motor_value->speed_rpm) > 10)//摩擦轮
//        {
//					if(S_FLAG==0)
//					{
//						S_FLAG=1;
//						Delete_All(Tx_buff,0,huart1);
//						Change_Number[0][0]=add;
//						Change_arrow1[0][0]=add;
//						Line_of_sight(huart1);
//						Change_Number[0][0]=modify;
//						Change_arrow1[0][0]=modify;
//					}
//					draw_seven_line(Tx_buff_seven,7,huart1,Data_S_OP);
//        }
//        else
//        {
//					if(S_FLAG==1)
//					{
//						S_FLAG=0;
//						Delete_All(Tx_buff,0,huart1);
//						Change_Number[0][0]=add;
//						Change_arrow1[0][0]=add;
//						Line_of_sight(huart1);
//						Change_Number[0][0]=modify;
//						Change_arrow1[0][0]=modify;
//					}
//					draw_seven_line(Tx_buff_seven,7,huart1,Data_S_CE);
//        }
//        /********************************************/
//        if(abs(Ammunition_Motor.motor_value->speed_rpm) > 100)//拨弹盘
//        {
//					if(B_FLAG==0)
//					{
//						B_FLAG=1;
//						Delete_All(Tx_buff,0,huart1);
//						Change_Number[0][0]=add;
//						Change_Number[1][0]=add;
//						Change_arrow1[0][0]=add;
//						Line_of_sight(huart1);
//						Change_Number[0][0]=modify;
//						Change_Number[1][0]=modify;
//						Change_arrow1[0][0]=modify;
//					}
//					draw_seven_line(Tx_buff_seven,6,huart1,Data_P_OP);
//        }
//        else
//        {
//          if(B_FLAG==1)
//					{
//						B_FLAG=0;
//						Delete_All(Tx_buff,0,huart1);
//						Change_Number[0][0]=add;
//						Change_Number[1][0]=add;
//						Change_arrow1[0][0]=add;
//						Line_of_sight(huart1);
//						Change_Number[0][0]=modify;
//						Change_Number[1][0]=modify;
//						Change_arrow1[0][0]=modify;
//					}
//					draw_seven_line(Tx_buff_seven,6,huart1,Data_P_CE);
//        }
//        /********************************************/
////        if(actChassis == CHASSIS_GYROSCOPE)//模式
////        {
////            if(Change_Data[2][3]!=yellow)
////            {
////                Delete_All(Tx_buff,0,huart1);
////                FLAG=0;
////            }
////            Change_Data[2][3]=yellow;//黄色
////        }
////        else if(actChassis == CHASSIS_NORMAL)
////        {
////            if(Change_Data[2][3]!=white)
////            {
////                Delete_All(Tx_buff,0,huart1);
////                FLAG=0;
////            }
////            Change_Data[2][3]=white;//白色
////        }
////        else
////        {
////            if(Change_Data[2][3]!=black)
////            {
////                Delete_All(Tx_buff,0,huart1);
////                FLAG=0;
////            }
////            Change_Data[2][3]=black;//黑色
////        }
//				
///**************************************************************************************************************************/
//			
//				if(Power.power_cap_percentage <= 10)//格子型电容显示
//				{
//					
//            if(FLAG!=1)
//            {
//						Delete_All(Tx_buff,0,huart1);
//						Change_Number[0][0]=add;
//						Change_Number[1][0]=add;
//						Change_arrow1[0][0]=add;
//						Line_of_sight(huart1);
//						Change_Number[0][0]=modify;
//						Change_Number[1][0]=modify;
//						Change_arrow1[0][0]=modify;
//             FLAG=1;
//            }
//					for(int ff=0;ff<5;ff++)
//					{
//						Change_capacitance[ff][6]=0;
//					}
//				}
//				else if(Power.power_cap_percentage > 10 && Power.power_cap_percentage <= 20)
//				{
//						if(FLAG!=2)
//            {
//						Delete_All(Tx_buff,0,huart1);
//						Change_Number[0][0]=add;
//						Change_Number[1][0]=add;
//						Change_arrow1[0][0]=add;
//						Line_of_sight(huart1);
//						Change_Number[0][0]=modify;
//						Change_Number[1][0]=modify;
//						Change_arrow1[0][0]=modify;
//             FLAG=2;
//            }
//					for(int ff=1;ff<5;ff++)
//					{
//						Change_capacitance[ff][6]=0;
//					}
//					for(int ff=0;ff<1;ff++)
//					{
//						Change_capacitance[ff][6]=40;
//					}
//				}
//				else if(Power.power_cap_percentage > 20 && Power.power_cap_percentage <= 40)
//				{
//						if(FLAG!=3)
//            {
//						Delete_All(Tx_buff,0,huart1);
//						Change_Number[0][0]=add;
//						Change_Number[1][0]=add;
//						Change_arrow1[0][0]=add;
//						Line_of_sight(huart1);
//						Change_Number[0][0]=modify;
//						Change_Number[1][0]=modify;
//						Change_arrow1[0][0]=modify;
//             FLAG=3;
//            }
//					for(int ff=2;ff<5;ff++)
//					{
//						Change_capacitance[ff][6]=0;
//					}
//					for(int ff=0;ff<2;ff++)
//					{
//						Change_capacitance[ff][6]=40;
//					}
//				}
//				else if(Power.power_cap_percentage > 40 && Power.power_cap_percentage <= 60)
//				{
//						if(FLAG!=4)
//            {
//						Delete_All(Tx_buff,0,huart1);
//						Change_Number[0][0]=add;
//						Change_Number[1][0]=add;
//						Change_arrow1[0][0]=add;
//						Line_of_sight(huart1);
//						Change_Number[0][0]=modify;
//						Change_Number[1][0]=modify;
//						Change_arrow1[0][0]=modify;
//             FLAG=4;
//            }
//					for(int ff=3;ff<5;ff++)
//					{
//						Change_capacitance[ff][6]=0;
//					}
//					for(int ff=0;ff<3;ff++)
//					{
//						Change_capacitance[ff][6]=40;
//					}
//				}
//				else if(Power.power_cap_percentage > 60 && Power.power_cap_percentage <= 80)
//				{
//						if(FLAG!=5)
//            {
//              Delete_All(Tx_buff,0,huart1);
//              Change_Number[0][0]=add;
//              Change_Number[1][0]=add;
//              Change_arrow1[0][0]=add;
//              Line_of_sight(huart1);
//              Change_Number[0][0]=modify;
//              Change_Number[1][0]=modify;
//              Change_arrow1[0][0]=modify;
//              FLAG=5;
//            }
//					for(int ff=4;ff<5;ff++)
//					{
//						Change_capacitance[ff][6]=0;
//					}
//					for(int ff=0;ff<4;ff++)
//					{
//						Change_capacitance[ff][6]=40;
//					}
//				}
//				else if(Power.power_cap_percentage > 80 && Power.power_cap_percentage <= 100)
//				{
//	
//						if(FLAG!=6)
//            {
//              Delete_All(Tx_buff,0,huart1);
//              Change_Number[0][0]=add;
//              Change_Number[1][0]=add;
//              Change_arrow1[0][0]=add;
//              Line_of_sight(huart1);
//              Change_Number[0][0]=modify;
//              Change_Number[1][0]=modify;
//              Change_arrow1[0][0]=modify;
//             FLAG=6;
//            }
//					for(int ff=5;ff<5;ff++)
//					{
//						Change_capacitance[ff][6]=0;
//					}
//					for(int ff=0;ff<5;ff++)
//					{
//						Change_capacitance[ff][6]=40;
//					}
//				}
//				draw_seven_line(Tx_buff_seven,3,huart1,Change_capacitance);

//				
//				
///**************************************************************************************************************************/
////				if(Gimbal_MotorYaw.motor_value->main_angle > 7680 || Gimbal_MotorYaw.motor_value->main_angle < 512)//上 MIDDLE1
////				{
////					if(FLAG!=0)
////           {
////            	Delete_All(Tx_buff,0,huart1);
////             FLAG=0;
////           }
////					draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow1);
////				}
////				else if(Gimbal_MotorYaw.motor_value->main_angle > 512 && Gimbal_MotorYaw.motor_value->main_angle < 1536)//左上 FORTYFIVE1
////				{
////					if(FLAG!=1)
////           {
////            	Delete_All(Tx_buff,0,huart1);
////             FLAG=1;
////           }
////           	draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow8);
////				}
////				else if(Gimbal_MotorYaw.motor_value->main_angle > 1536 && Gimbal_MotorYaw.motor_value->main_angle < 2560)//左 NINETY1
////				{
////					if(FLAG!=2)
////           {
////            	Delete_All(Tx_buff,0,huart1);
////             FLAG=2;
////           }
////					draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow7);
////				}
////				else if(Gimbal_MotorYaw.motor_value->main_angle > 2560 && Gimbal_MotorYaw.motor_value->main_angle < 3584)//左下 FORTYFIVE2
////				{
////					if(FLAG!=3)
////           {
////            	Delete_All(Tx_buff,0,huart1);
////             FLAG=3;
////           }
////           					draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow6);
////				}
////				else if(Gimbal_MotorYaw.motor_value->main_angle > 3584 && Gimbal_MotorYaw.motor_value->main_angle < 4608)//下 MIDDLE2
////				{
////					if(FLAG!=4)
////           {
////            	Delete_All(Tx_buff,0,huart1);
////             FLAG=4;
////           }
////           					draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow5);
////				}
////				else if(Gimbal_MotorYaw.motor_value->main_angle > 4608 && Gimbal_MotorYaw.motor_value->main_angle < 5632)//右下
////				{
////					if(FLAG!=5)
////           {
////            	Delete_All(Tx_buff,0,huart1);
////             FLAG=5;
////           }
////           					draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow4);
////				}
////				else if(Gimbal_MotorYaw.motor_value->main_angle > 5632 && Gimbal_MotorYaw.motor_value->main_angle < 6656)//右
////				{
////					if(FLAG!=6)
////           {
////            	Delete_All(Tx_buff,0,huart1);
////             FLAG=6;
////           }
////           					draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow3);
////				}
////				else if(Gimbal_MotorYaw.motor_value->main_angle > 6656 && Gimbal_MotorYaw.motor_value->main_angle < 7680)//右上
////				{
////					if(FLAG!=7)
////           {
////            	Delete_All(Tx_buff,0,huart1);
////             FLAG=7;
////           }
////           					draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow2);
////				}
//				/********************************************/
////        if((uint32_t)Power.power_cap_percentage >= 100) Power.power_cap_percentage = 100.0f;
////        if((uint32_t)Power.power_cap_percentage <= 1) Power.power_cap_percentage = 0.0f;
//        Change_Number[0][9]=(uint32_t)Power.power_cap_percentage;
//        Change_Number[1][9]=Gimbal_MotorPitch.motor_value->angle/8;
////        if(Change_Number[0][9] > 1020) Change_Number[0][9]=1020;
////        if(FLAGGG!=Change_Number[0][9])
////        {
////          FLAGGG=Change_Number[0][9];
////          Delete_All(Tx_buff,0,huart1);
////        }
//        
//				if(actChassis == CHASSIS_FOLLOW_GIMBAL)
//        {
//            if(((Gimbal_MotorYaw.motor_value->main_angle > 0)&&(Gimbal_MotorYaw.motor_value->main_angle < GIMBAL_YAW_ENCODER_NINETY1)) ||
//                    ((Gimbal_MotorYaw.motor_value->main_angle > GIMBAL_YAW_ENCODER_NINETY2)&&(Gimbal_MotorYaw.motor_value->main_angle < 8192)))
//            {
//                Change_arrow1[0][10]=1500-sin((GIMBAL_YAW_ENCODER_MIDDLE1 - Gimbal_MotorYaw.motor_value->main_angle) * 0.043945f * PI / 180.0f)*50;
//                Change_arrow1[0][11]=800+cos((GIMBAL_YAW_ENCODER_MIDDLE1 - Gimbal_MotorYaw.motor_value->main_angle) * 0.043945f * PI / 180.0f)*50;
//            }
//            else
//            {
//                Change_arrow1[0][10]=1500-sin((GIMBAL_YAW_ENCODER_MIDDLE2 - Gimbal_MotorYaw.motor_value->main_angle) * 0.043945f * PI / 180.0f)*50;
//                Change_arrow1[0][11]=800+cos((GIMBAL_YAW_ENCODER_MIDDLE2 - Gimbal_MotorYaw.motor_value->main_angle) * 0.043945f * PI / 180.0f)*50;
//            }
//        }
//        else
//        {
//            Change_arrow1[0][10]=1500-sin((GIMBAL_YAW_ENCODER_MIDDLE1 - Gimbal_MotorYaw.motor_value->main_angle) * 0.043945f * PI / 180.0f)*50;
//            Change_arrow1[0][11]=800+cos((GIMBAL_YAW_ENCODER_MIDDLE1 - Gimbal_MotorYaw.motor_value->main_angle) * 0.043945f * PI / 180.0f)*50;
//        }
//        
//        /********************************************/
////        if(!FLAG)
////        {
////            Delete_All(Tx_buff,0,huart1);
////            draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow2);
//        
//            Line_of_sight(huart1);
////						draw_seven_line(Tx_buff_seven,2,huart1,Change_arrow0);
////            FLAG=1;
////        }
//        vTaskDelay(TIME_STAMP_200MS); //按发送协议的要求，速率为5hz
//    }
//}



