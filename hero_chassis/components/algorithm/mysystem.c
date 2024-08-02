//#include "mysystem.h"

//flag_t Flag_status;//各种flag 状态位

//void LimitValue_f(float*VALUE, float MAX, float MIN)
//{
//    if(*VALUE > MAX)
//        *VALUE = MAX;
//    else if(*VALUE < MIN)
//        *VALUE = MIN;
//}

//void LimitValue_u16(uint16_t *VALUE, uint16_t MAX, uint16_t MIN)
//{
//    if(*VALUE > MAX)
//        *VALUE = MAX;
//    else if(*VALUE < MIN)
//        *VALUE = MIN;
//}

//void LimitValue_16(int16_t *VALUE, int16_t MAX, int16_t MIN)
//{
//    if(*VALUE > MAX)
//        *VALUE = MAX;
//    else if(*VALUE < MIN)
//        *VALUE = MIN;
//}

//float constrain_float(float amt, float low, float high)
//{
//    if (amt < low)
//        return low;
//    else if (amt > high)
//        return high;
//    else
//        return amt;
//}

///**
//  * @brief  斜坡函数,使目标输出值缓慢等于指针输入值
//  * @param  要在当前输出量上累加的值,目标输出量,递增快慢
//  * @retval 目标输出量
//  * @attention
//  *
//*/
//float RampInc_float( float *buffer, float now, float ramp )
//{

//    if (*buffer > 0)
//    {
//        if (*buffer > ramp)
//        {
//            now     += ramp;
//            *buffer -= ramp;
//        }
//        else
//        {
//            now     += *buffer;
//            *buffer  = 0;
//        }
//    }
//    else
//    {
//        if (*buffer < -ramp)
//        {
//            now     += -ramp;
//            *buffer -= -ramp;
//        }
//        else
//        {
//            now     += *buffer;
//            *buffer  = 0;
//        }
//    }

//    return now;
//}

///**
//  * @brief  斜坡函数,使目标输出值缓慢等于期望值
//  * @param  期望最终输出,当前输出,变化速度(越大越快)
//  * @retval 当前输出
//  * @attention
//  */
//float RAMP_float( float final, float now, float ramp )
//{
//    float buffer = 0;


//    buffer = final - now;

//    if (buffer > 0)
//    {
//        if (buffer > ramp)
//        {
//            now += ramp;
//        }
//        else
//        {
//            now += buffer;
//        }
//    }
//    else
//    {
//        if (buffer < -ramp)
//        {
//            now += -ramp;
//        }
//        else
//        {
//            now += buffer;
//        }
//    }

//    return now;
//}
///**
//  * @brief  角度回环 浮点
//  * @param  void
//  * @retval 角度值，最大角度值
//  * @attention
//  */
//void AngleLoop_f (float *angle , float max)
//{
//    if(*angle < -(max / 2))
//    {
//        *angle += max;
//    }
//    else if(*angle > (max / 2))
//    {
//        *angle -= max;
//    }
//}
///**
//  * @brief  角度回环
//  * @param  void
//  * @retval 角度值，最大角度值
//  * @attention
//  */
//void AngleLoop (float *angle , float max)
//{
//    if(*angle < -(max / 2))
//    {
//        *angle += max;
//    }
//    else if(*angle > (max / 2))
//    {
//        *angle -= max;
//    }
//}

///**
//  * @brief  float类型绝对值
//  * @param  void
//  * @retval
//  * @attention
//  */
//float abs_float(float a){
//	if(a<0)
//		return -a;
//	else
//		return a;
//}

//float queue_sum(int n,float*yaw_smooth,float*yaw_smooth1,float item1)
//{
//	//均值滤波
//	float cs_yaw_tot=0;
////	int cs_yaw_after=0;
//	static int smoothf=0;       	//均值滤波移位标志量
//  /********角度回环处理*********/
//  if(item1<80 && yaw_smooth1[(smoothf-3+n)%n]>280 )
//  {
//    yaw_smooth[smoothf]=360+item1-yaw_smooth1[(smoothf-3+n)%n];
//  }
//  else if(item1>280 && yaw_smooth1[(smoothf-3+n)%n]<80)
//  {
//    yaw_smooth[smoothf]=item1-yaw_smooth1[(smoothf-3+n)%n]-360;
//  }
//  else
//  {
//    yaw_smooth[smoothf]=item1-yaw_smooth1[(smoothf-3+n)%n];
//  }
//  yaw_smooth1[smoothf]=item1;
//  if(smoothf==n-1)
//  {
//    smoothf=0;
//  }
//  else
//  {
//    smoothf++;
//  }

//  for(int j=0;j<n;j++)
//	{
//		cs_yaw_tot+=yaw_smooth[j];
//	}     
//	return cs_yaw_tot;
//}
