#include "myfun.h"

//

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}



/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//绝对限制
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//判断符号位
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

//

void LimitValue_f(float*VALUE, float MAX, float MIN)
{
    if(*VALUE > MAX)
        *VALUE = MAX;
    else if(*VALUE < MIN)
        *VALUE = MIN;
}

void LimitValue_u16(uint16_t *VALUE, uint16_t MAX, uint16_t MIN)
{
    if(*VALUE > MAX)
        *VALUE = MAX;
    else if(*VALUE < MIN)
        *VALUE = MIN;
}

void LimitValue_16(int16_t *VALUE, int16_t MAX, int16_t MIN)
{
    if(*VALUE > MAX)
        *VALUE = MAX;
    else if(*VALUE < MIN)
        *VALUE = MIN;
}

float constrain_float(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于指针输入值
  * @param  要在当前输出量上累加的值,目标输出量,递增快慢
  * @retval 目标输出量
  * @attention
  *
*/
float RampInc_float( float *buffer, float now, float ramp )
{

    if (*buffer > 0)
    {
        if (*buffer > ramp)
        {
            now     += ramp;
            *buffer -= ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }
    else
    {
        if (*buffer < -ramp)
        {
            now     += -ramp;
            *buffer -= -ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }

    return now;
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
  * @attention
  */
float RAMP_float( float final, float now, float ramp )
{
    float buffer = 0;


    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {
            now += ramp;
        }
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}
/**
  * @brief  角度回环 浮点
  * @param  void
  * @retval 角度值，最大角度值
  * @attention
  */
void AngleLoop_f (float *angle , float max)
{
    if(*angle < -(max / 2))
    {
        *angle += max;
    }
    else if(*angle > (max / 2))
    {
        *angle -= max;
    }
}
/**
  * @brief  角度回环
  * @param  void
  * @retval 角度值，最大角度值
  * @attention
  */
void AngleLoop (float *angle , float max)
{
    if(*angle < -(max / 2))
    {
        *angle += max;
    }
    else if(*angle > (max / 2))
    {
        *angle -= max;
    }
}


/**
  * @brief  达秒电机角度回环 浮点
  * @param  void
  * @retval 目标角度，反馈角度
  * @attention
  */
float AngleLoop_DM(float *target_angle, float *angle) 
{
	float gap;
    // 将角度限制在0到2π之间
	    while (*target_angle < -_PI) 
		{
			*target_angle += _2PI;
		}
    while (*target_angle >= _PI)
		{
			*target_angle -= _2PI;
		}
		
    while (*angle < -_PI) 
		{
			*angle += _2PI;
		}
    while (*angle >= _PI)
		{
			*angle -= _2PI;
		}
		 gap=fabs(*target_angle-*angle);
	 if(gap>_PI&&*target_angle>*angle)
			*angle+=_2PI;
	 else if(gap>_PI&&*target_angle<*angle)
			*angle-=_2PI;
}


/**
  * @brief  float类型绝对值
  * @param  void
  * @retval
  * @attention
  */
float abs_float(float a){
	if(a<0)
		return -a;
	else
		return a;
}

float queue_sum(int n,float*yaw_smooth,float*yaw_smooth1,float item1)
{
	//均值滤波
	float cs_yaw_tot=0;
//	int cs_yaw_after=0;
	static int smoothf=0;       	//均值滤波移位标志量
  /********角度回环处理*********/
  if(item1<80 && yaw_smooth1[(smoothf-3+n)%n]>280 )
  {
    yaw_smooth[smoothf]=360+item1-yaw_smooth1[(smoothf-3+n)%n];
  }
  else if(item1>280 && yaw_smooth1[(smoothf-3+n)%n]<80)
  {
    yaw_smooth[smoothf]=item1-yaw_smooth1[(smoothf-3+n)%n]-360;
  }
  else
  {
    yaw_smooth[smoothf]=item1-yaw_smooth1[(smoothf-3+n)%n];
  }
  yaw_smooth1[smoothf]=item1;
  if(smoothf==n-1)
  {
    smoothf=0;
  }
  else
  {
    smoothf++;
  }

  for(int j=0;j<n;j++)
	{
		cs_yaw_tot+=yaw_smooth[j];
	}     
	return cs_yaw_tot;
}

/**
  * @brief  系统软件复位，相当于按reset
  * @param  void
  * @retval void
  * @attention void
  */
void soft_rest(void)
{
    __set_FAULTMASK(1); //关闭所有中断
    NVIC_SystemReset(); //复位
}

/**
  * @brief  在软件复位前，为了安全先停止所有电机
  * @param  void
  * @retval void
  * @attention void
  */
void Stop_All(void)
{
    int i = 30;
    while(i--)
    {
			set_moto5678_current(&hcan1,0,0,0,0);
			set_moto1234_current(&hcan1,0,0,0,0);
			osDelay(1);
    }
}
