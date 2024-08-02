#include "myfun.h"

//

/**
 * @brief  ���ø������ݵȱ���ת��������
 * @param  x_int     	Ҫת�����޷�������
 * @param  x_min      Ŀ�긡��������Сֵ
 * @param  x_max    	Ŀ�긡���������ֵ
 * @param  bits      	�޷���������λ��
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  ��������ת��Ϊ�޷�������
 * @param  x     			Ҫת���ĸ�����
 * @param  x_min      ����������Сֵ
 * @param  x_max    	�����������ֵ
 * @param  bits      	�޷���������λ��
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
 /// Converts a float to an unsigned int, given range and number of bits///
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}



/**
  * @brief          б��������ʼ��
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      ���ֵ
  * @param[in]      ��Сֵ
  * @retval         ���ؿ�
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
  * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
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
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//��������
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

//�жϷ���λ
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

//��������
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//�޷�����
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//ѭ���޷�����
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

//���ȸ�ʽ��Ϊ-PI~PI

//�Ƕȸ�ʽ��Ϊ-180~180
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
  * @brief  б�º���,ʹĿ�����ֵ��������ָ������ֵ
  * @param  Ҫ�ڵ�ǰ��������ۼӵ�ֵ,Ŀ�������,��������
  * @retval Ŀ�������
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
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
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
  * @brief  �ǶȻػ� ����
  * @param  void
  * @retval �Ƕ�ֵ�����Ƕ�ֵ
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
  * @brief  �ǶȻػ�
  * @param  void
  * @retval �Ƕ�ֵ�����Ƕ�ֵ
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
  * @brief  �������ǶȻػ� ����
  * @param  void
  * @retval Ŀ��Ƕȣ������Ƕ�
  * @attention
  */
float AngleLoop_DM(float *target_angle, float *angle) 
{
	float gap;
    // ���Ƕ�������0��2��֮��
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
  * @brief  float���;���ֵ
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
	//��ֵ�˲�
	float cs_yaw_tot=0;
//	int cs_yaw_after=0;
	static int smoothf=0;       	//��ֵ�˲���λ��־��
  /********�ǶȻػ�����*********/
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
  * @brief  ϵͳ�����λ���൱�ڰ�reset
  * @param  void
  * @retval void
  * @attention void
  */
void soft_rest(void)
{
    __set_FAULTMASK(1); //�ر������ж�
    NVIC_SystemReset(); //��λ
}

/**
  * @brief  �������λǰ��Ϊ�˰�ȫ��ֹͣ���е��
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
