#include "kalman_filter.h"
//float matrix_value1;
//float matrix_value2;

//void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
//{
//  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
//  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
//  mat_init(&F->z,2,1,(float *)I->z_data);
//  mat_init(&F->A,2,2,(float *)I->A_data);
//  mat_init(&F->H,2,2,(float *)I->H_data);
//  mat_init(&F->Q,2,2,(float *)I->Q_data);
//  mat_init(&F->R,2,2,(float *)I->R_data);
//  mat_init(&F->P,2,2,(float *)I->P_data);
//  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
//  mat_init(&F->K,2,2,(float *)I->K_data);
//  mat_init(&F->AT,2,2,(float *)I->AT_data);
//  mat_trans(&F->A, &F->AT);
//  mat_init(&F->HT,2,2,(float *)I->HT_data);
//  mat_trans(&F->H, &F->HT);
////  matrix_value2 = F->A.pData[1];
//}
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
    mat_init(&F->xhat, 2, 1, (float *)I->xhat_data); //矩阵指针 行 列 矩阵数据指针
    mat_init(&F->xhatminus, 2, 1, (float *)I->xhatminus_data);
    mat_init(&F->z, 2, 1, (float *)I->z_data);
    mat_init(&F->A, 2, 2, (float *)I->A_data);
    mat_init(&F->B, 2, 1, (float *)I->B_data);
    mat_init(&F->H, 2, 2, (float *)I->H_data);
    mat_init(&F->Q, 2, 2, (float *)I->Q_data);
    mat_init(&F->R, 2, 2, (float *)I->R_data);
    mat_init(&F->P, 2, 2, (float *)I->P_data);
    mat_init(&F->Pminus, 2, 2, (float *)I->Pminus_data);
    mat_init(&F->K, 2, 2, (float *)I->K_data);
    mat_init(&F->KT, 2, 2, (float *)I->KT_data);

    mat_init(&F->AT, 2, 2, (float *)I->AT_data);
    mat_trans(&F->A, &F->AT);
    mat_init(&F->HT, 2, 2, (float *)I->HT_data);
    mat_trans(&F->H, &F->HT);
}


// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param 卡尔曼参数结构体
  *@param 角度
  *@param 速度
*/
//float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
//{
//  float TEMP_data[4] = {0, 0, 0, 0};
//  float TEMP_data21[2] = {0, 0};
//  mat TEMP,TEMP21;

//  mat TEMP_Input;

//  mat_init(&TEMP,2,2,(float *)TEMP_data);//
//  mat_init(&TEMP21,2,1,(float *)TEMP_data21);//
//  mat_init(&TEMP_Input,2,1,(float *)TEMP_data21);//
//  TEMP_Input.pData[0] = 0;
//  TEMP_Input.pData[1] = signal2;

//  F->z.pData[0] = signal1;//z(k)
//  F->z.pData[1] = signal2;//z(k)

//  //1. xhat'(k)= A xhat(k-1)
//  mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

//  mat_add(&F->xhatminus,&TEMP_Input,&F->xhatminus);
//
//  //2. P'(k) = A P(k-1) AT + Q
//  mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
//  mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
//  mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

//  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
//  mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
//  mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
//  mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

//  mat_inv(&F->K, &F->P);//
//  mat_mult(&F->Pminus, &F->HT, &TEMP);//
//  mat_mult(&TEMP, &F->P, &F->K);//

//  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
//  mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
//  mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
//  mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
//  mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

//  //5. P(k) = (1-K(k)H)P'(k)
//  mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
//  mat_sub(&F->Q, &F->P, &TEMP);//
//  mat_mult(&TEMP, &F->Pminus, &F->P);

//  matrix_value1 = F->xhat.pData[0];
//  matrix_value2 = F->xhat.pData[1];

//  F->filtered_value[0] = F->xhat.pData[0];
//  F->filtered_value[1] = F->xhat.pData[1];
//  return F->filtered_value;
//}
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2, float signal3)
{
    float TEMP_data[4] = {0, 0, 0, 0};
    float TEMP_data21[2] = {0, 0};
    float TEMP_data22[4] = {0, 0, 0, 0};
    float I_data[4] = {1, 0, 0, 1};

    mat TEMP, TEMP21, TEMP22, I;

    float CONTROL;

    mat_init(&I, 2, 2, (float *)I_data);
    mat_init(&TEMP, 2, 2, (float *)TEMP_data);
    mat_init(&TEMP21, 2, 1, (float *)TEMP_data21);
    mat_init(&TEMP22, 2, 2, (float *)TEMP_data22);

    CONTROL = signal3; //加速度

    F->z.pData[0] = signal1;//z(k) 测量值 位置
    F->z.pData[1] = signal2;//z(k) 速度


    /////******时间更新*******/////
    //1. xhat'(k)= A xhat(k-1) 状态预测
    mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    mat_scale(&F->B, CONTROL, &TEMP21);
    mat_add(&F->xhatminus, &TEMP21, &F->xhatminus);

    //2. P'(k) = A P(k-1) AT + Q 协方差预测
    mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q


    /////******观测更新*******/////
    //3. K(k) = P'(k) HT / (H P'(k) HT + R) 卡尔曼增益
    mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

    mat_inv(&F->K, &F->P);                 //矩阵求逆
    mat_mult(&F->Pminus, &F->HT, &TEMP);
    mat_mult(&TEMP, &F->P, &F->K);

    //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))  状态更新
    mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

    //5. P(k) = (1-K(k)H)P'(k)  协方差更新
    /*公式（1）形式简单，计算量小，但是累积误差容易使协方差矩阵失去非负正定性甚至对称性，
    所以实际中常使用公式（2）*/

    //  mat_mult(&F->K, &F->H, &F->P);//            (1)p(k|k) = (I-kg(k)*H)*P(k|k-1)
    //  //mat_sub(&F->Q, &F->P, &TEMP);
    //  mat_sub(&I, &F->P, &TEMP);
    //  mat_mult(&TEMP, &F->Pminus, &F->P);

    mat_mult(&F->K, &F->H, &F->P);               //(2)p(k|k) = (I-kg(k)*H)*P(k|k-1)*(I-kg(k)*H) + kg(k)*R*kgT(k)
    mat_sub(&I, &F->P, &TEMP);
    mat_mult(&TEMP, &F->Pminus, &F->P);
    mat_mult(&F->P, &TEMP, &F->Pminus);

    mat_mult(&F->K, &F->R, &TEMP);
    mat_trans(&F->K, &F->KT);
    mat_mult(&TEMP, &F->KT, &TEMP22);
    mat_add(&F->Pminus, &TEMP22, &F->P);

    F->filtered_value[0] = F->xhat.pData[0];
    F->filtered_value[1] = F->xhat.pData[1];
    return F->filtered_value;
}

//float dt = 0.001f;
//float Q_angle = 1.0f;             //陀螺仪噪声协方差
//float Q_gyro = 0.6f;              //漂移噪声协方差
//float R_angle =0.05f;                 //角速度测量噪声协方差
//float Angle = 0;
//float Gyro = 0;
//float Q_bias = 0;
//float K_0, K_1;
//float PP[2][2] = { { 1, 0 },{ 0, 1 } };
//float IMU_value[2];
//float *Kalman_Filter_IMU(float newAngle,float newGyro)
//{
//

//  Angle += (newGyro - Q_bias) * dt; //先验估计
//
//  PP[0][0] += Q_angle - PP[0][1] * dt - PP[1][0] * dt;
//  PP[0][1] -= PP[1][1] * dt;   //先验估计误差协方差
//  PP[1][0] -= PP[1][1] * dt;
//  PP[1][1] += Q_gyro;
//
//  K_0 = PP[0][0] / (R_angle + PP[0][0]);
//  K_1 = PP[1][0] / (R_angle + PP[0][0]);
//
//  Angle += K_0 * (newAngle - Angle);
//  Q_bias += K_1 * (newAngle - Angle); //后验估计
//  Gyro = newGyro - Q_bias;
//  //后验估计误差协方差
//  PP[0][0] -= K_0 * PP[0][0];
//  PP[0][1] -= K_0 * PP[0][1];
//  PP[1][0] -= K_1 * PP[0][0];
//  PP[1][1] -= K_1 * PP[0][1];
//
//  IMU_value[0] = Angle;
//  IMU_value[1] = Gyro;
//  return  IMU_value;
//}
#define dt              0.001f//卡尔曼滤波采样周期 s
float R_angle   =       0.5f; //测量噪声的协方差（即是测量偏差）
float Q_angle   =       0.0001f;//过程噪声的协方差
float Q_gyro    =       0.0003f; //过程噪声的协方差  过程噪声协方差为一个一行两列矩阵
static float angle = 0;            //下时刻最优估计值角度
float q_bias = 0;        //陀螺仪的偏差
float P[2][2] = {{ 1, 0 }, { 0, 1 }};//过程协方差矩阵
float Kalman_Filter_IMU(const float newGyro, const float newAngle)
{

    float K_0;//含有卡尔曼增益的另外一个函数，用于计算最优估计值
    float K_1;//含有卡尔曼增益的函数，用于计算最优估计值的偏差

    float Y_0, Y_1;

    float Rate;//去除偏差后的角速度

    float Pdot[4];//过程协方差矩阵的微分矩阵

    float angle_err;//角度偏量

    float E;//计算的过程量

    Rate = newGyro - q_bias;

    //计算过程协方差矩阵的微分矩阵
    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] = - P[1][1];
    Pdot[2] = - P[1][1];
    Pdot[3] = Q_gyro;

    angle += Rate * dt; //角速度积分得出角度

    P[0][0] += Pdot[0] * dt; //计算协方差矩阵
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    angle_err = newAngle - angle; //计算角度偏差

    E = R_angle + P[0][0];
    K_0 = P[0][0] / E; //计算卡尔曼增益
    K_1 = P[1][0] / E;

    Y_0 = P[0][0];
    Y_1 = P[0][1];

    P[0][0] -= K_0 * Y_0; //跟新协方差矩阵
    P[0][1] -= K_0 * Y_1;
    P[1][0] -= K_1 * Y_0;
    P[1][1] -= K_1 * Y_1;

    angle += K_0 * angle_err; //给出最优估计值

    q_bias += K_1 * angle_err;//跟新最优估计值偏差

    return angle;
}
/**
  * @author  Liu heng
  * 一阶卡尔曼滤波器来自RoboMaster论坛
  *   一维卡尔曼滤波器
  *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器
  *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
  *          使用示例
  *          extKalman p;                  //定义一个卡尔曼滤波器结构体
  *          float SersorData;             //需要进行滤波的数据
  *          KalmanCreate(&p,20,200);      //初始化该滤波器的Q=20 R=200参数
  *          while(1)
  *          {
  *             SersorData = sersor();                     //获取数据
  *             SersorData = KalmanFilter(&p,SersorData);  //对数据进行滤波
  *          }
  */
/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *             反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid = p->A * p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A * p->P_last + p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid / (p->P_mid + p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1 - p->kg) * p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;                //输出预测结果x(k|k)
}
