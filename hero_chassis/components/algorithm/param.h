//#ifndef myparam_h
//#define myparam_h

///***********************底盘信息****************************************/
//#define CHASSIS_DECELE_RATIO  (3591.0f/187.0f)		//减速比  670*715*450
//#define LENGTH_A 218         //mm
//#define LENGTH_B 232         //mm
//#define WHEEL_PERIMETER 152  //mm 直径
///***********************YAW轴云台编码器的特定值******************/
//#define GIMBAL_YAW_ENCODER_MIDDLE1 1374		//底盘和云台朝向相同1，指向＋y
//#define GIMBAL_YAW_ENCODER_MIDDLE2 5458		//底盘和云台朝向相同2，指向-y
//#define GIMBAL_YAW_ENCODER_NINETY1 7607		//底盘和云台朝向90°，指向+90°
//#define GIMBAL_YAW_ENCODER_NINETY2 3382		//底盘和云台朝向90°，指向-90°
//#define GIMBAL_YAW_ENCODER_FORTYFIVE1 249	//底盘和云台朝向45°1，指向45°
//#define GIMBAL_YAW_ENCODER_FORTYFIVE2 6490	//底盘和云台朝向45°2，指向135°
//#define GIMBAL_YAW_ENCODER_FORTYFIVE3 4208	//底盘和云台朝向45°3，指向-135°
//#define GIMBAL_YAW_ENCODER_FORTYFIVE4 2231	//底盘和云台朝向45°4，指向-45°

////#define GIMBAL_YAW_ENCODER_MIDDLE2 (GIMBAL_YAW_ENCODER_MIDDLE1 + 4*1024)		//底盘和云台朝向相同2，指向-y
////#define GIMBAL_YAW_ENCODER_NINETY1 (GIMBAL_YAW_ENCODER_MIDDLE1 + 2*1024)		//底盘和云台朝向90°，指向+90°
////#define GIMBAL_YAW_ENCODER_NINETY2 (GIMBAL_YAW_ENCODER_MIDDLE1 + 6*1024)		//底盘和云台朝向90°，指向-90°
////#define GIMBAL_YAW_ENCODER_FORTYFIVE1 (GIMBAL_YAW_ENCODER_MIDDLE1 + 1*1024)	//底盘和云台朝向45°1，指向45°
////#define GIMBAL_YAW_ENCODER_FORTYFIVE2 (GIMBAL_YAW_ENCODER_MIDDLE1 + 3*1024)	//底盘和云台朝向45°2，指向135°
////#define GIMBAL_YAW_ENCODER_FORTYFIVE3 (GIMBAL_YAW_ENCODER_MIDDLE1 + 5*1024)	//底盘和云台朝向45°3，指向-135°
////#define GIMBAL_YAW_ENCODER_FORTYFIVE4 (GIMBAL_YAW_ENCODER_MIDDLE1 + 7*1024)	//底盘和云台朝向45°4，指向-45°
///**********各个模式下各个电机的限制电流大小***************/
////普通非跟随云台底盘限流
//#define NOMOAL_CHASSIS_MAX1 20000
//#define NOMOAL_CHASSIS_MAX2 20000
//#define NOMOAL_CHASSIS_MAX3 20000
//#define NOMOAL_CHASSIS_MAX4 20000
////爬坡非跟随云台底盘限流
//#define CLIMBING_CHASSIS_MAX1 30000
//#define CLIMBING_CHASSIS_MAX2 30000
//#define CLIMBING_CHASSIS_MAX3 30000
//#define CLIMBING_CHASSIS_MAX4 30000
////普通跟随云台底盘限流
//#define NOMAL_FOLLOW_CHASSIS_MAX1 20000
//#define NOMAL_FOLLOW_CHASSIS_MAX2 20000
//#define NOMAL_FOLLOW_CHASSIS_MAX3 20000
//#define NOMAL_FOLLOW_CHASSIS_MAX4 20000
////爬坡跟随云台底盘限流
//#define CLIMBING_FOLLOW_CHASSIS_MAX1 30000
//#define CLIMBING_FOLLOW_CHASSIS_MAX2 30000
//#define CLIMBING_FOLLOW_CHASSIS_MAX3 30000
//#define CLIMBING_FOLLOW_CHASSIS_MAX4 30000
////普通小陀螺/扭屁股限流
//#define NOMAL_GYRO_CHASSIS_MAX1 30000
//#define NOMAL_GYRO_CHASSIS_MAX2 30000
//#define NOMAL_GYRO_CHASSIS_MAX3 30000
//#define NOMAL_GYRO_CHASSIS_MAX4 30000
////爬坡小陀螺/扭屁股限流
//#define CLIMBING_GYRO_CHASSIS_MAX1 30000
//#define CLIMBING_GYRO_CHASSIS_MAX2 30000
//#define CLIMBING_GYRO_CHASSIS_MAX3 30000
//#define CLIMBING_GYRO_CHASSIS_MAX4 30000


///**********************************云台信息****************************************/
///***********************Pitch轴、YAW轴云台编码器限位****************************/
//#define GIMBAL_PITCH_ENCODER_MAX 4050    //up
//#define GIMBAL_PITCH_ENCODER_MIDDLE 3400
//#define GIMBAL_PITCH_ENCODER_MIN 3080    //down
//#define GIMBAL_YAW_ENCODER_MAX 6170       //right
//#define GIMBAL_YAW_ENCODER_MIDDLE 1374
//#define GIMBAL_YAW_ENCODER_MIN 2020        //left

///********************遥控器/键盘参数****************************/
//#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VX 300.0f	//底盘跟随云台模式灵敏度vx  越大灵敏度越小
//#define SENSITIVITY_REMOTE_CHASSIS_FOLLOW_VY 300.0f	//底盘跟随云台模式灵敏度vy  越大灵敏度越小


//#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VX 300.0f	//底盘不跟随云台模式灵敏度vx  越大灵敏度越小
//#define SENSITIVITY_REMOTE_CHASSIS_NOFOLLOW_VY 300.0f	//底盘不跟随云台模式灵敏度vy  越大灵敏度越小


//#define SENSITIVITY_REMOTE_GIMBAL_YAW 200.0f		//云台灵敏度yaw轴，越大灵敏度越小
//#define SENSITIVITY_REMOTE_GIMBAL_PITCH 200.0f		//云台灵敏度pitch轴，越大灵敏度越小

//#define SENSITIVITY_REMOTE_GIMBAL_YAW_IMU 1140.0f		//云台灵敏度yaw轴，越大灵敏度越小
//#define SENSITIVITY_REMOTE_GIMBAL_PITCH_IMU 1140.0f		//云台灵敏度pitch轴，越大灵敏度越小

///***********************视觉灵敏度*************************************/

//#define SENSITIVITY_VISION_GIMBAL_YAW_ENCODER 700.0f		//云台灵敏度yaw轴，越大灵敏度越小
//#define SENSITIVITY_VISION_GIMBAL_PITCH_ENCODER 700.0f		//云台灵敏度pitch轴，越大灵敏度越小

///***************************拨盘信息****************************************/
///******************拨盘电机限流****************/
//#define   REVOLVER_PID_POSITION_OUTMAX1       5000
//#define   REVOLVER_PID_POSITION_IMAX1         2000
//#define   REVOLVER_PID_SPEED_OUTMAX2    31000
//#define   REVOLVER_PID_SPEED_IMAX2      15000
///******************拨盘硬件尺寸******************/
////#define   REVOL_SPEED_RATIO   2160       //电机轴一秒转一圈,2160转子转速,60*36,乘射频再除以拨盘格数就可得相应射频下的转速
////#define 	REVOL_SPEED_GRID      4			//拨盘格数
////#define  	AN_BULLET         (24576.0f)		//单个子弹电机位置增加值(这个值得测鸭)


//#endif


