#include "Control.h"
#include "Kalman.h"
#include "bsp_encoder.h"
#include "bsp_imu.h"
#include "param_config.h"
#include "bsp_motor.h"

// PWM限幅，比17000小一些
#define Act_Motor_PWM_Freq_Max 15000
#define Act_Motor_PWM_Freq_Min -15000

//三环比例系数
float Vertical_Kp,Vertical_Kd;
float Velocity_Kp,Velocity_Ki;
float Turn_Kp,Turn_Kd;
uint8_t stop;   //用于蓝牙遥控直接停止


//Control()中需要读取的传感器数据
int Encoder_Left,Encoder_Right;      // 左右编码器
float pitch,roll,yaw;                // 欧拉角
float gyrox,gyroy,gyroz;             // 三轴角速度

//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_Turn;
float mech_med;  //机械中值（平衡时角度的偏移量。实际没有是最好的情况）
//Target_Turn在不遥控，只直立/惯性导航时，默认是0


//直立环PD控制器
//输入：期望角度、真实角度、角速度
int Vertical(float anticipation,float Angle,float gyro_Y)
{
	int16 temp;
	temp=Vertical_Kp*(Angle-anticipation)+Vertical_Kd*gyro_Y;
	return temp;
}

//速度环PI控制器
//输入：期望速度、真实速度（左右编码器）
int Velocity(int Target,int encoder_L,int encoder_R)
{
	static int Err_LowOut_last,Encoder_S; 
	static float a=0.7;
	int Err,Err_LowOut,temp;
	// 1.计算偏差值
	Err=(encoder_L+encoder_R)-Target;
	// 2.低通滤波
	Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
	Err_LowOut_last=Err_LowOut;
	// 3.积分
	Encoder_S+=Err_LowOut;
	// 4.积分限幅(-20000~20000,可更改)
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	if(stop==1)Encoder_S=0,stop=0;   // 用于实现蓝牙遥控时直接停止的功能
	// 5.速度环计算
	temp = Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
	return temp;
}

//转向环PD控制器
//输入：角速度、遥控时期望转过的角度值
int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
	return temp;
}

void Control(void)
{
	int16 PWM_L,PWM_R;   //保存直立环的输出值,直接给电机使其旋转
	
	// 1.读取并处理编码器的数据
	static int last_left=0,last_right=0;
	// a.读取编码器原始值(位置）
	int encoder_left_raw=encoder_get_left();
	int encoder_right_raw=-encoder_get_right();
	
	// b.计算速度
	Encoder_Left=encoder_left_raw-last_left;
	Encoder_Right=encoder_right_raw-last_right;
	
	// c.保存供下次使用
	last_left=encoder_left_raw;
	last_right=encoder_right_raw;
	
	// 2.读取陀螺仪的数据
	pitch = State.pitch;    // 俯仰角（度）
  roll = State.roll;      // 横滚角（度）
  yaw = State.yaw;        // 偏航角（度）
  gyrox = State.gyro_x;   // X轴角速度（°/s）
  gyroy = State.gyro_y;   // Y轴角速度（°/s）
  gyroz = State.gyro_z; // Z轴角速度（°/s）
	
	// 记得把Attitude_Update(dt)塞到定时器里
	
	// 3.数据传入PID控制器。结果->左右电机转速值
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
		#define MAX_ANGLE_OFFSET 3.0f  // 角度偏移限幅±3°
    if (Velocity_out > MAX_ANGLE_OFFSET) Velocity_out = MAX_ANGLE_OFFSET;
    if (Velocity_out < -MAX_ANGLE_OFFSET) Velocity_out = -MAX_ANGLE_OFFSET;
	
	Vertical_out=Vertical(Velocity_out+mech_med,pitch,gyroy);
	Turn_out=Turn(gyroz,Target_Turn);
	
	// 4.三环合成输出(直立±转向)
	PWM_L=Vertical_out+Turn_out;
	PWM_R=Vertical_out-Turn_out;
	
	// 5.输出限幅
	if(PWM_L>Act_Motor_PWM_Freq_Max){PWM_L=Act_Motor_PWM_Freq_Max;}
	if(PWM_R>Act_Motor_PWM_Freq_Max){PWM_R=Act_Motor_PWM_Freq_Max;}
	if(PWM_L<Act_Motor_PWM_Freq_Min){PWM_L=Act_Motor_PWM_Freq_Min;}
	if(PWM_R<Act_Motor_PWM_Freq_Min){PWM_R=Act_Motor_PWM_Freq_Min;}
	
	// 6.电机运行
	motor_set_left_speed(PWM_L);
	motor_set_right_speed(PWM_R);
	motor_update();
}









