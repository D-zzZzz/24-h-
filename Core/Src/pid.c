#include "pid.h"
#include "camera_uart.h"
#include "encoder.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "motor.h"

int Encoder_L,Encoder_R;//编码器 
//float Vertical_Kp=300,Vertical_Kd=2.2;//直立环 数量级（Kp:0`1000  Kd: 0~10）
//float Velocity_Kp=-0.44,Velocity_Ki=-0.0022;//速度环 数量级 (Kp:0~1)

//volatile float Vertical_Kp=230,Vertical_Kd=1.1;//直立环 数量级（Kp:0`1000  Kd: 0~10）
//volatile float Velocity_Kp=0.446,Velocity_Ki=0.0025;//速度环 数量级 (Kp:0~1)

volatile float  Vertical_Kp=670*0.6;//670*0.6;
volatile float	Vertical_Kd=1.72;//1.72;//直立环 数量级（Kp:0`1000  Kd: 0~10）
volatile float  Velocity_Kp=0.3;//0.3;
volatile float	Velocity_Ki=0.3/200;//0.3/200;//速度环 数量级 (Kp:0~1)
volatile float Turn_Kp  = 0.34f;
volatile float Turn_Kp2 = 0.015f;  // 新增：二次项系数
volatile float Turn_Kd  = 0.0f;   // 保留原来的陀螺仪反馈系数
volatile int    Target_Turn = 0;

volatile int Target_Speed = 50; /* Base forward speed; Bluetooth commands may override it. */

volatile float Med_Angle=21.25;//平衡时角度值的偏移量

/* Real-time contribution of each PID term, for OLED diagnostics. */
volatile float Vertical_Kp_Component;
volatile float Vertical_Kd_Component;
volatile float Velocity_Kp_Component;
volatile float Velocity_Ki_Component;

/* Keep gyro feedback disabled until the physical feedback sign is verified. */


volatile uint8_t stop;
volatile uint8_t tilt_stop_latched;

static float velocity_error_last;
static float velocity_error_sum;

volatile uint8_t dmp_read_error;

//陀螺仪
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short aacx,aacy,aacz;

//闭环控制中间变量
int Vertical_out,Turn_out;
float Velocity_out;
int MOTOR1,MOTOR2;


extern TIM_HandleTypeDef htim2,htim4;

//直立环PD控制器
//输入：期望角度、真实角度、角速度
int Vertical(float Med,float Angle,float gyro_Y)
{
    int temp;
    Vertical_Kp_Component = Vertical_Kp * (Angle - Med);//真实角度-期望角度
    Vertical_Kd_Component = Vertical_Kd * gyro_Y;
    temp = Vertical_Kp_Component + Vertical_Kd_Component;
    return temp;
}

//速度环PI控制器
//输入：期望速度、左编码器、右编码器
float Velocity(int Target,int encoder_L,int encoder_R)
{
    int Err;
    float Err_LowOut,temp;
    static float a=0.7;
    //1.计算偏差值
    Err=(-encoder_L+encoder_R)-Target;
    //2.低通滤波
    Err_LowOut=(1-a)*Err+a*velocity_error_last;
    velocity_error_last=Err_LowOut;
    //3.积分
    velocity_error_sum+=Err_LowOut;
    //4.积分限幅
    velocity_error_sum=velocity_error_sum>20000.0f?20000.0f:(velocity_error_sum<(-20000.0f)?(-20000.0f):velocity_error_sum);

    if(stop==1)velocity_error_sum=0,stop=0;
    //5.速度环计算
    Velocity_Kp_Component = Velocity_Kp * Err_LowOut;
    Velocity_Ki_Component = Velocity_Ki/10.0 * velocity_error_sum;
    temp = Velocity_Kp_Component + Velocity_Ki_Component;
    return temp;

}

/* Steering command feed-forward plus raw Z-gyro damping, not a yaw-angle PID.
 * gyro_Z is raw sensor counts (16.4 counts/(deg/s) at +/-2000 deg/s).
 * Feedback sign depends on motor wiring and sensor mounting orientation. */
int Turn(int gyro_Z, int target_turn)
{
    float error = (float)target_turn;
    float abs_error = error >= 0.0f ? error : -error;
    float output;

    output = Turn_Kp * error
           + Turn_Kp2 * error * abs_error
           + Turn_Kd * gyro_Z;

    if (output > 2000.0f)
        output = 2000.0f;
    else if (output < -2000.0f)
        output = -2000.0f;

    return (int)output;
}
static void StopForExcessiveTilt(void)
{
    /* Keep stale commands and accumulated speed error from surviving a fall. */
    Target_Speed = 0;
    Target_Turn = 0;
    velocity_error_last = 0.0f;
    velocity_error_sum = 0.0f;
    stop = 0U;
    Vertical_out = 0;
    Velocity_out = 0;
    Turn_out = 0;
    Vertical_Kp_Component = 0.0f;
    Vertical_Kd_Component = 0.0f;
    Velocity_Kp_Component = 0.0f;
    Velocity_Ki_Component = 0.0f;
    MOTOR1 = 0;
    MOTOR2 = 0;
    Load(0, 0);
}

void control(void) //每隔10ms调用一次
{
    int PWM_out;
    int steering_margin;
    //1.读取传感器的值，包括编码器和陀螺仪
    Encoder_L=Read_Speed(&htim2);
    Encoder_R=-Read_Speed(&htim4);
    dmp_read_error = mpu_dmp_get_data(&pitch,&roll,&yaw);

    /* Compare against the mechanical upright angle, never the speed-loop target. */
    if (pitch > Med_Angle + TILT_STOP_ANGLE_DEG ||
        pitch < Med_Angle - TILT_STOP_ANGLE_DEG)
        tilt_stop_latched = 1U;
    if (tilt_stop_latched != 0U)
    {
        StopForExcessiveTilt();
        return;
    }

    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
    MPU_Get_Accelerometer(&aacx,&aacy,&aacz);

    //2.将数据传入PID控制器,计算出输出结果，左右两个电机的转速值
    Velocity_out=Velocity(Camera_ControlTargets(),Encoder_L,Encoder_R);
    Vertical_out=Vertical(Velocity_out+Med_Angle,pitch,gyroy);
    Turn_out=Turn(gyroz,Target_Turn);
    PWM_out=Vertical_out;
    /* Preserve the balance command when a steering request reaches saturation. */
    if (PWM_out > MOTOR_PWM_LIMIT)
        PWM_out = MOTOR_PWM_LIMIT;
    else if (PWM_out < -MOTOR_PWM_LIMIT)
        PWM_out = -MOTOR_PWM_LIMIT;
    steering_margin = MOTOR_PWM_LIMIT - (PWM_out < 0 ? -PWM_out : PWM_out);
    if (Turn_out > steering_margin)
        Turn_out = steering_margin;
    else if (Turn_out < -steering_margin)
        Turn_out = -steering_margin;
    MOTOR1=PWM_out-Turn_out;
    MOTOR2=PWM_out+Turn_out;
    Limit(&MOTOR1,&MOTOR2);
    Load(MOTOR1,MOTOR2);
//	Load(7200,7200);

}
