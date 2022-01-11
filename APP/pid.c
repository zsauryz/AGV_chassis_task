/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       pid.c
 * @brief     
 * @note
 * @Version    V1.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "pid.h"
#include "math.h"
#include "bsp_can.h"
float angle_spin,n;

float angle0=0,yaw0=0;
extern imu_t imu;
extern float total_C_yaw;
extern uint8_t spin_flag;
float temp_n;
uint8_t motionflag=1,PASTmotion=1;
PID_TypeDef PID_GM6020[4];	// 定义GM6020 PID结构体
PID_TypeDef PID_GM6020_speed[4];
PID_TypeDef PID_M2006[2];		// 定义M2006 PID结构体
PID_TypeDef PID_M2006_ANGLE[2];// 定义M2006 角度PID结构体
PID_TypeDef PID_M3508[7];	// 定义M3508 PID结构体 前四个为底盘电机 后两个为摩擦轮电机
PID_TypeDef PID_M3508_ANGLE[7];// 定义M2006 角度PID结构体
PID_TypeDef PID_M3508_Follow[2];// 定义M3508底盘 PID结构体
PID_TypeDef PID_HEAT_PWM;// 定义陀螺仪加热 PID结构体
void abs_limit(float *a, float ABS_MAX) {
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void PID_Reset(PID_TypeDef	*pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
/**
  * @brief  初始化PID结构体
  * @param  PID结构体指针
    @param  比例系数
		@param  积分系数
		@param  微分系数
		@param  积分最大值
		@param  总输出最大值
  * @retval None
  */
void PID_Init(
    PID_TypeDef*	pid,
    uint32_t 			mode,
    uint32_t 			maxout,
    uint32_t 			intergral_limit,
    float 				kp,
    float 				ki,
    float 				kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;

    pid->target[0]=0;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

}

float PID_Calculate(PID_TypeDef *pid, float target, float feedback)
{
    pid->feedback[NOW] = feedback;
    pid->target[NOW] = target;
    pid->err[NOW] = target - feedback;

    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    if(pid->pid_mode == POSITION_PID)					 //位置式PID
    {
        pid->pout = pid->Kp * pid->err[NOW];
        pid->iout += pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - pid->err[LAST] );
				
				
//				if(pid==&PID_GM6020[1])//
//				{
//					if(fabs(pid->err[NOW])>=0&&fabs(pid->err[NOW])<=5)
//					{
//							pid->pout = 2*pid->pout;
//							pid->iout = pid->iout;
//							pid->dout = 4*pid->dout;
//					}
//					if(fabs(pid->err[NOW])>=5&&fabs(pid->err[NOW])<=10)
//					{
//							pid->pout = 1.8*pid->pout;
//							pid->iout = pid->iout;
//							pid->dout = 3.2*pid->dout;
//					}
//					if(fabs(pid->err[NOW])>10&&fabs(pid->err[NOW])<=20)
//					{
//							pid->pout = 1.4*pid->pout;
//							pid->iout = pid->iout;
//							pid->dout = 2*pid->dout;
//					}
//					if(fabs(pid->err[NOW])>20)
//					{
//							pid->pout = pid->pout;
//							pid->iout = pid->iout;
//							pid->dout = pid->dout;
//					}
//				}
//				else if(pid==&PID_GM6020[0])//
//				{
////					if()
//					if(fabs(pid->err[NOW])>35)
//					{
//							pid->pout = 1.5f*pid->pout;
//							pid->iout = 0.2f*pid->iout;
//							pid->dout = 0.75f*pid->dout;
//					}
//					if(fabs(pid->err[NOW])>20&&fabs(pid->err[NOW])<=35)
//					{
//							pid->pout = 1.35f*pid->pout;
//							pid->iout = 0.4f*pid->iout;
//							pid->dout = 1.5f*pid->dout;
//					}
//					if(fabs(pid->err[NOW])>10&&fabs(pid->err[NOW])<=20)
//					{
//							pid->pout = 1*pid->pout;
//							pid->iout = 0.6f*pid->iout;
//							pid->dout = 2*pid->dout;
//					}
//					
//					if(fabs(pid->err[NOW])>=5&&fabs(pid->err[NOW])<=10)
//					{
//							pid->pout = 0.8f*pid->pout;//0.75
//							pid->iout = 0.8f*pid->iout;
//							pid->dout = 2.5f*pid->dout;
//					}
//					if(fabs(pid->err[NOW])>=0&&fabs(pid->err[NOW])<=5)
//					{
//							pid->pout = 0.75f*pid->pout;
//							pid->iout = pid->iout;
//							pid->dout = 4*pid->dout;
//					}
//				}
				
        abs_limit(&(pid->iout), pid->IntegralLimit);				//限制积分输出
        pid->pos_out = pid->pout + pid->iout + pid->dout;		// 计算总输出
        abs_limit(&(pid->pos_out), pid->MaxOutput);					// 限制总输出
        pid->last_pos_out = pid->pos_out;										//更新上一次总输出
    }
    else if(pid->pid_mode == DELTA_PID)					//增量式PID
    {
        pid->pout = pid->Kp * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->feedback[LLAST] = pid->feedback[LAST];
    pid->feedback[LAST] = pid->feedback[NOW];
    pid->target[LLAST] = pid->target[LAST];
    pid->target[LAST] = pid->target[NOW];

//    return pid->pid_mode==POSITION_PID ? (fabs(target)<=0.1? 0 : pid->pos_out) : pid->delta_out;
	return pid->pid_mode==POSITION_PID ?  pid->pos_out : pid->delta_out;
}

void M3508_follow(PID_TypeDef *pid_6020,float target,int v[])//底盘跟随
{
    float n;
	
//    if(CAN_GM6020[1].angle>4095)
//        angle=-8191+CAN_GM6020[1].angle;
//	else
//	float angle=CAN_GM6020[0].total_angle/8191*360;
	 
	PID_Calculate(pid_6020,target,CAN_GM6020[0].total_angle);
	n=-pid_6020->pos_out;
	if(fabs(CAN_GM6020[0].total_angle-target)<300)
	{
		CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],0+v[0],CAN_M3508[0].speed);
		CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],0+v[1],CAN_M3508[1].speed);
		CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],0+v[2],CAN_M3508[2].speed);
		CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],0+v[3],CAN_M3508[3].speed);
	}
	else
	{

		CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],n+v[0],CAN_M3508[0].speed);
		CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],n+v[1],CAN_M3508[1].speed);
		CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],n+v[2],CAN_M3508[2].speed);
		CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],n+v[3],CAN_M3508[3].speed);
	}
}

void M3508_follow_2(PID_TypeDef *pid_6020,float target,float angle,float speed,float target2)
{
//	  float num_angle=0;
	  static uint8_t spin_flag=1;
	  if(0<=CAN_GM6020[0].angle && CAN_GM6020[0].angle<target)
			angle_spin=target-CAN_GM6020[0].angle;
		else if(8191-4095+target<=CAN_GM6020[0].angle && CAN_GM6020[0].angle<=8191)
			angle_spin=8191-CAN_GM6020[0].angle+target;
		else 
	    angle_spin=target-CAN_GM6020[0].angle;
		if(fabs(angle_spin)<200) angle_spin=0;
	  if (angle_spin<0){ 		
			PID_Calculate(pid_6020,0,-angle_spin);
		  n=-pid_6020->pos_out;}
		else{
		  PID_Calculate(pid_6020,0,angle_spin);
			n=pid_6020->pos_out;}
			angle=(double)(-CAN_GM6020[0].angle+target)/8191*2*pi+angle;
    if(client_state_now.motion==2) 
			client_state_now.motion=PASTmotion;
		if(rc.key[9]&&motionflag){ //按f切换	模式
			motionflag=0;
		}
			else if(!rc.key[9]){
				motionflag=1;
		}
		if(!rc.key[14]){    //v小陀螺
			if(client_state_now.motion==1)
				n=0;
			if(spin_flag)
			{
				spin_flag=0;
				CAN_GM6020[0].total_angle=CAN_GM6020[0].angle;
			}
			CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],n+cos(angle+pi/4)*speed,CAN_M3508[0].speed);
			CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],n-sin(angle+pi/4)*speed,CAN_M3508[1].speed);
			CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],n+sin(angle+pi/4)*speed,CAN_M3508[2].speed);
			CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],n-cos(angle+pi/4)*speed,CAN_M3508[3].speed);
		}
		else
		{
			//小陀螺
			spin_flag=1;
			CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],target2+cos(angle+pi/4)*speed,CAN_M3508[0].speed);
			CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],target2-sin(angle+pi/4)*speed,CAN_M3508[1].speed);
			CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],target2+sin(angle+pi/4)*speed,CAN_M3508[2].speed);
			CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],target2-cos(angle+pi/4)*speed,CAN_M3508[3].speed);
     	if(PASTmotion!=2) PASTmotion=client_state_now.motion;
			client_state_now.motion=2;
		}
}

