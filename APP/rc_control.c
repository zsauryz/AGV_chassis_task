/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       rc_control.c
 * @brief
 * @note
 * @Version    V3.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "rc_control.h"
#include "math.h"
#include "pid.h"

#define RACEFLAG 0
#define __weak		__attribute__((weak))
#define pitch_angle_min	283 //6280
#define pitch_angle_max	315//7400
extern imu_t imu;
extern int 			setSpeed[4];
extern VisionData datadata;
extern float total_C_yaw;
extern uint8_t coolingflag;
float Vision_aiming_pitch=0,Vision_aiming_yaw=-5;
float P_angle=0;
float GM6020_1feedback,GM6020_1feedback1,GM6020_flag=0,Flag=0,Follow_3508=0,Follow_3508_2=0;
shoot_control_t shoot_control;
float temp_speed;
float GM6020_spin=0;
extern float GM6020PITspeed;//pitch轴速度
int vl=1,v2=0;
float last_pitch_angle_set=0,imuyaw;
int16_t x,y;
float pix;
float spin_6020angle,spin_imu_yaw;
float temp1;
float temp2;

int shootcount=0;
float smoothTime=0;
int Tcount=0;
//float smooth=0;
float pitch_angle_lastset=285;


/*
SW1 = 1	: 不发射
SW1 = 2 ：开启拨盘和摩擦轮
SW1 = 3 ：只开启摩擦轮
*/
void RC_Shoot(float fri_speed,float dial_speed)
{
    CAN_M3508[0].set_current=0;
    CAN_M3508[1].set_current=0;
    CAN_M3508[2].set_current=0;
    CAN_M3508[3].set_current=0;
    // 根据SW1设置发射机构电机电流
    switch(rc.sw1)
    {
//	if(CAN_M2006[0].speed<dial_speed/10)rc.sw1=1;
    case 1:
        CAN_M3508[4].set_current = PID_Calculate(&PID_M3508[4], 0, CAN_M3508[4].speed);
        CAN_M3508[5].set_current = PID_Calculate(&PID_M3508[5], 0, CAN_M3508[5].speed);
        CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6],0,CAN_M3508[6].speed);
        break;
    case 3:
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6],0, CAN_M3508[6].speed);//0
        break;
    case 2:
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[0], dial_speed, CAN_M3508[6].speed);
        break;
    }
    // 发送电流值给电机
//    CAN_Shoot_SendCurrent();
}
void RC_Singleshot(float fri_speed,uint8_t pattern)//pattern为6时为小弹丸，4时为大弹丸
{
    temp_speed=CAN_M3508[4].speed;
    switch(rc.sw1)
    {

//	if(CAN_M2006[0].speed<dial_speed/10)rc.sw1=1;
    case 1:
        CAN_M3508[4].set_current = PID_Calculate(&PID_M3508[4], 0, CAN_M3508[4].speed);
        CAN_M3508[5].set_current = PID_Calculate(&PID_M3508[5], 0, CAN_M3508[5].speed);
//       CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6],0,CAN_M3508[6].speed);
        break;
    case 3:
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
//		CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6],0,CAN_M3508[6].speed);
//        if(rc_f==1){Pluck_angle(360/pattern*rc_n,0);rc_n++;rc_f=0;}
//		else{Pluck_angle(360/pattern*rc_n,0);}
        break;
    case 2:
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
//				LASER_On();
//        if(rc_f==0){Pluck_angle(360/pattern*rc_n,0);rc_n++;rc_f=1;}
//		else{Pluck_angle(360/pattern*rc_n,0);}
        break;
    }
    // 发送电流值给电机
//    CAN_Shoot_SendCurrent();
}
void RC_Vision_aiming()
{
    Vision_aiming();
//	RC_Shoot(fri_speed,dial_speed);
    if(rc.ch2!=0||rc.ch3!=0) {
        velocity(rc.ch2,rc.ch3);
        for(int i=0; i<4; i++) 	//对M3508的操作
        {
            setSpeed[i] = map(setSpeed[i],-660,660,-4000,4000);
            CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
        }
    }
    else
//		M3508_follow(&PID_M3508_Follow,0);
    {
        CAN_M3508[0].set_current = 0;
        CAN_M3508[1].set_current = 0;
        CAN_M3508[2].set_current = 0;
        CAN_M3508[3].set_current = 0;

    }
//    CAN_Gimbal_SendVoltage();
//    CAN_Chassis_SendCurrent();
}

