#ifndef _PID_H
#define _PID_H
#include "main.h"

#define ABS(x)		(((x)>0)? (x): -(x))
#define pi 3.1415926535898
enum {
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,

    POSITION_PID,
    DELTA_PID,
};
extern uint8_t motionflag;
typedef struct _PID_Typedef
{
    float Kp;
    float Ki;
    float Kd;

    float target[3];					//目标值,包含NOW， LAST， LLAST上上次
    float feedback[3];					//测量值
    float err[3];							//误差

    float pout;								//p输出
    float iout;								//i输出
    float dout;								//d输出

    float pos_out;						//本次位置式输出
    float last_pos_out;					//上次位置式输出
    float pos_out0;						//位置式输出最小值

    float delta_u;						//本次增量值
    float last_delta_out;				//上次增量式输出
    float delta_out;					//本次增量式输出 = last_delta_out + delta_u

    float max_err;
    float deadband;						//err < deadband return

    uint32_t pid_mode;
    int MaxOutput;				//输出限幅
    uint32_t IntegralLimit;		    //积分限幅
} PID_TypeDef;

extern PID_TypeDef PID_GM6020[4];	// 定义GM6020 PID结构体
extern PID_TypeDef PID_GM6020_speed[4];
extern PID_TypeDef PID_M2006[];		// 定义M2006 PID结构体
extern PID_TypeDef PID_M2006_ANGLE[];	// 定义M2006 角度PID结构体
extern PID_TypeDef PID_M3508[7];	// 定义M3508 PID结构体
extern PID_TypeDef PID_M3508_ANGLE[];  // 定义M3508 角度PID结构体
extern PID_TypeDef PID_M3508_Follow[];
extern PID_TypeDef PID_HEAT_PWM;

void PID_Init(
    PID_TypeDef*	pid,
    uint32_t 			mode,
    uint32_t 			maxout,
    uint32_t 			intergral_limit,
    float 				kp,
    float 				ki,
    float 				kd
);
void PID_Reset(PID_TypeDef	*pid, float kp, float ki, float kd);

float PID_Calculate(PID_TypeDef *pid, float target, float feedback);
void M3508_follow(PID_TypeDef *pid_6020,float target,int v[]);
void M3508_follow_2(PID_TypeDef *pid_6020,float target,float angle,float speed,float target2);
	
#endif



