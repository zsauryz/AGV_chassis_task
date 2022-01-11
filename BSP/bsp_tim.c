/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       bsp_pwm.c
 * @brief      
 * @note       TIM1 (IN1) (IN2) (IN3) (IN4)
 * 		       TIM5 (IN1) (IN2)
 * @Version    V1.0.0
 * @Date       2021.5     
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_tim.h"

uint16_t tim11_cnt=0;
float PITlastangle,YAWlastangle;
float PITthisangle,YAWthisangle;
float GM6020PITspeed,GM6020YAWspeed;
float GM6020PITspeed_last=0,GM6020YAWspeed_last=0;
extern imu_t imu;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}

