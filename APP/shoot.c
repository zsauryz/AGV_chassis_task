/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       shoot.c
 * @brief      
 * @note       
 * @Version    V1.0.0
 * @Date       2021.5    
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "shoot.h"

// ÉèÖÃÄ¦²ÁÂÖµç»ú
void Friction_SetSpeed(uint16_t speed_l,uint16_t speed_s)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,1100);	// H10 D ÓÒ±ß
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,1100);	// H11 C ×ó±ß
}

int  Judge_wheel_of(void)
{
    if(CAN_M3508[4].speed>1000 &&CAN_M3508[5].speed<-1000) 
			return 1;
	  else if(CAN_M3508[4].speed<100 && CAN_M3508[5].speed>-100)
			return 0;
		else
			return 2;
}
