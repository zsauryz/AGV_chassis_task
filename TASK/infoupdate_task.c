/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  */
#include "infoupdate_task.h"
#include "gimbal.h"
#include "kalman.h"
#include "chassis_task.h"
#include "bsp_usart.h"

int time_info;

uint16_t dialspd=2000,Startflag=0;
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void info_update_task(void const *pvParameters)
{
    //空闲一段时间
    osDelay(INFO_UPDATE_INIT_TIME);

    while (1)
    {
			
      time_info++;
				if(rc_flag<3)
				{
					  if(!Startflag)
							dialspd=2000,
							CAN_GM6020[0].total_angle=CAN_GM6020[0].angle,Startflag=1;
				}
      //系统绝对延时
      osDelay(10);
    }
}

