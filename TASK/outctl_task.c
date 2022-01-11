/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       led_task.c/h
  * @brief      led task,
  *             led灯任务
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
#include "outctl_task.h"
extern uint16_t RC_FLAG;
uint16_t time_crl;
uint8_t rc_flag=0;
/**
  * @brief          失控保护任务，间隔 OUTCTL_TASK_TIME_MS 100ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void outctl_task(void const *pvParameters)
{
    //空闲一段时间
    osDelay(OUTCTL_TASK_INIT_TIME);

    while (1)
    {
			    time_crl++;
//        if(RC_FLAG==1)
//            Beep_Off();
//        else {
//            CAN_M3508[0].set_current=0;
//            CAN_M3508[1].set_current=0;
//            CAN_M3508[2].set_current=0;
//            CAN_M3508[3].set_current=0;
//            CAN_M3508[4].set_current=0;
//            CAN_M3508[5].set_current=0;
//						CAN_M3508[6].set_current=0;
//            CAN_M3508[7].set_current=0;
//					  CAN_GM6020[0].set_voltage=0;
//            CAN_GM6020[1].set_voltage=0;
//						CAN_M2006[0].set_current=0;
//						CAN_M2006[1].set_current=0;
//						CAN_M2006[2].set_current=0;					
//            CAN_Chassis_SendCurrent();
//            CAN_Shoot_SendCurrent();

//        }
        rc_flag++;
				if(rc_flag>10) rc_flag=10;
        //系统相对延时
        osDelay(OUTCTL_TASK_TIME_MS);
    }
}

