/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       shoot.c
 * @brief      
 * @note       
 * @Version    V1.0.0
 * @Date       2021.5    
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
 
#include "shoot_task.h"
uint16_t time_shoot;
extern uint16_t RC_FLAG;

uint8_t ShotFlag;  //记录发射模式选择枪管
uint8_t shootflag2=1; //记录摩擦轮开关
uint16_t shoot_speed,dial_speed;   //发射弹速，发射频率

/**
  * @brief          发射遥控器控制函数
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void shoot_rc_ctrl(void);


/**
  * @brief          底盘任务，间隔 SHOOT_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void shoot_task(void const *pvParameters)
{
    //空闲一段时间
    osDelay(SHOOT_CONTROL_INIT_TIME);
	  shoot_speed=5000;//摩擦轮速度
	  dial_speed=500;//拨弹速度4350
    static portTickType lastWakeTime;  
		lastWakeTime = xTaskGetTickCount(); 
    while (1)
    {
			  time_shoot++;
			  if(rc_flag<3)
			  {
          shoot_rc_ctrl();
			  }
				else    //若遥控器失控关闭清空电流值
			  {
            CAN_M3508[4].set_current=0;
            CAN_M3508[5].set_current=0;					
			  }
        //CAN_Shoot_SendCurrent();
        //系统绝对延时
        osDelayUntil(&lastWakeTime,SHOOT_CONTROL_TIME_MS);
    }
}

void shoot_rc_ctrl(void){
if(rc.sw2!=1) {
# if RACEFLAG
		CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], shoot_speed, CAN_M3508[4].speed);
    CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], -shoot_speed, CAN_M3508[5].speed);
		CAN_M3508[6].set_current =  PID_Calculate(&PID_M3508[6], shoot_speed, CAN_M3508[6].speed);
    CAN_M3508[7].set_current =  PID_Calculate(&PID_M3508[7], -shoot_speed,CAN_M3508[7].speed);
# else

	if(rc.sw1!=1){shootflag2=0;}//按CTRL打开摩擦轮
	else{shootflag2=1;}//按G关闭摩擦轮
	
	if(shootflag2)
	{
		CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], 0, CAN_M3508[4].speed);
    CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], 0, CAN_M3508[5].speed);
	}
	
	else
	{
		CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -shoot_speed, CAN_M3508[4].speed);
    CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], shoot_speed, CAN_M3508[5].speed);
	}
#endif
		/*************************按f切换弹道*******************************/

		ShotFlag = 0;
//	ShotFlag = 2;
	if(rc.sw1==2){
		if(ShotFlag==0)
		{
//		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],dial_speed, CAN_M2006[0].speed);
//		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],0, CAN_M2006[1].speed);
		}
		else if(ShotFlag==1)
		{
//		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],0, CAN_M2006[0].speed);
//		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],dial_speed, CAN_M2006[1].speed);

		}
		else
		{
//		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],-dial_speed, CAN_M2006[0].speed);
//		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],dial_speed, CAN_M2006[1].speed);
		}
	}
	else{
//		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],0, CAN_M2006[0].speed);
//		CAN_M2006[1].set_current = PID_Calculate(&PID_M2006[1],0, CAN_M2006[1].speed);

	}
}
}
