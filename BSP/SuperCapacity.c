#include "stm32f4xx_hal.h"
#include "can.h"
#include "bsp_can.h"
#include "SuperCapacity.h"
uint8_t sendbuf[8];

//³¬¼¶µçÈÝ
void SuperCapacitance(uint16_t temPower) {
	
    sendbuf[0] = temPower*100 >>8;
    sendbuf[1] = temPower*100 ;

    CAN_SendMsg(&hcan1,0,0x210,8,sendbuf);
		
	HAL_Delay(200);
}
