/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       led.c
 * @brief      
 * @note       PE11 PF14 
 * @Version    V1.0.0
 * @Date       2021.5     
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_led.h"

/* µãÁÁÂÌµÆ */
//void LED_G_On(void)
//{
//	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
//}
/* µãÁÁºìµÆ */
void LED_R_Off(void)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
}
/* Ï¨ÃðÂÌµÆ */
//void LED_G_Off(void)
//{
//	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
//}
/* Ï¨ÃðºìµÆ */
void LED_R_On(void)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}
/* ·­×ªÂÌµÆ */
//void LED_G_Toggle(void)
//{
//	HAL_GPIO_TogglePin (LED_G_GPIO_Port, LED_G_Pin);
//}
/* ·­×ªºìµÆ */
void LED_R_Toggle(void)
{
	HAL_GPIO_TogglePin (LED_R_GPIO_Port, LED_R_Pin);
}
/* ºìµÆÂÌµÆÉÁË¸ */
//void LED_RG_Twinkle(uint32_t t)
//{
//	LED_G_On();
//	LED_R_On();
//	HAL_Delay(t);
//	LED_G_Off();
//	LED_R_Off();
//	HAL_Delay(t);
//}

void LASER_On(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}
// ¹Ø±Õ¼¤¹â
void LASER_Off(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
}
