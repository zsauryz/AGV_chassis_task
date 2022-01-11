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
#include "led_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void led_task(void const *pvParameters)
{
//    uint16_t i, j;
//    fp32 delta_alpha, delta_red, delta_green, delta_blue;
//    fp32 alpha,red,green,blue;
//    uint32_t aRGB;

    while(1)
    {

//        for(i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
//        {
//            alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
//            red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
//            green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
//            blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

//            delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
//            delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
//            delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
//            delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

//            delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
//            delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
//            delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
//            delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
//            for(j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
//            {
//                alpha += delta_alpha;
//                red += delta_red;
//                green += delta_green;
//                blue += delta_blue;

//                aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
//                aRGB_led_show(aRGB);
//  
//            }
              osDelay(10);
		}
}


