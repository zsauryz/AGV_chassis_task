/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       bsp_usart.c
 * @brief      
 * @note       
*
 * @Version    V1.0.0
 * @Date       2021.5    
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_usart.h"

// 山外串口调试助手底层函数
void UART_PutBuff (UART_HandleTypeDef *huart,uint8_t *buff, uint32_t len)
{
    while(len)
    {
        HAL_UART_Transmit(huart,buff,1,1);
        buff++;
        len--;
    }
}
void VCAN_SendWare(UART_HandleTypeDef *huart,void *wareaddr, uint32_t waresize)
{
#define CMD_WARE     3	// 串口调试助手 虚拟示波器的命令
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

    UART_PutBuff(huart,cmdf, sizeof(cmdf));    //先发送前命令
    UART_PutBuff(huart,(uint8_t *)wareaddr, waresize);    //发送数据
    UART_PutBuff(huart,cmdr, sizeof(cmdr));    //发送后命令
}

// 重定向串口2的printf
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
uint8_t aRxBuffer[1];		// 串口2的接收缓冲区，用于初始化，然后将值一个一个保存在USART2_RX_BUF中
uint8_t USART2_RX_BUF[200];	//最大接收200字节
uint16_t USART_RX_STA=0;       //接收状态标记
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->SR & (__FLAG__)) == (__FLAG__))
#define __HAL_UART_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->SR = ~(__FLAG__))
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
        uint32_t isrflags   = READ_REG(huart->Instance->SR);//手册上有讲，清错误都要先读SR
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))!=RESET)
        {
                READ_REG(huart->Instance->DR);//PE清标志，第二步读DR
                __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE);//清标志
        }
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))!=RESET)
        {
                READ_REG(huart->Instance->DR);//FE清标志，第二步读DR
                __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
        }
        
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))!=RESET)
        {
                READ_REG(huart->Instance->DR);//NE清标志，第二步读DR
                __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
        }        
        
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))!=RESET)
        {
                READ_REG(huart->Instance->CR1);//ORE清标志，第二步读CR
                __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
        }        
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


//	Beep(200);
//	printf("%s","11   ");
    if (huart == &huart1)
    {
//		OLED_Printf(1,1,"12");
//		OLED_RefreshGram();
		VisionUartRxCpltCallback();
//		printf("%s","33   ");
//		VisionUartRxCpltCallback_2();
	
    }
	if(huart == &huart6)
	{	//
//		LED_R_On();
//		HAL_UART_Receive_IT(&huart6, (uint8_t *)aRxBuffer, 1);
//		HAL_UART_Transmit_IT(&huart6, "12121", 10);
////		printf("%d",1);
//		BT_SendWave();
	}
    if (huart == &JUDGE_UART)
    {
        judgeUartRxCpltCallback();
    }
	HAL_UART_ErrorCallback(huart);
}
