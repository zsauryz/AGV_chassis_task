#ifndef __BSP_USART
#define __BSP_USART
#include "main.h"

#define JUDGE_UART		huart6   //C板丝印对应uart1
#define VISION_UART		huart1   //C板丝印对应uart2

extern uint8_t aRxBuffer[];		// 串口2的接收缓冲区，用于初始化，然后将值一个一个保存在USART2_RX_BUF中
extern uint8_t USART2_RX_BUF[];	//最大接收200字节
extern uint16_t USART_RX_STA;

void UART_PutBuff (UART_HandleTypeDef *huart,uint8_t *buff, uint32_t len);
void VCAN_SendWare(UART_HandleTypeDef *huart,void *wareaddr, uint32_t waresize);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
