/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       bsp_rc.c
 * @brief      this file contains rc data receive and processing function
 * @note       USART1 + DMA2 stream2
 * @Version    V1.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "bsp_rc.h"
#include "usart.h"

uint8_t   dbus_buf[DBUS_BUFLEN];
rc_info_t rc;
//bool rckey[8];
extern uint8_t rc_flag;


/**
  * @brief      enable global uart it and do not use DMA transfer done it
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
    uint32_t tmp1 = 0;

    tmp1 = huart->RxState;

    if (tmp1 == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode  = HAL_UART_ERROR_NONE;

        /* Enable the DMA Stream */
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

        /*
         * Enable the DMA transfer for the receiver request by setting the DMAR bit
         * in the UART CR3 register
         */
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param[in]  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
    /* Return the number of remaining data units for DMAy Streamx */
    return ((uint16_t)(dma_stream->NDTR));
}



/**
  * @brief       handle received rc data
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval
  */

void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
    rc->ch0 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch0 -= 1024;
    rc->ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch3 -= 1024;
	  rc->ch4 = (buff[16] | buff[17] << 8) & 0x07FF;
    rc->ch4 -= 1024;
	
    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

	rc->mouse_x = ((int16_t)buff[6]) | ((int16_t)buff[7] << 8);
	rc->mouse_y = ((int16_t)buff[8]) | ((int16_t)buff[9] << 8);
	rc->mouse_z = ((int16_t)buff[10]) | ((int16_t)buff[11] << 8); 
	
	rc->mouse_press_l = buff[12];
	rc->mouse_press_r = buff[13];
	
	//rc->key_v = ((int16_t)buff[14]);// | ((int16_t)pData[15] << 8);
		
	rc->key[15]=(buff[15]&0x80)>>7;  //b
	rc->key[14]=(buff[15]&0x40)>>6;  //v
	rc->key[13]=(buff[15]&0x20)>>5;	//c
	rc->key[12]=(buff[15]&0x10)>>4;  //x
	rc->key[11]=(buff[15]&0x08)>>3;  //z
	rc->key[10]=(buff[15]&0x04)>>2;	//g
	rc->key[9]=(buff[15]&0x02)>>1;	//f
	rc->key[8]=	buff[15]&0x01;		//r
	rc->key[7]=(buff[14]&0x80)>>7;  //e
	rc->key[6]=(buff[14]&0x40)>>6;  //q
	rc->key[5]=(buff[14]&0x20)>>5;	//ctrl
	rc->key[4]=(buff[14]&0x10)>>4;  //shift
	rc->key[3]=(buff[14]&0x08)>>3;  //d
	rc->key[2]=(buff[14]&0x04)>>2;	//a
	rc->key[1]=(buff[14]&0x02)>>1;	//s
	rc->key[0]=	buff[14]&0x01;		//w
//	int16_t temp=rc->mouse_x;
//	printf("%d\n",temp);
	//HAL_UART_Transmit_DMA(&huart1 ,(uint8_t*)temp,sizeof(temp));
//	if(((int16_t)buff[14])==0x)
//printf("%d\n",buff[14]);
//			for(int i=0;i<8;i++)
//			printf("%d ",rc.key[i]);
//		printf("\n");
	if ((abs(rc->ch0) > 660) || \
            (abs(rc->ch1) > 660) || \
            (abs(rc->ch2) > 660) || \
            (abs(rc->ch3) > 660))
    {
        memset(rc, 0, sizeof(rc_info_t));
    }
}
/**
  * @brief      clear idle it flag after uart receive a frame data
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* handle received data in idle interrupt */
    if (huart == &DBUS_HUART)
    {
        /* clear DMA transfer complete flag */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* handle dbus data dbus_buf from DMA */
        if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
        {
            rc_callback_handler(&rc, dbus_buf);
//					  Beep_On();
        }

        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

/**
  * @brief      callback this function when uart interrupt
  * @param[in]  huart: uart IRQHandler id
  * @retval
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
            __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
    {
        uart_rx_idle_callback(huart);
		    rc_flag=0;
    }
}

/**
  * @brief   initialize dbus uart device
  * @param
  * @retval
  */
void dbus_uart_init(void)
{
    /* open uart idle it */
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

    uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}



