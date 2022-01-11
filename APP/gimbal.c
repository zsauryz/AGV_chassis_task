/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       gimbal.c
 * @brief     
 * @note
 * @Version    V1.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
#include "gimbal.h"
#include "bsp_led.h"
#define num 10
float spin_angle_set = 0/*1120*/,pitch_angle_set=285,gyro_angle_set=0;
uint8_t Vision_receiving = 0;
uint8_t Vision_buffer[num] = {0}; 
uint8_t Vision_buffercnt = 0;
uint8_t tmp_vision;
VisionData datadata;
uint8_t Tdata[num];
uint8_t aTxVisionMessages[22];
extern PID_TypeDef PID_GM6020[];
float p=0;
//extern float Gyro[3],Accel[3];

void VisionUartRxCpltCallback()
{
	if(Vision_receiving){
		Vision_buffer[Vision_buffercnt] = tmp_vision;
		Vision_buffercnt++;
		if(tmp_vision == 0x65){
			if(Vision_buffer[1]>>7)
				datadata.yaw_angle=	-(100-(((Vision_buffer[1]<<8)| Vision_buffer[2])&0x7fff)*0.003051850947599);
			else
				datadata.yaw_angle=	(((Vision_buffer[1]<<8)| Vision_buffer[2])&0x7fff)*0.003051850947599;
			if(Vision_buffer[3]>>7)
				datadata.pitch_angle=  	-(100-(((Vision_buffer[3]<<8)| Vision_buffer[4])&0x7fff)*0.003051850947599);
			else
				datadata.pitch_angle=		(((Vision_buffer[3]<<8)| Vision_buffer[4])&0x7fff)*0.003051850947599;
			if(Vision_buffer[5]>>7)
			datadata.dis=	   			-(100-(((Vision_buffer[5]<<8)| Vision_buffer[6])&0x7fff)*0.003051850947599);
			else
				datadata.dis=	   		(((Vision_buffer[5]<<8)| Vision_buffer[6])&0x7fff)*0.003051850947599;
			Vision_buffercnt = 0,Vision_receiving=0;
			
//			printf("%f %f %f \n",datadata.pitch_angle,datadata.yaw_angle,datadata.dis);
			
		}
	}
	else{
		if(tmp_vision == 0x73){

			Vision_receiving = 1;
			Vision_buffercnt = 0;
			Vision_buffer[0] = tmp_vision;
			Vision_buffercnt++;
		}
	}

//	HAL_UART_Receive_IT(&huart6, &tmp_vision, 1);
	if(HAL_UART_Receive_DMA(&huart1, &tmp_vision, 1) != HAL_OK){
		Error_Handler();
	}
}
//void VisionUartRxCpltCallback_2()
//{
////	printf("%s","11");
//	if(Vision_receiving){
//		Vision_buffer[Vision_buffercnt] = tmp_vision;
//		Vision_buffercnt++;
//		if(tmp_vision == 0x65){
//			if(Vision_buffer[1]>>7)
//				datadata.pitch_angle=	-(100-(((Vision_buffer[1]<<8)| Vision_buffer[2])&0x7fff)*0.003051850947599);
//			else
//				datadata.pitch_angle=	(((Vision_buffer[1]<<8)| Vision_buffer[2])&0x7fff)*0.003051850947599;
//			if(Vision_buffer[3]>>7)
//				datadata.yaw_angle=  	-(100-(((Vision_buffer[3]<<8)| Vision_buffer[4])&0x7fff)*0.003051850947599);
//			else
//				datadata.yaw_angle=		(((Vision_buffer[3]<<8)| Vision_buffer[4])&0x7fff)*0.003051850947599;
//			if(Vision_buffer[5]>>7)
//			datadata.dis=	   			-(100-(((Vision_buffer[5]<<8)| Vision_buffer[6])&0x7fff)*0.003051850947599);
//			else
//				datadata.dis=	   		(((Vision_buffer[5]<<8)| Vision_buffer[6])&0x7fff)*0.003051850947599;
//			Vision_buffercnt = 0,Vision_receiving=0;
//			//HAL_UART_Receive_IT(&huart6, (uint8_t *)aRxBuffer, 1);
//			
//			printf("%f %f %f \n",datadata.pitch_angle,datadata.yaw_angle,datadata.dis);
//			
//		}
//	}
//	else{
//		if(tmp_vision == 0x73){

//			Vision_receiving = 1;
//			Vision_buffercnt = 0;
//			Vision_buffer[0] = tmp_vision;
//			Vision_buffercnt++;
//		}
//	}

//	HAL_UART_Receive_DMA(&huart1, &tmp_vision, 1);
////	if(HAL_UART_Receive_DMA(&huart6, &tmp_vision, 1) != HAL_OK){
////		Error_Handler();
////	}
//}
//void VisionUartRxCpltCallback_2()
//{
//static int CNT=0;if (CNT >= 10){CNT = 0;LED_R_Toggle();}CNT++;
//	if(Vision_receiving){
//		Vision_buffer[Vision_buffercnt] = tmp_vision;
//		Vision_buffercnt++;
//		if(Vision_buffercnt >=num){
//			datadata.pitch_angle.f=((Vision_buffer[1]<<8)| Vision_buffer[2]);
////			datadata.yaw_angle.f=  ((Vision_buffer[3]<<8)| Vision_buffer[4])*10/(32768-1);
////			datadata.z_angle.f=	   ((Vision_buffer[5]<<8)| Vision_buffer[6])*10/(32768-1);
////			datadata.dis.f=		   ((Vision_buffer[7]<<8)| Vision_buffer[8])*10/(32768-1);
////			datadata.pitch_angle.c[0]=Vision_buffer[1]<<8;
////			datadata.pitch_angle.c[1]=Vision_buffer[2];
//			Vision_buffercnt = 0,Vision_receiving=0;
////			LED_R_On();
//			
//		}
//	}
//	else{
//		if(tmp_vision == 0x73){
//			Vision_receiving = 1;
//			Vision_buffercnt = 0;
//			Vision_buffer[0] = tmp_vision;
//			Vision_buffercnt++;
//			}
//		}

//	

//	HAL_UART_Receive_DMA(&huart1, &tmp_vision, 1);
////	HAL_UART_Receive_DMA(&huart1, "121212", 10);
////	if(HAL_UART_Receive_DMA(&huart6, &tmp_vision, 1) != HAL_OK){
////		Error_Handler();
////	}
//}
//void VisionUartTxSenddata()
//{
////	datadata.pitch_angle.f=10.22;
////	datadata.yaw_angle.f=7.98;
//	
//	aTxVisionMessages[0] = 0xA5;
////    aTxVisionMessages[1] = CmdID1;
////    crcToUse.Append_CRC8_Check_Sum(aTxVisionMessages, 3);
//     
//    aTxVisionMessages[3] = datadata.pitch_angle.c[0];
//    aTxVisionMessages[4] = datadata.pitch_angle.c[1];
//    aTxVisionMessages[5] = datadata.pitch_angle.c[2];
//    aTxVisionMessages[6] = datadata.pitch_angle.c[3];

//    aTxVisionMessages[7] = datadata.yaw_angle.c[0];
//    aTxVisionMessages[8] = datadata.yaw_angle.c[1];
//    aTxVisionMessages[9] = datadata.yaw_angle.c[2];
//    aTxVisionMessages[10] = datadata.yaw_angle.c[3];

//    aTxVisionMessages[11] = datadata.dis.c[0];
//    aTxVisionMessages[12] = datadata.dis.c[1];
//    aTxVisionMessages[13] = datadata.dis.c[2];
//    aTxVisionMessages[14] = datadata.dis.c[3];

//    aTxVisionMessages[15] = datadata.ismiddle;
//    aTxVisionMessages[16] = datadata.isFindTarget;

//    aTxVisionMessages[17] = datadata.isfindDafu;
//    aTxVisionMessages[18] = 0x00;
//    aTxVisionMessages[19] = datadata.nearFace;
////	for(int i=0;i<sizeof(aTxVisionMessages);i++)
////	{
////		aTxVisionMessages[i]=0;
////	}
//	HAL_UART_Transmit_DMA(&huart1 ,(uint8_t*)aTxVisionMessages,sizeof(aTxVisionMessages));
////	HAL_UART_Transmit_DMA(&huart1 ,Gyro,sizeof(Gyro));
////    crcToUse.Append_CRC16_Check_Sum(Tdata, 22);
////    write(fd, Tdata, 22);
//}
void  Vision_aiming()
{
//	if(datadata.dis.f>0){
//		if(datadata.yaw_angle.f<15&&datadata.yaw_angle.f>-15&&datadata.pitch_angle.f<20&&datadata.pitch_angle.f>-20){
	

	if(datadata.dis!=0)
	{	
		p=CAN_GM6020[0].total_angle+datadata.pitch_angle;
//		y=CAN_GM6020[1].angle+datadata.yaw_angle+5 ;		

	}
		CAN_GM6020[0].set_voltage = PID_Calculate(&PID_GM6020[0],p,CAN_GM6020[0].total_angle);
//		CAN_GM6020[1].set_voltage = PID_Calculate(&PID_GM6020[1],y,CAN_GM6020[1].angle);			
		
}
void Pluck_angle(float angle,int m)
{
	  float n;
    n=PID_Calculate(&PID_M3508_ANGLE[6],-angle,CAN_M3508[6].total_angle/8191*360);
	  CAN_M3508[6].set_current = PID_Calculate(&PID_M3508[6],n,CAN_M3508[6].speed);
//	CAN_M2006.total_angle=0;
}
