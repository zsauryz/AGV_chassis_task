/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       bsp_can.c
 * @brief      
 * @note       CAN1 CAN2
 * @Version    V1.0.0
 * @Date       2021.5
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */

#include "can.h"
#include "bsp_can.h"


//记录电机的反馈值（角度、速度、电流、温度）
CAN_GM6020_TypeDef 	CAN_GM6020[4]	=	{0};	// 云台电机
CAN_M2006_TypeDef 	CAN_M2006[2]	=	{0};	// 拨盘电机
CAN_M3508_TypeDef 	CAN_M3508[7]	=	{0};	// 前四个是底盘电机 后两个是摩擦轮电机

/*指示CAN1和CAN2是否正常通信*/
uint16_t CAN1_CNT;
uint16_t CAN2_CNT;
uint16_t CAN_6020_CNT;

float C_yaw,last_C_yaw,total_C_yaw=0;
float powerData[4];//超级电容的数据
float can6020lastvoltage=0;
uint8_t imuget_flag=0;
extern float GM6020_spin;
/*******************************************************************************************
  * @Func		CAN_FilterInit
  * @Brief    	CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     	2019.10.19
 *******************************************************************************************/
void CAN_FilterInit(CAN_HandleTypeDef* hcan)
{
    CAN_FilterTypeDef CAN_FilterStructure;

    if(hcan->Instance ==CAN1)
        CAN_FilterStructure.FilterBank = 0,CAN_FilterStructure.SlaveStartFilterBank  = 0;
    else if(hcan->Instance ==CAN2)
        CAN_FilterStructure.FilterBank = 14,CAN_FilterStructure.SlaveStartFilterBank  = 14;

    CAN_FilterStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterStructure.FilterIdHigh = 0x0000;
    CAN_FilterStructure.FilterIdLow = 0x0000;
    CAN_FilterStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterStructure.FilterActivation = ENABLE;
//    CAN_FilterStructure.SlaveStartFilterBank  = 14;

    if(HAL_CAN_ConfigFilter(hcan,&CAN_FilterStructure)!=HAL_OK) {
        Error_Handler();
    }
    if(HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK) {
        Error_Handler();
    }
    if(HAL_CAN_Start(hcan)!=HAL_OK) {
        Error_Handler();
    }
    HAL_CAN_ConfigFilter(hcan, &CAN_FilterStructure);        // init can filter
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/***********************************************
函数功能：can发送数据
入口参数：
			_hcan	can1 can2
			ide：	0：标准帧
					1：扩展帧
			id：	帧ID
			len：	数据长度
			data：	数据
返回值：0：成功。1：失败
************************************************/
uint8_t CAN_SendMsg(CAN_HandleTypeDef* hcan,uint8_t ide,uint32_t id,uint8_t len,uint8_t *data)
{
    uint32_t   TxMailbox;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    HAL_StatusTypeDef   HAL_RetVal;
    uint16_t i=0;
    if(ide == 0) {
        CAN_TxHeader.IDE = CAN_ID_STD;	//标准帧
        CAN_TxHeader.StdId = id;
    }
    else {
        CAN_TxHeader.IDE = CAN_ID_EXT;			//扩展帧
        CAN_TxHeader.ExtId = id;
    }
    CAN_TxHeader.DLC = len;
    CAN_TxHeader.RTR = CAN_RTR_DATA;//数据帧,CAN_RTR_REMOTE遥控帧
    CAN_TxHeader.TransmitGlobalTime = DISABLE;
    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        i++;
        if(i>0xfffe)
            return 1;
    }
    HAL_Delay(1);
    HAL_RetVal = HAL_CAN_AddTxMessage(hcan,&CAN_TxHeader,data,&TxMailbox);
    if(HAL_RetVal != HAL_OK) {
        return 1;
    }
	
    return 0;
}

void CAN_Superpower(uint16_t temPower)
{
    uint8_t data[8];

    data[0] = temPower >> 8;
    data[1] = temPower;		
    data[2] = 0;
    data[3] = 0;		
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    CAN_SendMsg(&hcan1,0,0x210,8,data);
}
/***********************************************
函数功能：设置M3508的电流
入口参数：
			id		电机的id
			i1		M3508（-16384~16384）（+-20A）
			i2		M2006（-16384~16384）（+-10A）

************************************************/

// 发送电流值给底盘电机电调
void CAN_Chassis_SendCurrent(void)
{
    uint8_t data[8];

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = CAN_M3508[0].set_current >> 8;
    data[5] = CAN_M3508[0].set_current;
    data[6] = CAN_M3508[1].set_current >> 8;
    data[7] = CAN_M3508[1].set_current;

    CAN_SendMsg(&hcan1,0,0x200,8,data);
	
	  data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = CAN_M3508[2].set_current >> 8;
    data[5] = CAN_M3508[2].set_current;
    data[6] = CAN_M3508[3].set_current >> 8;
    data[7] = CAN_M3508[3].set_current;
		
		CAN_SendMsg(&hcan2,0,0x200,8,data);

}

void CAN_Gimbal_SendVoltage(void)
{	
	  uint8_t   tx_data[8];
	
	  tx_data[0] = (CAN_GM6020[0].set_voltage>>8)&0xff;
    tx_data[1] = (CAN_GM6020[0].set_voltage)&0xff;	
    tx_data[2] = (CAN_GM6020[1].set_voltage>>8)&0xff;;
    tx_data[3] = (CAN_GM6020[1].set_voltage)&0xff;;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;
    CAN_SendMsg(&hcan1,0,0x1FF,8,tx_data);
	
  	tx_data[0] = (CAN_GM6020[2].set_voltage>>8)&0xff;
    tx_data[1] = (CAN_GM6020[2].set_voltage)&0xff;	
    tx_data[2] = (CAN_GM6020[3].set_voltage>>8)&0xff;;
    tx_data[3] = (CAN_GM6020[3].set_voltage)&0xff;;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;
    CAN_SendMsg(&hcan2,0,0x1FF,8,tx_data);
}

void CAN_Shoot_SendCurrent(void)
{
	  uint8_t data[8];
	
    data[0] = 0;
    data[1] = 0;
    data[2] = CAN_M3508[4].set_current >> 8;
    data[3] = CAN_M3508[4].set_current;
    data[4] = CAN_M3508[5].set_current >> 8;
    data[5] = CAN_M3508[5].set_current;
    data[6] = 0;
    data[7] = 0;
    CAN_SendMsg(&hcan2,0,0x200,8,data);
	
	  data[0] = 0;
    data[1] = 0;
    data[2] = CAN_M3508[5].set_current >> 8;
    data[3] = CAN_M3508[5].set_current;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    CAN_SendMsg(&hcan1,0,0x1FF,8,data);
}
void CAN_TA(void)
{
    uint8_t data[8];

    data[0] = (int16_t)(GameRobotState.robot_id) >> 8;
    data[1] = (int16_t)(GameRobotState.robot_id);
    data[2] = 0;
    data[3] = 0;			// 摩擦轮 右
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    CAN_SendMsg(&hcan1,0,0x212,8,data);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //接收回调函数
{
    HAL_StatusTypeDef		HAL_RetVal;	// 接受状态
    CAN_RxHeaderTypeDef     RxMeg;		// 接受报文结构体
    uint8_t					rx_data[8];	// 存储接收到的数据
    uint8_t 				id;			// 临时记录id
//	  uint8_t 				flag=-1; 
    /* CAN1 回调*/
    if(hcan->Instance == CAN1)
    {
        // 接收消息
        HAL_RetVal=HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMeg,  rx_data);
        if ( HAL_OK==HAL_RetVal)
        {
            /* 底盘 M3508的反馈信息处理 */
            if(RxMeg.StdId >= CAN1_M3508_ID3 & RxMeg.StdId <= CAN1_M3508_ID4)	/* 0x203 ~ 0x204*/
            {
                CAN1_CNT ++;
                id = RxMeg.StdId-CAN1_M3508_ID3;
                // 将数据保存在结构体中
                CAN_M3508[id].angle=(rx_data[0]<<8)+rx_data[1];
                CAN_M3508[id].speed=(rx_data[2]<<8)+rx_data[3];
                CAN_M3508[id].current=(rx_data[4]<<8)+rx_data[5];
                CAN_M3508[id].temperature=(rx_data[6]<<8);
            }

            else if(RxMeg.StdId == 0x211) {//接收超级电容的数据
                extern float powerData[4];
                uint16_t *pPowerdata = (uint16_t *)rx_data;

                powerData[0] = (float)pPowerdata[0]/100.f;//输入电压
                powerData[1] = (float)pPowerdata[1]/100.f;//电容电压
                powerData[2] = (float)pPowerdata[2]/100.f;//输入电流
                powerData[3] = (float)pPowerdata[3]/100.f;//设置功率
							  client_state_now.capacity_percent=(int)((powerData[1]*powerData[1]-16*16)/320*100);
								client_state_now.InputPower =  powerData[0]*powerData[2];
//							  client_state_now.InputPower =powerData[3];
            }
			      else  if(RxMeg.StdId >= CAN1_GM6020_ID1 & RxMeg.StdId <= CAN1_GM6020_ID2)	/* 0x203 ~ 0x204*/
            {
                CAN1_CNT ++;
                id = RxMeg.StdId-CAN1_GM6020_ID1;
                CAN_GM6020[id].angle    = ((rx_data[0] << 8) | rx_data[1]);
                CAN_GM6020[id].speed    = ((rx_data[2] << 8) | rx_data[3]);
                CAN_GM6020[id].current  = ((rx_data[4] << 8) | rx_data[5]);
                CAN_GM6020[id].temperature   =   rx_data[6];}
            if (CAN1_CNT == 500)
            {
                CAN1_CNT = 0;
            }
        }
				get_total_C_yaw();
				get_total_angle(&CAN_M3508[6]);
    }
    if(hcan->Instance == CAN2)
    {
        // 接收消息
        HAL_RetVal=HAL_CAN_GetRxMessage(&hcan2,  CAN_RX_FIFO0, &RxMeg,  rx_data);
        if ( HAL_OK==HAL_RetVal)
        {

            /* GM6020的反馈信息处理 */
			      if(RxMeg.StdId >= CAN2_GM6020_ID1 & RxMeg.StdId <= CAN2_GM6020_ID2)	/* 0x203 ~ 0x204*/
            {
                CAN2_CNT ++;
                id = RxMeg.StdId-CAN2_GM6020_ID1+2;
                CAN_GM6020[id].angle    = ((rx_data[0] << 8) | rx_data[1]);
                CAN_GM6020[id].speed    = ((rx_data[2] << 8) | rx_data[3]);
                CAN_GM6020[id].current  = ((rx_data[4] << 8) | rx_data[5]);
                CAN_GM6020[id].temperature   =   rx_data[6];
            }
            if(RxMeg.StdId >= CAN2_M3508_ID3 & RxMeg.StdId <= CAN2_M3508_ID4)	/* 0x203 ~ 0x204*/
            {
                CAN1_CNT ++;
                id = RxMeg.StdId-CAN2_M3508_ID3+2;
                // 将数据保存在结构体中
                CAN_M3508[id].angle=(rx_data[0]<<8)+rx_data[1];
                CAN_M3508[id].speed=(rx_data[2]<<8)+rx_data[3];
                CAN_M3508[id].current=(rx_data[4]<<8)+rx_data[5];
                CAN_M3508[id].temperature=(rx_data[6]<<8);
            }
            if (CAN2_CNT == 500)
            {
                CAN2_CNT = 0;
 //             LED_R_Toggle();		
            }
        }
    }
		
}
void get_total_angle(CAN_M3508_TypeDef *p) {

    int res1, res2;
    float delta;
    if(p->angle < p->last_angle) {						//可能的情况
        res1 = p->angle + 8192 - p->last_angle;			//正转，delta=+
        res2 = p->angle - p->last_angle;				//反转	delta=-
    } 
	else {	//angle > last
        res1 = p->angle - 8192 - p->last_angle ;		//反转	delta -
        res2 = p->angle - p->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(__fabs(res1)<__fabs(res2))
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta/10;
    p->last_angle = p->angle;
}
void get_total_angle_2006(CAN_M2006_TypeDef *p) {

    int res1, res2;
    float delta;
    if(p->angle < p->last_angle) {						//可能的情况
        res1 = p->angle + 8192 - p->last_angle;			//正转，delta=+
        res2 = p->angle - p->last_angle;				//反转	delta=-
    } 
	else {	//angle > last
        res1 = p->angle - 8192 - p->last_angle ;		//反转	delta -
        res2 = p->angle - p->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(__fabs(res1)<__fabs(res2))
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta*0.001220703125f;
    p->last_angle = p->angle;
}
void get_total_angle_6020(CAN_GM6020_TypeDef *p) {

    int res1, res2;
    float delta;
    if(p->angle < p->last_angle) {						//可能的情况
        res1 = p->angle + 8192 - p->last_angle;			//正转，delta=+
        res2 = p->angle - p->last_angle;				//反转	delta=-
    } 
	else {	//angle > last
        res1 = p->angle - 8192 - p->last_angle ;		//反转	delta -
        res2 = p->angle - p->last_angle;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(__fabs(res1)<__fabs(res2))
	{
		delta = res1;		
	}
    else
        delta = res2;
	if(p->last_angle-p->angle>6000)//GM6020_spin++;
	{
		GM6020_spin++;
	}
	else if(p->angle-p->last_angle>6000)
	{
		GM6020_spin--;
	}
    p->total_angle += delta;
    p->last_angle = p->angle;
}
void get_total_C_yaw() {

    float res1, res2;
    float delta;
    if(C_yaw < last_C_yaw) {						//可能的情况
        res1 = C_yaw + 360 - last_C_yaw;			//正转，delta=+
        res2 = C_yaw - last_C_yaw;				//反转	delta=-
    } 
	else {	//angle > last
        res1 = C_yaw - 360 - last_C_yaw ;		//反转	delta -
        res2 = C_yaw - last_C_yaw;				//正转	delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if(__fabs(res1)<__fabs(res2))
        delta = res1;
    else
        delta = res2;

    total_C_yaw += delta;
    last_C_yaw = C_yaw;
		imuget_flag=1;
}
