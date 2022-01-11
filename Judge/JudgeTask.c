	/**
  ******************************************************************************
  * File Name          : JudgeTask.c
  * Description        : 裁判系统处理任务，得到裁判系统信息
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "JudgeTask.h"
#include "mycrc.h"
#include <stdlib.h>
#include <string.h>    
#include "usart.h"
#include "bsp_can.h"

uint8_t datalength,lenth;
uint8_t CliendTxBuffer[128];
uint8_t flag=0;
uint8_t tmp_judge;

int8_t hurtSum,hurtInit,hurtLck;
int16_t hurtTic;
float2uchar ph01;
ext_SendClientData_graphic_t  GraphicData;
ext_SendClientData_character_t CharacterData;
ext_client_state_t  client_state_now;
ext_client_state_t  client_state_past;
void InitJudgeUart(void) {
   // tx_free = 1;
//	Send_User_Data();
//	Referee_Transmit_UserData();
	  Referee_Transmit_graphic();
    if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK) {
        Error_Handler();
    }
	HAL_UART_Transmit_DMA(&JUDGE_UART, &tmp_judge, 1);
}
uint8_t receiving = 0;
uint8_t received = 0;
uint8_t buffer[128] = {0};
uint8_t buffercnt = 0;
uint16_t cmdID;

void judgeUartRxCpltCallback(void) {

    if(receiving) {
        if(buffercnt >40)buffercnt = 4;
        buffer[buffercnt] = tmp_judge;
        buffercnt++;

        if(buffercnt == 5) {
            if (Verify_CRC8_Check_Sum(buffer, 5)==0) {
                receiving = 0;
                buffercnt = 0;
            }
        }

        if(buffercnt == 7) 
					cmdID = (0x0000 | buffer[5]) | (buffer[6] << 8);

        if(buffercnt == 36 && cmdID == 0x0201) {
					if (Verify_CRC16_Check_Sum(buffer, 36)==1) {
                Referee_Update_RobotState();
        }}

        if(buffercnt == 25 && cmdID == 0x0202) {
					if (Verify_CRC16_Check_Sum(buffer, 25)==1) {
                Referee_Update_PowerHeatData();
		    }}

        if(buffercnt == 11 && cmdID == 0x0203) {
            if (Verify_CRC16_Check_Sum(buffer, 11)) {
                Referee_Update_BuffMask();
            }
        }

        if(buffercnt == 10 && cmdID == 0x0206) {
            if (Verify_CRC16_Check_Sum(buffer, 10)) {
                Referee_Update_hurt();
            }
        }

        if(buffercnt == 16 && cmdID == 0x0207) {
            if (Verify_CRC16_Check_Sum(buffer, 16)) 
                Referee_Update_ShootData();
        }
        //@yyp
        if(buffercnt == 16 && cmdID == 0x0301) {
            if (buffer[7] == 0x03 && buffer[8] == 0x02) {
                Judge_Refresh_Interact();
            }
        }

    }
    else {
        if(tmp_judge == 0xA5) {
            receiving = 1;
            buffercnt = 0;
            buffer[0] = tmp_judge;
            buffercnt++;
        }
    }
//		HAL_UART_AbortReceive(&AUTOAIM_UART);
//	if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK){
//		Error_Handler();
//	}

    HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1);
}
uint8_t JUDGE_Received = 0;
JudgeState_e JUDGE_State = OFFLINE;
uint16_t maxHP = 1500;
uint16_t remainHP;
uint16_t maxHeat0 = 480;
uint16_t remainHeat0 = 480;
uint16_t maxHeat1 = 480;
uint16_t remainHeat1 = 480;
uint16_t RealHeat0 = 0;
float realBulletSpeed0 = 22;
float cooldown0 = 72;
uint8_t shoot0Cnt = 0;
uint8_t shoot1Cnt = 0;
uint8_t syncCnt0 = 0;


void getJudgeState(void) {
    static int s_count_judge = 0;
    if(JUDGE_Received==1) {
        s_count_judge = 0;
        JUDGE_State = ONLINE;
        JUDGE_Received = 0;
    }
    else {
        s_count_judge++;
        if(s_count_judge > 150)
        {   //300ms
            JUDGE_State = OFFLINE;
        }
    }
}

extShowData_t user_data;

void Send_User_Data() {
    uint8_t buffer[22]= {0};
    //test
    user_data.data1 += 0.1f;
    user_data.data2 += 0.2f;
    user_data.data3 += 0.3f;
    //user_data.mask = 0xE0;//二进制最左位对应灯条最右灯

    unsigned char * bs1 = (unsigned char*)&user_data.data1;
    unsigned char * bs2 = (unsigned char*)&user_data.data2;
    unsigned char * bs3 = (unsigned char*)&user_data.data3;
    buffer[0] = 0xA5;
    buffer[1] = 13;
    buffer[3] = 1;
    buffer[4] = Get_CRC8_Check_Sum(buffer, 5, CRC8_INIT);
    buffer[5] = 0x00;
    buffer[6] = 0x01;
    for(int i=7; i<11; i++) buffer[i] = bs1[i-7];
    for(int i=11; i<15; i++) buffer[i] = bs2[i-11];
    for(int i=15; i<19; i++) buffer[i] = bs3[i-15];
    buffer[19] = user_data.mask;
    static uint16_t CRC16=0;
    CRC16 = Get_CRC16_Check_Sum(buffer, 20, CRC_INIT);
    buffer[20] = CRC16 & 0x00ff;//0xff
    buffer[21] = (CRC16 >> 8) & 0xff;

    //tx_free = 0;
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&buffer,22)!=HAL_OK);
}

//****************************************************
//**********************RM2019************************
//****************************************************
#define getbit(x,y) ((x) >> (y)&1)//获取某一位的值

ext_game_robot_state_t GameRobotState;
ext_power_heat_data_t PowerHeat;
ext_buff_musk_t BuffMask;
ext_shoot_data_t ShootData;
ext_robot_hurt_t hurtData;

void Referee_Update_RobotState(void) {
//    static uint16_t lastHP=200,receiveCnt=0;

    unsigned char* grs0 = (unsigned char*)&GameRobotState.robot_id;
    char tmp0[1] = {buffer[7]};
    grs0[0] = (unsigned char)tmp0[0];

    unsigned char* grs1 = (unsigned char*)&GameRobotState.robot_level;
    char tmp1[1] = {buffer[8]};
    grs1[0] = (unsigned char)tmp1[0];

    unsigned char* grs2 = (unsigned char*)&GameRobotState.remain_HP;
    char tmp2[2] = {buffer[9],buffer[10]};
    for(int i = 0; i<2; i++) {
        grs2[i] = (unsigned char)tmp2[i];
    }
//    OLED_Printf(1,1,"%d",grs2[0]);
//    OLED_RefreshGram();
    unsigned char* grs3 = (unsigned char*)&GameRobotState.max_HP;
    char tmp3[2] = {buffer[11], buffer[12]};
    for(int i = 0; i<2; i++) {
        grs3[i] = (unsigned char)tmp3[i];
    }

    unsigned char* grs4 = (unsigned char*)&GameRobotState.shooter_id1_17mm_cooling_rate;
    char tmp4[2] = {buffer[13], buffer[14]};
    for(int i = 0; i<2; i++) {
        grs4[i] = (unsigned char)tmp4[i];
    }

    unsigned char* grs5 = (unsigned char*)&GameRobotState.shooter_id1_17mm_cooling_limit;
    char tmp5[2] = {buffer[15], buffer[16]};
    for(int i = 0; i<2; i++) {
        grs5[i] = (unsigned char)tmp5[i];
    }

    unsigned char* grs6 = (unsigned char*)&GameRobotState.shooter_id1_17mm_speed_limit;
    char tmp6[2] = {buffer[17], buffer[18]};
    for(int i = 0; i<2; i++) {
        grs6[i] = (unsigned char)tmp6[i];
    }

    unsigned char* grs7 = (unsigned char*)&GameRobotState.shooter_id2_17mm_cooling_rate;
    char tmp7[2] = {buffer[19], buffer[20]};
    for(int i = 0; i<2; i++) {
        grs7[i] = (unsigned char)tmp7[i];
    }
		
		unsigned char* grs8 = (unsigned char*)&GameRobotState.shooter_id2_17mm_cooling_limit;
    char tmp8[2] = {buffer[21], buffer[22]};
    for(int i = 0; i<2; i++) {
        grs8[i] = (unsigned char)tmp8[i];
    }
		unsigned char* grs9 = (unsigned char*)&GameRobotState.shooter_id2_17mm_speed_limit;
    char tmp9[2] = {buffer[23], buffer[24]};
    for(int i = 0; i<2; i++) {
        grs9[i] = (unsigned char)tmp9[i];
    }
		

    unsigned char* grs10 = (unsigned char*)&GameRobotState.shooter_id1_42mm_cooling_rate;
    char tmp10[2] = {buffer[25], buffer[26]};
    for(int i = 0; i<2; i++) {
        grs10[i] = (unsigned char)tmp10[i];
    }

    unsigned char* grs11 = (unsigned char*)&GameRobotState.shooter_id1_42mm_cooling_limit;
    char tmp11[2] = {buffer[27], buffer[28]};
    for(int i = 0; i<2; i++) {
        grs11[i] = (unsigned char)tmp11[i];
    }

    unsigned char* grs12 = (unsigned char*)&GameRobotState.shooter_id1_42mm_speed_limit;
    char tmp12[2] = {buffer[29], buffer[30]};
    for(int i = 0; i<2; i++) {
        grs12[i] = (unsigned char)tmp12[i];
    }	

		unsigned char* grs13 = (unsigned char*)&GameRobotState.chassis_power_limit;
    char tmp13[2] = {buffer[31],buffer[32]};
    for(int i = 0; i<2; i++) {
        grs13[i] = (unsigned char)tmp13[i];
    }
	

    maxHP = GameRobotState.max_HP;
//=========================================
    /*
    SJR:可能是被攻击的策略

    remainHP = GameRobotState.remain_HP;
    if(lastHP-remainHP){hurtTic=20;}
    if(hurtTic){
    	hurtSum+=lastHP-remainHP;
    	hurtTic--;
    }else{hurtSum=0;}

    if(hurtLck){
    	hurtLck--;
    	hurtSum=0;
    }
    if(hurtSum>30){
    	if(hurtInit){
    		if(CMA.Real>-443){//碉堡被打
    			CmdTic=70;hurtLck=30;
    			if(ExtCmd==1){hurtLck=80;}
    			else{hurtLck=30;}
    			ExtCmd=2;
    		}else{//飞机被打
    			ExtCmd=1;CmdTic=40;hurtLck=30;
    		}
    		hurtSum=0;
    	}else{
    		hurtInit=1;
    		hurtSum=0;
    	}
    }
    lastHP=remainHP;


    receiveCnt++;//10hz
    if(receiveCnt>10&&StateFlee>1){StateFlee=0;}
    if(CmdTic<120){CmdTic++;}
    else{ExtCmd=0;}
    */
//========================================

    switch(maxHP) {
    case 200: {
        maxHeat0 = 240;
        cooldown0 = 40;
    }
    break;
    case 250: {
        maxHeat0 = 360;
        cooldown0 = 60;
    }
    break;
    case 300: {
        maxHeat0 = 480;
        cooldown0 = 80;
    }
    break;
    default: {
        maxHeat0 = 480;
        cooldown0 = 80;
    }
    break;
    }
    JUDGE_Received = 1;
    //电源输出情况TBD

//	unsigned char* grs = (unsigned char*)&GameRobotState;
//	char tmp[15];
//	for(int i = 0; i<15; i++){
//		 //strcpy(tmp, buffer[i+7]);
////		sprintf(tmp, "%s", (char *)buffer[i+7]);
//	}
}
		
    
void Referee_Update_PowerHeatData() {
    //电压tbd
    //电流tbd
    unsigned char * ph10 = (unsigned char*)&PowerHeat.chassis_volt;
    for(int i = 0; i<2; i++) {
        ph10[i] = (unsigned char)buffer[i+7];
    }
    unsigned char * ph11 = (unsigned char*)&PowerHeat.chassis_current;
    for(int i = 0; i<2; i++) {
        ph11[i] = (unsigned char)buffer[i+9];
    }
		unsigned char * ph12 = (unsigned char*)&PowerHeat.chassis_power;
    for(int i = 0; i<4; i++) {
        ph12[i] = (unsigned char)buffer[i+11];
    }
    unsigned char * ph1 = (unsigned char*)&PowerHeat.chassis_power_buffer;
    for(int i = 0; i<2; i++) {
        ph1[i] = (unsigned char)buffer[i+15];
    }

//    unsigned char * ph2 = (unsigned char*)&PowerHeat.shooter_id1_17mm_cooling_heat;
//    for(int i = 0; i<2; i++) {
//        ph2[i] = (unsigned char)buffer[i+17];
//    }
   unsigned char * ph2 = (unsigned char*)&PowerHeat.shooter_id1_17mm_cooling_heat;
    for(int i = 0; i<2; i++) {
        ph2[i] = (unsigned char)buffer[i+17];
    }

    unsigned char * ph3 = (unsigned char*)&PowerHeat.shooter_id2_17mm_cooling_heat;
    for(int i = 0; i<2; i++) {
        ph3[i] = (unsigned char)buffer[i+19];
    }
		    unsigned char * ph4 = (unsigned char*)&PowerHeat.shooter_id1_42mm_cooling_heat;
    for(int i = 0; i<2; i++) {
        ph4[i] = (unsigned char)buffer[i+21];
    }
    RealHeat0 = PowerHeat.shooter_id1_17mm_cooling_heat;
    remainHeat0 = maxHeat0 - PowerHeat.shooter_id1_17mm_cooling_heat;
    remainHeat1 = maxHeat1 - PowerHeat.shooter_id1_42mm_cooling_heat;
    JUDGE_Received = 1;
}

void Referee_Update_BuffMask() {
    unsigned char * bm = (unsigned char*)&BuffMask.power_rune_buff;
    for(int i = 0; i<1; i++) {
        bm[i] = (unsigned char)buffer[i+7];//机器人增益
    }
    JUDGE_Received = 1;
}

void Referee_Update_ShootData() {
    unsigned char * sd0 = (unsigned char*)&ShootData.bullet_type;
    sd0[0] = (unsigned char)buffer[7];//

    unsigned char * sd1 = (unsigned char*)&ShootData.shooter_id;
    sd1[0] = (unsigned char)buffer[8];//
	
    unsigned char * sd2 = (unsigned char*)&ShootData.bullet_freq;
    sd2[0] = (unsigned char)buffer[9];//
	
    unsigned char * sd3 = (unsigned char*)&ShootData.bullet_speed;
    for(int i = 0; i<4; i++) {
        sd3[i] = (unsigned char)buffer[i+10];//
    }
    realBulletSpeed0 = ShootData.bullet_speed;
    JUDGE_Received = 1;
}

void Referee_Update_hurt() { //@yyp
//	uint8_t tmp=buffer[7]&0xf0;
////	if(tmp==0x0 && StateSway==0){StateFlee=2;StateCnt=0;}
//	if(tmp==0x0){StateFlee+=2;StateCnt=0;}
//	JUDGE_Received = 1;

    uint8_t tmp=buffer[7]&0x0f;
    if(tmp==0x0) {
        //StateHurt=2;
        hurtLck=30;
    }
}
//======================================
void Judge_Refresh_Interact() {
    //if(buffer[13]==1){CmdTic=0;ExtCmd=1;SpeedRef=2100;}//躲飞机
    //else if(buffer[13]==2){CmdTic=0;ExtCmd=2;}
}


client_custom_data_t custom_data;
void Referee_Transmit_UserData() {
    uint8_t buffer[28]= {0};
    //test
    custom_data.data1 += 0.001f;
    custom_data.data2 += 0.002f;
    custom_data.data3 += 0.003f;
    //user_data.mask = 0xE0;//二进制最左位对应灯条最右灯

    unsigned char * bs1 = (unsigned char*)&custom_data.data1;
    unsigned char * bs2 = (unsigned char*)&custom_data.data2;
    unsigned char * bs3 = (unsigned char*)&custom_data.data3;
    buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
    buffer[1] = 19;//数据帧中 data 的长度,占两个字节

    buffer[3] = 1;//包序号
    buffer[4] = Get_CRC8_Check_Sum(buffer, 5,CRC_INIT);//帧头 CRC8 校验
    buffer[5] = 0x03;
    buffer[6] = 0x01;

    buffer[7] = 0xD1;//数据的内容 ID:0xD180  ,占两个字节
    buffer[8] = 0x80;
    buffer[9] = 0x00;//发送者的 ID, 占两个字节
    buffer[10] = GameRobotState.robot_id;;//存疑
    buffer[11] = 0x01;//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
    buffer[12] = GameRobotState.robot_id;//注意哨兵没有这个ID
    for(int i=13; i<17; i++) buffer[i] = bs1[i-13];
    for(int i=17; i<21; i++) buffer[i] = bs2[i-17];
    for(int i=21; i<25; i++) buffer[i] = bs3[i-21];
    buffer[25] = custom_data.masks;
    static uint16_t CRC16=0;
    CRC16 = Get_CRC16_Check_Sum(buffer, 26,CRC_INIT);
    buffer[26] = CRC16 & 0x00ff;//0xff
    buffer[27] = (CRC16 >> 8) & 0xff;

    //tx_free = 0;
    while(HAL_UART_Transmit_IT(&JUDGE_UART,(uint8_t *)&buffer,28)!=HAL_OK);
}

void Referee_Transmit_graphic() {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
	  GraphicData.txFrameHeader.SOF = 0xA5;
	  GraphicData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_graphic_number_t);
	  GraphicData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &GraphicData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  GraphicData.CmdID = 0x0301;
	  GraphicData.dataFrameHeader.data_cmd_id = 0x0104;//内容id
	  GraphicData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  GraphicData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端
	
	  GraphicData.graphicData.graphic_data_struct[0].graphic_name[0]=31;
		GraphicData.graphicData.graphic_data_struct[0].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[0].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[0].layer=1;
		GraphicData.graphicData.graphic_data_struct[0].color=6;
		GraphicData.graphicData.graphic_data_struct[0].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].width=1;
		GraphicData.graphicData.graphic_data_struct[0].start_x=960;
		GraphicData.graphicData.graphic_data_struct[0].start_y=1080-635;
		GraphicData.graphicData.graphic_data_struct[0].radius=0;
		GraphicData.graphicData.graphic_data_struct[0].end_x=960;
		GraphicData.graphicData.graphic_data_struct[0].end_y=1080-840;

		
		GraphicData.graphicData.graphic_data_struct[2].graphic_name[0]=33	;
		GraphicData.graphicData.graphic_data_struct[2].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[2].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[2].layer=1;
		GraphicData.graphicData.graphic_data_struct[2].color=6;
		GraphicData.graphicData.graphic_data_struct[2].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[2].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[2].width=2;
		GraphicData.graphicData.graphic_data_struct[2].start_x=947;
		GraphicData.graphicData.graphic_data_struct[2].start_y=270;
		GraphicData.graphicData.graphic_data_struct[2].radius=0;
		GraphicData.graphicData.graphic_data_struct[2].end_x=973;
		GraphicData.graphicData.graphic_data_struct[2].end_y=270;
		
		GraphicData.graphicData.graphic_data_struct[1].graphic_name[0]=34	;
		GraphicData.graphicData.graphic_data_struct[1].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[1].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[1].layer=1;
		GraphicData.graphicData.graphic_data_struct[1].color=5;
		GraphicData.graphicData.graphic_data_struct[1].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].width=2;
		GraphicData.graphicData.graphic_data_struct[1].start_x=942;
		GraphicData.graphicData.graphic_data_struct[1].start_y=305;
		GraphicData.graphicData.graphic_data_struct[1].radius=0;
		GraphicData.graphicData.graphic_data_struct[1].end_x=978;
		GraphicData.graphicData.graphic_data_struct[1].end_y=305;

		GraphicData.graphicData.graphic_data_struct[3].graphic_name[0]=35	;
		GraphicData.graphicData.graphic_data_struct[3].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[3].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[3].layer=1;
		GraphicData.graphicData.graphic_data_struct[3].color=5;
		GraphicData.graphicData.graphic_data_struct[3].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[3].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[3].width=2;
		GraphicData.graphicData.graphic_data_struct[3].start_x=940;
		GraphicData.graphicData.graphic_data_struct[3].start_y=356;
		GraphicData.graphicData.graphic_data_struct[3].radius=0;
		GraphicData.graphicData.graphic_data_struct[3].end_x=980;
		GraphicData.graphicData.graphic_data_struct[3].end_y=356;
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&GraphicData.CmdID, 
			(sizeof(GraphicData.CmdID)+ sizeof(GraphicData.dataFrameHeader)+ sizeof(GraphicData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(GraphicData));//写入数据段CRC16校验码
	  lenth=sizeof(GraphicData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);	
		
//	HAL_Delay(500);
/* 
	客户端绘制五条线
*/			
}
void Referee_Transmit_graphic2(void) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
	  GraphicData.txFrameHeader.SOF = 0xA5;
	  GraphicData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_graphic_number_t);
	  GraphicData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &GraphicData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  GraphicData.CmdID = 0x0301;
	  GraphicData.dataFrameHeader.data_cmd_id = 0x0104;//内容id
	  GraphicData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  GraphicData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端

	  GraphicData.graphicData.graphic_data_struct[0].graphic_name[0]=31;
		GraphicData.graphicData.graphic_data_struct[0].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[0].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[0].layer=2;
		GraphicData.graphicData.graphic_data_struct[0].color=5;
		GraphicData.graphicData.graphic_data_struct[0].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].width=1;
		GraphicData.graphicData.graphic_data_struct[0].start_x=897;
		GraphicData.graphicData.graphic_data_struct[0].start_y=447;
		GraphicData.graphicData.graphic_data_struct[0].radius=0;
		GraphicData.graphicData.graphic_data_struct[0].end_x=1022;
		GraphicData.graphicData.graphic_data_struct[0].end_y=447;

		
		GraphicData.graphicData.graphic_data_struct[2].graphic_name[0]=33	;
		GraphicData.graphicData.graphic_data_struct[2].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[2].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[2].layer=2;
		GraphicData.graphicData.graphic_data_struct[2].color=5;
		GraphicData.graphicData.graphic_data_struct[2].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[2].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[2].width=1;
		GraphicData.graphicData.graphic_data_struct[2].start_x=933;//
		GraphicData.graphicData.graphic_data_struct[2].start_y=440;
		GraphicData.graphicData.graphic_data_struct[2].radius=0;
		GraphicData.graphicData.graphic_data_struct[2].end_x=987;
		GraphicData.graphicData.graphic_data_struct[2].end_y=440;
		
//		GraphicData.graphicData.graphic_data_struct[1].graphic_name[0]=34	;
//		GraphicData.graphicData.graphic_data_struct[1].operate_type=1;
//	  GraphicData.graphicData.graphic_data_struct[1].graphic_type=0;
//		GraphicData.graphicData.graphic_data_struct[1].layer=1;
//		GraphicData.graphicData.graphic_data_struct[1].color=5;
//		GraphicData.graphicData.graphic_data_struct[1].start_angle=0;
//		GraphicData.graphicData.graphic_data_struct[1].end_angle=0;
//		GraphicData.graphicData.graphic_data_struct[1].width=2;
//		GraphicData.graphicData.graphic_data_struct[1].start_x=960;
//		GraphicData.graphicData.graphic_data_struct[1].start_y=540-120;
//		GraphicData.graphicData.graphic_data_struct[1].radius=0;
//		GraphicData.graphicData.graphic_data_struct[1].end_x=960+17;//
//		GraphicData.graphicData.graphic_data_struct[1].end_y=540-120;

		GraphicData.graphicData.graphic_data_struct[1].graphic_name[0]=34	;
		GraphicData.graphicData.graphic_data_struct[1].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[1].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[1].layer=2;
		GraphicData.graphicData.graphic_data_struct[1].color=5;
		GraphicData.graphicData.graphic_data_struct[1].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].width=2;
		GraphicData.graphicData.graphic_data_struct[1].start_x=943;
		GraphicData.graphicData.graphic_data_struct[1].start_y=424;
		GraphicData.graphicData.graphic_data_struct[1].radius=0;
		GraphicData.graphicData.graphic_data_struct[1].end_x=977;//
		GraphicData.graphicData.graphic_data_struct[1].end_y=424;
		
		GraphicData.graphicData.graphic_data_struct[3].graphic_name[0]=32	;
		GraphicData.graphicData.graphic_data_struct[3].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[3].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[3].layer=2;
		GraphicData.graphicData.graphic_data_struct[3].color=5;
		GraphicData.graphicData.graphic_data_struct[3].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[3].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[3].width=1;
		GraphicData.graphicData.graphic_data_struct[3].start_x=947;
		GraphicData.graphicData.graphic_data_struct[3].start_y=413;
		GraphicData.graphicData.graphic_data_struct[3].radius=0;
		GraphicData.graphicData.graphic_data_struct[3].end_x=973;//
		GraphicData.graphicData.graphic_data_struct[3].end_y=413;
		
		GraphicData.graphicData.graphic_data_struct[4].graphic_name[0]=35	;
		GraphicData.graphicData.graphic_data_struct[4].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[4].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[4].layer=2;
		GraphicData.graphicData.graphic_data_struct[4].color=5;
		GraphicData.graphicData.graphic_data_struct[4].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[4].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[4].width=1;
		GraphicData.graphicData.graphic_data_struct[4].start_x=950;
		GraphicData.graphicData.graphic_data_struct[4].start_y=386;
		GraphicData.graphicData.graphic_data_struct[4].radius=0;
		GraphicData.graphicData.graphic_data_struct[4].end_x=970;
		GraphicData.graphicData.graphic_data_struct[4].end_y=386;
	
		GraphicData.graphicData.graphic_data_struct[5].graphic_name[0]=36	;
		GraphicData.graphicData.graphic_data_struct[5].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[5].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[5].layer=2;
		GraphicData.graphicData.graphic_data_struct[5].color=5;
		GraphicData.graphicData.graphic_data_struct[5].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[5].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[5].width=1;
		GraphicData.graphicData.graphic_data_struct[5].start_x=951;
		GraphicData.graphicData.graphic_data_struct[5].start_y=358;
		GraphicData.graphicData.graphic_data_struct[5].radius=0;
		GraphicData.graphicData.graphic_data_struct[5].end_x=968;
		GraphicData.graphicData.graphic_data_struct[5].end_y=358;
		
		GraphicData.graphicData.graphic_data_struct[6].graphic_name[0]=37	;
		GraphicData.graphicData.graphic_data_struct[6].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[6].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[6].layer=2;
		GraphicData.graphicData.graphic_data_struct[6].color=5;
		GraphicData.graphicData.graphic_data_struct[6].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[6].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[6].width=1;
		GraphicData.graphicData.graphic_data_struct[6].start_x=953;
		GraphicData.graphicData.graphic_data_struct[6].start_y=330;
		GraphicData.graphicData.graphic_data_struct[6].radius=0;
		GraphicData.graphicData.graphic_data_struct[6].end_x=966;
		GraphicData.graphicData.graphic_data_struct[6].end_y=330;
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&GraphicData.CmdID, 
			(sizeof(GraphicData.CmdID)+ sizeof(GraphicData.dataFrameHeader)+ sizeof(GraphicData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(GraphicData));//写入数据段CRC16校验码
	  lenth=sizeof(GraphicData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);	
		
//	HAL_Delay(500);
/* 
	客户端绘制五条线
*/			
		}
void Referee_Transmit_graphic3() {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
	  GraphicData.txFrameHeader.SOF = 0xA5;
	  GraphicData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_graphic_number_t);
	  GraphicData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &GraphicData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  GraphicData.CmdID = 0x0301;
	  GraphicData.dataFrameHeader.data_cmd_id = 0x0104;//内容id
	  GraphicData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  GraphicData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端

	  GraphicData.graphicData.graphic_data_struct[0].graphic_name[0]=31;
		GraphicData.graphicData.graphic_data_struct[0].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[0].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[0].layer=3;
		GraphicData.graphicData.graphic_data_struct[0].color=6;
		GraphicData.graphicData.graphic_data_struct[0].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].width=1;
		GraphicData.graphicData.graphic_data_struct[0].start_x=960;
		GraphicData.graphicData.graphic_data_struct[0].start_y=540;
		GraphicData.graphicData.graphic_data_struct[0].radius=0;
		GraphicData.graphicData.graphic_data_struct[0].end_x=960;
		GraphicData.graphicData.graphic_data_struct[0].end_y=540-300;

		
//		GraphicData.graphicData.graphic_data_struct[2].graphic_name[0]=33	;
//		GraphicData.graphicData.graphic_data_struct[2].operate_type=1;
//	  GraphicData.graphicData.graphic_data_struct[2].graphic_type=0;
//		GraphicData.graphicData.graphic_data_struct[2].layer=2;
//		GraphicData.graphicData.graphic_data_struct[2].color=6;
//		GraphicData.graphicData.graphic_data_struct[2].start_angle=0;
//		GraphicData.graphicData.graphic_data_struct[2].end_angle=0;
//		GraphicData.graphicData.graphic_data_struct[2].width=1;
//		GraphicData.graphicData.graphic_data_struct[2].start_x=960+40;//
//		GraphicData.graphicData.graphic_data_struct[2].start_y=540-113;
//		GraphicData.graphicData.graphic_data_struct[2].radius=0;
//		GraphicData.graphicData.graphic_data_struct[2].end_x=960-40;
//		GraphicData.graphicData.graphic_data_struct[2].end_y=540-113;
//		
//		GraphicData.graphicData.graphic_data_struct[1].graphic_name[0]=34	;
//		GraphicData.graphicData.graphic_data_struct[1].operate_type=1;
//	  GraphicData.graphicData.graphic_data_struct[1].graphic_type=0;
//		GraphicData.graphicData.graphic_data_struct[1].layer=2;
//		GraphicData.graphicData.graphic_data_struct[1].color=6;
//		GraphicData.graphicData.graphic_data_struct[1].start_angle=0;
//		GraphicData.graphicData.graphic_data_struct[1].end_angle=0;
//		GraphicData.graphicData.graphic_data_struct[1].width=1;
//		GraphicData.graphicData.graphic_data_struct[1].start_x=960+28;
//		GraphicData.graphicData.graphic_data_struct[1].start_y=540-138;
//		GraphicData.graphicData.graphic_data_struct[1].radius=0;
//		GraphicData.graphicData.graphic_data_struct[1].end_x=960-28;//
//		GraphicData.graphicData.graphic_data_struct[1].end_y=540-138;
//		
//		GraphicData.graphicData.graphic_data_struct[3].graphic_name[0]=32	;
//		GraphicData.graphicData.graphic_data_struct[3].operate_type=1;
//	  GraphicData.graphicData.graphic_data_struct[3].graphic_type=0;
//		GraphicData.graphicData.graphic_data_struct[3].layer=2;
//		GraphicData.graphicData.graphic_data_struct[3].color=6;
//		GraphicData.graphicData.graphic_data_struct[3].start_angle=0;
//		GraphicData.graphicData.graphic_data_struct[3].end_angle=0;
//		GraphicData.graphicData.graphic_data_struct[3].width=1;
//		GraphicData.graphicData.graphic_data_struct[3].start_x=960+23;
//		GraphicData.graphicData.graphic_data_struct[3].start_y=540-147;
//		GraphicData.graphicData.graphic_data_struct[3].radius=0;
//		GraphicData.graphicData.graphic_data_struct[3].end_x=960-23;//
//		GraphicData.graphicData.graphic_data_struct[3].end_y=540-147;
//		
//		GraphicData.graphicData.graphic_data_struct[4].graphic_name[0]=35	;
//		GraphicData.graphicData.graphic_data_struct[4].operate_type=1;
//	  GraphicData.graphicData.graphic_data_struct[4].graphic_type=0;
//		GraphicData.graphicData.graphic_data_struct[4].layer=2;
//		GraphicData.graphicData.graphic_data_struct[4].color=6;
//		GraphicData.graphicData.graphic_data_struct[4].start_angle=0;
//		GraphicData.graphicData.graphic_data_struct[4].end_angle=0;
//		GraphicData.graphicData.graphic_data_struct[4].width=1;
//		GraphicData.graphicData.graphic_data_struct[4].start_x=960+17;
//		GraphicData.graphicData.graphic_data_struct[4].start_y=540-176;
//		GraphicData.graphicData.graphic_data_struct[4].radius=0;
//		GraphicData.graphicData.graphic_data_struct[4].end_x=960-17;
//		GraphicData.graphicData.graphic_data_struct[4].end_y=540-176;
//	
//		GraphicData.graphicData.graphic_data_struct[5].graphic_name[0]=36	;
//		GraphicData.graphicData.graphic_data_struct[5].operate_type=1;
//	  GraphicData.graphicData.graphic_data_struct[5].graphic_type=0;
//		GraphicData.graphicData.graphic_data_struct[5].layer=2;
//		GraphicData.graphicData.graphic_data_struct[5].color=6;
//		GraphicData.graphicData.graphic_data_struct[5].start_angle=0;
//		GraphicData.graphicData.graphic_data_struct[5].end_angle=0;
//		GraphicData.graphicData.graphic_data_struct[5].width=1;
//		GraphicData.graphicData.graphic_data_struct[5].start_x=960+15;
//		GraphicData.graphicData.graphic_data_struct[5].start_y=540-197;
//		GraphicData.graphicData.graphic_data_struct[5].radius=0;
//		GraphicData.graphicData.graphic_data_struct[5].end_x=960-15;
//		GraphicData.graphicData.graphic_data_struct[5].end_y=540-197;
//		
//		GraphicData.graphicData.graphic_data_struct[6].graphic_name[0]=37	;
//		GraphicData.graphicData.graphic_data_struct[6].operate_type=1;
//	  GraphicData.graphicData.graphic_data_struct[6].graphic_type=0;
//		GraphicData.graphicData.graphic_data_struct[6].layer=2;
//		GraphicData.graphicData.graphic_data_struct[6].color=6;
//		GraphicData.graphicData.graphic_data_struct[6].start_angle=0;
//		GraphicData.graphicData.graphic_data_struct[6].end_angle=0;
//		GraphicData.graphicData.graphic_data_struct[6].width=1;
//		GraphicData.graphicData.graphic_data_struct[6].start_x=960+12;
//		GraphicData.graphicData.graphic_data_struct[6].start_y=540-235;
//		GraphicData.graphicData.graphic_data_struct[6].radius=0;
//		GraphicData.graphicData.graphic_data_struct[6].end_x=960-12;
//		GraphicData.graphicData.graphic_data_struct[6].end_y=540-235;
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&GraphicData.CmdID, 
			(sizeof(GraphicData.CmdID)+ sizeof(GraphicData.dataFrameHeader)+ sizeof(GraphicData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(GraphicData));//写入数据段CRC16校验码
	  lenth=sizeof(GraphicData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);	
		
		}
void Referee_Transmit_graphic_2(int a) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
	  memset(&GraphicData,0,sizeof(ext_SendClientData_graphic_t));
	  GraphicData.txFrameHeader.SOF = 0xA5;
	  GraphicData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_graphic_number_t);
	  GraphicData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &GraphicData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  GraphicData.CmdID = 0x0301;
	  GraphicData.dataFrameHeader.data_cmd_id = 0x0104;//内容id
	  GraphicData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  GraphicData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端
	  //电容矩形
		//GraphicData.graphicData.graphic_data_struct[0].graphic_name[0]=31;
		//GraphicData.graphicData.graphic_data_struct[0].operate_type=1;
	  //GraphicData.graphicData.graphic_data_struct[0].graphic_type=1;
		//GraphicData.graphicData.graphic_data_struct[0].layer=1;
		//GraphicData.graphicData.graphic_data_struct[0].color=0;
		//GraphicData.graphicData.graphic_data_struct[0].start_angle=0;
		//GraphicData.graphicData.graphic_data_struct[0].end_angle=0;
		//GraphicData.graphicData.graphic_data_struct[0].width=2;
		//GraphicData.graphicData.graphic_data_struct[0].start_x=340;
		//GraphicData.graphicData.graphic_data_struct[0].start_y=1080-260;
		//GraphicData.graphicData.graphic_data_struct[0].radius=0;
		//GraphicData.graphicData.graphic_data_struct[0].end_x=1580;
		//GraphicData.graphicData.graphic_data_struct[0].end_y=1080-380;

		//黑整圆外
		//GraphicData.graphicData.graphic_data_struct[2].graphic_name[0]=33	;
		//GraphicData.graphicData.graphic_data_struct[2].operate_type=1;
	  //GraphicData.graphicData.graphic_data_struct[2].graphic_type=2;
		//GraphicData.graphicData.graphic_data_struct[2].layer=2;
		//GraphicData.graphicData.graphic_data_struct[2].color=7;
		//GraphicData.graphicData.graphic_data_struct[2].start_angle=0;
		//GraphicData.graphicData.graphic_data_struct[2].end_angle=0;
		//GraphicData.graphicData.graphic_data_struct[2].width=2;
		//GraphicData.graphicData.graphic_data_struct[2].start_x=1600;
		//GraphicData.graphicData.graphic_data_struct[2].start_y=1080-400;
		//GraphicData.graphicData.graphic_data_struct[2].radius=60;
		//GraphicData.graphicData.graphic_data_struct[2].end_x=0;
		//GraphicData.graphicData.graphic_data_struct[2].end_y=0;
		//黑整圆内
		//GraphicData.graphicData.graphic_data_struct[1].graphic_name[0]=32	;
		//GraphicData.graphicData.graphic_data_struct[1].operate_type=1;
	  //GraphicData.graphicData.graphic_data_struct[1].graphic_type=2;
		//GraphicData.graphicData.graphic_data_struct[1].layer=2;
		//GraphicData.graphicData.graphic_data_struct[1].color=7;
		//GraphicData.graphicData.graphic_data_struct[1].start_angle=0;
		//GraphicData.graphicData.graphic_data_struct[1].end_angle=0;
		//GraphicData.graphicData.graphic_data_struct[1].width=2;
		//GraphicData.graphicData.graphic_data_struct[1].start_x=1600;
		//GraphicData.graphicData.graphic_data_struct[1].start_y=1080-400;
		//GraphicData.graphicData.graphic_data_struct[1].radius=55;
		//GraphicData.graphicData.graphic_data_struct[1].end_x=0;
		//GraphicData.graphicData.graphic_data_struct[1].end_y=0;
		//圆弧
		GraphicData.graphicData.graphic_data_struct[3].graphic_name[0]=70	;
		GraphicData.graphicData.graphic_data_struct[3].operate_type=a;
	  GraphicData.graphicData.graphic_data_struct[3].graphic_type=4;
		GraphicData.graphicData.graphic_data_struct[3].layer=3;
		GraphicData.graphicData.graphic_data_struct[3].color=2;
		GraphicData.graphicData.graphic_data_struct[3].start_angle=360-client_state_now.capacity_percent*3.6;
		GraphicData.graphicData.graphic_data_struct[3].end_angle=360;
		GraphicData.graphicData.graphic_data_struct[3].width=3;
		GraphicData.graphicData.graphic_data_struct[3].start_x=570;
		GraphicData.graphicData.graphic_data_struct[3].start_y=1080-360;
		GraphicData.graphicData.graphic_data_struct[3].radius=0;
		GraphicData.graphicData.graphic_data_struct[3].end_x=40;
		GraphicData.graphicData.graphic_data_struct[3].end_y=40;
		
		//GraphicData.graphicData.graphic_data_struct[4].graphic_name[0]=35	;
		//GraphicData.graphicData.graphic_data_struct[4].operate_type=1;
	  //GraphicData.graphicData.graphic_data_struct[4].graphic_type=0;
		//GraphicData.graphicData.graphic_data_struct[4].layer=2;
		//GraphicData.graphicData.graphic_data_struct[4].color=6;
		//GraphicData.graphicData.graphic_data_struct[4].start_angle=0;
		//GraphicData.graphicData.graphic_data_struct[4].end_angle=0;
		//GraphicData.graphicData.graphic_data_struct[4].width=2;
		//GraphicData.graphicData.graphic_data_struct[4].start_x=895;
		//GraphicData.graphicData.graphic_data_struct[4].start_y=420;
		//GraphicData.graphicData.graphic_data_struct[4].radius=0;
		//GraphicData.graphicData.graphic_data_struct[4].end_x=1920-895;
		//GraphicData.graphicData.graphic_data_struct[4].end_y=420;




	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&GraphicData.CmdID, 
			(sizeof(GraphicData.CmdID)+ sizeof(GraphicData.dataFrameHeader)+ sizeof(GraphicData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(GraphicData));//写入数据段CRC16校验码
	  lenth=sizeof(GraphicData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);	
		}
int itoa(int num, char *dest){
    if(dest == NULL)
        return -1; 
 
    char temp[24];
    temp[23] = '\0';
    char *p = &temp[22];
    while(num/10 != 0){ 
        *(p--) = num%10 + 48; 
        num = num /10;
    }   
    *p = num%10 + 48; 
    strcpy(dest, p); 
    return 0;
}
void Referee_Transmit_character(uint8_t flag_character,char s[]) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
	  memset(CharacterData.graphicData.data,0,20);
		CharacterData.txFrameHeader.SOF = 0xA5;
		CharacterData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_character_t);
		CharacterData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &CharacterData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  CharacterData.CmdID = 0x0301;
	  CharacterData.dataFrameHeader.data_cmd_id = 0x0110;//内容id
	  CharacterData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  CharacterData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端
	  CharacterData.graphicData.graphic_data_struct.graphic_name[0]=51;
		CharacterData.graphicData.graphic_data_struct.operate_type=flag_character;
	  CharacterData.graphicData.graphic_data_struct.graphic_type=7;
		CharacterData.graphicData.graphic_data_struct.layer=0;
		CharacterData.graphicData.graphic_data_struct.color=5;
		CharacterData.graphicData.graphic_data_struct.start_angle=40;
		CharacterData.graphicData.graphic_data_struct.end_angle=2;
		CharacterData.graphicData.graphic_data_struct.width=5;
		CharacterData.graphicData.graphic_data_struct.start_x=460;
		CharacterData.graphicData.graphic_data_struct.start_y=1080-270;
    strcpy(CharacterData.graphicData.data,s);
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&CharacterData.CmdID, 
			(sizeof(CharacterData.CmdID)+ sizeof(CharacterData.dataFrameHeader)+ sizeof(CharacterData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(CharacterData));//写入数据段CRC16校验码
	  lenth=sizeof(CharacterData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);
			
}

void Referee_Transmit_character_top(uint8_t flag_character,char s[]) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
		memset(CharacterData.graphicData.data,0,20);
		CharacterData.txFrameHeader.SOF = 0xA5;
		CharacterData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_character_t);
		CharacterData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &CharacterData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  CharacterData.CmdID = 0x0301;
	  CharacterData.dataFrameHeader.data_cmd_id = 0x0110;//内容id
	  CharacterData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  CharacterData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端

	  CharacterData.graphicData.graphic_data_struct.graphic_name[0]=51;
		CharacterData.graphicData.graphic_data_struct.operate_type=flag_character;
	  CharacterData.graphicData.graphic_data_struct.graphic_type=7;
		CharacterData.graphicData.graphic_data_struct.layer=2;
		CharacterData.graphicData.graphic_data_struct.color=1;
		CharacterData.graphicData.graphic_data_struct.start_angle=20;
		CharacterData.graphicData.graphic_data_struct.end_angle=1;
		CharacterData.graphicData.graphic_data_struct.width=3;
		CharacterData.graphicData.graphic_data_struct.start_x=90;
		CharacterData.graphicData.graphic_data_struct.start_y=1080-260;
		
    strcpy(CharacterData.graphicData.data,s);
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&CharacterData.CmdID, 
			(sizeof(CharacterData.CmdID)+ sizeof(CharacterData.dataFrameHeader)+ sizeof(CharacterData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(CharacterData));//写入数据段CRC16校验码
	  lenth=sizeof(CharacterData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);
			
}

void Referee_Transmit_character_wheel(uint8_t flag_character,char s[]) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
		CharacterData.txFrameHeader.SOF = 0xA5;
		CharacterData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_character_t);
		CharacterData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &CharacterData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  CharacterData.CmdID = 0x0301;
	  CharacterData.dataFrameHeader.data_cmd_id = 0x0110;//内容id
	  CharacterData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  CharacterData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端

	  CharacterData.graphicData.graphic_data_struct.graphic_name[0]=52;
		CharacterData.graphicData.graphic_data_struct.operate_type=flag_character;
	  CharacterData.graphicData.graphic_data_struct.graphic_type=7;
		CharacterData.graphicData.graphic_data_struct.layer=2;
		CharacterData.graphicData.graphic_data_struct.color=2;
		CharacterData.graphicData.graphic_data_struct.start_angle=20;
		CharacterData.graphicData.graphic_data_struct.end_angle=1;
		CharacterData.graphicData.graphic_data_struct.width=3;
		CharacterData.graphicData.graphic_data_struct.start_x=90;
		CharacterData.graphicData.graphic_data_struct.start_y=1080-300;
    strcpy(CharacterData.graphicData.data,s);
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&CharacterData.CmdID, 
			(sizeof(CharacterData.CmdID)+ sizeof(CharacterData.dataFrameHeader)+ sizeof(CharacterData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(CharacterData));//写入数据段CRC16校验码
	  lenth=sizeof(CharacterData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);
			
}

void Referee_Transmit_character_AiMBot(uint8_t flag_character,char s[]) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
		memset(CharacterData.graphicData.data,0,20);
		CharacterData.txFrameHeader.SOF = 0xA5;
		CharacterData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_character_t);
		CharacterData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &CharacterData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  CharacterData.CmdID = 0x0301;
	  CharacterData.dataFrameHeader.data_cmd_id = 0x0110;//内容id
	  CharacterData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  CharacterData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端

	  CharacterData.graphicData.graphic_data_struct.graphic_name[0]=53;
		CharacterData.graphicData.graphic_data_struct.operate_type=flag_character;
	  CharacterData.graphicData.graphic_data_struct.graphic_type=7;
		CharacterData.graphicData.graphic_data_struct.layer=2;
		CharacterData.graphicData.graphic_data_struct.color=3;
		CharacterData.graphicData.graphic_data_struct.start_angle=20;
		CharacterData.graphicData.graphic_data_struct.end_angle=1;
		CharacterData.graphicData.graphic_data_struct.width=3;
		CharacterData.graphicData.graphic_data_struct.start_x=90;
		CharacterData.graphicData.graphic_data_struct.start_y=1080-340;
    strcpy(CharacterData.graphicData.data,s);
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&CharacterData.CmdID, 
			(sizeof(CharacterData.CmdID)+ sizeof(CharacterData.dataFrameHeader)+ sizeof(CharacterData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(CharacterData));//写入数据段CRC16校验码
	  lenth=sizeof(CharacterData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);
			
}

void Referee_Transmit_character_bin_cover(uint8_t flag_character,char s[]) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
		CharacterData.txFrameHeader.SOF = 0xA5;
		CharacterData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_character_t);
		CharacterData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &CharacterData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  CharacterData.CmdID = 0x0301;
	  CharacterData.dataFrameHeader.data_cmd_id = 0x0110;//内容id
	  CharacterData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  CharacterData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端

	  CharacterData.graphicData.graphic_data_struct.graphic_name[0]=54;
		CharacterData.graphicData.graphic_data_struct.operate_type=flag_character;
	  CharacterData.graphicData.graphic_data_struct.graphic_type=7;
		CharacterData.graphicData.graphic_data_struct.layer=2;
		CharacterData.graphicData.graphic_data_struct.color=4;
		CharacterData.graphicData.graphic_data_struct.start_angle=20;
		CharacterData.graphicData.graphic_data_struct.end_angle=1;
		CharacterData.graphicData.graphic_data_struct.width=3;
		CharacterData.graphicData.graphic_data_struct.start_x=90;
		CharacterData.graphicData.graphic_data_struct.start_y=1080-380;
    strcpy(CharacterData.graphicData.data,s);
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&CharacterData.CmdID, 
			(sizeof(CharacterData.CmdID)+ sizeof(CharacterData.dataFrameHeader)+ sizeof(CharacterData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(CharacterData));//写入数据段CRC16校验码
	  lenth=sizeof(CharacterData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);
			
}

void Referee_Transmit_character_rate(uint8_t flag_character,char s[]) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
		memset(CharacterData.graphicData.data,0,20);
		CharacterData.txFrameHeader.SOF = 0xA5;
		CharacterData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_character_t);
		CharacterData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &CharacterData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  CharacterData.CmdID = 0x0301;
	  CharacterData.dataFrameHeader.data_cmd_id = 0x0110;//内容id
	  CharacterData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  CharacterData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端

	  CharacterData.graphicData.graphic_data_struct.graphic_name[0]=55;
		CharacterData.graphicData.graphic_data_struct.operate_type=flag_character;
	  CharacterData.graphicData.graphic_data_struct.graphic_type=7;
		CharacterData.graphicData.graphic_data_struct.layer=2;
		CharacterData.graphicData.graphic_data_struct.color=5;
		CharacterData.graphicData.graphic_data_struct.start_angle=20;
		CharacterData.graphicData.graphic_data_struct.end_angle=1;
		CharacterData.graphicData.graphic_data_struct.width=3;
		CharacterData.graphicData.graphic_data_struct.start_x=90;
		CharacterData.graphicData.graphic_data_struct.start_y=1080-420;
    strcpy(CharacterData.graphicData.data,s);
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&CharacterData.CmdID, 
			(sizeof(CharacterData.CmdID)+ sizeof(CharacterData.dataFrameHeader)+ sizeof(CharacterData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(CharacterData));//写入数据段CRC16校验码
	  lenth=sizeof(CharacterData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);
			
}

void Referee_Transmit_character_present(uint8_t flag_character,char s[]) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
		memset(CharacterData.graphicData.data,0,20);
		CharacterData.txFrameHeader.SOF = 0xA5;
		CharacterData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_character_t);
		CharacterData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &CharacterData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  CharacterData.CmdID = 0x0301;
	  CharacterData.dataFrameHeader.data_cmd_id = 0x0110;//内容id
	  CharacterData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  CharacterData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端

	  CharacterData.graphicData.graphic_data_struct.graphic_name[0]=90;
		CharacterData.graphicData.graphic_data_struct.operate_type=flag_character;
	  CharacterData.graphicData.graphic_data_struct.graphic_type=7;
		CharacterData.graphicData.graphic_data_struct.layer=2;
		CharacterData.graphicData.graphic_data_struct.color=2;
		CharacterData.graphicData.graphic_data_struct.start_angle=15;
		CharacterData.graphicData.graphic_data_struct.end_angle=3;
		CharacterData.graphicData.graphic_data_struct.width=3;
		CharacterData.graphicData.graphic_data_struct.start_x=560+800*client_state_now.capacity_percent/100.0;
		CharacterData.graphicData.graphic_data_struct.start_y=1080-275;
    strcpy(CharacterData.graphicData.data,s);
	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&CharacterData.CmdID, 
			(sizeof(CharacterData.CmdID)+ sizeof(CharacterData.dataFrameHeader)+ sizeof(CharacterData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(CharacterData));//写入数据段CRC16校验码
	  lenth=sizeof(CharacterData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);
			
}

void Referee_Transmit_graphic_debug() {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
	  memset(&GraphicData, 0, sizeof(ext_SendClientData_graphic_t));
	  GraphicData.txFrameHeader.SOF = 0xA5;
	  GraphicData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_graphic_number_t);
	  GraphicData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &GraphicData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  GraphicData.CmdID = 0x0301;
	  GraphicData.dataFrameHeader.data_cmd_id = 0x0102;//内容id
	  GraphicData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  GraphicData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端
	
	  GraphicData.graphicData.graphic_data_struct[0].graphic_name[0]=41;
		GraphicData.graphicData.graphic_data_struct[0].operate_type=2;
	  GraphicData.graphicData.graphic_data_struct[0].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[0].layer=2;
		GraphicData.graphicData.graphic_data_struct[0].color=6;
		GraphicData.graphicData.graphic_data_struct[0].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].width=50;
		GraphicData.graphicData.graphic_data_struct[0].start_x=460;
		GraphicData.graphicData.graphic_data_struct[0].start_y=1080-230;
		GraphicData.graphicData.graphic_data_struct[0].radius=0;
		GraphicData.graphicData.graphic_data_struct[0].end_x=1920-460+client_state_now.capacity_percent-1000;
		GraphicData.graphicData.graphic_data_struct[0].end_y=1080-230;
		
	  GraphicData.graphicData.graphic_data_struct[1].graphic_name[0]=42;
		GraphicData.graphicData.graphic_data_struct[1].operate_type=2;
	  GraphicData.graphicData.graphic_data_struct[1].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[1].layer=3;
		GraphicData.graphicData.graphic_data_struct[1].color=7;
		GraphicData.graphicData.graphic_data_struct[1].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].width=60;
		GraphicData.graphicData.graphic_data_struct[1].start_x=1920-460-1000+600;
		GraphicData.graphicData.graphic_data_struct[1].start_y=1080-230;
		GraphicData.graphicData.graphic_data_struct[1].radius=0;
		GraphicData.graphicData.graphic_data_struct[1].end_x=1920-460-1000+610;
		GraphicData.graphicData.graphic_data_struct[1].end_y=1080-230;

	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&GraphicData.CmdID, 
			(sizeof(GraphicData.CmdID)+ sizeof(GraphicData.dataFrameHeader)+ sizeof(GraphicData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(GraphicData));//写入数据段CRC16校验码
	  lenth=sizeof(GraphicData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);	
}

void Referee_Transmit_graphic_Precent(uint8_t type) {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
	  memset(&GraphicData, 0, sizeof(ext_SendClientData_graphic_t));
	  GraphicData.txFrameHeader.SOF = 0xA5;
	  GraphicData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_graphic_number_t);
	  GraphicData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &GraphicData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  GraphicData.CmdID = 0x0301;
	  GraphicData.dataFrameHeader.data_cmd_id = 0x0102;//内容id
	  GraphicData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  GraphicData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端
		//粗直线
	  GraphicData.graphicData.graphic_data_struct[0].graphic_name[0]=45;
		GraphicData.graphicData.graphic_data_struct[0].operate_type=type;
	  GraphicData.graphicData.graphic_data_struct[0].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[0].layer=2;
		GraphicData.graphicData.graphic_data_struct[0].color=2;
		GraphicData.graphicData.graphic_data_struct[0].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].width=50;
		GraphicData.graphicData.graphic_data_struct[0].start_x=560;
		GraphicData.graphicData.graphic_data_struct[0].start_y=1080-230;
		GraphicData.graphicData.graphic_data_struct[0].radius=0;
		GraphicData.graphicData.graphic_data_struct[0].end_x=560+800*client_state_now.capacity_percent/100.0;
		GraphicData.graphicData.graphic_data_struct[0].end_y=1080-230;
		
		GraphicData.graphicData.graphic_data_struct[1].graphic_name[0]=42;
		GraphicData.graphicData.graphic_data_struct[1].operate_type=2;
	  GraphicData.graphicData.graphic_data_struct[1].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[1].layer=1;
		GraphicData.graphicData.graphic_data_struct[1].color=7;
		GraphicData.graphicData.graphic_data_struct[1].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].width=60;
		GraphicData.graphicData.graphic_data_struct[1].start_x=560+264;
		GraphicData.graphicData.graphic_data_struct[1].start_y=1080-230;
		GraphicData.graphicData.graphic_data_struct[1].radius=0;
		GraphicData.graphicData.graphic_data_struct[1].end_x=560+274;
		GraphicData.graphicData.graphic_data_struct[1].end_y=1080-230;

	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&GraphicData.CmdID, 
			(sizeof(GraphicData.CmdID)+ sizeof(GraphicData.dataFrameHeader)+ sizeof(GraphicData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(GraphicData));//写入数据段CRC16校验码
	  lenth=sizeof(GraphicData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);	
}

void Referee_Transmit_graphic_Init() {
	  uint16_t datalength;
	  memset(CliendTxBuffer,0,128);
	  memset(&GraphicData, 0, sizeof(ext_SendClientData_graphic_t));
	  GraphicData.txFrameHeader.SOF = 0xA5;
	  GraphicData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t)+ sizeof(ext_client_custom_graphic_number_t);
	  GraphicData.txFrameHeader.Seq = 0;					
	  memcpy(CliendTxBuffer, &GraphicData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
	  Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码
	  GraphicData.CmdID = 0x0301;
	  GraphicData.dataFrameHeader.data_cmd_id = 0x0102;//内容id
	  GraphicData.dataFrameHeader.send_ID 	 =GameRobotState.robot_id;//发送者的ID
	  GraphicData.dataFrameHeader.receiver_ID =(GameRobotState.robot_id | 0x0100);//客户端的ID，只能为发送者机器人对应的客户端
//		//粗直线
//	  GraphicData.graphicData.graphic_data_struct[0].graphic_name[0]=41;
//		GraphicData.graphicData.graphic_data_struct[0].operate_type=1;
//	  GraphicData.graphicData.graphic_data_struct[0].graphic_type=0;
//		GraphicData.graphicData.graphic_data_struct[0].layer=2;
//		GraphicData.graphicData.graphic_data_struct[0].color=6;
//		GraphicData.graphicData.graphic_data_struct[0].start_angle=0;
//		GraphicData.graphicData.graphic_data_struct[0].end_angle=0;
//		GraphicData.graphicData.graphic_data_struct[0].width=50;
//		GraphicData.graphicData.graphic_data_struct[0].start_x=560;
//		GraphicData.graphicData.graphic_data_struct[0].start_y=1080-230;
//		GraphicData.graphicData.graphic_data_struct[0].radius=0;
//		GraphicData.graphicData.graphic_data_struct[0].end_x=1920-560;
//		GraphicData.graphicData.graphic_data_struct[0].end_y=1080-230;
		//矩形
		GraphicData.graphicData.graphic_data_struct[0].graphic_name[0]=43;
		GraphicData.graphicData.graphic_data_struct[0].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[0].graphic_type=1;
		GraphicData.graphicData.graphic_data_struct[0].layer=1;
		GraphicData.graphicData.graphic_data_struct[0].color=7;
		GraphicData.graphicData.graphic_data_struct[0].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[0].width=5;
		GraphicData.graphicData.graphic_data_struct[0].start_x=555;
		GraphicData.graphicData.graphic_data_struct[0].start_y=1080-200;
		GraphicData.graphicData.graphic_data_struct[0].radius=0;
		GraphicData.graphicData.graphic_data_struct[0].end_x=1920-565;
		GraphicData.graphicData.graphic_data_struct[0].end_y=1080-260;
		
	  GraphicData.graphicData.graphic_data_struct[1].graphic_name[0]=42;
		GraphicData.graphicData.graphic_data_struct[1].operate_type=1;
	  GraphicData.graphicData.graphic_data_struct[1].graphic_type=0;
		GraphicData.graphicData.graphic_data_struct[1].layer=1;
		GraphicData.graphicData.graphic_data_struct[1].color=7;
		GraphicData.graphicData.graphic_data_struct[1].start_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].end_angle=0;
		GraphicData.graphicData.graphic_data_struct[1].width=60;
		GraphicData.graphicData.graphic_data_struct[1].start_x=560+264;
		GraphicData.graphicData.graphic_data_struct[1].start_y=1080-230;
		GraphicData.graphicData.graphic_data_struct[1].radius=0;
		GraphicData.graphicData.graphic_data_struct[1].end_x=560+274;
		GraphicData.graphicData.graphic_data_struct[1].end_y=1080-230;

	  memcpy(	
			CliendTxBuffer + 5,
			(uint8_t*)&GraphicData.CmdID, 
			(sizeof(GraphicData.CmdID)+ sizeof(GraphicData.dataFrameHeader)+ sizeof(GraphicData.graphicData))
		);
		
	  Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(GraphicData));//写入数据段CRC16校验码
	  lenth=sizeof(GraphicData);
		datalength = sizeof(CliendTxBuffer);
    while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&CliendTxBuffer,datalength)!=HAL_OK);	
}

void JUDEG_UI(void) {
		uint8_t UIflag=0;
	  char persent_x[5]={0};
		if(rc.key[15])
		{
			UIflag=1;
		}
		itoa(client_state_now.capacity_percent,persent_x);
    if(UIflag)
		{
      UIflag=0;			

			Referee_Transmit_graphic_Init();
			osDelay(100);
			Referee_Transmit_graphic();
			osDelay(100);
      Referee_Transmit_character_top(1,"move");
			osDelay(100);
			Referee_Transmit_character_wheel(1,"wheel:off");
			osDelay(100);
			Referee_Transmit_character_AiMBot(1,"no aim");
			osDelay(100);
			Referee_Transmit_character_bin_cover(1,"cover");
			osDelay(100);
			Referee_Transmit_character_rate(1,"x1.0");
			osDelay(100);
			//Referee_Transmit_character_present(1,persent_x);
			Referee_Transmit_character_present(1,persent_x);
		  osDelay(100);
			Referee_Transmit_graphic_Precent(1);
			osDelay(100);
		}
		//Referee_Transmit_graphic_2(2);
		Referee_Transmit_character_present(2,persent_x);
		osDelay(100);
		Referee_Transmit_graphic_Precent(2);
		osDelay(100);	
		
		if(client_state_now.aim!=client_state_past.aim)
		{
			switch(client_state_now.aim)
			{
				case 1:
					Referee_Transmit_character_AiMBot(2,"armor");
					osDelay(100);
					break;
				case 2:
					Referee_Transmit_character_AiMBot(2,"energy");
					osDelay(100);
					break;
				case 3:
					Referee_Transmit_character_AiMBot(2,"no aim");
					osDelay(100);
					break;
			}
//			client_state_past.aim=client_state_now.aim;
	  }
		if(client_state_now.speed_dial_shoot!=client_state_past.speed_dial_shoot)
		{
			switch(client_state_now.speed_dial_shoot)
			{
				case 1:
					Referee_Transmit_character_rate(2,"wheel:x1.0");
					osDelay(100);
					break;
				case 2:
					Referee_Transmit_character_rate(2,"wheel:x2.0");
					osDelay(100);
					break;
				case 3:
					Referee_Transmit_character_rate(2,"wheel:x3.0");
					osDelay(100);
					break;
			}
//			client_state_past.speed_dial_shoot=client_state_now.speed_dial_shoot;
	  }
		if(client_state_now.motion!=client_state_past.motion)
		{
			switch(client_state_now.motion)
			{
				case 1:
					Referee_Transmit_character_top(2,"move");
					osDelay(100);
					break;
				case 2:
					Referee_Transmit_character_top(2,"rotate");
					osDelay(100);
					break;
				case 3:
					Referee_Transmit_character_top(2,"follow");
					osDelay(100);
					break;
			}
//			client_state_past.motion=client_state_now.motion;
	}
		if(client_state_now.wheel_of!=client_state_past.wheel_of)
		{
			switch(client_state_now.wheel_of)
			{
				case 0:
					Referee_Transmit_character_wheel(2,"wheel:off");
					osDelay(100);
					break;
				case 1:
					Referee_Transmit_character_wheel(2,"wheel:on");
					osDelay(100);
					break;
		  }
//			client_state_past.motion=client_state_now.motion;

	}
		if(client_state_now.bin_cover_of!=client_state_past.bin_cover_of)
		{
			switch(client_state_now.bin_cover_of)
			{
				case 1:
					Referee_Transmit_character_bin_cover(2,"cover:open");
					osDelay(100);
					break;
				case 0:
					Referee_Transmit_character_bin_cover(2,"cover:off");
					osDelay(100);
					break;
			}
//			client_state_past.bin_cover_of=client_state_now.bin_cover_of;
    }
		client_state_past=client_state_now;
}
