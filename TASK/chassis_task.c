/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             ���̿�������
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
#include "chassis_task.h"
#include "rc_control.h"
#include "arm_math.h"

uint16_t time_chassis;



//�����˶�
uint16_t FollowInitAngle=5312; //4216
uint16_t classis_speed,spin_speed;   //���̻����ٶ�,С�����ٶȸ��ݲ���ϵͳ�ı�
uint8_t SpinStartFlag,IsSpinFlag;//С���ݱ�־
uint8_t FollowSwitchFlag,IsFollowFlag=1,LastIsFollowFlag=1;
/**
  * @brief          ���̿��ƺ���
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_rc_ctrl(void);
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //����һ��ʱ��
    osDelay(CHASSIS_TASK_INIT_TIME);
		classis_speed=1000;//�ƶ��ٶ�
		spin_speed=1000;//С�����ٶ�
	  for(int i=0; i<4; i++)	// �ĸ����̵��
    {
        PID_Init(&PID_M3508[i],DELTA_PID,8000,8000,10,0.05,0.5);//5,0.01
			  PID_Init(&PID_GM6020[i],POSITION_PID,10000,10000,35,0.01,110);
			  PID_Init(&PID_GM6020_speed[i],POSITION_PID,25000,25000,200,0,0);
    }
    static portTickType lastWakeTime;  
		lastWakeTime = xTaskGetTickCount(); 
    while (1)
    {
				time_chassis++;
				if(rc_flag<3)
				{
				
					chassis_rc_ctrl() ;
				}
				else    //��ң����ʧ�عر���յ���ֵ
				{
					  CAN_M3508[0].set_current=0;
            CAN_M3508[1].set_current=0;
            CAN_M3508[2].set_current=0;
            CAN_M3508[3].set_current=0;
					CAN_GM6020[0].set_voltage=0;
					CAN_GM6020[1].set_voltage=0;
					CAN_GM6020[2].set_voltage=0;
					CAN_GM6020[3].set_voltage=0;
				}
				//CANͨ��������ĸ�������͵���ֵ
				CAN_Chassis_SendCurrent();
				CAN_Gimbal_SendVoltage();
        //ϵͳ������ʱ
        osDelayUntil(&lastWakeTime,CHASSIS_CONTROL_TIME_MS);
    }
}
	/*************************�����˶�*******************************/
double Vx,Vy,Vw;
double setAngle[4];//6020�趨�Ƕ�
double Radius=1.0;//Խ����ҡ�˿���Խǿ     
int flag_a[4]={1,1,1,1};
int drct[4]={1,1,1,1};//3508��ת����
double angle_temp[4];  
double CHASSIS_6020_Y_ANGLE[4]={5370,536,3436,8077};//6020��ʼ�Ƕȣ���ʱ3508ȫ����ǰ��������ǰΪ��
double wheel_angle_last[4]={5370,536,3436,8077};    //
int flag_drct=1;

void calc_3508_speed(void);//3508�ٶȽ���
void calc_6020_angle(void);//6020�ǶȽ���
double Find_min_Angle(int16_t angle1,double angle2);
void AngleLoop_int(int16_t *a,int n);
void AngleLoop_f(double *a,int n);

void chassis_rc_ctrl(void)
{  
//	if(rc.sw2==1) {
//		        CAN_M3508[0].set_current=0;
//            CAN_M3508[1].set_current=0;
//						CAN_M3508[2].set_current=0;
//            CAN_M3508[3].set_current=0;
//		        CAN_M3508[4].set_current=0;
//            CAN_M3508[5].set_current=0;
//            CAN_GM6020[0].set_voltage=0;
//            CAN_GM6020[1].set_voltage=0;
//		        spin_angle_set=imu.yawtx;
//		        pitch_angle_set=(float)CAN_GM6020[1].angle/8191*360;
//						CAN_GM6020[0].total_angle=CAN_GM6020[0].angle;
//            velocity(rc.ch2,rc.ch3);
//            for(int i=0; i<4; i++) 
//            {
//                setSpeed[i] = map(setSpeed[i]+rc.ch0*0.6,-1320,1320,-10000,10000);
//                CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
//            }
//		}
  
  if(rc.sw2==1) {
      Vx=(double)rc.ch2/220.0;
			Vy=(double)rc.ch3/220.0;    
      Vw=(double)rc.ch1/220.0;    
		
		calc_3508_speed();
		calc_6020_angle();
		
		for(int i=0;i<4;i++){
			if(fabs(Find_min_Angle(CAN_GM6020[i].angle,setAngle[i]) ) > 2048 ){
				setAngle[i]+=4096;	 
        if(flag_a[i])	{			
				  drct[i] = -drct[i];		
				  flag_a[i]=0;
			  }	
			}
			else flag_a[i]=1;
			
	}
	
  for(int i=0;i<4;i++){//�ػ�,���ڱ仯��4096����������һ��
			for( ; setAngle[i] > 8191; )setAngle[i] -= 8191;
	    for( ; setAngle[i] < 0; )setAngle[i]+= 8191;
		}  
		
  if(Vx == 0 && Vy == 0 && Vw == 0)//ҡ�˻���,����ԭ�Ƕ�
  {
    for(int i=0;i<4;i++)
     setAngle[i] = wheel_angle_last[i];
  }
  else
  {
    for(int i=0;i<4;i++)
		  wheel_angle_last[i] = CAN_GM6020[i].angle;
  }
	
		for(int i=0;i<4;i++){
			   CAN_GM6020[i].set_voltage =  PID_Calculate(&PID_GM6020[i], setAngle[i], CAN_GM6020[i].angle);		
			   CAN_M3508[i].set_current  =   PID_Calculate(&PID_M3508[i], (drct[i])*(setSpeed[i]), CAN_M3508[i].speed);
		}
		if(flag_drct){
			for(int i=0;i<4;i++)drct[i]=1;
			flag_drct=0;
		}
	}
}


void calc_3508_speed(){
	
	  double  k=200;//
	  setSpeed[0] = sqrt(	pow(Vy - Vw * Radius * 0.707107f,2)
                       +	pow(Vx - Vw * Radius * 0.707107f,2)
                       ) * k;
    setSpeed[1] = sqrt(	pow(Vy - Vw * Radius * 0.707107f,2)
                       +	pow(Vx + Vw * Radius * 0.707107f,2)
                       ) * k;
    setSpeed[2] = sqrt(	pow(Vy + Vw * Radius * 0.707107f,2)
                       +	pow(Vx + Vw * Radius * 0.707107f,2)
                       ) * k;
    setSpeed[3] = sqrt(	pow(Vy + Vw * Radius * 0.707107f,2)
                       +	pow(Vx - Vw * Radius * 0.707107f,2) 
                       ) * k;
	  if(Vx == 0 && Vy == 0 && Vw == 0)//ҡ�˻���ʱ
  {
    for(int i=0;i<4;i++)
     setSpeed[i] = 0;
  }	
}


void calc_6020_angle(){

    double atan_angle[4]; 

	  //6020Ŀ��Ƕȼ���
    if(!((( Vx - Vw*Radius*0.707107f)==0)&&((Vx + Vw*Radius*0.707107f)==0)))//��ֹ��Ϊ0
    {
      atan_angle[0]=atan2(( Vx - Vw*Radius*0.707107f),(Vy - Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[1]=atan2(( Vx + Vw*Radius*0.707107f),(Vy - Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[2]=atan2(( Vx + Vw*Radius*0.707107f),(Vy + Vw*Radius*0.707107f))*180.0f/PI;
      atan_angle[3]=atan2(( Vx - Vw*Radius*0.707107f),(Vy + Vw*Radius*0.707107f))*180.0f/PI;	
    }  
		
		setAngle[0] = CHASSIS_6020_Y_ANGLE[0] + (atan_angle[0]*22.75);//22.75Ϊÿ�仯1�ȱ�����ֵ�ı仯
		setAngle[1] = CHASSIS_6020_Y_ANGLE[1] + (atan_angle[1]*22.75);//�� 8192/360.0
		setAngle[2] = CHASSIS_6020_Y_ANGLE[2] + (atan_angle[2]*22.75);
		setAngle[3] = CHASSIS_6020_Y_ANGLE[3] + (atan_angle[3]*22.75);

		for(int i=0;i<4;i++){//�ػ�
			for( ; setAngle[i] > 8191; )setAngle[i] -= 8191;
	    for( ; setAngle[i] < 0; )setAngle[i]+= 8191;
		}  

}

double Find_min_Angle(int16_t angle1,double angle2)
{
	  double err;
    err = (double)angle1 - angle2;
    if(fabs(err) > 4096)
    {
        err = 8192 - fabs(err);
    }
    return err;
}

