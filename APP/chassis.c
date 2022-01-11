/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       chassis.c
 * @brief     
 * @note        
 * @Version    V1.0.0
 * @Date       2021.5     
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 */
 
#include "math.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"
//#include "bsp_imu.h"

struct {
    float PI;
    float POW2;
    float angle1,angle2,angle3;//初始角度，预计算角度，零时角度变量
    float angleM;//操作手摄像头与底盘正向的夹角

    float angN;//目前十轴传感器接收到的偏航角
    float lastx,lasty,x,y;//输入的位移方向向量
    float x1,y1;//实测的方向向量
    float velV,palstance,velA;//输入底盘的位移速度，输入的旋转速度，计算出的麦轮角速度
    float K;//底盘旋转与麦轮角速度的比例系数：K = POW2*R/r(R为对角线上麦轮距离的二分之一(33cm)，r为麦轮半径7cm)
} BN;

float map(float i,float imin,float imax,float rmin,float rmax) {
    i = limit(i,imin,imax);
    return ((i - imin) * (rmax - rmin)/(imax - imin) + rmin);
}

float limit(float i,float min,float max) {
    if(i<min)i=min;
    if(i>max)i=max;
    return i;
}

void BNumberInit() {

    BN.PI = acos(-1.0);
    BN.POW2 = pow(2,0.5);
    BN.K = BN.POW2*33/7;

    BN.angle1 = 0;
    BN.angle2 = 0;
    BN.angle3 = 0;

    BN.angleM = 0;

    BN.angN = 0;

    BN.lastx = 0;
    BN.lasty = 0;

    BN.x = 0;
    BN.y = 0;

    BN.x1 = 0;
    BN.y1 = 0;

    BN.velV = 0;
    BN.palstance = 0;
    BN.velA = 0;
}

int setSpeed[4];	// 四个电机的速度

//单片机每100毫秒接收/刷新一次数据，在此期间以检测值预测计算补间数值10次，向电机发送数据100次

/*
地盘电机方向
	前
	1 2
	3 4
	后
*/
void velocity(float x,float y) { //velocity(位移x，位移y，旋转角速度)

    BNumberInit();

//	BN.angN = imu.yaw;//直接读取角度计（360°）
    BN.angle1 = atan(y/x) + BN.angleM;//计算全部默认弧度制
//    BN.angle2 = BN.angle1 + imu.yaw*BN.PI/180;
    BN.velV = pow(x*x+y*y,0.5);
    BN.velA = 0 * BN.K;//可能需要角度PID闭环
    BN.x = BN.velV * cos(BN.angle2);
    BN.y = BN.velV * sin(BN.angle2);

    BN.lastx = BN.x;
    BN.lasty = BN.y;

//	x = 2*x-imu.mx;//位移修正，需要等加速度计调好
//	y = 2*y-imu.mx;

    setSpeed[0] = x+y+BN.velA;
    setSpeed[1] = x-y+BN.velA;
    setSpeed[2] = -x+y+BN.velA;
    setSpeed[3] = -x-y+BN.velA;
}

