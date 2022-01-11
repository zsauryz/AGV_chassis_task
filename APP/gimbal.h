#ifndef __GIMBAL_H
#define __GIMBAL_H
#include "main.h"

#define angle8191_to_angle2pi(a) ((a)/8191.0*360)
#define yaw_to_2pi(a)	((a)+180)




typedef struct
{
	float yaw_angle;//偏航角
	float pitch_angle;//俯仰角
	float shoot_delay;
	float dis;//目标距离
    int ismiddle;//设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，目前暂不使用，默认置0
	int isFindTarget;//当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int isfindDafu;
    int nearFace;
} VisionData;
typedef union
{
    float f;
    unsigned char c[2];
} float2uchar;

////用于保存目标相关角度和距离信息及瞄准情况
//typedef struct
//{
//	float2uchar yaw_angle;//偏航角
//	float2uchar pitch_angle;//俯仰角
//	float2uchar z_angle;
//	float2uchar dis;//目标距离
//    int ismiddle;//设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，目前暂不使用，默认置0
//	int isFindTarget;//当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
//    int isfindDafu;
//    int nearFace;
//} VisionData;

extern float spin_angle_set;
extern float pitch_angle_set;
void VisionUartTxSenddata(void);
//void VisionUartRxCpltCallback_2(void);
void VisionUartRxCpltCallback(void);
void  Vision_aiming(void);
void Pluck_angle(float angle,int m);
#endif

