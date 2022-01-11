#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "math.h"
#include "stm32f4xx_hal.h"

extern int setSpeed[4];

float map(float i,float imin,float imax,float rmin,float rmax);
float limit(float i,float min,float max);

void velocity(float x,float y);
void BNumberInit(void);

#endif
