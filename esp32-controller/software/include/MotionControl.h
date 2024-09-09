#ifndef _MOTIONCONTROL_H_
#define _MOTIONCONTROL_H_

#include <esp_task_wdt.h>
#include <Arduino.h>
#include "leg_pos.h"
#include "lqr_k.h"
#include "leg_conv.h"
#include "leg_spd.h"
#include "MotorProcess.h"

#ifdef __cplusplus
extern "C" {
#endif

//腿部姿态结构体
typedef struct {
	float angle, length;   // rad, m
	float dAngle, dLength; // rad/s, m/s
	float ddLength;		   // m/s^2
} LegPos; 

extern LegPos leftLegPos, rightLegPos;

void LegPos_UpdateTask(void *arg);

#ifdef __cplusplus
}
#endif

#endif
