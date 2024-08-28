#ifndef _MOTORPROCESS_H_
#define _MOTORPROCESS_H_

#include "MWMotor.h"
#include <esp_task_wdt.h>
#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

extern MW_MOTOR_DATA MWjointData1;
extern MW_MOTOR_DATA MWjointData2;
extern MW_MOTOR_DATA MWwheelData3;
extern MW_MOTOR_DATA MWjointData4;
extern MW_MOTOR_DATA MWjointData5;
extern MW_MOTOR_DATA MWwheelData6;

extern MW_MOTOR_ACCESS_INFO MWjoint1;
extern MW_MOTOR_ACCESS_INFO MWjoint2;
extern MW_MOTOR_ACCESS_INFO MWwheel3;
extern MW_MOTOR_ACCESS_INFO MWjoint4;
extern MW_MOTOR_ACCESS_INFO MWjoint5;                               
extern MW_MOTOR_ACCESS_INFO MWwheel6;

void MotorBusSend(uint8_t busId, uint8_t can_id, uint8_t *data, uint8_t dataSize);
void MotorNotice(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId);
void Motor_InitAll();

#ifdef __cplusplus
}
#endif

#endif
