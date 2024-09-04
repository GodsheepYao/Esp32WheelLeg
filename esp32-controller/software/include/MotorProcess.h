#ifndef _MOTORPROCESS_H_
#define _MOTORPROCESS_H_

#include "MWMotor.h"
#include <esp_task_wdt.h>
#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_DEBUG 1

typedef struct Motor {
	float speed;			   // rad/s
	float angle, offsetAngle;  // rad
	float voltage, maxVoltage; // V
	float torque, torqueRatio; // Nm, voltage = torque / torqueRatio
	float dir;				   // 1 or -1
	float (*calcRevVolt)(float speed); // 指向反电动势计算函数
} Motor; 

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

extern Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;

void Motor_Update(Motor *motor, MW_MOTOR_DATA *data);
void MotorBusSend(uint8_t busId, uint8_t can_id, uint8_t *data, uint8_t dataSize);
void MotorNotice(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId);
void Motor_InitAll();

#ifdef __cplusplus
}
#endif

#endif
