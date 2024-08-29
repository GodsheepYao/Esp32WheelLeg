#include "MotorProcess.h"
#include "CANDrive.h"

/* 电机数据接收 */
MW_MOTOR_DATA MWjointData1;
MW_MOTOR_DATA MWjointData2;
MW_MOTOR_DATA MWwheelData3;
MW_MOTOR_DATA MWjointData4;
MW_MOTOR_DATA MWjointData5;
MW_MOTOR_DATA MWwheelData6;

/* 电机登记数据 */
MW_MOTOR_ACCESS_INFO MWjoint1 = {.busId = 1, 
                               .nodeId = 1, 
                               .motorData = &MWjointData1, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWjoint2 = {.busId = 1, 
                               .nodeId = 2, 
                               .motorData = &MWjointData2, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWwheel3 = {.busId = 1, 
                               .nodeId = 3, 
                               .motorData = &MWwheelData3, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWjoint4 = {.busId = 1, 
                               .nodeId = 4, 
                               .motorData = &MWjointData4, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWjoint5 = {.busId = 1, 
                               .nodeId = 5, 
                               .motorData = &MWjointData5, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWwheel6 = {.busId = 1, 
                               .nodeId = 6, 
                               .motorData = &MWwheelData6, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};

/* 用户自创建总线发送函数 */
void MotorBusSend(uint8_t busId, uint8_t can_id, uint8_t *data, uint8_t dataSize) {
    if(busId == 0x001) CAN_SendFrame(can_id, data);
}

/* 用户自创建总线消息函数 */
void MotorNotice(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId) {
    if(busId == 0x001) return;
}  

void Motor_Task(void *arg) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1) {
        printf("1Speed:%f  Pos:%f\r\n", MWjoint1.motorData->encoderVelEstimate, MWjoint1.motorData->encoderPosEstimate);
        printf("2Speed:%f  Pos:%f\r\n", MWjoint2.motorData->encoderVelEstimate, MWjoint2.motorData->encoderPosEstimate);
        printf("3Speed:%f  Pos:%f\r\n", MWwheel3.motorData->encoderVelEstimate, MWwheel3.motorData->encoderPosEstimate);
        printf("4Speed:%f  Pos:%f\r\n", MWjoint4.motorData->encoderVelEstimate, MWjoint4.motorData->encoderPosEstimate);
        printf("5Speed:%f  Pos:%f\r\n", MWjoint5.motorData->encoderVelEstimate, MWjoint5.motorData->encoderPosEstimate);
        printf("6Speed:%f  Pos:%f\r\n", MWwheel6.motorData->encoderVelEstimate, MWwheel6.motorData->encoderPosEstimate);
		vTaskDelayUntil(&xLastWakeTime, 50);
	}
}

/* 电机初始化 */
void Motor_InitAll() {
	/* 设置模式为位置滤波控制模式 */
	MWSetControllerMode(1, 1, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
	MWSetControllerMode(1, 2, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
	MWSetControllerMode(1, 3, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
    MWSetControllerMode(1, 4, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
	MWSetControllerMode(1, 5, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
	MWSetControllerMode(1, 6, MW_POSITION_CONTROL, MW_POSITION_FILTERING_INPUT);
	/* 设置惯量值为0 */
	MWSetTrajInertia(1, 1, 0);
	MWSetTrajInertia(1, 2, 0);
	MWSetTrajInertia(1, 3, 0);
    MWSetTrajInertia(1, 4, 0);
	MWSetTrajInertia(1, 5, 0);
	MWSetTrajInertia(1, 6, 0);

    // MWSetAxisState(1, 1, MW_AXIS_STATE_MOTOR_CALIBRATION);
	// MWSetAxisState(1, 2, MW_AXIS_STATE_MOTOR_CALIBRATION);
	// MWSetAxisState(1, 3, MW_AXIS_STATE_MOTOR_CALIBRATION);
    // MWSetAxisState(1, 4, MW_AXIS_STATE_MOTOR_CALIBRATION);
	// MWSetAxisState(1, 5, MW_AXIS_STATE_MOTOR_CALIBRATION);
	// MWSetAxisState(1, 6, MW_AXIS_STATE_MOTOR_CALIBRATION);
	/* 进入闭环控制状态 */
	MWSetAxisState(1, 1, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
	MWSetAxisState(1, 2, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
	MWSetAxisState(1, 3, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    MWSetAxisState(1, 4, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
	MWSetAxisState(1, 5, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
	MWSetAxisState(1, 6, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
	// /* 输入控制位置 */
    // MWPosControl(1, 1, 5, 0, 0);
    // MWPosControl(1, 2, 5, 0, 0);
	// MWPosControl(1, 3, 5, 0, 0);
    // MWPosControl(1, 4, 5, 0, 0);
    // MWPosControl(1, 5, 5, 0, 0);
	// MWPosControl(1, 6, 5, 0, 0);

    // Motor_Init(&leftJoint[0], 1.431, 7, 0.0316f, -1, Motor_CalcRevVolt4010);
	// Motor_Init(&leftJoint[1], -7.76, 7, 0.0317f, 1, Motor_CalcRevVolt4010);
	// Motor_Init(&leftWheel, 0, 4.0f, 0.0096f, 1, Motor_CalcRevVolt2804);
	// Motor_Init(&rightJoint[0], 0.343, 7, 0.0299f, -1, Motor_CalcRevVolt4010);
	// Motor_Init(&rightJoint[1], -2.446, 7, 0.0321f, -1, Motor_CalcRevVolt4010);
	// Motor_Init(&rightWheel, 0, 4.0f, 0.0101f, 1, Motor_CalcRevVolt2804);

	xTaskCreate(Motor_Task, "Motor_Task", 2048, NULL, 5, NULL);
}

