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

/**
 * @brief 电机目标数据
 * @note 数组位号与电机ID对应
 */    
MotorTarget motorTarget[7] = { 0 };

/**
 * @brief 电机结构体
 * @note leftJoint[0]:左前关节电机, leftJoint[1]:左后关节电机, leftWheel:左车轮电机
 *       rightJoint[0]:右前关节电机, rightJoint[1]:右后关节电机, rightWheel:右车轮电机
 */           
Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;

/* 电机控制模式标志位 */
MOTOR_STATUS MotorStatus = INITIAL_IDLE;

/* 从CAN总线接收到的数据中解析出算法中电机角度和速度 */
void Motor_Update(Motor *motor, MW_MOTOR_DATA *data) {
	// motor->angle = (*(int32_t *)&data[0] / 1000.0f - motor->offsetAngle) * motor->dir;
	// motor->speed = (*(int16_t *)&data[4] / 10 * 2 * M_PI / 60) * motor->dir;
	motor->angle = (((data->encoderPosEstimate / 9.67f) * 2.0f * M_PI) - motor->offsetAngle) * motor->dir;
	motor->speed = ((data->encoderVelEstimate / 9.67f) * 2.0f * M_PI) * motor->dir;
}

/* 用户自创建总线发送函数 */
void MotorBusSend(uint8_t busId, uint8_t can_id, uint8_t *data, uint8_t dataSize) {
    if(busId == 0x001) CAN_SendFrame(can_id, data);
}

/* 用户自创建总线消息函数 */
void MotorNotice(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId) {
    if(busId == 0x001) return;
}  

/* 电机算法数据初始化 */
void Motor_Init(Motor *motor, float offsetAngle, float maxVoltage, float torqueRatio, float dir) {
	motor->speed = motor->angle = motor->voltage = 0;
	motor->offsetAngle = offsetAngle;
	motor->maxVoltage = maxVoltage;
	motor->torqueRatio = torqueRatio;
	motor->dir = dir;
}

/* 电机周期任务 */
void Motor_Task(void *arg) {
    motorTarget[1].torque = 0;
    motorTarget[2].torque = 0;
    motorTarget[3].torque = 0;
    motorTarget[4].torque = 0;
    motorTarget[5].torque = 0;
    motorTarget[6].torque = 0;
    vTaskDelay(500);
    MotorStatus = DIRECT_TOQUECONTROL;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1) {
        switch (MotorStatus) {
            case DIRECT_VELCONTROL:
                
                break;
            case DIRECT_POSCONTROL:
                
                break;    
            case DIRECT_TOQUECONTROL:
                MWSetControllerMode(MWjoint1.busId, MWjoint1.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
                MWSetControllerMode(MWjoint2.busId, MWjoint2.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
                MWSetControllerMode(MWwheel3.busId, MWwheel3.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
                MWSetControllerMode(MWjoint4.busId, MWjoint4.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
                MWSetControllerMode(MWjoint5.busId, MWjoint5.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
                MWSetControllerMode(MWwheel6.busId, MWwheel6.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);

                MWTorqueControl(MWjoint1.busId, MWjoint1.nodeId, motorTarget[1].torque);
                MWTorqueControl(MWjoint2.busId, MWjoint2.nodeId, motorTarget[2].torque);
                MWTorqueControl(MWwheel3.busId, MWwheel3.nodeId, motorTarget[3].torque);
                MWTorqueControl(MWjoint4.busId, MWjoint4.nodeId, motorTarget[4].torque);
                MWTorqueControl(MWjoint5.busId, MWjoint5.nodeId, motorTarget[5].torque);
                MWTorqueControl(MWwheel6.busId, MWwheel6.nodeId, motorTarget[6].torque);
                break;
            default:
                break;
        }

		vTaskDelayUntil(&xLastWakeTime, 2);
	}
}

/* 电机初始化 */
void Motor_InitAll() {
	/* 设置控制模式 */
	MWSetControllerMode(MWjoint1.busId, MWjoint1.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
	MWSetControllerMode(MWjoint2.busId, MWjoint2.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
	MWSetControllerMode(MWwheel3.busId, MWwheel3.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
    MWSetControllerMode(MWjoint4.busId, MWjoint4.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
	MWSetControllerMode(MWjoint5.busId, MWjoint5.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
	MWSetControllerMode(MWwheel6.busId, MWwheel6.nodeId, MW_TORQUE_CONTROL, MW_DIRECT_CONTROL_INPUT);
	/* 设置惯量值为0 */
	MWSetTrajInertia(MWjoint1.busId, MWjoint1.nodeId, 0);
	MWSetTrajInertia(MWjoint2.busId, MWjoint2.nodeId, 0);
	MWSetTrajInertia(MWwheel3.busId, MWwheel3.nodeId, 0);
    MWSetTrajInertia(MWjoint4.busId, MWjoint4.nodeId, 0);
	MWSetTrajInertia(MWjoint5.busId, MWjoint5.nodeId, 0);
	MWSetTrajInertia(MWwheel6.busId, MWwheel6.nodeId, 0);

#ifdef MOTOR_DEBUG 
    MWSetPosGain(MWjoint1.busId, MWjoint1.nodeId, 0);
    MWSetPosGain(MWjoint2.busId, MWjoint2.nodeId, 0);
    MWSetPosGain(MWwheel3.busId, MWwheel3.nodeId, 0);
    MWSetPosGain(MWjoint4.busId, MWjoint4.nodeId, 0);
    MWSetPosGain(MWjoint5.busId, MWjoint5.nodeId, 0);
    MWSetPosGain(MWwheel6.busId, MWwheel6.nodeId, 0);

    MWSetVelGain(MWjoint1.busId, MWjoint1.nodeId, 0, 0);
    MWSetVelGain(MWjoint2.busId, MWjoint2.nodeId, 0, 0);
    MWSetVelGain(MWwheel3.busId, MWwheel3.nodeId, 0, 0);
    MWSetVelGain(MWjoint4.busId, MWjoint4.nodeId, 0, 0);
    MWSetVelGain(MWjoint5.busId, MWjoint5.nodeId, 0, 0);
    MWSetVelGain(MWwheel6.busId, MWwheel6.nodeId, 0, 0);
#endif
    MWSetPosGain(MWjoint1.busId, MWjoint1.nodeId, 20);
    MWSetPosGain(MWjoint2.busId, MWjoint2.nodeId, 20);
    MWSetPosGain(MWwheel3.busId, MWwheel3.nodeId, 20);
    MWSetPosGain(MWjoint4.busId, MWjoint4.nodeId, 20);
    MWSetPosGain(MWjoint5.busId, MWjoint5.nodeId, 20);
    MWSetPosGain(MWwheel6.busId, MWwheel6.nodeId, 20);

    MWSetVelGain(MWjoint1.busId, MWjoint1.nodeId, 0.16, 0.32);
    MWSetVelGain(MWjoint2.busId, MWjoint2.nodeId, 0.16, 0.32);
    MWSetVelGain(MWwheel3.busId, MWwheel3.nodeId, 0.16, 0.32);
    MWSetVelGain(MWjoint4.busId, MWjoint4.nodeId, 0.16, 0.32);
    MWSetVelGain(MWjoint5.busId, MWjoint5.nodeId, 0.16, 0.32);
    MWSetVelGain(MWwheel6.busId, MWwheel6.nodeId, 0.16, 0.32);
    // MWSetAxisState(1, 1, MW_AXIS_STATE_MOTOR_CALIBRATION);
	// MWSetAxisState(1, 2, MW_AXIS_STATE_MOTOR_CALIBRATION);
	// MWSetAxisState(1, 3, MW_AXIS_STATE_MOTOR_CALIBRATION);
    // MWSetAxisState(1, 4, MW_AXIS_STATE_MOTOR_CALIBRATION);
	// MWSetAxisState(1, 5, MW_AXIS_STATE_MOTOR_CALIBRATION);
	// MWSetAxisState(1, 6, MW_AXIS_STATE_MOTOR_CALIBRATION);
	/* 进入闭环控制状态 */
	MWSetAxisState(MWjoint1.busId, MWjoint1.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
	MWSetAxisState(MWjoint2.busId, MWjoint2.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
	MWSetAxisState(MWwheel3.busId, MWwheel3.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    MWSetAxisState(MWjoint4.busId, MWjoint4.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
	MWSetAxisState(MWjoint5.busId, MWjoint5.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
	MWSetAxisState(MWwheel6.busId, MWwheel6.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);

	// /* 输入控制位置 */
    // MWPosControl(1, 1, 5, 0, 0);
    // MWPosControl(1, 2, 5, 0, 0);
	// MWPosControl(1, 3, 5, 0, 0);
    // MWPosControl(1, 4, 5, 0, 0);
    // MWPosControl(1, 5, 5, 0, 0);
	// MWPosControl(1, 6, 5, 0, 0);
#ifdef UP_DEBUG
    /* 算法电机数据初始化 */
    Motor_Init(&leftJoint[0], 1.746f, 24, 0.042f, -1);
	Motor_Init(&leftJoint[1], 1.650f, 24, 0.042f, -1);
	Motor_Init(&leftWheel, 0, 24, 0.042f, -1);
	Motor_Init(&rightJoint[0], -1.895f, 24, 0.042f, 1);
	Motor_Init(&rightJoint[1], -1.835f, 24, 0.042f, 1);
	Motor_Init(&rightWheel, 0, 24, 0.042f, 1);
#else
    Motor_Init(&leftJoint[0], -0.1079f, 24, 0.40614f, -1);
	Motor_Init(&leftJoint[1], 3.5498f, 24, 0.40614f, -1);
	Motor_Init(&leftWheel, 0, 24, 0.40614f, -1);
	Motor_Init(&rightJoint[0], -0.0609f, 24, 0.40614f, 1);
	Motor_Init(&rightJoint[1], -3.0200f, 24, 0.40614f, 1);
	Motor_Init(&rightWheel, 0, 24, 0.40614f, 1);
#endif
	xTaskCreate(Motor_Task, "Motor_Task", 2048, NULL, 5, NULL);
}

