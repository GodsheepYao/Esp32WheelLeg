#include <Arduino.h>
#include <esp_task_wdt.h>
#include "MWMotor.h"
#include "MotorProcess.h"
#include "CANDrive.h"

twai_message_t msg;
twai_status_info_t status;

/******* CAN通信模块 *******/
/* CAN接收任务 */
void CAN_RecvTask(void *arg)
{
	twai_message_t msg;
	twai_status_info_t status;
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1) {
		twai_get_status_info(&status);
		for(uint8_t i = 0; i < status.msgs_to_rx; i++) {
			if(twai_receive(&msg, 0) == ESP_OK) {
				MWReceiver(1, msg.identifier, msg.data);
				uint8_t nodeId = msg.identifier >> 5;
				switch (nodeId) //根据CAN ID更新各电机数据
				{
					case 0x01:
						Motor_Update(&rightJoint[0], &MWjointData1);
						break;
					case 0x02:
						Motor_Update(&rightJoint[1], &MWjointData2);
						break;
					case 0x03:
						Motor_Update(&rightWheel, &MWwheelData3);
						break;
					case 0x04:
						Motor_Update(&leftJoint[1], &MWjointData4);
						break;
					case 0x05:
						Motor_Update(&leftJoint[0], &MWjointData5);
						break;
					case 0x06:
						Motor_Update(&leftWheel, &MWwheelData6);
						break;
				}
			}
		}
		vTaskDelayUntil(&xLastWakeTime, 2); //2ms轮询一次
	}
}

/* CAN发送 */
void CAN_SendFrame(uint32_t id, uint8_t *data) {
	msg.flags = 0;
	msg.identifier = id;
	msg.data_length_code = 8;
	memcpy(msg.data, data, 8);
	twai_transmit(&msg, 100);
}

/* CAN初始化 */
void CAN_Init(void)
{
	twai_general_config_t twai_conf = {
		.mode = TWAI_MODE_NORMAL,
		.tx_io = GPIO_NUM_6,
		.rx_io = GPIO_NUM_7,
		.clkout_io = TWAI_IO_UNUSED,
		.bus_off_io = TWAI_IO_UNUSED,
		.tx_queue_len = 5,
		.rx_queue_len = 10,
		.alerts_enabled = TWAI_ALERT_NONE,
		.clkout_divider = 0,
		.intr_flags = ESP_INTR_FLAG_LEVEL1};

	twai_timing_config_t twai_timing = TWAI_TIMING_CONFIG_500KBITS();

	twai_filter_config_t twai_filter = {
		.acceptance_code = 0x00000000,
		.acceptance_mask = 0xFFFFFFFF,
		.single_filter = true};

	twai_driver_install(&twai_conf, &twai_timing, &twai_filter);
	twai_start();
	xTaskCreate(CAN_RecvTask, "CAN_RecvTask", 2048, NULL, 5, NULL);
}
