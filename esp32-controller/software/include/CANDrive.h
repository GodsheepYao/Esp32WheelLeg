#ifndef _CANDrive_H_
#define _CANDrive_H_

#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t FDCAN1_buff[8];        //!<@brief FDCAN1接收缓冲区

void CAN_Init(void);

void CAN_SendFrame(uint32_t id, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif //_CANDrive_H_
