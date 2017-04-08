#ifndef __CANPARSE_H
#define __CANPARSE_H

#include "canIdentifer.h"
#include "FreeRTOS.h"

#include "stm32f4xx_hal.h"
#include "can.h"
#include "structPara.h"

#include "SEGGER_RTT.h"

void canSbusDataParse(CanRxMsgTypeDef rx);
void canAirSpeedParse(CanRxMsgTypeDef rx);
HAL_StatusTypeDef txDataSend(CAN_HandleTypeDef *hcan, uint32_t message, uint8_t len, uint8_t* buff);

#endif