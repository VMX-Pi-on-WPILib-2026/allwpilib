#pragma once

#include <stdint.h>
#include "hal/CAN.h"

inline void FRC_NetworkCommunication_CANSessionMux_sendMessage(
	uint32_t messageID, 
	const uint8_t *data, 
	uint8_t dataSize, 
	uint32_t periodMs, 
	int32_t *status) 
{
	HAL_CAN_SendMessage(messageID, data, dataSize, static_cast<int32_t>(periodMs), status);
}

inline void FRC_NetworkCommunication_CANSessionMux_receiveMessage(
	uint32_t *messageID, 
	uint32_t messageIDMask, 
	uint8_t *data, 
	uint8_t* dataSize, 
	uint32_t* timeStamp, 
	int32_t *status) 
{
	HAL_CAN_ReceiveMessage(messageID, messageIDMask, data, dataSize, timeStamp, status);
}
