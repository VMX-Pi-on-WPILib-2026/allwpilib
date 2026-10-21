/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/CAN.h"
#include "MauInternal.h"
#include "MauTime.h"
#include <VMXCAN.h>
#include <cstring>
#include "CANInternal.h"

namespace hal {
    namespace init {
    	static VMXCANReceiveStreamHandle blackboardStreamHandle = MAU_HAL_BLACKBOARD_STREAM_HANDLE;
    	static uint32_t defaultReceiveStreamMaxMessages = 100;
    	static bool blackboardInitialized = false;
        void InitializeCAN() {

        	/*
			 * CONFIGURE CAN; receive and display data; open a single stream configured
			 * to receive all Extended CAN ID (29-bit messages).
			 */

        	VMXErrorCode vmxerr;
			if (!mau::vmxCAN->OpenReceiveStream(blackboardStreamHandle, 0x0, 0x0, defaultReceiveStreamMaxMessages, &vmxerr)) {
				std::printf("Failed to open default CAN Receive Stream.  CAN Initialization Failed!\n");
				return;
			} else {
				if (!mau::vmxCAN->EnableReceiveStreamBlackboard(blackboardStreamHandle, true, &vmxerr)) {
					std::printf("Failed to enable default CAN Reception");
				} else {
					blackboardInitialized = true;
				}
			}
			/* Flush Rx/Tx fifo not necessary if invoking reset above. */
			if (!mau::vmxCAN->FlushRxFIFO(&vmxerr)) {
				printf("Error Flushing CAN RX FIFO.\n");
			} else {
#if 0
				printf("Flushed CAN RX FIFO\n");
#endif
			}

			if (!mau::vmxCAN->FlushTxFIFO(&vmxerr)) {
				printf("Error Flushing CAN TX FIFO.\n");
			} else {
#if 0
				printf("Flushed CAN TX FIFO\n");
#endif
			}

			if (!mau::vmxCAN->SetMode(VMXCAN::VMXCAN_NORMAL, &vmxerr)) {
				printf("Error setting CAN Mode to NORMAL\n");
			} else {
				printf("Set VMX CAN Mode to NORMAL.\n");
			}
			/* It's recommended to delay 20 Milliseconds after transitioning modes -
			 * to allow the CAN circuitry to stabilize; otherwise, sometimes
			 * there will be errors transmitting data during this period.
			 */
			mau::vmxTime->DelayMilliseconds(10);
        }
    }
}

extern "C" {

    void HAL_CAN_SendMessage(uint32_t messageID, const uint8_t* data, uint8_t dataSize, int32_t periodMs, int32_t* status) {
	if (!mau::vmxCAN) {
		printf("HAL_CAN_SendMessage() invoked when mau::vmxCAN is NULL!\n");
		return;
	}
	if (status) *status = 0;

    	VMXCANMessage message;
	memset(&message,0,sizeof(message));
	message.messageID = messageID;
	message.setData(data, dataSize);
	message.dataSize = dataSize;
    	if (mau::vmxCAN->SendMessage(message, periodMs, status)) {
#if 0
		if (messageID ==0x2052601) {
			printf("HAL_CAN_SendMessage success - MessageID 0x%x, size:  %d, %02X %02X %02X %02X %02X %02X %02X %02x - periodMs:  %d\n",
				messageID, dataSize, message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5], message.data[6], message.data[7], periodMs);
		}
#endif
	} else {
#if 0
		if (messageID ==0x2052601) {
			printf("HAL_CAN_SendMessage FAIL - MessageID 0x%x, size:  %d, %02X %02X %02X %02X %02X %02X %02X %02x - periodMs:  %d, status:  %d\n",
				messageID, dataSize, message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5], message.data[6], message.data[7], periodMs, *status);
		}
#endif
	}
    }

    // timeStamp represents the system timestamp (i.e., "FPGA Timestamp") converted to milliseconds.
    // Returning a status == 0 indicates a fresh message has been received.
    // messageID is a pointer to the CAN Message ID to return;
    // In theory, any message ANDed with the messageIDMask that matches should be returned
    // In practice, the WPI Library always invokes this function with an "all bits set mask" (0x1FFFFFFF)
    // TODO:  Potentially add support for masks with less bits set, and therefore the possibility that
    // this function may modify the messageID (to reflect the exact message ID returned).
    void HAL_CAN_ReceiveMessage(uint32_t* messageID, uint32_t messageIDMask, uint8_t* data, uint8_t* dataSize,
                                uint32_t* timeStamp, int32_t* status) {
	if (!mau::vmxCAN) {
		printf("HAL_CAN_ReceiveMessage() invoked when mau::vmxCAN is NULL!\n");
		return;
	}
		VMXCANTimestampedMessage blackboard_msg;
		uint64_t sys_timestamp;
		bool already_retrieved;
		if (!hal::init::blackboardInitialized) {
			*status = VMXERR_CAN_INVALID_RECEIVE_STREAM_HANDLE;
			return;
		}
		if (status) *status = 0;
		if (mau::vmxCAN->GetBlackboardEntry(hal::init::blackboardStreamHandle, *messageID, blackboard_msg, sys_timestamp, already_retrieved, status)) {
			if (*status != VMXERR_CAN_BLACKBOARD_ENTRY_NOT_PRESENT) {
				if (!already_retrieved) {
					*status = 0;
				} else {
					*status = 1; // A non-zero status indicates message has already been retrieved (TODO:  Make this a VMX Error code?)
				}
				*messageID = blackboard_msg.messageID;
				std::memcpy(data, blackboard_msg.data, blackboard_msg.dataSize);
				*dataSize = blackboard_msg.dataSize;
				uint32_t timestamp_milliseconds = static_cast<uint32_t>(blackboard_msg.sysTimeStampUS / 1000);
				*timeStamp = timestamp_milliseconds;
			}
		}
    }

    void HAL_CAN_OpenStreamSession(uint32_t* sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages,
                                   int32_t* status) {
	if (!mau::vmxCAN) {
		printf("HAL_CAN_OpenStreamSession() invoked when mau::vmxCAN is NULL!\n");
		return;
	}
    	VMXCANReceiveStreamHandle handle;
    	if (status) *status = 0;
	if (mau::vmxCAN->OpenReceiveStream(handle, messageID, messageIDMask, maxMessages, status)) {
    		*sessionHandle = handle;
    	}
    }

    void HAL_CAN_CloseStreamSession(uint32_t sessionHandle) {
	if (!mau::vmxCAN) {		
		printf("HAL_CAN_CloseStreamSession() invoked when mau::vmxCAN is NULL!\n");
		return;
	}
    	VMXErrorCode error;
    	mau::vmxCAN->CloseReceiveStream(sessionHandle, &error);
    }

#define NUM_VMXCAN_MESSAGES_IN_BATCH 10  // TODO: Tune this based on usage patterns.

    void HAL_CAN_ReadStreamSession(uint32_t sessionHandle, struct HAL_CANStreamMessage* messages, uint32_t messagesToRead,
                                   uint32_t* messagesRead, int32_t* status) {
	if (!mau::vmxCAN) {
		printf("HAL_CAN_ReadStreamSession() invoked when mau::vmxCAN is NULL!\n");
		return;
	}
    	VMXCANTimestampedMessage vmxMessages[NUM_VMXCAN_MESSAGES_IN_BATCH];
    	uint32_t remainingMessagesToRead = messagesToRead;
    	uint32_t currCANStreamMessageIndex = 0;
	if (status) *status = 0;
    	while (remainingMessagesToRead > 0) {
    		uint32_t currMessagesToRead =
    				(remainingMessagesToRead > NUM_VMXCAN_MESSAGES_IN_BATCH) ?
    						NUM_VMXCAN_MESSAGES_IN_BATCH : remainingMessagesToRead;
    		remainingMessagesToRead -= currMessagesToRead;
    		uint32_t currMessagesRead = 0;
    	   	if (!mau::vmxCAN->ReadReceiveStream(sessionHandle, vmxMessages, currMessagesToRead, currMessagesRead, status)) {
    	   		break;
    	   	}
    	   	if (currMessagesRead > 0) {
    	   		for ( uint32_t i = 0; i < currMessagesRead; i++) {
    	   			messages[currCANStreamMessageIndex].messageID = vmxMessages[i].messageID;
    	   			std::memcpy(messages[currCANStreamMessageIndex].data, vmxMessages[i].data,
    	   					(vmxMessages[i].dataSize > sizeof(vmxMessages[i].data)) ? sizeof(vmxMessages[i].data) : vmxMessages[i].dataSize);
				messages[currCANStreamMessageIndex].dataSize = vmxMessages[i].dataSize;
    	   			messages[currCANStreamMessageIndex].timeStamp = vmxMessages[i].sysTimeStampUS / 1000;
    	   			currCANStreamMessageIndex++;
    	   		}
    	   	}
    	}
    	*messagesRead = currCANStreamMessageIndex;
     }

    void HAL_CAN_GetCANStatus(float* percentBusUtilization, uint32_t* busOffCount, uint32_t* txFullCount,
                              uint32_t* receiveErrorCount, uint32_t* transmitErrorCount, int32_t* status) {
	if (!mau::vmxCAN) {
		printf("HAL_CAN_GetCANStatus() invoked when mau::vmxCAN is NULL!\n");
		return;
	}
	if (status) *status = 0;
    	VMXCANBusStatus busStatus;
    	if (mau::vmxCAN->GetCANBUSStatus(busStatus, status)) {
    		*percentBusUtilization = busStatus.percentBusUtilization;
    		*busOffCount = busStatus.busOffCount;
    		*txFullCount = busStatus.txFullCount;
    		*receiveErrorCount = busStatus.receiveErrorCount;
    		*transmitErrorCount = busStatus.transmitErrorCount;
    	}
    }
}
