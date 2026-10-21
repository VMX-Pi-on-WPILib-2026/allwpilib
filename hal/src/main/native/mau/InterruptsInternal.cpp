/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "InterruptsInternal.h"
#include "AnalogInternal.h"
#include "AnalogTriggerInternal.h"
#include "DigitalInternal.h"
#include <VMXChannel.h>

namespace hal {

bool GetVMXInterruptResourceIndexForDigitalSourceHandle(HAL_Handle digitalSourceHandle, VMXResourceIndex& int_res_index, VMXChannelInfo& vmx_chan_info, int32_t* status)
{
	// Calculate index source vmx channel index & type (digital input or analog trigger)
	HAL_HandleEnum handleType = getHandleType(digitalSourceHandle);
	if (handleType == HAL_HandleEnum::DIO) {
        auto port = digitalChannelHandles->Get(static_cast<HAL_DigitalHandle>(digitalSourceHandle), HAL_HandleEnum::DIO);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }
        if (port->vmx_chan_info.index == INVALID_VMX_CHANNEL_INDEX) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }
        if (!(port->vmx_chan_info.capabilities & VMXChannelCapability::DigitalInput)) {
        	*status = MAU_DIO_NOT_INPUT_CAPABLE;
        	return false;
        }

        vmx_chan_info = port->vmx_chan_info;
        int_res_index = port->vmx_chan_info.index;
	} else if (handleType == HAL_HandleEnum::AnalogTrigger) {
        auto analogTriggerResource = analogTriggerHandles->Get(static_cast<HAL_AnalogTriggerHandle>(digitalSourceHandle));
        if (analogTriggerResource == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }
        if (INVALID_VMX_RESOURCE_HANDLE(analogTriggerResource->vmx_res_handle)) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }
        // Calculate VMX-pi interrupt resource index, which is the same as the analog trigger's analog input vmxchannel index.
        auto analogInputResource = analogInputHandles->Get(analogTriggerResource->analogInputHandle);
        if (analogInputResource == nullptr) {
        	*status = HAL_HANDLE_ERROR;
        	return false;
        }
        vmx_chan_info = analogInputResource->vmx_chan_info;
        int_res_index = analogInputResource->vmx_chan_info.index;
	} else {
		// Invalid index source digitalSourceHandle handle type
        *status = HAL_HANDLE_ERROR;
        return false;
	}

	return false;
}

}
