/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedHandleResource.h"
#include "hal/Counter.h"
#include "PortsInternal.h"
#include <VMXResource.h>
#include <VMXResourceConfig.h>

namespace hal {

struct CounterResource {
	VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
			VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
	InputCaptureConfig vmx_config;
	HAL_Counter_Mode hal_counter_mode = HAL_Counter_Mode::HAL_Counter_kTwoPulse;
	HAL_DigitalHandle up_source_digital_handle = HAL_kInvalidHandle;
	HAL_DigitalHandle down_source_digital_handle = HAL_kInvalidHandle;
	int32_t index = 0; // index into resource handle table; this is also used as the externally-visibile "FPGA Index"
				   	   // it is not at all clear why the FPGA Index is made visible from the HAL API....
	InputCaptureConfig::CaptureChannelActiveEdge up_source_active_edge = InputCaptureConfig::CaptureChannelActiveEdge::ACTIVE_RISING;
	InputCaptureConfig::CaptureChannelActiveEdge down_source_active_edge = InputCaptureConfig::CaptureChannelActiveEdge::ACTIVE_FALLING;
	bool update_when_empty = false;
	bool reverse_direction = false;
	int32_t samplesToAverage = 1;
	HAL_EncoderHandle proxy_encoder_handle = HAL_kInvalidHandle; // if not invalid, this counter is implemented using a VMX-pi Encoder Resource
};

extern LimitedHandleResource<HAL_CounterHandle, CounterResource, kNumCounters,
                             HAL_HandleEnum::Counter>* counterHandles;

}  // namespace hal
