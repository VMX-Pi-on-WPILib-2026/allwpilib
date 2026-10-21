/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

#include <memory>

#include "hal/AnalogTrigger.h"
#include "hal/Ports.h"
#include "hal/Types.h"
#include "hal/handles/HandlesInternal.h"
#include "PortsInternal.h"
#include "MauInternal.h"
#include "hal/handles/LimitedHandleResource.h"

namespace hal {

struct AnalogTriggerResource {
	VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(VMXResourceType::Undefined,INVALID_VMX_RESOURCE_INDEX);
	AnalogTriggerConfig vmx_config;
	HAL_AnalogInputHandle analogInputHandle = HAL_kInvalidHandle;
	int16_t index = hal::InvalidHandleIndex;
	HAL_Bool trigState = false;
};

constexpr HAL_DigitalHandle kMaxValidAnalogTriggerIndex = kNumAnalogTriggers-1;

extern LimitedHandleResource<HAL_AnalogTriggerHandle, AnalogTriggerResource,
kNumAnalogTriggers, HAL_HandleEnum::AnalogTrigger>*
analogTriggerHandles;

}  // namespace hal
