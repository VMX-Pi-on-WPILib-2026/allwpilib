/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/AnalogOutput.h"

#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/IndexedHandleResource.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "MauErrors.h"

using namespace hal;

namespace {
struct AnalogOutput {
	uint8_t channel;
};
}  // namespace

static IndexedHandleResource<HAL_AnalogOutputHandle, AnalogOutput,
		kNumAnalogOutputs, HAL_HandleEnum::AnalogOutput>* analogOutputHandles;

namespace hal {
namespace init {
void InitializeAnalogOutput() {
	static IndexedHandleResource<HAL_AnalogOutputHandle, AnalogOutput,
			kNumAnalogOutputs, HAL_HandleEnum::AnalogOutput> aoH;
	analogOutputHandles = &aoH;
}
}  // namespace init
}  // namespace hal

extern "C" {
HAL_AnalogOutputHandle HAL_InitializeAnalogOutputPort(HAL_PortHandle portHandle,
		const char* allocationLocation, int32_t* status) {
	hal::init::CheckInit();
	*status = MAU_ANALOG_OUTPUTS_UNSUPPORTED;
	return HAL_kInvalidHandle;
}

void HAL_FreeAnalogOutputPort(HAL_AnalogOutputHandle analogOutputHandle) {
}

HAL_Bool HAL_CheckAnalogOutputChannel(int32_t channel) {
	return channel < kNumAnalogOutputs && channel >= 0;
}

void HAL_SetAnalogOutput(HAL_AnalogOutputHandle analogOutputHandle,
		double voltage, int32_t* status) {
}

double HAL_GetAnalogOutput(HAL_AnalogOutputHandle analogOutputHandle,
		int32_t* status) {
	return 0.0;
}
}  // extern "C"
