/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "AnalogInternal.h"

#include "hal/AnalogInput.h"
#include "PortsInternal.h"
#include "MauInternal.h"

#define HANDLE_VERSION 0

namespace hal {
    IndexedHandleResource<HAL_AnalogInputHandle, hal::AnalogPort, kNumAnalogInputs,
            HAL_HandleEnum::AnalogInput> *analogInputHandles;
}

namespace hal {
    namespace init {
        void InitializeAnalogInternal() {
            static IndexedHandleResource<HAL_AnalogInputHandle, hal::AnalogPort,
                    kNumAnalogInputs, HAL_HandleEnum::AnalogInput>
                    aiH;
            analogInputHandles = &aiH;
        }
    }

    HAL_AnalogInputHandle getAnalogInputHandleForVMXChannelIndex(VMXChannelIndex index) {
    	constexpr int32_t kFirstAnalogInChannelIndex = kNumFlexDIOChannels + kNumHiCurrDIOChannels;
    	constexpr int32_t kLastAnalogInChannelIndex = kFirstAnalogInChannelIndex + kNumAnalogInputs - 1;
    	if (index < kFirstAnalogInChannelIndex) {
    		return HAL_kInvalidHandle;
    	} else if ((index >= kFirstAnalogInChannelIndex) && (index <= kLastAnalogInChannelIndex)) {
    		return createHandle(index - kFirstAnalogInChannelIndex, HAL_HandleEnum::AnalogInput, HANDLE_VERSION);
    	} else {
    		return HAL_kInvalidHandle;
    	}
    }

    std::shared_ptr<AnalogPort> allocateAnalogInputPort(HAL_AnalogInputHandle anInHandle, int32_t *status)
    {
    	HAL_AnalogInputHandle handle;
    	analogInputHandles->Allocate(anInHandle, &handle, status);

    	if (HAL_kInvalidHandle == handle) {
    		return nullptr;
    	}

    	auto port = analogInputHandles->Get(handle);
    	if (port == nullptr) {  // would only occur on thread issue.
    		*status = HAL_HANDLE_ERROR;
    		return nullptr;
    	}

    	return port;
    }

    std::shared_ptr<AnalogPort> allocateAnalogInputHandleAndInitializedPort(int32_t wpi_channel, HAL_AnalogInputHandle& anInHandle, int32_t *status)
    {
    	VMXChannelInfo vmx_chan_info;
    	VMXChannelIndex vmx_chan_index = getVMXChannelIndexAndVMXChannelInfo(HAL_HandleEnum::AnalogInput, wpi_channel, vmx_chan_info, status);
    	if (INVALID_VMX_CHANNEL_INDEX == vmx_chan_index) {
    		return nullptr;
    	}

    	anInHandle = getAnalogInputHandleForVMXChannelIndex(vmx_chan_index);
    	if (anInHandle == HAL_kInvalidHandle) {
    		return nullptr;
    	}

    	auto port = allocateAnalogInputPort(anInHandle, status);
    	if (port != nullptr) {
    		port->vmx_chan_info = vmx_chan_info;
    		port->channel = wpi_channel;
    	}

    	return port;
    }

    bool AllocateVMXAnalogIn(std::shared_ptr<AnalogPort> anPort, int32_t* status) {
    	/* Use the default accumulator configuration */
    	if (!mau::vmxIO->ChannelSupportsCapability(anPort->vmx_chan_info.index, VMXChannelCapability::AccumulatorInput)) {
    		*status = VMXERR_IO_INVALID_CHANNEL_TYPE;
    		return false;
    	}

    	if (!mau::vmxIO->ActivateSinglechannelResource(anPort->vmx_chan_info, &anPort->vmx_config, anPort->vmx_res_handle, status)) {
    		return false;
    	}

    	return true;
    }

    /* Can be used to deallocate either a DigitalIO or a PWMGenerator VMXPi Resource. */
    void DeallocateVMXAnalogIn(std::shared_ptr<AnalogPort> anPort, bool& isActive) {
    	isActive = false;
    	mau::vmxIO->IsResourceActive(anPort->vmx_res_handle, isActive, mau::vmxError);
    	if (isActive) {
    		mau::vmxIO->DeallocateResource(anPort->vmx_res_handle, mau::vmxError);
    	}
    	anPort->vmx_res_handle = 0;
    }



}
