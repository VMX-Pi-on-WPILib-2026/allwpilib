/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DigitalInternal.h"

#include "ConstantsInternal.h"
#include "hal/AnalogTrigger.h"
#include "hal/HAL.h"
#include "hal/Ports.h"
#include "PortsInternal.h"

#define HANDLE_VERSION 0

namespace hal {

DigitalHandleResource<HAL_DigitalHandle, DigitalPort, kNumDigitalChannels>*
    digitalChannelHandles;

namespace init {
void InitializeDigitalInternal() {
  static DigitalHandleResource<HAL_DigitalHandle, DigitalPort, kNumDigitalChannels>
      dcH;
  digitalChannelHandles = &dcH;
}
}  // namespace init

/* returns a contiguous, 0-based index for the (digital) VMXChannelIndex */
HAL_DigitalHandle getDigitalHandleForVMXChannelIndex(VMXChannelIndex index, HAL_HandleEnum handleType) {
	constexpr int32_t kFirstAnalogInChannelIndex = (kNumFlexDIOChannels + kNumHiCurrDIOChannels);
	constexpr int32_t kFirstCommDIOChannelIndex = (kNumFlexDIOChannels + kNumHiCurrDIOChannels + kNumAnalogInputs);
	constexpr int32_t kLastCommDIOChannelIndex = (kFirstCommDIOChannelIndex + kNumCommDIOChannels) - 1;
	if (index < kFirstAnalogInChannelIndex) {
		return createHandle(index, handleType, HANDLE_VERSION);
	} else if ((index >= kFirstCommDIOChannelIndex) && (index <= kLastCommDIOChannelIndex)) {
		return createHandle(index - kNumAnalogInputs, handleType, HANDLE_VERSION);
	} else {
		return HAL_kInvalidHandle;
	}
}

HAL_DigitalHandle getDigitalHandleAndVMXChannelInfo(HAL_HandleEnum handleType, int32_t wpiLibPwmChannel, VMXChannelInfo& info, int32_t *status)
{
	VMXChannelIndex vmx_chan_index = getVMXChannelIndexAndVMXChannelInfo(handleType, wpiLibPwmChannel, info, status);
	if (INVALID_VMX_CHANNEL_INDEX == vmx_chan_index) {
		return HAL_kInvalidHandle;
	}
	return getDigitalHandleForVMXChannelIndex(vmx_chan_index, handleType);
}

std::shared_ptr<DigitalPort> allocateDigitalPort(HAL_DigitalHandle digHandle, HAL_HandleEnum handleType, int32_t *status)
{
	HAL_DigitalHandle handle;
	digitalChannelHandles->Allocate(digHandle, handleType, &handle, status);

	if (HAL_kInvalidHandle == handle) {
		return nullptr;
	}

	auto port = digitalChannelHandles->Get(handle, handleType);
	if (port == nullptr) {  // would only occur on thread issue.
		*status = HAL_HANDLE_ERROR;
		return nullptr;
	}

	return port;
}

std::shared_ptr<DigitalPort> allocateDigitalHandleAndInitializedPort(HAL_HandleEnum type, int32_t wpi_channel, HAL_DigitalHandle& digHandle, int32_t *status)
{
	VMXChannelInfo vmx_chan_info;
	VMXChannelIndex vmx_chan_index = getVMXChannelIndexAndVMXChannelInfo(type, wpi_channel, vmx_chan_info, status);
	if (INVALID_VMX_CHANNEL_INDEX == vmx_chan_index) {
		return nullptr;
	}

	digHandle = getDigitalHandleForVMXChannelIndex(vmx_chan_index, type);
	if (digHandle == HAL_kInvalidHandle) {
		return nullptr;
	}

	auto port = allocateDigitalPort(digHandle, type, status);
	if (port != nullptr) {
		port->vmx_chan_info = vmx_chan_info;
		port->channel = wpi_channel;
	}

	return port;
}

/**
 * Map DIO channel numbers from their physical number (10 to 26) to their
 * position in the bit field.
 */
int32_t remapMXPChannel(int32_t channel) { return channel - 10; }

int32_t remapMXPPWMChannel(int32_t channel) {
  if (channel < 14) {
    return channel - 10;  // first block of 4 pwms (MXP 0-3)
  } else {
    return channel - 6;  // block of PWMs after SPI
  }
}

/**
 * remap the digital source channel and set the module.
 * If it's an analog trigger, determine the module from the high order routing
 * channel else do normal digital input remapping based on channel number
 * (MXP)
 */
bool remapDigitalSource(HAL_Handle digitalSourceHandle,
                        HAL_AnalogTriggerType analogTriggerType,
                        uint8_t& channel, uint8_t& module,
                        bool& analogTrigger) {
  if (isHandleType(digitalSourceHandle, HAL_HandleEnum::AnalogTrigger)) {
    // If handle passed, wpiIndex is not negative
    int32_t index = getHandleIndex(digitalSourceHandle);
    channel = (index << 2) + analogTriggerType;
    module = channel >> 4;
    analogTrigger = true;
    return true;
  } else if (isHandleType(digitalSourceHandle, HAL_HandleEnum::DIO)) {
    int32_t index = getHandleIndex(digitalSourceHandle);
    if (index >= kNumDigitalHeaders) {
      channel = remapMXPChannel(index);
      module = 1;
    } else {
      channel = index;
      module = 0;
    }
    analogTrigger = false;
    return true;
  } else {
    return false;
  }
}

int32_t GetDigitalInputChannel(HAL_DigitalHandle handle, int32_t* status) {
  auto digital = digitalChannelHandles->Get(handle, HAL_HandleEnum::DIO);
  if (digital == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return -1;
  }

  return digital->channel;
}
}  // namespace hal
