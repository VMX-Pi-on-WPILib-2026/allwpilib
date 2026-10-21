/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/HAL.h"
#include "hal/Ports.h"
#include "PortsInternal.h"
#include "MauInternal.h"
#include "MauErrors.h"

namespace hal {

bool isWPILibChannelValid(HAL_ChannelAddressDomain channelDomain, int32_t wpiLibDigitalChannel)
{
	return (getVMXChannelIndexForWPILibChannel(channelDomain, wpiLibDigitalChannel) != INVALID_VMX_CHANNEL_INDEX);
}

VMXChannelIndex getVMXChannelIndexAndVMXChannelInfo(HAL_HandleEnum handleType, int32_t wpiLibChannel, VMXChannelInfo& vmx_chan_info, int32_t *status)
{
	vmx_chan_info = mau::GetChannelInfo(handleType, wpiLibChannel);
	if (!vmx_chan_info.IsValid()) {
		*status = MAU_CHANNEL_MAP_ERROR;
		return INVALID_VMX_CHANNEL_INDEX;
	}

	VMXChannelIndex vmx_chan_index = vmx_chan_info.index;
	if (vmx_chan_index == INVALID_VMX_CHANNEL_INDEX) {
		*status = MAU_CHANNEL_MAP_ERROR;
	} else {
		/* Verify the required Channel IO direction matches the HiCurrDIO input/output mode setting. */
		if (HAL_HandleEnum::PWM == handleType) {
			if (!(vmx_chan_info.capabilities & VMXChannelCapability::DigitalOutput)) {
				*status = MAU_HICURRDIO_OUTPUTMODE_NOT_ENABLED;
				vmx_chan_index = INVALID_VMX_CHANNEL_INDEX;
			}
		}
	}

	return vmx_chan_index;
}

/* returns a VMXChannelIndex given a WPI Library channel & type */
VMXChannelIndex getVMXChannelIndexForWPILibChannel(HAL_ChannelAddressDomain channelDomain, int32_t wpiLibChannel)
{
	VMXChannelIndex vmxpi_digital_channel = INVALID_VMX_CHANNEL_INDEX;
	switch (channelDomain) {
	case HAL_ChannelAddressDomain::PWM:
		if (wpiLibChannel < kNumHiCurrDIOChannels) {
			vmxpi_digital_channel = wpiLibChannel + kNumFlexDIOChannels;
		} else {
			vmxpi_digital_channel = wpiLibChannel - kNumHiCurrDIOChannels;
		}
		break;
	default:
	case HAL_ChannelAddressDomain::DIO:
		vmxpi_digital_channel = wpiLibChannel;
		break;
	case HAL_ChannelAddressDomain::AnalogInput:
		vmxpi_digital_channel = kNumFlexDIOChannels + kNumHiCurrDIOChannels + wpiLibChannel;
		break;
	}
	if (vmxpi_digital_channel > kMaxValidVMXChannelIndex) {
		return INVALID_VMX_CHANNEL_INDEX;
	} else {
		return vmxpi_digital_channel;
	}
}

VMXChannelIndex getVMXChannelIndexForWpiLibRelay(int32_t wpiLibRelayChannel, bool fwd)
{
	VMXChannelIndex vmxpi_digital_channel = INVALID_VMX_CHANNEL_INDEX;
	if ((wpiLibRelayChannel < kNumRelayHeaders) && (wpiLibRelayChannel >= 0)) {
		vmxpi_digital_channel = kNumFlexDIOChannels + (wpiLibRelayChannel * 2) + (fwd ? 0 : 1);
	}
	return vmxpi_digital_channel;
}

}  // namespace hal
