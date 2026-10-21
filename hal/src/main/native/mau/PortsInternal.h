/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>
#include "hal/handles/HandlesInternal.h"
#include <VMXChannel.h>

/* NOTE:  The VMX-pi HAL provides differing numbers of various channel and resource types than athena. */

/* NOTE:  These values need to be synchronized with the Mau Channel Map.  It is possible in the future */
/*        that these values will be dynamic (based upon the current Mau Channel Map.                   */

namespace hal {

	constexpr int32_t kNumDigitalHeaders = 10;				/* FlexDIO [0-9] */
	constexpr int32_t kNumDigitalExtraHeaders = 2;			/* FlexDIO [10-11] */
	constexpr int32_t kNumFlexDIOChannels =
						kNumDigitalHeaders +
						kNumDigitalExtraHeaders;

	constexpr int32_t kNumHiCurrDIOChannels = 10;
	constexpr int32_t kNumPWMHeaders = kNumHiCurrDIOChannels;

	constexpr int32_t kNumCommDIOInputCapableChannels = 4;	/* I2C SDA/SCL, UART RX, SPI MISO */
	constexpr int32_t kNumCommDIOOutputCapableChannels = 6;	/* I2C SDA/SCL, UART TX, SPI CLK/CS/MOSI */
	constexpr int32_t kNumCommDIOChannels = 8;				/* I2C SDA/SCL, UART RX/TX, SPI CLK/CS/MOSI/MISO */

    constexpr int32_t kNumAccumulators = 2;					/* Accumulators implemented in VMX-pi firmware */
    constexpr int32_t kNumAnalogTriggers = 4;				/* Triggers implemented in VMX-pi firmware */
    constexpr int32_t kNumAnalogInputs = 4;					/* VMX-pi Analog Input Header */ /* Ref'd by wpilibc/j/SensorUtil */
    constexpr int32_t kNumAnalogOutputs = 0;				/* Not supported on VMX-pi */ /* Ref'd by wpilibc/j/SensorUtil */
    constexpr int32_t kNumCounters = 6;						/* VMX-pi InputCapture Resources (Future: extend to RPI DIOs) */
    constexpr int32_t kNumEncoders = 5;						/* VMX-pi Encoder Resources (Future:  extend to RPI DIOs) */
    constexpr int32_t kNumDigitalChannels =
    					kNumDigitalHeaders +
						kNumDigitalExtraHeaders +
						kNumPWMHeaders +
						kNumCommDIOChannels; 				/* 30.  Ref'd by wpilibc/j/SensorUtil */
    constexpr int32_t kNumPWMChannels =
    					kNumPWMHeaders +
    					kNumDigitalHeaders;					/* 20.  Ref'd by wpilibc/j/SensorUtil */
    constexpr int32_t kNumDigitalPWMOutputs =
    					kNumDigitalExtraHeaders +
						kNumCommDIOOutputCapableChannels;	/* 8.  PWM Outputs not including in kNumPWMChannels */
    constexpr int32_t kNumInterrupts = 8;					/* 8.  Note that VMX-pi supports 30 interrupts, however this is    */
    														/*     artificially limited to 8 here to match the limit which is  */
    													    /*     imposed by the existing HAL API.                            */
    														/*     (see discussion on "mask" in Interrupts.cpp)                */
    constexpr int32_t kNumVMXPiInterrupts =
    					kNumFlexDIOChannels +
						kNumPWMHeaders +
						kNumAnalogTriggers +
						kNumCommDIOInputCapableChannels;	/* 30. This respresents the underlying number of VMX-pi interrupts. */
    constexpr int32_t kNumRelayChannels = 8;				/* Note:  NOT Dedicated (shared w/PWM Headers) (Dynamic - may be 0 if Jumper set to 'input' */
    constexpr int32_t kNumRelayHeaders = kNumRelayChannels / 2; /* Accessors:  wpilibj/SensorUtil */

    /* The following are limited by CAN addressing, */
    /* rather than by VMX IO/Resource constraints.  */
    constexpr int32_t kNumCTREPCMModules = 63;
    constexpr int32_t kNumCTRESolenoidChannels = 8;
    constexpr int32_t kNumCTREPDPModules = 63;
    constexpr int32_t kNumCTREPDPChannels = 16;
    constexpr int32_t kNumREVPDHModules = 63;
    constexpr int32_t kNumREVPDHChannels = 24;
    constexpr int32_t kNumPDSimModules = kNumREVPDHModules;
    constexpr int32_t kNumPDSimChannels = kNumREVPDHChannels;
    constexpr int32_t kNumREVPHModules = 63;
    constexpr int32_t kNumREVPHChannels = 16;
    constexpr int32_t kNumCanTalons = 63;
    constexpr int32_t kNumDutyCycles = kNumDigitalChannels; // TODO:  Is this correct for VMX-pi HAL?
    constexpr int32_t kNumAddressableLEDs = 1;

    enum class HAL_ChannelAddressDomain {
    	/* NOTE:  This enumeration does not contain Relay Channels */
    	DIO,		/* kNumDigitalChannels */
		PWM,		/* kNumPWMChannels */
		AnalogInput,/* kNumAnalogInputs */
    };

    constexpr VMXChannelIndex kMaxValidVMXChannelIndex = kNumDigitalChannels + kNumAnalogInputs - 1;

    bool isWPILibChannelValid(HAL_ChannelAddressDomain channelDomain, int32_t wpiLibDigitalChannel);
    VMXChannelIndex getVMXChannelIndexAndVMXChannelInfo(HAL_HandleEnum handleType, int32_t wpiLibChannel, VMXChannelInfo& vmx_chan_info, int32_t *status);
    VMXChannelIndex getVMXChannelIndexForWPILibChannel(HAL_ChannelAddressDomain channelDomain, int32_t wpiLibDigitalChannel);
    VMXChannelIndex getVMXChannelIndexForWpiLibRelay(int32_t wpiLibRelayChannel, bool fwd);
}
