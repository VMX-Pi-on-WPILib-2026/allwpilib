/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Ports.h"

#include "PortsInternal.h"
#include "MauInternal.h"
#include <VMXIO.h>
#include <VMXChannel.h>

using namespace hal;
using namespace mau;

namespace hal {
    namespace init {
        void InitializePorts() {}
    }
}

extern "C" {
    int32_t HAL_GetNumAccumulators(void) { return kNumAccumulators; }
    int32_t HAL_GetNumAnalogTriggers(void) { return kNumAnalogTriggers; }
    int32_t HAL_GetNumAnalogInputs(void) { return kNumAnalogInputs; }
    int32_t HAL_GetNumAnalogOutputs(void) { return kNumAnalogOutputs; }
    int32_t HAL_GetNumCounters(void) { return kNumCounters; }
    int32_t HAL_GetNumDigitalHeaders(void) { return kNumDigitalHeaders; }
    int32_t HAL_GetNumPWMHeaders(void) { return kNumPWMHeaders; }
    int32_t HAL_GetNumDigitalChannels(void) { return kNumDigitalChannels; }
    int32_t HAL_GetNumPWMChannels(void) { return kNumPWMChannels; }
    int32_t HAL_GetNumDigitalPWMOutputs(void) { return kNumDigitalPWMOutputs; }
    int32_t HAL_GetNumEncoders(void) { return kNumEncoders; }
    int32_t HAL_GetNumInterrupts(void) { return kNumInterrupts; }
    int32_t HAL_GetNumRelayChannels(void) { return kNumRelayChannels; }
    int32_t HAL_GetNumRelayHeaders(void) { return kNumRelayHeaders; }
    int32_t HAL_GetNumCTREPCMModules(void) { return kNumCTREPCMModules; }
    int32_t HAL_GetNumCTRESolenoidChannels(void) { return kNumCTRESolenoidChannels; }
    int32_t HAL_GetNumCTREPDPModules(void) { return kNumCTREPDPModules; }
    int32_t HAL_GetNumCTREPDPChannels(void) { return kNumCTREPDPChannels; }
    int32_t HAL_GetNumREVPDHModules(void) { return kNumREVPDHModules; }
    int32_t HAL_GetNumREVPDHChannels(void) { return kNumREVPDHChannels; }
    int32_t HAL_GetNumREVPHModules(void) { return kNumREVPHModules; }
    int32_t HAL_GetNumREVPHChannels(void) { return kNumREVPHChannels; }
    int32_t HAL_GetNumDutyCycles(void) { return kNumDutyCycles; }
    int32_t HAL_GetNumAddressableLEDs(void) { return kNumAddressableLEDs; }
}
