/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Relay.h"
#include "hal/DIO.h"

#include "hal/handles/IndexedHandleResource.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "MauErrors.h"

using namespace hal;

// WPI Library Relay "channels" represent pairs of two digital channels,
// a "forward digital channel", and a "reverse digital channel".
//
// The HAL manages RelayHandles, which represent each "relay" digital channel
// On VMX-pi, there are no dedicated relay digital channels, as there are on
// the reference implementation.  Rather, relay digital channels are mapped onto
// a subset of the digital channels.
//
// Relays channels are clasically used to control the IFI "Spike" Relay,
// comprised of a H-bridge with control signals carried over
// 3-wire cables (GND, Fwd, Rev).  The Fwd and Rev signals are digital, and
// per the Spike Relay specifications require a High signal ranging from 3-12V @4mA
// Therefore, the Relay channels can be electrically supported on any of the
// VMXPi Digital Outputs channels.
//
// The forward relay channel indexes start from 0;
// The reverse relay channel indexes start from kNumRelayHeaders;

namespace hal {
    struct Relay {
        uint32_t channel;	/* WPI Library Channel Number (in Relay Channel Address Domain) */
        bool fwd = true;	/* True if channel is "Forward" channel; false if "Reverse" */
        HAL_DigitalHandle digital_channel_handle = HAL_kInvalidHandle;
    };
    static IndexedHandleResource<HAL_RelayHandle, Relay, kNumRelayChannels,
            HAL_HandleEnum::Relay>* relayHandles = 0;

    namespace init {
        void InitializeRelay() {
            static IndexedHandleResource<HAL_RelayHandle, Relay, kNumRelayChannels,
                    HAL_HandleEnum::Relay>
                    rH;
            relayHandles = &rH;
        }
    }  // namespace init
}  // namespace hal

extern "C" {
    HAL_RelayHandle HAL_InitializeRelayPort(HAL_PortHandle portHandle, HAL_Bool fwd, const char* allocationLocation, int32_t* status) {
        hal::init::CheckInit();
        if (*status != 0) return HAL_kInvalidHandle;

        int16_t wpi_relay_channel = getPortHandleChannel(portHandle);
        if (wpi_relay_channel == InvalidHandleIndex) {
            *status = PARAMETER_OUT_OF_RANGE;
            return HAL_kInvalidHandle;
        }

        // Calculate the corresponding VMXPi Channel Index
        VMXChannelIndex vmx_chan_index = getVMXChannelIndexForWpiLibRelay(wpi_relay_channel, fwd);
        if (vmx_chan_index == INVALID_VMX_CHANNEL_INDEX) {
        	*status = MAU_CHANNEL_MAP_ERROR;
            return HAL_kInvalidHandle;
        }

        // Since VMX-pi relays reuse Digital channels, calculate the
        // corresponding digital handle index for this vmx channel
        int32_t corresponding_wpi_dio_channel;
        bool found = false;
        for (int32_t wpi_dio_channel = 0; wpi_dio_channel < kNumDigitalChannels; wpi_dio_channel++) {
        	if (getVMXChannelIndexForWPILibChannel(HAL_ChannelAddressDomain::DIO,wpi_dio_channel) == vmx_chan_index) {
        		corresponding_wpi_dio_channel = wpi_dio_channel;
        		found = true;
        		break;
        	}
        }
        if (!found) {
        	*status = MAU_CHANNEL_MAP_ERROR;
            return HAL_kInvalidHandle;
        }

        if (!fwd) wpi_relay_channel += kNumRelayHeaders;  // add 4 to reverse channels

        HAL_RelayHandle relay_handle;
        relayHandles->Allocate(wpi_relay_channel, &relay_handle, status);

        if (*status != 0) {
            return HAL_kInvalidHandle;  // failed to allocate. Pass error back.
        }

        auto port = relayHandles->Get(relay_handle);
        if (port == nullptr) {  // would only occur on thread issue.
            *status = HAL_HANDLE_ERROR;
            return HAL_kInvalidHandle;
        }

        HAL_Bool input = false;
        uint8_t module = 1;
        HAL_DigitalHandle digital_channel_handle =
        		HAL_InitializeDIOPort(createPortHandle(static_cast<uint8_t>(corresponding_wpi_dio_channel), module), input, nullptr, status);
        if (digital_channel_handle == HAL_kInvalidHandle) {
            relayHandles->Free(relay_handle);
            return HAL_kInvalidHandle;
        }

        port->channel = corresponding_wpi_dio_channel;
        port->fwd = fwd;
        port->digital_channel_handle = digital_channel_handle;

        return relay_handle;
    }

    void HAL_FreeRelayPort(HAL_RelayHandle relayPortHandle) {
        auto port = relayHandles->Get(relayPortHandle);
        if (port == nullptr) return;
        HAL_FreeDIOPort(port->digital_channel_handle);
        relayHandles->Free(relayPortHandle);
    }

    HAL_Bool HAL_CheckRelayChannel(int32_t channel) {
        return channel < kNumRelayHeaders && channel >= 0;
    }

    void HAL_SetRelay(HAL_RelayHandle relayPortHandle, HAL_Bool on,
                      int32_t* status) {
        auto port = relayHandles->Get(relayPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }
        HAL_SetDIO(port->digital_channel_handle, on, status);
    }

    HAL_Bool HAL_GetRelay(HAL_RelayHandle relayPortHandle, int32_t* status) {
        auto port = relayHandles->Get(relayPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return false;
        }
        return HAL_GetDIO(port->digital_channel_handle, status);
    }
}
