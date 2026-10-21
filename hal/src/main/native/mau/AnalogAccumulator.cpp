/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <algorithm>

#include "hal/AnalogAccumulator.h"

#include "AnalogInternal.h"
#include "hal/handles/HandlesInternal.h"
#include "MauInternal.h"
#include "MauTime.h"
#include <VMXIO.h>
#include <VMXErrors.h>
#include <VMXResource.h>

using namespace hal;
using namespace mau;

namespace hal {
    namespace init {
        void InitializeAnalogAccumulator() {}
    }
}

extern "C" {
    HAL_Bool HAL_IsAccumulatorChannel(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
		auto port = analogInputHandles->Get(analogPortHandle);
		if (port == nullptr) {
			*status = HAL_HANDLE_ERROR;
			return false;
		}
		for (int32_t i = 0; i < kNumAccumulators; i++) {
			if (port->channel == kAccumulatorChannels[i]) return true;
		}
		return false;
    }

    void HAL_InitAccumulator(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
           *status = HAL_HANDLE_ERROR;
            return;
        }

        if (!HAL_IsAccumulatorChannel(analogPortHandle, status)) {
            *status = HAL_INVALID_ACCUMULATOR_CHANNEL;
            return;
        }

        // Deallocate the existing VMX Resource
        bool active;
        DeallocateVMXAnalogIn(port, active);

        // Modify configuration to indicate the accumulator should be used now

        port->vmx_config.SetEnableAccumulationCounter(true);

        // Reallocate the existing VMX Resource

        AllocateVMXAnalogIn(port, status);
    }

    void HAL_ResetAccumulator(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }
        mau::vmxIO->Accumulator_Counter_Reset(port->vmx_res_handle, status);
    }

    /**
     * Set the center value of the accumulator.
     *
     * The center value is subtracted from each A/D value before it is added to the
     * accumulator. This is used for the center value of devices like gyros and
     * accelerometers to take the device offset into account when integrating.
     *
     * This center value is based on the output of the oversampled and averaged
     * source from the accumulator channel. Because of this, any non-zero
     * oversample bits will affect the size of the value for this field.
     */
    void HAL_SetAccumulatorCenter(HAL_AnalogInputHandle analogPortHandle, int32_t center, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

        if (!HAL_IsAccumulatorChannel(analogPortHandle, status)) {
            *status = HAL_INVALID_ACCUMULATOR_CHANNEL;
            return;
        }

        // Deallocate the existing VMX Resource
        bool active;
        DeallocateVMXAnalogIn(port, active);

        // Modify configuration to indicate the accumulator should be used now

        port->vmx_config.SetAccumulationCounterCenter(center);

        // Reallocate the existing VMX Resource
        if (active) {
        	AllocateVMXAnalogIn(port, status);
        }

     }

    /**
     * Set the accumulator's deadband.
     */
    void HAL_SetAccumulatorDeadband(HAL_AnalogInputHandle analogPortHandle, int32_t deadband, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

        // Deallocate the existing VMX Resource
        bool active;
        DeallocateVMXAnalogIn(port, active);

        // Modify configuration to indicate the accumulator should be used now

        port->vmx_config.SetAccumulationCounterDeadband(deadband);

        // Reallocate the existing VMX Resource
        if (active) {
        	AllocateVMXAnalogIn(port, status);
        }
    }

    /**
     * Read the accumulated value.
     *
     * Read the value that has been accumulating.
     * The accumulator is attached after the oversample and average engine.
     *
     * @return The 64-bit value accumulated since the last Reset().
     */
    int64_t HAL_GetAccumulatorValue(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
        int64_t accumulator_counter_value = 0;
        int64_t accumulator_counter_count = 0;
        HAL_GetAccumulatorOutput(analogPortHandle, &accumulator_counter_value, &accumulator_counter_count, status);
        return accumulator_counter_value;
    }

    /**
     * Read the number of accumulated values.
     *
     * Read the count of the accumulated values since the accumulator was last
     * Reset().
     *
     * @return The number of times samples from the channel were accumulated.
     */
    int64_t HAL_GetAccumulatorCount(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
        int64_t accumulator_counter_value = 0;
        int64_t accumulator_counter_count = 0;
        HAL_GetAccumulatorOutput(analogPortHandle, &accumulator_counter_value, &accumulator_counter_count, status);
        return accumulator_counter_count;
    }

    /**
     * Read the accumulated value and the number of accumulated values atomically.
     *
     * This function reads the value and count from the FPGA atomically.
     * This can be used for averaging.
     *
     * @param value Reference to the 64-bit accumulated output.
     * @param count Reference to the number of accumulation cycles.
     */
    void HAL_GetAccumulatorOutput(HAL_AnalogInputHandle analogPortHandle, int64_t* value, int64_t* count, int32_t* status) {
        auto port = analogInputHandles->Get(analogPortHandle);
        if (port == nullptr) {
            *status = HAL_HANDLE_ERROR;
            return;
        }

        int64_t accumulator_counter_value = 0;
        uint32_t accumulator_counter_count = 0;
        mau::vmxIO->Accumulator_Counter_GetValueAndCount(port->vmx_res_handle, accumulator_counter_value, accumulator_counter_count, status);
        *value = accumulator_counter_value;
        *count = static_cast<int64_t>(accumulator_counter_count);
     }
}
