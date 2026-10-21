/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Interrupts.h"

#include <chrono>
#include <memory>

#include <wpi/condition_variable.h>
#include <wpi/mutex.h>

#include "DigitalInternal.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedHandleResource.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "InterruptsInternal.h"
#include <VMXChannel.h>
#include <VMXResource.h>
#include <VMXIO.h>
#include <atomic>

using namespace hal;

static constexpr int64_t kHalWaitResultInterruptTimeout = 0x0;

// Since 2022 the HAL only supports synchronous (waiting) interrupts. Edges
// reported by the VMX-pi interrupt thread are accumulated here until a
// waiter consumes them. Mask format (unchanged from the 2020 port):
//   Bits 0-7:  rising edge,  bit position = HAL interrupt index
//   Bits 8-15: falling edge, bit position = HAL interrupt index + 8
static wpi::mutex waitMutex;
static wpi::condition_variable waitCond;
static uint32_t pendingMask = 0;
static uint32_t releasedMask = 0;

static uint32_t edgeMaskForIndex(int16_t hal_int_index) {
	return (1u << hal_int_index) | (1u << (hal_int_index + 8));
}

struct InterruptResource {
	VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
			VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
	VMXChannelInfo vmx_chan_info;
	InterruptConfig interrupt_config = InterruptConfig(false);
	void Init() {
		vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
				VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
		vmx_chan_info = VMXChannelInfo();
		interrupt_config = InterruptConfig(false);
	}
};

static LimitedHandleResource<HAL_InterruptHandle, InterruptResource, kNumInterrupts,
		HAL_HandleEnum::Interrupt>* interruptHandles = 0;

static std::atomic<InterruptResource *> handler_lookup_table[kNumInterrupts] = {};
static std::atomic<uint8_t> hal_int_index_lookup_table[kNumVMXPiInterrupts] = {};

static constexpr uint8_t kInvalidHALInterruptIndex = 255;

namespace hal {
namespace init {
void InitializeInterrupts() {
	static LimitedHandleResource<HAL_InterruptHandle, InterruptResource, kNumInterrupts,
			HAL_HandleEnum::Interrupt> iH;
	interruptHandles = &iH;
	for (int i = 0; i < kNumInterrupts; i++) {
		handler_lookup_table[i].store(nullptr);
	}
	for (int i = 0; i < kNumVMXPiInterrupts; i++) {
		hal_int_index_lookup_table[i] = kInvalidHALInterruptIndex;
	}
}
}  // namespace init
}  // namespace hal

/* This single function handles and dispatches all VMX-pi Interrupts */
static void VMXPi_InterruptHandler(uint32_t vmx_channel_index,
		InterruptEdgeType edge, void* param, uint64_t timestamp_us) {

	// Convert vmx_channel_index to wpi hal resource handle
	VMXResourceIndex interrupt_resource_index =
			static_cast<VMXResourceIndex>(vmx_channel_index);

	// get corresponding HAL_Interrupt object
	if (interrupt_resource_index >= kNumVMXPiInterrupts)
		return;

	uint8_t hal_int_index = hal_int_index_lookup_table[interrupt_resource_index];
	if (hal_int_index != kInvalidHALInterruptIndex) {
		InterruptResource *interrupt = handler_lookup_table[hal_int_index].load();
		if (interrupt != nullptr) {
			// NOTE:  This mask format limits the number of interrupts to 8
			uint32_t mask =
					(edge == InterruptEdgeType::RISING_EDGE_INTERRUPT) ?
							(1u << hal_int_index) :
							(1u << (hal_int_index + 8));
			{
				std::scoped_lock lock(waitMutex);
				pendingMask |= mask;
			}
			waitCond.notify_all();
		}
	}
}

extern "C" {
static void EnableInterrupts(HAL_InterruptHandle interruptHandle, int32_t* status);
static void DisableInterrupts(HAL_InterruptHandle interruptHandle, int32_t* status);
}

extern "C" {

/**
 * API Function
 *
 * Allocates one of the interrupt handles (in the reference implementation, they are limited,
 * but on VMX-pi there is one for each VMXChannelIndex (both digital and analog).
 *
 * Note that at this point, the input channel is not yet known.
 *
 * Interrupts are always used in synchronous mode, where the user program
 * blocks in HAL_WaitForInterrupt() while waiting for the interrupt to occur.
 **/

HAL_InterruptHandle HAL_InitializeInterrupts(int32_t* status) {
	hal::init::CheckInit();
	HAL_InterruptHandle handle = interruptHandles->Allocate();
	if (handle == HAL_kInvalidHandle) {
		*status = NO_AVAILABLE_RESOURCES;
		return HAL_kInvalidHandle;
	}

	auto anInterrupt = interruptHandles->Get(handle);
	if (anInterrupt == nullptr) {
		*status = NO_AVAILABLE_RESOURCES;
		return HAL_kInvalidHandle;
	}

	/* Since the input source channel is not yet known, */
	/* Defer VMX Resource allocation until later.       */
	anInterrupt->interrupt_config = InterruptConfig();

	int16_t hal_interrupt_handle_index = getHandleIndex(handle);
	{
		std::scoped_lock lock(waitMutex);
		pendingMask &= ~edgeMaskForIndex(hal_interrupt_handle_index);
		releasedMask &= ~edgeMaskForIndex(hal_interrupt_handle_index);
	}

	return handle;
}

/**
 * API Function
 *
 * Cancel interrupts on this device.
 *
 * This deallocates all structures and disables any interrupts.
 **/
void HAL_CleanInterrupts(HAL_InterruptHandle interruptHandle) {
	int32_t localStatus = 0;
	int32_t* status = &localStatus;
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		return;
	}

	int16_t hal_interrupt_handle_index = getHandleIndex(interruptHandle);
	if (hal_interrupt_handle_index >= kNumInterrupts) {
		return;
	}

	// Wake up anyone still waiting on this interrupt.
	HAL_ReleaseWaitingInterrupt(interruptHandle, status);

	if (!INVALID_VMX_RESOURCE_HANDLE(anInterrupt->vmx_res_handle)) {
		VMXResourceIndex vmx_interrupt_res_index = EXTRACT_VMX_RESOURCE_INDEX(anInterrupt->vmx_res_handle);

		DisableInterrupts(interruptHandle, status);
		mau::vmxIO->DeallocateResource(anInterrupt->vmx_res_handle, status);

		if (vmx_interrupt_res_index < kNumVMXPiInterrupts) {
			hal_int_index_lookup_table[vmx_interrupt_res_index] =
					kInvalidHALInterruptIndex;
		}
	}
	anInterrupt->Init();
	handler_lookup_table[hal_interrupt_handle_index].store(nullptr);

	interruptHandles->Free(interruptHandle);
}

/**
 * In synchronous mode, wait for the defined interrupt to occur.
 * @param timeout Timeout in seconds
 * @param ignorePrevious If true, ignore interrupts that happened before
 * waitForInterrupt was called.
 * @return The mask of interrupts that fired:
 * 0x000:  Timeout
 * 0x001:  Rising Edge
 * 0x100:  Falling Edge
 * 0x101:  Both Rising and Falling Edge
 *
 * NOTE:  HAL_RequestInterrupts will have already been invoked before this.
 *
 */

static int64_t WaitForInterruptMask(uint32_t waitMask, double timeout,
		HAL_Bool ignorePrevious) {
	std::unique_lock lock(waitMutex);
	if (ignorePrevious) {
		pendingMask &= ~waitMask;
	}
	auto ready = [&] { return (pendingMask & waitMask) || (releasedMask & waitMask); };
	if (!waitCond.wait_for(lock, std::chrono::duration<double>(timeout), ready)) {
		return kHalWaitResultInterruptTimeout;
	}
	int64_t result = pendingMask & waitMask;
	pendingMask &= ~waitMask;
	// A released wait reports a timeout, like athena.
	releasedMask &= ~waitMask;
	return result;
}

int64_t HAL_WaitForInterrupt(HAL_InterruptHandle interruptHandle,
		double timeout, HAL_Bool ignorePrevious, int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	// NOTE:  HAL_RequestInterrupts must have already been invoked previously.
	int16_t hal_interrupt_handle_index = getHandleIndex(interruptHandle);
	return WaitForInterruptMask(edgeMaskForIndex(hal_interrupt_handle_index),
			timeout, ignorePrevious);
}

/**
 * Waits for any of the interrupts in mask (bit n = HAL interrupt index n).
 */
int64_t HAL_WaitForMultipleInterrupts(HAL_InterruptHandle interruptHandle,
		int64_t mask, double timeout, HAL_Bool ignorePrevious, int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	uint32_t indexMask = static_cast<uint32_t>(mask) & 0xFF;
	return WaitForInterruptMask(indexMask | (indexMask << 8), timeout,
			ignorePrevious);
}

/**
 * Enable interrupts to occur on this input.
 * Interrupts are disabled when the RequestInterrupt call is made. This gives
 * time to do the setup of the other options before starting to field
 * interrupts.
 */
static void EnableInterrupts(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	mau::vmxIO->Interrupt_SetEnabled(anInterrupt->vmx_res_handle, true, status);
}

/**
 * Disable Interrupts without without deallocating structures.
 */
static void DisableInterrupts(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	mau::vmxIO->Interrupt_SetEnabled(anInterrupt->vmx_res_handle, false,
			status);
}

/**
 * Return the timestamp for the rising interrupt that occurred most recently.
 * This is in the same time domain as GetClock().
 * @return Timestamp in seconds since boot.
 */
int64_t HAL_ReadInterruptRisingTimestamp(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	uint64_t timestamp_microseconds;
	if (!mau::vmxIO->Interrupt_GetLastRisingEdgeTimestampMicroseconds(
			anInterrupt->vmx_res_handle, timestamp_microseconds, status)) {
		return 0;
	}

	return timestamp_microseconds;
}

/**
 * Return the timestamp for the falling interrupt that occurred most recently.
 * This is in the same time domain as GetClock().
 * @return Timestamp in seconds since boot.
 */
int64_t HAL_ReadInterruptFallingTimestamp(HAL_InterruptHandle interruptHandle,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return 0;
	}

	uint64_t timestamp_microseconds;
	if (!mau::vmxIO->Interrupt_GetLastFallingEdgeTimestampMicroseconds(
			anInterrupt->vmx_res_handle, timestamp_microseconds, status)) {
		return 0;
	}

	return timestamp_microseconds;
}

/**
 * API Function
 *
 * Attach an asynchronous interrupt handler to the interrupt handle
 * previously returned from HAL_RequestInterrupts.  This handler will
 * be invoked asynchronously when the interrupt fires.
 *
 * This function is also invoked for synchronous interrupt handling.
 *
 * wpiblic invokes this before HAL_EnableInterrupts is invoked.
 *
 * TODO:  Determine what the purpose of the analogTriggerType in this case is.
 */

void HAL_RequestInterrupts(HAL_InterruptHandle interruptHandle,
		HAL_Handle digitalSourceHandle, HAL_AnalogTriggerType analogTriggerType,
		int32_t* status) {
	auto anInterrupt = interruptHandles->Get(interruptHandle);
	if (anInterrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	VMXChannelInfo vmx_src_channel_info;
	VMXResourceIndex vmx_interrupt_res_index;
	if (!GetVMXInterruptResourceIndexForDigitalSourceHandle(digitalSourceHandle,
			vmx_interrupt_res_index, vmx_src_channel_info, status)) {
		return;
	}

	if (vmx_interrupt_res_index >= kNumVMXPiInterrupts) {
		*status = VMXERR_IO_INVALID_RESOURCE_INDEX;
		return;
	}

	int16_t hal_interrupt_handle_index = getHandleIndex(interruptHandle);
	if (hal_interrupt_handle_index >= kNumInterrupts) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	hal_int_index_lookup_table[vmx_interrupt_res_index] =
			hal_interrupt_handle_index;
	handler_lookup_table[hal_interrupt_handle_index].store(anInterrupt.get());

	anInterrupt->interrupt_config.SetHandler(VMXPi_InterruptHandler);

	if (!mau::vmxIO->ActivateSinglechannelResource(vmx_src_channel_info,
			&anInterrupt->interrupt_config, anInterrupt->vmx_res_handle,
			status)) {
		handler_lookup_table[hal_interrupt_handle_index].store(nullptr);
		hal_int_index_lookup_table[vmx_interrupt_res_index] =
				kInvalidHALInterruptIndex;
		return;
	}
	anInterrupt->vmx_chan_info = vmx_src_channel_info;

	// HAL_EnableInterrupts() no longer exists; interrupts are live once requested.
	EnableInterrupts(interruptHandle, status);
}

/**
 * API Function
 *
 * Configures the previously-allocated interrupt handle to trigger on
 * rising edge, falling edge, or both edges.
 *
 * WPILib 2026 calls this *after* HAL_RequestInterrupts (and again whenever
 * the edges change), so an active resource is reconfigured in place.
 */

void HAL_SetInterruptUpSourceEdge(HAL_InterruptHandle interruptHandle,
		HAL_Bool risingEdge, HAL_Bool fallingEdge, int32_t* status) {
	auto interrupt = interruptHandles->Get(interruptHandle);
	if (interrupt == nullptr) {
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (risingEdge) {
		if (fallingEdge) {
			interrupt->interrupt_config.SetEdge(
					InterruptConfig::InterruptEdge::BOTH);
		} else {
			interrupt->interrupt_config.SetEdge(
					InterruptConfig::InterruptEdge::RISING);
		}
	} else {
		interrupt->interrupt_config.SetEdge(
				InterruptConfig::InterruptEdge::FALLING);
	}

	if (!INVALID_VMX_RESOURCE_HANDLE(interrupt->vmx_res_handle)) {
		// Edge configuration is applied when the VMX resource is activated.
		mau::vmxIO->DeactivateResource(interrupt->vmx_res_handle, status);
		if (mau::vmxIO->SetResourceConfig(interrupt->vmx_res_handle,
				&interrupt->interrupt_config, status)) {
			mau::vmxIO->ActivateResource(interrupt->vmx_res_handle, status);
		}
	}
}

void HAL_ReleaseWaitingInterrupt(HAL_InterruptHandle interruptHandle,
                                 int32_t* status) {
  auto anInterrupt = interruptHandles->Get(interruptHandle);
  if (anInterrupt == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  int16_t hal_interrupt_handle_index = getHandleIndex(interruptHandle);
  if (hal_interrupt_handle_index >= kNumInterrupts) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  {
    std::scoped_lock lock(waitMutex);
    releasedMask |= edgeMaskForIndex(hal_interrupt_handle_index);
  }
  waitCond.notify_all();
}


}  // extern "C"
