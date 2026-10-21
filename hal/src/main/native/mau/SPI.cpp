/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <mutex>

#include "hal/SPI.h"
#include "hal/Errors.h"

#include "HALInitializer.h"
#include "InterruptsInternal.h"
#include "MauInternal.h"
#include <wpi/mutex.h>
static constexpr int32_t kSpiMaxHandles = 5;

using namespace hal;

namespace hal {
    namespace init {
        void InitializeSPI() {}
    }
}

static VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
		VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
static SPIConfig spi_cfg(500000 /* Bitrate */);
static int handle = -1;  // This handle is specified by the user.

static AutoTransmitEngineHandle engine_handle = INVALID_AUTO_TRANSMIT_ENGINE_HANDLE;

static wpi::mutex spiAutoMutex;
static int32_t spiAutoPort = kSpiMaxHandles;
static std::atomic_bool spiAutoRunning{false};

static void ReinitializeSPIIfAlreadyAllocated(HAL_SPIPort port, int32_t *status) {
	if (INVALID_VMX_RESOURCE_HANDLE(vmx_res_handle)) {
		return;
	}
	HAL_CloseSPI(port);
	HAL_InitializeSPI(port, status);
}

/* VMX-pi only has one SPI port, whereas the reference implementation uses 5, */
/* comprised of two engines (Onboard and MXP).  Onboard has 4 chip selects,   */
/* MXP has 1.                                                                 */
/* For convenience, both of the WPI SPI port identifiers map the same         */
/* underlying VMX-pi resource.                                                */

void HAL_InitializeSPI(HAL_SPIPort port, int32_t* status) {
    hal::init::CheckInit();

	if (!INVALID_VMX_RESOURCE_HANDLE(vmx_res_handle)) {
		*status = NO_AVAILABLE_RESOURCES;
		return;
	}

	VMXChannelInfo spi_channels[4] = {
		{ VMXChannelInfo(mau::vmxIO->GetSoleChannelIndex(VMXChannelCapability::SPI_CLK), VMXChannelCapability::SPI_CLK) },
		{ VMXChannelInfo(mau::vmxIO->GetSoleChannelIndex(VMXChannelCapability::SPI_MOSI), VMXChannelCapability::SPI_MOSI) },
		{ VMXChannelInfo(mau::vmxIO->GetSoleChannelIndex(VMXChannelCapability::SPI_MISO), VMXChannelCapability::SPI_MISO) },
		{ VMXChannelInfo(mau::vmxIO->GetSoleChannelIndex(VMXChannelCapability::SPI_CS), VMXChannelCapability::SPI_CS) }};

	if (!mau::vmxIO->ActivateQuadchannelResource(spi_channels[0], spi_channels[1], spi_channels[2], spi_channels[3], &spi_cfg, vmx_res_handle, status)) {
		return;
	}

}

/* NOTE:  In the future, VMX-pi SPI may support multiple chip selects */

/**
 * Generic transaction.
 *
 * This is a lower-level interface to the spi hardware giving you more control
 * over each transaction.
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @param dataToSend Buffer of data to send as part of the transaction.
 * @param dataReceived Buffer to read data into.
 * @param size Number of bytes to transfer. [0..7]
 * @return Number of bytes transferred, -1 for error
 */
int32_t HAL_TransactionSPI(HAL_SPIPort port, const uint8_t* dataToSend, uint8_t* dataReceived, int32_t size) {
	int32_t status __attribute__((unused)) = 0;
	if (dataToSend == nullptr) {
		return -1;
	}
	if (dataReceived == nullptr) {
		return -1;
	}
	if (mau::vmxIO->SPI_Transaction(vmx_res_handle,
			const_cast<uint8_t *>(dataToSend),
			dataReceived,
			static_cast<uint16_t>(size), &status)) {
		return size;
	}
	return -1;
}

int32_t HAL_WriteSPI(HAL_SPIPort port, const uint8_t* dataToSend, int32_t sendSize) {
	int32_t status __attribute__((unused)) = 0;
	if (dataToSend == nullptr) {
		return -1;
	}
	if (mau::vmxIO->SPI_Write(vmx_res_handle,
			const_cast<uint8_t *>(dataToSend),
			static_cast<uint16_t>(sendSize), &status)) {
		return sendSize;
	}
	return -1;
}

int32_t HAL_ReadSPI(HAL_SPIPort port, uint8_t* buffer, int32_t count) {
	int32_t status __attribute__((unused)) = 0;
	if (buffer == nullptr) {
		return -1;
	}
	if (mau::vmxIO->SPI_Read(vmx_res_handle,
			const_cast<uint8_t *>(buffer),
			static_cast<uint16_t>(count), &status)) {
		return count;
	}
	return -1;
}

void HAL_CloseSPI(HAL_SPIPort port) {
	if (!INVALID_VMX_RESOURCE_HANDLE(vmx_res_handle)) {
		int32_t status __attribute__((unused)) = 0;
		mau::vmxIO->DeallocateResource(vmx_res_handle, &status);
		vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
				VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
	}
}

/**
 * Set the clock speed for the SPI bus.
 *
 * NOTE:  The default value is 500,000Hz.
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @param speed The speed in Hz (0-1MHz)
 */
void HAL_SetSPISpeed(HAL_SPIPort port, int32_t speed) {
	if (spi_cfg.GetBitrate() == static_cast<uint32_t>(speed)) {
		return;
	}
	spi_cfg.SetBitrate(speed);
	int32_t status __attribute__((unused));
	ReinitializeSPIIfAlreadyAllocated(port, &status);
}

/**
 * Set the SPI options
 *
 * @param port The number of the port to use. 0-3 for Onboard CS0-CS2, 4 for MXP
 * @param msbFirst True to write the MSB first, False for LSB first
 * @param sampleOnTrailing True to sample on the trailing edge, False to sample
 * on the leading edge
 * @param clkIdleHigh True to set the clock to active low, False to set the
 * clock active high
 */
void HAL_SetSPIOpts(HAL_SPIPort port, HAL_Bool msbFirst, HAL_Bool sampleOnTrailing, HAL_Bool clkIdleHigh) {
	uint8_t cpha = sampleOnTrailing ? (1 << 0) : 0;
	uint8_t cpol = clkIdleHigh ? (1 << 1) : 0;
	uint8_t mode = 	cpol + cpha;
	if ((msbFirst != spi_cfg.GetMSBFirst()) || (mode != spi_cfg.GetMode())) {
		spi_cfg.SetMSBFirst(msbFirst);
		spi_cfg.SetMode(mode);
		int32_t status __attribute__((unused));
		ReinitializeSPIIfAlreadyAllocated(port, &status);
	}
}

void HAL_SetSPIChipSelectActiveHigh(HAL_SPIPort port, int32_t* status) {

	if (!spi_cfg.GetCSActiveLow()) {
		return;
	}

	spi_cfg.SetCSActiveLow(false);
	ReinitializeSPIIfAlreadyAllocated(port, status);

}

void HAL_SetSPIChipSelectActiveLow(HAL_SPIPort port, int32_t* status) {

	if (spi_cfg.GetCSActiveLow()) {
		return;
	}

	spi_cfg.SetCSActiveLow(false);
	ReinitializeSPIIfAlreadyAllocated(port, status);

}

int32_t HAL_GetSPIHandle(HAL_SPIPort port) {
    return handle;
}

void HAL_SetSPIHandle(HAL_SPIPort port, int32_t handle_value) {
	handle = handle_value;
}

/****************************************************************************
 * SPIAuto:  Automatic SPI transfer engine.
 *
 * A single engine is provided, which:
 *
 * - performs periodic SPI transactions at a specified rate
 * - stores a command buffer, comprised of up to 16 bytes (sent to device),
 *   followed by up to 127 bytes of received data.
 * - can optionally be triggered by a digital input or analog trigger
 * - can optionally be forcibly triggered by software
 * - allows acquiring the most-recently read data
 * - allows getting a count of available databytes
 * - manages a count of number of times a triggered transaction did not occur
 *   because the most-recently read data has not yet been read
 */

/**
 * Initialize automatic SPI transfer engine.
 *
 * Only a single engine is available, and use of it blocks use of all other
 * chip select usage on the same physical SPI port while it is running.
 *
 * @param bufferSize buffer size in bytes
 */
void HAL_InitSPIAuto(HAL_SPIPort port, int32_t bufferSize, int32_t* status) {
	mau::vmxIO->AutoTransmit_Allocate(engine_handle, status);
	spiAutoPort = port;
}

/**
 * Frees the automatic SPI transfer engine.
 */
void HAL_FreeSPIAuto(HAL_SPIPort port, int32_t* status) {
	mau::vmxIO->AutoTransmit_Stop(engine_handle, status);
	mau::vmxIO->AutoTransmit_Deallocate(engine_handle, status);
	engine_handle = INVALID_AUTO_TRANSMIT_ENGINE_HANDLE;
	spiAutoPort = kSpiMaxHandles;
}

/**
 * Start running the automatic SPI transfer engine at a periodic rate.
 *
 * InitAuto() and SetAutoTransmitData() must be called before calling this
 * function.
 *
 * @param period period between transfers, in seconds (us resolution)
 */
void HAL_StartSPIAutoRate(HAL_SPIPort port, double period, int32_t* status) {
	uint32_t repeat_every_ms = static_cast<uint32_t>(period * 1000);
	mau::vmxIO->AutoTransmit_StartPeriodic(engine_handle, vmx_res_handle, repeat_every_ms, status);
}

/**
 * Start running the automatic SPI transfer engine when a trigger occurs.
 *
 * InitAuto() and SetAutoTransmitData() must be called before calling this
 * function.
 *
 * @param source digital source for the trigger (may be an analog trigger)
 * @param rising trigger on the rising edge
 * @param falling trigger on the falling edge
 */
void HAL_StartSPIAutoTrigger(HAL_SPIPort port, HAL_Handle digitalSourceHandle, HAL_AnalogTriggerType analogTriggerType,
                             HAL_Bool triggerRising, HAL_Bool triggerFalling, int32_t* status) {
	VMXResourceIndex int_res_index;
	VMXChannelInfo vmx_chan_info;
	if (GetVMXInterruptResourceIndexForDigitalSourceHandle(digitalSourceHandle, int_res_index, vmx_chan_info, status)) {
		InterruptConfig::InterruptEdge edge_type;
		if (triggerRising) {
			if (triggerFalling) {
				edge_type = InterruptConfig::InterruptEdge::BOTH;
			} else {
				edge_type = InterruptConfig::InterruptEdge::RISING;
			}
		} else {
			edge_type = InterruptConfig::InterruptEdge::FALLING;
		}
		mau::vmxIO->AutoTransmit_StartTrigger(engine_handle, vmx_res_handle, vmx_chan_info.index, edge_type, status);
	}
}

/**
 * Stop running the automatic SPI transfer engine.
 */
void HAL_StopSPIAuto(HAL_SPIPort port, int32_t* status) {
	mau::vmxIO->AutoTransmit_Stop(engine_handle, status);
}

/**
 * Set the data to be transmitted by the engine.
 *
 * Up to 16 bytes are configurable, and may be followed by up to 127 zero
 * bytes.
 *
 * @param dataToSend data to send (maximum 16 bytes)
 * @param zeroSize number of zeros to send after the data
 */
void HAL_SetSPIAutoTransmitData(HAL_SPIPort port, const uint8_t* dataToSend, int32_t dataSize, int32_t zeroSize,
                                int32_t* status) {
	mau::vmxIO->AutoTransmit_SetData(engine_handle, const_cast<uint8_t *>(dataToSend), dataSize, zeroSize, status);
}

/**
 * Force the engine to make a single transfer.
 */
void HAL_ForceSPIAutoRead(HAL_SPIPort port, int32_t* status) {
	mau::vmxIO->AutoTransmit_Immediate(engine_handle, status);
}

/**
 * Read data that has been transferred by the automatic SPI transfer engine.
 *
 * Transfers may be made a byte at a time, so it's necessary for the caller
 * to handle cases where an entire transfer has not been completed.
 *
 * Blocks until numToRead bytes have been read or timeout expires.
 * May be called with numToRead=0 to retrieve how many bytes are available.
 *
 * NOTE:  The buffer is a buffer of uint32_t (32-bit ints).  The first int is
 * a timestamp; the remainder of ints store the byte in the lower 8 bits.
 * (This clarification provided by Thad House on 3/7/2019).
 * 
 * TODO:  Update implementation to match this new 32-bit int format.
 *
 * @param buffer buffer where read bytes are stored
 * @param numToRead number of bytes to read
 * @param timeout timeout in seconds (ms resolution)
 * @return Number of bytes remaining to be read
 */
int32_t HAL_ReadSPIAutoReceivedData(HAL_SPIPort port, uint32_t* buffer,  int32_t numToRead, double timeout,
                                    int32_t* status) {
	int32_t num_bytes_remaining = 0;
	uint32_t timeout_ms = static_cast<uint32_t>(timeout * 1000);
	mau::vmxIO->AutoTransmit_GetData(engine_handle, (uint8_t *)buffer, numToRead, timeout_ms, num_bytes_remaining, status);
	return num_bytes_remaining;
}

/**
 * Get the number of bytes dropped by the automatic SPI transfer engine due
 * to the receive buffer being full.
 *
 * @return Number of bytes dropped
 */
int32_t HAL_GetSPIAutoDroppedCount(HAL_SPIPort port, int32_t* status) {
	int32_t num_dropped = 0;
	mau::vmxIO->AutoTransmit_GetNumDropped(engine_handle, num_dropped, status);
	return num_dropped;
}

void HAL_ConfigureSPIAutoStall(HAL_SPIPort port, int32_t csToSclkTicks,
                               int32_t stallTicks, int32_t pow2BytesPerRead,
                               int32_t* status) {
  std::scoped_lock lock(spiAutoMutex);
  // FPGA only has one auto SPI engine
  if (port != spiAutoPort) {
    *status = INCOMPATIBLE_STATE;
    return;
  }

  std::printf("TODO:  HAL_ConfigureSPIAutoStall() implementation goes here.");
  // TODO:  Implement this in VMX-pi HAL Library
}

// 2026 replaced HAL_SetSPIOpts() with SPI modes; map them onto the same config.
void HAL_SetSPIMode(HAL_SPIPort port, HAL_SPIMode mode) {
	bool sampleOnTrailing = (mode == HAL_SPI_kMode1) || (mode == HAL_SPI_kMode3);
	bool clkIdleHigh = (mode == HAL_SPI_kMode2) || (mode == HAL_SPI_kMode3);
	HAL_SetSPIOpts(port, true, sampleOnTrailing, clkIdleHigh);
}

HAL_SPIMode HAL_GetSPIMode(HAL_SPIPort port) {
	return static_cast<HAL_SPIMode>(spi_cfg.GetMode());
}
