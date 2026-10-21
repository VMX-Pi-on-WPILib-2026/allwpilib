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
#include "HALInternal.h"
#include "InterruptsInternal.h"
#include "MauInternal.h"
#include <wpi/mutex.h>

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

	spi_cfg.SetCSActiveLow(true);
	ReinitializeSPIIfAlreadyAllocated(port, status);

}

int32_t HAL_GetSPIHandle(HAL_SPIPort port) {
    return handle;
}

void HAL_SetSPIHandle(HAL_SPIPort port, int32_t handle_value) {
	handle = handle_value;
}

// The VMX SDK exposes byte-only auto transfers without the per-transfer
// timestamps required by the 2026 HAL. Reject the feature at initialization
// rather than returning packed bytes as timestamped uint32_t samples.
static void UnsupportedSPIAuto(int32_t* status) {
    *status = MAU_UNSUPPORTED_FEATURE;
    hal::SetLastError(status, "VMX-pi SDK does not support WPILib timestamped automatic SPI transfers");
}

void HAL_InitSPIAuto(HAL_SPIPort, int32_t, int32_t* status) { UnsupportedSPIAuto(status); }
void HAL_FreeSPIAuto(HAL_SPIPort, int32_t*) {}
void HAL_StartSPIAutoRate(HAL_SPIPort, double, int32_t* status) { UnsupportedSPIAuto(status); }
void HAL_StartSPIAutoTrigger(HAL_SPIPort, HAL_Handle, HAL_AnalogTriggerType,
                             HAL_Bool, HAL_Bool, int32_t* status) { UnsupportedSPIAuto(status); }
void HAL_StopSPIAuto(HAL_SPIPort, int32_t*) {}
void HAL_SetSPIAutoTransmitData(HAL_SPIPort, const uint8_t*, int32_t, int32_t,
                                int32_t* status) { UnsupportedSPIAuto(status); }
void HAL_ForceSPIAutoRead(HAL_SPIPort, int32_t* status) { UnsupportedSPIAuto(status); }
int32_t HAL_ReadSPIAutoReceivedData(HAL_SPIPort, uint32_t*, int32_t, double,
                                    int32_t* status) {
    UnsupportedSPIAuto(status);
    return 0;
}
int32_t HAL_GetSPIAutoDroppedCount(HAL_SPIPort, int32_t* status) {
    UnsupportedSPIAuto(status);
    return 0;
}
void HAL_ConfigureSPIAutoStall(HAL_SPIPort, int32_t, int32_t, int32_t,
                               int32_t* status) { UnsupportedSPIAuto(status); }

// 2026 replaced HAL_SetSPIOpts() with SPI modes; map them onto the same config.
void HAL_SetSPIMode(HAL_SPIPort port, HAL_SPIMode mode) {
	bool sampleOnTrailing = (mode == HAL_SPI_kMode1) || (mode == HAL_SPI_kMode3);
	bool clkIdleHigh = (mode == HAL_SPI_kMode2) || (mode == HAL_SPI_kMode3);
	HAL_SetSPIOpts(port, true, sampleOnTrailing, clkIdleHigh);
}

HAL_SPIMode HAL_GetSPIMode(HAL_SPIPort port) {
	return static_cast<HAL_SPIMode>(spi_cfg.GetMode());
}
