/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/I2C.h"
#include "hal/Errors.h"

#include "HALInitializer.h"
#include "MauInternal.h"

using namespace hal;

namespace hal {
namespace init {
void InitializeI2C() {
}
}
}

static VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
		VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);

/* VMX-pi only has one I2C port, whereas the reference implementation uses 2. */
/* For convenience, both of the WPI I2C port identifiers map the same         */
/* underlying VMX-pi resource.                                                */

extern "C" {
void HAL_InitializeI2C(HAL_I2CPort port, int32_t* status) {
	hal::init::CheckInit();
	if (!INVALID_VMX_RESOURCE_HANDLE(vmx_res_handle)) {
		*status = NO_AVAILABLE_RESOURCES;
		return;
	}

	if (port < 0 || port > 1) {
		*status = RESOURCE_OUT_OF_RANGE;
	    return;
	}

	VMXChannelInfo i2c_channels[2] = { { VMXChannelInfo(
			mau::vmxIO->GetSoleChannelIndex(VMXChannelCapability::I2C_SDA),
			VMXChannelCapability::I2C_SDA) }, { VMXChannelInfo(
			mau::vmxIO->GetSoleChannelIndex(VMXChannelCapability::I2C_SCL),
			VMXChannelCapability::I2C_SCL) } };

	I2CConfig i2c_cfg;

	if (!mau::vmxIO->ActivateDualchannelResource(i2c_channels[0], i2c_channels[1],
			&i2c_cfg, vmx_res_handle, status)) {
		return;
	}
}

/**
 * Generic transaction.
 *
 * This is a lower-level interface to the I2C hardware giving you more control
 * over each transaction.
 *
 * @param dataToSend Buffer of data to send as part of the transaction.
 * @param sendSize Number of bytes to send as part of the transaction.
 * @param dataReceived Buffer to read data into.
 * @param receiveSize Number of bytes to read from the device.
 * @return >= 0 on success or -1 on transfer abort.
 */
int32_t HAL_TransactionI2C(HAL_I2CPort port, int32_t deviceAddress,
		const uint8_t* dataToSend, int32_t sendSize, uint8_t* dataReceived,
		int32_t receiveSize) {

	int32_t status __attribute__((unused)) = 0;

	if (dataToSend == nullptr) {
		//*status = NULL_PARAMETER_MESSAGE;
		return -1;
	}

	if (dataReceived == nullptr) {
		//*status = NULL_PARAMETER_MESSAGE;
		return -1;
	}

	if (port < 0 || port > 1) {
		//*status = RESOURCE_OUT_OF_RANGE;
	    return -1;
	}

	if (mau::vmxIO->I2C_Transaction(vmx_res_handle,
			static_cast<uint8_t>(deviceAddress), const_cast<uint8_t *>(dataToSend),
			static_cast<uint16_t>(sendSize), dataReceived,
			static_cast<uint16_t>(receiveSize), &status)) {
		return 0;
	}
	return -1;
}

/**
 * Execute a write transaction with the device.
 *
 * Write a single byte to a register on a device and wait until the
 *   transaction is complete.
 *
 * @param deviceAddress The address of  the device to be written to.
 * @param dataToSend The bytes to write to the register on the device.
 *        NOTE:  The first byte is always set to the registerAddress
 * @param sendSize:  The number of bytes to send (including the first
 *        byte, which is the register address;
 * @return >= 0 on success or -1 on transfer abort.
 */
int32_t HAL_WriteI2C(HAL_I2CPort port, int32_t deviceAddress,
		const uint8_t* dataToSend, int32_t sendSize) {

	int32_t status __attribute__((unused)) = 0;
	if (dataToSend == nullptr) {
		//*status = NULL_PARAMETER_MESSAGE;
		return -1;
	}
	// There must be at least 2 bytes (registerAddress, and one data byte)
	if (sendSize < 2) {
		//*status = PARAMETER_OUT_OF_RANGE;
		return -1;
	}

	if (port < 0 || port > 1) {
		//*status = RESOURCE_OUT_OF_RANGE;
	    return -1;
	}

	uint8_t registerAddress = dataToSend[0];
	if (mau::vmxIO->I2C_Write(vmx_res_handle,
				static_cast<uint8_t>(deviceAddress),
				registerAddress,
				const_cast<uint8_t *>(dataToSend),
				static_cast<uint16_t>(sendSize),
				&status)) {
		return 0;
	}
	return -1;

	}

	/**
	 * Execute a read transaction with the device.
	 *
	 * Read bytes from a device.
	 * Most I2C devices will auto-increment the register pointer internally allowing
	 *   you to read consecutive registers on a device in a single transaction.
	 *
	 * @param registerAddress The register to read first in the transaction.
	 * @param count The number of bytes to read in the transaction.
	 * @param buffer A pointer to the array of bytes to store the data read from the
	 * device.
	 * @return >= 0 on success or -1 on transfer abort.
	 */
	int32_t HAL_ReadI2C(HAL_I2CPort port, int32_t deviceAddress,
			uint8_t* buffer, int32_t count) {
		int32_t status __attribute__((unused)) = 0;
		if (buffer == nullptr) {
			return -1;
		}

		if (count < 1) {
			return -1;
		}

		if (port < 0 || port > 1) {
		    //*status = RESOURCE_OUT_OF_RANGE;
		    return -1;
		}

		// Read-only transactions send a 0-sized send count to the I2C Transaction handler.
		uint16_t sendSize = 0;

		if (mau::vmxIO->I2C_Transaction(vmx_res_handle,
				static_cast<uint8_t>(deviceAddress),
				nullptr, sendSize,
				buffer, count, &status)) {
			return 0;
		}
		return -1;
	}

	void HAL_CloseI2C(HAL_I2CPort port) {
		if (port < 0 || port > 1) {
			//*status = RESOURCE_OUT_OF_RANGE;
		    return;
		}
		if (!INVALID_VMX_RESOURCE_HANDLE(vmx_res_handle)) {
			int32_t status __attribute__((unused)) = 0;
			mau::vmxIO->DeallocateResource(vmx_res_handle, &status);
			vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
					VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
		}
	}
}  // extern "C"
