/* Copyright (c) FIRST and other WPILib contributors.
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project. */

#include "hal/I2C.h"
#include "hal/Errors.h"

#include <array>
#include <limits>
#include <mutex>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "HALInitializer.h"
#include "MauInternal.h"

namespace hal::init {
void InitializeI2C() {}
}

namespace {
VMXResourceHandle vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
    VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
std::mutex busMutex;
// Both WPILib port aliases share the one VMX I2C bus. Each device opens it.
std::array<unsigned, 2> references{};

bool validPort(HAL_I2CPort port) {
  return port >= 0 && port < static_cast<int>(references.size());
}
}

extern "C" {
void HAL_InitializeI2C(HAL_I2CPort port, int32_t* status) {
  hal::init::CheckInit();
  std::scoped_lock lock(busMutex);
  if (!validPort(port)) {
    *status = RESOURCE_OUT_OF_RANGE;
    return;
  }
  if (INVALID_VMX_RESOURCE_HANDLE(vmx_res_handle)) {
    VMXChannelInfo sda(mau::vmxIO->GetSoleChannelIndex(VMXChannelCapability::I2C_SDA),
                       VMXChannelCapability::I2C_SDA);
    VMXChannelInfo scl(mau::vmxIO->GetSoleChannelIndex(VMXChannelCapability::I2C_SCL),
                       VMXChannelCapability::I2C_SCL);
    I2CConfig config;
    if (!mau::vmxIO->ActivateDualchannelResource(sda, scl, &config, vmx_res_handle, status)) return;
  }
  ++references[port];
}

int32_t HAL_TransactionI2C(HAL_I2CPort port, int32_t deviceAddress,
                          const uint8_t* dataToSend, int32_t sendSize,
                          uint8_t* dataReceived, int32_t receiveSize) {
  std::scoped_lock lock(busMutex);
  if (!validPort(port) || !references[port] || deviceAddress < 0 || deviceAddress > 127 ||
      sendSize < 0 || sendSize > std::numeric_limits<uint16_t>::max() ||
      receiveSize < 0 || receiveSize > std::numeric_limits<uint16_t>::max() ||
      (sendSize && !dataToSend) || (receiveSize && !dataReceived)) return -1;
  if (sendSize == 0 && receiveSize == 0) {
    // The SDK skips both segments for an empty transaction and reports success
    // without testing ACK. Its Raspberry Pi hardware I2C provider uses bus 1.
    int fd = open("/dev/i2c-1", O_RDWR | O_CLOEXEC);
    if (fd < 0) return -1;
    i2c_msg message{};
    message.addr = static_cast<uint16_t>(deviceAddress);
    i2c_rdwr_ioctl_data request{&message, 1};
    int result = ioctl(fd, I2C_RDWR, &request);
    close(fd);
    return result == 1 ? 0 : -1;
  }
  int32_t status = 0;
  // Zero-sized directions (including AddressOnly) are valid transactions.
  return mau::vmxIO->I2C_Transaction(
      vmx_res_handle, static_cast<uint8_t>(deviceAddress),
      const_cast<uint8_t*>(dataToSend), static_cast<uint16_t>(sendSize),
      dataReceived, static_cast<uint16_t>(receiveSize), &status) ? 0 : -1;
}

int32_t HAL_WriteI2C(HAL_I2CPort port, int32_t deviceAddress,
                     const uint8_t* dataToSend, int32_t sendSize) {
  // HAL callers supply the complete wire payload, including any register byte.
  return HAL_TransactionI2C(port, deviceAddress, dataToSend, sendSize, nullptr, 0);
}

int32_t HAL_ReadI2C(HAL_I2CPort port, int32_t deviceAddress,
                    uint8_t* buffer, int32_t count) {
  return HAL_TransactionI2C(port, deviceAddress, nullptr, 0, buffer, count);
}

void HAL_CloseI2C(HAL_I2CPort port) {
  std::scoped_lock lock(busMutex);
  if (!validPort(port) || !references[port]) return;
  --references[port];
  if (references[0] || references[1] || INVALID_VMX_RESOURCE_HANDLE(vmx_res_handle)) return;
  int32_t status = 0;
  if (mau::vmxIO->DeallocateResource(vmx_res_handle, &status)) {
    vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(
        VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX);
  }
}
}  // extern "C"
