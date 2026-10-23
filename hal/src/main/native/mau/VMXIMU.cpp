// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/VMXIMU.h"

#include <cmath>
#include <cstring>
#include <mutex>

#include "hal/Errors.h"
#include "HALInitializer.h"
#include <AHRS.h>

namespace mau {
extern vmx::AHRS* vmxIMU;
}

namespace {
// Serialize application reads/resets, without opening another SPI/VMX device.
std::mutex imuMutex;
vmx::AHRS* GetIMU(int32_t* status) {
  *status = 0;
  if (!hal::init::HAL_IsInitialized.load() || !mau::vmxIMU) {
    *status = INCOMPATIBLE_STATE;
    return nullptr;
  }
  return mau::vmxIMU;
}
}  // namespace

extern "C" {
double HAL_GetVMXIMUValue(HAL_VMXIMUProperty property, int32_t* status) {
  std::scoped_lock lock(imuMutex);
  auto* imu = GetIMU(status);
  if (!imu)
    return 0;
  switch (property) {
    case HAL_VMXIMU_Pitch:
      return imu->GetPitch();
    case HAL_VMXIMU_Yaw:
      return imu->GetYaw();
    case HAL_VMXIMU_Roll:
      return imu->GetRoll();
    case HAL_VMXIMU_CompassHeading:
      return imu->GetCompassHeading();
    case HAL_VMXIMU_IsCalibrating:
      return imu->IsCalibrating();
    case HAL_VMXIMU_IsConnected:
      return imu->IsConnected();
    case HAL_VMXIMU_ByteCount:
      return imu->GetByteCount();
    case HAL_VMXIMU_UpdateCount:
      return imu->GetUpdateCount();
    case HAL_VMXIMU_LastSensorTimestamp:
      return imu->GetLastSensorTimestamp();
    case HAL_VMXIMU_WorldLinearAccelX:
      return imu->GetWorldLinearAccelX();
    case HAL_VMXIMU_WorldLinearAccelY:
      return imu->GetWorldLinearAccelY();
    case HAL_VMXIMU_WorldLinearAccelZ:
      return imu->GetWorldLinearAccelZ();
    case HAL_VMXIMU_IsMoving:
      return imu->IsMoving();
    case HAL_VMXIMU_IsRotating:
      return imu->IsRotating();
    case HAL_VMXIMU_BarometricPressure:
      return imu->GetBarometricPressure();
    case HAL_VMXIMU_Altitude:
      return imu->GetAltitude();
    case HAL_VMXIMU_IsAltitudeValid:
      return imu->IsAltitudeValid();
    case HAL_VMXIMU_FusedHeading:
      return imu->GetFusedHeading();
    case HAL_VMXIMU_IsMagneticDisturbance:
      return imu->IsMagneticDisturbance();
    case HAL_VMXIMU_IsMagnetometerCalibrated:
      return imu->IsMagnetometerCalibrated();
    case HAL_VMXIMU_QuaternionW:
      return imu->GetQuaternionW();
    case HAL_VMXIMU_QuaternionX:
      return imu->GetQuaternionX();
    case HAL_VMXIMU_QuaternionY:
      return imu->GetQuaternionY();
    case HAL_VMXIMU_QuaternionZ:
      return imu->GetQuaternionZ();
    case HAL_VMXIMU_VelocityX:
      return imu->GetVelocityX();
    case HAL_VMXIMU_VelocityY:
      return imu->GetVelocityY();
    case HAL_VMXIMU_VelocityZ:
      return imu->GetVelocityZ();
    case HAL_VMXIMU_DisplacementX:
      return imu->GetDisplacementX();
    case HAL_VMXIMU_DisplacementY:
      return imu->GetDisplacementY();
    case HAL_VMXIMU_DisplacementZ:
      return imu->GetDisplacementZ();
    case HAL_VMXIMU_Angle:
      return imu->GetAngle();
    case HAL_VMXIMU_Rate:
      return imu->GetRate();
    case HAL_VMXIMU_RawGyroX:
      return imu->GetRawGyroX();
    case HAL_VMXIMU_RawGyroY:
      return imu->GetRawGyroY();
    case HAL_VMXIMU_RawGyroZ:
      return imu->GetRawGyroZ();
    case HAL_VMXIMU_RawAccelX:
      return imu->GetRawAccelX();
    case HAL_VMXIMU_RawAccelY:
      return imu->GetRawAccelY();
    case HAL_VMXIMU_RawAccelZ:
      return imu->GetRawAccelZ();
    case HAL_VMXIMU_RawMagX:
      return imu->GetRawMagX();
    case HAL_VMXIMU_RawMagY:
      return imu->GetRawMagY();
    case HAL_VMXIMU_RawMagZ:
      return imu->GetRawMagZ();
    case HAL_VMXIMU_Pressure:
      return imu->GetPressure();
    case HAL_VMXIMU_TempC:
      return imu->GetTempC();
    case HAL_VMXIMU_ActualUpdateRate:
      return imu->GetActualUpdateRate();
    case HAL_VMXIMU_RequestedUpdateRate:
      return imu->GetRequestedUpdateRate();
    case HAL_VMXIMU_BoardYawAxis:
      return static_cast<int>(imu->GetBoardYawAxis().board_axis);
    case HAL_VMXIMU_IsBoardYawAxisUp:
      return imu->GetBoardYawAxis().up;
    default:
      *status = PARAMETER_OUT_OF_RANGE;
      return 0;
  }
}
void HAL_ZeroVMXIMUYaw(int32_t* status) {
  std::scoped_lock lock(imuMutex);
  if (auto* imu = GetIMU(status))
    imu->ZeroYaw();
}
void HAL_ResetVMXIMU(int32_t* status) {
  std::scoped_lock lock(imuMutex);
  if (auto* imu = GetIMU(status))
    imu->Reset();
}
void HAL_ResetVMXIMUDisplacement(int32_t* status) {
  std::scoped_lock lock(imuMutex);
  if (auto* imu = GetIMU(status))
    imu->ResetDisplacement();
}
void HAL_UpdateVMXIMUDisplacement(double accelX, double accelY, int32_t rate,
                                  HAL_Bool moving, int32_t* status) {
  std::scoped_lock lock(imuMutex);
  auto* imu = GetIMU(status);
  if (!imu)
    return;
  if (!std::isfinite(accelX) || !std::isfinite(accelY) ||
      std::abs(accelX) > 16 || std::abs(accelY) > 16 || rate < 1 ||
      rate > 200) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  imu->UpdateDisplacement(static_cast<float>(accelX),
                          static_cast<float>(accelY), rate, moving != 0);
}
void HAL_GetVMXIMUFirmwareVersion(char* buffer, int32_t capacity,
                                  int32_t* status) {
  std::scoped_lock lock(imuMutex);
  auto* imu = GetIMU(status);
  if (!imu)
    return;
  if (!buffer) {
    *status = NULL_PARAMETER;
    return;
  }
  auto version = imu->GetFirmwareVersion();
  if (capacity <= 0 || version.size() >= static_cast<size_t>(capacity)) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  std::memcpy(buffer, version.c_str(), version.size() + 1);
}
}  // extern "C"
