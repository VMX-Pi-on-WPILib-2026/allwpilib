// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>
#include "hal/Types.h"

/** Scalar properties of the HAL-owned VMX IMU. Values retain SDK units. */
HAL_ENUM(HAL_VMXIMUProperty) {
  HAL_VMXIMU_Pitch = 0,
  HAL_VMXIMU_Yaw = 1,
  HAL_VMXIMU_Roll = 2,
  HAL_VMXIMU_CompassHeading = 3,
  HAL_VMXIMU_IsCalibrating = 4,
  HAL_VMXIMU_IsConnected = 5,
  HAL_VMXIMU_ByteCount = 6,
  HAL_VMXIMU_UpdateCount = 7,
  HAL_VMXIMU_LastSensorTimestamp = 8,
  HAL_VMXIMU_WorldLinearAccelX = 9,
  HAL_VMXIMU_WorldLinearAccelY = 10,
  HAL_VMXIMU_WorldLinearAccelZ = 11,
  HAL_VMXIMU_IsMoving = 12,
  HAL_VMXIMU_IsRotating = 13,
  HAL_VMXIMU_BarometricPressure = 14,
  HAL_VMXIMU_Altitude = 15,
  HAL_VMXIMU_IsAltitudeValid = 16,
  HAL_VMXIMU_FusedHeading = 17,
  HAL_VMXIMU_IsMagneticDisturbance = 18,
  HAL_VMXIMU_IsMagnetometerCalibrated = 19,
  HAL_VMXIMU_QuaternionW = 20,
  HAL_VMXIMU_QuaternionX = 21,
  HAL_VMXIMU_QuaternionY = 22,
  HAL_VMXIMU_QuaternionZ = 23,
  HAL_VMXIMU_VelocityX = 24,
  HAL_VMXIMU_VelocityY = 25,
  HAL_VMXIMU_VelocityZ = 26,
  HAL_VMXIMU_DisplacementX = 27,
  HAL_VMXIMU_DisplacementY = 28,
  HAL_VMXIMU_DisplacementZ = 29,
  HAL_VMXIMU_Angle = 30,
  HAL_VMXIMU_Rate = 31,
  HAL_VMXIMU_RawGyroX = 32,
  HAL_VMXIMU_RawGyroY = 33,
  HAL_VMXIMU_RawGyroZ = 34,
  HAL_VMXIMU_RawAccelX = 35,
  HAL_VMXIMU_RawAccelY = 36,
  HAL_VMXIMU_RawAccelZ = 37,
  HAL_VMXIMU_RawMagX = 38,
  HAL_VMXIMU_RawMagY = 39,
  HAL_VMXIMU_RawMagZ = 40,
  HAL_VMXIMU_Pressure = 41,
  HAL_VMXIMU_TempC = 42,
  HAL_VMXIMU_ActualUpdateRate = 43,
  HAL_VMXIMU_RequestedUpdateRate = 44,
  HAL_VMXIMU_BoardYawAxis = 45,
  HAL_VMXIMU_IsBoardYawAxisUp = 46,
};

#ifdef __cplusplus
extern "C" {
#endif

/** Read an IMU property after HAL initialization. Does not create/open
 * hardware. Returns 0 and a nonzero status if unavailable or property is
 * invalid. Platforms without a VMX backend return INCOMPATIBLE_STATE.
 */
double HAL_GetVMXIMUValue(HAL_VMXIMUProperty property, int32_t* status);
/** Zero shared yaw. Affects every reader of this board's IMU. */
void HAL_ZeroVMXIMUYaw(int32_t* status);
/** Reset shared continuous angle. */
void HAL_ResetVMXIMU(int32_t* status);
/** Reset shared integrated displacement. */
void HAL_ResetVMXIMUDisplacement(int32_t* status);
/** Update displacement; acceleration is in g, rate is in Hz (1..200). */
void HAL_UpdateVMXIMUDisplacement(double accelX, double accelY, int32_t rate,
                                  HAL_Bool moving, int32_t* status);
/** Copy the NUL-terminated SDK firmware version into the caller's buffer.
 * Reports PARAMETER_OUT_OF_RANGE when capacity is insufficient.
 */
void HAL_GetVMXIMUFirmwareVersion(char* buffer, int32_t capacity,
                                  int32_t* status);

#ifdef __cplusplus
}  // extern "C"
#endif
