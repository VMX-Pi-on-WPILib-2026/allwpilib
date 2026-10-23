// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/VMXIMU.h"
#include "hal/Errors.h"

// A VMX IMU is not present on roboRIO or desktop. Fail explicitly.
extern "C" {
double HAL_GetVMXIMUValue(HAL_VMXIMUProperty, int32_t* status) {
  *status = INCOMPATIBLE_STATE;
  return 0;
}
void HAL_ZeroVMXIMUYaw(int32_t* status) {
  *status = INCOMPATIBLE_STATE;
}
void HAL_ResetVMXIMU(int32_t* status) {
  *status = INCOMPATIBLE_STATE;
}
void HAL_ResetVMXIMUDisplacement(int32_t* status) {
  *status = INCOMPATIBLE_STATE;
}
void HAL_UpdateVMXIMUDisplacement(double, double, int32_t, HAL_Bool,
                                  int32_t* status) {
  *status = INCOMPATIBLE_STATE;
}
void HAL_GetVMXIMUFirmwareVersion(char*, int32_t, int32_t* status) {
  *status = INCOMPATIBLE_STATE;
}
}  // extern "C"
