// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>
#include "edu_wpi_first_hal_VMXIMUJNI.h"
#include "HALUtil.h"
#include "hal/VMXIMU.h"

extern "C" {
JNIEXPORT jdouble JNICALL
Java_edu_wpi_first_hal_VMXIMUJNI_getValue(JNIEnv* env, jclass, jint property) {
  int32_t status = 0;
  auto value =
      HAL_GetVMXIMUValue(static_cast<HAL_VMXIMUProperty>(property), &status);
  hal::CheckStatusForceThrow(env, status);
  return value;
}
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_VMXIMUJNI_zeroYaw(JNIEnv* env,
                                                                jclass) {
  int32_t status = 0;
  HAL_ZeroVMXIMUYaw(&status);
  hal::CheckStatusForceThrow(env, status);
}
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_VMXIMUJNI_reset(JNIEnv* env,
                                                              jclass) {
  int32_t status = 0;
  HAL_ResetVMXIMU(&status);
  hal::CheckStatusForceThrow(env, status);
}
JNIEXPORT void JNICALL
Java_edu_wpi_first_hal_VMXIMUJNI_resetDisplacement(JNIEnv* env, jclass) {
  int32_t status = 0;
  HAL_ResetVMXIMUDisplacement(&status);
  hal::CheckStatusForceThrow(env, status);
}
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_VMXIMUJNI_updateDisplacement(
    JNIEnv* env, jclass, jdouble x, jdouble y, jint rate, jboolean moving) {
  int32_t status = 0;
  HAL_UpdateVMXIMUDisplacement(x, y, rate, moving, &status);
  hal::CheckStatusForceThrow(env, status);
}
JNIEXPORT jstring JNICALL
Java_edu_wpi_first_hal_VMXIMUJNI_getFirmwareVersion(JNIEnv* env, jclass) {
  char version[64]{};
  int32_t status = 0;
  HAL_GetVMXIMUFirmwareVersion(version, sizeof(version), &status);
  if (!hal::CheckStatusForceThrow(env, status))
    return nullptr;
  return env->NewStringUTF(version);
}
}  // extern "C"
