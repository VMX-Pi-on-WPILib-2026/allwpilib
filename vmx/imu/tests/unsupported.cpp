#include <cassert>
#include "hal/Errors.h"
#include "hal/VMXIMU.h"
int main() {
  int32_t status = 0;
  HAL_GetVMXIMUValue(HAL_VMXIMU_Yaw, &status);
  assert(status == INCOMPATIBLE_STATE);
  status = 0;
  HAL_ZeroVMXIMUYaw(&status);
  assert(status == INCOMPATIBLE_STATE);
  status = 0;
  HAL_ResetVMXIMU(&status);
  assert(status == INCOMPATIBLE_STATE);
  status = 0;
  HAL_ResetVMXIMUDisplacement(&status);
  assert(status == INCOMPATIBLE_STATE);
  status = 0;
  HAL_UpdateVMXIMUDisplacement(0, 0, 50, true, &status);
  assert(status == INCOMPATIBLE_STATE);
  char buffer[64];
  status = 0;
  HAL_GetVMXIMUFirmwareVersion(buffer, sizeof(buffer), &status);
  assert(status == INCOMPATIBLE_STATE);
}
