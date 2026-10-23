#include <atomic>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>
#include "AHRS.h"
#include "frc/VMXIMU.h"
#include "hal/Errors.h"
#include "hal/VMXIMU.h"
#include <wpi/sendable/SendableRegistry.h>

namespace hal::init {
std::atomic_bool HAL_IsInitialized{false};
}
namespace mau {
vmx::AHRS* vmxIMU = nullptr;
}
// Dashboard registry is unrelated to sensor transport/lifetime in this test.
void wpi::SendableRegistry::AddLW(wpi::Sendable*, std::string_view) {}
bool wpi::SendableRegistry::Remove(wpi::Sendable*) {
  return true;
}

int main() {
  int32_t status = 0;
  assert(HAL_GetVMXIMUValue(HAL_VMXIMU_Yaw, &status) == 0);
  assert(status == INCOMPATIBLE_STATE);
  try {
    frc::VMXIMU early;
    assert(false);
  } catch (const std::runtime_error&) {
  }
  hal::init::HAL_IsInitialized = true;
  HAL_ResetVMXIMU(&status);
  assert(status == INCOMPATIBLE_STATE);  // initialized but no SDK sensor

  vmx::AHRS device;
  mau::vmxIMU = &device;
  device.values["GetYaw"] = -179.5;
  device.values["GetPitch"] = 12.5;
  device.values["GetRoll"] = -24.5;
  device.values["GetAngle"] = 810.0;
  device.values["GetRate"] = -32.0;
  device.values["GetLastSensorTimestamp"] = 4294967297.0;
  device.values["GetWorldLinearAccelX"] = 0.25;
  device.values["GetRawAccelX"] = 1.25;
  device.values["GetActualUpdateRate"] = 50;
  device.values["GetRequestedUpdateRate"] = 50;
  device.values["IsCalibrating"] = 1;
  // Disconnected/calibrating is a readable condition, not failed construction.
  frc::VMXIMU first;
  assert(!first.IsConnected());
  assert(first.IsCalibrating());
  assert(first.GetYaw() == -179.5 && first.GetPitch() == 12.5);
  assert(first.GetRoll() == -24.5 && first.GetRate() == -32.0);
  assert(first.GetAngle() == 810.0);
  assert(first.GetLastSensorTimestamp() == 4294967297LL);
  assert(first.GetWorldLinearAccelX() == 0.25 && first.GetRawAccelX() == 1.25);
  assert(first.GetActualUpdateRate() == 50 &&
         first.GetRequestedUpdateRate() == 50);
  assert(std::abs(first.GetRotation2d().Degrees().value() + 810.0) < 1e-9);
  assert(first.GetBoardYawAxis() == 2 && first.IsBoardYawAxisUp());
  device.axis = {kBoardAxisX, false};
  assert(first.GetBoardYawAxis() == 0 && !first.IsBoardYawAxisUp());
  assert(first.GetFirmwareVersion() == "3.0.436");
  {
    frc::VMXIMU second;
    device.values["IsConnected"] = 1;
    assert(first.IsConnected() && second.IsConnected());
    second.ZeroYaw();
    assert(first.GetYaw() == 0 && device.zeroYawCalls == 1);
    second.Reset();
    assert(first.GetAngle() == 0 && device.resetCalls == 1);
    second.ResetDisplacement();
    assert(device.resetDisplacementCalls == 1);
    second.UpdateDisplacement(0.25, -0.5, 50, true);
    assert(device.updateCalls == 1 && device.lastRate == 50);
    assert(device.lastX == 0.25 && device.lastY == -0.5 && device.lastMoving);
  }
  assert(device.stops == 0);  // destroying one view must not stop other readers
  assert(first.IsConnected());
  char buffer[8];
  HAL_GetVMXIMUFirmwareVersion(buffer, 8, &status);
  assert(status == 0 && std::string_view(buffer) == "3.0.436");
  char small[] = "xx";
  HAL_GetVMXIMUFirmwareVersion(small, 2, &status);
  assert(status == PARAMETER_OUT_OF_RANGE && std::string_view(small) == "xx");
  HAL_GetVMXIMUFirmwareVersion(nullptr, 8, &status);
  assert(status == NULL_PARAMETER);
  HAL_GetVMXIMUFirmwareVersion(buffer, -1, &status);
  assert(status == PARAMETER_OUT_OF_RANGE);
  HAL_GetVMXIMUValue(static_cast<HAL_VMXIMUProperty>(999), &status);
  assert(status == PARAMETER_OUT_OF_RANGE);
  HAL_UpdateVMXIMUDisplacement(0, 0, 0, true, &status);
  assert(status == PARAMETER_OUT_OF_RANGE);
  HAL_UpdateVMXIMUDisplacement(0, 0, 201, true, &status);
  assert(status == PARAMETER_OUT_OF_RANGE);
  HAL_UpdateVMXIMUDisplacement(std::numeric_limits<double>::quiet_NaN(), 0, 50,
                               true, &status);
  assert(status == PARAMETER_OUT_OF_RANGE);
  HAL_UpdateVMXIMUDisplacement(0, std::numeric_limits<double>::infinity(), 50,
                               true, &status);
  assert(status == PARAMETER_OUT_OF_RANGE && device.updateCalls == 1);
  hal::init::HAL_IsInitialized = false;
  try {
    first.GetYaw();
    assert(false);
  } catch (const std::runtime_error&) {
  }
}
