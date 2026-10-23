// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/VMXIMU.h"

#include <hal/VMXIMU.h>
#include <stdexcept>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

namespace {
void Check(int32_t status) {
  if (status != 0)
    throw std::runtime_error("VMXIMU HAL operation failed; status=" +
                             std::to_string(status));
}
double Read(HAL_VMXIMUProperty property) {
  int32_t status = 0;
  double value = HAL_GetVMXIMUValue(property, &status);
  Check(status);
  return value;
}
}  // namespace
namespace frc {
VMXIMU::VMXIMU() {
  // Check the HAL, but allow a temporarily disconnected/calibrating sensor.
  Read(HAL_VMXIMU_IsConnected);
  wpi::SendableRegistry::AddLW(this, "VMXIMU");
}
double VMXIMU::GetPitch() const {
  return static_cast<double>(Read(HAL_VMXIMU_Pitch));
}
double VMXIMU::GetYaw() const {
  return static_cast<double>(Read(HAL_VMXIMU_Yaw));
}
double VMXIMU::GetRoll() const {
  return static_cast<double>(Read(HAL_VMXIMU_Roll));
}
double VMXIMU::GetCompassHeading() const {
  return static_cast<double>(Read(HAL_VMXIMU_CompassHeading));
}
bool VMXIMU::IsCalibrating() const {
  return static_cast<bool>(Read(HAL_VMXIMU_IsCalibrating));
}
bool VMXIMU::IsConnected() const {
  return static_cast<bool>(Read(HAL_VMXIMU_IsConnected));
}
double VMXIMU::GetByteCount() const {
  return static_cast<double>(Read(HAL_VMXIMU_ByteCount));
}
double VMXIMU::GetUpdateCount() const {
  return static_cast<double>(Read(HAL_VMXIMU_UpdateCount));
}
int64_t VMXIMU::GetLastSensorTimestamp() const {
  return static_cast<int64_t>(Read(HAL_VMXIMU_LastSensorTimestamp));
}
double VMXIMU::GetWorldLinearAccelX() const {
  return static_cast<double>(Read(HAL_VMXIMU_WorldLinearAccelX));
}
double VMXIMU::GetWorldLinearAccelY() const {
  return static_cast<double>(Read(HAL_VMXIMU_WorldLinearAccelY));
}
double VMXIMU::GetWorldLinearAccelZ() const {
  return static_cast<double>(Read(HAL_VMXIMU_WorldLinearAccelZ));
}
bool VMXIMU::IsMoving() const {
  return static_cast<bool>(Read(HAL_VMXIMU_IsMoving));
}
bool VMXIMU::IsRotating() const {
  return static_cast<bool>(Read(HAL_VMXIMU_IsRotating));
}
double VMXIMU::GetBarometricPressure() const {
  return static_cast<double>(Read(HAL_VMXIMU_BarometricPressure));
}
double VMXIMU::GetAltitude() const {
  return static_cast<double>(Read(HAL_VMXIMU_Altitude));
}
bool VMXIMU::IsAltitudeValid() const {
  return static_cast<bool>(Read(HAL_VMXIMU_IsAltitudeValid));
}
double VMXIMU::GetFusedHeading() const {
  return static_cast<double>(Read(HAL_VMXIMU_FusedHeading));
}
bool VMXIMU::IsMagneticDisturbance() const {
  return static_cast<bool>(Read(HAL_VMXIMU_IsMagneticDisturbance));
}
bool VMXIMU::IsMagnetometerCalibrated() const {
  return static_cast<bool>(Read(HAL_VMXIMU_IsMagnetometerCalibrated));
}
double VMXIMU::GetQuaternionW() const {
  return static_cast<double>(Read(HAL_VMXIMU_QuaternionW));
}
double VMXIMU::GetQuaternionX() const {
  return static_cast<double>(Read(HAL_VMXIMU_QuaternionX));
}
double VMXIMU::GetQuaternionY() const {
  return static_cast<double>(Read(HAL_VMXIMU_QuaternionY));
}
double VMXIMU::GetQuaternionZ() const {
  return static_cast<double>(Read(HAL_VMXIMU_QuaternionZ));
}
double VMXIMU::GetVelocityX() const {
  return static_cast<double>(Read(HAL_VMXIMU_VelocityX));
}
double VMXIMU::GetVelocityY() const {
  return static_cast<double>(Read(HAL_VMXIMU_VelocityY));
}
double VMXIMU::GetVelocityZ() const {
  return static_cast<double>(Read(HAL_VMXIMU_VelocityZ));
}
double VMXIMU::GetDisplacementX() const {
  return static_cast<double>(Read(HAL_VMXIMU_DisplacementX));
}
double VMXIMU::GetDisplacementY() const {
  return static_cast<double>(Read(HAL_VMXIMU_DisplacementY));
}
double VMXIMU::GetDisplacementZ() const {
  return static_cast<double>(Read(HAL_VMXIMU_DisplacementZ));
}
double VMXIMU::GetAngle() const {
  return static_cast<double>(Read(HAL_VMXIMU_Angle));
}
double VMXIMU::GetRate() const {
  return static_cast<double>(Read(HAL_VMXIMU_Rate));
}
double VMXIMU::GetRawGyroX() const {
  return static_cast<double>(Read(HAL_VMXIMU_RawGyroX));
}
double VMXIMU::GetRawGyroY() const {
  return static_cast<double>(Read(HAL_VMXIMU_RawGyroY));
}
double VMXIMU::GetRawGyroZ() const {
  return static_cast<double>(Read(HAL_VMXIMU_RawGyroZ));
}
double VMXIMU::GetRawAccelX() const {
  return static_cast<double>(Read(HAL_VMXIMU_RawAccelX));
}
double VMXIMU::GetRawAccelY() const {
  return static_cast<double>(Read(HAL_VMXIMU_RawAccelY));
}
double VMXIMU::GetRawAccelZ() const {
  return static_cast<double>(Read(HAL_VMXIMU_RawAccelZ));
}
double VMXIMU::GetRawMagX() const {
  return static_cast<double>(Read(HAL_VMXIMU_RawMagX));
}
double VMXIMU::GetRawMagY() const {
  return static_cast<double>(Read(HAL_VMXIMU_RawMagY));
}
double VMXIMU::GetRawMagZ() const {
  return static_cast<double>(Read(HAL_VMXIMU_RawMagZ));
}
double VMXIMU::GetPressure() const {
  return static_cast<double>(Read(HAL_VMXIMU_Pressure));
}
double VMXIMU::GetTempC() const {
  return static_cast<double>(Read(HAL_VMXIMU_TempC));
}
int VMXIMU::GetActualUpdateRate() const {
  return static_cast<int>(Read(HAL_VMXIMU_ActualUpdateRate));
}
int VMXIMU::GetRequestedUpdateRate() const {
  return static_cast<int>(Read(HAL_VMXIMU_RequestedUpdateRate));
}
int VMXIMU::GetBoardYawAxis() const {
  return static_cast<int>(Read(HAL_VMXIMU_BoardYawAxis));
}
bool VMXIMU::IsBoardYawAxisUp() const {
  return static_cast<bool>(Read(HAL_VMXIMU_IsBoardYawAxisUp));
}
void VMXIMU::ZeroYaw() {
  int32_t status = 0;
  HAL_ZeroVMXIMUYaw(&status);
  Check(status);
}
void VMXIMU::Reset() {
  int32_t status = 0;
  HAL_ResetVMXIMU(&status);
  Check(status);
}
void VMXIMU::ResetDisplacement() {
  int32_t status = 0;
  HAL_ResetVMXIMUDisplacement(&status);
  Check(status);
}
void VMXIMU::UpdateDisplacement(double accelX, double accelY, int rate,
                                bool moving) {
  int32_t status = 0;
  HAL_UpdateVMXIMUDisplacement(accelX, accelY, rate, moving, &status);
  Check(status);
}
std::string VMXIMU::GetFirmwareVersion() const {
  char version[64]{};
  int32_t status = 0;
  HAL_GetVMXIMUFirmwareVersion(version, sizeof(version), &status);
  Check(status);
  return version;
}
Rotation2d VMXIMU::GetRotation2d() const {
  return Rotation2d{units::degree_t{-GetAngle()}};
}
void VMXIMU::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Gyro");
  builder.AddDoubleProperty("Value", [this] { return GetAngle(); }, nullptr);
}
}  // namespace frc
