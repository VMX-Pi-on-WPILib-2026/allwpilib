// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>
#include <string>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include "frc/geometry/Rotation2d.h"

namespace frc {
/** VMX-pi onboard IMU accessed through the existing WPILib HAL.
 *
 * Construct after HAL initialization (e.g. in RobotInit). No navX vendordep,
 * SPI port allocation, second VMXPi instance, or extra polling thread is used.
 * The HAL owns the sensor for the process lifetime. Destroying this view does
 * not stop the shared sensor. Resets affect all views. Individual reads are
 * current SDK values, not an atomic multi-axis snapshot.
 * Angles/rates use the SDK's clockwise-positive degrees/degrees per second;
 * GetRotation2d uses WPILib's counterclockwise-positive convention.
 * Unsupported platforms and unavailable HAL throw std::runtime_error.
 */
class VMXIMU : public wpi::Sendable, public wpi::SendableHelper<VMXIMU> {
 public:
  VMXIMU();
  ~VMXIMU() override = default;
  VMXIMU(const VMXIMU&) = delete;
  VMXIMU& operator=(const VMXIMU&) = delete;
  /** Pitch in degrees. */
  double GetPitch() const;
  /** Yaw in degrees, from -180 to 180, clockwise positive. */
  double GetYaw() const;
  /** Roll in degrees. */
  double GetRoll() const;
  /** Compass heading in degrees, from 0 to 360. */
  double GetCompassHeading() const;
  /** Whether the IMU is calibrating. Keep the robot still. */
  bool IsCalibrating() const;
  /** Whether the VMX IMU is receiving updates. */
  bool IsConnected() const;
  /** Total bytes received by the SDK. */
  double GetByteCount() const;
  /** Total sensor updates received by the SDK. */
  double GetUpdateCount() const;
  /** Last sensor timestamp in milliseconds. */
  int64_t GetLastSensorTimestamp() const;
  /** Gravity-compensated world X acceleration in g. */
  double GetWorldLinearAccelX() const;
  /** Gravity-compensated world Y acceleration in g. */
  double GetWorldLinearAccelY() const;
  /** Gravity-compensated world Z acceleration in g. */
  double GetWorldLinearAccelZ() const;
  /** Whether acceleration indicates movement. */
  bool IsMoving() const;
  /** Whether the gyro indicates rotation. */
  bool IsRotating() const;
  /** Barometric pressure in millibars; requires supported hardware. */
  double GetBarometricPressure() const;
  /** Altitude in meters; check IsAltitudeValid first. */
  double GetAltitude() const;
  /** Whether altitude data is valid. */
  bool IsAltitudeValid() const;
  /** Fused heading in degrees, from 0 to 360. */
  double GetFusedHeading() const;
  /** Whether a magnetic disturbance is detected. */
  bool IsMagneticDisturbance() const;
  /** Whether the magnetometer is calibrated. */
  bool IsMagnetometerCalibrated() const;
  /** Unit quaternion W component. */
  double GetQuaternionW() const;
  /** Unit quaternion X component. */
  double GetQuaternionX() const;
  /** Unit quaternion Y component. */
  double GetQuaternionY() const;
  /** Unit quaternion Z component. */
  double GetQuaternionZ() const;
  /** Integrated X velocity in meters per second. */
  double GetVelocityX() const;
  /** Integrated Y velocity in meters per second. */
  double GetVelocityY() const;
  /** Integrated Z velocity in meters per second. */
  double GetVelocityZ() const;
  /** Integrated X displacement in meters. */
  double GetDisplacementX() const;
  /** Integrated Y displacement in meters. */
  double GetDisplacementY() const;
  /** Integrated Z displacement in meters. */
  double GetDisplacementZ() const;
  /** Continuous accumulated yaw in degrees, clockwise positive. */
  double GetAngle() const;
  /** Yaw rate in degrees per second, clockwise positive. */
  double GetRate() const;
  /** Sensor-frame X angular velocity in degrees per second. */
  double GetRawGyroX() const;
  /** Sensor-frame Y angular velocity in degrees per second. */
  double GetRawGyroY() const;
  /** Sensor-frame Z angular velocity in degrees per second. */
  double GetRawGyroZ() const;
  /** Sensor-frame X acceleration including gravity in g. */
  double GetRawAccelX() const;
  /** Sensor-frame Y acceleration including gravity in g. */
  double GetRawAccelY() const;
  /** Sensor-frame Z acceleration including gravity in g. */
  double GetRawAccelZ() const;
  /** Sensor-frame X magnetic field in microteslas. */
  double GetRawMagX() const;
  /** Sensor-frame Y magnetic field in microteslas. */
  double GetRawMagY() const;
  /** Sensor-frame Z magnetic field in microteslas. */
  double GetRawMagZ() const;
  /** Pressure in millibars; requires supported hardware. */
  double GetPressure() const;
  /** IMU temperature in degrees Celsius. */
  double GetTempC() const;
  /** Actual IMU update rate in Hz. */
  int GetActualUpdateRate() const;
  /** HAL-configured IMU update rate in Hz (currently 50 Hz). */
  int GetRequestedUpdateRate() const;
  /** Omnimount yaw axis: 0 = X, 1 = Y, 2 = Z. */
  int GetBoardYawAxis() const;
  /** Whether the configured yaw axis points up. */
  bool IsBoardYawAxisUp() const;
  /** Zero shared yaw. */
  void ZeroYaw();
  /** Reset shared continuous yaw angle. */
  void Reset();
  /** Reset shared velocity/displacement integrators. */
  void ResetDisplacement();
  /** Update shared integration with acceleration in g and rate in Hz (1..200).
   * Normally the SDK updates integration; call only for manual integration.
   */
  void UpdateDisplacement(double accelX, double accelY, int rate, bool moving);
  /** Firmware version string from the VMX SDK. */
  std::string GetFirmwareVersion() const;
  /** Counterclockwise-positive rotation derived from continuous angle. */
  Rotation2d GetRotation2d() const;
  void InitSendable(wpi::SendableBuilder& builder) override;
};
}  // namespace frc
