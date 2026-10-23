// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

import edu.wpi.first.hal.VMXIMUJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * View of the HAL-owned onboard VMX-pi IMU. Construct after HAL initialization.
 * No navX vendordep, SPI allocation or additional VMX connection is needed.
 * Closing this view never stops the shared IMU; resets affect every reader.
 * Angles are clockwise-positive degrees, rates degrees/second, accelerations g,
 * velocities m/s and displacements meters. Individual reads are not an atomic
 * snapshot. Unsupported platforms or unavailable HAL throw a HAL exception.
 */
public final class VMXIMU implements Sendable, AutoCloseable {
  private boolean closed;

  public VMXIMU() {
    VMXIMUJNI.getValue(VMXIMUJNI.IsConnected);
    SendableRegistry.addLW(this, "VMXIMU");
  }

  private void checkOpen() {
    if (closed) throw new IllegalStateException("VMXIMU view is closed");
  }

  private double read(int property) {
    checkOpen();
    return VMXIMUJNI.getValue(property);
  }

  /** Pitch in degrees. */
  public double getPitch() {
    return read(VMXIMUJNI.Pitch);
  }

  /** Yaw in degrees, from -180 to 180, clockwise positive. */
  public double getYaw() {
    return read(VMXIMUJNI.Yaw);
  }

  /** Roll in degrees. */
  public double getRoll() {
    return read(VMXIMUJNI.Roll);
  }

  /** Compass heading in degrees, from 0 to 360. */
  public double getCompassHeading() {
    return read(VMXIMUJNI.CompassHeading);
  }

  /** Whether the IMU is calibrating. Keep the robot still. */
  public boolean isCalibrating() {
    return read(VMXIMUJNI.IsCalibrating) != 0;
  }

  /** Whether the VMX IMU is receiving updates. */
  public boolean isConnected() {
    return read(VMXIMUJNI.IsConnected) != 0;
  }

  /** Total bytes received by the SDK. */
  public double getByteCount() {
    return read(VMXIMUJNI.ByteCount);
  }

  /** Total sensor updates received by the SDK. */
  public double getUpdateCount() {
    return read(VMXIMUJNI.UpdateCount);
  }

  /** Last sensor timestamp in milliseconds. */
  public long getLastSensorTimestamp() {
    return (long) read(VMXIMUJNI.LastSensorTimestamp);
  }

  /** Gravity-compensated world X acceleration in g. */
  public double getWorldLinearAccelX() {
    return read(VMXIMUJNI.WorldLinearAccelX);
  }

  /** Gravity-compensated world Y acceleration in g. */
  public double getWorldLinearAccelY() {
    return read(VMXIMUJNI.WorldLinearAccelY);
  }

  /** Gravity-compensated world Z acceleration in g. */
  public double getWorldLinearAccelZ() {
    return read(VMXIMUJNI.WorldLinearAccelZ);
  }

  /** Whether acceleration indicates movement. */
  public boolean isMoving() {
    return read(VMXIMUJNI.IsMoving) != 0;
  }

  /** Whether the gyro indicates rotation. */
  public boolean isRotating() {
    return read(VMXIMUJNI.IsRotating) != 0;
  }

  /** Barometric pressure in millibars; requires supported hardware. */
  public double getBarometricPressure() {
    return read(VMXIMUJNI.BarometricPressure);
  }

  /** Altitude in meters; check isAltitudeValid first. */
  public double getAltitude() {
    return read(VMXIMUJNI.Altitude);
  }

  /** Whether altitude data is valid. */
  public boolean isAltitudeValid() {
    return read(VMXIMUJNI.IsAltitudeValid) != 0;
  }

  /** Fused heading in degrees, from 0 to 360. */
  public double getFusedHeading() {
    return read(VMXIMUJNI.FusedHeading);
  }

  /** Whether a magnetic disturbance is detected. */
  public boolean isMagneticDisturbance() {
    return read(VMXIMUJNI.IsMagneticDisturbance) != 0;
  }

  /** Whether the magnetometer is calibrated. */
  public boolean isMagnetometerCalibrated() {
    return read(VMXIMUJNI.IsMagnetometerCalibrated) != 0;
  }

  /** Unit quaternion W component. */
  public double getQuaternionW() {
    return read(VMXIMUJNI.QuaternionW);
  }

  /** Unit quaternion X component. */
  public double getQuaternionX() {
    return read(VMXIMUJNI.QuaternionX);
  }

  /** Unit quaternion Y component. */
  public double getQuaternionY() {
    return read(VMXIMUJNI.QuaternionY);
  }

  /** Unit quaternion Z component. */
  public double getQuaternionZ() {
    return read(VMXIMUJNI.QuaternionZ);
  }

  /** Integrated X velocity in meters per second. */
  public double getVelocityX() {
    return read(VMXIMUJNI.VelocityX);
  }

  /** Integrated Y velocity in meters per second. */
  public double getVelocityY() {
    return read(VMXIMUJNI.VelocityY);
  }

  /** Integrated Z velocity in meters per second. */
  public double getVelocityZ() {
    return read(VMXIMUJNI.VelocityZ);
  }

  /** Integrated X displacement in meters. */
  public double getDisplacementX() {
    return read(VMXIMUJNI.DisplacementX);
  }

  /** Integrated Y displacement in meters. */
  public double getDisplacementY() {
    return read(VMXIMUJNI.DisplacementY);
  }

  /** Integrated Z displacement in meters. */
  public double getDisplacementZ() {
    return read(VMXIMUJNI.DisplacementZ);
  }

  /** Continuous accumulated yaw in degrees, clockwise positive. */
  public double getAngle() {
    return read(VMXIMUJNI.Angle);
  }

  /** Yaw rate in degrees per second, clockwise positive. */
  public double getRate() {
    return read(VMXIMUJNI.Rate);
  }

  /** Sensor-frame X angular velocity in degrees per second. */
  public double getRawGyroX() {
    return read(VMXIMUJNI.RawGyroX);
  }

  /** Sensor-frame Y angular velocity in degrees per second. */
  public double getRawGyroY() {
    return read(VMXIMUJNI.RawGyroY);
  }

  /** Sensor-frame Z angular velocity in degrees per second. */
  public double getRawGyroZ() {
    return read(VMXIMUJNI.RawGyroZ);
  }

  /** Sensor-frame X acceleration including gravity in g. */
  public double getRawAccelX() {
    return read(VMXIMUJNI.RawAccelX);
  }

  /** Sensor-frame Y acceleration including gravity in g. */
  public double getRawAccelY() {
    return read(VMXIMUJNI.RawAccelY);
  }

  /** Sensor-frame Z acceleration including gravity in g. */
  public double getRawAccelZ() {
    return read(VMXIMUJNI.RawAccelZ);
  }

  /** Sensor-frame X magnetic field in microteslas. */
  public double getRawMagX() {
    return read(VMXIMUJNI.RawMagX);
  }

  /** Sensor-frame Y magnetic field in microteslas. */
  public double getRawMagY() {
    return read(VMXIMUJNI.RawMagY);
  }

  /** Sensor-frame Z magnetic field in microteslas. */
  public double getRawMagZ() {
    return read(VMXIMUJNI.RawMagZ);
  }

  /** Pressure in millibars; requires supported hardware. */
  public double getPressure() {
    return read(VMXIMUJNI.Pressure);
  }

  /** IMU temperature in degrees Celsius. */
  public double getTempC() {
    return read(VMXIMUJNI.TempC);
  }

  /** Actual IMU update rate in Hz. */
  public int getActualUpdateRate() {
    return (int) read(VMXIMUJNI.ActualUpdateRate);
  }

  /** HAL-configured IMU update rate in Hz (currently 50 Hz). */
  public int getRequestedUpdateRate() {
    return (int) read(VMXIMUJNI.RequestedUpdateRate);
  }

  /** Omnimount yaw axis: 0 = X, 1 = Y, 2 = Z. */
  public int getBoardYawAxis() {
    return (int) read(VMXIMUJNI.BoardYawAxis);
  }

  /** Whether the configured yaw axis points up. */
  public boolean isBoardYawAxisUp() {
    return read(VMXIMUJNI.IsBoardYawAxisUp) != 0;
  }

  /** Reset the corresponding shared IMU state. */
  public void zeroYaw() {
    checkOpen();
    VMXIMUJNI.zeroYaw();
  }

  /** Reset the corresponding shared IMU state. */
  public void reset() {
    checkOpen();
    VMXIMUJNI.reset();
  }

  /** Reset the corresponding shared IMU state. */
  public void resetDisplacement() {
    checkOpen();
    VMXIMUJNI.resetDisplacement();
  }

  /** Manual integration with acceleration in g and update rate in Hz (1..200). */
  public void updateDisplacement(double accelX, double accelY, int rate, boolean moving) {
    checkOpen();
    VMXIMUJNI.updateDisplacement(accelX, accelY, rate, moving);
  }

  /** Returns the SDK firmware version. */
  public String getFirmwareVersion() {
    checkOpen();
    return VMXIMUJNI.getFirmwareVersion();
  }

  /** Returns counterclockwise-positive WPILib rotation. */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-getAngle());
  }

  /** Closes only this view, without stopping the HAL-owned sensor. */
  @Override
  public void close() {
    closed = true;
    SendableRegistry.remove(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", this::getAngle, null);
  }
}
