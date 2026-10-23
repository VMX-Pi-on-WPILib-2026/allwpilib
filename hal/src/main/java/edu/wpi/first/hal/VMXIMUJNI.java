// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.hal;

/** HAL access to the shared onboard VMX IMU. */
public final class VMXIMUJNI extends JNIWrapper {
  public static final int Pitch = 0;
  public static final int Yaw = 1;
  public static final int Roll = 2;
  public static final int CompassHeading = 3;
  public static final int IsCalibrating = 4;
  public static final int IsConnected = 5;
  public static final int ByteCount = 6;
  public static final int UpdateCount = 7;
  public static final int LastSensorTimestamp = 8;
  public static final int WorldLinearAccelX = 9;
  public static final int WorldLinearAccelY = 10;
  public static final int WorldLinearAccelZ = 11;
  public static final int IsMoving = 12;
  public static final int IsRotating = 13;
  public static final int BarometricPressure = 14;
  public static final int Altitude = 15;
  public static final int IsAltitudeValid = 16;
  public static final int FusedHeading = 17;
  public static final int IsMagneticDisturbance = 18;
  public static final int IsMagnetometerCalibrated = 19;
  public static final int QuaternionW = 20;
  public static final int QuaternionX = 21;
  public static final int QuaternionY = 22;
  public static final int QuaternionZ = 23;
  public static final int VelocityX = 24;
  public static final int VelocityY = 25;
  public static final int VelocityZ = 26;
  public static final int DisplacementX = 27;
  public static final int DisplacementY = 28;
  public static final int DisplacementZ = 29;
  public static final int Angle = 30;
  public static final int Rate = 31;
  public static final int RawGyroX = 32;
  public static final int RawGyroY = 33;
  public static final int RawGyroZ = 34;
  public static final int RawAccelX = 35;
  public static final int RawAccelY = 36;
  public static final int RawAccelZ = 37;
  public static final int RawMagX = 38;
  public static final int RawMagY = 39;
  public static final int RawMagZ = 40;
  public static final int Pressure = 41;
  public static final int TempC = 42;
  public static final int ActualUpdateRate = 43;
  public static final int RequestedUpdateRate = 44;
  public static final int BoardYawAxis = 45;
  public static final int IsBoardYawAxisUp = 46;

  public static native double getValue(int property);
  public static native void zeroYaw();
  public static native void reset();
  public static native void resetDisplacement();
  public static native void updateDisplacement(double accelX, double accelY, int rate, boolean moving);
  public static native String getFirmwareVersion();
  private VMXIMUJNI() {}
}
