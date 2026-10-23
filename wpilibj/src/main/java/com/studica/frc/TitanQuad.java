// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
package com.studica.frc;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Legacy-compatible Titan Quad motor channel. Refresh set() within 100 ms. Motor commands require a
 * Driver Station that is attached and enabled.
 */
public class TitanQuad implements SpeedController, AutoCloseable {
  private final TitanBus bus;
  private final int motor;
  private boolean inverted;
  private boolean invertRPM;
  private volatile boolean closed;

  public TitanQuad(int deviceID, int motor) {
    this(deviceID, 15600, motor);
  }

  public TitanQuad(int deviceID, int frequency, int motor) {
    TitanBus.channel(motor);
    if (frequency < 0 || frequency > 20000)
      throw new IllegalArgumentException("Titan frequency must be 0..20000 Hz");
    this.motor = motor;
    bus = TitanBus.acquire(deviceID);
    try {
      bus.register(motor, frequency);
    } catch (RuntimeException ex) {
      TitanBus.release(bus);
      throw ex;
    }
  }

  TitanBus device() {
    checkOpen();
    return bus;
  }

  private void checkOpen() {
    if (closed) throw new IllegalStateException("Titan is closed");
  }

  public int getID() {
    return device().id;
  }

  public int getFrequency() {
    byte[] d = device().request(11, 12, 2, new byte[] {(byte) motor, 0, 0, 0, 0, 0, 0, 0});
    return (d[0] & 255) | ((d[1] & 255) << 8);
  }

  public String getFirmwareVersion() {
    byte[] d = device().request(4, 5, 6, new byte[8]);
    return "Firmware Version: [" + (d[1] & 255) + "." + (d[2] & 255) + "." + (d[3] & 255) + "]";
  }

  public String getHardwareVersion() {
    byte[] d = device().request(4, 5, 6, new byte[8]);
    String type =
        (d[4] & 255) == 1
            ? "Titan Quad"
            : (d[4] & 255) == 2 ? "Titan Small" : "Type " + (d[4] & 255);
    return "Hardware: " + type + ", Version: " + (d[5] & 255);
  }

  public double getControllerTemp() {
    byte[] d = device().read(47, 2);
    return (d[0] & 255) + (d[1] & 255) / 100.0;
  }

  @Override
  public synchronized void set(double speed) {
    device().set(motor, inverted ? -speed : speed);
  }

  @Override
  public synchronized double get() {
    return device().get(motor);
  }

  @Override
  public synchronized void setInverted(boolean value) {
    inverted = value;
  }

  @Override
  public synchronized boolean getInverted() {
    return inverted;
  }

  @Override
  public synchronized void disable() {
    device().stop(motor);
  }

  @Override
  public void stopMotor() {
    disable();
  }

  @Override
  public void pidWrite(double output) {
    set(output);
  }

  public double getLimitSwitch(int motor, boolean direction) {
    TitanBus.channel(motor);
    return device().read(45, 8)[motor * 2 + (direction ? 1 : 0)] == 0 ? 0 : 1;
  }

  public void invertRPM() {
    invertRPM = true;
  }

  public int getRPM() {
    return (int) (device().rpm(motor) * (invertRPM ? -1 : 1));
  }

  public String getSerialNumber() {
    StringBuilder result = new StringBuilder();
    for (int api = 7; api <= 9; api++) {
      if (api != 7) result.append('-');
      result.append(
          String.format(
              java.util.Locale.ROOT,
              "%08X",
              TitanBus.int32(device().request(6, api, 4, new byte[8]))));
    }
    return result.toString();
  }

  @Override
  public synchronized void close() {
    if (closed) return;
    closed = true;
    try {
      bus.remove(motor);
    } finally {
      TitanBus.release(bus);
    }
  }
}
