// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
package com.studica.frc;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PWM;

/** Studica positional servo: 0..300 degrees, 500..2500 microseconds. */
public class Servo extends PWM {
  protected static final double kDefaultMaxServoPWM = 2.5;
  protected static final double kDefaultMinServoPWM = 0.5;

  @SuppressWarnings("this-escape")
  public Servo(int channel) {
    super(channel);
    setBoundsMicroseconds(2500, 0, 0, 0, 500);
    setPeriodMultiplier(PeriodMultiplier.k4X);
    setDisabled();
    SendableRegistry.setName(this, "Studica Servo", channel);
  }

  public void set(double value) {
    if (!Double.isFinite(value)) {
      setDisabled();
      throw new IllegalArgumentException("Servo position must be finite");
    }
    setPosition(Math.max(0, Math.min(1, value)));
  }

  public double get() {
    return getPosition();
  }

  public void setAngle(double degrees) {
    set(degrees / 300);
  }

  public double getAngle() {
    return getPosition() * 300;
  }

  public void setOffline() {
    setDisabled();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Servo");
    builder.addDoubleProperty("Value", this::get, this::set);
  }

  /** Legacy SendableBuilder package overload. */
  public void initSendable(edu.wpi.first.wpilibj.smartdashboard.SendableBuilder builder) {
    initSendable((SendableBuilder) builder);
  }
}
