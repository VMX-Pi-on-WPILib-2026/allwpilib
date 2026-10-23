// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
package com.studica.frc;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

/** Studica continuous servo using the 2026 PWM motor interface. */
public class ServoContinuous extends PWMMotorController {
  public ServoContinuous(int channel) {
    super("Studica ServoContinuous", channel);
    m_pwm.setBoundsMicroseconds(2500, 1550, 1500, 1450, 500);
    m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
    m_pwm.setSpeed(0);
    m_pwm.setZeroLatch();
  }
}
