// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md.
package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** Source compatibility for legacy Studica motor controllers. */
public interface SpeedController extends MotorController {
  /** Applies a legacy PID output as a motor command. */
  default void pidWrite(double output) {
    set(output);
  }
}
