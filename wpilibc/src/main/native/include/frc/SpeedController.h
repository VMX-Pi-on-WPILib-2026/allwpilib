// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md.
#pragma once
#include "frc/motorcontrol/MotorController.h"
namespace frc {
/** Source compatibility for pre-2022 Studica motor controllers. */
class SpeedController : public MotorController {
 public:
  virtual void PIDWrite(double output) { Set(output); }
};
}  // namespace frc
