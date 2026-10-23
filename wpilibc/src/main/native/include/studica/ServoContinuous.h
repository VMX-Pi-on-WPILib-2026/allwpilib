// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#pragma once
#include <frc/motorcontrol/PWMMotorController.h>
namespace studica {
/** Studica continuous servo. Legacy constructor and motor methods are retained.
 */
class ServoContinuous : public frc::PWMMotorController {
 public:
  explicit ServoContinuous(int channel);
  ServoContinuous(ServoContinuous&&) = default;
  ServoContinuous& operator=(ServoContinuous&&) = default;
};
}  // namespace studica
