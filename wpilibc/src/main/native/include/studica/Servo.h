// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#pragma once
#include <frc/PWM.h>
#include <frc/smartdashboard/SendableBuilder.h>
namespace studica {
/** Studica positional servo: 0..300 degrees, 500..2500 microseconds. */
class Servo : public frc::PWM {
 public:
  explicit Servo(int channel);
  Servo(Servo&&) = default;
  Servo& operator=(Servo&&) = default;
  void Set(double value);
  void SetOffline();
  double Get() const;
  void SetAngle(double angle);
  double GetAngle() const;
  double GetMaxAngle() const;
  double GetMinAngle() const;
  void InitSendable(wpi::SendableBuilder& builder) override;
};
}  // namespace studica
