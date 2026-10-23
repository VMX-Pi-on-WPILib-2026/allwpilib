// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#include "studica/Servo.h"
#include "studica/ServoContinuous.h"
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace studica {
Servo::Servo(int channel) : PWM(channel) {
  SetBounds(2.5_ms, 0_ms, 0_ms, 0_ms, 0.5_ms);
  SetPeriodMultiplier(kPeriodMultiplier_4X);
  SetDisabled();
  wpi::SendableRegistry::SetName(this, "Studica Servo", channel);
}
void Servo::Set(double value) {
  if (!std::isfinite(value)) {
    SetDisabled();
    throw std::invalid_argument("Servo position must be finite");
  }
  SetPosition(std::clamp(value, 0.0, 1.0));
}
void Servo::SetOffline() {
  SetDisabled();
}
double Servo::Get() const {
  return GetPosition();
}
void Servo::SetAngle(double angle) {
  Set(angle / 300.0);
}
double Servo::GetAngle() const {
  return GetPosition() * 300;
}
double Servo::GetMaxAngle() const {
  return 300;
}
double Servo::GetMinAngle() const {
  return 0;
}
void Servo::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Servo");
  builder.AddDoubleProperty(
      "Value", [this] { return Get(); }, [this](double v) { Set(v); });
}
ServoContinuous::ServoContinuous(int channel)
    : PWMMotorController("Studica ServoContinuous", channel) {
  m_pwm.SetBounds(2.5_ms, 1.55_ms, 1.5_ms, 1.45_ms, 0.5_ms);
  m_pwm.SetPeriodMultiplier(frc::PWM::kPeriodMultiplier_4X);
  m_pwm.SetSpeed(0);
  m_pwm.SetZeroLatch();
}
}  // namespace studica
