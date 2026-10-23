// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#pragma once
#include <memory>
#include <string>
#include <frc/SpeedController.h>
namespace studica {
namespace detail {
class TitanDevice;
}
/** One motor channel (0..3) of a Titan Quad. CAN device IDs are 1..63.
 * Refresh Set() at least every 100 ms while DS-attached and enabled.
 * All channels on one CAN ID share the same HAL transport and device lifetime.
 */
class TitanQuad : public frc::SpeedController {
 public:
  TitanQuad(int deviceID, int motor_set);
  TitanQuad(int deviceID, int frequency_set, int motor_set);
  ~TitanQuad() override;
  TitanQuad(const TitanQuad&) = delete;
  TitanQuad& operator=(const TitanQuad&) = delete;
  int GetID();
  int GetFrequency();
  std::string GetFirmwareVersion();
  std::string GetHardwareVersion();
  double GetControllerTemp();
  void Set(double speed) override;
  double Get() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  void Disable() override;
  void StopMotor() override;
  void PIDWrite(double output) override;
  double GetLimitSwitch(bool direction);
  void InvertRPM();
  int GetRPM();
  std::string GetSerialNumber();

 private:
  friend class TitanQuadEncoder;
  std::shared_ptr<detail::TitanDevice> m_device;
  int m_motor;
  bool m_inverted = false;
  bool m_invertRPM = false;
};
}  // namespace studica
