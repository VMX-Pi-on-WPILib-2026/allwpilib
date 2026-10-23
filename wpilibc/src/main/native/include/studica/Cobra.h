// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#pragma once
#include <memory>
namespace studica {
/** Cobra ADS1015 ADC at I2C address 0x48 on MXP. Channels are 0..3. */
class Cobra {
 public:
  Cobra();
  explicit Cobra(float vRef);
  ~Cobra();
  Cobra(const Cobra&) = delete;
  Cobra& operator=(const Cobra&) = delete;
  int GetRawValue(int channel);
  float GetVoltage(int channel);

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
  float m_vRef;
};
}  // namespace studica
