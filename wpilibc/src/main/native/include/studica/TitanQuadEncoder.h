// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#pragma once
#include "studica/TitanQuad.h"
#include <chrono>
namespace studica {
/** Titan encoder counts, distance and velocity. Distance units follow
 * distPerTick. Legacy double-valued periods are retained instead of inheriting
 * the 2026 CounterBase, whose period methods use units::second_t.
 */
class TitanQuadEncoder {
 public:
  TitanQuadEncoder(TitanQuad& titan, int encoder, double distPerTick);
  int Get() const;
  void Reset();
  double GetPeriod() const;
  void SetMaxPeriod(double maxPeriod);
  bool GetStopped() const;
  double GetEncoderDistance();
  double GetRaw();
  void InvertSpeed();
  double GetSpeed();
  bool GetDirection() const;
  void SetReverseDirection();

 private:
  std::shared_ptr<detail::TitanDevice> m_device;
  int m_encoder;
  double m_distancePerTick;
  double m_maxPeriod = 0.5;
  mutable bool m_sampled = false;
  mutable int32_t m_lastCount = 0;
  mutable std::chrono::steady_clock::time_point m_lastEdge{};
  mutable double m_period = 0;
  bool m_reverse = false;
  bool m_invertSpeed = false;
};
}  // namespace studica
