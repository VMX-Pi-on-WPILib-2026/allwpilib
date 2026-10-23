// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#include "studica/TitanQuadEncoder.h"
#include "TitanDevice.h"
#include <cmath>
#include <bit>
#include <algorithm>
#include <limits>
#include <stdexcept>
using namespace studica;
TitanQuadEncoder::TitanQuadEncoder(TitanQuad& titan, int encoder,
                                   double distPerTick)
    : m_device(titan.m_device),
      m_encoder(encoder),
      m_distancePerTick(distPerTick) {
  if (encoder < 0 || encoder > 3)
    throw std::out_of_range("Titan encoder must be 0..3");
  if (!std::isfinite(distPerTick))
    throw std::invalid_argument("Distance per tick must be finite");
}
int TitanQuadEncoder::Get() const {
  int64_t count = m_device->Count(m_encoder);
  if (m_reverse)
    count = -count;
  if (count > INT32_MAX || count < INT32_MIN)
    throw std::overflow_error("Inverted encoder count exceeds int32");
  return int(count);
}
void TitanQuadEncoder::Reset() {
  m_device->Write(13, {uint8_t(m_encoder)});
  m_sampled = false;
}
double TitanQuadEncoder::GetPeriod() const {
  int32_t count = m_device->Count(m_encoder);
  auto now = std::chrono::steady_clock::now();
  if (!m_sampled) {
    m_sampled = true;
    m_lastCount = count;
    m_lastEdge = now;
    m_period = std::numeric_limits<double>::infinity();
    return m_period;
  }
  double elapsed = std::chrono::duration<double>(now - m_lastEdge).count();
  int64_t delta =
      std::bit_cast<int32_t>(uint32_t(count) - uint32_t(m_lastCount));
  if (delta != 0) {
    m_period = elapsed / std::abs(double(delta));
    m_lastEdge = now;
    m_lastCount = count;
    return m_period;
  }
  return std::max(m_period, elapsed);
}
void TitanQuadEncoder::SetMaxPeriod(double value) {
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument("Max period must be finite and positive");
  m_maxPeriod = value;
}
bool TitanQuadEncoder::GetStopped() const {
  return GetPeriod() > m_maxPeriod;
}
double TitanQuadEncoder::GetEncoderDistance() {
  return double(Get()) * m_distancePerTick;
}
double TitanQuadEncoder::GetRaw() {
  return m_device->Count(m_encoder);
}
void TitanQuadEncoder::InvertSpeed() {
  m_invertSpeed = true;
}
double TitanQuadEncoder::GetSpeed() {
  // Legacy GetSpeed reports RPM (same Titan telemetry as GetRPM), not m/s.
  return m_device->RPM(m_encoder) * (m_invertSpeed ? -1 : 1);
}
bool TitanQuadEncoder::GetDirection() const {
  return m_device->RPM(m_encoder) * (m_reverse ? -1 : 1) >= 0;
}
void TitanQuadEncoder::SetReverseDirection() {
  m_reverse = true;
}
