// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#include "studica/TitanQuad.h"
#include "TitanDevice.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
using namespace studica;
TitanQuad::TitanQuad(int id, int motor) : TitanQuad(id, 15600, motor) {}
TitanQuad::TitanQuad(int id, int frequency, int motor)
    : m_device(detail::TitanDevice::Acquire(id)), m_motor(motor) {
  m_device->Register(motor, frequency);
}
TitanQuad::~TitanQuad() {
  m_device->Release(m_motor);
}
int TitanQuad::GetID() {
  return m_device->id;
}
int TitanQuad::GetFrequency() {
  auto data = m_device->Request(11, 12, {uint8_t(m_motor)}, 2);
  return data[0] | (int(data[1]) << 8);
}
std::string TitanQuad::GetFirmwareVersion() {
  auto d = m_device->Request(4, 5, {}, 6);
  return "Firmware Version: [" + std::to_string(d[1]) + "." +
         std::to_string(d[2]) + "." + std::to_string(d[3]) + "]";
}
std::string TitanQuad::GetHardwareVersion() {
  auto d = m_device->Request(4, 5, {}, 6);
  std::string type = d[4] == 1   ? "Titan Quad"
                     : d[4] == 2 ? "Titan Small"
                                 : "Type " + std::to_string(d[4]);
  return "Hardware: " + type + ", Version: " + std::to_string(d[5]);
}
double TitanQuad::GetControllerTemp() {
  auto d = m_device->Read(47, 2);
  return d[0] + d[1] / 100.0;
}
void TitanQuad::Set(double speed) {
  m_device->Set(m_motor, m_inverted ? -speed : speed);
}
double TitanQuad::Get() const {
  return m_device->Get(m_motor);
}
void TitanQuad::SetInverted(bool inverted) {
  m_inverted = inverted;
}
bool TitanQuad::GetInverted() const {
  return m_inverted;
}
void TitanQuad::Disable() {
  m_device->Stop(m_motor);
}
void TitanQuad::StopMotor() {
  Disable();
}
void TitanQuad::PIDWrite(double output) {
  Set(output);
}
double TitanQuad::GetLimitSwitch(bool direction) {
  return m_device->Read(45)[m_motor * 2 + int(direction)] != 0;
}
void TitanQuad::InvertRPM() {
  m_invertRPM = true;
}
int TitanQuad::GetRPM() {
  return int(m_device->RPM(m_motor) * (m_invertRPM ? -1 : 1));
}
std::string TitanQuad::GetSerialNumber() {
  std::ostringstream out;
  out << std::uppercase << std::hex << std::setfill('0');
  for (int api = 7; api <= 9; ++api) {
    auto d = m_device->Request(6, api, {}, 4);
    if (api != 7)
      out << '-';
    out << std::setw(8) << uint32_t(detail::DecodeInt32(d));
  }
  return out.str();
}
