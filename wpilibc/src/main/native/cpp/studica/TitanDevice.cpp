// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#include "TitanDevice.h"
#include <hal/CAN.h>
#include <hal/DriverStation.h>
#include <algorithm>
#include <bit>
#include <cmath>
#include <map>
#include <stdexcept>
using namespace std::chrono_literals;
namespace studica::detail {
namespace {
constexpr uint32_t kBase = 0x020c0000;
void CheckMotor(int motor) {
  if (motor < 0 || motor > 3)
    throw std::out_of_range("Titan motor/encoder must be 0..3");
}
}  // namespace
int32_t DecodeInt32(const Frame& d) {
  return std::bit_cast<int32_t>(uint32_t{d[0]} | (uint32_t{d[1]} << 8) |
                                (uint32_t{d[2]} << 16) |
                                (uint32_t{d[3]} << 24));
}
std::shared_ptr<TitanDevice> TitanDevice::Acquire(int id) {
  if (id < 1 || id > 63)
    throw std::out_of_range("Titan CAN ID must be 1..63");
  static std::mutex mutex;
  static std::map<int, std::weak_ptr<TitanDevice>> devices;
  std::scoped_lock lock(mutex);
  if (auto device = devices[id].lock())
    return device;
  auto device = std::make_shared<TitanDevice>(id);
  devices[id] = device;
  return device;
}
TitanDevice::TitanDevice(int deviceID) : id(deviceID) {
  Send(0, {});  // Construction never enables a motor.
  m_thread = std::thread([this] {
    std::unique_lock lock(m_mutex);
    while (!m_quit) {
      try {
        Tick();
      } catch (...) {
        m_fault = true;
        for (auto& motor : m_motors) {
          motor.speed = 0;
          motor.updated = {};
        }
        try {
          Send(0, {});
        } catch (...) {
        }
      }
      m_wake.wait_for(lock, 20ms, [this] { return m_quit; });
    }
  });
}
TitanDevice::~TitanDevice() {
  {
    std::scoped_lock lock(m_mutex);
    m_quit = true;
  }
  m_wake.notify_all();
  if (m_thread.joinable())
    m_thread.join();
  try {
    Send(0, {});
  } catch (...) {
  }
}
bool TitanDevice::Safe() const {
  HAL_ControlWord word{};
  return HAL_GetControlWord(&word) == 0 && word.dsAttached && word.enabled &&
         !word.eStop;
}
void TitanDevice::Send(int api, const Frame& data) {
  int32_t status = 0;
  HAL_CAN_SendMessage(kBase + (uint32_t(api) << 6) + id, data.data(),
                      data.size(), 0, &status);
  if (status != 0) {
    m_fault = true;
    for (auto& motor : m_motors) {
      motor.speed = 0;
      motor.updated = {};
    }
    throw std::runtime_error("Titan CAN send failed: " +
                             std::to_string(status));
  }
}
void TitanDevice::Write(int api, Frame payload) {
  std::scoped_lock lock(m_mutex);
  Send(api, payload);
}
void TitanDevice::Register(int motor, int frequency) {
  CheckMotor(motor);
  if (frequency < 0 || frequency > 20000)
    throw std::out_of_range("Titan frequency must be 0..20000 Hz");
  std::scoped_lock lock(m_mutex);
  if (m_motors[motor].registered)
    throw std::runtime_error("Titan motor is already allocated");
  Send(10, {uint8_t(motor), uint8_t(frequency), uint8_t(frequency >> 8)});
  m_motors[motor].registered = true;
}
void TitanDevice::Release(int motor) noexcept {
  std::scoped_lock lock(m_mutex);
  m_motors[motor] = {};
  try {
    Send(3, {uint8_t(motor)});
  } catch (...) {
    m_fault = true;
  }
}
void TitanDevice::Set(int motor, double speed) {
  if (!std::isfinite(speed)) {
    Stop(motor);
    throw std::invalid_argument("Titan speed must be finite");
  }
  std::scoped_lock lock(m_mutex);
  if (m_fault)
    throw std::runtime_error(
        "Titan CAN fault; recreate the controller before driving again");
  auto& channel = m_motors[motor];
  bool safe = Safe();
  channel.speed = safe ? std::clamp(speed, -1.0, 1.0) : 0;
  channel.updated = safe ? Clock::now() : Clock::time_point{};
  Tick();
}
double TitanDevice::Get(int motor) {
  std::scoped_lock lock(m_mutex);
  auto& channel = m_motors[motor];
  return Safe() && Clock::now() - channel.updated <= 100ms ? channel.speed : 0;
}
void TitanDevice::Stop(int motor) {
  std::scoped_lock lock(m_mutex);
  m_motors[motor].speed = 0;
  m_motors[motor].updated = {};
  Send(3, {uint8_t(motor)});
}
void TitanDevice::Tick() {
  bool safe = Safe() && !m_fault;
  bool active = false;
  for (auto& motor : m_motors) {
    if (!safe || Clock::now() - motor.updated > 100ms) {
      motor.speed = 0;
      motor.updated = {};
    }
    active |= motor.registered && motor.updated != Clock::time_point{};
  }
  if (!active) {
    Send(0, {});
    return;
  }
  Send(1, {});
  for (int i = 0; i < 4; ++i) {
    const auto& motor = m_motors[i];
    if (!motor.registered)
      continue;
    if (motor.updated == Clock::time_point{}) {
      Send(3, {uint8_t(i)});
      continue;
    }
    double value = motor.speed;
    Send(2, {uint8_t(i), uint8_t(std::abs(value) * 100), uint8_t(value >= 0),
             uint8_t(value <= 0)});
  }
}
Frame TitanDevice::Read(int api, int minSize) {
  std::scoped_lock lock(m_mutex);
  uint32_t address = kBase + (uint32_t(api) << 6) + id;
  Frame data{};
  uint8_t size = 0;
  uint32_t timestamp = 0;
  int32_t status = 0;
  HAL_CAN_ReceiveMessage(&address, 0x1fffffff, data.data(), &size, &timestamp,
                         &status);
  // The VMX blackboard returns status 1 for a previously retrieved frame.
  if ((status != 0 && status != 1) || size < minSize)
    throw std::runtime_error(
        "Titan telemetry unavailable or truncated for API " +
        std::to_string(api));
  auto now = Clock::now();
  auto seen = m_received.find(api);
  if (seen == m_received.end() || seen->second.first != timestamp)
    m_received[api] = {timestamp, now};
  else if (now - seen->second.second > 250ms)
    throw std::runtime_error("Titan telemetry is stale");
  return data;
}
Frame TitanDevice::Request(int request, int response, Frame payload,
                           int minSize) {
  Write(request, payload);
  // Bounded request/response wait; never hold the motor-output mutex while
  // waiting.
  for (int i = 0; i < 10; ++i) {
    std::this_thread::sleep_for(10ms);
    try {
      return Read(response, minSize);
    } catch (const std::runtime_error&) {
    }
  }
  throw std::runtime_error("Titan telemetry request timed out");
}
int32_t TitanDevice::Count(int encoder) {
  CheckMotor(encoder);
  return DecodeInt32(Read(37 + encoder, 4));
}
double TitanDevice::RPM(int motor) {
  CheckMotor(motor);
  return DecodeInt32(Read(41 + motor, 4)) / 100.0;
}
}  // namespace studica::detail
