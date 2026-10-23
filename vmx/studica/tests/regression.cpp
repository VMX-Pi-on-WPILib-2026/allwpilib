// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md.
#include <studica/Cobra.h>
#include <hal/I2C.h>
#include <hal/Errors.h>
#include <hal/SerialPort.h>
#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>
#include <frc/RobotController.h>
#include <hal/CAN.h>
#include <hal/DriverStation.h>
#include "TitanDevice.h"
#include "LidarProtocol.h"
#include "LinuxSerialBaud.h"
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>
#include <pty.h>
#include <unistd.h>
using namespace std::chrono_literals;
using Frame = studica::detail::Frame;
namespace {
struct Message {
  int id;
  int api;
  Frame data;
};
std::mutex mutex;
std::vector<Message> sent;
std::map<int, std::vector<uint8_t>> replies;
std::atomic<bool> enabled = false, attached = true, estop = false,
                  badSend = false;
std::atomic<int> receiveStatus = 0;
bool i2cFail = false;
int i2cChannel = 0;
uint32_t canTimestamp = 1;
void Clear() {
  std::scoped_lock lock(mutex);
  sent.clear();
}
bool Has(int api, int motor = -1) {
  std::scoped_lock lock(mutex);
  for (const auto& msg : sent)
    if (msg.api == api && (motor < 0 || msg.data[0] == motor))
      return true;
  return false;
}
Frame LastSpeed(int motor) {
  std::scoped_lock lock(mutex);
  for (auto i = sent.rbegin(); i != sent.rend(); ++i)
    if (i->api == 2 && i->data[0] == motor)
      return i->data;
  assert(false);
  return {};
}
template <class F>
void Throws(F action) {
  bool thrown = false;
  try {
    action();
  } catch (const std::exception&) {
    thrown = true;
  }
  assert(thrown);
}
std::vector<uint8_t> Packet(bool ring, double angle, uint16_t distance) {
  uint16_t a = uint16_t(angle * 64) * 2 + 1;
  std::vector<uint8_t> p = {0xaa,
                            0x55,
                            uint8_t(ring),
                            1,
                            uint8_t(a),
                            uint8_t(a >> 8),
                            uint8_t(a),
                            uint8_t(a >> 8),
                            0,
                            0,
                            uint8_t(distance * 4),
                            uint8_t((distance * 4) >> 8)};
  uint16_t sum = 0;
  for (int i : {0, 2, 4, 6, 10})
    sum ^= uint16_t(p[i]) | (uint16_t(p[i + 1]) << 8);
  p[8] = uint8_t(sum);
  p[9] = uint8_t(sum >> 8);
  return p;
}
}  // namespace
extern "C" {
void HAL_InitializeI2C(HAL_I2CPort port, int32_t* status) {
  assert(port == HAL_I2C_kMXP);
  *status = 0;
}
int32_t HAL_TransactionI2C(HAL_I2CPort port, int32_t address, const uint8_t* tx,
                           int32_t txSize, uint8_t* rx, int32_t rxSize) {
  assert(port == HAL_I2C_kMXP && address == 0x48);
  if (i2cFail)
    return -1;
  if (txSize == 3) {
    assert(tx[0] == 1 && tx[2] == 0x83);
    i2cChannel = ((tx[1] >> 4) & 7) - 4;
  } else if (txSize == 1 && rxSize == 2) {
    if (tx[0] == 1) {
      rx[0] = 0x80;
      rx[1] = 0;
    } else {
      assert(tx[0] == 0);
      rx[0] = 0x40;
      rx[1] = 0;
    }
  } else
    assert(false);
  return 0;
}
void HAL_CloseI2C(HAL_I2CPort) {}
int HAL_GetSerialFD(HAL_SerialPortHandle, int32_t* status) {
  *status = HAL_HANDLE_ERROR;
  return -1;
}

int32_t HAL_GetControlWord(HAL_ControlWord* word) {
  *word = {};
  word->enabled = enabled;
  word->dsAttached = attached;
  word->eStop = estop;
  return 0;
}
void HAL_CAN_SendMessage(uint32_t id, const uint8_t* data, uint8_t size,
                         int32_t period, int32_t* status) {
  assert(size == 8 && period == 0);
  std::scoped_lock lock(mutex);
  Frame frame{};
  std::copy(data, data + 8, frame.begin());
  sent.push_back({int(id & 63), int((id >> 6) & 1023), frame});
  *status = badSend ? -20 : 0;
}
void HAL_CAN_ReceiveMessage(uint32_t* id, uint32_t mask, uint8_t* data,
                            uint8_t* size, uint32_t* timestamp,
                            int32_t* status) {
  assert(mask == 0x1fffffff);
  std::scoped_lock lock(mutex);
  int api = (*id >> 6) & 1023;
  auto i = replies.find(api);
  if (i == replies.end()) {
    *status = -1;
    *size = 0;
    return;
  }
  *size = i->second.size();
  std::copy(i->second.begin(), i->second.end(), data);
  *timestamp = canTimestamp;
  *status = receiveStatus;
}
}
units::volt_t frc::RobotController::GetBatteryVoltage() {
  return units::volt_t{12};
}
int main() {
  static_assert(std::is_base_of_v<frc::MotorController, studica::TitanQuad>);
  static_assert(std::is_base_of_v<frc::SpeedController, studica::TitanQuad>);
  Throws([] { studica::TitanQuad invalid(64, 0); });
  {
    studica::TitanQuad a(42, 0), b(42, 1);
    Throws([] { studica::TitanQuad duplicate(42, 0); });
    assert(a.GetID() == 42);
    a.Set(0.7);
    assert(a.Get() == 0);
    assert(!Has(1));
    enabled = true;
    Clear();
    a.Set(2);
    auto f = LastSpeed(0);
    assert(f[1] == 100 && f[2] == 1 && f[3] == 0);
    a.SetInverted(true);
    a.PIDWrite(0.25);
    f = LastSpeed(0);
    assert(f[1] == 25 && f[2] == 0 && f[3] == 1);
    b.Set(0.5);
    a.Disable();
    assert(Has(3, 0));
    assert(b.Get() == 0.5);
    {
      studica::TitanQuad other(42, 2);
      other.Set(0.25);
    }
    b.Set(0.5);
    assert(LastSpeed(1)[1] == 50);
    Clear();
    enabled = false;
    std::this_thread::sleep_for(45ms);
    assert(Has(0));
    enabled = true;
    Clear();
    std::this_thread::sleep_for(25ms);
    assert(!Has(1));  // no stale restart
    b.Set(0.5);
    Clear();
    std::this_thread::sleep_for(145ms);
    assert(Has(0) && b.Get() == 0);
    b.Set(0.5);
    Clear();
    estop = true;
    std::this_thread::sleep_for(45ms);
    assert(Has(0));
    estop = false;
    replies[37] = {0xfe, 0xff, 0xff, 0xff};
    replies[41] = {0xe4, 0xf9, 0xff, 0xff};  // -2 ticks, -15.64 RPM
    studica::TitanQuadEncoder encoder(a, 0, 0.5);
    assert(encoder.Get() == -2 && encoder.GetEncoderDistance() == -1);
    studica::TitanQuadEncoder scaled(a, 0, -0.5);
    assert(scaled.GetEncoderDistance() == 1);
    assert(std::abs(encoder.GetSpeed() + 15.64) < 1e-9);
    encoder.InvertSpeed();
    assert(std::abs(encoder.GetSpeed() - 15.64) < 1e-9);
    assert(!encoder.GetDirection());
    encoder.SetReverseDirection();
    assert(encoder.Get() == 2 && encoder.GetDirection());
    encoder.Reset();
    assert(Has(13, 0));
    assert(std::isinf(encoder.GetPeriod()));
    std::this_thread::sleep_for(2ms);
    {
      std::scoped_lock lock(mutex);
      replies[37] = {4, 0, 0, 0};
    }
    assert(encoder.GetPeriod() < 0.01);
    encoder.SetMaxPeriod(0.01);
    std::this_thread::sleep_for(15ms);
    assert(encoder.GetStopped());
    Throws([&] { encoder.SetMaxPeriod(0); });
    receiveStatus = 1;
    assert(encoder.GetRaw() == 4);
    receiveStatus = 0;
    {
      std::scoped_lock lock(mutex);
      replies[37] = {0xff, 0xff, 0xff, 0x7f};
      ++canTimestamp;
    }
    encoder.Reset();
    assert(std::isinf(encoder.GetPeriod()));
    std::this_thread::sleep_for(2ms);
    {
      std::scoped_lock lock(mutex);
      replies[37] = {0, 0, 0, 0x80};
      ++canTimestamp;
    }
    assert(encoder.GetPeriod() < 0.01);
    {
      std::scoped_lock lock(mutex);
      replies[37] = {1, 2};
    }
    Throws([&] { encoder.Get(); });
    replies[12] = {0xf0, 0x3c};
    assert(a.GetFrequency() == 15600);
    replies[5] = {42, 1, 2, 3, 1, 4};
    assert(a.GetFirmwareVersion() == "Firmware Version: [1.2.3]");
    assert(a.GetHardwareVersion() == "Hardware: Titan Quad, Version: 4");
    Throws([&] { b.Set(NAN); });
    assert(b.Get() == 0);
    badSend = true;
    Throws([&] { b.Set(0.5); });
    badSend = false;
    Throws([&] { b.Set(0.5); });  // CAN fault remains latched
  }
  assert(Has(0));
  {
    studica::detail::LidarProtocol parser;
    auto start = Packet(true, 0, 1000), data = Packet(false, 90, 2000),
         end = Packet(true, 0, 3000);
    parser.Feed({start.data(), 3});
    studica::detail::LidarProtocol::Scan angles, distances;
    assert(!parser.Next(angles, distances));
    parser.Feed({start.data() + 3, start.size() - 3});
    parser.Feed(data);
    auto corrupt = Packet(false, 180, 4000);
    corrupt[8] ^= 1;
    parser.Feed(corrupt);
    parser.Feed(end);
    assert(parser.Next(angles, distances));
    bool first = false, second = false;
    for (float distance : distances) {
      first |= distance == 1000;
      second |= distance == 2000;
      assert(distance != 4000);
    }
    assert(first && second);
    for (int i = 0; i < 100; i++) {
      std::vector<uint8_t> junk(1024, 0xaa);
      parser.Feed(junk);
      parser.Next(angles, distances);
    }
  }
  {
    studica::Cobra adc;
    for (int i = 0; i < 4; i++) {
      assert(adc.GetRawValue(i) == 0x4000 && i2cChannel == i);
      assert(std::abs(adc.GetVoltage(i) - 2.5f) < 1e-6);
    }
    Throws([&] { adc.GetRawValue(4); });
    i2cFail = true;
    Throws([&] { adc.GetVoltage(0); });
    i2cFail = false;
  }
  int32_t dtrStatus = 0;
  HAL_SetSerialDTR(123, 1, &dtrStatus);
  assert(dtrStatus == HAL_HANDLE_ERROR);
  int master, slave;
  assert(openpty(&master, &slave, nullptr, nullptr, nullptr) == 0);
  assert(hal::vmx::SetCustomSerialBaud(slave, 128000));
  assert(!hal::vmx::SetCustomSerialBaud(slave, 0));
  close(master);
  close(slave);
  puts(
      "Studica C++ protocol, motor safety, sharing, encoders, LiDAR and custom "
      "baud tests passed");
}
