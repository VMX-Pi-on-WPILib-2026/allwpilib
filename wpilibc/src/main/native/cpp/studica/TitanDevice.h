// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#pragma once
#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <map>
#include <thread>
#include <condition_variable>
#include <cstdint>
namespace studica::detail {
using Frame = std::array<uint8_t, 8>;
class TitanDevice {
 public:
  static std::shared_ptr<TitanDevice> Acquire(int id);
  explicit TitanDevice(int id);
  ~TitanDevice();
  void Register(int motor, int frequency);
  void Release(int motor) noexcept;
  void Set(int motor, double speed);
  double Get(int motor);
  void Stop(int motor);
  Frame Read(int api, int minSize = 8);
  Frame Request(int request, int response, Frame payload = {}, int minSize = 8);
  void Write(int api, Frame payload = {});
  int32_t Count(int encoder);
  double RPM(int motor);
  const int id;

 private:
  using Clock = std::chrono::steady_clock;
  struct Motor {
    bool registered = false;
    double speed = 0;
    Clock::time_point updated{};
  };
  bool Safe() const;
  void Send(int api, const Frame& frame);
  void Tick();
  std::mutex m_mutex;
  std::condition_variable m_wake;
  std::array<Motor, 4> m_motors{};
  std::map<int, std::pair<uint32_t, Clock::time_point>> m_received;
  bool m_quit = false;
  bool m_fault = false;
  std::thread m_thread;
};
int32_t DecodeInt32(const Frame& data);
}  // namespace studica::detail
