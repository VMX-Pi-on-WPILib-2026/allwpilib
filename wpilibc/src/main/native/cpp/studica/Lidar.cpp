// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#include "studica/Lidar.h"
#include "LidarProtocol.h"
#include <frc/SerialPort.h>
#include <chrono>
#include <stdexcept>
namespace studica {
struct Lidar::Impl {
  frc::SerialPort serial;
  detail::LidarProtocol parser;
  static frc::SerialPort::Port Port(int port) {
    if (port != 2 && port != 3)
      throw std::out_of_range("Lidar port must be kUSB1 or kUSB2");
    return static_cast<frc::SerialPort::Port>(port);
  }
  Impl(int port, int baud) : serial(baud, Port(port)) {
    serial.SetTimeout(0.05_s);
    serial.DisableTermination();
    serial.SetReadBufferSize(1024);
    serial.SetDTR(false);
    const char command[] = {char(0xa5), char(0x60)};
    if (serial.Write(command, 2) != 2)
      throw std::runtime_error("Lidar scan command failed");
  }
  ~Impl() {
    try {
      const char command[] = {char(0xa5), char(0x65)};
      serial.Write(command, 2);
      serial.SetDTR(true);
    } catch (...) {
    }
  }
};
Lidar::Lidar(int port) : Lidar(port, 128000) {}
Lidar::Lidar(int port, int baud) : m_impl(std::make_unique<Impl>(port, baud)) {}
Lidar::~Lidar() = default;
void Lidar::GetData() {
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  detail::LidarProtocol::Scan angles, distances;
  do {
    if (m_impl->parser.Next(angles, distances)) {
      std::copy(angles.begin(), angles.end(), angle);
      std::copy(distances.begin(), distances.end(), distance);
      return;
    }
    char bytes[512];
    int size = m_impl->serial.Read(bytes, sizeof(bytes));
    if (size > 0)
      m_impl->parser.Feed(
          {reinterpret_cast<const uint8_t*>(bytes), size_t(size)});
  } while (std::chrono::steady_clock::now() < deadline);
  throw std::runtime_error(
      "Lidar scan timed out (check port, baud rate, model and motor power)");
}
}  // namespace studica
