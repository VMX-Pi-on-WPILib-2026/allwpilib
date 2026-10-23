// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#pragma once
#include <memory>
namespace studica {
/** Original Studica 360-degree LiDAR, non-intensity triangle protocol.
 * Distances are millimeters; missing rays are NaN. Scan reads time out after 1
 * s. The default 128000-baud model is distinct from newer T-Mini devices.
 */
class Lidar {
 public:
  enum Port { kUSB1 = 2, kUSB2 = 3 };
  float angle[360]{};
  float distance[360]{};
  explicit Lidar(int port);
  Lidar(int port, int baudRate);
  ~Lidar();
  Lidar(const Lidar&) = delete;
  Lidar& operator=(const Lidar&) = delete;
  void GetData();

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};
}  // namespace studica
