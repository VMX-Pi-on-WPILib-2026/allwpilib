// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>
namespace studica::detail {
/** Original non-intensity YDLIDAR triangle packets (Studica 360-degree LiDAR).
 * Header, check bits, bounded length and XOR checksum are validated before use.
 */
class LidarProtocol {
 public:
  using Scan = std::array<float, 360>;
  void Feed(std::span<const uint8_t> bytes) {
    if (m_bytes.size() + bytes.size() > 4096) {
      m_bytes.clear();
      m_started = false;
    }
    m_bytes.insert(m_bytes.end(), bytes.begin(), bytes.end());
  }
  bool Next(Scan& angles, Scan& distances) {
    while (m_bytes.size() >= 10) {
      if (m_bytes[0] != 0xaa || m_bytes[1] != 0x55) {
        m_bytes.erase(m_bytes.begin());
        continue;
      }
      int count = m_bytes[3];
      int size = 10 + 2 * count;
      if (count == 0 || !(m_bytes[4] & 1) || !(m_bytes[6] & 1)) {
        m_bytes.erase(m_bytes.begin());
        continue;
      }
      if (int(m_bytes.size()) < size)
        return false;
      uint16_t sum = Word(0) ^ Word(2) ^ Word(4) ^ Word(6);
      for (int i = 0; i < count; i++)
        sum ^= Word(10 + 2 * i);
      if (sum != Word(8)) {
        m_bytes.erase(m_bytes.begin());
        continue;
      }
      double first = (Word(4) >> 1) / 64.0;
      double last = (Word(6) >> 1) / 64.0;
      if (first >= 360 || last >= 360) {
        m_bytes.erase(m_bytes.begin(), m_bytes.begin() + size);
        continue;
      }
      bool complete = false;
      if (m_bytes[2] & 1) {
        if (m_started) {
          angles = m_angles;
          distances = m_distances;
          complete = true;
        }
        m_started = true;
        for (int i = 0; i < 360; i++) {
          m_angles[i] = float(i);
          m_distances[i] = std::numeric_limits<float>::quiet_NaN();
        }
      }
      if (m_started) {
        double diff = last - first;
        if (diff < 0)
          diff += 360;
        for (int i = 0; i < count; i++) {
          double distance = Word(10 + 2 * i) / 4.0;
          if (distance <= 0)
            continue;
          double angle = first + (count > 1 ? diff * i / (count - 1) : 0);
          angle += std::atan(21.8 * (155.3 - distance) / (155.3 * distance)) *
                   180.0 / 3.141592653589793;
          angle = std::fmod(angle + 360.0, 360.0);
          int bin = int(angle);
          m_angles[bin] = float(angle);
          m_distances[bin] = float(distance);
        }
      }
      m_bytes.erase(m_bytes.begin(), m_bytes.begin() + size);
      if (complete)
        return true;
    }
    return false;
  }

 private:
  uint16_t Word(int i) const {
    return uint16_t(m_bytes[i]) | (uint16_t(m_bytes[i + 1]) << 8);
  }
  std::vector<uint8_t> m_bytes;
  Scan m_angles{};
  Scan m_distances{};
  bool m_started = false;
};
}  // namespace studica::detail
