// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>
#include <span>
#include <vector>

namespace mau::comms {
// TCP has no message boundaries. Keep at most one incomplete u16-sized frame.
class TcpPacketBuffer {
 public:
  void reset() { m_pending.clear(); }

  template <typename Consumer>
  void append(std::span<const uint8_t> bytes, Consumer consume) {
    for (uint8_t byte : bytes) {
      m_pending.push_back(byte);
      if (m_pending.size() < 2) continue;
      size_t length = (static_cast<size_t>(m_pending[0]) << 8) | m_pending[1];
      if (length == 0) {
        reset();  // Zero-length padding/end marker has no tag.
      } else if (m_pending.size() == length + 2) {
        consume(std::span<const uint8_t>{m_pending});
        reset();
      }
    }
  }
 private:
  std::vector<uint8_t> m_pending;
};
}  // namespace mau::comms
