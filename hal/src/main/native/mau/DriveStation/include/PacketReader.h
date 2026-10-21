// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace mau::comms {
// A cursor bounded by a single protocol frame/tag, including nested fields.
class PacketReader {
 public:
  explicit PacketReader(std::span<const uint8_t> bytes) : m_bytes(bytes) {}
  size_t remaining() const { return m_bytes.size(); }
  bool read(uint8_t& value) {
    if (m_bytes.empty()) return false;
    value = m_bytes.front();
    m_bytes = m_bytes.subspan(1);
    return true;
  }
  bool read16(uint16_t& value) {
    std::span<const uint8_t> bytes;
    if (!take(2, bytes)) return false;
    value = (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
    return true;
  }
  bool take(size_t size, std::span<const uint8_t>& bytes) {
    if (size > remaining()) return false;
    bytes = m_bytes.first(size);
    m_bytes = m_bytes.subspan(size);
    return true;
  }
 private:
  std::span<const uint8_t> m_bytes;
};
}  // namespace mau::comms
