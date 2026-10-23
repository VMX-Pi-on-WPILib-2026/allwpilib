// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md and vmx/studica/NOTICE.md.
#include "studica/Cobra.h"
#include <hal/I2C.h>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <thread>
namespace studica {
struct Cobra::Impl {
  Impl() {
    int32_t status = 0;
    HAL_InitializeI2C(HAL_I2C_kMXP, &status);
    if (status != 0)
      throw std::runtime_error("Cobra I2C initialization failed");
  }
  ~Impl() { HAL_CloseI2C(HAL_I2C_kMXP); }
  bool Transaction(const uint8_t* tx, int txSize, uint8_t* rx, int rxSize) {
    return HAL_TransactionI2C(HAL_I2C_kMXP, 0x48, tx, txSize, rx, rxSize) < 0;
  }
};
namespace {
std::mutex cobraMutex;
}
Cobra::Cobra() : Cobra(5.0f) {}
Cobra::Cobra(float vRef) : m_vRef(vRef) {
  if (!std::isfinite(vRef) || vRef <= 0)
    throw std::invalid_argument(
        "Cobra reference voltage must be finite and positive");
  m_impl = std::make_unique<Impl>();
}
Cobra::~Cobra() = default;
int Cobra::GetRawValue(int channel) {
  if (channel < 0 || channel > 3)
    throw std::out_of_range("Cobra channel must be 0..3");
  std::scoped_lock lock(cobraMutex);
  const uint16_t config = 0x8583 | ((4 + channel) << 12);
  uint8_t tx[] = {1, uint8_t(config >> 8), uint8_t(config)};
  if (m_impl->Transaction(tx, 3, nullptr, 0))
    throw std::runtime_error("Cobra configuration write failed");
  // 1600 SPS; poll the conversion-ready bit with a bounded deadline.
  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  uint8_t rx[2]{};
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    uint8_t reg = 1;
    if (m_impl->Transaction(&reg, 1, rx, 2))
      throw std::runtime_error("Cobra status read failed");
    if (rx[0] & 0x80) {
      reg = 0;
      if (m_impl->Transaction(&reg, 1, rx, 2))
        throw std::runtime_error("Cobra conversion read failed");
      return (int(rx[0]) << 8) |
             rx[1];  // Original API returns the left-aligned raw register.
    }
  } while (std::chrono::steady_clock::now() < deadline);
  throw std::runtime_error("Cobra conversion timed out");
}
float Cobra::GetVoltage(int channel) {
  return GetRawValue(channel) * (m_vRef / 32768.0f);
}
}  // namespace studica
