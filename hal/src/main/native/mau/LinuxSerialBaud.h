// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md.
#pragma once
#include <cstdint>
#include <sys/ioctl.h>
namespace hal::vmx {
// Linux generic termios2 ABI (aarch64 and x86-64). Keep it separate from the
// libc termios structure to avoid conflicting asm/termbits.h definitions.
struct SerialTermios2 {
  uint32_t input, output, control, local;
  uint8_t line;
  uint8_t characters[19];
  uint32_t inputSpeed, outputSpeed;
};
inline bool SetCustomSerialBaud(int fd, int baud) {
  if (baud <= 0)
    return false;
  constexpr unsigned long get = _IOR('T', 0x2a, SerialTermios2);
  constexpr unsigned long set = _IOW('T', 0x2b, SerialTermios2);
  SerialTermios2 options{};
  if (ioctl(fd, get, &options) != 0)
    return false;
  options.control = (options.control & ~uint32_t{0x100f}) | 0x1000;
  options.inputSpeed = options.outputSpeed = uint32_t(baud);
  if (ioctl(fd, set, &options) != 0 || ioctl(fd, get, &options) != 0)
    return false;
  return options.inputSpeed == uint32_t(baud) &&
         options.outputSpeed == uint32_t(baud);
}
}  // namespace hal::vmx
