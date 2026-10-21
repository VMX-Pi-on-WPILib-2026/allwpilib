// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include <string_view>

// Same as athena/HALInternal.h minus the FPGA interrupt helper; the error
// helpers are implemented in cpp/ErrorHandling.cpp, GetDSInitializeTime in HAL.cpp.
namespace hal {
void SetLastError(int32_t* status, std::string_view value);
void SetLastErrorIndexOutOfRange(int32_t* status, std::string_view message,
                                 int32_t minimum, int32_t maximum,
                                 int32_t channel);
void SetLastErrorPreviouslyAllocated(int32_t* status, std::string_view message,
                                     int32_t channel,
                                     std::string_view previousAllocation);
uint64_t GetDSInitializeTime();
}  // namespace hal
