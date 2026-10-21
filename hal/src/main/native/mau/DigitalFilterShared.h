/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

#include "DigitalInternal.h"

/* Not sure yet this is needed... */

constexpr uint8_t kNumDigitalFilters = 4;
constexpr uint8_t kMaxDigitalFilterIndex = kNumDigitalFilters - 1;

void HAL_Internal_SetDigitalFilterPeriodNanoseconds(uint8_t digFilterIndex, uint64_t period_nanoseconds);
uint64_t HAL_Internal_GetDigitalFilterPeriodNanoseconds(uint8_t digFilterIndex);
