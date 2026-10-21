// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/LEDs.h"

// The VMX-pi has no radio status LED; the state is only stored.
static HAL_RadioLEDState radioLEDState = HAL_RadioLED_kOff;

extern "C" {

void HAL_SetRadioLEDState(HAL_RadioLEDState state, int32_t* status) {
  radioLEDState = state;
}

HAL_RadioLEDState HAL_GetRadioLEDState(int32_t* status) {
  return radioLEDState;
}

}  // extern "C"
