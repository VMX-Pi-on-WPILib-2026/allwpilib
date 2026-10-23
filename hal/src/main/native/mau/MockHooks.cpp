// Copyright (c) 2026 FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/simulation/MockHooks.h"

#include <atomic>

namespace {
std::atomic<HAL_Bool> programStarted{false};
}

extern "C" {

// These hooks are part of wpilibc's public shared library on hardware too.
// The real VMX clock and Driver Station own timing and program start state.
void HALSIM_SetRuntimeType(HAL_RuntimeType) {}
void HALSIM_WaitForProgramStart(void) {}
void HALSIM_SetProgramStarted(void) { programStarted.store(true); }
HAL_Bool HALSIM_GetProgramStarted(void) { return programStarted.load(); }
void HALSIM_RestartTiming(void) {}
void HALSIM_PauseTiming(void) {}
void HALSIM_ResumeTiming(void) {}
HAL_Bool HALSIM_IsTimingPaused(void) { return false; }
void HALSIM_StepTiming(uint64_t) {}
void HALSIM_StepTimingAsync(uint64_t) {}
void HALSIM_SetSendError(HALSIM_SendErrorHandler) {}
void HALSIM_SetSendConsoleLine(HALSIM_SendConsoleLineHandler) {}

}  // extern "C"
