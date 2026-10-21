/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// VMX-pi Driver Station HAL.
//
// The DS protocol itself is still implemented by DriveStation/DriverComms.cpp
// (unchanged from the 2020 port), which stores packet data in Mau_DriveData.
// This file adapts that data to the 2023+ HAL model used by WPILib 2026: the
// robot loop calls HAL_RefreshDSData() to take a consistent snapshot, and
// registered event handles are signaled whenever a new DS packet arrives.
// The cache handling follows athena/FRCDriverStation.cpp.

#include "hal/DriverStation.h"
#include "hal/Power.h"
#include "hal/CAN.h"
#include "hal/Errors.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

#include <fmt/format.h>
#include <wpi/EventVector.h>
#include <wpi/mutex.h>

#include "HALInitializer.h"
#include "DriveStation/include/socket.hpp"
#include "DriveStation/include/DriverComms.hpp"
#include "DriveStation/include/MauDriveData.h"
#include "MauInternal.h"

static wpi::mutex msgMutex;

static uint64_t ds_packet_count = 0;
#define UPDATE_CAN_AND_POWER_EVERY 5

namespace {
struct JoystickDataCache {
  JoystickDataCache() { std::memset(this, 0, sizeof(*this)); }
  void Update() {
    Mau_DriveData::scribeUdpData(axes, povs, buttons, &controlWord,
                                 &allianceStation, &matchTime);
  }

  HAL_JoystickAxes axes[HAL_kMaxJoysticks];
  HAL_JoystickPOVs povs[HAL_kMaxJoysticks];
  HAL_JoystickButtons buttons[HAL_kMaxJoysticks];
  HAL_AllianceStationID allianceStation;
  float matchTime;
  HAL_ControlWord controlWord;
};
static_assert(std::is_standard_layout_v<JoystickDataCache>);

struct TcpCache {
  TcpCache() { std::memset(this, 0, sizeof(*this)); }
  void Update() { Mau_DriveData::scribeTcpData(&matchInfo, descriptors); }

  HAL_MatchInfo matchInfo;
  HAL_JoystickDescriptor descriptors[HAL_kMaxJoysticks];
};
static_assert(std::is_standard_layout_v<TcpCache>);
}  // namespace

#define CHECK_JOYSTICK_NUMBER(stickNum)                  \
  if ((stickNum) < 0 || (stickNum) >= HAL_kMaxJoysticks) \
  return PARAMETER_OUT_OF_RANGE

static wpi::EventVector* newDataEvents = nullptr;

// Triple buffering, same scheme as athena: the DS thread fills cacheToUpdate,
// HAL_RefreshDSData() swaps the newest one into currentRead.
static HAL_ControlWord newestControlWord;
static JoystickDataCache caches[3];
static JoystickDataCache* currentRead = &caches[0];
static JoystickDataCache* currentReadLocal = &caches[0];
static std::atomic<JoystickDataCache*> currentCache{nullptr};
static JoystickDataCache* lastGiven = &caches[1];
static JoystickDataCache* cacheToUpdate = &caches[2];
static wpi::mutex cacheMutex;

static TcpCache tcpCurrent;
static wpi::mutex tcpCacheMutex;

// Keeps the VMX-pi IO watchdog in step with the DS enable state. In the 2020
// port this ran from HAL_WaitForDSData(), which no longer exists.
static void UpdateIOWatchdog(const HAL_ControlWord& controlWord) {
  VMXErrorCode vmxerr;
  if (controlWord.dsAttached && controlWord.enabled) {
    // Feed Watchdog whenever new control words are received and enabled.
    mau::vmxIO->FeedWatchdog(&vmxerr);
    mau::vmxIOActive = true;
  } else if (mau::vmxIOActive) {
    // Immediately Expire the Watchdog when disabled state is entered.
    mau::vmxIO->ExpireWatchdogNow(&vmxerr);
    mau::vmxIOActive = false;
  }
}

// Reports battery voltage and CAN status back to the DS.
static void UpdateCANAndPowerStatus() {
  int32_t status = 0;
  mau::comms::setInputVoltage(HAL_GetVinVoltage(&status));

  float percentBusUtilization;
  uint32_t busOffCount;
  uint32_t txFullCount;
  uint32_t rxErrorCount;
  uint32_t txErrorCount;
  status = 0;
  HAL_CAN_GetCANStatus(&percentBusUtilization, &busOffCount, &txFullCount,
                       &rxErrorCount, &txErrorCount, &status);
  if (!status) {
    mau::comms::setCANStatus(percentBusUtilization, busOffCount, txFullCount,
                             rxErrorCount, txErrorCount);
  }
}

// Called by Mau_DriveData from the DS comms threads after each control packet
// (and on DS disconnect). Serialized because both paths can call it.
static wpi::mutex newDataCallbackMutex;
static void NewDSDataCallback() {
  std::scoped_lock callbackLock{newDataCallbackMutex};
  cacheToUpdate->Update();

  JoystickDataCache* given = cacheToUpdate;
  JoystickDataCache* prev = currentCache.exchange(cacheToUpdate);
  if (prev == nullptr) {
    cacheToUpdate = currentReadLocal;
    currentReadLocal = lastGiven;
  } else {
    // Current read local does not update
    cacheToUpdate = prev;
  }
  lastGiven = given;

  if (mau::vmxIO) {
    UpdateIOWatchdog(given->controlWord);
    if ((ds_packet_count++ % UPDATE_CAN_AND_POWER_EVERY) == 0) {
      UpdateCANAndPowerStatus();
    }
  }

  if (newDataEvents) {
    newDataEvents->Wakeup();
  }
}

namespace hal {
    namespace init {
        void InitializeDriverStation(void (*shutdown_handler)(int)) {
            std::memset(&newestControlWord, 0, sizeof(newestControlWord));
            static wpi::EventVector events;
            newDataEvents = &events;
            mau::comms::setShutdownHandler(shutdown_handler);
            Mau_DriveData::setNewDataCallback(NewDSDataCallback);
        }

        void TerminateDriverStation() {
            Mau_DriveData::setNewDataCallback(nullptr);
        	mau::comms::stop();
        }
    }

    // Starts the DS protocol threads. This was HAL_InitializeDriverStation()
    // in the 2020 HAL API; it is now called from HAL_Initialize().
    void StartDriverStationComms() {
	mau::comms::start_ds_protocol_threads();
	mau::comms::setRobotState(MAU_COMMS_STATE_DISABLED);
	mau::comms::setRobotESTOPActive(false);
    }
}

using namespace hal;

namespace hal {
static void DefaultPrintErrorImpl(const char* line, size_t size) {
  std::fwrite(line, size, 1, stderr);
}
}  // namespace hal

static std::atomic<void (*)(const char* line, size_t size)> gPrintErrorImpl{
    hal::DefaultPrintErrorImpl};

extern "C" {
    int32_t HAL_SendError(HAL_Bool isError, int32_t errorCode, HAL_Bool isLVCode,
                          const char* details, const char* location, const char* callStack, HAL_Bool printMsg) {
        // Avoid flooding console by keeping track of previous 5 error
        // messages and only printing again if they're longer than 1 second old.
        static constexpr int KEEP_MSGS = 5;
        std::lock_guard<wpi::mutex> lock(msgMutex);
        static std::string prevMsg[KEEP_MSGS];
        static std::chrono::time_point<std::chrono::steady_clock>
                prevMsgTime[KEEP_MSGS];
        static bool initialized = false;
        if (!initialized) {
            for (int i = 0; i < KEEP_MSGS; i++) {
                prevMsgTime[i] =
                        std::chrono::steady_clock::now() - std::chrono::seconds(2);
            }
            initialized = true;
        }

        auto curTime = std::chrono::steady_clock::now();
        int i;
        for (i = 0; i < KEEP_MSGS; ++i) {
            if (prevMsg[i] == details) break;
        }
        int retval = 0;
        if (i == KEEP_MSGS || (curTime - prevMsgTime[i]) >= std::chrono::seconds(1)) {

            uint16_t num_occur = 1;

            std::string_view detailsRef{details};
            std::string_view locationRef{location};
            std::string_view callStackRef{callStack};

            // 1 tag, 4 timestamp, 2 seqnum
            // 2 numOccur, 4 error code, 1 flags, 6 strlen
            // 1 extra needed for padding on Netcomm end.
            size_t baseLength = 21;

            if (baseLength + detailsRef.size() + locationRef.size() +
                    callStackRef.size() <=
                65536) {
                // Pass through
                retval = mau::comms::enqueueErrorMessage(num_occur, errorCode, isError ? 1 : 0, details, location, callStack);
            } else if (baseLength + detailsRef.size() > 65536) {
              // Details too long, cut both location and stack
              auto newLen = 65536 - baseLength;
              std::string newDetails{details, newLen};
              char empty = '\0';
              retval = mau::comms::enqueueErrorMessage(num_occur, errorCode, isError ? 1 : 0, newDetails.c_str(), &empty, &empty);
            } else if (baseLength + detailsRef.size() + locationRef.size() > 65536) {
              // Location too long, cut stack
              auto newLen = 65536 - baseLength - detailsRef.size();
              std::string newLocation{location, newLen};
              char empty = '\0';
              retval = mau::comms::enqueueErrorMessage(num_occur, errorCode, isError ? 1 : 0, details, newLocation.c_str(), &empty);
            } else {
              // Stack too long
              auto newLen = 65536 - baseLength - detailsRef.size() - locationRef.size();
              std::string newCallStack{callStack, newLen};
              retval = mau::comms::enqueueErrorMessage(num_occur, errorCode, isError ? 1 : 0, details, location, newCallStack.c_str());
            }
            if (printMsg) {
              fmt::memory_buffer buf;
              if (location && location[0] != '\0') {
                fmt::format_to(fmt::appender{buf},
                               "{} at {}: ", isError ? "Error" : "Warning", location);
              }
              fmt::format_to(fmt::appender{buf}, "{}\n", details);
              if (callStack && callStack[0] != '\0') {
                fmt::format_to(fmt::appender{buf}, "{}\n", callStack);
              }
              auto printError = gPrintErrorImpl.load();
              printError(buf.data(), buf.size());
            }
            if (i == KEEP_MSGS) {
                // replace the oldest one
                i = 0;
                auto first = prevMsgTime[0];
                for (int j = 1; j < KEEP_MSGS; ++j) {
                    if (prevMsgTime[j] < first) {
                        first = prevMsgTime[j];
                        i = j;
                    }
                }
                prevMsg[i] = details;
            }
            prevMsgTime[i] = curTime;
        }
        return retval;
    }

    void HAL_SetPrintErrorImpl(void (*func)(const char* line, size_t size)) {
      gPrintErrorImpl.store(func ? func : hal::DefaultPrintErrorImpl);
    }

    int32_t HAL_SendConsoleLine(const char* line) {
      std::string_view lineRef{line};
      if (lineRef.size() <= 65535) {
        // Send directly
        mau::comms::enqueuePrintMessage((char *)line);
        return 0;
      } else {
        // Need to truncate
        std::string newLine{line, 65535};
        mau::comms::enqueuePrintMessage((char *)newLine.c_str());
        return 0;
      }
    }

    int32_t HAL_GetControlWord(HAL_ControlWord* controlWord) {
        std::scoped_lock lock{cacheMutex};
        *controlWord = newestControlWord;
        return 0;
    }

    HAL_AllianceStationID HAL_GetAllianceStation(int32_t* status) {
        std::scoped_lock lock{cacheMutex};
        return currentRead->allianceStation;
    }

    int32_t HAL_GetJoystickAxes(int32_t joystickNum, HAL_JoystickAxes* axes) {
        CHECK_JOYSTICK_NUMBER(joystickNum);
        std::scoped_lock lock{cacheMutex};
        *axes = currentRead->axes[joystickNum];
        return 0;
    }

    int32_t HAL_GetJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs* povs) {
        CHECK_JOYSTICK_NUMBER(joystickNum);
        std::scoped_lock lock{cacheMutex};
        *povs = currentRead->povs[joystickNum];
        return 0;
    }

    int32_t HAL_GetJoystickButtons(int32_t joystickNum, HAL_JoystickButtons* buttons) {
        CHECK_JOYSTICK_NUMBER(joystickNum);
        std::scoped_lock lock{cacheMutex};
        *buttons = currentRead->buttons[joystickNum];
        return 0;
    }

    void HAL_GetAllJoystickData(HAL_JoystickAxes* axes, HAL_JoystickPOVs* povs,
                                HAL_JoystickButtons* buttons) {
        std::scoped_lock lock{cacheMutex};
        std::memcpy(axes, currentRead->axes, sizeof(currentRead->axes));
        std::memcpy(povs, currentRead->povs, sizeof(currentRead->povs));
        std::memcpy(buttons, currentRead->buttons, sizeof(currentRead->buttons));
    }

    int32_t HAL_GetJoystickDescriptor(int32_t joystickNum, HAL_JoystickDescriptor* desc) {
        CHECK_JOYSTICK_NUMBER(joystickNum);
        std::scoped_lock lock{tcpCacheMutex};
        *desc = tcpCurrent.descriptors[joystickNum];
        return 0;
    }

    HAL_Bool HAL_GetJoystickIsXbox(int32_t joystickNum) {
        HAL_JoystickDescriptor joystickDesc;
        if (HAL_GetJoystickDescriptor(joystickNum, &joystickDesc) < 0) {
            return 0;
        }
        return joystickDesc.isXbox;
    }

    int32_t HAL_GetJoystickType(int32_t joystickNum) {
        HAL_JoystickDescriptor joystickDesc;
        if (HAL_GetJoystickDescriptor(joystickNum, &joystickDesc) < 0) {
            return -1;
        }
        return joystickDesc.type;
    }

    void HAL_GetJoystickName(struct WPI_String* name, int32_t joystickNum) {
        HAL_JoystickDescriptor joystickDesc;
        const char* cName = joystickDesc.name;
        if (HAL_GetJoystickDescriptor(joystickNum, &joystickDesc) < 0) {
            cName = "";
        }
        auto len = std::strlen(cName);
        auto write = WPI_AllocateString(name, len);
        std::memcpy(write, cName, len);
    }

    int32_t HAL_GetJoystickAxisType(int32_t joystickNum, int32_t axis) {
        HAL_JoystickDescriptor joystickDesc;
        if (HAL_GetJoystickDescriptor(joystickNum, &joystickDesc) < 0) {
            return -1;
        }
        return joystickDesc.axisTypes[axis];
    }

    int32_t HAL_SetJoystickOutputs(int32_t joystickNum, int64_t outputs, int32_t leftRumble, int32_t rightRumble) {
        CHECK_JOYSTICK_NUMBER(joystickNum);
        Mau_DriveData::updateJoyOutputs(joystickNum, outputs, leftRumble, rightRumble);
        return 0;
    }

    double HAL_GetMatchTime(int32_t* status) {
        std::scoped_lock lock{cacheMutex};
        return currentRead->matchTime;
    }

    int32_t HAL_GetMatchInfo(HAL_MatchInfo* info) {
        std::scoped_lock lock{tcpCacheMutex};
        *info = tcpCurrent.matchInfo;
        return 0;
    }

    HAL_Bool HAL_RefreshDSData(void) {
        JoystickDataCache* prev;
        {
            std::scoped_lock lock{cacheMutex};
            prev = currentCache.exchange(nullptr);
            if (prev != nullptr) {
                currentRead = prev;
            }
            if (!currentRead->controlWord.dsAttached) {
                // With no DS attached the rest of the control word is stale.
                std::memset(&currentRead->controlWord, 0,
                            sizeof(currentRead->controlWord));
            }
            newestControlWord = currentRead->controlWord;
        }

        // Descriptors and match info arrive over TCP and change rarely; take a
        // fresh copy along with each new control packet.
        if (prev != nullptr) {
            TcpCache tcpCache;
            tcpCache.Update();
            std::scoped_lock tcpLock(tcpCacheMutex);
            tcpCurrent = tcpCache;
        }
        return prev != nullptr;
    }

    void HAL_ProvideNewDataEventHandle(WPI_EventHandle handle) {
        hal::init::CheckInit();
        newDataEvents->Add(handle);
    }

    void HAL_RemoveNewDataEventHandle(WPI_EventHandle handle) {
        newDataEvents->Remove(handle);
    }

    // Outputs are enabled while the VMX-pi IO watchdog is being fed.
    HAL_Bool HAL_GetOutputsEnabled(void) {
        return mau::vmxIOActive;
    }

// Invoked when user program starts; Purpose:  Tell the DS that the robot is ready to be enabled
void HAL_ObserveUserProgramStarting(void) {
	mau::comms::setRobotProgramStarted(true);
}

// Invoked once when user program is disabled
void HAL_ObserveUserProgramDisabled(void) {
	mau::comms::setRobotState(MAU_COMMS_STATE_DISABLED);
}

// Invoked once when user program enters autonomous mode
void HAL_ObserveUserProgramAutonomous(void) {
	mau::comms::setRobotState(MAU_COMMS_STATE_AUTONOMOUS);
}

void HAL_ObserveUserProgramTeleop(void) {
	mau::comms::setRobotState(MAU_COMMS_STATE_TELEOP);
}

void HAL_ObserveUserProgramTest(void) {
	mau::comms::setRobotState(MAU_COMMS_STATE_TEST);
}
}  // extern "C"
