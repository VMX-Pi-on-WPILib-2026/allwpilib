// Host regression tests against the actual VMX HAL sources; no robot required.
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <span>
#include <string>
#include <vector>
#include <cstdarg>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "AnalogInternal.h"
#include "AnalogTriggerInternal.h"
#include "DigitalInternal.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "DriveStation/include/DriverComms.hpp"
#include "DriveStation/include/MauDriveData.h"
#include "DriveStation/include/TcpPacketBuffer.h"
#include "hal/AnalogInput.h"
#include "hal/I2C.h"
#include "hal/PWM.h"
#include "hal/SPI.h"

namespace wpi {
[[noreturn]] void report_bad_alloc_error(const char*, bool) { throw std::bad_alloc{}; }
}

// Minimal SDK transport fakes. HAL logic and its handle tables remain real.
class PIGPIOClient {};
class IOCXClient {};
class MISCClient {};
class VMXChannelManager {};
class VMXResourceManager {};
VMXTime::VMXTime(PIGPIOClient& p, MISCClient& m) : pigpio(p), misc(m), p_impl(nullptr) {}
VMXTime::~VMXTime() = default;
uint64_t VMXTime::GetCurrentTotalMicroseconds() { return 1000000; }
VMXIO::VMXIO(PIGPIOClient& p, IOCXClient& i, MISCClient& m, VMXChannelManager& c,
             VMXResourceManager& r, VMXTime& t)
    : pigpio(p), iocx(i), misc(m), chan_mgr(c), res_mgr(r), time(t),
      p_int_sink(nullptr), p_pulse_manager(nullptr), p_auto_transaction_mgr(nullptr) {}
VMXIO::~VMXIO() = default;
class VMXPi {
 public:
  static VMXTime& clock() {
    static PIGPIOClient p; static MISCClient m; static VMXTime t(p, m); return t;
  }
  static VMXIO& io() {
    static PIGPIOClient p; static IOCXClient i; static MISCClient m;
    static VMXChannelManager c; static VMXResourceManager r;
    static VMXIO io(p, i, m, c, r, clock()); return io;
  }
};
namespace mau {
VMXTime* vmxTime = &VMXPi::clock();
VMXIO* vmxIO = &VMXPi::io();
}
namespace hal::init {
std::atomic_bool HAL_IsInitialized{true};
void RunInitialize() { assert(false); }
}
namespace {
int opens = 0, closes = 0, transactions = 0;
bool failOpen = false, lastCsActiveLow = true, probeAck = true;
int probeOpens = 0, probeCloses = 0, probeTransfers = 0;
uint16_t lastPulse = 0;
float fullScale = 5.0;
std::vector<uint8_t> lastWrite;
int32_t errorCode = 0;
std::string errorMessage;
}
extern "C" int __wrap_open(const char* path, int, ...) {
  assert(std::strcmp(path, "/dev/i2c-1") == 0);
  ++probeOpens;
  return 123;
}
extern "C" int __wrap_close(int fd) { assert(fd == 123); ++probeCloses; return 0; }
extern "C" int __wrap_ioctl(int fd, unsigned long request, ...) {
  assert(fd == 123 && request == I2C_RDWR);
  va_list args; va_start(args, request);
  auto* transfer = va_arg(args, i2c_rdwr_ioctl_data*); va_end(args);
  assert(transfer->nmsgs == 1 && transfer->msgs[0].addr == 0x20 && transfer->msgs[0].len == 0);
  ++probeTransfers;
  return probeAck ? 1 : -1;
}
namespace hal {
DigitalHandleResource<HAL_DigitalHandle, DigitalPort, kNumDigitalChannels>* digitalChannelHandles;
IndexedHandleResource<HAL_AnalogInputHandle, AnalogPort, kNumAnalogInputs,
                       HAL_HandleEnum::AnalogInput>* analogInputHandles;
void SetLastError(int32_t* status, std::string_view message) {
  errorCode = *status;
  errorMessage = message;
  *status = HAL_USE_LAST_ERROR;
}
}
VMXChannelIndex VMXIO::GetSoleChannelIndex(VMXChannelCapability) { return 0; }
bool VMXIO::ActivateDualchannelResource(const VMXChannelInfo&, const VMXChannelInfo&,
    const VMXResourceConfig*, VMXResourceHandle& handle, VMXErrorCode* status) {
  if (failOpen) { *status = -1; return false; }
  ++opens; handle = CREATE_VMX_RESOURCE_HANDLE(VMXResourceType::I2C, 0); *status = 0; return true;
}
bool VMXIO::ActivateQuadchannelResource(const VMXChannelInfo&, const VMXChannelInfo&,
    const VMXChannelInfo&, const VMXChannelInfo&, const VMXResourceConfig* config,
    VMXResourceHandle& handle, VMXErrorCode* status) {
  lastCsActiveLow = static_cast<const SPIConfig*>(config)->cs_active_low;
  handle = CREATE_VMX_RESOURCE_HANDLE(VMXResourceType::SPI, 0); *status = 0; return true;
}
bool VMXIO::DeallocateResource(VMXResourceHandle, VMXErrorCode* status) { ++closes; *status = 0; return true; }
bool VMXIO::I2C_Transaction(VMXResourceHandle, uint8_t, uint8_t* send, uint16_t size,
    uint8_t* receive, uint16_t count, VMXErrorCode* status) {
  ++transactions; lastWrite.clear();
  if (size) lastWrite.assign(send, send + size);
  if (count) std::memset(receive, 0x42, count);
  *status = 0; return true;
}
bool VMXIO::PWMGenerator_SetDutyCycle(VMXResourceHandle, VMXResourcePortIndex,
    uint16_t value, VMXErrorCode* status) { lastPulse = value; *status = 0; return true; }
bool VMXIO::PWMGenerator_GetDutyCycle(VMXResourceHandle, VMXResourcePortIndex,
    uint16_t* value, VMXErrorCode* status) { *value = lastPulse; *status = 0; return true; }
bool VMXIO::Accumulator_GetFullScaleVoltage(float& value, VMXErrorCode* status) {
  value = fullScale; *status = 0; return true;
}

static void tcp(std::vector<uint8_t> packet) {
  mau::comms::decodeTcpPacket(reinterpret_cast<char*>(packet.data()), packet.size());
}
static int udp(std::vector<uint8_t> packet) {
  return mau::comms::decodeUdpPacket(reinterpret_cast<char*>(packet.data()), packet.size());
}
static int callbacks = 0;
static void testDriverStation() {
  Mau_DriveData::initializeDriveData();
  Mau_DriveData::setNewDataCallback([] { ++callbacks; });
  Mau_DriveData::updateJoyOutputs(0, 0, 1, 1);  // Must return and release the DS lock.
  Mau_DriveData::updateDSAttached(true);
  assert(callbacks == 1);
  for (int id : {-1, 6, 255}) {
    HAL_JoystickDescriptor desc{};
    Mau_DriveData::updateJoyDescriptor(id, &desc);
    Mau_DriveData::updateJoyOutputs(id, 0, 0, 0);
  }
  uint16_t povValues[] = {90, 180, 270};
  Mau_DriveData::updateJoyPOV(0, 3, povValues);
  HAL_JoystickPOVs pov{};
  Mau_DriveData::scribeJoyPOVs(0, &pov);
  assert(pov.count == 3 && pov.povs[1] == 180 && pov.povs[2] == 270);

  // Invalid joystick ID is ignored, and a subsequent valid descriptor still works.
  tcp({0,8,2,6,1,1,0,0,0,0});
  std::vector<uint8_t> descriptor{0,9,2,0,1,2,1,'X',0,0,0};
  mau::comms::TcpPacketBuffer stream;
  for (uint8_t byte : descriptor) stream.append({&byte, 1}, [](std::span<const uint8_t> frame) {
    mau::comms::decodeTcpPacket(reinterpret_cast<char*>(const_cast<uint8_t*>(frame.data())), frame.size());
  });
  HAL_JoystickDescriptor desc{}; Mau_DriveData::scribeJoyDescriptor(0, &desc);
  assert(std::strcmp(desc.name, "X") == 0);
  std::vector<uint8_t> events{0,206,7,200};
  events.insert(events.end(), 200, 'A');
  events.insert(events.end(), {1,0x12,0x34,2});
  tcp(events);
  HAL_MatchInfo info{}; Mau_DriveData::scribeMatchInfo(&info);
  assert(std::strlen(info.eventName) == 63 && info.matchNumber == 0x1234 && info.replayNumber == 2);
  std::vector<uint8_t> game{1,45,14}; game.insert(game.end(), 300, 'B');
  tcp(game); Mau_DriveData::scribeMatchInfo(&info);
  assert(info.gameSpecificMessageSize == 64 && info.gameSpecificMessage[63] == 'B');

  // Combined frames and reconnect reset must not mix connections.
  int frames = 0;
  std::vector<uint8_t> combined = descriptor;
  combined.insert(combined.end(), descriptor.begin(), descriptor.end());
  stream.append(combined, [&](auto) { ++frames; }); assert(frames == 2);
  stream.append(std::span<const uint8_t>{descriptor}.first(1), [&](auto) { assert(false); });
  stream.reset(); stream.append(descriptor, [&](auto) { ++frames; }); assert(frames == 3);

  // Three POVs and 32 buttons; removing the joystick clears old input.
  std::vector<uint8_t> control{0,1,1,4,0,1, 14,12,0,32,1,2,3,4,3,0,90,0,180,1,14};
  assert(udp(control) == MAU_COMMS_SHUTDOWN_NONE);
  HAL_JoystickButtons buttons{}; Mau_DriveData::scribeJoyButtons(0, &buttons);
  assert(buttons.buttons == 0x01020304);
  Mau_DriveData::scribeJoyPOVs(0, &pov);
  assert(pov.count == 3 && pov.povs[1] == 180 && pov.povs[2] == 270);
  udp({0,2,1,0,0,1}); Mau_DriveData::scribeJoyButtons(0, &buttons); assert(buttons.count == 0);
  assert(udp({0,3,1,0,8,1}) == MAU_COMMS_SHUTDOWN_REBOOT);
  assert(udp({0,4,1,0,4,1}) == MAU_COMMS_SHUTDOWN_RESTART);
  assert(udp({0,5,1,128,0,1}) == MAU_COMMS_SHUTDOWN_ESTOP);

  // Truncation at every byte, excessive counts and arbitrary bytes must be safe.
  for (size_t n = 0; n < descriptor.size(); ++n) tcp({descriptor.begin(), descriptor.begin() + n});
  for (size_t n = 0; n < events.size(); ++n) tcp({events.begin(), events.begin() + n});
  for (size_t n = 7; n < control.size(); ++n) {
    int before = callbacks; udp({control.begin(), control.begin() + n}); assert(before == callbacks);
  }
  // The HAL permits twelve POVs (the old temporary buffer had only four).
  std::vector<uint8_t> maxPovs{0,1,1,4,0,1,28,12,0,0,12};
  for (int i = 0; i < 12; ++i) { maxPovs.push_back(0); maxPovs.push_back(i); }
  udp(maxPovs); Mau_DriveData::scribeJoyPOVs(0, &pov);
  assert(pov.count == 12 && pov.povs[11] == 11);
  for (auto malformed : std::vector<std::vector<uint8_t>>{
           {0,1,1,4,0,1,4,12,13,0,0},
           {0,1,1,4,0,1,4,12,0,33,0},
           {0,1,1,4,0,1,4,12,0,0,13}}) {
    int before = callbacks; udp(malformed); assert(before == callbacks);
  }
  auto tooMany = std::vector<uint8_t>{0,1,1,4,0,1};
  for (int i = 0; i < 7; ++i) tooMany.insert(tooMany.end(), {4,12,0,0,0});
  int before = callbacks; udp(tooMany); assert(before == callbacks);
  std::mt19937 random(2026);
  for (int i = 0; i < 5000; ++i) {
    std::vector<uint8_t> bytes(random() % 256);
    for (auto& b : bytes) b = random();
    tcp(bytes); udp(bytes);
  }
  Mau_DriveData::setNewDataCallback(nullptr);
  std::puts("PASS DS: locks, indices, names, framing, POV/buttons, malformed TCP/UDP");
}

static void testI2C() {
  int32_t status = 0;
  HAL_InitializeI2C(HAL_I2C_kOnboard, &status); assert(status == 0 && opens == 1);
  HAL_InitializeI2C(HAL_I2C_kOnboard, &status); assert(status == 0 && opens == 1);
  HAL_InitializeI2C(HAL_I2C_kMXP, &status); assert(status == 0 && opens == 1);
  assert(HAL_TransactionI2C(HAL_I2C_kOnboard, 0x20, nullptr, 0, nullptr, 0) == 0);
  assert(probeTransfers == 1 && probeOpens == probeCloses);
  probeAck = false;
  assert(HAL_TransactionI2C(HAL_I2C_kOnboard, 0x20, nullptr, 0, nullptr, 0) == -1);
  assert(probeTransfers == 2 && probeOpens == probeCloses);
  probeAck = true;
  uint8_t payload[] = {0x12, 0x34};
  assert(HAL_WriteI2C(HAL_I2C_kOnboard, 0x20, payload, 2) == 0 && lastWrite == std::vector<uint8_t>({0x12,0x34}));
  uint8_t response = 0; assert(HAL_ReadI2C(HAL_I2C_kMXP, 0x21, &response, 1) == 0 && response == 0x42);
  int before = transactions;
  assert(HAL_TransactionI2C(HAL_I2C_kOnboard, 0x20, nullptr, 1, nullptr, 0) == -1);
  assert(HAL_TransactionI2C(HAL_I2C_kOnboard, 0x20, payload, -1, nullptr, 0) == -1);
  assert(HAL_TransactionI2C(HAL_I2C_kOnboard, 0x20, payload, 65536, nullptr, 0) == -1);
  assert(before == transactions);
  HAL_CloseI2C(HAL_I2C_kOnboard); HAL_CloseI2C(HAL_I2C_kOnboard); assert(closes == 0);
  HAL_CloseI2C(HAL_I2C_kMXP); assert(closes == 1);
  HAL_CloseI2C(HAL_I2C_kMXP); assert(closes == 1);
  failOpen = true; HAL_InitializeI2C(HAL_I2C_kOnboard, &status); assert(status != 0);
  failOpen = false; status = 0; HAL_InitializeI2C(HAL_I2C_kOnboard, &status); assert(status == 0 && opens == 2);
  HAL_CloseI2C(HAL_I2C_kOnboard);
  std::puts("PASS I2C: shared lifetime, address-only, raw payload, input validation, retry");
}

static void testHardwareConversions() {
  int32_t status = 0;
  HAL_InitializeSPI(HAL_SPI_kOnboardCS0, &status); assert(status == 0);
  HAL_SetSPIChipSelectActiveHigh(HAL_SPI_kOnboardCS0, &status); assert(!lastCsActiveLow);
  HAL_SetSPIChipSelectActiveLow(HAL_SPI_kOnboardCS0, &status); assert(lastCsActiveLow);
  HAL_CloseSPI(HAL_SPI_kOnboardCS0);
  HAL_InitSPIAuto(HAL_SPI_kOnboardCS0, 1024, &status);
  assert(status == HAL_USE_LAST_ERROR && errorCode == MAU_UNSUPPORTED_FEATURE && !errorMessage.empty());
  uint32_t guard = 0x12345678; status = 0;
  assert(HAL_ReadSPIAutoReceivedData(HAL_SPI_kOnboardCS0, &guard, 1, 0, &status) == 0);
  assert(status == HAL_USE_LAST_ERROR && guard == 0x12345678);

  static hal::DigitalHandleResource<HAL_DigitalHandle, hal::DigitalPort, hal::kNumDigitalChannels> digital;
  hal::digitalChannelHandles = &digital;
  HAL_DigitalHandle pwm; status = 0;
  digital.Allocate(0, hal::HAL_HandleEnum::PWM, &pwm, &status); assert(status == 0);
  HAL_SetPWMConfigMicroseconds(pwm, 2000, 1450, 1400, 1350, 1000, &status);
  HAL_SetPWMSpeed(pwm, 0, &status); assert(status == 0 && lastPulse == 1400);
  HAL_SetPWMSpeed(pwm, 0.5, &status); assert(std::abs(HAL_GetPWMSpeed(pwm, &status)-0.5) < .002);
  HAL_SetPWMEliminateDeadband(pwm, true, &status);
  HAL_SetPWMSpeed(pwm, 0.5, &status); assert(lastPulse == 1725);
  HAL_SetPWMSpeed(pwm, -0.5, &status); assert(lastPulse == 1175);
  HAL_SetPWMSpeed(pwm, std::numeric_limits<double>::quiet_NaN(), &status); assert(lastPulse == 1400);
  HAL_SetPWMSpeed(pwm, 100, &status); assert(lastPulse == 2000);

  HAL_DigitalHandle dio;
  auto dioPort = digital.Allocate(1, hal::HAL_HandleEnum::DIO, &dio, &status);
  dioPort->configSet = true; dioPort->minPwm = 1000; dioPort->centerPwm = 1500;
  dioPort->maxPwm = 2000;
  HAL_SetPWMSpeed(dio, 0, &status); assert(status == 0 && lastPulse == 1500);

  static hal::IndexedHandleResource<HAL_AnalogInputHandle, hal::AnalogPort, hal::kNumAnalogInputs,
                             hal::HAL_HandleEnum::AnalogInput> analog;
  hal::analogInputHandles = &analog;
  HAL_AnalogInputHandle input; analog.Allocate(0, &input, &status); assert(status == 0);
  auto weight = HAL_GetAnalogLSBWeight(input, &status); assert(weight == 1220703);
  auto raw = HAL_GetAnalogVoltsToValue(input, 2.5, &status); assert(raw >= 2047 && raw <= 2048);
  assert(std::abs(HAL_GetAnalogValueToVolts(input, 2048, &status)-2.5) < .00001);
  fullScale = 3.3; assert(HAL_GetAnalogLSBWeight(input, &status) > 800000);
  fullScale = 0; assert(HAL_GetAnalogLSBWeight(input, &status) == 0 && status != 0);

  static hal::LimitedHandleResource<HAL_AnalogTriggerHandle, hal::AnalogTriggerResource,
      hal::kNumAnalogTriggers, hal::HAL_HandleEnum::AnalogTrigger> triggers;
  hal::analogTriggerHandles = &triggers;
  auto trigger = hal::analogTriggerHandles->Allocate();
  status = 0; HAL_SetAnalogTriggerAveraged(trigger, false, &status); assert(status == 0);
  HAL_SetAnalogTriggerAveraged(trigger, true, &status); assert(status == HAL_USE_LAST_ERROR && errorCode < 0);
  status = 0; HAL_SetAnalogTriggerFiltered(trigger, true, &status); assert(status == HAL_USE_LAST_ERROR && errorCode < 0);
  std::puts("PASS hardware: SPI polarity/errors, PWM scaling/NaN, ADC conversion, trigger errors");
}
int main() {
  testDriverStation(); testI2C(); testHardwareConversions();
}
