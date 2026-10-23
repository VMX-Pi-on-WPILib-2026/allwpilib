// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md.
#include <cassert>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "MauMap.h"
#include "Translator/include/FileHandler.h"

int main() {
  Mau_FileHandler handler;
  auto* converter = handler.getEnumConverter();
  // This must be resolved from the actual 2026 enum (DIO=48), not 2020's 1.
  assert(converter->getHandleLabel(hal::HAL_HandleEnum::DIO) == "DIO");
  assert(converter->getHandleValue("DIO") == hal::HAL_HandleEnum::DIO);
  assert(converter->getHandleValue("PWM") == hal::HAL_HandleEnum::PWM);
  assert(converter->getHandleLabel(hal::HAL_HandleEnum::AddressableLED) == "AddressableLED");
  std::unique_ptr<Mau_ChannelMap> map(handler.readChannelMap("/nonexistent/vmx-regression-map.json"));
  // Reproduce Sensor::Init LED_GREEN=18 / LED_RED=19 in the real translator.
  const auto dio = converter->getHandleLabel(hal::HAL_HandleEnum::DIO);
  assert(map->getChannelInfo(dio, 18).index == 18);
  assert(map->getChannelInfo(dio, 19).index == 19);
  assert(map->getChannelInfo(dio, 22).index == 26);
  assert(map->getChannelInfo(dio, 29).index == 33);
  assert(map->getChannelInfo("PWM", 0).index == 12);
  assert(map->getChannelInfo("PWM", 17).index == 7);
  assert(map->getChannelInfo("AnalogInput", 0).index == 22);
  assert(!map->getChannelInfo("AnalogInput", 4).IsValid());
  assert(!map->getChannelInfo("AnalogOutput", 0).IsValid());
  for (const char* label : {"Counter", "DigitalPWM"}) {
    assert(map->getChannel(label, 18) == map->getChannel("DIO", 18));
  }
  assert(map->getChannel("AnalogTrigger", 0) == map->getChannel("AnalogInput", 0));
  assert(map->getChannel("Interrupt", 0) == map->getChannel("AnalogInput", 0));
  assert(map->getChannel("Relay", 0) == map->getChannel("PWM", 0));
  for (const char* label : {"DIO", "Counter", "Relay", "missing", "Not Found!", "Encoder"}) {
    assert(!map->getChannelInfo(label, -1).IsValid());
    assert(!map->getChannelInfo(label, 256).IsValid());
  }
  assert(!map->getChannelInfo("DIO", 30).IsValid());
  assert(!map->getChannelInfo("missing", 0).IsValid());
  assert(!map->getChannelInfo("Encoder", 0).IsValid());
  map->initializeChannel("missing", 0, VMXChannelCapability::DigitalOutput, nullptr);
  map->initializeChannel("DIO", 18, VMXChannelCapability::DigitalOutput, nullptr);
  assert(map->getChannelInfo("Counter", 18).capabilities == VMXChannelCapability::DigitalOutput);

  // Exercise override loading using an old EnumValue; the value must be ignored.
  const auto file = std::filesystem::current_path() / "channel-override.json";
  std::ofstream(file) << R"({"DIO":{"EnumValue":1,"Count":2,"FirstMXP":1,
      "ReferenceTo":"VMXChannel","Channels":{"0":11,"1":12}},
      "Counter":{"EnumValue":11,"Channels":"DIO"}})";
  std::unique_ptr<Mau_ChannelMap> custom(handler.readChannelMap(file.c_str()));
  assert(custom->getChannelInfo(dio, 0).index == 11);
  assert(custom->getChannelInfo("Counter", 1).index == 12);
  assert(!custom->getChannelInfo("DIO", 2).IsValid());
  assert(!custom->getChannelInfo("PWM", 0).IsValid());
  for (const char* invalid : {"{", "[]", R"({"DIO":{"Count":-1,"Channels":{}}})",
        R"({"DIO":{"Count":2,"FirstMXP":0,"ReferenceTo":"VMXChannel","Channels":{"0":0}}})"}) {
    std::ofstream(file) << invalid;
    try {
      std::unique_ptr<Mau_ChannelMap> bad(handler.readChannelMap(file.c_str()));
      assert(false);
    } catch (const std::runtime_error&) {}
  }
  std::filesystem::remove(file);
}
