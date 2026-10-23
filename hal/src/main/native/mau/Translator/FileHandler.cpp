#include "Translator/include/FileHandler.h"

#include <memory>
#include <stdexcept>
#include <utility>

// Location of VMX-pi override maps

const constexpr char CHANNEL_MAP_JSON[] = "/usr/local/frc/third-party/lib/Maps/ChannelMap.json";

static constexpr char DEFAULT_CHANNEL_MAP_JSON[] = "\
{\
  \"DIO\": {\
    \"EnumValue\":1,\
    \"ReferenceTo\":\"VMXChannel\",\
    \"Count\":30,\
    \"FirstMXP\":10,\
    \"Channels\": {\
      \"0\":0,\
      \"1\":1,\
      \"2\":2,\
      \"3\":3,\
      \"4\":4,\
      \"5\":5,\
      \"6\":6,\
      \"7\":7,\
      \"8\":8,\
      \"9\":9,\
      \"10\":10,\
      \"11\":11,\
      \"12\":12,\
      \"13\":13,\
      \"14\":14,\
      \"15\":15,\
      \"16\":16,\
      \"17\":17,\
      \"18\":18,\
      \"19\":19,\
      \"20\":20,\
      \"21\":21,\
      \"22\":26,\
      \"23\":27,\
      \"24\":28,\
      \"25\":29,\
      \"26\":30,\
      \"27\":31,\
      \"28\":32,\
      \"29\":33\
    }\
  },\
\
  \"Interrupt\": {\
    \"EnumValue\":4,\
    \"Channels\":\"AnalogInput\"\
  },\
\
  \"AnalogOutput\": {\
    \"EnumValue\":5,\
    \"ReferenceTo\":\"VMXChannel\",\
    \"Count\":2,\
    \"FirstMXP\":0,\
    \"Channels\": {\
      \"0\":-1,\
      \"1\":-1\
    }\
  },\
\
  \"AnalogInput\": {\
    \"EnumValue\":6,\
    \"ReferenceTo\":\"VMXChannel\",\
    \"Count\":8,\
    \"FirstMXP\":4,\
    \"Channels\": {\
      \"0\":22,\
      \"1\":23,\
      \"2\":24,\
      \"3\":25,\
      \"4\":-1,\
      \"5\":-1,\
      \"6\":-1,\
      \"7\":-1\
    }\
  },\
\
  \"AnalogTrigger\": {\
    \"EnumValue\":7,\
    \"Channels\":\"AnalogInput\"\
  },\
\
  \"Relay\": {\
    \"EnumValue\":8,\
    \"Channels\":\"PWM\"\
  },\
\
  \"PWM\": {\
    \"EnumValue\":9,\
    \"ReferenceTo\":\"VMXChannel\",\
    \"Count\":28,\
    \"FirstMXP\":10,\
    \"Channels\": {\
      \"0\":12,\
      \"1\":13,\
      \"2\":14,\
      \"3\":15,\
      \"4\":16,\
      \"5\":17,\
      \"6\":18,\
      \"7\":19,\
      \"8\":20,\
      \"9\":21,\
      \"10\":0,\
      \"11\":1,\
      \"12\":2,\
      \"13\":3,\
      \"14\":4,\
      \"15\":5,\
      \"16\":6,\
      \"17\":7,\
      \"18\":8,\
      \"19\":9,\
      \"20\":10,\
      \"21\":11,\
      \"22\":26,\
      \"23\":27,\
      \"24\":28,\
      \"25\":30,\
      \"26\":31,\
      \"27\":33\
    }\
  },\
\
  \"DigitalPWM\": {\
    \"EnumValue\":10,\
    \"Channels\":\"DIO\"\
  },\
\
  \"Counter\": {\
    \"EnumValue\":11,\
    \"Channels\":\"DIO\"\
  },\
\
  \"FPGAEncoder\": {\
    \"EnumValue\":13,\
    \"Channels\":\"Encoder\"\
  },\
\
  \"Encoder\": {\
    \"EnumValue\":13,\
    \"Count\":10,\
    \"Channels\": {\
      \"0\": { \"DIO\":0, \"Pair\":1, \"EncoderInput\":\"A\" },\
      \"1\": { \"DIO\":1, \"Pair\":0, \"EncoderInput\":\"B\" },\
      \"2\": { \"DIO\":2, \"Pair\":3, \"EncoderInput\":\"A\" },\
      \"3\": { \"DIO\":3, \"Pair\":2, \"EncoderInput\":\"B\" },\
      \"4\": { \"DIO\":4, \"Pair\":5, \"EncoderInput\":\"A\" },\
      \"5\": { \"DIO\":5, \"Pair\":4, \"EncoderInput\":\"B\" },\
      \"6\": { \"DIO\":6, \"Pair\":7, \"EncoderInput\":\"A\" },\
      \"7\": { \"DIO\":7, \"Pair\":6, \"EncoderInput\":\"B\" },\
      \"8\": { \"DIO\":8, \"Pair\":9, \"EncoderInput\":\"A\" },\
      \"9\": { \"DIO\":9, \"Pair\":8, \"EncoderInput\":\"B\" }\
    }\
  },\
\
  \"SPI\": {\
    \"Count\":4,\
    \"WPIPort\": {\
      \"0\":33,\
      \"1\":-1,\
      \"2\":-1,\
      \"3\":-1\
    },\
    \"MXPCount\":1,\
    \"MXPPort\": {\
      \"4\":-1\
    }\
  },\
\
  \"I2C\": {\
    \"Count\":1,\
    \"WPIPort\": {\
      \"0\":[26,27]\
    },\
    \"MXPCount\":1,\
    \"MXPPort\": {\
      \"1\":-1\
    }\
  },\
\
  \"Serial\": {\
    \"Count\":3,\
    \"WPIPort\": {\
      \"0\":-1,\
      \"2\":-1,\
      \"3\":-1\
    },\
    \"MXPCount\":1,\
    \"MXPPort\": {\
      \"1\":[28,29]\
    }\
  }\
}\
";

// -------- Generators -------- //

Mau_Channel* Mau_FileHandler::genGroupChannels(Mau_ChannelMap* mauMap, const char* labelVal, rapidjson::Document* doc) {
    auto& group = (*doc)[labelVal];
    if (!group.HasMember("Count") || !group["Count"].IsInt() ||
        group["Count"].GetInt() <= 0 || group["Count"].GetInt() > 256 ||
        !group.HasMember("FirstMXP") || !group["FirstMXP"].IsInt() ||
        !group.HasMember("ReferenceTo") || !group["ReferenceTo"].IsString() ||
        std::string(group["ReferenceTo"].GetString()) != "VMXChannel" ||
        !group["Channels"].IsObject()) {
        throw std::runtime_error(std::string("Invalid VMX channel group: ") + labelVal);
    }
    int count = group["Count"].GetInt();
    auto channels = std::make_unique<Mau_Channel[]>(count);
    for (int index = 0; index < count; ++index) {
        const std::string key = std::to_string(index);
        if (!group["Channels"].HasMember(key.c_str()) ||
            !group["Channels"][key.c_str()].IsInt()) {
            throw std::runtime_error(std::string("Missing VMX channel: ") + labelVal + "/" + key);
        }
        int reference = group["Channels"][key.c_str()].GetInt();
        if (reference < -1 || reference >= INVALID_VMX_CHANNEL_INDEX) {
            throw std::runtime_error("Invalid VMX channel index");
        }
        auto& channel = channels[index];
        channel.wpiIndex = index;
        channel.isMXP = index >= group["FirstMXP"].GetInt();
        channel.unsupported = reference == -1;
        if (!channel.unsupported) channel.vmxIndex = reference;
    }
    return channels.release();
}

void Mau_FileHandler::genHandleGroup(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc, Mau_EnumConverter* enums) {
    auto& group = (*doc)[label.c_str()];
    if (group["Channels"].IsString()) {
        mauMap->setChannelAsReference(label, group["Channels"].GetString());
        return;  // Do not replace the reference with a group containing null channels.
    }
    auto* channels = genGroupChannels(mauMap, label.c_str(), doc);
    // JSON EnumValue belongs to the old image format, not the current HAL ABI.
    // DIO is 48 in WPILib 2026, even when an override still contains EnumValue: 1.
    mauMap->setGroup(label, std::make_shared<Mau_HandledGroup>(
        channels, group["Count"].GetInt(), enums->getHandleValue(label)));
}

void Mau_FileHandler::genEncoderChannel(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc, Mau_EnumConverter* enums) {
    //Mau_Channel channel;
}

void Mau_FileHandler::genSPIChannel(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc, Mau_EnumConverter* enums) {
    //Mau_Channel channel;
}

void Mau_FileHandler::genI2CChannel(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc, Mau_EnumConverter* enums) {
    //Mau_Channel channel;
}

void Mau_FileHandler::genSerialChannel(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc, Mau_EnumConverter* enums) {
    //Mau_Channel channel;
}

// -------- Readers -------- //

Mau_EnumConverter* Mau_FileHandler::readEnums() {
    auto* converter = new Mau_EnumConverter();
    // Bind labels to C++ enum constants; never duplicate enum values in JSON.
    converter->setHandlePair("Undefined", hal::HAL_HandleEnum::Undefined);
    converter->setHandlePair("DIO", hal::HAL_HandleEnum::DIO);
    converter->setHandlePair("Port", hal::HAL_HandleEnum::Port);
    converter->setHandlePair("Notifier", hal::HAL_HandleEnum::Notifier);
    converter->setHandlePair("Interrupt", hal::HAL_HandleEnum::Interrupt);
    converter->setHandlePair("AnalogOutput", hal::HAL_HandleEnum::AnalogOutput);
    converter->setHandlePair("AnalogInput", hal::HAL_HandleEnum::AnalogInput);
    converter->setHandlePair("AnalogTrigger", hal::HAL_HandleEnum::AnalogTrigger);
    converter->setHandlePair("Relay", hal::HAL_HandleEnum::Relay);
    converter->setHandlePair("PWM", hal::HAL_HandleEnum::PWM);
    converter->setHandlePair("DigitalPWM", hal::HAL_HandleEnum::DigitalPWM);
    converter->setHandlePair("Counter", hal::HAL_HandleEnum::Counter);
    converter->setHandlePair("FPGAEncoder", hal::HAL_HandleEnum::FPGAEncoder);
    converter->setHandlePair("Encoder", hal::HAL_HandleEnum::Encoder);
    converter->setHandlePair("Compressor", hal::HAL_HandleEnum::Compressor);
    converter->setHandlePair("Solenoid", hal::HAL_HandleEnum::Solenoid);
    converter->setHandlePair("AnalogGyro", hal::HAL_HandleEnum::AnalogGyro);
    converter->setHandlePair("Vendor", hal::HAL_HandleEnum::Vendor);
    converter->setHandlePair("SimulationJni", hal::HAL_HandleEnum::SimulationJni);
    converter->setHandlePair("CAN", hal::HAL_HandleEnum::CAN);
    converter->setHandlePair("SerialPort", hal::HAL_HandleEnum::SerialPort);
    converter->setHandlePair("DutyCycle", hal::HAL_HandleEnum::DutyCycle);
    converter->setHandlePair("DMA", hal::HAL_HandleEnum::DMA);
    converter->setHandlePair("AddressableLED", hal::HAL_HandleEnum::AddressableLED);
    converter->setHandlePair("CTREPCM", hal::HAL_HandleEnum::CTREPCM);
    converter->setHandlePair("CTREPDP", hal::HAL_HandleEnum::CTREPDP);
    converter->setHandlePair("REVPDH", hal::HAL_HandleEnum::REVPDH);
    converter->setHandlePair("REVPH", hal::HAL_HandleEnum::REVPH);
    return converter;
}

Mau_EnumConverter* Mau_FileHandler::getEnumConverter() {
    return enums;
}

Mau_ChannelMap* Mau_FileHandler::readChannelMap(const char* overridePath) {
    auto channelMap = std::make_unique<Mau_ChannelMap>();
    rapidjson::Document mauDoc;
    FILE* mauJson = fopen(overridePath ? overridePath : CHANNEL_MAP_JSON, "r");
    if (mauJson) {
        char readBuffer[65536];
        rapidjson::FileReadStream vmxStream(mauJson, readBuffer, sizeof(readBuffer));
        mauDoc.ParseStream(vmxStream);
	fclose(mauJson);
    } else {
        // Initialize with default memory contents
        rapidjson::MemoryStream vmxMemStream(DEFAULT_CHANNEL_MAP_JSON, strlen(DEFAULT_CHANNEL_MAP_JSON));
        mauDoc.ParseStream(vmxMemStream);
    }
    if (mauDoc.HasParseError() || !mauDoc.IsObject()) {
        throw std::runtime_error("Invalid VMX ChannelMap.json");
    }
    // Referenced groups (Counter/DigitalPWM -> DIO, Relay -> PWM, etc.) must be
    // bound only after all concrete groups exist; generator keys are sorted.
    for (int pass = 0; pass < 2; ++pass) {
        for (const auto& [label, creator] : allGenerators) {
            if (creator != &genHandleGroup) continue;  // Dedicated SDK paths handle the others.
            if (!mauDoc.HasMember(label.c_str())) continue;
            const auto& group = mauDoc[label.c_str()];
            if (!group.IsObject() || !group.HasMember("Channels")) {
                throw std::runtime_error("Invalid VMX channel group: " + label);
            }
            if (group["Channels"].IsString() != (pass == 1)) continue;
            creator(channelMap.get(), label, &mauDoc, enums);
        }
    }
    return channelMap.release();
}

Mau_FileHandler::Mau_FileHandler() {
    enums = readEnums();
}

Mau_FileHandler::~Mau_FileHandler() {
    delete enums;
}
