#include "Translator/include/FileHandler.h"

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

// EnumPairs JSON (this cannot be overridden)

static constexpr char DEFAULT_ENUMPAIRS_JSON[] = "\
{\
	\"HAL_HandleEnum\": {\
		\"0\":\"Undefined\",\
		\"1\":\"DIO\",\
		\"2\":\"Port\",\
		\"3\":\"Notifier\",\
		\"4\":\"Interrupt\",\
		\"5\":\"AnalogOutput\",\
		\"6\":\"AnalogInput\",\
		\"7\":\"AnalogTrigger\",\
		\"8\":\"Relay\",\
		\"9\":\"PWM\",\
		\"10\":\"DigitalPWM\",\
		\"11\":\"Counter\",\
		\"12\":\"FPGAEncoder\",\
		\"13\":\"Encoder\",\
		\"14\":\"Compressor\",\
		\"15\":\"Solenoid\",\
		\"16\":\"AnalogGyro\",\
		\"17\":\"Vendor\",\
		\"18\":\"SimulationJni\",\
		\"19\":\"CAN\"\
	},\
\
	\"VMXChannelType\": {\
		\"0\":\"INVALID\",\
		\"1\":\"FlexDIO\",\
		\"2\":\"AnalogIn\",\
		\"3\":\"HiCurrDIO\",\
		\"4\":\"CommDIO\"\
	},\
\
	\"VMXChannelCapability\": {\
		\"0\":\"NoCapabilities\",\
		\"1\":\"DigitalInput\",\
		\"2\":\"DigitalOutput\",\
		\"3\":\"PWMGeneratorOutput\",\
		\"4\":\"PWMGeneratorOutput2\",\
		\"5\":\"PWMCaptureInput\",\
		\"6\":\"PWMCaptureInput2\",\
		\"7\":\"EncoderAInput\",\
		\"8\":\"EncoderBInput\",\
		\"9\":\"AccumulatorInput\",\
		\"10\":\"AnalogTriggerInput\",\
		\"11\":\"InterruptInput\",\
		\"12\":\"UART_TX\",\
		\"13\":\"UART_RX\",\
		\"14\":\"SPI_CLK\",\
		\"15\":\"SPI_MISO\",\
		\"16\":\"SPI_MOSI\",\
		\"17\":\"SPI_CS\",\
		\"18\":\"I2C_SDA\",\
		\"19\":\"I2C_SCL\"\
	}\
}\
";

// -------- Generators -------- //

Mau_Channel* Mau_FileHandler::genGroupChannels(Mau_ChannelMap* mauMap, const char* labelVal, rapidjson::Document* doc) {
    bool allRef = (*doc)[labelVal]["Channels"].IsString();

    if (allRef) {
        std::string label(labelVal);
        mauMap->setChannelAsReference(label, (*doc)[labelVal]["Channels"].GetString());
        return nullptr;
    } else {
        int chanCount = (*doc)[labelVal]["Count"].GetInt();
        Mau_Channel* newChannels = new Mau_Channel[chanCount];

        int firstMXP = (*doc)[labelVal]["FirstMXP"].GetInt();
        std::string ref = (*doc)[labelVal]["ReferenceTo"].GetString();
        for (int count = 0; count < chanCount; count++) {
            Mau_Channel curChannel;
            curChannel.isMXP = (count >= firstMXP);

            rapidjson::Value curVal(std::to_string(count).c_str(), doc->GetAllocator());

            curChannel.wpiIndex = count;
            int refIndex = (*doc)[labelVal]["Channels"][curVal].GetInt();


            curChannel.unsupported = (refIndex == -1);

            if (!curChannel.unsupported) {
                if (ref == "VMXChannel") {
                	curChannel.vmxIndex = refIndex;
                }
            }

            newChannels[count] = curChannel;
        }

        return newChannels;
    }
}

void Mau_FileHandler::genHandleGroup(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc) {
    const char* labelVal = label.c_str();

    int handle = (*doc)[labelVal]["EnumValue"].GetInt();

    Mau_Channel* channels = genGroupChannels(mauMap, labelVal, doc);
    Mau_HandledGroup* group = new Mau_HandledGroup(channels, handle);

    mauMap->setGroup(label, group);
}

void Mau_FileHandler::genEncoderChannel(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc) {
    //Mau_Channel channel;
}

void Mau_FileHandler::genSPIChannel(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc) {
    //Mau_Channel channel;
}

void Mau_FileHandler::genI2CChannel(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc) {
    //Mau_Channel channel;
}

void Mau_FileHandler::genSerialChannel(Mau_ChannelMap* mauMap, std::string label, rapidjson::Document* doc) {
    //Mau_Channel channel;
}

// -------- Readers -------- //

Mau_EnumConverter* Mau_FileHandler::Mau_FileHandler::readEnums() {
    Mau_EnumConverter* enums = new Mau_EnumConverter();
    rapidjson::Document vmxDoc;
    // Initialize with default memory contents
    rapidjson::MemoryStream vmxMemStream(DEFAULT_ENUMPAIRS_JSON, strlen(DEFAULT_ENUMPAIRS_JSON));
    vmxDoc.ParseStream(vmxMemStream);
    assert(vmxDoc.IsObject());

    for(int handleCount = 0; handleCount < 20; handleCount++) {
        auto value = std::to_string(handleCount).c_str();
        std::string label = vmxDoc["HAL_HandleEnum"][value].GetString();
        hal::HAL_HandleEnum handle = (hal::HAL_HandleEnum) handleCount;
        enums->setHandlePair(label, handle);
    }
    return enums;
}

Mau_EnumConverter* Mau_FileHandler::getEnumConverter() {
    return enums;
}

Mau_ChannelMap* Mau_FileHandler::readChannelMap() {
	Mau_ChannelMap* channelMap = new Mau_ChannelMap();    
    rapidjson::Document mauDoc;
    FILE* mauJson = fopen(CHANNEL_MAP_JSON, "r");
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
    assert(mauDoc.IsObject());

    for (auto it = allGenerators.begin(); it != allGenerators.end(); ++it) {
        rapidjson::Value label(it->first.c_str(), mauDoc.GetAllocator());

        genFuncs creator = allGenerators.at(it->first);
        creator(channelMap, it->first, &mauDoc);
    }

    return channelMap;
}

Mau_FileHandler::Mau_FileHandler() {
    enums = readEnums();
}

Mau_FileHandler::~Mau_FileHandler() {
    delete enums;
}
