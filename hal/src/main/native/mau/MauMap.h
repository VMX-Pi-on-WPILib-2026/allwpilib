#ifndef NATIVE_CHANNELMAP_H
#define NATIVE_CHANNELMAP_H

#include "hal/handles/HandlesInternal.h"
#include "Translator/include/MauEnumConverter.h"
#include <VMXChannel.h>
#include <VMXResourceConfig.h>
#include <vector>
#include <string>
#include <map>
#include <memory>

struct Mau_Channel {
    int wpiIndex = -1;
    bool unsupported = true;
    bool isMXP = false;

    VMXChannelIndex vmxIndex = INVALID_VMX_CHANNEL_INDEX;
    VMXChannelCapability vmxAbility = VMXChannelCapability::NoCapabilities;

    VMXResourceHandle vmxResHandle = INVALID_VMX_RESOURCE_INDEX;
    VMXResourceConfig* vmxResConfig = nullptr;

    VMXChannelInfo getInfo();
    VMXResourceIndex getResourceIndex();
};

class Mau_ChannelGroup {
    std::unique_ptr<Mau_Channel[]> channels;
    int count;
public:
    Mau_ChannelGroup(Mau_Channel* newChannels, int count);
    virtual ~Mau_ChannelGroup() = default;
    Mau_Channel* getChannel(int index);
};

class Mau_HandledGroup : public Mau_ChannelGroup {
    hal::HAL_HandleEnum WPI_Handle;

public:
    Mau_HandledGroup(Mau_Channel* newChannels, int count, hal::HAL_HandleEnum handle);
    hal::HAL_HandleEnum getHandle();
};

class Mau_ChannelMap {
    std::map<std::string, std::shared_ptr<Mau_ChannelGroup>> groups;

    friend class Mau_FileHandler;
    void setChannelAsReference(std::string target, std::string ref);
    void setGroup(std::string target, std::shared_ptr<Mau_ChannelGroup> group);
public:
    Mau_ChannelMap();
    Mau_Channel* getChannel(std::string label, int index);
    VMXChannelInfo getChannelInfo(std::string label, int index);

    void initializeChannel(std::string label, int index, VMXChannelCapability ability, VMXResourceConfig* config);
};

#endif //NATIVE_CHANNELMAP_H
