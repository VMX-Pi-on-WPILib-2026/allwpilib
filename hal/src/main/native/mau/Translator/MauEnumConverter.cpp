#include <VMXChannel.h>
#include "hal/handles/HandlesInternal.h"
#include "Translator/include/MauEnumConverter.h"
#include <utility>

void Mau_EnumConverter::setHandlePair(std::string label, hal::HAL_HandleEnum handle) {
    std::pair<std::string, hal::HAL_HandleEnum> newHandle;
    newHandle.first = label;
    newHandle.second = handle;
    halHandles.push_back(newHandle);
}

std::string Mau_EnumConverter::getHandleLabel(hal::HAL_HandleEnum handle) {
    for(std::pair<std::string, hal::HAL_HandleEnum> curPair : halHandles) {
        if(curPair.second == handle) {
            return curPair.first;
        }
    }
    return "Not Found!";
}

hal::HAL_HandleEnum Mau_EnumConverter::getHandleValue(std::string label) {
    for(std::pair<std::string, hal::HAL_HandleEnum> curPair : halHandles) {
        if(curPair.first == label) {
            return curPair.second;
        }
    }
    return (hal::HAL_HandleEnum)0;
}
