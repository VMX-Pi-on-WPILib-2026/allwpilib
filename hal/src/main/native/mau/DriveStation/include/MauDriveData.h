#ifndef NATIVE_DRIVEDATA_H

#include "hal/DriverStation.h"
#include <wpi/mutex.h>
#include <wpi/condition_variable.h>

#define NATIVE_DRIVEDATA_H

struct Mau_SharedJoystick {
	bool initd;
	// Data from Driver Station to Robot
    HAL_JoystickAxes joyAxes;
    HAL_JoystickPOVs joyPOVs;
    HAL_JoystickButtons joyButtons;
    HAL_JoystickDescriptor joyDescriptor;
    // Data from Robot to Driver Station
    int64_t outputs;
    int32_t leftRumble;
    int32_t rightRumble;
};

class Mau_DriveData {
    static wpi::mutex memLock;
    static wpi::condition_variable memSignal;

    static HAL_AllianceStationID allianceID;
	static HAL_MatchInfo matchInfo;
    static HAL_ControlWord controlWord;
    static Mau_SharedJoystick joysticks[HAL_kMaxJoysticks];

    static volatile uint32_t newDSDataAvailableCounter;

    static volatile float matchTime;

    static void unlockAndSignal();
public:
    static void initializeDriveData();

    // Updates upon receipt of Driver Station Packets (from IO Worker Threads)

    static void updateMatchGameSpecificMessage(uint8_t msg_len, uint8_t *msg_data);
    static void updateMatchIdentifyInfo(char *event_name, uint8_t match_type, uint16_t match_number, uint8_t replay_number);

    static void updateControlWordAndAllianceID(bool enabled, bool auton, bool test, bool eStop, bool fms, bool ds, HAL_AllianceStationID id);
    static void updateDSAttached(bool attached);
    static uint32_t getNewDSDataAvailableCounter();

    static void updateJoyAxis(int joyNumber, int16_t axisCount, int8_t* axes);
    static void updateJoyPOV(int joyNumber, int povsCount, uint16_t* povs);
    static void updateJoyButtons(int joyNumber, uint8_t buttonCount, uint32_t buttons);
    static void updateJoyDescriptor(int joyNumber, HAL_JoystickDescriptor* desc);
    static void updateJoyOutputs(int32_t joyNumber, int64_t outputs, int32_t leftRumble, int32_t rightRumble);
    static void updateLock();
    static void updateUnlockAndSignal();
    static void updateControlWordAndAllianceIDUnsafe(bool enabled, bool auton, bool test, bool eStop, bool fms, bool ds, HAL_AllianceStationID id);
    static void updateJoyAxisUnsafe(int joyNumber, int16_t axisCount, int8_t* axes);
    static void updateJoyPOVUnsafe(int joyNumber, int povsCount, uint16_t* povs);
    static void updateJoyButtonsUnsafe(int joyNumber, uint8_t buttonCount, uint32_t buttons);
    static void updateMatchTimeUnsafe(float currMatchTime);
    // Methods to retrieve copies of cached DriverStation data, invoked from wpilibc Driver Station

	static void scribeMatchInfo(HAL_MatchInfo* info);
	static void scribeJoyAxes(int joyNumber, HAL_JoystickAxes* axes);
	static void scribeJoyPOVs(int joyNumber, HAL_JoystickPOVs* povs);
	static void scribeJoyButtons(int joyNumber, HAL_JoystickButtons* buttons);
	static void scribeJoyDescriptor(int joyNumber, HAL_JoystickDescriptor* desc);
	static void scribeJoyName(int joyNumber, char* name);

    static HAL_AllianceStationID readAllianceID();
    static HAL_MatchType readMatchType();
    static HAL_ControlWord readControlWord();
	static HAL_Bool readJoyIsXbox(int joyNumber);
	static int32_t readJoyType(int joyNumber);
	static int32_t readJoyAxisType(int joyNumber, int axisNumber);
	static float readMatchTime();

    static wpi::mutex* getMutex();
    static wpi::condition_variable* getDataSignal();

    // 2026 HAL support: consistent snapshots of the cached data, plus a callback
    // invoked (outside the lock) after each DS control packet or DS disconnect.
    static void scribeUdpData(HAL_JoystickAxes* axes, HAL_JoystickPOVs* povs, HAL_JoystickButtons* buttons,
                              HAL_ControlWord* cw, HAL_AllianceStationID* alliance, float* currMatchTime);
    static void scribeTcpData(HAL_MatchInfo* info, HAL_JoystickDescriptor* descs);
    static void setNewDataCallback(void (*callback)());

private:
    static void (*newDataCallback)();
    static void invokeNewDataCallback();
};

#endif //NATIVE_DRIVEDATA_H
