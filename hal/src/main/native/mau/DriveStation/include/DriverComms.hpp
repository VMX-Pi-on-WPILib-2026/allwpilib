#pragma once

#include <cstdint>
#include <mutex>

#define IS_BIT_SET(expression, bit) ((expression & (1 << bit)) != 0)

#define MAU_COMMS_SHUTDOWN_NONE		0
#define MAU_COMMS_SHUTDOWN_ESTOP 	1
#define MAU_COMMS_SHUTDOWN_REBOOT	2
#define MAU_COMMS_SHUTDOWN_RESTART  	3

#define MAU_COMMS_STATE_DISABLED	0
#define MAU_COMMS_STATE_AUTONOMOUS	1
#define MAU_COMMS_STATE_TELEOP		2
#define MAU_COMMS_STATE_TEST		3

namespace mau {
    namespace comms {
        typedef struct {
            uint8_t axis_count, pov_count, button_count;
            uint16_t pov[4];
            int8_t axis[16];
            uint32_t button_mask;
            bool has_update;
        } _TempJoyData;

        extern uint8_t sq_1;
        extern uint8_t sq_2;
        extern uint8_t control;
        extern uint8_t req;

        extern _TempJoyData joys[6];
        extern long long lastDecodeTime;
    	extern void (*shutdown_handler)(int);

        void setShutdownHandler(void (*shutdown_handler)(int));
        void start_log_capture();
        void start_ds_protocol_threads();
        void stop();
        int decodeUdpPacket(char* data, int length); // returns MAU_COMMS_SHUTDOWN_XXX code
        void encodePacket(char* data);
        void decodeTcpPacket(char* data, int length);
        void setInputVoltage(double voltage);
        void setCANStatus(float percentBusUtilization, uint32_t busOffCount, uint32_t txFifoFullCount, uint8_t rxErrorCount, uint8_t txErrorCount);
        int32_t enqueueErrorMessage(uint16_t num_occur, int32_t errorCode, uint8_t flags, const char *details, const char *location, const char *callStack);
        int32_t enqueuePrintMessage(char *msg);
	void setRobotProgramStarted(bool program_started);
	void setRobotState(uint8_t mode /*MAU_COMMS_STATE_xxx*/);
	void setRobotBrownoutProtectionActive(bool brownout_protection_active);
	void setRobotESTOPActive(bool estop_active);
	void setNotUserCode(bool not_user_code);
    }
}
