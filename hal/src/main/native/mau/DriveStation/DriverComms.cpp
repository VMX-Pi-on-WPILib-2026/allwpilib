#include "socket.hpp"
#include "DriverComms.hpp"
#include "include/MauDriveData.h"
#include "MauTime.h"
#include <wpi/priority_mutex.h>
#include "ErrorOrPrintMessage.h"
#include "buddy.h"
#include "PacketReader.h"
#include "TcpPacketBuffer.h"
#include <algorithm>
#include <array>
#include <span>
#include <cstring>
#include <unistd.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <signal.h>

// UDP Driver Station Thread constants
#define WPI_DRIVESTATION_UDP_ROBOTLISTEN_PORT   1110
#define WPI_DRIVESTATION_UDP_DSLISTEN_PORT		1150

// TCP Driver Station Thread constants
#define WPI_DRIVESTATION_TCP_PORT   			1740
#define MAX_NUM_DS_CONNECTIONS      			1
#define MAX_WPI_DRIVESTATION_TCP_READSIZE		8192
#define MAX_WPI_DRIVESTATION_UDP_READSIZE		4096

// TCP Log Server Thread constants
#define MAX_NUM_CLIENT_CONNECTIONS				5
#define WPI_LOGGING_PORT            			1741
#define NEW_DS_TCP_MESSAGE_WAIT_TIMEOUT_MS 		5

// STDOUT/ERR Capture constants
#define STD_CAPTURE_TIMEOUT_MS				50
#define MAX_STD_LINE_LEN				32768

// Buddy-system (constant-time memory allocator) for message memory allocation
#define LOG_BUFFER_SIZE_POWER  					16	// 16:  64K
#define LOG_BUFFER_SIZE 						(1 << LOG_BUFFER_SIZE_POWER) // NOTE:  Must be power of 2-sized.

// Logging Message constants
#define MAX_MESSAGE_LEN							4096	// Maximum allowable individual message length

static bool first_packet = true;
static uint8_t last_udp_ds_control = 0;
static uint8_t last_udp_ds_request = 0;
static uint8_t last_udp_robot_status = 0;
static uint8_t last_udp_robot_program_trace = 0;
static char ds_ctl_string[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
static char ds_req_string[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
static char robot_status_string[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
static char robot_program_trace_string[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint64_t tcp_packet_count = 0;


static uint64_t ntoh64(uint64_t *input) {
	uint64_t rval;
	uint8_t *data = (uint8_t *)&rval;

	data[0] = *input >> 56;
	data[1] = *input >> 48;
	data[2] = *input >> 40;
	data[3] = *input >> 32;
	data[4] = *input >> 24;
	data[5] = *input >> 16;
	data[6] = *input >> 8;
	data[7] = *input >> 0;

	return rval;
}

static uint64_t hton64(uint64_t *input) {
	return (ntoh64(input));
}

#include <thread>
namespace mau {
    namespace comms {

    	// DriveStation Protocol working buffers
    	char ds_udp_send_buffer[63] = {};			// UDP Send Buffer
    	char ds_udp_rcv_buffer[MAX_WPI_DRIVESTATION_UDP_READSIZE] = {};		// UDP Decode Buffer
    	char ds_tcp_rcv_buffer[MAX_WPI_DRIVESTATION_TCP_READSIZE] = {};		// TCP Receive Buffer

    	volatile bool version_data_requested = false;

    	// Buddy-system memory allocator, for allocation of log messages
    	BuddyPool *logmsg_mem_allocator = 0;
    	std::mutex allocLock;

    	// Multi-producer, single-consumer message queue
    	// Newer messages are placed later in the linked list
    	MessageHeader *s_p_head = 0;		// oldest message in queue
    	MessageHeader *s_p_tail = 0;		// newset message in queue
    	std::mutex queueLock;
    	std::mutex queueNewDataLock;
    	std::condition_variable queueNewDataCondition;

    	// Log Message Sequence numbers
    	int sequence_number = 0;

    	// DS UDP Protocol Working Variables
    	uint8_t sq_1 = 0;				// Sequence Number
    	uint8_t sq_2 = 0;				// Sequence Number
    	uint8_t udp_ds_control = 0;			// Control Word
	uint8_t udp_ds_request = 0;			// 

    	void (*shutdown_handler)(int) = 0;

    	_TempJoyData joys[HAL_kMaxJoysticks] = {};			// Last-received Joystick Data
    	long long lastDsUDPUpdateReceivedTimestamp = 0;

    	// Cached System Status
    	volatile double voltage = 0.0f;

    	// Cached CAN Bus Status;
    	volatile float		canStatusPercentBusUtilization = 0.0f;
    	volatile uint32_t	canStatusBusOffCount = 0;
    	volatile uint32_t	canStatusTxFifoFullCount = 0;
    	volatile uint8_t	canStatusRxErrorCount = 0;
    	volatile uint8_t	canStatusTxErrorCount = 0;

	volatile bool		robotProgramStarted = false;
	volatile uint8_t	robotMode = MAU_COMMS_STATE_DISABLED;
	volatile bool		robotBrownoutProtectionActive = false;
	volatile bool		robotESTOPActive = false;		
	volatile bool		notUserCode = false;

    	std::thread udpThread;
    	std::thread tcpThread;
    	std::thread logServerThread;
    	std::thread stdoutCaptureThread;
        std::thread stderrCaptureThread;
    	volatile bool isRunning = false;
        volatile bool isLogCaptureRunning = false;

    	// Internal Threads
    	void tcpProcess();
    	void udpProcess();
    	void logServerProcess();
        void stdCaptureProcess(int fd);

    	// Message memory allocation routines
    	void *AllocMessageMemory(size_t len);
    	void FreeMessageMemory(void *p_mem);

    	// Message Queue (linked list) routines
    	void AddTail(MessageHeader *p_new);
    	MessageHeader *RemoveHead();
    	// Integrated Memory Allocation & Queue Routines
    	void *AllocMessageMemoryWithReclamation(size_t len);

        uint32_t getTotalSystemMemory();
        uint64_t getAvailableDiskSpace();
    }
}

uint32_t mau::comms::getTotalSystemMemory() {
	long pages = sysconf(_SC_PHYS_PAGES);
	long page_size = sysconf(_SC_PAGE_SIZE);
	uint32_t total_system_memory = static_cast<uint32_t>(pages) * static_cast<uint32_t>(page_size);
	return total_system_memory;
}

uint64_t mau::comms::getAvailableDiskSpace() {
	struct statvfs stat;
	if (statvfs("/", &stat) != 0) {
		return 0;
	}
	return static_cast<uint64_t>(stat.f_bsize) * static_cast<uint64_t>(stat.f_bavail);
}

void mau::comms::setShutdownHandler(void (*shutdown_handler_func)(int)) {
	mau::comms::shutdown_handler = shutdown_handler_func;
}

void mau::comms::start_log_capture() {
    if (!isLogCaptureRunning) {

   	logmsg_mem_allocator = new BuddyPool(LOG_BUFFER_SIZE);

	isLogCaptureRunning = true;
        stdoutCaptureThread = std::thread(stdCaptureProcess, STDOUT_FILENO);
        stdoutCaptureThread.detach();

	stderrCaptureThread = std::thread(stdCaptureProcess, STDERR_FILENO);
	stderrCaptureThread.detach();
    }
}

void mau::comms::start_ds_protocol_threads() {
    if (!isRunning) {
        Toast::Net::Socket::socket_init();

        isRunning = true;
        printf("Server Running...\n");

        udpThread = std::thread(udpProcess);
        udpThread.detach();
        tcpThread = std::thread(tcpProcess);
        tcpThread.detach();
    }
}

void mau::comms::stop() {
    isLogCaptureRunning = false;
    isRunning = false;
}

//// ----- DriverStation Comms: Encode ----- ////

#define UDP_DS_PROTOCOL_VERSION				0x01

// All protocol tags are sent in ROBOT->DS direction, except where indicated below
#define DS_UDP_PROTOCOL_HID_RUMBLE					 1 // u32 HIDOutputs, u16 leftRumble, u16 rightRumble
#define DS_UDP_PROTOCOL_DISK_USAGE					 4 // u64, in bytes
#define DS_UDP_PROTOCOL_CPU_USAGE					 5 // u8 count, f32:  percent for each (Normal, Timed Structs, Time Critical, ISR)
#define DS_UDP_PROTOCOL_MEMORY_USAGE				 6 // u32:  free memory, u32:  largest free block
#define DS_UDP_PROTOCOL_MATCH_TIME_COUNTDOWN		 7 // DS->ROBOT:  f32 with a sentinel of -1 when not available
#define DS_UDP_PROTOCOL_PDP_STATUS					 8 // u8 PDP Device ID, followed by 3 concatenated status frames (24 bytes)
#define DS_UDP_PROTOCOL_PDP_ENERGY					 9 // u8 PDP Device ID, followed by the energy frame (8 bytes)
#define DS_UDP_PROTOCOL_TAG_JOYSTICK_STATE			12 // DS->ROBOT:  axes (u8 count, i8 value) per axis, buttons (u8 count, u8 bit array [lsb], and pov (u8 count, i16 value per pov)
#define DS_UDP_PROTOCOL_TAG_CAN_STATS				14 // 1Hz; u32 pctBusUtil, u32 busOffCount, u32 txFifoFullCnt, u8 rxErrCnt, u8 txErrCnt
#define DS_UDP_PROTOCOL_TAG_DATE_TIME				15 // DS->ROBOT:  // Selected to match the linux API UTC Timezone (requested by robot)
#define DS_UDP_PROTOCOL_TAG_TIME_ZONE				16 // DS->ROBOT:  // ASCII string matching the linux std (requested by robot)

#define DS_UDP_PROTOCOL_HEADER_LENGTH						6
#define DS_UDP_PROTOCOL_HEADER_INDEX_VERSION		 		2
#define DS_UDP_PROTOCOL_HEADER_INDEX_CONTROLBYTE	 		3
#define DS_UDP_PROTOCOL_HEADER_INDEX_REQUESTBYTE	 		4
#define DS_UDP_PROTOCOL_HEADER_INDEX_ALLIANCE_STATION_ID	5


#define DS_COMM_TIMEOUT_MICROSECONDS 		2000000


#define UDP_DS_STATUSBYTE_ESTOP			0x80
#define UDP_DS_STATUSBYTE_BROWNOUTPREVENT	0x10
#define UDP_DS_STATUSBYTE_PROGSTART		0x08
#define UDP_DS_STATUSBYTE_ENABLED		0x04
#define UDP_DS_STATUSBYTE_AUTONOMOUS		0x02
#define UDP_DS_STATUSBYTE_TEST			0x01

#define UDP_DS_USERPROGRAM_TRACE_USERCODE	0x20
#define UDP_DS_USERPROGRAM_TRACE_ROBORIO	0x10
#define UDP_DS_USERPROGRAM_TRACE_TEST		0x08
#define UDP_DS_USERPROGRAM_TRACE_AUTONOMOUS	0x04
#define UDP_DS_USERPROGRAM_TRACE_TELEOP		0x02
#define UDP_DS_USERPROGRAM_TRACE_DISABLED	0x01

#define UDP_DS_REQUESTBYTE_DISABLE_BIT		0x02
#define UDP_DS_REQUESTBYTE_DATETIME_BIT		0x01

void mau::comms::encodePacket(char* data) {

    ErrorOrPrintMessage m;

    // echo last-received sequence #
    data[0] = sq_1;
    data[1] = sq_2;
    
    data[2] = UDP_DS_PROTOCOL_VERSION;						// Comm Version					
    data[3] = 0;								// STATUSBYTE
    if (!first_packet && robotProgramStarted) {
	data[3] |= UDP_DS_STATUSBYTE_PROGSTART;
    }
    if (robotMode != MAU_COMMS_STATE_DISABLED) {
        data[3] |= UDP_DS_STATUSBYTE_ENABLED;
    }
    if (robotBrownoutProtectionActive) {
	data[3] |= UDP_DS_STATUSBYTE_BROWNOUTPREVENT;
    }
    if (robotESTOPActive) {
	data[3] |= UDP_DS_STATUSBYTE_ESTOP;
    }
    if (robotMode == MAU_COMMS_STATE_AUTONOMOUS) {
	data[3] |= UDP_DS_STATUSBYTE_AUTONOMOUS;
    }
    if (robotMode == MAU_COMMS_STATE_TEST) {
	data[3] |= UDP_DS_STATUSBYTE_TEST;
    }

    data[4] = UDP_DS_USERPROGRAM_TRACE_ROBORIO;					// USERPROGRAM_TRACE
    if (!first_packet && !notUserCode) {
    	data[4] |= UDP_DS_USERPROGRAM_TRACE_USERCODE;	
    }			
    if (robotMode == MAU_COMMS_STATE_DISABLED) {
	data[4] |= UDP_DS_USERPROGRAM_TRACE_DISABLED;
    }
    if (robotMode == MAU_COMMS_STATE_AUTONOMOUS) {
	data[4] |= UDP_DS_USERPROGRAM_TRACE_AUTONOMOUS;
    }
    if (robotMode == MAU_COMMS_STATE_TELEOP) {
	data[4] |= UDP_DS_USERPROGRAM_TRACE_TELEOP;
    }
    if (robotMode == MAU_COMMS_STATE_TEST) {
	data[4] |= UDP_DS_USERPROGRAM_TRACE_TEST;
    }

    double voltage_intpart =
    		static_cast<double>(static_cast<int>(voltage));
    data[5] = (uint8_t) voltage_intpart;
    double voltage_decpart = voltage - voltage_intpart;
    data[6] = (uint8_t) (voltage_decpart * 100);
    if (first_packet) {
	data[7] |= UDP_DS_REQUESTBYTE_DISABLE_BIT;
        data[7] |= UDP_DS_REQUESTBYTE_DATETIME_BIT;
    } else {
        data[7] = 0;								// REQUESTBYTE: Used to make requests of the DS
    }										// TODO:  Add Date/Time request here, at startup

    // Tagged Data goes here....
    data[8] = 9;
    data[9] = DS_UDP_PROTOCOL_MEMORY_USAGE;
    uint32_t free_memory = getTotalSystemMemory();
    *(uint32_t *)&data[10] = htonl(free_memory); // free memory total
    *(uint32_t *)&data[14] = htonl(free_memory); // largest free block

    data[18] = 9;
    data[19] = DS_UDP_PROTOCOL_DISK_USAGE;
    uint64_t free_disk = getAvailableDiskSpace();
    *(uint64_t *)&data[20] = hton64(&free_disk);

    data[28] = 15;
    data[29] = DS_UDP_PROTOCOL_TAG_CAN_STATS; // 1 Hz.  TODO:  only send this periodically....
    *(uint32_t *)&data[30] = m.FloatToNetworkOrderedU32(canStatusPercentBusUtilization);
    *(uint32_t *)&data[34] = htonl(canStatusBusOffCount);
    *(uint32_t *)&data[38] = htonl(canStatusTxFifoFullCount);
    data[42] = canStatusRxErrorCount;
    data[43] = canStatusTxErrorCount;

    data[44] = 18; // 1 for tag + 1 for uint8 cpu % and 16 (4 each for float for normal/timedstructs/timecritical/isr)
    data[45] = DS_UDP_PROTOCOL_CPU_USAGE;
    data[46] = 4; // Count of the following Floats
    *(uint32_t *)&data[47] = m.FloatToNetworkOrderedU32(0.20F);	// Normal
    *(uint32_t *)&data[51] = m.FloatToNetworkOrderedU32(0.02F);	// Timed Structs
    *(uint32_t *)&data[55] = m.FloatToNetworkOrderedU32(0.03F); // TimeCritical
    *(uint32_t *)&data[59] = m.FloatToNetworkOrderedU32(0.01F);  // ISR

}

void mau::comms::setInputVoltage(double new_voltage) {
	voltage = new_voltage;
}

void mau::comms::setCANStatus(float percentBusUtilization, uint32_t busOffCount, uint32_t txFifoFullCount, uint8_t rxErrorCount, uint8_t txErrorCount)
{
	mau::comms::canStatusPercentBusUtilization = percentBusUtilization;
	mau::comms::canStatusBusOffCount = busOffCount;
	mau::comms::canStatusTxFifoFullCount = txFifoFullCount;
	mau::comms::canStatusRxErrorCount = rxErrorCount;
	mau::comms::canStatusTxErrorCount = txErrorCount;
}

void mau::comms::setRobotState(uint8_t mode /*MAU_COMMS_STATE_xxx*/)
{
	mau::comms::robotMode = mode;
}

void mau::comms::setRobotBrownoutProtectionActive(bool brownout_protection_active)
{
	mau::comms::robotBrownoutProtectionActive = brownout_protection_active;
}

void mau::comms::setNotUserCode(bool not_user_code)
{
	mau::comms::notUserCode = not_user_code;
}

void mau::comms::setRobotESTOPActive(bool estop_active)
{
	mau::comms::robotESTOPActive = estop_active;
}

void mau::comms::setRobotProgramStarted(bool program_started)
{
	mau::comms::robotProgramStarted = program_started;
}

//// ----- DriverStation Comms: Decode ----- ////

int32_t mau::comms::enqueueErrorMessage(uint16_t num_occur, int32_t errorCode, uint8_t flags, const char *details, const char *location, const char *callStack) {
	ErrorOrPrintMessage m;
	// Calculate required length
	int len = m.FormatErrorMessage(NULL, MAX_MESSAGE_LEN, sequence_number, num_occur, errorCode, flags, details, location, callStack);
	if (len < 0) return -1;
	// Allocate storage for message
	unsigned char *p_mem = static_cast<unsigned char *>(AllocMessageMemoryWithReclamation(len));
	if (p_mem) {
		len = m.FormatErrorMessage(p_mem, len, sequence_number, num_occur, errorCode, flags, details, location, callStack);
		if (len > 0) {
			sequence_number++;
			AddTail(static_cast<MessageHeader *>(static_cast<void *>(p_mem)));
			return 0;
		} else {
			FreeMessageMemory(p_mem);
		}
	}
	return -1;
}

int32_t mau::comms::enqueuePrintMessage(char *msg) {
	ErrorOrPrintMessage m;
	// Calculate required length
	int len = m.FormatPrintMessage(NULL, MAX_MESSAGE_LEN, sequence_number, msg);
	if (len < 0) return - 1;

	// Allocate storage for message
	unsigned char *p_mem = static_cast<unsigned char *>(AllocMessageMemoryWithReclamation(len));
	if (p_mem) {
		len = m.FormatPrintMessage(p_mem, len, sequence_number, msg);
		if (len > 0) {
			sequence_number++;
			AddTail(static_cast<MessageHeader *>(static_cast<void *>(p_mem)));
                        return 0;
		} else {
			FreeMessageMemory(p_mem);
		}
	}
	return -1;
}

// All protocol tags are sent in ROBOT->DS direction, except where indicated below
#define DS_TCP_PROTOCOL_TAG_ERROR_WARN_MSG			0	// Same format as currently used, <markup>, etc.
#define DS_TCP_PROTOCOL_USAGE_REPORT				1	// Same as currently used
#define DS_TCP_PROTOCOL_TAG_JOYSTICK_DESCRIPTIONS	2	// DS->ROBOT:
#define DS_TCP_PROTOCOL_TAG_MATCH_INFO				3	// DS->ROBOT: u8 eventNameLen, chars[x] ASCII Encoded name, u8 Match tupe ('P','Q','E' or '00'), u8 matchNumber
#define DS_TCP_PROTOCOL_TAG_DISABLE_COUNT			4	// u16 System Comm time-out count, u16 Input Power fault count
#define DS_TCP_PROTOCOL_TAG_USER_RAIL_FAULT_COUNT	5	// 6V, 5V, 3.3V rail fault count (u16s)
#define DS_TCP_PROTOCOL_TAG_PDPLOGRECORD			6	// NO LONGER USED
#define DS_TCP_PROTOCOL_TAG_NEWER_MATCH_INFO		7	// DS->ROBOT; newer match info (u8 event len + chars[var]), u8 MatchType (enum None/Practice/Qualification/Elimination/Test), u16 matchNum, u8 replayNUmber
#define DS_TCP_PROTOCOL_TAG_ROBOT_IPADDR_FOR_DB		8	// u32
#define DS_TCP_PROTOCOL_TAG_DB_WINDOW_MODE			9	// u8
#define DS_TCP_PROTOCOL_TAG_VERSION_DATA		   10	// one tag for each version (u32 id if model, otherwise 0), u8 elementNameLen, char[var], u8 versionStringLen, char[var]
#define DS_TCP_PROTOCOL_TAG_ERROR_DATA			   11	// See ErrorOrPrintMessage.h
#define DS_TCP_PROTOCOL_TAG_CONSOLE_PRINT_DATA	   12	// See ErrorOrPrintMessage.h
#define DS_TCP_PROTOCOL_TAG_USER_RAIL_FAULT_DATA   13	// u16 5VBrownoutCOunt, u8 6VRailStatusEnum, u8 5VRailStatusEnum, u8 3.3VRailStatusEnum
#define DS_TCP_PROTOCOL_TAG_GAME_SPECIFIC_MSG	   14	// DS->ROBOT; (the message is the entire size of the tag)

void mau::comms::decodeTcpPacket(char* data, int length) {
    if (!data || length <= 0) return;
    PacketReader packets{{reinterpret_cast<const uint8_t*>(data), static_cast<size_t>(length)}};
    while (packets.remaining()) {
        uint16_t size;
        std::span<const uint8_t> frame;
        if (!packets.read16(size) || !size || !packets.take(size, frame)) return;
        PacketReader fields{frame};
        uint8_t tag;
        fields.read(tag);
        switch (tag) {
            case DS_TCP_PROTOCOL_TAG_JOYSTICK_DESCRIPTIONS:
                while (fields.remaining()) {
                    uint8_t joyid, xbox, type, nameLength, axisCount, buttonCount, povCount;
                    std::span<const uint8_t> name, axes;
                    if (!fields.read(joyid) || !fields.read(xbox) || !fields.read(type) ||
                        !fields.read(nameLength) || !fields.take(nameLength, name) ||
                        !fields.read(axisCount) || !fields.take(axisCount, axes) ||
                        !fields.read(buttonCount) || !fields.read(povCount)) return;
                    if (joyid >= HAL_kMaxJoysticks || axisCount > HAL_kMaxJoystickAxes ||
                        povCount > HAL_kMaxJoystickPOVs || buttonCount > 32) continue;
                    HAL_JoystickDescriptor desc{};
                    desc.isXbox = xbox == 1;
                    desc.type = type;
                    std::memcpy(desc.name, name.data(), name.size());
                    desc.axisCount = axisCount;
                    std::copy(axes.begin(), axes.end(), desc.axisTypes);
                    desc.buttonCount = buttonCount;
                    desc.povCount = povCount;
                    Mau_DriveData::updateJoyDescriptor(joyid, &desc);
                }
                break;
            case DS_TCP_PROTOCOL_TAG_NEWER_MATCH_INFO: {
                uint8_t nameLength, matchType, replayNumber;
                uint16_t matchNumber;
                std::span<const uint8_t> name;
                if (!fields.read(nameLength) || !fields.take(nameLength, name) ||
                    !fields.read(matchType) || !fields.read16(matchNumber) ||
                    !fields.read(replayNumber)) return;
                char eventName[sizeof(HAL_MatchInfo::eventName)]{};
                std::memcpy(eventName, name.data(), std::min(name.size(), sizeof(eventName) - 1));
                Mau_DriveData::updateMatchIdentifyInfo(eventName, matchType, matchNumber, replayNumber);
                break;
            }
            case DS_TCP_PROTOCOL_TAG_GAME_SPECIFIC_MSG: {
                std::span<const uint8_t> message;
                fields.take(fields.remaining(), message);
                Mau_DriveData::updateMatchGameSpecificMessage(message.size(), message.data());
                break;
            }
            default:
                break;
        }
    }
}

// Validate and stage the entire datagram before publishing any control data.
int mau::comms::decodeUdpPacket(char* data, int length) {
    if (!data || length < DS_UDP_PROTOCOL_HEADER_LENGTH) return MAU_COMMS_SHUTDOWN_NONE;
    const auto* bytes = reinterpret_cast<const uint8_t*>(data);
    if (bytes[DS_UDP_PROTOCOL_HEADER_INDEX_VERSION] != 1) return MAU_COMMS_SHUTDOWN_NONE;
    PacketReader tags{{bytes + DS_UDP_PROTOCOL_HEADER_LENGTH,
                       static_cast<size_t>(length - DS_UDP_PROTOCOL_HEADER_LENGTH)}};
    std::array<_TempJoyData, HAL_kMaxJoysticks> parsed{};
    int joyCount = 0;
    bool hasMatchTime = false;
    float matchTime = 0;
    while (tags.remaining()) {
        uint8_t size, tag;
        std::span<const uint8_t> tagBytes;
        if (!tags.read(size) || !size || !tags.take(size, tagBytes)) return MAU_COMMS_SHUTDOWN_NONE;
        PacketReader fields{tagBytes};
        fields.read(tag);
        if (tag == DS_UDP_PROTOCOL_TAG_JOYSTICK_STATE) {
            if (joyCount >= HAL_kMaxJoysticks) return MAU_COMMS_SHUTDOWN_NONE;
            auto& joy = parsed[joyCount++];
            std::span<const uint8_t> axes, buttons;
            if (!fields.read(joy.axis_count) || joy.axis_count > HAL_kMaxJoystickAxes ||
                !fields.take(joy.axis_count, axes) || !fields.read(joy.button_count) ||
                joy.button_count > 32 || !fields.take((joy.button_count + 7) / 8, buttons) ||
                !fields.read(joy.pov_count) || joy.pov_count > HAL_kMaxJoystickPOVs) return MAU_COMMS_SHUTDOWN_NONE;
            for (size_t i = 0; i < axes.size(); ++i) joy.axis[i] = static_cast<int8_t>(axes[i]);
            for (uint8_t byte : buttons) joy.button_mask = (joy.button_mask << 8) | byte;
            for (int i = 0; i < joy.pov_count; ++i) {
                if (!fields.read16(joy.pov[i])) return MAU_COMMS_SHUTDOWN_NONE;
            }
        } else if (tag == DS_UDP_PROTOCOL_MATCH_TIME_COUNTDOWN) {
            std::span<const uint8_t> value;
            if (!fields.take(4, value)) return MAU_COMMS_SHUTDOWN_NONE;
            uint32_t bits = (static_cast<uint32_t>(value[0]) << 24) |
                            (static_cast<uint32_t>(value[1]) << 16) |
                            (static_cast<uint32_t>(value[2]) << 8) | value[3];
            std::memcpy(&matchTime, &bits, sizeof(matchTime));
            hasMatchTime = true;
        }
    }

    sq_1 = bytes[0];
    sq_2 = bytes[1];
    udp_ds_control = bytes[DS_UDP_PROTOCOL_HEADER_INDEX_CONTROLBYTE];
    udp_ds_request = bytes[DS_UDP_PROTOCOL_HEADER_INDEX_REQUESTBYTE];
    if (IS_BIT_SET(udp_ds_request, 0)) version_data_requested = true;
    if (IS_BIT_SET(udp_ds_request, 2) || IS_BIT_SET(udp_ds_request, 3)) {
        setRobotProgramStarted(false);
        return IS_BIT_SET(udp_ds_request, 3) ? MAU_COMMS_SHUTDOWN_REBOOT : MAU_COMMS_SHUTDOWN_RESTART;
    }
    if (IS_BIT_SET(udp_ds_control, 7)) {
        setRobotProgramStarted(false);
        setRobotESTOPActive(true);
        return MAU_COMMS_SHUTDOWN_ESTOP;
    }

    Mau_DriveData::updateLock();
    for (int i = 0; i < HAL_kMaxJoysticks; ++i) {
        joys[i] = parsed[i];
        auto& joy = joys[i];
        Mau_DriveData::updateJoyAxisUnsafe(i, joy.axis_count, joy.axis);
        Mau_DriveData::updateJoyPOVUnsafe(i, joy.pov_count, joy.pov);
        Mau_DriveData::updateJoyButtonsUnsafe(i, joy.button_count, joy.button_mask);
    }
    if (hasMatchTime) Mau_DriveData::updateMatchTimeUnsafe(matchTime);
    Mau_DriveData::updateControlWordAndAllianceIDUnsafe(
        IS_BIT_SET(udp_ds_control, 2), IS_BIT_SET(udp_ds_control, 1),
        IS_BIT_SET(udp_ds_control, 0), false, IS_BIT_SET(udp_ds_control, 3), true,
        static_cast<HAL_AllianceStationID>(bytes[DS_UDP_PROTOCOL_HEADER_INDEX_ALLIANCE_STATION_ID]));
    Mau_DriveData::updateUnlockAndSignal();
    lastDsUDPUpdateReceivedTimestamp = mau::vmxGetTime();
    return MAU_COMMS_SHUTDOWN_NONE;
}

namespace mau {
    namespace comms {

        static std::array<TcpPacketBuffer, MAX_NUM_DS_CONNECTIONS> tcpInput;

        // Called by the single TCP worker; each accepted connection starts empty.
        void driverStationTCPDataReceivedCallback(int client_id, Toast::Net::Socket::ClientSocket sock) {
            if (client_id < 0 || client_id >= MAX_NUM_DS_CONNECTIONS) return;
            int ret = sock.read(ds_tcp_rcv_buffer, MAX_WPI_DRIVESTATION_TCP_READSIZE);
            if (ret > 0) {
                tcpInput[client_id].append(
                    {reinterpret_cast<const uint8_t*>(ds_tcp_rcv_buffer), static_cast<size_t>(ret)},
                    [](std::span<const uint8_t> frame) {
                        decodeTcpPacket(reinterpret_cast<char*>(const_cast<uint8_t*>(frame.data())), frame.size());
                    });
            } else if (ret == 0) {
                tcpInput[client_id].reset();
            }
        }

	void tcpProcess() {

            // The driver station server socket accepts a single TCP client connections
            // This is configured as non-blocking.
	    Toast::Net::Socket::ServerSocket dsServerSocket(WPI_DRIVESTATION_TCP_PORT);
            int nonblock_status = Toast::Net::Socket::socket_nonblock(dsServerSocket.get_socket());
	    if (nonblock_status != 0) {
		printf("Error %d setting DS Server Socket nonblocking!\n", nonblock_status);
	    }
            Toast::Net::Socket::SelectiveServerSocket ds_sock(dsServerSocket.get_socket(), MAX_NUM_DS_CONNECTIONS);
            int ss_prepare_status = ds_sock.prepare();
	    if (ss_prepare_status != 0) {
		printf("Error %d preparing DS Selective Server Socket!\n", ss_prepare_status);
	    }
            ds_sock.on_data(mau::comms::driverStationTCPDataReceivedCallback);
            ds_sock.on_connect([](int client_id, Toast::Net::Socket::ClientSocket) {
                tcpInput[client_id].reset();
            });
	    if (dsServerSocket.SetReuseAddress() != 0) {
	         printf("ERROR setting ServerSocket SO_REUSEADDR option.\n");
    	    }
            int ss_open_status = dsServerSocket.open();
	    if (ss_open_status != 0) {
		perror("Bind Failed.  Error");
		printf("Error %d opening DS Server Socket!\n", ss_open_status);
	    }
            // The secondary logging server socket accepts multiple TCP client connections
            // This is configured as non-blocking.
            Toast::Net::Socket::ServerSocket logServerSocket(WPI_LOGGING_PORT);
            Toast::Net::Socket::socket_nonblock(logServerSocket.get_socket());
            Toast::Net::Socket::SelectiveServerSocket log_sock(logServerSocket.get_socket(), MAX_NUM_CLIENT_CONNECTIONS);
            log_sock.prepare();
            logServerSocket.open();

	    int num_ds_connected_clients = 0;
	    int num_log_connected_clients = 0;
	    while (isRunning) {

    			try {
					unsigned char version_data_buffer[1024];

					// Check for new DS client (non-blocking)
					// New data from connected DS client will invoke driverStationTCPDataReceivedCallback().
					ds_sock.accept();
					// Prune dead DS client, if they have become disconnected.
					ds_sock.prune_disconnected_clients();

					int curr_num_ds_connected_clients = ds_sock.get_num_connected_clients();

					// Check for new non-DS logging clients (non-blocking)
					log_sock.accept();
					// Prune dead non-DS logging clients
					log_sock.prune_disconnected_clients();

					int curr_num_log_connected_clients = log_sock.get_num_connected_clients();

					if (curr_num_ds_connected_clients != num_ds_connected_clients) {	
#if 0
						printf("Changed # of ds connected clients.  Was:  %d, Now:  %d\n", num_ds_connected_clients, curr_num_ds_connected_clients);
#endif
						num_ds_connected_clients = curr_num_ds_connected_clients;
					}
					if (curr_num_log_connected_clients != num_log_connected_clients) {	
#if 0
						printf("Changed # of log connected clients.  Was:  %d, Now:  %d\n", num_log_connected_clients, curr_num_log_connected_clients);
#endif
						num_log_connected_clients = curr_num_log_connected_clients;
					}

					if (version_data_requested) {
						version_data_requested = false;

						ErrorOrPrintMessage m;
						uint32_t model_id = 0;
						const char *element_names[4];
						element_names[0] = "roboRIO Image";
						//element_names[1] = "FRC_Lib_Version";
						//element_names[2] = 0;
						const char *version_strings[4];
						version_strings[0] = "0.0.000";
						//version_strings[0] = "FRC_roboRIO_2019_v13";
						//version_strings[1] = "Scott's Test Version";
						//version_strings[2] = 0;
						if (m.FormatVersionDataMessage(version_data_buffer, sizeof(version_data_buffer),
								model_id,
								DS_TCP_PROTOCOL_TAG_VERSION_DATA,
								1,
								element_names,
								version_strings) != -1) {
							RobotVersionDataHeader *header = (RobotVersionDataHeader *)version_data_buffer;
							ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(header->len))), ntohs(header->len) + sizeof(uint16_t));
						}

						element_names[0] = "FRC_Lib_Version";
						version_strings[0] = "C++ 2019.4.1";
						if (m.FormatVersionDataMessage(version_data_buffer, sizeof(version_data_buffer),
								model_id,
								DS_TCP_PROTOCOL_TAG_VERSION_DATA,
								1,
								element_names,
								version_strings) != -1) {
							RobotVersionDataHeader *header = (RobotVersionDataHeader *)version_data_buffer;
							ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(header->len))), ntohs(header->len) + sizeof(uint16_t));
						}


						element_names[0] = "VMXPiFirmware";
						version_strings[0] = "3.1.222";
						if (m.FormatVersionDataMessage(version_data_buffer, sizeof(version_data_buffer),
								model_id,
								DS_TCP_PROTOCOL_TAG_VERSION_DATA,
								1,
								element_names,
								version_strings) != -1) {
							RobotVersionDataHeader *header = (RobotVersionDataHeader *)version_data_buffer;
							ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(header->len))), ntohs(header->len) + sizeof(uint16_t));
						}

						// Send "end of version info" indicator.
						element_names[0] = 0;
						version_strings[0] = 0;

						if (m.FormatVersionDataMessage(version_data_buffer, sizeof(version_data_buffer),
								model_id,
								DS_TCP_PROTOCOL_TAG_VERSION_DATA,
								1,
								element_names,
								version_strings) != -1) {
							RobotVersionDataHeader *header = (RobotVersionDataHeader *)version_data_buffer;
							ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(header->len))), ntohs(header->len) + sizeof(uint16_t));
						}

					}

					// Send pending logging messages to DS client and all non-DS logging clients (if any are connected).
					// Always send the oldest messages first.
					// Note that all client sockets are also non-blocking.
					if (ds_sock.get_num_connected_clients() > 0) {
					    MessageHeader *p_head = RemoveHead();
					    while (p_head) {
						ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(p_head->len))), ntohs(p_head->len) + sizeof(uint16_t));
						log_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(p_head->len))), ntohs(p_head->len) + sizeof(uint16_t));
						FreeMessageMemory(p_head);
						p_head = RemoveHead();
					    }
					}

					tcp_packet_count++;

					// Block, waiting either for new messages, or a brief timeout
					std::unique_lock<std::mutex> lck(queueNewDataLock);
					queueNewDataCondition.wait_for(lck, std::chrono::milliseconds(NEW_DS_TCP_MESSAGE_WAIT_TIMEOUT_MS));
        		} catch(const std::exception& ex) {
        			printf("mau::comms::tcpProcess - Caught exception:  %s\n", ex.what());
        		}
			}

			/* Before terminating, send any remaining log messages */
			MessageHeader *p_head = RemoveHead();
			while (p_head) {
				ds_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(p_head->len))), ntohs(p_head->len) + sizeof(uint16_t));
				log_sock.send_to_all_connected_clients(static_cast<char *>(static_cast<void *>(&(p_head->len))), ntohs(p_head->len) + sizeof(uint16_t));
				FreeMessageMemory(p_head);
				p_head = RemoveHead();
			}

			ds_sock.close_connected_clients();
			ds_sock.close();

		}

		void udpProcess() {

			/* Set this thread as very high (but not highest) priority using SCHED_FIFO scheduling. */
			struct sched_param param;
			param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
			sched_setscheduler(0, SCHED_FIFO, &param);

			Toast::Net::Socket::DatagramSocket sock(WPI_DRIVESTATION_UDP_ROBOTLISTEN_PORT);
			sock.bind();

			Toast::Net::Socket::SocketAddress addr;
			while (true) {
				if (!isRunning)
					break;

				try {
					memset(ds_udp_rcv_buffer, 0, MAX_WPI_DRIVESTATION_UDP_READSIZE);
					int len = sock.read_with_timeout(ds_udp_rcv_buffer, MAX_WPI_DRIVESTATION_UDP_READSIZE, &addr, 500);
					if (len > 0) {
						uint8_t shutdown_code = mau::comms::decodeUdpPacket(ds_udp_rcv_buffer, len);
						mau::comms::encodePacket(ds_udp_send_buffer);
						addr.set_port(WPI_DRIVESTATION_UDP_DSLISTEN_PORT);
						sock.send(ds_udp_send_buffer, 63, &addr);

#if 0
						bool state_update = false;
#endif
						if (first_packet || (mau::comms::udp_ds_control != last_udp_ds_control)) {
#if 0
							state_update = true;
#endif
							ds_ctl_string[0] = IS_BIT_SET(mau::comms::udp_ds_control,7) ? '!' : '-';		  // E-Stop
							ds_ctl_string[1] = '.';
							ds_ctl_string[2] = '.';
							ds_ctl_string[3] = '.';
							ds_ctl_string[4] = IS_BIT_SET(mau::comms::udp_ds_control,3) ? 'F' : '-';		  // FMS Present
							ds_ctl_string[5] = IS_BIT_SET(mau::comms::udp_ds_control,2) ? 'E' : '-';		  // Enabled
							ds_ctl_string[6] = IS_BIT_SET(mau::comms::udp_ds_control,1) ? 'A' : '-';           // Auto
							ds_ctl_string[7] = IS_BIT_SET(mau::comms::udp_ds_control,0) ? 'X' : '-';		  // Test
							ds_ctl_string[8] = 0;							
						}
						if (first_packet || (mau::comms::udp_ds_request != last_udp_ds_request)) {
#if 0
							state_update = true;
#endif
							ds_req_string[0] = '.';
							ds_req_string[1] = '.';
							ds_req_string[2] = '.';
							ds_req_string[3] = IS_BIT_SET(mau::comms::udp_ds_request,4) ? 'P' : '-';            // Prog Start
							ds_req_string[4] = IS_BIT_SET(mau::comms::udp_ds_request,3) ? 'H' : '-';            // Hard Restart
							ds_req_string[5] = IS_BIT_SET(mau::comms::udp_ds_request,2) ? 'S' : '-';		   // Soft Restart
							ds_req_string[6] = IS_BIT_SET(mau::comms::udp_ds_request,1) ? 'U' : '-';		   // Usage Request
							ds_req_string[7] = IS_BIT_SET(mau::comms::udp_ds_request,0) ? 'V' : '-';		   // Version Request
							ds_req_string[8] = 0;							
						}

						char robot_status_code = ds_udp_send_buffer[3];
						char robot_program_trace = ds_udp_send_buffer[4];
						
						if (first_packet || (robot_status_code != last_udp_robot_status)) {
#if 0
							state_update = true;
#endif
							robot_status_string[0] = IS_BIT_SET(robot_status_code,7) ? '!' : '-';       // E-Stop
							robot_status_string[1] = '.';
							robot_status_string[2] = '.';
							robot_status_string[3] = IS_BIT_SET(robot_status_code,4) ? 'B' : '-';	    // Brownout-Prevention
							robot_status_string[4] = IS_BIT_SET(robot_status_code,3) ? 'P' : '-';	    // ProgStart
							robot_status_string[5] = IS_BIT_SET(robot_status_code,2) ? 'E' : '-';       // Enabled
							robot_status_string[6] = IS_BIT_SET(robot_status_code,1) ? 'A' : '-';       // Auto
							robot_status_string[7] = IS_BIT_SET(robot_status_code,0) ? 'X' : '-';       // Test
							robot_status_string[8] = 0;							
						}
						if (first_packet || (robot_program_trace != last_udp_robot_program_trace)) {
#if 0
							state_update = true;
#endif
							robot_program_trace_string[0] = '.';
							robot_program_trace_string[1] = '.';
							robot_program_trace_string[2] = IS_BIT_SET(robot_program_trace,5) ? 'U' : '-'; // UserCode
							robot_program_trace_string[3] = IS_BIT_SET(robot_program_trace,4) ? 'R' : '-'; // roboRIO
							robot_program_trace_string[4] = IS_BIT_SET(robot_program_trace,3) ? 'X' : '-'; // Test
							robot_program_trace_string[5] = IS_BIT_SET(robot_program_trace,2) ? 'A' : '-'; // Auto
							robot_program_trace_string[6] = IS_BIT_SET(robot_program_trace,1) ? 'T' : '-'; // Tele
							robot_program_trace_string[7] = IS_BIT_SET(robot_program_trace,0) ? 'D' : '-'; // Disabled
							robot_program_trace_string[8] = 0;							
						}

#if 0
						if (state_update) {
							printf("DS CTL: %s REQ: %s - ROBOT STATUS:  %s PROGTRACE:  %s\n", ds_ctl_string, ds_req_string, robot_status_string, robot_program_trace_string);
						}
#endif

						last_udp_ds_control = mau::comms::udp_ds_control;
						last_udp_ds_request = mau::comms::udp_ds_request;
						last_udp_robot_status = robot_status_code;
						last_udp_robot_program_trace = robot_program_trace;
						first_packet = false;

						if (shutdown_code != MAU_COMMS_SHUTDOWN_NONE) {


						    // If a shutdown was requested, send a few addtional responses to help
						    // ensure the driver station is aware the request was received
				        	    int timeout = 2;
						    std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
						    sock.send(ds_udp_send_buffer, 63, &addr);

						    std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
						    sock.send(ds_udp_send_buffer, 63, &addr);						    					

						    // Initiate program shutdown
					            if (shutdown_handler) {
                            shutdown_handler(shutdown_code);
					            } else {
						            stop();
						    }							
						}
					} else {
						// No data read.  Check to see if the ds is still attached.
					        if (mau::vmxGetTime() - lastDsUDPUpdateReceivedTimestamp > DS_COMM_TIMEOUT_MICROSECONDS) {					            
					            Mau_DriveData::updateDSAttached(false);
					        }
					}
				} catch(const std::exception& ex){
					printf("mau::comms::udpProcess - Caught exception:  %s\n", ex.what());
        			}
			}
			sock.close();
		}

    	void *AllocMessageMemory(size_t len) {
		if (!logmsg_mem_allocator) return 0;
    		void *p_mem;
    		allocLock.lock();
    		p_mem = logmsg_mem_allocator->Acquire(len);
    		allocLock.unlock();
    		return p_mem;

    	}

    	void FreeMessageMemory(void *p_mem) {
		if (!p_mem) return;
    		allocLock.lock();
    		logmsg_mem_allocator->Release(p_mem);
    		allocLock.unlock();
    	}

    	void AddTail(MessageHeader *p_new) {

    		if (!p_new) return;

    		p_new->next = 0;

    	    queueLock.lock();

    		if (s_p_tail) {
    			s_p_tail->next = p_new;
    		}

    		s_p_tail = p_new;

    		if (!s_p_head) {
    			// List is empty
    			s_p_head = p_new;
    		}

    		queueLock.unlock();

    	    queueNewDataCondition.notify_all();
    	}

    	MessageHeader *RemoveHead() {

    		queueLock.lock();

    		MessageHeader *p_head = s_p_head;

    		if (p_head) {

    			s_p_head = p_head->next;

    			if (!s_p_head) {
    				// List is now empty
    				s_p_tail = 0;
    			}

    			p_head->next = 0;
    		}

    		queueLock.unlock();

    		return p_head;
    	}

    	// Allocates memory for a Message.  If insufficient memory remains,
    	// the oldest allocated messages are deleted, until there is sufficient
    	// memory available for this message.
    	void *AllocMessageMemoryWithReclamation(size_t len) {
		if (!logmsg_mem_allocator) return 0;
    		unsigned char *p_mem = static_cast<unsigned char *>(AllocMessageMemory(len));
    		if (!p_mem) {
    			// No memory available.  Keep removing oldest messages until sufficient memory is available
    			MessageHeader *p_head = RemoveHead();
    			while (!p_mem && p_head) {
    				FreeMessageMemory(p_head);
    				p_mem = static_cast<unsigned char *>(AllocMessageMemory(len));
    				if (!p_mem) {
    					p_head = RemoveHead();
    				}
    			}
    			// FATAL ERROR:  The Message Memory Pool is not big enough to handle this message.  (TODO:  Add logging here????).
    			if (!p_mem) return 0;
    		}
    		return p_mem;
    	}

        void stdCaptureProcess(int capture_fd) {

        	fd_set set;
        	struct timeval timeout;

        	int out_pipe[2];
        	int std_prev_fd;

        	// Save current std file descriptor
        	std_prev_fd = dup(capture_fd);
        	if (std_prev_fd == -1) return;

        	if (pipe(out_pipe) != 0) return;

        	// Redirect stdout to the pipe
        	if (dup2(out_pipe[1], capture_fd) == -1) return;
        	close(out_pipe[1]);

        	FILE *pipe_file = fdopen(out_pipe[0], "r");

        	char *line = (char *)malloc(MAX_STD_LINE_LEN+1);
		memset(line,0,MAX_STD_LINE_LEN + 1);

		bool quit_requested = false;

        	while (isLogCaptureRunning || !quit_requested) {

			quit_requested = !isLogCaptureRunning;

        		try {
					FD_ZERO(&set);
					FD_SET(out_pipe[0], &set);
					timeout.tv_sec = 0;
					timeout.tv_usec = STD_CAPTURE_TIMEOUT_MS * 1000;
 
					int ret = select((out_pipe[0] + 1), &set, NULL, NULL, &timeout);
					if (ret == 0) {
						// Timeout
					} else if (ret > 0) {
						if (FD_ISSET(out_pipe[0], &set)) {
							// Input Available; occurs at end of every line, since STDOUT is configured
							// for line buffering in Mau HAL Initialization.
							ssize_t num_read;
							size_t len = MAX_STD_LINE_LEN;
							while ((num_read = getline(&line, &len, pipe_file)) != -1) {
								// Output to console FD
								write(std_prev_fd, line, num_read);
								// Enqueue Message for transmission to WPI Logging Client
								if (num_read > 0) {
									line[num_read - 1] = 0; // replace final line feed with null
								}
								mau::comms::enqueuePrintMessage(line);
							}
						}
					} else {
						// Error
					}
        		} catch(const std::exception& ex){
        			printf("mau::comms::stdCaptureProcess - Caught exception:  %s\n", ex.what());
        		}
        	}

        	free(line);

        	// Restore flags and reconnect stdout after done
        	dup2(std_prev_fd, capture_fd);
        	fclose(pipe_file);
        }
    }
}
