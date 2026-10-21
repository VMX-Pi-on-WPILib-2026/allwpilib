/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/HAL.h"

#include <signal.h>
#include <unistd.h>

#include <utility>
#include <vector>

#include <wpi/raw_ostream.h>
#include <wpi/mutex.h>
#include <wpi/timestamp.h>

#include "ErrorsInternal.h"
#include "hal/DriverStation.h"
#include "hal/Errors.h"
#include "hal/Extensions.h"
#include "hal/handles/HandlesInternal.h"
#include "HALInitializer.h"
#include "MauClockInternal.h"
#include "MauInternal.h"
#include "Translator/include/FileHandler.h"
#include "MauTime.h"
#include "MauErrors.h"
#include "DriveStation/include/MauDriveData.h"
#include <VMXPi.h>
#include "DriveStation/include/DriverComms.hpp"

using namespace hal;

Mau_FileHandler* fileHandler = 0;
VMXPi* vmxpi = 0;

vmx::AHRS* mau::vmxIMU = 0;
VMXIO* mau::vmxIO = 0;
VMXCAN* mau::vmxCAN = 0;
VMXTime* mau::vmxTime = 0;
VMXPower* mau::vmxPower = 0;
VMXThread* mau::vmxThread = 0;

bool mau::vmxIOActive = false;

static VMXErrorCode vmxErrCode;
VMXErrorCode* mau::vmxError = &vmxErrCode;

Mau_ChannelMap* mau::channelMap = 0;
Mau_EnumConverter* mau::enumConverter = 0;

static bool shutdown_handler_invoked = false;

namespace hal {
    namespace init {

	// Cleans up "external" connections to devices and clients
        void ExternalCleanup() {
		if (mau::vmxIO) {
			if (!mau::vmxIO->ExpireWatchdogNow(mau::vmxError)) {
				printf("MAU HAL:  Error expiring IO Watchdog.\n");
				fflush(stdout);
			}
			mau::vmxIOActive = false;
		}
		// Shut down the drive station threads, give them time to terminate.
		TerminateDriverStation();
        	int timeout = 1500;
  		std::this_thread::sleep_for(std::chrono::milliseconds(timeout));		
	}

	// This shutdown handler is triggered up requests from Driver Station for shutdown/restart/estop.
        void ShutdownHandler(int param) {

        	printf("VMX HAL:  Shutdown Handler invoked.\n");
        shutdown_handler_invoked = true;
		fflush(stdout);

		ExternalCleanup();

		// The 2020 VMX image ran /usr/local/frc/bin/frcKillRobot.sh for E-Stop and
		// "Restart Robot Code". On the 2026 setup robot_manager supervises the
		// program and restarts it when it exits, so both just end the program.
		switch(param) {
		case MAU_COMMS_SHUTDOWN_REBOOT:
			system("systemctl reboot");
			break;
		default:
	        	kill(0, SIGTERM); // This should end up invoking the InternalShutdownHandler below.
		}
        }

    /* The InternalKillHandler() ensures that this process is killed; this is required in cases
     * where the SIGTERM handler is not sufficient to terminate the process.  Note that this
     * handler should only be invoked after all resources are released.
     *
     * This handle is typically invoked from a SIGTERM signal handler.
     */
	void InternalKillHandler() {

		printf("Entering VMX HAL Internal Kill Handler.\n");
  		fflush(stdout);
		fprintf(stderr, "Entering VMX HAL Internal Kill Handler.\n");
		shutdown_handler_invoked = true;

		ExternalCleanup();
		int timeout = 1500;
  		int kill_status = kill(0,0);
  		if (kill_status == 0) {
  			printf("FRC robot app did not respond to SIGTERM within %d ms.  Sending SIGKILL.\n", timeout);
  	  		fflush(stdout);
  			std::this_thread::sleep_for(std::chrono::milliseconds(10));
  			// Force kill -9
  			auto forceKill = kill(0, SIGKILL);
  			if (forceKill != 0) {
  				auto errorMsg = std::strerror(forceKill);
  				printf("Kill -9 error: %s\n", errorMsg);
  		  		fflush(stdout);
			}
  			// Give a bit of time for the kill to take place
  			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		} else {
  			printf("kill(0,0) returned %d; Robot Application terminated successfully via SIGTERM.\n", kill_status);
  	  		fflush(stdout);
		}
	}

        bool InitializeHAL() {

	    bool success = true;
       	    Mau_DriveData::initializeDriveData();

            fileHandler = new Mau_FileHandler();
            Mau_EnumConverter* enums = fileHandler->getEnumConverter();
            Mau_ChannelMap* maps = fileHandler->readChannelMap();

            mau::enumConverter = enums;
            mau::channelMap = maps;

            bool realtime = true;
            uint8_t hertz = 50;
            vmxpi = new VMXPi(realtime, hertz);

            if (!vmxpi->IsOpen()) {
           	//delete vmxpi;
            	printf("VMX HAL:  Error initializing VMX-pi HAL Library.\n");
            	//vmxpi = 0;
            	success = false;
            } else {
		printf("VMX HAL:  Library version %s\n", vmxpi->version.GetHALVersion().c_str());
	    }

            mau::vmxIMU = &vmxpi->ahrs;
            mau::vmxIO = &vmxpi->io;
            mau::vmxCAN = &vmxpi->can;
            mau::vmxTime = &vmxpi->time;
            mau::vmxPower = &vmxpi->power;
            mau::vmxThread = &vmxpi->thread;

    	    VMXPi *p_vmxpi = VMXPi::getInstance();
    	    if (p_vmxpi) {
    	    	p_vmxpi->registerFinalShutdownHandler(hal::init::InternalKillHandler);
    	    }

	    /* Initialize (early) the CAN subsystem. */
            InitializeCAN();
            InitializeCANAPI();

	    /* Enable the IO Watchdog */
	    VMXErrorCode vmxerr;
	    mau::vmxIO->SetWatchdogManagedOutputs(true /* flexdio */, true /* hicurrdio */, true /* commdio */, &vmxerr);
	    mau::vmxIO->SetWatchdogTimeoutPeriodMS(250, &vmxerr);
	    mau::vmxIO->SetWatchdogEnabled(true, &vmxerr);

	    /* Wait a few milliseconds, then force the watchdog to expire. */
	    /* Feeding the IO Watchdog will occur when                     */
	    /* DriveStation control words are received.                    */
	    mau::vmxTime->DelayMilliseconds(10);
	    
	    for (int i = 0; i < 3; i++) {
		if (!mau::vmxIO->ExpireWatchdogNow(&vmxerr)) {
			printf("MAU HAL:  Error expiring IO Watchdog.\n");
		} else {
			break;
		}
	    }

            InitializeAccelerometer();
	    InitializeAddressableLED();
            InitializeAnalogAccumulator();
            InitializeAnalogGyro();
            InitializeAnalogInput();
            InitializeAnalogInternal();
            InitializeAnalogOutput();
            InitializeCTREPCM();
            InitializeCTREPDP();
            InitializeConstants();
            InitializeCounter();
            InitializeDigitalInternal();
            InitializeDIO();
            InitializeDriverStation(hal::init::ShutdownHandler);
            InitializeEncoder();
            InitializeExtensions();
            InitializeI2C();
            InitializeInterrupts();
            InitializeMauClock();
            InitializeNotifier();
            InitializeOSSerialPort();
            InitializePorts();
            InitializePower();
            InitializePWM();
            InitializeRelay();
            InitializeSerialPort();
            InitializeREVPDH();
            InitializeREVPH();
            InitializeSPI();
            InitializeThreads();

            return success;
        }
    }
}

HAL_PortHandle HAL_GetPort(int32_t channel) {
    // Dont allow a number that wouldn't fit in a uint8_t
    if (channel < 0 || channel >= 255) return HAL_kInvalidHandle;
    return hal::createPortHandle(channel, 1);
}

/**
 * @deprecated Uses module numbers
 */
HAL_PortHandle HAL_GetPortWithModule(int32_t module, int32_t channel) {
    // Dont allow a number that wouldn't fit in a uint8_t
    if (channel < 0 || channel >= 255) return HAL_kInvalidHandle;
    if (module < 0 || module >= 255) return HAL_kInvalidHandle;
    return hal::createPortHandle(channel, module);
}

const char* HAL_GetErrorMessage(int32_t code) {
    switch (code) {
        case 0:
            return "";
        case CTR_RxTimeout:
            return CTR_RxTimeout_MESSAGE;
        case CTR_TxTimeout:
            return CTR_TxTimeout_MESSAGE;
        case CTR_InvalidParamValue:
            return CTR_InvalidParamValue_MESSAGE;
        case CTR_UnexpectedArbId:
            return CTR_UnexpectedArbId_MESSAGE;
        case CTR_TxFailed:
            return CTR_TxFailed_MESSAGE;
        case CTR_SigNotUpdated:
            return CTR_SigNotUpdated_MESSAGE;
        case NiFpga_Status_FifoTimeout:
            return NiFpga_Status_FifoTimeout_MESSAGE;
        case NiFpga_Status_TransferAborted:
            return NiFpga_Status_TransferAborted_MESSAGE;
        case NiFpga_Status_MemoryFull:
            return NiFpga_Status_MemoryFull_MESSAGE;
        case NiFpga_Status_SoftwareFault:
            return NiFpga_Status_SoftwareFault_MESSAGE;
        case NiFpga_Status_InvalidParameter:
            return NiFpga_Status_InvalidParameter_MESSAGE;
        case NiFpga_Status_ResourceNotFound:
            return NiFpga_Status_ResourceNotFound_MESSAGE;
        case NiFpga_Status_ResourceNotInitialized:
            return NiFpga_Status_ResourceNotInitialized_MESSAGE;
        case NiFpga_Status_HardwareFault:
            return NiFpga_Status_HardwareFault_MESSAGE;
        case NiFpga_Status_IrqTimeout:
            return NiFpga_Status_IrqTimeout_MESSAGE;
        case SAMPLE_RATE_TOO_HIGH:
            return SAMPLE_RATE_TOO_HIGH_MESSAGE;
        case VOLTAGE_OUT_OF_RANGE:
            return VOLTAGE_OUT_OF_RANGE_MESSAGE;
        case LOOP_TIMING_ERROR:
            return LOOP_TIMING_ERROR_MESSAGE;
        case SPI_WRITE_NO_MOSI:
            return SPI_WRITE_NO_MOSI_MESSAGE;
        case SPI_READ_NO_MISO:
            return SPI_READ_NO_MISO_MESSAGE;
        case SPI_READ_NO_DATA:
            return SPI_READ_NO_DATA_MESSAGE;
        case INCOMPATIBLE_STATE:
            return INCOMPATIBLE_STATE_MESSAGE;
        case NO_AVAILABLE_RESOURCES:
            return NO_AVAILABLE_RESOURCES_MESSAGE;
        case RESOURCE_IS_ALLOCATED:
            return RESOURCE_IS_ALLOCATED_MESSAGE;
        case RESOURCE_OUT_OF_RANGE:
            return RESOURCE_OUT_OF_RANGE_MESSAGE;
        case HAL_INVALID_ACCUMULATOR_CHANNEL:
            return HAL_INVALID_ACCUMULATOR_CHANNEL_MESSAGE;
        case HAL_HANDLE_ERROR:
            return HAL_HANDLE_ERROR_MESSAGE;
        case NULL_PARAMETER:
            return NULL_PARAMETER_MESSAGE;
        case ANALOG_TRIGGER_LIMIT_ORDER_ERROR:
            return ANALOG_TRIGGER_LIMIT_ORDER_ERROR_MESSAGE;
        case ANALOG_TRIGGER_PULSE_OUTPUT_ERROR:
            return ANALOG_TRIGGER_PULSE_OUTPUT_ERROR_MESSAGE;
        case PARAMETER_OUT_OF_RANGE:
            return PARAMETER_OUT_OF_RANGE_MESSAGE;
        case HAL_COUNTER_NOT_SUPPORTED:
            return HAL_COUNTER_NOT_SUPPORTED_MESSAGE;
        case HAL_ERR_CANSessionMux_InvalidBuffer:
            return ERR_CANSessionMux_InvalidBuffer_MESSAGE;
        case HAL_ERR_CANSessionMux_MessageNotFound:
            return ERR_CANSessionMux_MessageNotFound_MESSAGE;
        case HAL_WARN_CANSessionMux_NoToken:
            return WARN_CANSessionMux_NoToken_MESSAGE;
        case HAL_ERR_CANSessionMux_NotAllowed:
            return ERR_CANSessionMux_NotAllowed_MESSAGE;
        case HAL_ERR_CANSessionMux_NotInitialized:
            return ERR_CANSessionMux_NotInitialized_MESSAGE;
        case VI_ERROR_SYSTEM_ERROR:
            return VI_ERROR_SYSTEM_ERROR_MESSAGE;
        case VI_ERROR_INV_OBJECT:
            return VI_ERROR_INV_OBJECT_MESSAGE;
        case VI_ERROR_RSRC_LOCKED:
            return VI_ERROR_RSRC_LOCKED_MESSAGE;
        case VI_ERROR_RSRC_NFOUND:
            return VI_ERROR_RSRC_NFOUND_MESSAGE;
        case VI_ERROR_INV_RSRC_NAME:
            return VI_ERROR_INV_RSRC_NAME_MESSAGE;
        case VI_ERROR_QUEUE_OVERFLOW:
            return VI_ERROR_QUEUE_OVERFLOW_MESSAGE;
        case VI_ERROR_IO:
            return VI_ERROR_IO_MESSAGE;
        case VI_ERROR_ASRL_PARITY:
            return VI_ERROR_ASRL_PARITY_MESSAGE;
        case VI_ERROR_ASRL_FRAMING:
            return VI_ERROR_ASRL_FRAMING_MESSAGE;
        case VI_ERROR_ASRL_OVERRUN:
            return VI_ERROR_ASRL_OVERRUN_MESSAGE;
        case VI_ERROR_RSRC_BUSY:
            return VI_ERROR_RSRC_BUSY_MESSAGE;
        case VI_ERROR_INV_PARAMETER:
            return VI_ERROR_INV_PARAMETER_MESSAGE;
        case HAL_PWM_SCALE_ERROR:
            return HAL_PWM_SCALE_ERROR_MESSAGE;
        case HAL_CAN_TIMEOUT:
            return HAL_CAN_TIMEOUT_MESSAGE;
	// TODO:  These new error IDs are referenced in the latest WPILib, but not defined????
        //case ERR_FRCSystem_NetCommNotResponding:
        //    return ERR_FRCSystem_NetCommNotResponding_MESSAGE;
        //case ERR_FRCSystem_NoDSConnection:
        //    return ERR_FRCSystem_NoDSConnection_MESSAGE;
        /* Mau-specific errors */

        case MAU_UNSUPPORTED_FEATURE:
            return MAU_UNSUPPORTED_FEATURE_MESSAGE;

        case MAU_CHANNEL_MAP_ERROR:
        	return MAU_CHANNEL_MAP_ERROR_MESSAGE;

        default:
        	/* If no match was yet found, treat as a VMX HAL Library error code. */
        	return GetVMXErrorString(code);
    }
}

/**
 * Returns the runtime type of this HAL
 */
// Report a real robot (RobotBase.isReal()); RoboRIO2 avoids roboRIO 1-only code paths.
HAL_RuntimeType HAL_GetRuntimeType(void) { return HAL_Runtime_RoboRIO2; }

/**
 * Return the FPGA Version number.
 * For now, expect this to be competition year.
 * @return FPGA Version number.
 */
int32_t HAL_GetFPGAVersion(int32_t* status) {
    return 2018;  // Automatically script this at some point
}

/**
 * Return the FPGA Revision number.
 * The format of the revision is 3 numbers.
 * The 12 most significant bits are the Major Revision.
 * the next 8 bits are the Minor Revision.
 * The 12 least significant bits are the Build Number.
 * @return FPGA Revision number.
 */
int64_t HAL_GetFPGARevision(int32_t* status) {
    return 0;  // TODO: Find a better number to return;
}

void HAL_GetSerialNumber(struct WPI_String* serialNumber) {
    const char* serialNum = std::getenv("serialnum");
    if (!serialNum) serialNum = "";
    auto len = std::strlen(serialNum);
    auto write = WPI_AllocateString(serialNumber, len);
    std::memcpy(write, serialNum, len);
}

void HAL_GetComments(struct WPI_String* comments) {
    WPI_AllocateString(comments, 0);
}

int32_t HAL_GetTeamNumber(void) {
    return 0;  // The VMX-pi has no stored team number.
}

/**
 * Read the microsecond-resolution timer on the FPGA.
 *
 * @return The current time in microseconds according to the FPGA (since FPGA
 * reset).
 */
uint64_t HAL_GetFPGATime(int32_t* status) {
	*status = 0;
    return Mau_getTime();
}

uint64_t HAL_ExpandFPGATime(uint32_t unexpandedLower, int32_t* status) {
    // Capture the current FPGA time.  This will give us the upper half of the
    // clock.
    uint64_t fpgaTime = HAL_GetFPGATime(status);
    if (*status != 0) {
        return 0;
    }

    // Now, we need to detect the case where the lower bits rolled over after we
    // sampled.  In that case, the upper bits will be 1 bigger than they should
    // be.

    // Break it into lower and upper portions.
    uint32_t lower = fpgaTime & 0xffffffffull;
    uint64_t upper = (fpgaTime >> 32) & 0xffffffff;

    // The time was sampled *before* the current time, so roll it back.
    if (lower < unexpandedLower) {
        --upper;
    }

    return (upper << 32) + static_cast<uint64_t>(unexpandedLower);
}

/**
 * Get the state of the "USER" button on the roboRIO
 * @return true if the button is currently pressed down
 */
HAL_Bool HAL_GetFPGAButton(int32_t* status) {
    // return SimRoboRioData[0].GetFPGAButton();
    return 0; // TODO:  Add code to determine state of "FPGA" button
}

HAL_Bool HAL_GetSystemActive(int32_t* status) {
    return mau::vmxIOActive;
}

HAL_Bool HAL_GetBrownedOut(int32_t* status) {
    return false;  // TODO:  Add code to determine brownout state
}

int32_t HAL_GetCommsDisableCount(int32_t* status) {
    return 0;
}

HAL_Bool HAL_GetRSLState(int32_t* status) {
    return false;  // The VMX-pi has no RSL output.
}

HAL_Bool HAL_GetSystemTimeValid(int32_t* status) {
    return true;  // The Raspberry Pi system clock is managed by the OS.
}

static uint64_t dsStartTime;

namespace hal {
uint64_t GetDSInitializeTime() {
    return dsStartTime;
}
}  // namespace hal

HAL_Bool HAL_Initialize(int32_t timeout, int32_t mode) {

    // Initialize stdout/stderr capture.
    static std::atomic_bool initialized{false};
    static wpi::mutex initializeMutex;
    // Initial check, as if it's true initialization has finished
    if (initialized) return true;

    std::lock_guard<wpi::mutex> lock(initializeMutex);
    // Second check in case another thread was waiting
    if (initialized) return true;

    setlinebuf(stdin);
    setlinebuf(stdout);
    setlinebuf(stderr);
    wpi::outs().SetUnbuffered();
    mau::comms::start_log_capture();

    if (!hal::init::InitializeHAL()) {
    	// in case of failure, init driver station so that status reporting can occur.
    	hal::StartDriverStationComms();
    	mau::comms::setRobotProgramStarted(false);
    	mau::comms::setNotUserCode(true);
    	while (true) {
    		printf("Error Initializing VMX-pi HAL.  Robot app failed to start.\n");
    		// Now, wait for 5 seconds - giving time for the remote driver
    		// station to receive logging of errors occurring during startup.
    		int timeout = 5000;
    		std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
    	}
    	hal::init::TerminateDriverStation();
    	return false;
    }

    hal::init::HAL_IsInitialized.store(true);

    if (HAL_LoadExtensions() < 0) return false;
    //Mau_restartTiming();
    hal::StartDriverStationComms();
    int32_t status = 0;
    dsStartTime = HAL_GetFPGATime(&status);

    // Set WPI_Now to use FPGA timestamp
    wpi::SetNowImpl([]() -> uint64_t {
        int32_t status = 0;
        uint64_t rv = HAL_GetFPGATime(&status);
        if (status != 0) {
            wpi::errs()
               << "Call to HAL_GetFPGATime failed."
               << "Initialization might have failed. Time will not be correct\n";
            wpi::errs().flush();
           return 0u;
        }
        return rv;
    });

    initialized = true;
    return true;  // Add initialization if we need to at a later point
}

static wpi::mutex gOnShutdownMutex;
static std::vector<std::pair<void*, void (*)(void*)>> gOnShutdown;

void HAL_Shutdown(void) {
    std::vector<std::pair<void*, void (*)(void*)>> funcs;
    {
        std::scoped_lock lock(gOnShutdownMutex);
        funcs.swap(gOnShutdown);
    }
    for (auto&& func : funcs) {
        func.second(func.first);
    }
}

void HAL_OnShutdown(void* param, void (*func)(void*)) {
    std::scoped_lock lock(gOnShutdownMutex);
    gOnShutdown.emplace_back(param, func);
}

void HAL_SimPeriodicBefore(void) {}

void HAL_SimPeriodicAfter(void) {}

int64_t HAL_Report(int32_t resource, int32_t instanceNumber, int32_t context,
                   const char* feature) {
    return 0;  // Do nothing for now
}
