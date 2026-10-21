/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Notifier.h"

#include <atomic>
#include <cstdlib>  // For std::atexit()
#include <memory>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
//#include <siginfo.h>
#include <time.h>
#include <sys/syscall.h>

//#include <wpi/condition_variable.h>
//#include <wpi/mutex.h>
#include <condition_variable>
#include <mutex>

#include "HALInitializer.h"
#include "hal/Errors.h"
#include "hal/HAL.h"
#include "hal/handles/UnlimitedHandleResource.h"
#include <VMXPi.h>
#include "MauTime.h"
#include "MauInternal.h"
#include "hal/cpp/fpga_clock.h"

using namespace hal;

static timer_t notifierTimer;

static std::mutex notifierMutex;
static timer_t *notifierAlarm = 0;
static uint64_t closestTrigger{UINT64_MAX};

namespace {

struct Notifier {
  uint64_t triggerTime = UINT64_MAX;
  uint64_t triggeredTime = UINT64_MAX;
  bool active = true;
  std::mutex mutex;
  std::condition_variable cond;
};

}  // namespace

static std::atomic_flag notifierAtexitRegistered{ATOMIC_FLAG_INIT};
static std::atomic_int notifierRefCount{0};

using namespace hal;

class NotifierHandleContainer
    : public UnlimitedHandleResource<HAL_NotifierHandle, Notifier,
                                     HAL_HandleEnum::Notifier> {
 public:
  void Cleanup() {
    ForEach([](HAL_NotifierHandle handle, Notifier* notifier) {
      {
        std::lock_guard<std::mutex> lock(notifier->mutex);
        notifier->active = false;
      }
      notifier->cond.notify_all();  // wake up any waiting threads
    });
  }
  virtual ~NotifierHandleContainer() {
	 Cleanup();
  }
};

static NotifierHandleContainer* notifierHandles = 0;

// Linux high-resolution timer callback, invoked via signal

//#define ENABLE_ALARM_DEBUG

static void alarmCallback(int signum) {
  int32_t status = 0;
  uint64_t currentTime = 0;

  //if (signum != SIGALRM) return;
  if (!notifierHandles) return;
  if (!notifierAlarm) return;

  std::lock_guard<std::mutex> lock(notifierMutex);

#ifdef ENABLE_ALARM_DEBUG
  struct timespec tp;
  char buffer [1024];

  uint64_t currFPGATime = HAL_GetFPGATime(&status);
  uint64_t currFPGADelta = currFPGATime - closestTrigger;

  if (notifierAlarm) {
    if (clock_gettime (CLOCK_MONOTONIC, &tp) == -1)
        perror ("clock_gettime");

    sprintf (buffer, "alarmCallback:  FPGATime:  %lld, Closest Trigger:  %lld (delta:  %lld); %ld s %ld ns overrun = %d\n", currFPGATime, closestTrigger, currFPGADelta, tp.tv_sec,
                        tp.tv_nsec, timer_getoverrun (*notifierAlarm));
    write (STDOUT_FILENO, buffer, strlen (buffer));
  }
#endif

#ifdef ENABLE_ALARM_DEBUG
        sprintf (buffer, "alarmCallback:  Got notifierMutex.\n");
        write (STDOUT_FILENO, buffer, strlen (buffer));
#endif

  // the hardware disables itself after each alarm
  closestTrigger = UINT64_MAX;

  // process all notifiers
  notifierHandles->ForEach([&](HAL_NotifierHandle handle, Notifier* notifier) {
    if (notifier->triggerTime == UINT64_MAX) return;
    if (currentTime == 0) currentTime = HAL_GetFPGATime(&status);
    std::unique_lock<std::mutex> lock(notifier->mutex);
    if (notifier->triggerTime < currentTime) {
      notifier->triggerTime = UINT64_MAX;
      notifier->triggeredTime = currentTime;
      lock.unlock();
      notifier->cond.notify_one();
    } else if (notifier->triggerTime < closestTrigger) {
      closestTrigger = notifier->triggerTime;
    }
  });

  if (notifierAlarm && (closestTrigger != UINT64_MAX)) {
    struct itimerspec new_value, old_value;
    uint64_t delta;
    if (closestTrigger > currentTime) {
    	delta = closestTrigger - currentTime;
    } else {
    	delta = 1000;
    }

    // Configure one-shot timer to expire at next closest trigger time
    new_value.it_value.tv_sec  = delta / 1000000; // NUM_MICROSECS_PER_SEC
    new_value.it_value.tv_nsec = (delta % 1000000) * 1000; // NUM_MICROSECS_PER_SEC, converted to nanoseconds
    new_value.it_interval.tv_sec = 0;
    new_value.it_interval.tv_nsec = 0;

    if (timer_settime (*notifierAlarm, 0, &new_value, &old_value) == -1) {
        perror ("timer_settime in Notifier alarmCallback.");
#ifdef ENABLE_ALARM_DEBUG
        sprintf (buffer, "Error invoking timer_settime, trying to set new ClosestTrigger:  delta:  %llu; tv_sec:  %lu; tv_nsec:  %lu (currentTime: %llu; closestTrigger:  %llu)\n", delta, new_value.it_value.tv_sec, new_value.it_value.tv_nsec, currentTime, closestTrigger);
        write (STDOUT_FILENO, buffer, strlen (buffer));
#endif
    } else {
#ifdef ENABLE_ALARM_DEBUG
        sprintf (buffer, "alarmCallback set new delta:  %llu; tv_sec:  %lu; tv_nsec:  %lu (currentTime: %llu; closestTrigger:  %llu)\n", delta, new_value.it_value.tv_sec, new_value.it_value.tv_nsec, currentTime, closestTrigger);
        write (STDOUT_FILENO, buffer, strlen (buffer));
#endif
    }
  } else {
#ifdef ENABLE_ALARM_DEBUG
        sprintf (buffer, "alarmCallback No new closest Trigger.\n");
        write (STDOUT_FILENO, buffer, strlen (buffer));
#endif
  }
}

static bool quit_sigalrm_receiver_thread = false;
static pthread_t sigalrm_receiver_thread;
static volatile bool notifier_active = false;

void *sigalrm_receiver_thread_func(void *arg) {
	struct sched_param param;
    struct sigaction action;
    sigevent_t sev;
    sigset_t set;
    siginfo_t siginfo;
    struct timespec timeout;

    /* SIGALRM for alarm callback */
    memset(&action, 0, sizeof (struct sigaction));
    memset(&sev, 0, sizeof(sev));
    memset(&set, 0, sizeof(set));
    memset(&siginfo, 0, sizeof(siginfo));
    memset(&timeout, 0, sizeof(timeout));

    action.sa_handler = alarmCallback;
    if (sigaction (SIGALRM, &action, NULL) == -1) {
  	  perror ("sigaction");
    }

    sev.sigev_notify = SIGEV_THREAD_ID;
    sev.sigev_signo = SIGALRM;
    //sev.sigev_notify_thread_id = syscall(__NR_gettid);
    sev._sigev_un._tid = syscall(__NR_gettid);
    sev.sigev_value.sival_int = 0;

    sigemptyset(&set);
    sigaddset(&set, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    if (timer_create (CLOCK_MONOTONIC, &sev, &notifierTimer) == -1) {
  	  perror ("timer_create");
    } else {
      notifierAlarm = &notifierTimer;
    }

#ifdef ENABLE_ALARM_DEBUG
    printf("sigalrm_receiver_thread_func() started.\n");
#endif

    quit_sigalrm_receiver_thread = false;

	/* Set this thread as highest priority */
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &param);

	while (!quit_sigalrm_receiver_thread) {
		timeout.tv_nsec = 50 * 1000000; // 50 milliseconds
		timeout.tv_sec = 0;
		int signum = sigtimedwait(&set, &siginfo, &timeout);
		if (signum == -1) {
				if (errno == EAGAIN) {
					if (notifier_active) {
#ifdef ENABLE_ALARM_DEBUG
						perror("Timeout in SIGALRM sigtimedwait().\n");
#endif
					}
				} else {
					perror("sigwait in sigalrm_receiver_thread().");
				}
		}
		alarmCallback(signum);
	}
	return NULL;
}

static void cleanupNotifierAtExit() {
  if (notifierAlarm) {
    timer_delete(*notifierAlarm);
    notifierAlarm = nullptr;
    quit_sigalrm_receiver_thread = true;
  }
}

static void HAL_Mau_NotifierTerminateHandler(void) {
	cleanupNotifierAtExit();
	if (notifierHandles != 0) {
		notifierHandles->Cleanup();
		notifierHandles = 0;
	}
}

namespace hal {
namespace init {
void InitializeNotifier() {
  static NotifierHandleContainer nH;
  notifierHandles = &nH;
  VMXPi *p_vmxpi = VMXPi::getInstance();
  if (p_vmxpi) {
	  p_vmxpi->registerShutdownHandler(HAL_Mau_NotifierTerminateHandler);
  }
}
}  // namespace init
}  // namespace hal

extern "C" {

HAL_NotifierHandle HAL_InitializeNotifier(int32_t* status) {
  hal::init::CheckInit();
  if (!notifierAtexitRegistered.test_and_set())
    std::atexit(cleanupNotifierAtExit);

  if (notifierRefCount.fetch_add(1) == 0) {
    std::lock_guard<std::mutex> lock(notifierMutex);
    // create alarm if not already created

    if (!notifierAlarm) {
      pthread_create(&sigalrm_receiver_thread, NULL, sigalrm_receiver_thread_func, NULL);
    }
  }

  std::shared_ptr<Notifier> notifier = std::make_shared<Notifier>();
  HAL_NotifierHandle handle = notifierHandles->Allocate(notifier);
  if (handle == HAL_kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return HAL_kInvalidHandle;
  }
  return handle;
}

void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle, const char* name,
                         int32_t* status) {}

void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return;

  {
    std::lock_guard<std::mutex> lock(notifier->mutex);
    notifier->triggerTime = UINT64_MAX;
    notifier->triggeredTime = 0;
    notifier->active = false;
  }
  notifier->cond.notify_all();  // wake up any waiting threads
}

void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle) {
  auto notifier = notifierHandles->Free(notifierHandle);
  if (!notifier) return;

  // Just in case HAL_StopNotifier() wasn't called...
  {
    std::lock_guard<std::mutex> lock(notifier->mutex);
    notifier->triggerTime = UINT64_MAX;
    notifier->triggeredTime = 0;
    notifier->active = false;
  }
  notifier->cond.notify_all();

  if (notifierRefCount.fetch_sub(1) == 1) {
    // if this was the last notifier, clean up alarm and manager
    // the notifier can call back into our callback, so don't hold the lock
    // here (the atomic fetch_sub will prevent multiple parallel entries
    // into this function)

    // Cleaning up the manager takes up to a second to complete, so don't do
    // that here. Fix it more permanently in 2019...

    // if (notifierAlarm) notifierAlarm->writeEnable(false, status);
    // if (notifierManager) notifierManager->disable(status);

    // std::lock_guard<std::mutex> lock(notifierMutex);
    // notifierAlarm = nullptr;
    // notifierManager = nullptr;
    // closestTrigger = UINT64_MAX;
  }
}

void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             uint64_t triggerTime, int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return;
  if (!notifierAlarm)  return;
  notifier_active = true;

#ifdef ENABLE_ALARM_DEBUG
  printf("TRACE:  HAL_UpdateNotifierAlarm entered - new trigger time:  %llu.\n", triggerTime);
  fflush(stdout);
#endif

  {
    std::lock_guard<std::mutex> lock(notifier->mutex);
    notifier->triggerTime = triggerTime;
    notifier->triggeredTime = UINT64_MAX;
  }

  std::lock_guard<std::mutex> lock(notifierMutex);

  // Update alarm time if closer than current.
  if (triggerTime < closestTrigger) {
    struct itimerspec new_value, old_value;

    bool was_active = (closestTrigger != UINT64_MAX);
    closestTrigger = triggerTime;

    int32_t status = 0;
    uint64_t currentTime = HAL_GetFPGATime(&status);

    uint64_t delta = closestTrigger - currentTime;

    if (currentTime > closestTrigger) {
	if (was_active) {
    	    {
    		std::lock_guard<std::mutex> notifier_lock(notifier->mutex);
    		// closest trigger already expired; trigger callback directly
    		// this will cause the next closest trigger to be configured.
    		notifier->triggerTime = UINT64_MAX;
    		notifier->triggeredTime = currentTime;
    	    }
            notifier->cond.notify_all();
#ifdef ENABLE_ALARM_DEBUG
	    printf("ERROR:  HAL_UpdateNotifierAlarm:  Notifier is active, and currentTime (%llu) > closestTrigger(%llu).\n", currentTime, closestTrigger);
	    fflush(stdout);
#endif
        } else {
#ifdef ENABLE_ALARM_DEBUG
	    printf("ERROR:  HAL_UpdateNotifierAlarm:  Notifier NOT active, and currentTime (%llu) > closestTrigger(%llu).\n", currentTime, closestTrigger);
	    fflush(stdout);
#endif
	}
	// Current time is before closest trigger; retrigger notifier right way
	delta = 100;
    }

    // Configure one-shot timer to expire at next closest trigger time
    new_value.it_value.tv_sec  = delta / 1000000; // NUM_MICROSECS_PER_SEC
    new_value.it_value.tv_nsec = (delta % 1000000) * 1000; // NUM_MICROSECS_PER_SEC, converted to nanoseconds
    new_value.it_interval.tv_sec = 0;
    new_value.it_interval.tv_nsec = 0;

    // Enable the alarm (whether it already was active or not)
    if (timer_settime (*notifierAlarm, 0, &new_value, &old_value) == -1) {
        perror ("timer_settime in HAL_UpdateNotifierAlarm.");
#ifdef ENABLE_ALARM_DEBUG
        printf("currentTime:     %llu\n", currentTime);
        printf("closestTrigger:  %llu\n", closestTrigger);
        printf("delta:           %llu\n", delta);    
        printf("tv_sec:          %lu\n", new_value.it_value.tv_sec);
        printf("tv_nsec:         %lu\n", new_value.it_value.tv_nsec);
        printf("*notifierAlarm:  %p\n", *notifierAlarm);
#endif
    } else {
#ifdef ENABLE_ALARM_DEBUG
    	printf("HAL_UpdateNotifierAlarm:  Set new delta:  %llu for notifierAlarm %p\n", delta, *notifierAlarm);
    	fflush(stdout);
#endif
    }
  } else {
#ifdef ENABLE_ALARM_DEBUG
    printf("HAL_UpdateNotifierAlarm:  Trigger Time:  %llu is >= closestTrigger  %llu\n", triggerTime, closestTrigger);
#endif
  }
#ifdef ENABLE_ALARM_DEBUG
  printf("TRACE:  HAL_UpdateNotifierAlarm exiting.\n");
  fflush(stdout);
#endif
}

void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return;

  {
    std::lock_guard<std::mutex> lock(notifier->mutex);
    notifier->triggerTime = UINT64_MAX;
  }
}

#if 0
// NOTE:  This version with timeout doesn't work in Java...
uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return 0;
  // Create a timeout, just in case of troubles
  auto timeoutTime = hal::fpga_clock::epoch()
			+ std::chrono::duration<double>(5.0);

  std::unique_lock<std::mutex> lock(notifier->mutex);
  bool wait_timeout = notifier->cond.wait_until(lock, timeoutTime, [&] {
    return !notifier->active || notifier->triggeredTime != UINT64_MAX;
  });
  if (wait_timeout) {
	  printf("Error!  HAL_WaitForNotifierAlarm() did not respond within the expected amount of time.\n");
	  fflush(stdout);
	  usleep(50000);
	  return 0;
  }
  return notifier->active ? notifier->triggeredTime : 0;
}
#endif
uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status) {
  auto notifier = notifierHandles->Get(notifierHandle);
  if (!notifier) return 0;
  try {
    std::unique_lock<std::mutex> lock(notifier->mutex);
    notifier->cond.wait(lock, [&] {
      return (!notifier->active || notifier->triggeredTime != UINT64_MAX);
    });
  } catch(std::exception& e) {
	printf("%s exception during HAL_WaitForNotifierAlarm\n", e.what());
	notifier->triggeredTime = 0;
  } catch(...) {
	printf("Error!  HAL_WaitForNotifierAlarm exception of unknown type.\n");
	notifier->triggeredTime = 0;
  }
  if (!notifier->active) {
	printf("Error!  Inactive Notifier discovered during HAL_WaitForNotifierAlarm.");
  }
  if (notifier->triggeredTime == 0) {
	printf("Error!  Notifier Triggered Time of 0 discovered during HAL_WaitForNotifierAlarm.");
  }
  if (0 != *status) {
	printf("Error!  Unclean status (%d) discovered during HAL_WaitForNotifierAlarm.", *status);
  }
  return notifier->active ? notifier->triggeredTime : 0;
}

HAL_Bool HAL_SetNotifierThreadPriority(HAL_Bool realTime, int32_t priority,
                                       int32_t* status) {
  return true;  // Same as the sim HAL; the VMX-pi notifier runs on timer signals.
}
}  // extern "C"
