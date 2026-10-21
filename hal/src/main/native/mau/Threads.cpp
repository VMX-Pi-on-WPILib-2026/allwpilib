/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/Threads.h"
#include "hal/Errors.h"

#include <pthread.h>

namespace hal {
    namespace init {
        void InitializeThreads() {}
    }
}

int32_t HAL_GetThreadPriority(NativeThreadHandle handle, HAL_Bool* isRealTime, int32_t* status) {
	// NativeThreadHandle is const pthread_t*
	int policy;
	struct sched_param schedparam;
	*isRealTime = false;
	if (!pthread_getschedparam(*reinterpret_cast<const pthread_t*>(handle), &policy, &schedparam)) {
		*isRealTime = (policy == SCHED_FIFO);
		return schedparam.__sched_priority;
	} else {
		*status = HAL_THREAD_PRIORITY_ERROR;
	}
	return 0;
}

int32_t HAL_GetCurrentThreadPriority(HAL_Bool* isRealTime, int32_t* status) {
    pthread_t curr_thread = pthread_self();
    return HAL_GetThreadPriority(&curr_thread, isRealTime, status);
}

// TODO:  In general, how should the "realTime" parameter be handled below?

HAL_Bool HAL_SetThreadPriority(NativeThreadHandle handle, HAL_Bool realTime, int32_t priority, int32_t* status) {
	int policy;
	struct sched_param schedparam;

	int curr_policy = realTime ? SCHED_FIFO : SCHED_OTHER;

	int min_priority = sched_get_priority_min(curr_policy);
	int max_priority = sched_get_priority_max(curr_policy);

	if ((priority < min_priority) || (priority > max_priority)) {
		*status = HAL_THREAD_PRIORITY_RANGE_ERROR;
		return false;
	}

	if (!pthread_getschedparam(*reinterpret_cast<const pthread_t*>(handle), &policy, &schedparam)) {
		schedparam.__sched_priority = priority;
		if (!pthread_setschedparam(*reinterpret_cast<const pthread_t*>(handle), policy, &schedparam)) {
			return true;
		}
	}
    return false;
}

HAL_Bool HAL_SetCurrentThreadPriority(HAL_Bool realTime, int32_t priority, int32_t* status) {
    pthread_t curr_thread = pthread_self();
    return HAL_SetThreadPriority(&curr_thread, priority, realTime, status);
}
