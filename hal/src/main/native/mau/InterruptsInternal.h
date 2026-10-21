/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>
#include "hal/handles/HandlesInternal.h"
#include <VMXResource.h>

namespace hal {
	bool GetVMXInterruptResourceIndexForDigitalSourceHandle(HAL_Handle digitalSourceHandle, VMXResourceIndex& int_res_index, VMXChannelInfo& vmx_chan_info, int32_t* status);
}
