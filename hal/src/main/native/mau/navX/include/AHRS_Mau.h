/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>
#include <string>

#include "hal/Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void	HAL_Mau_AHRS_Init(uint8_t update_rate_hz);
void	HAL_Mau_AHRS_ZeroYaw();
bool	HAL_Mau_AHRS_IsConnected();
double	HAL_Mau_AHRS_GetByteCount();
double	HAL_Mau_AHRS_GetUpdateCount();
void	HAL_Mau_AHRS_ResetDisplacement();
bool	HAL_Mau_AHRS_BlockOnNewCurrentRegisterData(uint32_t timeout_ms, uint8_t *first_reg_addr_out, uint8_t *p_data_out, uint8_t requested_len, uint8_t *p_len_out);
bool    HAL_Mau_AHRS_ReadConfigurationData(uint8_t first_reg, uint8_t *p_data_out, uint8_t requested_len);

#ifdef __cplusplus
}  // extern "C"
/** @} */
#endif
