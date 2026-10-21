/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../include/AHRS_Mau.h"

#include <AHRS.h>
#include <VMXPi.h>

static vmx::AHRS *p_ahrs = 0;

void	HAL_Mau_AHRS_Init(uint8_t update_rate_hz)
{
	VMXPi *vmx_pi = VMXPi::getInstance();
	p_ahrs = &(vmx_pi->getAHRS());
}

void	HAL_Mau_AHRS_ZeroYaw()
{
	p_ahrs->ZeroYaw();
}

void	HAL_Mau_AHRS_ResetDisplacement()
{
	p_ahrs->ResetDisplacement();
}

bool	HAL_Mau_AHRS_IsConnected()
{
	return p_ahrs->IsConnected();
}

double	HAL_Mau_AHRS_GetByteCount()
{
	return p_ahrs->GetByteCount();
}

double	HAL_Mau_AHRS_GetUpdateCount()
{
	return p_ahrs->GetUpdateCount();
}

bool HAL_Mau_AHRS_BlockOnNewCurrentRegisterData(uint32_t timeout_ms, uint8_t *first_reg_addr_out, uint8_t *p_data_out, uint8_t requested_len, uint8_t *p_len_out)
{
	return p_ahrs->BlockOnNewCurrentRegisterData(timeout_ms, first_reg_addr_out, p_data_out, requested_len, p_len_out);
}

bool HAL_Mau_AHRS_ReadConfigurationData(uint8_t first_reg, uint8_t *p_data_out, uint8_t requested_len)
{
	return p_ahrs->ReadConfigurationData(first_reg, p_data_out, requested_len);
}
