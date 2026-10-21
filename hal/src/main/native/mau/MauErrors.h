/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#pragma once

#define MAU_CHANNEL_MAP_ERROR_MESSAGE					"Requested channel not present in VMX-pi Channel Map"
#define MAU_CHANNEL_MAP_ERROR							-21000

#define MAU_HICURRDIO_OUTPUTMODE_NOT_ENABLED_MESSAGE	"HighCurrentDIO Output Mode must be enabled"
#define MAU_HICURRDIO_OUTPUTMODE_NOT_ENABLED 			-21001

#define MAU_DIO_NOT_OUTPUT_CAPABLE_MESSAGE				"This Digital Channel is not output-capable"
#define MAU_DIO_NOT_OUTPUT_CAPABLE						-21002

#define MAU_DIO_ASSIGNED_TO_PWMGEN_MESSAGE				"This Digital Channel is currently routed to a PWM Generator"
#define MAU_DIO_ASSIGNED_TO_PWMGEN						-21003

#define MAU_DIO_CHANNEL_ENCODER_INCOMPATIBILITY_MESSAGE	"The specified Digital Channels cannot be used with a VMX-pi Encoder"
#define MAU_DIO_CHANNEL_ENCODER_INCOMPATIBILITY			-21004

#define ENCODER_INDEX_SRC_INT_CONFIG_CONFLICT_MESSAGE   "The provided encoder index source is configured for interrupt edges conflicting with requested encoder indexing type"
#define ENCODER_INDEX_SRC_INT_CONFIG_CONFLICT			-21005

#define MAU_DIO_NOT_INPUT_CAPABLE_MESSAGE				"This Digital Channel is not input-capable"
#define MAU_DIO_NOT_INPUT_CAPABLE						-21006

#define MAU_ENCODER_ANALOG_INPUTS_UNSUPPORTED_MESSAGE	"VMX-pi Analog Inputs may not be used as Encoder source signals"
#define MAU_ENCODER_ANALOG_INPUTS_UNSUPPORTED			-21007

#define MAU_COUNTER_PULSELENGTH_MODE_UNSUPPORTED_MESSAGE "Counter Pulse Length mode not currently supported"
#define MAU_COUNTER_PULSELENGTH_MODE_UNSUPPORTED		-21008

#define MAU_COUNTER_ANALOG_INPUTS_UNSUPPORTED_MESSAGE	"VMX-pi Analog Inputs may not be used as Counter source signals"
#define MAU_COUNTER_ANALOG_INPUTS_UNSUPPORTED			-21009

#define MAU_COUNTER_DUAL_INPUTS_UNSUPPORTED_MESSAGE		"VMX-pi Counters only support dual input sources in ExternalDirection mode"
#define MAU_COUNTER_DUAL_INPUTS_UNSUPPORTED				-21010

#define MAU_DIO_CHANNEL_COUNTER_INCOMPATIBILITY_MESSAGE	"The specified Digital Channels cannot be used with a VMX-pi Counter"
#define MAU_DIO_CHANNEL_COUNTER_INCOMPATIBILITY			-21011

#define MAU_COUNTER_EXT_DIRECTION_MODE_REQUIRED_MESSAGE	"Counter External Direction mode required for this feature"
#define MAU_COUNTER_EXT_DIRECTION_MODE_REQUIRED			-21012

#define MAU_COUNTER_SEMIPERIOD_MODE_REQUIRED_MESSAGE	"Counter SemiPeriod mode required for this feature"
#define MAU_COUNTER_SEMIPERIOD_MODE_REQUIRED			-21013

#define MAU_COUNTER_DUAL_INPUTS_REQUIRED_MESSAGE		"VMX-pi Counters in External Direction mode require two input sources"
#define MAU_COUNTER_DUAL_INPUTS_REQUIRED				-21014

#define MAU_ANALOG_OUTPUTS_UNSUPPORTED_MESSAGE			"VMX-pi does not currently support Analog Outputs"
#define MAU_ANALOG_OUTPUTS_UNSUPPORTED					-21015

#define MAU_PWM_CHANNEL_LEDARRAY_INCOMPATIBILITY_MESSAGE	"The specified PWM Channel cannot be used with a VMX-pi LEDArray channel"
#define MAU_PWM_CHANNEL_LEDARRAY_INCOMPATIBILITY			-21016

// Mau error codes are negative values <= MAU_ERRNO_CODE_BASE
#define MAU_ERRNO_CODE_BASE	MAU_CHANNEL_MAP_ERROR

