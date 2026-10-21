/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <jni.h>

#include "com_kauailabs_vmx_AHRSJNI.h"
#include "../include/AHRS_Mau.h"
#include <wpi/jni_util.h>

#include "hal/Types.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     com_kauailabs_vmx_AHRSJNI
 * Method:    Init
 * Signature: (B)V
 */
JNIEXPORT void JNICALL Java_com_kauailabs_vmx_AHRSJNI_Init
  (JNIEnv *, jclass, jbyte update_rate_hz)
{
	HAL_Mau_AHRS_Init(update_rate_hz);
}

/*
 * Class:     com_kauailabs_vmx_AHRSJNI
 * Method:    ZeroYaw
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_kauailabs_vmx_AHRSJNI_ZeroYaw
  (JNIEnv *, jclass)
{
	HAL_Mau_AHRS_ZeroYaw();
}

/*
 * Class:     com_kauailabs_vmx_AHRSJNI
 * Method:    IsConnected
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_kauailabs_vmx_AHRSJNI_IsConnected
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_IsConnected();
}

/*
 * Class:     com_kauailabs_vmx_AHRSJNI
 * Method:    GetByteCount
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_com_kauailabs_vmx_AHRSJNI_GetByteCount
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetByteCount();
}

/*
 * Class:     com_kauailabs_vmx_AHRSJNI
 * Method:    GetUpdateCount
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_com_kauailabs_vmx_AHRSJNI_GetUpdateCount
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetUpdateCount();
}

/*
 * Class:     com_kauailabs_vmx_AHRSJNI
 * Method:    ResetDisplacement
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_kauailabs_vmx_AHRSJNI_ResetDisplacement
  (JNIEnv *, jclass)
{
	HAL_Mau_AHRS_ResetDisplacement();
}

/*
 * Class:     com_kauailabs_vmx_AHRSJNI
 * Method:    BlockOnNewCurrentRegisterData
 * Signature: (I[B[BB[B)Z
 */
JNIEXPORT jboolean JNICALL Java_com_kauailabs_vmx_AHRSJNI_BlockOnNewCurrentRegisterData
  (JNIEnv * env, jclass, jint timeout_ms, jbyteArray first_reg_addr_out, jbyteArray data_out, jbyte requested_data_len, jbyteArray data_len_out )
{
  jbyte *firstRegAddrPtr = env->GetByteArrayElements(first_reg_addr_out, NULL);
  jbyte *dataOutPtr      = env->GetByteArrayElements(data_out, NULL);
  jbyte *dataLenOutPtr   = env->GetByteArrayElements(data_len_out, NULL);
  jboolean returnValue = HAL_Mau_AHRS_BlockOnNewCurrentRegisterData(
				timeout_ms,
				(uint8_t *)firstRegAddrPtr,
				(uint8_t *)dataOutPtr,
				requested_data_len,
				(uint8_t *)dataLenOutPtr);
  if (returnValue) {
      env->ReleaseByteArrayElements(first_reg_addr_out, firstRegAddrPtr, 0);
      env->ReleaseByteArrayElements(data_out, dataOutPtr, 0);
      env->ReleaseByteArrayElements(data_len_out, dataLenOutPtr, 0);
  }
  return returnValue;
}

/*
 * Class:     com_kauailabs_vmx_AHRSJNI
 * Method:    ReadConfigurationData
 * Signature: (I[BB)Z
 */
JNIEXPORT jboolean JNICALL Java_com_kauailabs_vmx_AHRSJNI_ReadConfigurationData
  (JNIEnv * env, jclass, jbyte first_reg, jbyteArray data_out, jbyte requested_data_len )
{
  jbyte *dataOutPtr      = env->GetByteArrayElements(data_out, NULL);
  jboolean returnValue = HAL_Mau_AHRS_ReadConfigurationData(
				first_reg,
				(uint8_t *)dataOutPtr,
				requested_data_len);
  if (returnValue) {
      env->ReleaseByteArrayElements(data_out, dataOutPtr, 0);
  }
  return returnValue;

}

}  // extern "C"
