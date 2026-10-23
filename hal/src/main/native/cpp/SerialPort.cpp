// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md.
#include "hal/SerialPort.h"
#include "hal/Errors.h"
#ifdef __linux__
#include <sys/ioctl.h>
#endif
extern "C" void HAL_SetSerialDTR(HAL_SerialPortHandle handle, HAL_Bool asserted,
                                 int32_t* status) {
#ifdef __linux__
  int fd = HAL_GetSerialFD(handle, status);
  if (*status != 0)
    return;
  int flag = TIOCM_DTR;
  if (ioctl(fd, asserted ? TIOCMBIS : TIOCMBIC, &flag) != 0)
    *status = HAL_SERIAL_PORT_ERROR;
#else
  (void)handle;
  (void)asserted;
  *status = HAL_SERIAL_PORT_ERROR;
#endif
}
