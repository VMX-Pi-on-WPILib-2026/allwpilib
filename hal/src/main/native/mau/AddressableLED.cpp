/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/PWM.h"
#include "hal/AddressableLED.h"
#include "hal/HALBase.h"

#include <cstring>

#include "ConstantsInternal.h"
#include "DigitalInternal.h"
#include "HALInitializer.h"
#include "PortsInternal.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedHandleResource.h"

using namespace hal;

namespace {
struct AddressableLED {
  HAL_DigitalHandle pwm_handle = HAL_kInvalidHandle;
  int buffer_num_pixels = 0;
  LEDArrayBufferHandle buffer_handle = 0;
  pthread_t update_thread;
  bool update_thread_running = false;
  bool stop_update_thread = false;
  int32_t wpi_digital_input_channel = -1;
  HAL_AddressableLEDColorOrder colorOrder = HAL_ALED_GRB;
};
}  // namespace

static LimitedHandleResource<
    HAL_AddressableLEDHandle, AddressableLED, kNumAddressableLEDs,
    HAL_HandleEnum::AddressableLED>* addressableLEDHandles;

namespace hal {
namespace init {
void InitializeAddressableLED() {
  static LimitedHandleResource<HAL_AddressableLEDHandle, AddressableLED,
                               kNumAddressableLEDs,
                               HAL_HandleEnum::AddressableLED>
      alH;
  addressableLEDHandles = &alH;
}
}  // namespace init
}  // namespace hal

void *ledarray_update_thread_func(void *arg) {
    AddressableLED *p_ledinfo = (AddressableLED *)arg;
    if (p_ledinfo) {
	if (p_ledinfo->pwm_handle != HAL_kInvalidHandle) {
  		auto port =
		      hal::digitalChannelHandles->Get(p_ledinfo->pwm_handle, hal::HAL_HandleEnum::PWM);
		if (port) {
			while(!p_ledinfo->stop_update_thread) {
				int32_t status;
				mau::vmxIO->LEDArray_Render(port->vmx_res_handle, &status);
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
		}
        }
    }
    return 0;
}

extern "C" {

// The PWM Port Handle passed in is currently allocated.  This function must
// first deallocate the port handle, then reallocate it as LEDArray_OneWire.

HAL_AddressableLEDHandle HAL_InitializeAddressableLED(
    HAL_DigitalHandle outputPort, int32_t* status) {
  hal::init::CheckInit();

  auto port =
      hal::digitalChannelHandles->Get(outputPort, hal::HAL_HandleEnum::PWM);

  if (!port) {
    *status = HAL_LED_CHANNEL_ERROR;
    return HAL_kInvalidHandle;
  }

  // Retrieve certain values from the DigitalPort object
  int32_t wpi_digital_input_channel = port->channel;
  VMXChannelInfo vmx_chan_info = port->vmx_chan_info;

  // Verify the channel has LEDArray_OneWire capability
  if (!mau::vmxIO->ChannelSupportsCapability(vmx_chan_info.index, VMXChannelCapability::LEDArray_OneWire)) {
    *status = MAU_PWM_CHANNEL_LEDARRAY_INCOMPATIBILITY;
    return HAL_kInvalidHandle;
  }

  vmx_chan_info.capabilities = VMXChannelCapability::LEDArray_OneWire;

  auto handle = addressableLEDHandles->Allocate();

  if (handle == HAL_kInvalidHandle) {
    *status = NO_AVAILABLE_RESOURCES;
    return HAL_kInvalidHandle;
  }

  auto led = addressableLEDHandles->Get(handle);

  if (!led) {
    addressableLEDHandles->Free(handle);
    *status = HAL_HANDLE_ERROR;
    return HAL_kInvalidHandle;
  }

  // The passed-in PWM Handle is already allocated.  It must be deallocated before rellocation
  // as a LEDArray_OneWire channel
  HAL_FreePWMPort(outputPort);
  if (*status != 0) {
    addressableLEDHandles->Free(handle);
    return HAL_kInvalidHandle;
  }

  // Reallocate the PWM handle
  HAL_DigitalHandle pwmHandle;
  auto new_port = allocateDigitalHandleAndInitializedPort(HAL_HandleEnum::PWM, wpi_digital_input_channel, pwmHandle, status);
  if (new_port == nullptr) {
      return HAL_kInvalidHandle;
  }

  /* Set Configuration to defaults, including WPI-library compliant PWM Frequency and DutyCycle */
  new_port->ledarray_config = LEDArray_OneWireConfig();
  if (!mau::vmxIO->ActivateSinglechannelResource(vmx_chan_info, &new_port->ledarray_config, new_port->vmx_res_handle, status)) {
      int32_t temp_status;
      digitalChannelHandles->Free(pwmHandle, HAL_HandleEnum::PWM);
      HAL_InitializePWMPort(HAL_GetPort(wpi_digital_input_channel), nullptr, &temp_status);
      addressableLEDHandles->Free(handle);
      return HAL_kInvalidHandle;
  } else {
    led->pwm_handle = pwmHandle;
    new_port->configSet = true;
    led->wpi_digital_input_channel = wpi_digital_input_channel;
  }

  return handle;
}

void HAL_FreeAddressableLED(HAL_AddressableLEDHandle handle) {

  auto led = addressableLEDHandles->Get(handle);

  if (!led) {
    return;
  }

  auto port =
      hal::digitalChannelHandles->Get(led->pwm_handle, hal::HAL_HandleEnum::PWM);

  if (!port) {
    return;
  }

  // Stop thread (if any) which is currently updating the LEDARray.
  int32_t status;
  HAL_StopAddressableLEDOutput(handle, &status);

  // Now that the thread is stopped, free the LEDArray Buffer
  if (led->buffer_handle) {
    mau::vmxIO->LEDArrayBuffer_Delete(led->buffer_handle, &status);
    led->buffer_handle = 0;
  }

  // Deallocate the LEDArray_OneWire Resource, if currently allocated
  VMXResourceHandle vmxResource = port->vmx_res_handle;
  bool allocated = false;
  bool isShared = false;
  mau::vmxIO->IsResourceAllocated(vmxResource, allocated, isShared, &status);
  if (allocated) {
	mau::vmxIO->DeallocateResource(vmxResource, &status);
        port->vmx_res_handle = CREATE_VMX_RESOURCE_HANDLE(VMXResourceType::Undefined,INVALID_VMX_RESOURCE_INDEX);
	port->configSet = false;
  }

  HAL_InitializePWMPort(HAL_GetPort(led->wpi_digital_input_channel), nullptr, &status);

  addressableLEDHandles->Free(handle);
}

// NOTE:  This does not appear to be used, and thus it's not strictly required to implement it now.

void HAL_SetAddressableLEDOutputPort(HAL_AddressableLEDHandle handle,
                                     HAL_DigitalHandle outputPort,
                                     int32_t* status) {
  std::printf("HAL_SetAddressableLEDOutputPort - TODO - Implementation goes Here.");
}

void HAL_SetAddressableLEDLength(HAL_AddressableLEDHandle handle,
                                 int32_t length, int32_t* status) {
  auto led = addressableLEDHandles->Get(handle);

  if (!led) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  auto port =
      hal::digitalChannelHandles->Get(led->pwm_handle, hal::HAL_HandleEnum::PWM);

  if (!port) {
    *status = HAL_LED_CHANNEL_ERROR;
    return;
  }

  VMXResourceHandle vmxResource = port->vmx_res_handle;
  port->ledarray_config.SetNumPixels(length);
  if (!mau::vmxIO->LEDArray_Configure(vmxResource, port->ledarray_config, status)) {
    std::printf("HAL_SetAddressableLEDLength Error:  %s\n", HAL_GetErrorMessage(*status));
    return;
  } else {
    led->buffer_num_pixels = length;
    if (led->buffer_handle) {
      if (!mau::vmxIO->LEDArrayBuffer_Delete(led->buffer_handle, status)) {
        std::printf("Error deleting VMXIO LEDArray Buffer:  %s\n", HAL_GetErrorMessage(*status));
      }
      led->buffer_handle = 0;
    }
    if (!mau::vmxIO->LEDArrayBuffer_Create(length, led->buffer_handle, status)) {
      std::printf("Error creating VMXIO LEDArray Buffer:  %s\n", HAL_GetErrorMessage(*status));
    }
    if (!mau::vmxIO->LEDArray_SetBuffer(vmxResource, led->buffer_handle, status)) {
      std::printf("Error setting LEDArrayBuffer to LEDArray Driver:  %s\n", HAL_GetErrorMessage(*status));
    }
  }
}

static_assert(4 == sizeof(HAL_AddressableLEDData),
              "LED Structs MUST be the same size");

void HAL_WriteAddressableLEDData(HAL_AddressableLEDHandle handle,
                                 const struct HAL_AddressableLEDData* data,
                                 int32_t length, int32_t* status) {
  auto led = addressableLEDHandles->Get(handle);

  if (!led) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  int num_pixels = led->buffer_num_pixels;

  auto port =
      hal::digitalChannelHandles->Get(led->pwm_handle, hal::HAL_HandleEnum::PWM);

  if (!port) {
    *status = HAL_LED_CHANNEL_ERROR;
    return;
  }

  LEDArrayBufferHandle buffer_handle = led->buffer_handle;

  int num_leds_to_write = (length > num_pixels) ? num_pixels : length;
  for (int i = 0; i < num_leds_to_write; i++) {
    // The VMX-pi LED driver sends each pixel in WS2812 (GRB) order:
    // (green arg, red arg, blue arg). Place the colors so the wire order
    // matches the requested color order.
    uint8_t wire[3];
    switch (led->colorOrder) {
      case HAL_ALED_RGB: wire[0] = data[i].r; wire[1] = data[i].g; wire[2] = data[i].b; break;
      case HAL_ALED_RBG: wire[0] = data[i].r; wire[1] = data[i].b; wire[2] = data[i].g; break;
      case HAL_ALED_BGR: wire[0] = data[i].b; wire[1] = data[i].g; wire[2] = data[i].r; break;
      case HAL_ALED_BRG: wire[0] = data[i].b; wire[1] = data[i].r; wire[2] = data[i].g; break;
      case HAL_ALED_GBR: wire[0] = data[i].g; wire[1] = data[i].b; wire[2] = data[i].r; break;
      case HAL_ALED_GRB:
      default:           wire[0] = data[i].g; wire[1] = data[i].r; wire[2] = data[i].b; break;
    }
    if (!mau::vmxIO->LEDArrayBuffer_SetRGBValue(buffer_handle, i, wire[1], wire[0], wire[2], status)) {
      return;
    }
  }
}

void HAL_SetAddressableLEDColorOrder(HAL_AddressableLEDHandle handle,
                                     HAL_AddressableLEDColorOrder colorOrder,
                                     int32_t* status) {
  auto led = addressableLEDHandles->Get(handle);
  if (!led) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  led->colorOrder = colorOrder;
}

void HAL_SetAddressableLEDBitTiming(HAL_AddressableLEDHandle handle,
                                    int32_t lowTime0NanoSeconds,
                                    int32_t highTime0NanoSeconds,
                                    int32_t lowTime1NanoSeconds,
                                    int32_t highTime1NanoSeconds,
                                    int32_t* status) {
  auto led = addressableLEDHandles->Get(handle);

  if (!led) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  auto port =
      hal::digitalChannelHandles->Get(led->pwm_handle, hal::HAL_HandleEnum::PWM);

  if (!port) {
    *status = HAL_LED_CHANNEL_ERROR;
    return;
  }

  VMXResourceHandle vmxResource = port->vmx_res_handle;
  port->ledarray_config.SetOneSymbolHighTimeNanoseconds(highTime1NanoSeconds);
  port->ledarray_config.SetZeroSymbolHighTimeNanoseconds(highTime0NanoSeconds);
  if (!mau::vmxIO->LEDArray_Configure(vmxResource, port->ledarray_config, status)) {
    std::printf("LEDArray_Configure Error:  %s\n", HAL_GetErrorMessage(*status));
    return;
  }
}

void HAL_SetAddressableLEDSyncTime(HAL_AddressableLEDHandle handle,
                                   int32_t syncTimeMicroSeconds,
                                   int32_t* status) {
  auto led = addressableLEDHandles->Get(handle);

  if (!led) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  auto port =
      hal::digitalChannelHandles->Get(led->pwm_handle, hal::HAL_HandleEnum::PWM);

  if (!port) {
    *status = HAL_LED_CHANNEL_ERROR;
    return;
  }

  VMXResourceHandle vmxResource = port->vmx_res_handle;
  port->ledarray_config.SetResetWaitTimeMicroseconds(syncTimeMicroSeconds);
  if (!mau::vmxIO->LEDArray_Configure(vmxResource, port->ledarray_config, status)) {
    std::printf("LEDArray_Configure Error:  %s\n", HAL_GetErrorMessage(*status));
    return;
  }
}

void HAL_StartAddressableLEDOutput(HAL_AddressableLEDHandle handle,
                                  int32_t* status) {
  auto led = addressableLEDHandles->Get(handle);

  if (!led) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  if (!led->update_thread_running) {
    led->stop_update_thread = false;
    if (!pthread_create(&led->update_thread, NULL, ledarray_update_thread_func, led.get())) {
      led->update_thread_running = true;
    } else {
      *status = HAL_HANDLE_ERROR;
      return;
    }
  }
}

void HAL_StopAddressableLEDOutput(HAL_AddressableLEDHandle handle,
                                  int32_t* status) {
  auto led = addressableLEDHandles->Get(handle);

  if (!led) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  if (led->update_thread_running) {
    led->stop_update_thread = true;
    pthread_join(led->update_thread,NULL);
    led->update_thread_running = false;
  } else {
    *status = HAL_HANDLE_ERROR;
    return;
  }
}

}  // extern "C"
