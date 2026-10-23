#include <atomic>
#include <jni.h>
#include "AHRS.h"

namespace hal::init {
std::atomic_bool HAL_IsInitialized{false};
}
static vmx::AHRS device;
namespace mau {
vmx::AHRS* vmxIMU = &device;
}
// Only the SDK and the HAL exception reporter are faked; production JNI is
// used.
namespace hal {
void ThrowError(JNIEnv* env, int32_t, int32_t, int32_t, int32_t) {
  env->ThrowNew(env->FindClass("java/lang/RuntimeException"), "HAL IMU error");
}
}  // namespace hal
extern "C" JNIEXPORT void JNICALL
Java_IMURegression_setInitialized(JNIEnv*, jclass, jboolean initialized) {
  hal::init::HAL_IsInitialized = initialized;
  device.values["GetYaw"] = -170;
  device.values["GetAngle"] = 450;
  device.values["GetRate"] = -15;
  device.values["IsConnected"] = 1;
}
