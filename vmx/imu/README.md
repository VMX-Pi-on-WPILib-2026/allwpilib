# VMX onboard IMU in WPILib

`frc::VMXIMU` (C++) and `edu.wpi.first.wpilibj.VMXIMU` (Java) read the onboard
VMX-pi IMU through the VMX HAL. No external navX/Studica vendordep is required.
Use matching HAL and robot libraries from `2026.2.2-vmx.8` or later; older
VMX releases do not expose these functions. Published VMXIMU binaries target
linuxarm64. Other HAL platform classifiers in the bundle remain upstream WPILib
binaries; using VMXIMU on another platform requires building this tree (including
its explicit unsupported-platform HAL stubs).

The transport follows Studica Robotics' ROS2 IMU driver:
- https://github.com/Studica-Robotics/ROS2/blob/main/drivers/imu.cpp
- https://github.com/Studica-Robotics/ROS2/blob/main/drivers/imu.hpp

The implementation uses the HAL's existing `mau::vmxIMU` (`VMXPi::ahrs`), rather
than opening an MXP SPI port or constructing another VMXPi. The proprietary VMX
SDK stays inside the HAL backend; its headers/types are not needed by robot code
and its binary is not redistributed. Upstream reference license:
[Studica ROS2 Apache-2.0](../studica/Studica-ROS2-LICENSE.txt).

## Use

Construct after WPILib/HAL starts, for example in `RobotInit()`. C++:

```cpp
#include <frc/VMXIMU.h>
#include <memory>

std::unique_ptr<frc::VMXIMU> imu;

void Robot::RobotInit() {
  imu = std::make_unique<frc::VMXIMU>();
}

void Robot::RobotPeriodic() {
  if (imu->IsConnected() && !imu->IsCalibrating()) {
    double headingDegrees = imu->GetYaw();
    // Use headingDegrees in your control loop.
  }
}
```

Java:

```java
import edu.wpi.first.wpilibj.VMXIMU;

private VMXIMU imu;

@Override
public void robotInit() {
  imu = new VMXIMU();
}

@Override
public void robotPeriodic() {
  if (imu.isConnected() && !imu.isCalibrating()) {
    double headingDegrees = imu.getYaw();
    // Use headingDegrees in your control loop.
  }
}
```

The APIs expose yaw/pitch/roll, continuous angle/rate, compass/fused heading,
quaternion, gravity-compensated acceleration, raw sensor-frame readings,
velocity/displacement, firmware, timestamps, configured yaw axis, update rates,
and connection/calibration flags. Barometer/altitude methods reflect the SDK;
check validity flags and the board's capabilities before using them.

- Angles: degrees; yaw and continuous angle are clockwise positive.
- `GetRotation2d()` / `getRotation2d()`: counterclockwise positive, as in WPILib.
- Angular velocity: degrees/second. Acceleration: g (including gravity for raw).
- Velocity: m/s. Displacement/altitude: meters. Magnetometer: microteslas.
- Pressure: millibars. Temperature: Celsius. Sensor timestamp: milliseconds.
- The HAL currently requests 50 Hz. Constructors do not change the board-wide
  rate; inspect actual/requested rate methods. Individual reads are not an
  atomic snapshot of one sensor update.
- `ZeroYaw`/`Reset`/`ResetDisplacement` change the shared sensor state for all
  consumers. `UpdateDisplacement` is an explicit SDK manual-integration operation:
  pass finite acceleration within +/-16 g and an integer rate in 1..200 Hz.
- Destruction/Java `close()` releases only this view. It never calls the SDK's
  global `Stop()` and does not close SPI or stop other IMU consumers. Low-level
  SDK callbacks/register reads and `Stop()` are deliberately not public APIs.
- Before HAL initialization, a missing IMU, or on roboRIO/desktop, operations
  report an error (C++ runtime exception / Java HAL exception) instead of reading
  a null pointer or returning a successful zero heading. A disconnected or
  calibrating sensor remains readable through its status methods.

## Verification

Build HAL (including JNI) and wpilibc for linuxarm64 and wpilibj using Gradle.
The SDK headers and `libvmxpi_hal_cpp.so` are required only for the VMX build.

```bash
./gradlew :hal:linkHalReleaseSharedLibrary :hal:linkHalJNIReleaseSharedLibrary \
  :wpilibc:linkWpilibcReleaseSharedLibrary :wpilibj:jar \
  -Ponlylinuxarm64 -PskipJavaFormat
```

After building local Java jars, run the hardware-independent tests with a C++20
compiler and JDK17+ (`JAVA_HOME` must point to that JDK):

```bash
CXX=g++ python3 vmx/imu/tests/run_tests.py
```

These execute the production HAL bridge/C++ API under ASan/UBSan and the actual
Java API/JNI against a fake SDK. They cover pre-init/missing sensor errors,
disconnected/calibration state, axis/sign conventions, 64-bit timestamps, shared
reset behavior and view lifetime, invalid integration inputs, firmware buffers,
Java close handling, and explicit unsupported-platform errors. Board communication
and measured orientation still need a real VMX test.
