# Studica Titan 1.1.8 API on WPILib 2026

C++ keeps `#include <studica/TitanQuad.h>` and the `studica` namespace.
Java keeps `com.studica.frc` and the original lower-camel-case methods.
The implementation is included in **this fork's wpilibc and wpilibj**, so Gradle
and CMake include the new sources, public headers and Java classes automatically.

## API compatibility

| Class | Implementation |
| --- | --- |
| TitanQuad | Titan CAN protocol through the existing WPILib HAL; one motor (0..3) per object |
| TitanQuadEncoder | Signed 32-bit counts, reset, distance scale, direction, RPM and sampled edge period |
| Cobra | MXP I2C ADS1015 at 0x48; 4 single-ended channels; bounded conversion timeout |
| Servo | 0..300 degrees using 500..2500 microsecond PWM |
| ServoContinuous | Continuous PWM servo, 1500 microsecond neutral |
| Lidar | Original Studica 360-degree / YDLIDAR triangle packets, 128000 baud, 360 ray arrays, mm distance |

C++ and Java `SpeedController` source adapters extend the 2026 MotorController
interface. The old SendableBuilder package/header is adapted to the 2026 interface.
`TitanQuad` works with modern drive classes as well as legacy
SpeedController references. `PIDWrite()` / `pidWrite()` now apply the output
instead of silently ignoring it.

C++ `TitanQuadEncoder` keeps its original double-valued `GetPeriod()` and
`SetMaxPeriod(double)`. It no longer privately inherits CounterBase, whose 2026
C++ interface uses units::second_t. Java still implements CounterBase. These
are **source-compatible public classes**, not binary replacements for the old
2020 library or its `com.studica.frc.jni` implementation classes. Recompile the
project and remove the old Titan vendor dependency to avoid duplicate classes.

## Example

```cpp
#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>

studica::TitanQuad motor{42, 15600, 0};
studica::TitanQuadEncoder encoder{motor, 0, 0.001};
// In enabled periodic code (refresh within 100 ms):
// motor.Set(0.25);
// auto distance = encoder.GetEncoderDistance();
```

```java
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

TitanQuad motor = new TitanQuad(42, 15600, 0);
TitanQuadEncoder encoder = new TitanQuadEncoder(motor, 0, 0.001);
// In enabled periodic code: motor.set(0.25);
// On shutdown: motor.close();
```

The original constructor overloads, methods, inversion controls and per-channel
limit switch calls are retained. RPM telemetry is little-endian int32 / 100,
including negative values. `GetSpeed()` / `getSpeed()` report shaft RPM (not
linear distance per second). Distance uses signed encoder ticks × distPerTick.
Periods are inferred from observed count changes; they are not hardware-captured
edge times. Missing/truncated telemetry throws; it is not reported as valid zero.
Java rejects frames older than 250 ms; native reads reject an unchanged timestamp
after 250 ms of observation.

## Motor lifetime and disable behavior

A CAN ID owns a shared board object for all four motor channels. Two objects
cannot allocate the same channel. Creating or destroying one channel does not
close the CAN transport for other channels. Native encoders hold the shared
transport alive; Java encoders keep their TitanQuad reference and reject reads
after that motor is explicitly closed.

Construction starts disabled. A Driver Station must be attached, enabled and
not E-stopped before a Set call is accepted. A 20 ms board service sends only
commands refreshed within 100 ms. On DS disable/disconnect, E-stop, expiry, or
CAN write failure, commands are cleared and the affected outputs are stopped.
Re-enabling cannot replay an old nonzero command. CAN write failures latch a
fault until the board objects are recreated. No unconditional repeating motor
CAN packets or second VMXPi instance are created. The controller's own CAN
watchdog remains an additional protection if the process or worker stops.

## LiDAR

`Lidar::kUSB1` / `Lidar.Port.kUSB1` is the original upper USB2 serial port;
`kUSB2` is the lower one. Existing C++ `GetData()` fills the public `angle[360]`
and `distance[360]`; Java `getData(float[], float[])` fills the caller's arrays.
Missing rays are NaN. Reads validate packet length, angle check bits and XOR
checksum and stop waiting after one second. Serial DTR controls the original
USB adapter's motor; custom Linux baud configuration supports 128000 baud.

The default is the **original non-intensity, triangulation Studica 360-degree
model** from the 1.1.8 generation. The newer T-Mini/ToF/intensity packet formats
are not interchangeable. A two-argument constructor allows a different baud
rate for the same packet format; it does not change the sensor protocol.

## Build and distribution

Build this fork's `wpilibc`, `wpilibj`, HAL and HAL JNI together. Do not mix the
new SerialPort Java code with an older HAL JNI, which lacks serialSetDTR.

```sh
./gradlew :wpilibc:publishToMavenLocal :wpilibj:publishToMavenLocal :hal:publishToMavenLocal \
  -Ponlylinuxarm64 -PpublishVersion=2026.2.2-vmx.5 -PskipJavaFormat \
  -Dmaven.repo.local="$HOME/wpilib/2026/vmx/maven"
```

This builds artifacts locally. The matching published package is allwpilib
`2026.2.2-vmx.5`; install it with VMX2026 extension 2.0.6 or later.
A robot project must resolve the matching custom **wpilibc-cpp / wpilibj-java**
and HAL artifacts, instead of the upstream 2026.2.2 WPILib artifacts. VMX2026 extension 2.0.6 recognizes the release manifest
`robotLibraries` capability and selects the matching custom wpilibc/wpilibj for
VMX targets. Run `Update VMX2026 Version` after installing the extension.
Remove the old Titan vendor dependency. RoboRIO targets keep upstream robot
libraries; older extension versions do not install the added Studica APIs.

For manual local builds without extension-managed dependencies, a project using
the VMX GradleRIO fork can select the local artifacts explicitly in `build.gradle`:

```groovy
def studicaVmxVersion = '2026.2.2-vmx.5' // matching custom build or release version
wpi.versions.vmxHalVersion.set(studicaVmxVersion)
configurations.configureEach {
    resolutionStrategy.eachDependency { dependency ->
        if ((dependency.requested.group == 'edu.wpi.first.wpilibc' &&
             dependency.requested.name == 'wpilibc-cpp') ||
            (dependency.requested.group == 'edu.wpi.first.wpilibj' &&
             dependency.requested.name == 'wpilibj-java')) {
            dependency.useVersion(studicaVmxVersion)
        }
    }
}
```

The example publication builds robot-side `linuxarm64` native artifacts. Desktop
simulation needs the matching native artifacts built for the desktop platform
as well; do not substitute an upstream library without the added Studica symbols.

## Tests

```sh
CXX=c++ python3 vmx/studica/tests/run_tests.py
```

The runner uses the actual WPILib 2026 headers/jars and fake CAN/I2C transports.
It checks frame encoding, shared channels, DS/E-stop/timeout behavior, fault
latching, negative encoder/RPM data, ADC scaling, malformed/fragmented LiDAR
frames, and real Linux termios2 configuration through a pseudo-terminal.
C++ runs under ASan/UBSan. Set `--java-classpath` if the WPILib jars are not in
the local Gradle cache. Physical motor behavior, old/new firmware differences,
USB adapter DTR polarity, LiDAR model settings and sensors still require tests
on the actual robot.

Protocol reference and attribution: [NOTICE.md](NOTICE.md).
