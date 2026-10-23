# VMX-pi HAL for WPILib 2026

This branch adds a HAL backend for the Studica / Kauai Labs VMX-pi, so WPILib 2026
robot programs (C++ and Java) run on a VMX-pi with a 64-bit (aarch64) OS.

- Backend: `hal/src/main/native/mau`, the 2020 VMX-pi HAL (`2020.3.2-vmxpi-20`)
  ported to the 2026 HAL API. It is used for the `linuxarm64` platform only; every
  other platform keeps the upstream `athena` / `sim` backends.
- It talks to the VMX through Studica's VMX HAL library (`libvmxpi_hal_cpp.so`,
  headers `VMXPi.h` ...). That library is **not** part of this repository and is
  not redistributed: the robot gets it from its `vmxpi-hal` package, and the build
  reads it from `vmxpi-hal/` (gitignored), see below.

## What changed compared to the 2020 port

Most files only follow 2026 signature changes (`allocationLocation`, `Free`/`Clean`
without `status`, handle allocation, `std::string_view`). The larger changes:

| Area | Change |
|---|---|
| Driver Station | The DS protocol validates packet boundaries and joystick limits; TCP frames are reassembled per connection. The HAL side follows the 2023+ model of `athena/FRCDriverStation.cpp`: `HAL_RefreshDSData()` snapshots, new-data events. The DS threads start in `HAL_Initialize()` (the 2023 test builds never started them). |
| IO watchdog | Fed / expired on every DS control packet (it was in `HAL_WaitForDSData()`, which no longer exists). Outputs stay off unless the DS is attached and enabled. |
| Interrupts | Synchronous waits only (the 2026 API); edges can be changed after `HAL_RequestInterrupts()`. |
| PWM | 2026 microsecond API. One VMX duty-cycle tick is 1 us (200 Hz, 5000 ticks), so values map directly. |
| PCM / PDP / REV | The 2020 PCM/PDP code is replaced by `CTREPCM`, `CTREPDP`, `PowerDistribution`, `REVPH`, `REVPDH` copied from `athena` (they only use the CAN API). |
| DS restart / E-Stop | End the program; robot_manager restarts it (the 2020 image used `/usr/local/frc/bin/frcKillRobot.sh`). |
| New 2026 functions | Color order, SPI mode, CPU temperature, shutdown callbacks...; features the VMX does not have return an error or are no-ops (DMA, DutyCycle, radio LED, user rails). |

`HAL_GetAnalogLSBWeight()` reports nanovolts per 12-bit ADC count. Automatic
SPI transfers and averaged/filtered analog triggers are not supported by the
current VMX SDK; requesting them returns a HAL error instead of reporting success
with incorrect data. Manual SPI transfers and raw analog triggers are supported.

## Building

Requirements: the WPILib arm64 cross compiler (`./gradlew installArm64Toolchain`) and a
JDK that runs Gradle 8.14 (17 or 21).

1. Put Studica's aarch64 VMX HAL here (from the `vmxpi-hal` package or the robot's
   `/usr/local/include/vmxpi` and `/usr/local/lib/vmxpi`):

   ```
   vmxpi-hal/include/*.h
   vmxpi-hal/lib/libvmxpi_hal_cpp.so
   ```

   Another location: `-PvmxpiHalDir=/path`.

2. Build the VMX HAL Maven repository used by the VMX GradleRIO fork:

   ```bash
   python3 vmx/build_vmx_hal_maven.py --vmx-version 2026.2.2-vmx.6 --with-robot-libraries
   ```

   It builds `:hal` for linuxarm64 and writes `edu.wpi.first.hal:hal-cpp` / `hal-java`
   at that version into `~/wpilib/2026/vmx/maven`, with the other platforms taken from
   the upstream WPILib release (2026.2.2). `--with-robot-libraries` also builds
   custom wpilibc (linuxarm64) and wpilibj with the Studica APIs at the same
   version. The VMX HAL also retains WPILib HALSIM callbacks used by wpilibc.
   `libvmxpi_hal_cpp.so` is not copied.

Just the library: `./gradlew :hal:linkHalReleaseSharedLibrary -Ponlylinuxarm64`.

## Robot

- 64-bit OS with glibc 2.36 or newer (WPILib 2026 linuxarm64 binaries): Ubuntu 24.04 /
  26.04 or Debian 12. Ubuntu 22.04 is too old.
- Studica's VMX HAL installed (`/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so` on the
  library path).
- The program runs as root (pigpio). Deploy and supervision: see `VMX.md` in the VMX
  GradleRIO fork.

## Host regression tests

With a C++20 GCC/Clang compiler and the VMX SDK headers in `vmxpi-hal/include`:

```bash
CXX=g++ python3 vmx/tests/run_tests.py
```

`--sdk-include /path/to/include` selects other SDK headers. The tests compile the
actual VMX HAL code with ASan/UBSan and replace SDK transport calls with fakes;
no SDK binaries, Gradle, network or robot are needed. They cover malformed and
fragmented DS packets, joystick input, I2C shared bus lifetime, SPI polarity,
PWM scaling, ADC conversion and errors for unsupported features. Physical sensor,
watchdog and motor behavior still requires robot testing.

I2C device objects share the VMX bus. Address-only probes use a zero-length Linux
I2C transfer on `/dev/i2c-1` (the bus used by the SDK); an SDK empty transaction
would report success without sending an address. Bus/adapter errors return a
failed transaction, including when the adapter cannot perform zero-length probes.

## Studica 1.1.8 compatibility APIs

The in-tree wpilibc and wpilibj include the original `studica::TitanQuad` /
`com.studica.frc.TitanQuad` APIs and their encoder, Cobra, servo and LiDAR
classes. See [Studica integration](studica/README.md) for build requirements,
source compatibility, motor disable behavior, sensor models and host tests.
