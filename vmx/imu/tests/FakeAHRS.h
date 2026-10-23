// Minimal SDK transport fake. The production ARM64 build uses the real VMX SDK.
#pragma once
#include <map>
#include <string>

enum BoardAxis { kBoardAxisX, kBoardAxisY, kBoardAxisZ };
struct BoardYawAxis {
  BoardAxis board_axis;
  bool up;
};
namespace vmx {
class AHRS {
 public:
  std::map<std::string, double> values;
  int zeroYawCalls = 0;
  int resetCalls = 0;
  int resetDisplacementCalls = 0;
  int updateCalls = 0;
  int stops = 0;
  double lastX = 0, lastY = 0;
  int lastRate = 0;
  bool lastMoving = false;
  BoardYawAxis axis{kBoardAxisZ, true};
  std::string firmware = "3.0.436";
  void ZeroYaw() {
    ++zeroYawCalls;
    values["GetYaw"] = 0;
  }
  void Reset() {
    ++resetCalls;
    values["GetAngle"] = 0;
  }
  void ResetDisplacement() { ++resetDisplacementCalls; }
  void UpdateDisplacement(float x, float y, int rate, bool moving) {
    ++updateCalls;
    lastX = x;
    lastY = y;
    lastRate = rate;
    lastMoving = moving;
  }
  void Stop() { ++stops; }
  BoardYawAxis GetBoardYawAxis() { return axis; }
  std::string GetFirmwareVersion() { return firmware; }
  double GetPitch() { return static_cast<double>(values["GetPitch"]); }
  double GetYaw() { return static_cast<double>(values["GetYaw"]); }
  double GetRoll() { return static_cast<double>(values["GetRoll"]); }
  double GetCompassHeading() {
    return static_cast<double>(values["GetCompassHeading"]);
  }
  bool IsCalibrating() { return static_cast<bool>(values["IsCalibrating"]); }
  bool IsConnected() { return static_cast<bool>(values["IsConnected"]); }
  double GetByteCount() { return static_cast<double>(values["GetByteCount"]); }
  double GetUpdateCount() {
    return static_cast<double>(values["GetUpdateCount"]);
  }
  long GetLastSensorTimestamp() {
    return static_cast<long>(values["GetLastSensorTimestamp"]);
  }
  double GetWorldLinearAccelX() {
    return static_cast<double>(values["GetWorldLinearAccelX"]);
  }
  double GetWorldLinearAccelY() {
    return static_cast<double>(values["GetWorldLinearAccelY"]);
  }
  double GetWorldLinearAccelZ() {
    return static_cast<double>(values["GetWorldLinearAccelZ"]);
  }
  bool IsMoving() { return static_cast<bool>(values["IsMoving"]); }
  bool IsRotating() { return static_cast<bool>(values["IsRotating"]); }
  double GetBarometricPressure() {
    return static_cast<double>(values["GetBarometricPressure"]);
  }
  double GetAltitude() { return static_cast<double>(values["GetAltitude"]); }
  bool IsAltitudeValid() {
    return static_cast<bool>(values["IsAltitudeValid"]);
  }
  double GetFusedHeading() {
    return static_cast<double>(values["GetFusedHeading"]);
  }
  bool IsMagneticDisturbance() {
    return static_cast<bool>(values["IsMagneticDisturbance"]);
  }
  bool IsMagnetometerCalibrated() {
    return static_cast<bool>(values["IsMagnetometerCalibrated"]);
  }
  double GetQuaternionW() {
    return static_cast<double>(values["GetQuaternionW"]);
  }
  double GetQuaternionX() {
    return static_cast<double>(values["GetQuaternionX"]);
  }
  double GetQuaternionY() {
    return static_cast<double>(values["GetQuaternionY"]);
  }
  double GetQuaternionZ() {
    return static_cast<double>(values["GetQuaternionZ"]);
  }
  double GetVelocityX() { return static_cast<double>(values["GetVelocityX"]); }
  double GetVelocityY() { return static_cast<double>(values["GetVelocityY"]); }
  double GetVelocityZ() { return static_cast<double>(values["GetVelocityZ"]); }
  double GetDisplacementX() {
    return static_cast<double>(values["GetDisplacementX"]);
  }
  double GetDisplacementY() {
    return static_cast<double>(values["GetDisplacementY"]);
  }
  double GetDisplacementZ() {
    return static_cast<double>(values["GetDisplacementZ"]);
  }
  double GetAngle() { return static_cast<double>(values["GetAngle"]); }
  double GetRate() { return static_cast<double>(values["GetRate"]); }
  double GetRawGyroX() { return static_cast<double>(values["GetRawGyroX"]); }
  double GetRawGyroY() { return static_cast<double>(values["GetRawGyroY"]); }
  double GetRawGyroZ() { return static_cast<double>(values["GetRawGyroZ"]); }
  double GetRawAccelX() { return static_cast<double>(values["GetRawAccelX"]); }
  double GetRawAccelY() { return static_cast<double>(values["GetRawAccelY"]); }
  double GetRawAccelZ() { return static_cast<double>(values["GetRawAccelZ"]); }
  double GetRawMagX() { return static_cast<double>(values["GetRawMagX"]); }
  double GetRawMagY() { return static_cast<double>(values["GetRawMagY"]); }
  double GetRawMagZ() { return static_cast<double>(values["GetRawMagZ"]); }
  double GetPressure() { return static_cast<double>(values["GetPressure"]); }
  double GetTempC() { return static_cast<double>(values["GetTempC"]); }
  int GetActualUpdateRate() {
    return static_cast<int>(values["GetActualUpdateRate"]);
  }
  int GetRequestedUpdateRate() {
    return static_cast<int>(values["GetRequestedUpdateRate"]);
  }
};
}  // namespace vmx
