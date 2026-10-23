// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see LICENSE.md.
#include <studica/TitanQuad.h>
#include <studica/TitanQuadEncoder.h>
#include <studica/Cobra.h>
#include <studica/Servo.h>
#include <studica/ServoContinuous.h>
#include <studica/Lidar.h>
void CompileLegacyStudicaApi(studica::TitanQuad& motor,
                             studica::TitanQuadEncoder& encoder,
                             studica::Cobra& cobra, studica::Servo& servo,
                             studica::ServoContinuous& continuous,
                             studica::Lidar& lidar) {
  frc::SpeedController& legacy = motor;
  frc::MotorController& modern = motor;
  legacy.PIDWrite(0.2);
  modern.Set(0.2);
  motor.SetInverted(true);
  (void)motor.GetInverted();
  motor.Disable();
  motor.StopMotor();
  (void)motor.GetID();
  (void)motor.GetFrequency();
  (void)motor.GetFirmwareVersion();
  (void)motor.GetHardwareVersion();
  (void)motor.GetSerialNumber();
  (void)motor.GetControllerTemp();
  (void)motor.GetLimitSwitch(true);
  motor.InvertRPM();
  (void)motor.GetRPM();
  (void)motor.Get();
  (void)encoder.Get();
  encoder.Reset();
  double period = encoder.GetPeriod();
  encoder.SetMaxPeriod(period);
  (void)encoder.GetStopped();
  (void)encoder.GetDirection();
  (void)encoder.GetEncoderDistance();
  (void)encoder.GetRaw();
  encoder.InvertSpeed();
  (void)encoder.GetSpeed();
  encoder.SetReverseDirection();
  (void)cobra.GetRawValue(0);
  (void)cobra.GetVoltage(0);
  servo.Set(0.5);
  servo.SetAngle(150);
  (void)servo.Get();
  (void)servo.GetAngle();
  (void)servo.GetMaxAngle();
  (void)servo.GetMinAngle();
  servo.SetOffline();
  continuous.Set(0.1);
  continuous.StopMotor();
  lidar.GetData();
  (void)lidar.angle[0];
  (void)lidar.distance[0];
}
