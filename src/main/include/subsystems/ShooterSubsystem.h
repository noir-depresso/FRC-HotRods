// ShooterSubsystem.h

#pragma once

#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>

class ShooterSubsystem : public frc2::SubsystemBase {

 public:
  explicit ShooterSubsystem();

  //void In();
  //void Out();
  void SpinDrivingMotors();
  //void SpinTurningMotor();
  void StopDrivingMotors();
  //void StopTurningMotor();
  void SetPercent(double percent);
  void SetTurnPercent(double percent);
  void StopTurningMotor();

 private:
  rev::spark::SparkMax m_drivingMotor1;
  rev::spark::SparkMax m_drivingMotor2;
  rev::spark::SparkMax m_turningMotor;
};
