// ShooterSubsystem.h

#pragma once

#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  explicit ShooterSubsystem();

  void Periodic() override;

  // Open-loop utility controls
  void SpinDrivingMotors();
  void StopDrivingMotors();
  void SetPercent(double percent);
  void SetTurnPercent(double percent);
  void StopTurningMotor();

 private:
  rev::spark::SparkMax m_drivingMotor1;
  rev::spark::SparkMax m_drivingMotor2;
  rev::spark::SparkMax m_turningMotor;
};
