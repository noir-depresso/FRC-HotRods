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
  void StopAll();
  void SetPercent(double percent);
  void SetHoodPercent(double percent);
  void SetTurretPercent(double percent);
  void StopHoodMotor();
  void StopTurretMotor();

  void SetFlywheelRPM(units::revolutions_per_minute_t rpm);
  void SetHoodAngle(units::radian_t angle);
  bool AtFlywheelSetpoint() const;
  bool AtHoodSetpoint() const;
  bool AtTurretSetpoint() const;
  bool IsReadyToShoot(bool hasValidTarget) const;

 private:
  rev::spark::SparkMax m_drivingMotor1;
  rev::spark::SparkMax m_drivingMotor2;
  rev::spark::SparkMax m_turretMotor;
  rev::spark::SparkMax m_hoodMotor;
  bool m_flywheelCommanded = false;
  bool m_hoodCommanded = false;
  bool m_turretCommanded = false;
  int m_stableCycles = 0;
  units::revolutions_per_minute_t m_targetFlywheelRpm{0.0};
  units::radian_t m_targetHoodAngle{0.0};
};
