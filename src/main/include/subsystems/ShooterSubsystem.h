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

  // Open-loop flywheel/aim controls.
  // NOTE: These are percent outputs, not closed-loop position/velocity controls.
  void SpinDrivingMotors();
  void StopDrivingMotors();
  void StopAll();
  void SetPercent(double percent);
  // Hood controls vertical launch angle.
  void SetHoodPercent(double percent);
  // Turret controls horizontal yaw (rotating plate under shooter + Limelight).
  void SetTurretPercent(double percent);
  void StopHoodMotor();
  void StopTurretMotor();

  // Convenience APIs that map desired setpoints into open-loop duty estimates.
  // Watch out: these are placeholders until real encoder feedback is integrated.
  void SetFlywheelRPM(units::revolutions_per_minute_t rpm);
  void SetHoodAngle(units::radian_t angle);
  bool AtFlywheelSetpoint() const;
  bool AtHoodSetpoint() const;
  bool AtTurretSetpoint() const;
  bool IsReadyToShoot(bool hasValidTarget) const;

 private:
  // Flywheel pair
  rev::spark::SparkMax m_drivingMotor1;
  rev::spark::SparkMax m_drivingMotor2;
  // Independent aim axes
  rev::spark::SparkMax m_turretMotor;
  rev::spark::SparkMax m_hoodMotor;
  bool m_flywheelCommanded = false;
  bool m_hoodCommanded = false;
  bool m_turretCommanded = false;
  double m_lastHoodCmdPercent = 0.0;
  double m_lastTurretCmdPercent = 0.0;
  double m_measuredFlywheelRpm = 0.0;
  int m_stableCycles = 0;
  units::revolutions_per_minute_t m_targetFlywheelRpm{0.0};
  units::radian_t m_targetHoodAngle{0.0};
};
