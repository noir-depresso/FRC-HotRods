// ShooterSubsystem.h

#pragma once

#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
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
  // Turret closed-loop helpers in turret output angle units.
  void SetTurretAngle(units::radian_t angle);
  void NudgeTurretAngle(units::radian_t deltaAngle);
  units::radian_t GetTurretAngle() const;
  void ZeroTurretEncoder();
  void StopHoodMotor();
  void StopTurretMotor();

  // Closed-loop shooter setpoint APIs.
  void SetFlywheelRPM(units::revolutions_per_minute_t rpm);
  // Hood angle convention: 0 deg is vertical-up, positive tilts forward.
  void SetHoodAngle(units::radian_t angle);
  bool AtFlywheelSetpoint() const;
  bool AtHoodSetpoint() const;
  bool AtTurretSetpoint() const;
  bool IsReadyToShoot(bool hasValidTarget) const;

  // One-press helper used by button 2 for parking/initial hood angle.
  void MoveHoodToInitialAngle();

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
  rev::spark::SparkRelativeEncoder m_flywheelEncoder1 = m_drivingMotor1.GetEncoder();
  rev::spark::SparkRelativeEncoder m_flywheelEncoder2 = m_drivingMotor2.GetEncoder();
  rev::spark::SparkRelativeEncoder m_turretEncoder = m_turretMotor.GetEncoder();
  rev::spark::SparkAbsoluteEncoder m_hoodAbsoluteEncoder =
      m_hoodMotor.GetAbsoluteEncoder();
  rev::spark::SparkClosedLoopController m_flywheelController1 =
      m_drivingMotor1.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_flywheelController2 =
      m_drivingMotor2.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_turretController =
      m_turretMotor.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_hoodController =
      m_hoodMotor.GetClosedLoopController();
  units::revolutions_per_minute_t m_targetFlywheelRpm{0.0};
  units::radian_t m_targetHoodAngle{0.0};
  bool m_flywheelClosedLoopActive = false;
  bool m_hoodClosedLoopActive = false;
  bool m_turretClosedLoopActive = false;
  int m_flywheelAtSetpointCycles = 0;
  units::radian_t m_targetTurretAngle{0.0};
};
