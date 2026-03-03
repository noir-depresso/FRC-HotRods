//ShooterSubsystem.cpp

#include "subsystems/ShooterSubsystem.h"

#include <algorithm>

#include <frc/MathUtil.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

namespace {
constexpr auto kEstimatedMaxFlywheelRpm = 5600.0;
constexpr auto kEstimatedMaxHoodAngle = units::degree_t{55.0};
constexpr int kRequiredStableCycles = 10;
}  // namespace

ShooterSubsystem::ShooterSubsystem()
    : m_drivingMotor1(DriveConstants::kShooterDriving1,
                      rev::spark::SparkMax::MotorType::kBrushless),
      m_drivingMotor2(DriveConstants::kShooterDriving2,
                      rev::spark::SparkMax::MotorType::kBrushless),
      m_turningMotor(DriveConstants::kShooterTurning,
                     rev::spark::SparkMax::MotorType::kBrushless) {
  rev::spark::SparkMaxConfig neo20config;

  neo20config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  neo20config.SmartCurrentLimit(40);
  neo20config.OpenLoopRampRate(0.10);
  neo20config.VoltageCompensation(12.0);

  m_drivingMotor1.Configure(neo20config, rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);

  m_drivingMotor2.Configure(neo20config, rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);

  m_turningMotor.Configure(neo20config, rev::ResetMode::kResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
}

void ShooterSubsystem::Periodic() {
  if (m_flywheelCommanded && m_hoodCommanded) {
    ++m_stableCycles;
  } else {
    m_stableCycles = 0;
  }
}

void ShooterSubsystem::SetPercent(double percent) {
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);

  m_drivingMotor1.Set(percent);
  m_drivingMotor2.Set(percent);
  m_flywheelCommanded = (std::abs(percent) > 1e-6);
}

void ShooterSubsystem::SetHoodPercent(double percent) {
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);

  m_turningMotor.Set(percent);
  m_hoodCommanded = (std::abs(percent) > 1e-6);
}

void ShooterSubsystem::SpinDrivingMotors() { SetPercent(+0.75); }

void ShooterSubsystem::StopDrivingMotors() {
  m_drivingMotor1.StopMotor();
  m_drivingMotor2.StopMotor();
  m_flywheelCommanded = false;
}

void ShooterSubsystem::StopHoodMotor() {
  m_turningMotor.StopMotor();
  m_hoodCommanded = false;
}

void ShooterSubsystem::StopAll() {
  StopDrivingMotors();
  StopHoodMotor();
}

void ShooterSubsystem::SetFlywheelRPM(units::revolutions_per_minute_t rpm) {
  m_targetFlywheelRpm = rpm;

  const double duty = std::clamp(rpm.value() / kEstimatedMaxFlywheelRpm, -1.0, 1.0);
  SetPercent(duty);
}

void ShooterSubsystem::SetHoodAngle(units::radian_t angle) {
  m_targetHoodAngle = angle;

  const double duty = std::clamp(
      (angle / kEstimatedMaxHoodAngle).value(), -1.0, 1.0);
  SetHoodPercent(duty);
}

bool ShooterSubsystem::AtFlywheelSetpoint() const {
  // Placeholder readiness behavior until encoder velocity feedback is wired.
  return m_flywheelCommanded;
}

bool ShooterSubsystem::AtHoodSetpoint() const {
  // Placeholder readiness behavior until hood angle feedback is wired.
  return m_hoodCommanded;
}

bool ShooterSubsystem::IsReadyToShoot(bool hasValidTarget) const {
  return hasValidTarget && AtFlywheelSetpoint() && AtHoodSetpoint() &&
         (m_stableCycles >= kRequiredStableCycles);
}
