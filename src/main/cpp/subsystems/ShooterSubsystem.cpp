//ShooterSubsystem.cpp

#include "subsystems/ShooterSubsystem.h"

#include <algorithm>
#include <cmath>

#include <frc/MathUtil.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

namespace {
// These values are used only for open-loop approximation.
// If they drift from real hardware behavior, aim consistency will suffer.
constexpr auto kEstimatedMaxFlywheelRpm = 5600.0;
constexpr auto kEstimatedMaxHoodAngle = units::degree_t{55.0};
constexpr int kRequiredStableCycles = 10;
constexpr double kFlywheelRpmTolerance = 150.0;
constexpr double kAimSettledPercent = 0.04;
constexpr double kMaxHoodPercent = 0.45;
constexpr double kMaxTurretPercent = 0.60;
}  // namespace

ShooterSubsystem::ShooterSubsystem()
    : m_drivingMotor1(DriveConstants::kShooterDriving1,
                      rev::spark::SparkMax::MotorType::kBrushless),
      m_drivingMotor2(DriveConstants::kShooterDriving2,
                      rev::spark::SparkMax::MotorType::kBrushless),
      m_turretMotor(DriveConstants::kShooterTurret,
                    rev::spark::SparkMax::MotorType::kBrushless),
      m_hoodMotor(DriveConstants::kShooterHood,
                  rev::spark::SparkMax::MotorType::kBrushless) {
  // Shared conservative config for all shooter motors.
  // Watch out: brake mode on hood/turret increases holding torque, but can raise current/heat.
  rev::spark::SparkMaxConfig neo20config;

  neo20config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  neo20config.SmartCurrentLimit(40);
  neo20config.OpenLoopRampRate(0.10);
  neo20config.VoltageCompensation(12.0);

  m_drivingMotor1.Configure(neo20config, rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);

  m_drivingMotor2.Configure(neo20config, rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);

  m_turretMotor.Configure(neo20config, rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
  m_hoodMotor.Configure(neo20config, rev::ResetMode::kResetSafeParameters,
                        rev::PersistMode::kPersistParameters);
}

void ShooterSubsystem::Periodic() {
  m_measuredFlywheelRpm =
      0.5 * (std::abs(m_drivingMotor1.GetEncoder().GetVelocity()) +
             std::abs(m_drivingMotor2.GetEncoder().GetVelocity()));

  if (AtFlywheelSetpoint() && AtHoodSetpoint() && AtTurretSetpoint()) {
    ++m_stableCycles;
  } else {
    m_stableCycles = 0;
  }
}

void ShooterSubsystem::SetPercent(double percent) {
  // Deadband removes joystick noise and prevents idle motor buzz.
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);

  m_drivingMotor1.Set(percent);
  m_drivingMotor2.Set(percent);
  m_flywheelCommanded = (std::abs(percent) > 0.02);
  m_targetFlywheelRpm =
      units::revolutions_per_minute_t{std::abs(percent) * kEstimatedMaxFlywheelRpm};
}

void ShooterSubsystem::SetHoodPercent(double percent) {
  // Hood changes shot trajectory arc (vertical aiming).
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -kMaxHoodPercent, kMaxHoodPercent);

  m_hoodMotor.Set(percent);
  m_lastHoodCmdPercent = percent;
  m_hoodCommanded = true;
}

void ShooterSubsystem::SetTurretPercent(double percent) {
  // Turret rotates the whole shooter + camera assembly (horizontal aiming).
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -kMaxTurretPercent, kMaxTurretPercent);

  m_turretMotor.Set(percent);
  m_lastTurretCmdPercent = percent;
  m_turretCommanded = true;
}

void ShooterSubsystem::SpinDrivingMotors() { SetPercent(+0.75); }

void ShooterSubsystem::StopDrivingMotors() {
  m_drivingMotor1.StopMotor();
  m_drivingMotor2.StopMotor();
  m_flywheelCommanded = false;
  m_targetFlywheelRpm = units::revolutions_per_minute_t{0.0};
}

void ShooterSubsystem::StopHoodMotor() {
  m_hoodMotor.StopMotor();
  m_lastHoodCmdPercent = 0.0;
  m_hoodCommanded = false;
}

void ShooterSubsystem::StopTurretMotor() {
  m_turretMotor.StopMotor();
  m_lastTurretCmdPercent = 0.0;
  m_turretCommanded = false;
}

void ShooterSubsystem::StopAll() {
  StopDrivingMotors();
  StopHoodMotor();
  StopTurretMotor();
}

void ShooterSubsystem::SetFlywheelRPM(units::revolutions_per_minute_t rpm) {
  const auto clampedRpm = units::revolutions_per_minute_t{
      std::clamp(rpm.value(), 0.0, kEstimatedMaxFlywheelRpm)};
  m_targetFlywheelRpm = clampedRpm;
  // Open-loop estimate only. Real closed-loop RPM control should use velocity feedback.
  const double duty = std::clamp(clampedRpm.value() / kEstimatedMaxFlywheelRpm, 0.0, 1.0);
  SetPercent(duty);
  m_targetFlywheelRpm = clampedRpm;
}

void ShooterSubsystem::SetHoodAngle(units::radian_t angle) {
  m_targetHoodAngle = angle;
  // Open-loop estimate only. Real hood angle control should use absolute angle feedback.
  const double duty =
      std::clamp((angle / kEstimatedMaxHoodAngle).value(), -1.0, 1.0);
  SetHoodPercent(duty);
}

bool ShooterSubsystem::AtFlywheelSetpoint() const {
  if (!m_flywheelCommanded) {
    return false;
  }
  return std::abs(m_measuredFlywheelRpm - m_targetFlywheelRpm.value()) <=
         kFlywheelRpmTolerance;
}

bool ShooterSubsystem::AtHoodSetpoint() const {
  return m_hoodCommanded &&
         (std::abs(m_lastHoodCmdPercent) <= kAimSettledPercent);
}

bool ShooterSubsystem::AtTurretSetpoint() const {
  return m_turretCommanded &&
         (std::abs(m_lastTurretCmdPercent) <= kAimSettledPercent);
}

bool ShooterSubsystem::IsReadyToShoot(bool hasValidTarget) const {
  return hasValidTarget && AtFlywheelSetpoint() && AtHoodSetpoint() &&
         AtTurretSetpoint() &&
         (m_stableCycles >= kRequiredStableCycles);
}
