//ShooterSubsystem.cpp

#include "subsystems/ShooterSubsystem.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/math.h>

#include "Constants.h"

using namespace units::literals;

namespace {
// These values are used only for open-loop approximation.
// If they drift from real hardware behavior, aim consistency will suffer.
constexpr auto kEstimatedMaxFlywheelRpm = 5600.0;
constexpr auto kHoodMinAngle = units::degree_t{0.0};
constexpr auto kHoodMaxAngle = units::degree_t{32.0};
constexpr auto kInitialHoodAngle = units::degree_t{8.0};
constexpr auto kFlywheelTolerance = 120_rpm;
constexpr auto kHoodTolerance = units::degree_t{1.0};
constexpr int kRequiredStableCycles = 10;
constexpr int kRequiredFlywheelCycles = 5;

constexpr double kFlywheelP = 0.00022;
constexpr double kFlywheelI = 0.0;
constexpr double kFlywheelD = 0.0;
constexpr double kFlywheelKv = 1.0 / kEstimatedMaxFlywheelRpm;

constexpr double kHoodP = 2.8;
constexpr double kHoodI = 0.0;
constexpr double kHoodD = 0.0;

constexpr double kTurretP = 0.18;
constexpr double kTurretI = 0.0;
constexpr double kTurretD = 0.0;
constexpr double kTurretToleranceMotorRot = 0.08;
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
 rev::spark::SparkMaxConfig sharedConfig;
  sharedConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  sharedConfig.SmartCurrentLimit(40);
  sharedConfig.OpenLoopRampRate(0.10);
  sharedConfig.VoltageCompensation(12.0);

  rev::spark::SparkMaxConfig flywheelConfig;
  flywheelConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  flywheelConfig.SmartCurrentLimit(40);
  flywheelConfig.OpenLoopRampRate(0.10);
  flywheelConfig.VoltageCompensation(12.0);
  flywheelConfig.encoder.VelocityConversionFactor(1.0);  // RPM
  flywheelConfig.closedLoop
      .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
      .Pid(kFlywheelP, kFlywheelI, kFlywheelD)
      .OutputRange(-1.0, 1.0)
      .feedForward.kV(kFlywheelKv);

  rev::spark::SparkMaxConfig hoodConfig;
  hoodConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  hoodConfig.SmartCurrentLimit(40);
  hoodConfig.OpenLoopRampRate(0.10);
  hoodConfig.VoltageCompensation(12.0);
  hoodConfig.absoluteEncoder
      .PositionConversionFactor(2.0 * std::numbers::pi)         // radians
      .VelocityConversionFactor((2.0 * std::numbers::pi) / 60.0)  // rad/s
      .Inverted(false);
  hoodConfig.closedLoop
      .SetFeedbackSensor(rev::spark::FeedbackSensor::kAbsoluteEncoder)
      .Pid(kHoodP, kHoodI, kHoodD)
      .OutputRange(-0.45, 0.45);

  m_drivingMotor1.Configure(flywheelConfig, rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);

  m_drivingMotor2.Configure(flywheelConfig, rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);

  rev::spark::SparkMaxConfig turretConfig;
  turretConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  turretConfig.SmartCurrentLimit(40);
  turretConfig.OpenLoopRampRate(0.10);
  turretConfig.VoltageCompensation(12.0);
  turretConfig.closedLoop
      .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
      .Pid(kTurretP, kTurretI, kTurretD)
      .OutputRange(-0.6, 0.6);

  m_turretMotor.Configure(turretConfig, rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);

  m_hoodMotor.Configure(hoodConfig, rev::ResetMode::kResetSafeParameters,
                        rev::PersistMode::kPersistParameters);
}

void ShooterSubsystem::Periodic() {

  if (m_flywheelCommanded && m_hoodCommanded && m_turretCommanded) {
    ++m_stableCycles;
  } else {
    m_stableCycles = 0;
  }
  
  if (AtFlywheelSetpoint()) {
    ++m_flywheelAtSetpointCycles;
  } else {
    m_flywheelAtSetpointCycles = 0;
  }

  const units::radian_t actualHoodAngle{m_hoodAbsoluteEncoder.GetPosition()};
  const units::revolutions_per_minute_t actualFlywheelRpm{m_flywheelEncoder1.GetVelocity()};
  frc::SmartDashboard::PutNumber("Shooter/HoodTargetDeg",
                                 units::degree_t{m_targetHoodAngle}.value());
  frc::SmartDashboard::PutNumber("Shooter/HoodActualDeg",
                                 units::degree_t{actualHoodAngle}.value());
  frc::SmartDashboard::PutNumber(
      "Shooter/HoodErrorDeg",
      units::degree_t{m_targetHoodAngle - actualHoodAngle}.value());
  frc::SmartDashboard::PutNumber("Shooter/FlywheelTargetRpm",
                                 m_targetFlywheelRpm.value());
  frc::SmartDashboard::PutNumber("Shooter/FlywheelActualRpm", actualFlywheelRpm.value());
  frc::SmartDashboard::PutNumber("Shooter/TurretTargetMotorRot",
                                 m_targetTurretMotorRot);
  frc::SmartDashboard::PutNumber("Shooter/TurretActualMotorRot",
                                 m_turretEncoder.GetPosition());
}

void ShooterSubsystem::SetPercent(double percent) {
  // Deadband removes joystick noise and prevents idle motor buzz.
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);

  m_drivingMotor1.Set(percent);
  m_drivingMotor2.Set(percent);
    m_flywheelClosedLoopActive = false;
  m_flywheelCommanded = (std::abs(percent) > 1e-6);
}

void ShooterSubsystem::SetHoodPercent(double percent) {
  // Hood changes shot trajectory arc (vertical aiming).
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);

 const units::radian_t hoodAngle{m_hoodAbsoluteEncoder.GetPosition()};
  if ((hoodAngle <= kHoodMinAngle && percent < 0.0) ||
      (hoodAngle >= kHoodMaxAngle && percent > 0.0)) {
    percent = 0.0;
  }

  m_hoodMotor.Set(percent);
  m_hoodClosedLoopActive = false;
  m_hoodCommanded = (std::abs(percent) > 1e-6);
}

void ShooterSubsystem::SetTurretPercent(double percent) {
  // Turret rotates the whole shooter + camera assembly (horizontal aiming).
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);

  m_turretMotor.Set(percent);
    m_turretClosedLoopActive = false;
  m_turretCommanded = (std::abs(percent) > 1e-6);
}

void ShooterSubsystem::SetTurretAngleMotorRot(double motorRot) {
  m_targetTurretMotorRot = motorRot;
  m_turretClosedLoopActive = true;
  m_turretController.SetSetpoint(m_targetTurretMotorRot,
                                 rev::spark::SparkMax::ControlType::kPosition);
  m_turretCommanded = true;
}

void ShooterSubsystem::NudgeTurretAngleMotorRot(double deltaMotorRot) {
  SetTurretAngleMotorRot(m_turretEncoder.GetPosition() + deltaMotorRot);
}

void ShooterSubsystem::ZeroTurretEncoder() {
  m_turretEncoder.SetPosition(0.0);
  m_targetTurretMotorRot = 0.0;
}

void ShooterSubsystem::SpinDrivingMotors() { SetPercent(+0.75); }

void ShooterSubsystem::StopDrivingMotors() {
  m_drivingMotor1.StopMotor();
  m_drivingMotor2.StopMotor();
    m_flywheelClosedLoopActive = false;
  m_flywheelCommanded = false;
}

void ShooterSubsystem::StopHoodMotor() {
  m_hoodMotor.StopMotor();
    m_hoodClosedLoopActive = false;
  m_hoodCommanded = false;
}

void ShooterSubsystem::StopTurretMotor() {
  m_turretMotor.StopMotor();
  m_turretClosedLoopActive = false;
  m_turretCommanded = false;
}

void ShooterSubsystem::StopAll() {
  StopDrivingMotors();
  StopHoodMotor();
  StopTurretMotor();
}

void ShooterSubsystem::SetFlywheelRPM(units::revolutions_per_minute_t rpm) {
  m_targetFlywheelRpm = rpm;
  m_flywheelClosedLoopActive = true;
  m_flywheelController1.SetSetpoint(rpm.value(), rev::spark::SparkMax::ControlType::kVelocity);
  m_flywheelController2.SetSetpoint(rpm.value(), rev::spark::SparkMax::ControlType::kVelocity);
  m_flywheelCommanded = true;
}

void ShooterSubsystem::SetHoodAngle(units::radian_t angle) {
  m_targetHoodAngle = std::clamp(angle, units::radian_t{kHoodMinAngle},
                                 units::radian_t{kHoodMaxAngle});
  m_hoodClosedLoopActive = true;
  m_hoodController.SetSetpoint(m_targetHoodAngle.value(),
                               rev::spark::SparkMax::ControlType::kPosition);
  m_hoodCommanded = true;
}

bool ShooterSubsystem::AtFlywheelSetpoint() const {
  if (!m_flywheelClosedLoopActive) {
    return m_flywheelCommanded;
  }

  const units::revolutions_per_minute_t actualRpm{m_flywheelEncoder1.GetVelocity()};
  return units::math::abs(actualRpm - m_targetFlywheelRpm) <= kFlywheelTolerance;
}

bool ShooterSubsystem::AtHoodSetpoint() const {
  if (!m_hoodClosedLoopActive) {
    return m_hoodCommanded;
  }

  const units::radian_t actualAngle{m_hoodAbsoluteEncoder.GetPosition()};
  return units::math::abs(actualAngle - m_targetHoodAngle) <= kHoodTolerance;
}

bool ShooterSubsystem::AtTurretSetpoint() const {
  if (!m_turretClosedLoopActive) {
    return m_turretCommanded;
  }

  return std::abs(m_turretEncoder.GetPosition() - m_targetTurretMotorRot) <=
         kTurretToleranceMotorRot;
}

bool ShooterSubsystem::IsReadyToShoot(bool hasValidTarget) const {
  return hasValidTarget && AtFlywheelSetpoint() && AtHoodSetpoint() &&
         AtTurretSetpoint() &&
         (m_stableCycles >= kRequiredStableCycles) &&
         (m_flywheelAtSetpointCycles >= kRequiredFlywheelCycles);
}

void ShooterSubsystem::MoveHoodToInitialAngle() {
  SetHoodAngle(kInitialHoodAngle);
}
