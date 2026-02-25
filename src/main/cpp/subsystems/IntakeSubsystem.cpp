#include "subsystems/IntakeSubsystem.h"

#include <algorithm>
#include <frc/MathUtil.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

IntakeSubsystem::IntakeSubsystem()
    : m_motor(IntakeConstants::kMotorCanId,
              rev::spark::SparkMax::MotorType::kBrushless) {
  rev::spark::SparkMaxConfig intakeConfig;
  intakeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  intakeConfig.SmartCurrentLimit(IntakeConstants::kCurrentLimitAmps);
  intakeConfig.OpenLoopRampRate(IntakeConstants::kOpenLoopRampSeconds);
  intakeConfig.VoltageCompensation(12.0);

  m_motor.Configure(
    intakeConfig,
    rev::ResetMode::kResetSafeParameters,
    rev::PersistMode::kPersistParameters
  );
}

void IntakeSubsystem::SetPercent(double percent) {
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);
  m_motor.Set(percent);
}

void IntakeSubsystem::In() {
  SetPercent(IntakeConstants::kInPercent);
}

void IntakeSubsystem::Out() {
  SetPercent(IntakeConstants::kOutPercent);
}

void IntakeSubsystem::Stop() {
  m_motor.StopMotor();
}
