#include "subsystems/IndexerSubsystem.h"

#include <algorithm>

#include <frc/MathUtil.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

IndexerSubsystem::IndexerSubsystem()
    : m_motor(DriveConstants::kIndexerCanID,
              rev::spark::SparkMax::MotorType::kBrushless) {
  rev::spark::SparkMaxConfig config;
  config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  config.SmartCurrentLimit(30);
  config.OpenLoopRampRate(0.08);
  config.VoltageCompensation(12.0);

  m_motor.Configure(config, rev::ResetMode::kResetSafeParameters,
                    rev::PersistMode::kPersistParameters);
}

void IndexerSubsystem::SetPercent(double percent) {
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);
  m_motor.Set(percent);
}

void IndexerSubsystem::FeedToStorage() { SetPercent(+0.50); }

void IndexerSubsystem::FeedToShooter() { SetPercent(+0.70); }

void IndexerSubsystem::Reverse() { SetPercent(-0.40); }

void IndexerSubsystem::Stop() { m_motor.StopMotor(); }
