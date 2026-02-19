//Intake.cpp

#include "subsystems/IntakeSubsystem.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include <frc/MathUtil.h>
#include "Constants.h"

IntakeSubsystem::IntakeSubsystem()

    : m_motor(DriveConstants::kIntakeCanID,
              rev::spark::SparkMax::MotorType::kBrushless) {

  rev::spark::SparkMaxConfig neo20config;

  neo20config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  neo20config.SmartCurrentLimit(40);
  neo20config.OpenLoopRampRate(0.10);
  neo20config.VoltageCompensation(12.0);

  m_motor.Configure(
    neo20config,
    rev::ResetMode::kResetSafeParameters,
    rev::PersistMode::kPersistParameters
  );
}


void IntakeSubsystem::SetPercent(double percent) {
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);
  m_motor.Set(percent);
}

void IntakeSubsystem::In()  { SetPercent(+0.45); } // start here, tune later
void IntakeSubsystem::Out() { SetPercent(-0.35); } // usually slower outtake
void IntakeSubsystem::Stop(){ m_motor.StopMotor(); }