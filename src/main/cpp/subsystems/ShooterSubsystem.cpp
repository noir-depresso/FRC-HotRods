//ShooterSubsystem.cpp

#include "subsystems/ShooterSubsystem.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include <frc/MathUtil.h>
#include "Constants.h"

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

  m_drivingMotor1.Configure(
    neo20config,
    rev::ResetMode::kResetSafeParameters,
    rev::PersistMode::kPersistParameters
  );

m_drivingMotor2.Configure(
    neo20config,
    rev::ResetMode::kResetSafeParameters,
    rev::PersistMode::kPersistParameters
  );

m_turningMotor.Configure(
    neo20config,
    rev::ResetMode::kResetSafeParameters,
    rev::PersistMode::kPersistParameters
  );
}


// void ShooterSubsystem::SetPercent(double percent, int motorPath) {
//   percent = frc::ApplyDeadband(percent, 0.02);
//   percent = std::clamp(percent, -1.0, 1.0);
//   if (motorPath == 1) {
//     m_drivingMotor1.Set(percent);
//     m_drivingMotor2.Set(percent);
//   } else {
//     m_turningMotor.Set(percent);
//   }
// }

void ShooterSubsystem::SetPercent(double percent) {
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);

  m_drivingMotor1.Set(percent);
  m_drivingMotor2.Set(percent);

}

void ShooterSubsystem::SetTurnPercent(double percent) {
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);

  m_turningMotor.Set(percent);
}

void ShooterSubsystem::SpinDrivingMotors()  { SetPercent(+0.75); }
//void ShooterSubsystem::SpinTurningMotor()  { SetPercent(+0.5, 2); }
// void ShooterSubsystem::Out() { SetPercent(-0.35); } // usually slower outtake
void ShooterSubsystem::StopDrivingMotors() {
  m_drivingMotor1.StopMotor();
  m_drivingMotor2.StopMotor();
}
void ShooterSubsystem::StopTurningMotor() { m_turningMotor.StopMotor(); }
