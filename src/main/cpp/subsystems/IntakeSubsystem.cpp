//Intake.cpp

#include "subsystems/IntakeSubsystem.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include <algorithm>
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/MathUtil.h>
#include "Constants.h"
#include "LimelightHelpers.h"

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

void IntakeSubsystem::In()  { SetPercent(+0.675); } // start here, tune later
void IntakeSubsystem::Out() { SetPercent(-0.35); } // usually slower outtake
void IntakeSubsystem::Stop(){ m_motor.StopMotor(); }

void IntakeSubsystem::EnableAprilTagDirectionControl(bool enable) {
  m_aprilTagDirectionControlEnabled = enable;
  m_lastTagDistanceMeters.reset();

  if (!enable) {
    Stop();
  }
}

bool IntakeSubsystem::IsAprilTagDirectionControlEnabled() const {
  return m_aprilTagDirectionControlEnabled;
}

double IntakeSubsystem::GetClosestAprilTagDistanceMeters() const {
  const auto fiducials = LimelightHelpers::getRawFiducials("limelight");
  if (fiducials.empty()) {
    return -1.0;
  }

  const auto closestTag = std::min_element(
      fiducials.begin(), fiducials.end(),
      [](const auto& first, const auto& second) {
        return first.distToCamera < second.distToCamera;
      });

  return closestTag->distToCamera;
}

void IntakeSubsystem::Periodic() {
  frc::SmartDashboard::PutBoolean("Intake/AprilTagDirectionEnabled",
                                  m_aprilTagDirectionControlEnabled);

  if (!m_aprilTagDirectionControlEnabled) {
    return;
  }

  if (!LimelightHelpers::getTV("limelight")) {
    m_lastTagDistanceMeters.reset();
    Stop();
    return;
  }

  const double currentDistanceMeters = GetClosestAprilTagDistanceMeters();
  if (currentDistanceMeters < 0.0) {
    m_lastTagDistanceMeters.reset();
    Stop();
    return;
  }

  frc::SmartDashboard::PutNumber("Intake/ClosestTagDistanceMeters",
                                 currentDistanceMeters);

  if (!m_lastTagDistanceMeters.has_value()) {
    m_lastTagDistanceMeters = currentDistanceMeters;
    Stop();
    return;
  }

  constexpr double kDistanceDeadbandMeters = 0.03;
  const double distanceDeltaMeters =
      currentDistanceMeters - m_lastTagDistanceMeters.value();

  frc::SmartDashboard::PutNumber("Intake/TagDistanceDeltaMeters",
                                 distanceDeltaMeters);

  if (std::fabs(distanceDeltaMeters) <= kDistanceDeadbandMeters) {
    Stop();
  } else if (distanceDeltaMeters < 0.0) {
    SetPercent(0.5);
  } else {
    SetPercent(-0.5);
  }

  m_lastTagDistanceMeters = currentDistanceMeters;
}