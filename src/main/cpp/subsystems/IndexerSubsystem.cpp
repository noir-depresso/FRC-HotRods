//IndexerSubsystem.cpp

#include "subsystems/IndexerSubsystem.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include <algorithm>
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/MathUtil.h>
#include "Constants.h"
#include "LimelightHelpers.h"

IndexerSubsystem::IndexerSubsystem()

    : m_motor(DriveConstants::kIntakeCanID,
              rev::spark::SparkMax::MotorType::kBrushless) {

  rev::spark::SparkMaxConfig neo20config;

  // Conservative defaults to protect motor and avoid brownout spikes.
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


void IndexerSubsystem::SetPercent(double percent) {
  // Deadband removes joystick noise that can make rollers "creep" at idle.
  percent = frc::ApplyDeadband(percent, 0.02);
  percent = std::clamp(percent, -1.0, 1.0);
  m_motor.Set(percent);
}

void IndexerSubsystem::In()  { SetPercent(+0.75); } // start here, tune later
void IndexerSubsystem::Out() { SetPercent(-0.35); } // usually slower outtake
void IndexerSubsystem::Stop(){ m_motor.StopMotor(); }

void IndexerSubsystem::Periodic() {
  // No periodic control loop yet; keep method defined for SubsystemBase vtable.
}

// void IntakeSubsystem::EnableAprilTagDirectionControl(bool enable) {
//   m_aprilTagDirectionControlEnabled = enable;
//   // Reset previous sample so first cycle after enable does not use stale delta.
//   m_lastTagDistanceMeters.reset();

//   if (!enable) {
//     Stop();
//   }
// }

// bool IntakeSubsystem::IsAprilTagDirectionControlEnabled() const {
//   return m_aprilTagDirectionControlEnabled;
// }

// double IntakeSubsystem::GetClosestAprilTagDistanceMeters() const {
//   // Raw fiducials gives per-tag camera distance; we use nearest as control signal.
//   const auto fiducials = LimelightHelpers::getRawFiducials("limelight");
//   if (fiducials.empty()) {
//     return -1.0;
//   }

//   const auto closestTag = std::min_element(
//       fiducials.begin(), fiducials.end(),
//       [](const auto& first, const auto& second) {
//         return first.distToCamera < second.distToCamera;
//       });

//   return closestTag->distToCamera;
// }

// void IntakeSubsystem::Periodic() {
//   frc::SmartDashboard::PutBoolean("Intake/AprilTagDirectionEnabled",
//                                   m_aprilTagDirectionControlEnabled);

//   if (!m_aprilTagDirectionControlEnabled) {
//     return;
//   }

//   if (!LimelightHelpers::getTV("limelight")) {
//     // No valid vision target means no direction cue, so fail safe to stopped.
//     m_lastTagDistanceMeters.reset();
//     Stop();
//     return;
//   }

//   const double currentDistanceMeters = GetClosestAprilTagDistanceMeters();
//   if (currentDistanceMeters < 0.0) {
//     m_lastTagDistanceMeters.reset();
//     Stop();
//     return;
//   }

//   frc::SmartDashboard::PutNumber("Intake/ClosestTagDistanceMeters",
//                                  currentDistanceMeters);

//   if (!m_lastTagDistanceMeters.has_value()) {
//     // Prime filter with first measurement; no direction decision yet.
//     m_lastTagDistanceMeters = currentDistanceMeters;
//     Stop();
//     return;
//   }

//   constexpr double kDistanceDeadbandMeters = 0.03;
//   const double distanceDeltaMeters =
//       currentDistanceMeters - m_lastTagDistanceMeters.value();

//   frc::SmartDashboard::PutNumber("Intake/TagDistanceDeltaMeters",
//                                  distanceDeltaMeters);

//   if (std::fabs(distanceDeltaMeters) <= kDistanceDeadbandMeters) {
//     // Ignore minor frame-to-frame jitter from vision noise.
//     Stop();
//   } else if (distanceDeltaMeters < 0.0) {
//     // Tag is getting closer: run intake in.
//     SetPercent(0.5);
//   } else {
//     // Tag is moving farther away: reverse intake.
//     SetPercent(-0.5);
//   }

//   m_lastTagDistanceMeters = currentDistanceMeters;
// }
