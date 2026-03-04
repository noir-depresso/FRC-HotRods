// #pragma once

// #include <algorithm>
// #include <string>

// #include <frc/Timer.h>
// #include <frc/apriltag/AprilTagFieldLayout.h>
// #include <frc/controller/PIDController.h>
// #include <frc/geometry/Pose2d.h>
// #include <frc/geometry/Transform2d.h>
// #include <frc/smartdashboard/SmartDashboard.h>
// #include <frc2/command/Command.h>
// #include <frc2/command/CommandHelper.h>
// #include <fmt/core.h>
// #include <units/angular_velocity.h>
// #include <units/velocity.h>

// #include "Constants.h"
// #include "subsystems/DriveSubsystem.h"
// #include "Constants.h"

// /**
//  * Minimal autonomous command: drive to a fixed offset from a selected AprilTag.
//  */
// class AutoDriveToTagPose
//     : public frc2::CommandHelper<frc2::Command, AutoDriveToTagPose> {
//  public:
//   AutoDriveToTagPose(DriveSubsystem* drive,
//                      int targetTagId,
//                      frc::Transform2d robotOffsetFromTag)
//       : m_drive(drive),
//         m_targetTagId(targetTagId),
//         m_robotOffsetFromTag(robotOffsetFromTag),
//         m_xPid(1.5, 0.0, 0.0),
//         m_yPid(1.5, 0.0, 0.0),
//         m_thetaPid(3.0, 0.0, 0.0) {
//     AddRequirements({m_drive});
//     m_thetaPid.EnableContinuousInput(-180.0, 180.0);
//   }

//   void Initialize() override {
//     m_finished = false;
//     m_usedFallbackPose = false;
//     m_endReason = "InProgress";
//     m_timer.Reset();
//     m_timer.Start();

//     auto fieldLayout = frc::AprilTagFieldLayout::LoadField(
//         frc::AprilTagField::k2026RebuiltWelded);

//     auto maybeTagPose = fieldLayout.GetTagPose(m_targetTagId);
//     if (maybeTagPose.has_value()) {
//       m_goalPose = maybeTagPose.value().ToPose2d().TransformBy(m_robotOffsetFromTag);
//       std::cout<< " DETECTED A TAG YES YES YES YES ";
//       fmt::print("Auto target tag {} -> goal X={:.2f}m Y={:.2f}m Rot={:.1f}deg\n",
//                  m_targetTagId,
//                  m_goalPose.X().value(),
//                  m_goalPose.Y().value(),
//                  m_goalPose.Rotation().Degrees().value());
//     } else {
//       // Fallback keeps robot near center if configured tag is missing.
//       m_goalPose = frc::Pose2d{5.0_m, 5.0_m, 0_deg};
//       std::cout<< "CANT DETECT A TAG CANT CANT CANT CANT CANT FFFFFFFFFFF";
//       fmt::print("Auto target tag {} not found in layout; fallback goal X={:.2f}m Y={:.2f}m Rot={:.1f}deg\n",
//                  m_targetTagId,
//                  m_goalPose.X().value(),
//                  m_goalPose.Y().value(),
//                  m_goalPose.Rotation().Degrees().value());
//     }

//     m_xPid.SetTolerance(0.20);
//     m_yPid.SetTolerance(0.20);
//     m_thetaPid.SetTolerance(8.0);

//     frc::SmartDashboard::PutBoolean("Auto/UsedFallbackTagPose", m_usedFallbackPose);
//     frc::SmartDashboard::PutNumber("Auto/TargetTagId", m_targetTagId);
//   }

//   void Execute() override {
//     const frc::Pose2d current = m_drive->GetPose();

//     const double rawVx = m_xPid.Calculate(current.X().value(), m_goalPose.X().value());
//     const double rawVy = m_yPid.Calculate(current.Y().value(), m_goalPose.Y().value());
//     const double rawOmegaDegPerSec =
//         m_thetaPid.Calculate(current.Rotation().Degrees().value(),
//                              m_goalPose.Rotation().Degrees().value());

//     // DriveSubsystem::Drive expects normalized [-1, 1] inputs.
//     const double xNorm = std::clamp(vx / DriveConstants::kMaxSpeed.value(), -1.0, 1.0);
//     const double yNorm = std::clamp(vy / DriveConstants::kMaxSpeed.value(), -1.0, 1.0);
//     const double rotNorm = std::clamp(
//         units::radians_per_second_t{units::degrees_per_second_t{omegaDegPerSec}}.value() /
//             DriveConstants::kMaxAngularSpeed.value(),
//         -1.0,
//         1.0);

//     m_drive->Drive(units::meters_per_second_t{xNorm},
//                    units::meters_per_second_t{yNorm},
//                    units::radians_per_second_t{rotNorm},
//                    true);

//     if (m_xPid.AtSetpoint() && m_yPid.AtSetpoint() && m_thetaPid.AtSetpoint()) {
//       m_finished = true;
//       m_endReason = "AtSetpoint";
//     }
//   }

//   bool IsFinished() override {
//     if (m_finished) {
//       return true;
//     }

//     if (m_timer.Get() > 6_s) {
//       m_endReason = "Timeout";
//       return true;
//     }

//     return false;
//   }

//   void End(bool interrupted) override {
//     m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, true);

//     if (interrupted) {
//       m_endReason = "Interrupted";
//     }

//     frc::SmartDashboard::PutString("Auto/EndReason", m_endReason);
//     fmt::print("AutoDriveToTagPose ended: {}\n", m_endReason);
//   }

//  private:
//   DriveSubsystem* m_drive;
//   int m_targetTagId;
//   frc::Transform2d m_robotOffsetFromTag;
//   frc::Pose2d m_goalPose{0_m, 0_m, 0_deg};

//   frc::PIDController m_xPid;
//   frc::PIDController m_yPid;
//   frc::PIDController m_thetaPid;

//   frc::Timer m_timer;
//   bool m_finished{false};
//   bool m_usedFallbackPose{false};
//   std::string m_endReason{"NotStarted"};
// };