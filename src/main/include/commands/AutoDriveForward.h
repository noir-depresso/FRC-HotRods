// #pragma once

// #include <frc2/command/CommandHelper.h>
// #include <frc2/command/Command.h>
// #include "subsystems/DriveSubsystem.h"
// #include <units/length.h>
// #include <units/velocity.h>
// #include <frc/Timer.h>

// /**
//  * Simple autonomous: drive forward a set distance and stop.
//  */
// class AutoDriveForward
//     : public frc2::CommandHelper<frc2::Command, AutoDriveForward> {
//  public:
//   AutoDriveForward(DriveSubsystem* drive, units::meter_t distance)
//       : m_drive(drive), m_targetDistance(distance) {
//     AddRequirements({m_drive});
//   }

// void AutoDriveForward::Initialize() {
//     m_startPose = m_drive->GetPose();
//     m_finished = false; // Always reset state variables here!
//     m_timer.Reset();
//     m_timer.Start();
// }

// void AutoDriveForward::Execute() {
//     // Just drive. Don't worry about the stop logic here.
//     m_drive->Drive(1_mps, 0_mps, 0_rad_per_s, true);
// }

// bool AutoDriveForward::IsFinished() {
//     // Calculate actual distance traveled regardless of starting angle
//     auto currentPose = m_drive->GetPose();
//     units::meter_t distanceTraveled = currentPose.Translation().Distance(m_startPose.Translation());

//     return distanceTraveled >= m_targetDistance || m_timer.Get() > 5_s;
// }

// void AutoDriveForward::End(bool interrupted) {
//     m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, true);
//     m_timer.Stop();
// }

//  private:
//   DriveSubsystem* m_drive;
//   frc::Pose2d m_startPose;
//   units::meter_t m_targetDistance;
//   frc::Timer m_timer;
//   bool m_finished = false;
// };