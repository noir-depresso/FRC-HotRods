#pragma once



#include <frc2/command/CommandHelper.h>

#include <frc2/command/Command.h>

#include "subsystems/DriveSubsystem.h"

#include <units/length.h>

#include <units/velocity.h>

#include <frc/Timer.h>



/**

 * Simple autonomous: drive forward a set distance and stop.

 */

class AutoDriveForward

    : public frc2::CommandHelper<frc2::Command, AutoDriveForward> {

 public:

  AutoDriveForward(DriveSubsystem* drive, units::meter_t distance)

      : m_drive(drive), m_targetDistance(distance) {

    AddRequirements({m_drive});

  }



  void Initialize() override {

    m_startPose = m_drive->GetPose(); // remember starting position

    m_timer.Reset();

    m_timer.Start();

  }



  void Execute() override {

    // Simple proportional drive forward

    frc::Pose2d currentPose = m_drive->GetPose();

    double dx = (currentPose.X() - m_startPose.X()).value();



    // Target reached?

    if (dx < m_targetDistance.value()) {

      m_drive->Drive(1_mps, 0_mps, 0_rad_per_s, true);  // forward 1 m/s

    } else {

      m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, true);  // stop

      m_finished = true;

    }

  }



  bool IsFinished() override {

    return m_finished || m_timer.Get() > 5_s; // safety timeout

  }



  void End(bool interrupted) override {

    m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, true);

  }



 private:

  DriveSubsystem* m_drive;

  frc::Pose2d m_startPose;

  units::meter_t m_targetDistance;

  frc::Timer m_timer;

  bool m_finished = false;

};