// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <frc2/command/CommandScheduler.h>

using namespace units::literals;

#include "RobotContainer.h"

void Robot::RobotInit() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // Autonomous is intentionally disabled for this robot configuration.
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand.reset();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    // Watch out: if auto is still running when teleop starts, it can keep
    // owning subsystems and block driver commands unless we cancel here.
    m_autonomousCommand->Cancel();
    m_autonomousCommand.reset();   // clears the optional
  }

  // This project intentionally re-zeros odometry for teleop at field origin.
  // If you need pose continuity between auto and teleop, remove this reset.
  m_container.GetDriveSubsystem().ResetOdometry(
      frc::Pose2d{0_m, 0_m, 0_deg});
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

