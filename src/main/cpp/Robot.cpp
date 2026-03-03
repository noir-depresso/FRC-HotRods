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
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
<<<<<<< HEAD
    m_autonomousCommand->Schedule();
=======
    m_autonomousCommand.Schedule();
>>>>>>> 1f913b448f354b1f822500312f6ad9cc0ea46032
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
<<<<<<< HEAD
    m_autonomousCommand->Cancel();
    m_autonomousCommand.reset();   // clears the optional
=======
    m_autonomousCommand.Cancel();
    m_autonomousCommand = frc2::CommandPtr();
>>>>>>> 1f913b448f354b1f822500312f6ad9cc0ea46032
  }

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
