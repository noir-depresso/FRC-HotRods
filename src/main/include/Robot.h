// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
<<<<<<< HEAD
#include <frc2/command/CommandPtr.h>
#include <optional>
=======
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
>>>>>>> 1f913b448f354b1f822500312f6ad9cc0ea46032

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
<<<<<<< HEAD
std::optional<frc2::CommandPtr> m_autonomousCommand;
=======
  frc2::CommandPtr m_autonomousCommand;
>>>>>>> 1f913b448f354b1f822500312f6ad9cc0ea46032

  RobotContainer m_container;
};