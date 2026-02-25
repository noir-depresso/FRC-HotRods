// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/GenericHID.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  frc::GenericHID m_driverController{OIConstants::kDriverControllerPort};
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;

  frc::SendableChooser<frc2::Command*> m_chooser;
  bool m_isIntakeRunning = false;

  void ConfigureButtonBindings();
};
