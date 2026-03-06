// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//#include <frc/XboxController.h>
#include <frc/GenericHID.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/IndexerSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ShootingAutoAim.h"
#include "subsystems/PistonSubsystem.h"

#include <memory>
#include "io/VisionIO.h"


class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

DriveSubsystem& GetDriveSubsystem() { return m_drive; }

 private:
  // The driver's controller
  //frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::GenericHID m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  IndexerSubsystem m_indexer;
  ShooterSubsystem m_shooter;
  PistonSubsystem m_pistonSubsystem;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  bool m_shooterDriveRunning = false;
  // bool m_shooterTurnRunning = false;
  bool m_autoAimEnabled = false;
  bool m_aprilTagDirectionRunning = false;
  bool m_intakeRequest = false;
  bool m_storageRequest = false;
  bool m_shootFeedRequest = false;
  bool m_estopRequest = false;

  ShootingAutoAim m_autoAim;

  void ConfigureButtonBindings();
  void UpdateSuperstructure();

  // Vision system pointer
  std::shared_ptr<VisionIO> vision;
};
