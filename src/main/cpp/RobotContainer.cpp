// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include "RobotContainer.h"
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "commands/AutoDriveForward.h"
//#include "commands/AutoDriveToTagPose.h"
#include "commands/AutoDriveToFieldPoseSafe.h"
#include <frc/RobotBase.h>

#include "io/VisionIO.h"
#include "io/VisionIOLimelight.h"
#include "io/VisionIOSim.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>
#include <pathplanner/lib/auto/NamedCommands.h>

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureButtonBindings();

    // ----- Initialize vision -----
  if (frc::RobotBase::IsSimulation()) {
      vision = std::make_shared<VisionIOSim>();
  } else {
      vision = std::make_shared<VisionIOLimelight>();
  }
  // -----------------------------

  // Default drive command (teleop)
  // Left stick = translation, right stick X = rotation (based on your axis mapping)
  m_drive.SetDefaultCommand(
      frc2::RunCommand(
          [this] {
            const auto xSpeed = -units::meters_per_second_t{
                frc::ApplyDeadband(m_driverController.GetRawAxis(1),
                                  OIConstants::kDriveDeadband)};
            const auto ySpeed = -units::meters_per_second_t{
                frc::ApplyDeadband(m_driverController.GetRawAxis(0),
                                  OIConstants::kDriveDeadband)};
            const auto rot = -units::radians_per_second_t{
                frc::ApplyDeadband(m_driverController.GetRawAxis(4),
                                  OIConstants::kDriveDeadband)};

            // Last argument: fieldRelative
            m_drive.Drive(xSpeed, ySpeed, rot, false);
          },
          {&m_drive}));

pathplanner::RobotConfig config;

try {
    config = pathplanner::RobotConfig::fromGUISettings();
    fmt::print("Loaded PathPlanner RobotConfig successfully!\n");
} catch (const std::exception& e) {
    fmt::print("Failed to load RobotConfig from GUI: {}\n", e.what());
    //config = pathplanner::RobotConfig(); // fallback to default safe values
}

    pathplanner::AutoBuilder::configure(
        [this]() { return m_drive.GetPose(); },
        [this](const frc::Pose2d& pose) { m_drive.ResetOdometry(pose); },
        [this]() { return m_drive.GetRobotRelativeSpeeds(); },
        [this](const frc::ChassisSpeeds& speeds) {
            m_drive.Drive(speeds.vx, speeds.vy, speeds.omega, false);
            // AutoBuilder gives real robot-relative velocities. Send those
            // directly to the drivetrain without joystick normalization.
            m_drive.DriveRobotRelative(speeds);
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            pathplanner::PIDConstants(5.0, 0.0, 0.0),
            pathplanner::PIDConstants(5.0, 0.0, 0.0)
        ),
        config,
        []() {
            auto alliance = frc::DriverStation::GetAlliance();
            return alliance &&
                   alliance.value() == frc::DriverStation::Alliance::kBlue;
        },
        &m_drive
    );

    pathplanner::NamedCommands::registerCommand(
    "DriveForward",
    AutoDriveForward(&m_drive, 5_m).ToPtr()
);
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController, 6)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

       // Schedule ExampleCommand when exampleCondition changes to true
 frc2::JoystickButton(&m_driverController, 4).OnTrue(new frc2::InstantCommand([this] {
    m_intakeRunning = !m_intakeRunning;

        if (m_intakeRunning)
            m_intake.In();
        else
            m_intake.Stop();
    }));

     frc2::JoystickButton(&m_driverController, 1).OnTrue(new frc2::InstantCommand([this] {
    m_shooterDriveRunning = !m_shooterDriveRunning;

        if (m_shooterDriveRunning)
            m_shooter.SpinDrivingMotors();
        else
            m_shooter.StopDrivingMotors();
    }));

    frc2::JoystickButton(&m_driverController, 2).OnTrue(new frc2::InstantCommand([this] {
    m_shooterTurnRunning = !m_shooterTurnRunning;

        if (m_shooterTurnRunning)
            m_shooter.SetHoodPercent(+0.25);
        else
            m_shooter.StopHoodMotor();
    }));



  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule ExampleMethodCommand when the Xbox controller's B button is
  // // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}



frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  //return new AutoDriveForward(&m_drive, 5_m); // potential issue
  // Field-goal autonomous using AprilTag-corrected robot pose.
  // Goal can be any field location (example: near midfield lane).
//   constexpr frc::Pose2d kGoalPose{10.5_m, 1.1_m, 0_deg};

//   // Approximate circular keep-out around center obstacle/goal hub area.
//   constexpr frc::Translation2d kObstacleCenter{8.3_m, 4.1_m};
//   constexpr units::meter_t kObstacleRadius = 1.3_m;
//   constexpr units::meter_t kClearance = 0.7_m;

//   return new AutoDriveToFieldPoseSafe(
//       &m_drive,
//       kGoalPose,
//       kObstacleCenter,
//       kObstacleRadius,
//       kClearance);

return pathplanner::AutoBuilder::buildAuto("Auto 2");
}
