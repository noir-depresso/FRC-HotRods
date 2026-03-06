// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
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
#include <frc/Errors.h>
#include <fmt/core.h>

#include "io/VisionIO.h"
#include "io/VisionIOLimelight.h"
#include "io/VisionIOSim.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/DriverStation.h>
#include <pathplanner/lib/auto/NamedCommands.h>

using namespace DriveConstants;

namespace {
void LogEvent(const std::string& msg) {
  fmt::print("{}\n", msg);
  FRC_ReportWarning("{}", msg);
}
}  // namespace

RobotContainer::RobotContainer()
    : m_autoAim("limelight", m_drive) {
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
            // Raw axes are from GenericHID; adjust mapping if controller layout changes.
            const double xInput = frc::ApplyDeadband(m_driverController.GetRawAxis(1),
                                                     OIConstants::kDriveDeadband);
            const double yInput = frc::ApplyDeadband(m_driverController.GetRawAxis(0),
                                                     OIConstants::kDriveDeadband);
            const double rotInput = frc::ApplyDeadband(m_driverController.GetRawAxis(4),
                                                       OIConstants::kDriveDeadband);

            const auto xSpeed = -xInput * DriveConstants::kMaxSpeed;
            const auto ySpeed = -yInput * DriveConstants::kMaxSpeed;
            auto rot = -rotInput * DriveConstants::kMaxAngularSpeed;

            if (m_autoAimEnabled) {
              // Auto-aim controls turret + hood only. Drivetrain rotation stays on driver input.
              m_autoAim.UpdateAim(m_shooter);
            } else {
              // When pre-aim is off, hold turret aligned with robot forward heading.
              m_shooter.SetTurretAngle(units::radian_t{0.0});
              m_shooter.StopHoodMotor();
            }

            // Last argument: fieldRelative
            m_drive.Drive(xSpeed, ySpeed, rot, false);
          },
          {&m_drive}));

pathplanner::RobotConfig config;

try {
    // Pull robot geometry from PathPlanner GUI settings.
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
                   alliance.value() == frc::DriverStation::Alliance::kRed;
        },
        &m_drive
    );

    pathplanner::NamedCommands::registerCommand(
    "DriveForward",
    AutoDriveForward(&m_drive, 5_m).ToPtr().WithTimeout(3_s)
);
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController, 6).OnTrue(frc2::InstantCommand([this] {
    LogEvent("Button 6 pressed: X-lock enabled");
  }).ToPtr());

  frc2::JoystickButton(&m_driverController, 6).OnFalse(frc2::InstantCommand([this] {
    LogEvent("Button 6 released: X-lock disabled");
  }).ToPtr());

  frc2::JoystickButton(&m_driverController, 6)
      .WhileTrue(frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}).ToPtr());

       // Schedule ExampleCommand when exampleCondition changes to true
 frc2::JoystickButton(&m_driverController, 4).OnTrue(frc2::InstantCommand([this] {
    LogEvent("Button 4 pressed: toggling intake");
    m_intakeRunning = !m_intakeRunning;

        if (m_intakeRunning)
            m_intake.In();
        else
            m_intake.Stop();
    }).ToPtr());

 frc2::JoystickButton(&m_driverController, 9).OnTrue(frc2::InstantCommand([this] {
    LogEvent("Button 9 pressed: toggling indexer");
    m_indexerRunning = !m_indexerRunning;

        if (m_indexerRunning)
            m_indexer.In();
        else
            m_indexer.Stop();
    }).ToPtr());

     frc2::JoystickButton(&m_driverController, 1).OnTrue(frc2::InstantCommand([this] {
    LogEvent("Button 1 pressed: toggling shooter flywheels");
    m_shooterDriveRunning = !m_shooterDriveRunning;

        if (m_shooterDriveRunning)
            m_shooter.SpinDrivingMotors();
        else
            m_shooter.StopDrivingMotors();
    }).ToPtr());

     frc2::JoystickButton(&m_driverController, 3).OnTrue(frc2::InstantCommand([this] {
    LogEvent("Button 3 pressed: toggling auto-aim mode");
    m_autoAimEnabled = !m_autoAimEnabled;

        if (m_autoAimEnabled) {
            m_shooter.ZeroTurretEncoder();
            m_autoAim.Initialize();
        }
        else {
            m_autoAim.End();
            // Return turret to robot-forward alignment when pre-aim is disabled.
            m_shooter.SetTurretAngle(units::radian_t{0.0});
            m_shooter.StopHoodMotor();
        }
    }).ToPtr());

    frc2::JoystickButton(&m_driverController, 2).OnTrue(new frc2::InstantCommand([this] {
      LogEvent("Button 2 pressed: move hood to calculated initial angle");
      if (const auto theta = m_autoAim.CalculateBallisticHoodAngle(); theta.has_value()) {
        m_shooter.SetHoodAngle(theta.value());
      } else {
        m_shooter.MoveHoodToInitialAngle();
      }
    }));

  frc2::JoystickButton(&m_driverController, 5)
    .OnTrue(frc2::InstantCommand([this] { m_pistonSubsystem.Toggle(); }, {&m_pistonSubsystem}).ToPtr());

  // Use 'B' to extend and 'X' to retract
  frc2::JoystickButton(&m_driverController, 7)
    .OnTrue(frc2::InstantCommand([this] { m_pistonSubsystem.Extend(); }, {&m_pistonSubsystem}).ToPtr());
    
  frc2::JoystickButton(&m_driverController, 8)
    .OnTrue(frc2::InstantCommand([this] { m_pistonSubsystem.Retract(); }, {&m_pistonSubsystem}).ToPtr());

    frc2::JoystickButton(&m_driverController, 10).OnTrue(frc2::InstantCommand([this] {
    m_aprilTagDirectionRunning = !m_aprilTagDirectionRunning;
    m_intake.EnableAprilTagDirectionControl(m_aprilTagDirectionRunning);

        if (m_aprilTagDirectionRunning) {
            m_intakeRunning = true;
        } else {
            m_intakeRunning = false;
            m_intake.Stop();
        }
    }).ToPtr());



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

return pathplanner::PathPlannerAuto("Auto 2").ToPtr();
}
