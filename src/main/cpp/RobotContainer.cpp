// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <units/velocity.h>

#include <fmt/core.h>

#include "Constants.h"
#include "commands/AutoDriveForward.h"
#include "io/VisionIO.h"
#include "io/VisionIOLimelight.h"
#include "io/VisionIOSim.h"

using namespace DriveConstants;
using namespace units::literals;

namespace {
void LogEvent(const std::string& msg) {
  fmt::print("{}\n", msg);
  FRC_ReportWarning("{}", msg);
}
}  // namespace

RobotContainer::RobotContainer()
    : m_autoAim("limelight", m_drive) {
  ConfigureButtonBindings();

  if (frc::RobotBase::IsSimulation()) {
    vision = std::make_shared<VisionIOSim>();
  } else {
    vision = std::make_shared<VisionIOLimelight>();
  }

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        const double xInput =
            frc::ApplyDeadband(m_driverController.GetRawAxis(1),
                               OIConstants::kDriveDeadband);
        const double yInput =
            frc::ApplyDeadband(m_driverController.GetRawAxis(0),
                               OIConstants::kDriveDeadband);
        const double rotInput =
            frc::ApplyDeadband(m_driverController.GetRawAxis(4),
                               OIConstants::kDriveDeadband);

        const auto xSpeed = -xInput * DriveConstants::kMaxSpeed;
        const auto ySpeed = -yInput * DriveConstants::kMaxSpeed;
        const auto rot = -rotInput * DriveConstants::kMaxAngularSpeed;

        if (m_estopRequest) {
          if (m_drive.IsRobotSpeedWithinRange(0.10_mps, 100.0_mps)) {
            m_drive.ApplyEmergencyStop();
          } else {
            m_drive.SetX();
          }
          UpdateSuperstructure();
          return;
        }

        if (m_autoAimEnabled) {
          m_autoAim.UpdateAim(m_shooter);
        } else {
          m_shooter.SetTurretAngle(units::radian_t{0.0});
          m_shooter.StopHoodMotor();
        }

        m_drive.Drive(xSpeed, ySpeed, rot, false);
        UpdateSuperstructure();
      },
      {&m_drive}));

  pathplanner::RobotConfig config;
  try {
    config = pathplanner::RobotConfig::fromGUISettings();
    fmt::print("Loaded PathPlanner RobotConfig successfully!\n");
  } catch (const std::exception& e) {
    fmt::print("Failed to load RobotConfig from GUI: {}\n", e.what());
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
          pathplanner::PIDConstants(5.0, 0.0, 0.0)),
      config,
      []() {
        auto alliance = frc::DriverStation::GetAlliance();
        return alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
      },
      &m_drive);

  pathplanner::NamedCommands::registerCommand(
      "DriveForward", AutoDriveForward(&m_drive, 5_m).ToPtr().WithTimeout(3_s));
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController, 6)
      .OnTrue(frc2::InstantCommand(
                  [this] { LogEvent("Button 6 pressed: X-lock enabled"); })
                  .ToPtr());

  frc2::JoystickButton(&m_driverController, 6)
      .OnFalse(frc2::InstantCommand(
                   [this] { LogEvent("Button 6 released: X-lock disabled"); })
                   .ToPtr());

  frc2::JoystickButton(&m_driverController, 6)
      .WhileTrue(frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}).ToPtr());

  frc2::JoystickButton(&m_driverController, 4)
      .OnTrue(frc2::InstantCommand([this] {
                LogEvent("Button 4 pressed: intake request ON");
                m_intakeRequest = true;
                UpdateSuperstructure();
              }).ToPtr());

  frc2::JoystickButton(&m_driverController, 4)
      .OnFalse(frc2::InstantCommand([this] {
                 LogEvent("Button 4 released: intake request OFF");
                 m_intakeRequest = false;
                 UpdateSuperstructure();
               }).ToPtr());

  frc2::JoystickButton(&m_driverController, 9)
      .OnTrue(frc2::InstantCommand([this] {
                LogEvent("Button 9 pressed: storage request ON");
                m_storageRequest = true;
                UpdateSuperstructure();
              }).ToPtr());

  frc2::JoystickButton(&m_driverController, 9)
      .OnFalse(frc2::InstantCommand([this] {
                 LogEvent("Button 9 released: storage request OFF");
                 m_storageRequest = false;
                 UpdateSuperstructure();
               }).ToPtr());

  frc2::JoystickButton(&m_driverController, 1)
      .OnTrue(frc2::InstantCommand([this] {
                LogEvent("Button 1 pressed: toggling shooter flywheels");
                m_shooterDriveRunning = !m_shooterDriveRunning;
                if (m_shooterDriveRunning) {
                  m_shooter.SpinDrivingMotors();
                } else {
                  m_shooter.StopDrivingMotors();
                }
              }).ToPtr());

  frc2::JoystickButton(&m_driverController, 3)
      .OnTrue(frc2::InstantCommand([this] {
                LogEvent("Button 3 pressed: toggling auto-aim mode");
                m_autoAimEnabled = !m_autoAimEnabled;
                if (m_autoAimEnabled) {
                  m_shooter.ZeroTurretEncoder();
                  m_autoAim.Initialize();
                } else {
                  m_autoAim.End();
                  m_shooter.SetTurretAngle(units::radian_t{0.0});
                  m_shooter.StopHoodMotor();
                }
              }).ToPtr());

  frc2::JoystickButton(&m_driverController, 2)
      .OnTrue(frc2::InstantCommand([this] {
                LogEvent("Button 2 pressed: move hood to calculated initial angle");
                if (const auto theta = m_autoAim.CalculateBallisticHoodAngle();
                    theta.has_value()) {
                  m_shooter.SetHoodAngle(theta.value());
                } else {
                  m_shooter.MoveHoodToInitialAngle();
                }
              }).ToPtr());

  frc2::JoystickButton(&m_driverController, 11)
      .OnTrue(frc2::InstantCommand([this] {
                LogEvent("Button 11 pressed: shooter feed request ON");
                m_shootFeedRequest = true;
                UpdateSuperstructure();
              }).ToPtr());

  frc2::JoystickButton(&m_driverController, 11)
      .OnFalse(frc2::InstantCommand([this] {
                 LogEvent("Button 11 released: shooter feed request OFF");
                 m_shootFeedRequest = false;
                 UpdateSuperstructure();
               }).ToPtr());

  frc2::JoystickButton(&m_driverController, 12)
      .OnTrue(frc2::InstantCommand([this] {
                LogEvent("Button 12 pressed: E-stop request ON");
                m_estopRequest = true;
                UpdateSuperstructure();
              }).ToPtr());

  frc2::JoystickButton(&m_driverController, 12)
      .OnFalse(frc2::InstantCommand([this] {
                 LogEvent("Button 12 released: E-stop request OFF");
                 m_estopRequest = false;
                 UpdateSuperstructure();
               }).ToPtr());

  frc2::JoystickButton(&m_driverController, 5)
      .OnTrue(frc2::InstantCommand(
                  [this] {
                    m_pistonSubsystem.Toggle();
                    UpdateSuperstructure();
                  },
                  {&m_pistonSubsystem})
                  .ToPtr());

  frc2::JoystickButton(&m_driverController, 7)
      .OnTrue(frc2::InstantCommand(
                  [this] {
                    m_pistonSubsystem.Extend();
                    UpdateSuperstructure();
                  },
                  {&m_pistonSubsystem})
                  .ToPtr());

  frc2::JoystickButton(&m_driverController, 8)
      .OnTrue(frc2::InstantCommand(
                  [this] {
                    m_pistonSubsystem.Retract();
                    UpdateSuperstructure();
                  },
                  {&m_pistonSubsystem})
                  .ToPtr());

  frc2::JoystickButton(&m_driverController, 10)
      .OnTrue(frc2::InstantCommand([this] {
                m_aprilTagDirectionRunning = !m_aprilTagDirectionRunning;
                m_intake.EnableAprilTagDirectionControl(m_aprilTagDirectionRunning);
                m_intakeRequest = m_aprilTagDirectionRunning;
                UpdateSuperstructure();
              }).ToPtr());
}

void RobotContainer::UpdateSuperstructure() {
  if (m_estopRequest) {
    m_intake.Stop();
    m_indexer.Stop();
    m_shooter.StopDrivingMotors();
    return;
  }

  if (m_shootFeedRequest && m_autoAimEnabled) {
    if (!m_pistonSubsystem.IsExtended()) {
      m_pistonSubsystem.Extend();
    }

    if (m_shooter.IsReadyToShoot(m_autoAim.HasValidTarget())) {
      m_indexer.FeedToShooter();
      m_intake.SetPercent(0.45);
    } else {
      m_indexer.Stop();
      m_intake.Stop();
    }
    return;
  }

  if (m_intakeRequest) {
    if (!m_pistonSubsystem.IsExtended()) {
      m_pistonSubsystem.Extend();
    }
    m_intake.In();
    m_indexer.FeedToStorage();
    return;
  }

  if (m_storageRequest) {
    m_intake.Stop();
    m_indexer.FeedToStorage();
    return;
  }

  m_intake.Stop();
  m_indexer.Stop();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return pathplanner::PathPlannerAuto("Auto 2").ToPtr();
}
