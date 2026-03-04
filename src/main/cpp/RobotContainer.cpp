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
#include "commands/AutoDriveToTagPose.h"
#include "commands/AutoDriveToFieldPoseSafe.h"
#include <frc/RobotBase.h>
#include <frc/Errors.h>
#include <fmt/core.h>

#include "io/VisionIO.h"
#include "io/VisionIOLimelight.h"
#include "io/VisionIOSim.h"

using namespace DriveConstants;

namespace {
void LogEvent(const std::string& msg) {
  fmt::print("{}\n", msg);
  FRC_ReportWarning("{}", msg);
}
}  // namespace

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
              const auto aimTurn = m_autoAim.GetTurnCommand();
              if (aimTurn.has_value()) {
                rot = units::radians_per_second_t{
                    aimTurn.value() * DriveConstants::kMaxAngularSpeed.value()};
                m_shooter.SetTurnPercent(aimTurn.value());
              } else {
                m_shooter.StopTurningMotor();
              }
            } else {
              m_shooter.StopTurningMotor();
            }

            // Last argument: fieldRelative
            m_drive.Drive(xSpeed, ySpeed, rot, false);
          },
          {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController, 6).OnTrue(new frc2::InstantCommand([this] {
    LogEvent("Button 6 pressed: X-lock enabled");
  }));

  frc2::JoystickButton(&m_driverController, 6).OnFalse(new frc2::InstantCommand([this] {
    LogEvent("Button 6 released: X-lock disabled");
  }));

  frc2::JoystickButton(&m_driverController, 6)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

       // Schedule ExampleCommand when exampleCondition changes to true
 frc2::JoystickButton(&m_driverController, 4).OnTrue(new frc2::InstantCommand([this] {
    LogEvent("Button 4 pressed: toggling intake");
    m_intakeRunning = !m_intakeRunning;

        if (m_intakeRunning)
            m_intake.In();
        else
            m_intake.Stop();
    }));

     frc2::JoystickButton(&m_driverController, 1).OnTrue(new frc2::InstantCommand([this] {
    LogEvent("Button 1 pressed: toggling shooter flywheels");
    m_shooterDriveRunning = !m_shooterDriveRunning;

        if (m_shooterDriveRunning)
            m_shooter.SpinDrivingMotors();
        else
            m_shooter.StopDrivingMotors();
    }));

     frc2::JoystickButton(&m_driverController, 3).OnTrue(new frc2::InstantCommand([this] {
    LogEvent("Button 3 pressed: toggling auto-aim mode");
    m_autoAimEnabled = !m_autoAimEnabled;

        if (m_autoAimEnabled)
            m_autoAim.Initialize();
        else {
            m_autoAim.End();
            m_shooter.StopTurningMotor();
        }
    }));

    //      frc2::JoystickButton(&m_driverController, 2).OnTrue(new frc2::InstantCommand([this] {
    // m_shooterTurnRunning = !m_shooterTurnRunning;

    //     if (m_shooterTurnRunning)
    //         m_shooter.SpinTurningMotor();
    //     else
    //         m_shooter.StopTurningMotor();
    // }));



  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule ExampleMethodCommand when the Xbox controller's B button is
  // // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}



frc2::Command* RobotContainer::GetAutonomousCommand() {
  //return new AutoDriveForward(&m_drive, 5_m); // potential issue
  // Field-goal autonomous using AprilTag-corrected robot pose.
  // Goal can be any field location (example: near midfield lane).
  constexpr frc::Pose2d kGoalPose{10.5_m, 1.1_m, 0_deg};

  // Approximate circular keep-out around center obstacle/goal hub area.
  constexpr frc::Translation2d kObstacleCenter{8.3_m, 4.1_m};
  constexpr units::meter_t kObstacleRadius = 1.3_m;
  constexpr units::meter_t kClearance = 0.7_m;

  return new AutoDriveToFieldPoseSafe(
      &m_drive,
      kGoalPose,
      kObstacleCenter,
      kObstacleRadius,
      kClearance);
}
