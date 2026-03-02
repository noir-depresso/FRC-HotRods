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
#include "RobotContainer.h"
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "commands/AutoDriveForward.h"

#include "io/VisionIO.h"
#include "io/VisionIOLimelight.h"
#include "io/VisionIOSim.h"

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
  return new AutoDriveForward(&m_drive, 2_m);
}
