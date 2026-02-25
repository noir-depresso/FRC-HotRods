// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

RobotContainer::RobotContainer() {
  ConfigureButtonBindings();

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
            m_drive.Drive(xSpeed, ySpeed, rot, true);
          },
          {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController, 6)
      .WhileTrue(frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}).ToPtr());

  frc2::JoystickButton(&m_driverController, 4).OnTrue(
      frc2::InstantCommand([this] {
        m_isIntakeRunning = !m_isIntakeRunning;
        if (m_isIntakeRunning) {
          m_intake.In();
        } else {
          m_intake.Stop();
        }
      }).ToPtr());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return nullptr;
}
