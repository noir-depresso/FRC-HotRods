// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADIS16470_IMU.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include <string>

#include "Constants.h"
#include "MAXSwerveModule.h"
#include "LimelightHelpers.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

 private:
  static constexpr int kVisionLogPeriod = 10;

  void PrintPoseEstimate(const LimelightHelpers::PoseEstimate& ll);

  frc::SwerveDriveKinematics<4> m_driveKinematics{
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2}};

  // Module order is FL, FR, RL, RR for all kinematics/odometry calls.
  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_rearRight;

  frc::ADIS16470_IMU m_gyro;

  frc::SwerveDriveOdometry<4> m_odometry;
  std::string m_lastVisionMessage{};
  int m_visionLogCounter = 0;

  frc::Rotation2d GetRotation2d() const;
};
