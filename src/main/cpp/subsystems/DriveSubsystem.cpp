#include "subsystems/DriveSubsystem.h"

#include <hal/FRCUsageReporting.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <fmt/core.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{m_driveKinematics,
                 GetRotation2d(),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {
  HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
             HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
}

void DriveSubsystem::Periodic() {
  m_odometry.Update(
      GetRotation2d(),
      {m_frontLeft.GetPosition(),
       m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(),
       m_rearRight.GetPosition()});

  const auto rotation = GetRotation2d();
  const double yawDeg = rotation.Degrees().value();
  const double yawRateDegPerSec =
      m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();

  const auto ntInst = nt::NetworkTableInstance::GetDefault();
  const auto limelightTable = ntInst.GetTable(VisionConstants::kLimelightName);
  if (limelightTable != nullptr) {
    LimelightHelpers::SetRobotOrientation(
        VisionConstants::kLimelightName,
        yawDeg,
        yawRateDegPerSec,
        0.0,
        0.0,
        0.0,
        0.0);
  }

  const auto ll =
      LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(
          VisionConstants::kLimelightName);
  double tx = 0.0;
  bool hasTarget = false;
  if (limelightTable != nullptr) {
    const double tv = limelightTable->GetNumber("tv", 0.0);
    hasTarget = (tv > 0.5);
    if (hasTarget) {
      tx = limelightTable->GetNumber("tx", 0.0);
    }
  }

  std::string direction = "No target";
  if (hasTarget) {
    if (tx > VisionConstants::kTxDeadbandDeg) {
      direction = "Right";
    } else if (tx < -VisionConstants::kTxDeadbandDeg) {
      direction = "Left";
    } else {
      direction = "Aligned";
    }
  }

  const std::string msg = fmt::format(
      "LL tv={} tx={:.2f} deg | dir={} | tags={}",
      hasTarget ? 1 : 0,
      tx,
      direction,
      ll.tagCount);

  ++m_visionLogCounter;
  const bool timeToPrint = (m_visionLogCounter % kVisionLogPeriod) == 0;
  const bool changed = (msg != m_lastVisionMessage);

  if (changed || timeToPrint) {
    fmt::print("{}\n", msg);
    m_lastVisionMessage = msg;
  }

  if (ll.tagCount > 0 && (m_visionLogCounter % kVisionLogPeriod) == 0) {
    PrintPoseEstimate(ll);
  }
}

frc::Rotation2d DriveSubsystem::GetRotation2d() const {
  const double sign = DriveConstants::kGyroReversed ? -1.0 : 1.0;
  return frc::Rotation2d{units::degree_t{
      sign * m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}};
}

void DriveSubsystem::PrintPoseEstimate(
    const LimelightHelpers::PoseEstimate& ll) {
  fmt::print(
      "\n--- Limelight Pose ---\n"
      "X: {:.3f} m\n"
      "Y: {:.3f} m\n"
      "Rot: {:.2f} deg\n"
      "Latency: {:.1f} ms\n"
      "TagCount: {}\n"
      "AvgTagDist: {:.2f} m\n"
      "Timestamp: {:.3f} s\n\n",
      ll.pose.X().value(),
      ll.pose.Y().value(),
      ll.pose.Rotation().Degrees().value(),
      ll.latency,
      ll.tagCount,
      ll.avgTagDist,
      ll.timestampSeconds.value());
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  frc::ChassisSpeeds speeds =
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                        GetRotation2d())
          : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);
  m_driveKinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  m_frontLeft.SetDesiredState(states[0]);
  m_frontRight.SetDesiredState(states[1]);
  m_rearLeft.SetDesiredState(states[2]);
  m_rearRight.SetDesiredState(states[3]);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState({0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState({0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState({0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState({0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  m_driveKinematics.DesaturateWheelSpeeds(&desiredStates, kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return GetRotation2d().Degrees();
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  const double sign = DriveConstants::kGyroReversed ? -1.0 : 1.0;
  return sign * m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
}
