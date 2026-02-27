#include "subsystems/DriveSubsystem.h"
#include <frc/DriverStation.h>
#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <fmt/core.h>
#include "LimelightHelpers.h"

#include "Constants.h"

using namespace DriveConstants;


std::string g_lastLLMessage = "";
int g_printCounter = 0; // for rate limiting
DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{
                     m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {
  HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
             HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
}


void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeed.value() * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeed.value() * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      rot.value() * DriveConstants::kMaxAngularSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return frc::Rotation2d(
             units::radian_t{m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)})
      .Degrees();
}

void DriveSubsystem::ZeroHeading() { m_gyro.Reset(); }

double DriveSubsystem::GetTurnRate() {
  return -m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

//Update() in Unity
void DriveSubsystem::Periodic() {
  // Correct gyro rotation??? (degrees -> Rotation2d)
  frc::Rotation2d gyroRot{units::degree_t{
      m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}};

  // ORDER QUESTOINABLE: FL, FR, RL, RR
  m_odometry.Update(
      gyroRot,
      {m_frontLeft.GetPosition(),
       m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(),
       m_rearRight.GetPosition()});

  const double yawDeg = gyroRot.Degrees().value();
  const double yawRateDegPerSec =
      m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();

  LimelightHelpers::SetRobotOrientation(
      "limelight",
      yawDeg,
      yawRateDegPerSec,
      0.0, 0.0,   // pitch, pitchRate
      0.0, 0.0    // roll, rollRate
  );

  // Pose estimate
  auto ll = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

  if (ll.tagCount > 0) {
    static int counter = 0;
    if ((++counter % 10) == 0) {
      PrintPoseEstimate(ll);
    }
  }

  // Simple turn hint from tx
  constexpr double kDeadbandDeg = 1.5;
  bool hasTarget = LimelightHelpers::getTV("limelight");
  double tx = hasTarget ? LimelightHelpers::getTX("limelight") : 0.0;

  std::string direction = "No target";
  if (hasTarget) {
    if (tx > kDeadbandDeg) direction = "Right";
    else if (tx < -kDeadbandDeg) direction = "Left";
    else direction = "Aligned";
  }

  std::string msg = fmt::format("LL tx={:.2f} deg | dir={}", tx, direction);

  g_printCounter++;
  bool timeToPrint = (g_printCounter % 10) == 0;
  bool changed = (msg != g_lastLLMessage);

  if (changed || timeToPrint) {
    fmt::print("ERROR OCCURED WITH TARGETING -- rion"); // enable output
    g_lastLLMessage = msg;
  }
}
void DriveSubsystem::PrintPoseEstimate(const LimelightHelpers::PoseEstimate& ll) {
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
      ll.timestampSeconds.value()    // <-- units::second_t
  );
}