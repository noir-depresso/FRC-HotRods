#include "subsystems/DriveSubsystem.h"

#include <algorithm>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>

#include <frc/geometry/Pose2d.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/config/RobotConfig.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <fmt/core.h>

#include "Constants.h"
#include "LimelightHelpers.h"

using namespace DriveConstants;
  using namespace pathplanner;


// File-local state (avoid global symbol collisions across cpp files)
static std::string g_lastLLMessage{};
static int g_printCounter = 0;
static units::second_t g_lastVisionResetTime = 0_s;
static bool g_autoVisionSeeded = false;

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

    frc::SmartDashboard::PutData("Field", &m_field);

  // Start robot at origin (0,0,0)
  m_pose = frc::Pose2d();

  auto inst = nt::NetworkTableInstance::GetDefault();

  m_pose3dPub = inst.GetStructTopic<frc::Pose3d>("RobotPose3d").Publish();

  // pathplanner auto builder
// RobotConfig config;

// try {
//     config = RobotConfig::fromGUISettings();
// } catch (...) {
//     fmt::print("Failed to load RobotConfig from GUI\n");
// }

// AutoBuilder::configure(
//     [this]() { return GetPose(); },

//     [this](const frc::Pose2d& pose) { ResetOdometry(pose); },

//     [this]() {
//         return frc::ChassisSpeeds{
//             units::meters_per_second_t{m_lastXSpeed},
//             units::meters_per_second_t{m_lastYSpeed},
//             units::radians_per_second_t{m_lastRot}
//         };
//     },

//     [this](const frc::ChassisSpeeds& speeds) {
//         Drive(
//             speeds.vx,
//             speeds.vy,
//             speeds.omega,
//             false
//         );
//     },

//     std::make_shared<PPHolonomicDriveController>(
//         PIDConstants(5.0, 0.0, 0.0),
//         PIDConstants(5.0, 0.0, 0.0)
//     ),

//     config,

//     []() {
//         auto alliance = frc::DriverStation::GetAlliance();
//         return alliance &&
//                alliance.value() == frc::DriverStation::Alliance::kRed;
//     },

//     this
// );
}


void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  const units::meters_per_second_t xSpeedDelivered =
      std::clamp(xSpeed, -DriveConstants::kMaxSpeed, DriveConstants::kMaxSpeed);
  const units::meters_per_second_t ySpeedDelivered =
      std::clamp(ySpeed, -DriveConstants::kMaxSpeed, DriveConstants::kMaxSpeed);
  const units::radians_per_second_t rotDelivered = std::clamp(
      rot, -DriveConstants::kMaxAngularSpeed, DriveConstants::kMaxAngularSpeed);

  if (fieldRelative) {
    m_lastXSpeed  = xSpeedDelivered.value();
    m_lastYSpeed  = ySpeedDelivered.value();
  } else {
    const frc::ChassisSpeeds fieldSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered},
        GetRotation2d());
    m_lastXSpeed  = fieldSpeeds.vx.value();
    m_lastYSpeed  = fieldSpeeds.vy.value();
  }
  m_lastRot = rotDelivered.value();

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

void DriveSubsystem::DriveRobotRelative(const frc::ChassisSpeeds& speeds) {
  auto states = kDriveKinematics.ToSwerveModuleStates(speeds);
  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);

  m_lastXSpeed = speeds.vx.value();
  m_lastYSpeed = speeds.vy.value();
  m_lastRot = speeds.omega.value();
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

frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeSpeeds() const {
    // Compute robot-relative speeds from module states
    return kDriveKinematics.ToChassisSpeeds(
        m_frontLeft.GetState(),
        m_frontRight.GetState(),
        m_rearLeft.GetState(),
        m_rearRight.GetState()
    );
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

  auto ll = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

  if (!frc::DriverStation::IsAutonomousEnabled()) {
    g_autoVisionSeeded = false;
  }

  if (frc::DriverStation::IsAutonomousEnabled() && ll.tagCount > 0) {
    const units::second_t now = frc::Timer::GetFPGATimestamp();
    if (!g_autoVisionSeeded || (now - g_lastVisionResetTime) > 0.50_s) {
      ResetOdometry(ll.pose);
      g_lastVisionResetTime = now;
      g_autoVisionSeeded = true;
    }
  }

  if (ll.tagCount > 0) {
    static int posePrintCounter = 0;
    if ((++posePrintCounter % 10) == 0) {
      std::string ids;
      for (size_t i = 0; i < ll.rawFiducials.size(); ++i) {
        ids += std::to_string(ll.rawFiducials[i].id);
        if (i + 1 < ll.rawFiducials.size()) ids += ",";
      }
      fmt::print("Detected AprilTag IDs: [{}]\n", ids);
      PrintPoseEstimate(ll);
    }
  }


  // Simple turn hint from tx
  constexpr double kDeadbandDeg = 1.5;
  const bool hasTarget = LimelightHelpers::getTV("limelight");
  const double tx = hasTarget ? LimelightHelpers::getTX("limelight") : 0.0;

  std::string direction = "No target";
  if (hasTarget) {
    if (tx > kDeadbandDeg) direction = "Right";
    else if (tx < -kDeadbandDeg) direction = "Left";
    else direction = "Aligned";
  }

  const std::string msg = fmt::format("LL tv={} tx={:.2f} deg | dir={}",
                                      hasTarget ? 1 : 0, tx, direction);

  // Rate limit prints to not spam the DS console, but still print instantly on change.
  ++g_printCounter;
  const bool timeToPrint = (g_printCounter % 10) == 0;
  const bool changed = (msg != g_lastLLMessage);

  if (changed || timeToPrint) {
    fmt::print("{}\n", msg);
    g_lastLLMessage = msg;
  }

     // Current pose from odometry
  // In sim, advance a lightweight kinematic pose integration for visualization.
  // On real robot, do NOT overwrite odometry here (that would undo encoder/vision updates).
if (frc::RobotBase::IsSimulation()) {
    frc::Pose2d pose = m_odometry.GetPose();
    constexpr double kDt = 0.02;

    pose = frc::Pose2d{
        pose.X() + units::meter_t{m_lastXSpeed  * kDt},
        pose.Y() + units::meter_t{m_lastYSpeed  * kDt},
        pose.Rotation() + frc::Rotation2d{units::radian_t{m_lastRot * kDt}}
    };

    m_odometry.ResetPosition(
        GetRotation2d(),
        {m_frontLeft.GetPosition(),
         m_frontRight.GetPosition(),
         m_rearLeft.GetPosition(),
         m_rearRight.GetPosition()},
        pose);
}

  // Publish whichever pose odometry currently holds (including any vision correction).
  m_field.SetRobotPose(m_odometry.GetPose());
}

frc::Rotation2d DriveSubsystem::GetRotation2d() const {
  return frc::Rotation2d{
      units::degree_t{m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}};
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
      ll.timestampSeconds.value());
}
