#include "subsystems/ShootingAutoAim.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/length.h>
#include <units/velocity.h>

#include "LimelightHelpers.h"
#include "shooter/ShotPlanner.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

using namespace units::literals;

namespace {
// Initial ballistic model constants; tune on-robot with real release geometry.
constexpr units::meter_t kGoalCenterOffsetX = 0.595_m;
constexpr units::meter_t kGoalRelativeHeight = 1.83_m;
constexpr units::meters_per_second_t kNominalLaunchSpeed = 6.0_mps; // 12.0
constexpr bool kPreferHighArc = false;
constexpr auto kHoodVerticalReference = units::degree_t{90.0};
constexpr auto kHoodForwardLimit = units::degree_t{32.0};
constexpr auto kTurretStepPerLoop = units::degree_t{1.8}; // speed of turret rotation
constexpr auto kTurretSearchStepPerLoop = units::degree_t{1.8};
constexpr auto kTurretSearchMinAngle = units::degree_t{-85.0};
constexpr auto kTurretSearchMaxAngle = units::degree_t{85.0};
constexpr auto kSearchReverseBuffer = units::degree_t{0.5};
constexpr auto kSearchDelay = units::second_t{2.0};

std::optional<frc::Translation2d> ComputeAllianceGoalCenter(
    const std::vector<int>& targetTags) {
  if (targetTags.empty()) {
    return std::nullopt;
  }

  const frc::AprilTagFieldLayout fieldLayout = frc::AprilTagFieldLayout::LoadField(
      frc::AprilTagField::k2026RebuiltWelded);

  units::meter_t sumX = 0_m;
  units::meter_t sumY = 0_m;
  int count = 0;

  for (int tagId : targetTags) {
    const auto tagPose = fieldLayout.GetTagPose(tagId);
    if (!tagPose.has_value()) {
      continue;
    }

    sumX += tagPose->X();
    sumY += tagPose->Y();
    ++count;
  }

  if (count == 0) {
    return std::nullopt;
  }

  return frc::Translation2d{sumX / count, sumY / count};
}
}

ShootingAutoAim::ShootingAutoAim(std::string limelightName,
                                 DriveSubsystem& drive)
    : m_ll(std::move(limelightName)),
      m_drive(drive),
      // Separate loops: turret (yaw) and hood (pitch/trajectory).
      m_turretPID(0.03, 0.0, 0.002),
       m_hoodPID(0.025, 0.0, 0.0015),
      // Pre-aim from odometry is intentionally gentle; vision does final lock.
      m_poseTurretPID(0.38, 0.0, 0.0) {
  m_turretPID.SetTolerance(1.0);
  m_hoodPID.SetTolerance(1.0);
  m_poseTurretPID.SetTolerance(2.0);
  UpdateAllianceTagIDs();
}

void ShootingAutoAim::Initialize() {
  // Rebuild alliance-dependent IDs/offsets every enable.
  // Watch out: if alliance is unknown at init, blue defaults are used below.
  UpdateAllianceTagIDs();
  m_turretPID.Reset();
  m_hoodPID.Reset();
  m_poseTurretPID.Reset();
  m_lastBestId = -1;
  m_loggedNoTarget = false;
  m_searchActive = false;
  m_searchDirection = 1.0;
  m_enableTimer.Reset();
  m_enableTimer.Start();
  LimelightHelpers::SetFiducialIDFiltersOverride(m_ll, m_centerIDs);
}

void ShootingAutoAim::End() {
  m_lastBestId = -1;
  m_turretPID.Reset();
  m_hoodPID.Reset();
  m_poseTurretPID.Reset();
  m_loggedNoTarget = false;
  m_searchActive = false;
  m_searchDirection = 1.0;
  m_enableTimer.Stop();
  m_enableTimer.Reset();
}

bool ShootingAutoAim::HasValidTarget() const {
  return LimelightHelpers::getTV(m_ll);
}

void ShootingAutoAim::UpdateAim(ShooterSubsystem& shooter) {
  // Hard fail-safe: no target means both aim axes are stopped.
  if (!LimelightHelpers::getTV(m_ll)) {
    if (!IsSearchDelayElapsed()) {
      m_searchActive = false;
      if (const auto preAimAngle = ComputePosePreAimTurretCommand(); preAimAngle.has_value()) {
        shooter.SetTurretAngle(preAimAngle.value());
      } else {
        shooter.StopTurretMotor();
      }
    } else {
      RunTurretSearch(shooter);
    }
    shooter.StopHoodMotor();
    m_loggedNoTarget = true;
    frc::SmartDashboard::PutBoolean("AutoAim/SearchActive", m_searchActive);
    frc::SmartDashboard::PutNumber("AutoAim/EnableTimeSec", m_enableTimer.Get().value());
    frc::SmartDashboard::PutNumber("AutoAim/SearchDirection", m_searchDirection);
    return;
  }

  m_loggedNoTarget = false;
  m_searchActive = false;

  const auto bestTag = SelectBestTag();
  if (!bestTag.has_value()) {
    shooter.StopTurretMotor();
    shooter.StopHoodMotor();
    return;
  }

  const auto aimOffsets = GetAimOffsets(bestTag.value());
  if (!aimOffsets.has_value()) {
    shooter.StopTurretMotor();
    shooter.StopHoodMotor();
    return;
  }

  m_lastBestId = bestTag.value();
  // Force Limelight to prioritize the selected tag for tx/ty stability.
  LimelightHelpers::setPriorityTagID(m_ll, bestTag.value());

  const double tx = LimelightHelpers::getTX(m_ll);
  const double ty = LimelightHelpers::getTY(m_ll);

  double turretCmd = m_turretPID.Calculate(tx, aimOffsets->txDeg);
  // Clamp protects mechanism and prevents aggressive oscillation.
  turretCmd = std::clamp(turretCmd, -0.6, 0.6);

  shooter.NudgeTurretAngle(kTurretStepPerLoop * turretCmd);

  const auto solvedTheta = CalculateBallisticHoodAngle();
  if (solvedTheta.has_value()) {
    shooter.SetHoodAngle(solvedTheta.value());
  } else {
    // Fallback to ty PID if ballistic solve is unavailable/unreachable.
    double hoodCmd = m_hoodPID.Calculate(ty, aimOffsets->tyDeg);
    hoodCmd = std::clamp(hoodCmd, -0.45, 0.45);
    shooter.SetHoodPercent(hoodCmd);
  }

  frc::SmartDashboard::PutBoolean("AutoAim/SearchActive", m_searchActive);
  frc::SmartDashboard::PutNumber("AutoAim/EnableTimeSec", m_enableTimer.Get().value());
  frc::SmartDashboard::PutNumber("AutoAim/SearchDirection", m_searchDirection);
}


std::optional<units::radian_t> ShootingAutoAim::CalculateBallisticHoodAngle() const {
  const auto bestTag = SelectBestTag();
  if (!bestTag.has_value()) {
    return std::nullopt;
  }

  for (const auto& f : LimelightHelpers::getRawFiducials(m_ll)) {
    if (f.id != bestTag.value()) {
      continue;
    }

    const units::meter_t x = units::meter_t{f.distToRobot} + kGoalCenterOffsetX;
    const auto launchTheta =
        ShotPlanner::SolveLaunchAngle(x, kGoalRelativeHeight, kNominalLaunchSpeed,
                                      kPreferHighArc);
    if (!launchTheta.has_value()) {
      return std::nullopt;
    }

    // Ballistics solves theta from horizontal; hood is defined from vertical-up.
    const units::radian_t hoodFromVertical =
        units::radian_t{kHoodVerticalReference} - launchTheta.value();
    if (hoodFromVertical < 0_deg || hoodFromVertical > kHoodForwardLimit) {
      return std::nullopt;
    }

    return hoodFromVertical;
  }

  return std::nullopt;
}

std::optional<units::radian_t> ShootingAutoAim::ComputePosePreAimTurretCommand() {
  const frc::Pose2d pose = m_drive.GetPose();

  if (!m_allianceGoalCenter.has_value()) {
    return std::nullopt;
  }

  const units::meter_t dx = m_allianceGoalCenter->X() - pose.X();
  const units::meter_t dy = m_allianceGoalCenter->Y() - pose.Y();
  const double distSq = (dx.value() * dx.value()) + (dy.value() * dy.value());
  if (distSq < 1e-4) {
    return std::nullopt;
  }

  const units::radian_t fieldBearing{std::atan2(dy.value(), dx.value())};
  const units::radian_t robotHeading = pose.Rotation().Radians();
  const units::radian_t robotRelativeError =
      frc::AngleModulus(fieldBearing - robotHeading);
  // Command turret absolute angle relative to robot-forward reference.
  return robotRelativeError;
}


std::optional<int> ShootingAutoAim::SelectBestTag() const {
  auto fiducials = LimelightHelpers::getRawFiducials(m_ll);
  if (fiducials.empty()) {
    return std::nullopt;
  }

  int bestId = -1;
  double bestScore = 1e9;

  for (const auto& f : fiducials) {
    if (std::find(m_centerIDs.begin(), m_centerIDs.end(), f.id) == m_centerIDs.end()) {
      continue;
    }

    // Lower score is better: low ambiguity and closer targets are preferred.
    const double score = f.ambiguity + 0.05 * f.distToRobot;
    if (score < bestScore) {
      bestScore = score;
      bestId = f.id;
    }
  }

  if (bestId == -1) {
    return std::nullopt;
  }

  return bestId;
}

std::optional<ShootingAutoAim::AimOffsets> ShootingAutoAim::GetAimOffsets(
    int tagId) const {
  const auto it = m_targetOffsetsByTag.find(tagId);
  if (it == m_targetOffsetsByTag.end()) {
    return std::nullopt;
  }
  return it->second;
}

void ShootingAutoAim::UpdateAllianceTagIDs() {
  const auto alliance = frc::DriverStation::GetAlliance();
  if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
    // 2026 field target tags on the red end.
    m_centerIDs = {8, 9, 10, 11, 2, 5};
    m_targetOffsetsByTag = {
        {8, {.txDeg = 0.0, .tyDeg = -1.0}}, {9, {.txDeg = 0.8, .tyDeg = -0.7}},
        {10, {.txDeg = -0.6, .tyDeg = -0.9}}, {11, {.txDeg = 0.0, .tyDeg = -0.6}},
        {2, {.txDeg = 0.4, .tyDeg = -1.2}}, {5, {.txDeg = -0.7, .tyDeg = -1.0}}};
  } else {
    // Default to blue set when alliance is unavailable.
    m_centerIDs = {18, 19, 20, 21, 24, 27};
    m_targetOffsetsByTag = {
        {18, {.txDeg = 0.0, .tyDeg = -1.0}}, {19, {.txDeg = -0.8, .tyDeg = -0.7}},
        {20, {.txDeg = 0.6, .tyDeg = -0.9}}, {21, {.txDeg = 0.0, .tyDeg = -0.6}},
        {24, {.txDeg = -0.4, .tyDeg = -1.2}}, {27, {.txDeg = 0.7, .tyDeg = -1.0}}};
  }

  m_allianceGoalCenter = ComputeAllianceGoalCenter(m_centerIDs);
}

bool ShootingAutoAim::IsSearchDelayElapsed() const {
  return m_enableTimer.HasElapsed(kSearchDelay);
}

void ShootingAutoAim::RunTurretSearch(ShooterSubsystem& shooter) {
  m_searchActive = true;

  const auto turretAngle = units::degree_t{shooter.GetTurretAngle()};
  if (turretAngle >= (kTurretSearchMaxAngle - kSearchReverseBuffer)) {
    m_searchDirection = -1.0;
  } else if (turretAngle <= (kTurretSearchMinAngle + kSearchReverseBuffer)) {
    m_searchDirection = 1.0;
  }

  const auto step = kTurretSearchStepPerLoop * m_searchDirection;
  const auto nextTurretAngle = std::clamp(turretAngle + step, kTurretSearchMinAngle,
                                          kTurretSearchMaxAngle);
  shooter.SetTurretAngle(units::radian_t{nextTurretAngle});
}
