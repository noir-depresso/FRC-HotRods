#include "subsystems/ShootingAutoAim.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/geometry/Translation2d.h>
#include <units/length.h>

#include "LimelightHelpers.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

using namespace units::literals;

namespace {
// 2022 field center hub approximation in WPILib field coordinates.
constexpr frc::Translation2d kGoalHubCenter{8.27_m, 4.11_m};
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
  LimelightHelpers::SetFiducialIDFiltersOverride(m_ll, m_centerIDs);
}

void ShootingAutoAim::End() {
  m_lastBestId = -1;
  m_turretPID.Reset();
  m_hoodPID.Reset();
  m_poseTurretPID.Reset();
}

bool ShootingAutoAim::HasValidTarget() const {
  return LimelightHelpers::getTV(m_ll);
}

void ShootingAutoAim::UpdateAim(ShooterSubsystem& shooter) {
  // Hard fail-safe: no target means both aim axes are stopped.
  if (!LimelightHelpers::getTV(m_ll)) {
     if (const auto preAimCmd = ComputePosePreAimTurretCommand(); preAimCmd.has_value()) {
      shooter.SetTurretPercent(preAimCmd.value());
    } else {
      shooter.StopTurretMotor();
    }
    shooter.StopHoodMotor();
    m_loggedNoTarget = true;
    return;
  }

  m_loggedNoTarget = false;

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

  double hoodCmd = m_hoodPID.Calculate(ty, aimOffsets->tyDeg);
  // Hood is intentionally clamped tighter than turret to reduce over-correction.
  hoodCmd = std::clamp(hoodCmd, -0.45, 0.45);

  shooter.SetTurretPercent(turretCmd);
  shooter.SetHoodPercent(hoodCmd);
}

std::optional<double> ShootingAutoAim::ComputePosePreAimTurretCommand() {
  const frc::Pose2d pose = m_drive.GetPose();

  const units::meter_t dx = kGoalHubCenter.X() - pose.X();
  const units::meter_t dy = kGoalHubCenter.Y() - pose.Y();
  const double distSq = (dx.value() * dx.value()) + (dy.value() * dy.value());
  if (distSq < 1e-4) {
    return std::nullopt;
  }

  const units::radian_t fieldBearing{std::atan2(dy.value(), dx.value())};
  const units::radian_t robotHeading = pose.Rotation().Radians();
  const units::radian_t robotRelativeError =
      frc::AngleModulus(fieldBearing - robotHeading);
  const double robotRelativeErrorDeg = units::degree_t{robotRelativeError}.value();

  double cmd = m_poseTurretPID.Calculate(0.0, robotRelativeErrorDeg);
  cmd = std::clamp(cmd, -0.35, 0.35);
  return cmd;
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
    m_centerIDs = {8, 9, 10, 11, 2, 5};
    m_targetOffsetsByTag = {
        {8, {.txDeg = 0.0, .tyDeg = -1.0}}, {9, {.txDeg = -0.8, .tyDeg = -0.7}},
        {10, {.txDeg = 0.6, .tyDeg = -0.9}}, {11, {.txDeg = 0.0, .tyDeg = -0.6}},
        {2, {.txDeg = -0.4, .tyDeg = -1.2}}, {5, {.txDeg = 0.7, .tyDeg = -1.0}}};
  } else {
    // Default to blue set when alliance is unavailable.
    // Tune these values on-robot; they are initial aiming offsets only.
    m_centerIDs = {18, 19, 20, 21, 24, 27};
    m_targetOffsetsByTag = {
        {18, {.txDeg = 0.0, .tyDeg = -1.0}}, {19, {.txDeg = 0.8, .tyDeg = -0.7}},
        {20, {.txDeg = -0.6, .tyDeg = -0.9}}, {21, {.txDeg = 0.0, .tyDeg = -0.6}},
        {24, {.txDeg = 0.4, .tyDeg = -1.2}}, {27, {.txDeg = -0.7, .tyDeg = -1.0}}};
  }
}
