#include "subsystems/ShootingAutoAim.h"

#include <algorithm>
#include <utility>

#include <frc/DriverStation.h>

#include "LimelightHelpers.h"
#include "subsystems/ShooterSubsystem.h"

ShootingAutoAim::ShootingAutoAim(std::string limelightName)
    : m_ll(std::move(limelightName)),
      m_turretPID(0.03, 0.0, 0.002),
      m_hoodPID(0.025, 0.0, 0.0015) {
  m_turretPID.SetTolerance(1.0);
  m_hoodPID.SetTolerance(1.0);
  UpdateAllianceTagIDs();
}

void ShootingAutoAim::Initialize() {
  UpdateAllianceTagIDs();
  m_turretPID.Reset();
  m_hoodPID.Reset();
  m_lastBestId = -1;
  m_loggedNoTarget = false;
  LimelightHelpers::SetFiducialIDFiltersOverride(m_ll, m_centerIDs);
}

void ShootingAutoAim::End() {
  m_lastBestId = -1;
  m_turretPID.Reset();
  m_hoodPID.Reset();
}

bool ShootingAutoAim::HasValidTarget() const {
  return LimelightHelpers::getTV(m_ll);
}

void ShootingAutoAim::UpdateAim(ShooterSubsystem& shooter) {
  if (!LimelightHelpers::getTV(m_ll)) {
    shooter.StopTurretMotor();
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
  LimelightHelpers::setPriorityTagID(m_ll, bestTag.value());

  const double tx = LimelightHelpers::getTX(m_ll);
  const double ty = LimelightHelpers::getTY(m_ll);

  double turretCmd = m_turretPID.Calculate(tx, aimOffsets->txDeg);
  turretCmd = std::clamp(turretCmd, -0.6, 0.6);

  double hoodCmd = m_hoodPID.Calculate(ty, aimOffsets->tyDeg);
  hoodCmd = std::clamp(hoodCmd, -0.45, 0.45);

  shooter.SetTurretPercent(turretCmd);
  shooter.SetHoodPercent(hoodCmd);
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
  if (alliance && alliance.value() == frc::DriverStation::Alliance::kBlue) {
    m_centerIDs = {18, 19, 20, 21, 24, 27};
    m_targetOffsetsByTag = {
        {18, {.txDeg = 0.0, .tyDeg = -1.0}}, {19, {.txDeg = -0.8, .tyDeg = -0.7}},
        {20, {.txDeg = 0.6, .tyDeg = -0.9}}, {21, {.txDeg = 0.0, .tyDeg = -0.6}},
        {24, {.txDeg = -0.4, .tyDeg = -1.2}}, {27, {.txDeg = 0.7, .tyDeg = -1.0}}};
  } else {
    m_centerIDs = {8, 9, 10, 11, 2, 5};
    m_targetOffsetsByTag = {
        {8, {.txDeg = 0.0, .tyDeg = -1.0}}, {9, {.txDeg = 0.8, .tyDeg = -0.7}},
        {10, {.txDeg = -0.6, .tyDeg = -0.9}}, {11, {.txDeg = 0.0, .tyDeg = -0.6}},
        {2, {.txDeg = 0.4, .tyDeg = -1.2}}, {5, {.txDeg = -0.7, .tyDeg = -1.0}}};
  }
}
