#include "subsystems/ShootingAutoAim.h"

#include <algorithm>
#include <utility>

#include <frc/DriverStation.h>

#include "LimelightHelpers.h"

ShootingAutoAim::ShootingAutoAim(std::string limelightName)
    : m_ll(std::move(limelightName)), m_turnPID(0.03, 0.0, 0.002) {
  m_turnPID.SetTolerance(1.0);
  UpdateAllianceTagIDs();
}

void ShootingAutoAim::Initialize() {
  UpdateAllianceTagIDs();
  m_turnPID.Reset();
  m_lastBestId = -1;
  m_loggedNoTarget = false;
  LimelightHelpers::SetFiducialIDFiltersOverride(m_ll, m_centerIDs);
}

void ShootingAutoAim::End() {
  m_lastBestId = -1;
  m_turnPID.Reset();
}

std::optional<double> ShootingAutoAim::GetTurnCommand() {
  if (!LimelightHelpers::getTV(m_ll)) {
    if (!m_loggedNoTarget) {
      m_loggedNoTarget = true;
    }
    return std::nullopt;
  }

  m_loggedNoTarget = false;

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

  m_lastBestId = bestId;
  LimelightHelpers::setPriorityTagID(m_ll, bestId);

  const double tx = LimelightHelpers::getTX(m_ll);
  double turnCmd = m_turnPID.Calculate(tx, 0.0);
  turnCmd = std::clamp(turnCmd, -0.6, 0.6);
  return turnCmd;
}

void ShootingAutoAim::UpdateAllianceTagIDs() {
  const auto alliance = frc::DriverStation::GetAlliance();
  if (alliance && alliance.value() == frc::DriverStation::Alliance::kBlue) {
    m_centerIDs = {18, 19, 20, 21, 24, 27};
  } else {
    m_centerIDs = {8, 9, 10, 11, 2, 5};
  }
}
