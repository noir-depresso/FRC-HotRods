#include "subsystems/ShootingAutoAim.h"

#include "LimelightHelpers.h"

#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <fmt/core.h>

#include <algorithm>

ShootingAutoAim::ShootingAutoAim(std::string limelightName)
    : m_ll(std::move(limelightName)),
      m_turnPID(0.03, 0.0, 0.002) {
  m_turnPID.SetTolerance(1.0);  // degrees
}

void ShootingAutoAim::UpdateAllianceTagIDs() {
  // Defaults to red set if alliance is unknown.
  m_centerIDs = {8, 9, 10, 11, 2, 5};

  const auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kBlue) {
    m_centerIDs = {18, 19, 20, 21, 24, 27};
  }
}

void ShootingAutoAim::Initialize() {
  UpdateAllianceTagIDs();
  LimelightHelpers::SetFiducialIDFiltersOverride(m_ll, m_centerIDs);
  m_lastBestId = -1;
  m_loggedNoTarget = false;

  const auto msg = fmt::format("AutoAim ON: scanning tags with {} IDs", m_centerIDs.size());
  fmt::print("{}\n", msg);
  FRC_ReportWarning("{}", msg);
}

std::optional<double> ShootingAutoAim::GetTurnCommand() {
  // Keep filters synced with alliance in case FMS alliance appears after boot.
  UpdateAllianceTagIDs();
  LimelightHelpers::SetFiducialIDFiltersOverride(m_ll, m_centerIDs);

  if (!LimelightHelpers::getTV(m_ll)) {
    if (!m_loggedNoTarget) {
      constexpr const char* kMsg = "AutoAim: no AprilTag target visible";
      fmt::print("{}\n", kMsg);
      FRC_ReportWarning("{}", kMsg);
      m_loggedNoTarget = true;
    }
    return std::nullopt;
  }

  auto fiducials = LimelightHelpers::getRawFiducials(m_ll);
  if (fiducials.empty()) {
    if (!m_loggedNoTarget) {
      constexpr const char* kMsg = "AutoAim: Limelight has TV but no raw fiducials";
      fmt::print("{}\n", kMsg);
      FRC_ReportWarning("{}", kMsg);
      m_loggedNoTarget = true;
    }
    return std::nullopt;
  }

  int bestId = -1;
  double bestScore = 1e9;

  for (const auto& f : fiducials) {
    if (std::find(m_centerIDs.begin(), m_centerIDs.end(), f.id) ==
        m_centerIDs.end()) {
      continue;
    }

    const double score = f.ambiguity + 0.05 * f.distToRobot;
    if (score < bestScore) {
      bestScore = score;
      bestId = f.id;
    }
  }

  if (bestId == -1) {
    if (!m_loggedNoTarget) {
      constexpr const char* kMsg = "AutoAim: no allowed target ID in current view";
      fmt::print("{}\n", kMsg);
      FRC_ReportWarning("{}", kMsg);
      m_loggedNoTarget = true;
    }
    return std::nullopt;
  }

  m_loggedNoTarget = false;

  LimelightHelpers::setPriorityTagID(m_ll, bestId);
  if (bestId != m_lastBestId) {
    const auto msg = fmt::format("AutoAim: locked AprilTag {}", bestId);
    fmt::print("{}\n", msg);
    FRC_ReportWarning("{}", msg);
    m_lastBestId = bestId;
  }

  const double tx = LimelightHelpers::getTX(m_ll);  // degrees
  double turnCmd = m_turnPID.Calculate(tx, 0.0);
  turnCmd = std::clamp(turnCmd, -0.6, 0.6);

  return turnCmd;
}

void ShootingAutoAim::End() {
  LimelightHelpers::SetFiducialIDFiltersOverride(m_ll, {});
  m_lastBestId = -1;
  m_loggedNoTarget = false;
  constexpr const char* kMsg = "AutoAim OFF: stopped scanning tags";
  fmt::print("{}\n", kMsg);
  FRC_ReportWarning("{}", kMsg);
}
