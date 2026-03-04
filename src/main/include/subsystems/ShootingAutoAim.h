#pragma once

#include <frc/controller/PIDController.h>

#include <optional>
#include <string>
#include <vector>

class ShootingAutoAim {
 public:
  explicit ShootingAutoAim(std::string limelightName);

  void Initialize();
  void End();

  // Returns a normalized turn command in [-0.6, 0.6] when a valid tag is found.
  std::optional<double> GetTurnCommand();

 private:
  void UpdateAllianceTagIDs();

  std::string m_ll;
  std::vector<int> m_centerIDs;
  frc::PIDController m_turnPID;
  int m_lastBestId{-1};
  bool m_loggedNoTarget{false};
};
