#pragma once

#include <frc/controller/PIDController.h>

#include <unordered_map>
#include <optional>
#include <string>
#include <vector>

class ShooterSubsystem;

class ShootingAutoAim {
 public:
  explicit ShootingAutoAim(std::string limelightName);

  void Initialize();
  void End();
  void UpdateAim(ShooterSubsystem& shooter);
  bool HasValidTarget() const;

 private:
  struct AimOffsets {
    double txDeg = 0.0;
    double tyDeg = 0.0;
  };

  void UpdateAllianceTagIDs();
  std::optional<int> SelectBestTag() const;
  std::optional<AimOffsets> GetAimOffsets(int tagId) const;

  std::string m_ll;
  std::vector<int> m_centerIDs;
  std::unordered_map<int, AimOffsets> m_targetOffsetsByTag;
  frc::PIDController m_turretPID;
  frc::PIDController m_hoodPID;
  int m_lastBestId{-1};
  bool m_loggedNoTarget{false};
};
