#pragma once

#include <frc/controller/PIDController.h>

#include <unordered_map>
#include <optional>
#include <string>
#include <vector>

class ShooterSubsystem;
class DriveSubsystem;

class ShootingAutoAim {
 public:
  ShootingAutoAim(std::string limelightName, DriveSubsystem& drive);

  // Apply alliance tag filters and reset aim controllers.
  void Initialize();
  void End();

  // Computes turret + hood commands and sends them to ShooterSubsystem.
  // This aims to a tag-relative offset, not the tag center itself.
  void UpdateAim(ShooterSubsystem& shooter);
  bool HasValidTarget() const;

 private:
  struct AimOffsets {
    // Camera-space setpoints in Limelight angular units (degrees).
    double txDeg = 0.0;
    double tyDeg = 0.0;
  };

  // Builds allowed tag list + per-tag offsets for the current alliance.
  void UpdateAllianceTagIDs();
  // Chooses the most reliable visible allowed tag.
  std::optional<int> SelectBestTag() const;
  std::optional<AimOffsets> GetAimOffsets(int tagId) const;
  std::optional<double> ComputePosePreAimTurretCommand();

  std::string m_ll;
  DriveSubsystem& m_drive;
  std::vector<int> m_centerIDs;
  std::unordered_map<int, AimOffsets> m_targetOffsetsByTag;
  frc::PIDController m_turretPID;
  frc::PIDController m_hoodPID;
  frc::PIDController m_poseTurretPID;
  int m_lastBestId{-1};
  bool m_loggedNoTarget{false};
};
