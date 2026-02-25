#include "LimelightHelpers.h"

#include <frc/controller/PIDController.h>
#include <algorithm>
#include <string>
#include <vector>

namespace {
class AimAtTags {
 public:
  AimAtTags(std::string limelightName, std::vector<int> centerTagIDs)
      : m_ll(std::move(limelightName)),
        m_centerIDs(std::move(centerTagIDs)),
        m_turnPID(0.03, 0.0, 0.002) {
    m_turnPID.SetTolerance(1.0);
  }

  void Initialize() {
    // Only allow these IDs (prevents random tags from being used)
    LimelightHelpers::SetFiducialIDFiltersOverride(m_ll, m_centerIDs);
  }

  void Execute(/* your DriveSubsystem& drive */) {
    if (!LimelightHelpers::getTV(m_ll)) {
      // drive.Drive(0,0,0);
      return;
    }

    // Get all visible tags (with distance + ambiguity)
    auto fiducials = LimelightHelpers::getRawFiducials(m_ll);
    if (fiducials.empty()) return;

    // Pick the best tag among allowed IDs:
    // - prefer low ambiguity
    // - prefer closer distance
    int bestId = -1;
    double bestScore = 1e9;

    for (const auto& f : fiducials) {
      // only accept IDs we care about
      if (std::find(m_centerIDs.begin(), m_centerIDs.end(), f.id) == m_centerIDs.end())
        continue;

      // score: ambiguity + small weight on distance
      double score = f.ambiguity + 0.05 * f.distToRobot;
      if (score < bestScore) {
        bestScore = score;
        bestId = f.id;
      }
    }

    if (bestId == -1) return;

    // Tell Limelight to use this tag for tx/ty tracking
    LimelightHelpers::setPriorityTagID(m_ll, bestId);

    // Now use tx from Limelight to rotate toward target
    double tx = LimelightHelpers::getTX(m_ll);  // degrees
    double turnCmd = m_turnPID.Calculate(tx, 0.0);

    // Clamp and apply to drivetrain/turret
    turnCmd = std::clamp(turnCmd, -0.6, 0.6);

    // Example: rotate in place (replace with your drive API)
    // drive.Drive(0.0, 0.0, turnCmd);

  }

  bool IsFinished() const {
    return m_turnPID.AtSetpoint();
  }

  void End() {
    // drive.Drive(0,0,0);
    // (optional) clear filters if you want normal detection again
    // LimelightHelpers::SetFiducialIDFiltersOverride(m_ll, {});
  }

 private:
  std::string m_ll;
  std::vector<int> m_centerIDs;
  frc::PIDController m_turnPID;
};
}  // namespace
