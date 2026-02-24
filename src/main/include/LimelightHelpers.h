#pragma once

#include <string>
#include <vector>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <networktables/NetworkTableInstance.h>

namespace LimelightHelpers {

/**
 * Limelight MegaTag2 publishes bot pose arrays via NetworkTables.
 * For MegaTag2, Limelight docs list these keys:
 *   botpose_orb_wpiblue, botpose_orb_wpired, botpose_orb
 * and requires SetRobotOrientation(...) to be called every frame. (See docs.)
 */
struct PoseEstimate {
  frc::Pose2d pose;            // field pose (x,y,yaw)
  double timestampSeconds = 0; // capture timestamp (FPGA-time-ish)
  int tagCount = 0;            // number of tags used (0 => invalid)
  double latencyMs = 0;        // total latency in ms (best effort)
  double avgTagDistMeters = -1;
};

/**
 * Tell Limelight your robot orientation for MegaTag2.
 *
 * robotYawDeg: CCW+, 0 deg faces toward red alliance wall under Limelight "orb/wpiBlue" conventions.
 * pitch/roll and rates are optional; keep 0 if you don't have them.
 */
inline void SetRobotOrientation(const std::string& limelightName,
                                double robotYawDeg,
                                double robotPitchDeg,
                                double robotRollDeg,
                                double robotYawRateDegPerSec,
                                double robotPitchRateDegPerSec,
                                double robotRollRateDegPerSec);

/**
 * Get MegaTag2 pose estimate in the "wpiBlue/orb" coordinate frame.
 * Limelight recommends using the blue origin consistently for 2024+.
 */
PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(const std::string& limelightName);

/** Optional convenience: is Limelight currently seeing a valid target? ("tv" == 1) */
bool getTV(const std::string& limelightName);

}  // namespace LimelightHelpers