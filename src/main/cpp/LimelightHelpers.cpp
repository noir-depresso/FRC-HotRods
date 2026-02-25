// // namespace LimelightHelpers
// #include "LimelightHelpers.h"
// #include <array>
// #include <cmath>

// namespace LimelightHelpers {

// static std::shared_ptr<nt::NetworkTable> GetTable(const std::string& name) {
//   return nt::NetworkTableInstance::GetDefault().GetTable(name);
// }

// bool getTV(const std::string& limelightName) {
//   auto t = GetTable(limelightName);
//   return t->GetNumber("tv", 0.0) >= 1.0;
// }
// void SetRobotOrientation(const std::string& limelightName,
//                          double robotYawDeg,
//                          double robotPitchDeg,
//                          double robotRollDeg,
//                          double robotYawRateDegPerSec,
//                          double robotPitchRateDegPerSec,
//                          double robotRollRateDegPerSec) {
//   auto t = GetTable(limelightName);

//   const std::array<double, 6> data{
//       robotYawDeg, robotPitchDeg, robotRollDeg,
//       robotYawRateDegPerSec, robotPitchRateDegPerSec, robotRollRateDegPerSec
//   };

//   t->PutNumberArray("robot_orientation_set", data);
// }

// PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(const std::string& limelightName) {
//   PoseEstimate out;
//   auto t = GetTable(limelightName);

//   auto arr = t->GetNumberArray("botpose_orb_wpiblue", {});

//   if (arr.size() < 7 || !getTV(limelightName)) {
//     out.tagCount = 0;
//     return out;
//   }

//   const double x = arr[0];
//   const double y = arr[1];
//   const double yawDeg = arr[5];
//   const double latencyMs = arr[6];

//   if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yawDeg)) {
//     out.tagCount = 0;
//     return out;
//   }

//   out.pose = frc::Pose2d{units::meter_t{x}, units::meter_t{y},
//                         frc::Rotation2d{units::degree_t{yawDeg}}};
//   out.latencyMs = latencyMs;

//   out.tagCount = (arr.size() >= 8) ? static_cast<int>(arr[7]) : 1;
//   out.avgTagDistMeters = (arr.size() >= 10) ? arr[9] : -1.0;

//   const double now = frc::Timer::GetFPGATimestamp().value();
//   out.timestampSeconds = now - (latencyMs / 1000.0);

//   return out;
// }

// } 