// VisionIOSim.h
#pragma once

#include "VisionIO.h"

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/PhotonCameraSim.h>
#include <photon/simulation/VisionSystemSim.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>

#include <memory>
#include <optional>

class VisionIOSim : public VisionIO {
 public:
  VisionIOSim()
      : camera("simCam"),
        robotToCam(frc::Translation3d(0.3_m, 0.0_m, 0.5_m), frc::Rotation3d()) {
    // Load official 2026 field layout.
    fieldLayout = frc::AprilTagFieldLayout::LoadField(
        frc::AprilTagField::k2026RebuiltWelded);

    // Create pose estimator using current PhotonPoseEstimator constructor.
    poseEstimator =
        std::make_unique<photon::PhotonPoseEstimator>(fieldLayout, robotToCam);

    // Set up vision simulation system.
    visionSim = std::make_unique<photon::VisionSystemSim>("main");
    cameraSim = std::make_unique<photon::PhotonCameraSim>(&camera);

    visionSim->AddCamera(cameraSim.get(), robotToCam);
    visionSim->AddAprilTags(fieldLayout);
  }

  void UpdateSim(frc::Pose2d robotPose) { visionSim->Update(robotPose); }

  std::optional<photon::EstimatedRobotPose> GetEstimatedPose(
      frc::Pose2d currentEstimate) override {
    (void)currentEstimate;

    auto results = camera.GetAllUnreadResults();
    if (results.empty()) {
      return std::nullopt;
    }

    // Use the newest unread frame once per loop.
    return poseEstimator->EstimateCoprocMultiTagPose(results.back());
  }

 private:
  photon::PhotonCamera camera;

  std::unique_ptr<photon::PhotonCameraSim> cameraSim;
  std::unique_ptr<photon::PhotonPoseEstimator> poseEstimator;
  std::unique_ptr<photon::VisionSystemSim> visionSim;

  frc::Transform3d robotToCam;
  frc::AprilTagFieldLayout fieldLayout;
};
