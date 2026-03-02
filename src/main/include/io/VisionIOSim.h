// VisionIOSim.h
#pragma once
#include "VisionIO.h"
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/PhotonCameraSim.h>
#include <photon/simulation/VisionSystemSim.h>
#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <memory>
#include <optional>

class VisionIOSim : public VisionIO {
public:
    VisionIOSim() 
        : camera("simCam"),
          robotToCam(frc::Translation3d(0.3_m, 0.0_m, 0.5_m), frc::Rotation3d())
    {

std::shared_ptr<frc::AprilTagFieldLayout> fieldLayout;

// in constructor:
fieldLayout = std::make_shared<frc::AprilTagFieldLayout>(
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded)
);

poseEstimator = std::make_unique<photon::PhotonPoseEstimator>(
    fieldLayout,
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
    camera,
    robotToCam
);

        visionSim = std::make_unique<photon::VisionSystemSim>("main");
        cameraSim = std::make_unique<photon::PhotonCameraSim>(&camera);
        visionSim->AddCamera(
            cameraSim.get(),
            robotToCam
        );
        visionSim->AddAprilTags(*fieldLayout);
    }

    void UpdateSim(frc::Pose2d robotPose) {
        visionSim->Update(robotPose);
    }

    std::optional<photon::EstimatedRobotPose> GetEstimatedPose(frc::Pose2d currentEstimate) override {
        auto result = camera.GetLatestResult();
        return poseEstimator->Update(result);
    }

private:
    photon::PhotonCamera camera;
    std::unique_ptr<photon::PhotonCameraSim> cameraSim;
    std::unique_ptr<photon::PhotonPoseEstimator> poseEstimator;
    std::unique_ptr<photon::VisionSystemSim> visionSim;
    frc::Transform3d robotToCam;
    std::shared_ptr<frc::AprilTagFieldLayout> fieldLayout;
};