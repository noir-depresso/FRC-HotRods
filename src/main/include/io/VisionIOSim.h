// VisionIOSim.h
#pragma once
#include "VisionIO.h"
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <photonlib/simulation/PhotonCameraSim.h>
#include <photonlib/simulation/VisionSystemSim.h>
#include <frc/geometry/Transform3d.h>

class VisionIOSim : public VisionIO {
public:
    VisionIOSim() 
        : camera("simCam"),
          robotToCam(frc::Translation3d(0.3_m, 0.0_m, 0.5_m), frc::Rotation3d())
    {
        fieldLayout = AprilTagFieldLayout::LoadField(AprilTagFields::k2026Field);

        poseEstimator = std::make_unique<photonlib::PhotonPoseEstimator>(
            *fieldLayout,
            photonlib::PoseStrategy::MULTI_TAG_PNP,
            robotToCam
        );

        visionSim = std::make_unique<photonlib::sim::VisionSystemSim>("main");
        visionSim->AddCamera(
            std::make_shared<photonlib::sim::PhotonCameraSim>(camera),
            robotToCam
        );
        visionSim->AddAprilTags(*fieldLayout);
    }

    void UpdateSim(frc::Pose2d robotPose) {
        visionSim->Update(robotPose);
    }

    std::optional<photonlib::EstimatedRobotPose> GetEstimatedPose(frc::Pose2d currentEstimate) override {
        auto result = camera.GetLatestResult();
        return poseEstimator->Update(result);
    }

private:
    photonlib::PhotonCamera camera;
    std::unique_ptr<photonlib::PhotonPoseEstimator> poseEstimator;
    std::unique_ptr<photonlib::sim::VisionSystemSim> visionSim;
    frc::Transform3d robotToCam;
    std::shared_ptr<AprilTagFieldLayout> fieldLayout;
};