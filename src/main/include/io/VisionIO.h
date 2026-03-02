// VisionIO.h
#pragma once
#include <optional>
#include <frc/geometry/Pose2d.h>
#include <photon/PhotonPoseEstimator.h>

class VisionIO {
public:
    virtual ~VisionIO() = default;

    // Returns optional pose estimate based on current robot estimate
    virtual std::optional<photon::EstimatedRobotPose> GetEstimatedPose(frc::Pose2d currentEstimate) = 0;
};