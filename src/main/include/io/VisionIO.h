// VisionIO.h
#pragma once

#include <optional>
#include <frc/geometry/Pose2d.h>
#include <photonlib/EstimatedRobotPose.h>

class VisionIO {
public:
    virtual ~VisionIO() = default;

    // Returns optional pose estimate based on current robot estimate
    virtual std::optional<photonlib::EstimatedRobotPose> GetEstimatedPose(frc::Pose2d currentEstimate) = 0;
};