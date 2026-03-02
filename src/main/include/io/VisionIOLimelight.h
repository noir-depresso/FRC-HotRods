// VisionIOLimelight.h
#pragma once
#include "VisionIO.h"
#include <LimelightHelpers.h>

class VisionIOLimelight : public VisionIO {
public:
    std::optional<photon::EstimatedRobotPose> GetEstimatedPose(frc::Pose2d /*currentEstimate*/) override {
        auto estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight");
        if (!LimelightHelpers::validPoseEstimate(estimate)) {
            return std::nullopt;
        }

        return photon::EstimatedRobotPose{
            frc::Pose3d{estimate.pose},
            estimate.timestampSeconds,
            {},
            photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR
        };
    }
};