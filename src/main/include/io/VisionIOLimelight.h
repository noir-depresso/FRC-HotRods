// VisionIOLimelight.h
#pragma once
#include "VisionIO.h"
#include <frc/Timer.h>
#include <networktables/NetworkTableInstance.h>
#include <cscore_oo.h>

class VisionIOLimelight : public VisionIO {
public:
    std::optional<photon::EstimatedRobotPose> GetEstimatedPose(frc::Pose2d currentEstimate) override {
        auto result = LimelightHelpers::GetBotPoseEstimate_wpiBlue("limelight");
        if (result.has_value() && result->targets.size() > 0) {
            photon::EstimatedRobotPose pose{
                result->pose,
                frc::Timer::GetFPGATimestamp(),
                static_cast<int>(result->targets.size())
            };
            return pose;
        }
        return std::nullopt;
    }
};
