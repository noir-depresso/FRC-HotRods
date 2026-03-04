// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"

namespace AutoConstants {

// Motion profile limits for heading control in autonomous.
// Watch out: these values must stay consistent with drivetrain capability,
// or the theta controller can command turns the modules cannot physically track.
const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};

}  // namespace AutoConstants
