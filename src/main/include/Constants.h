// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 3.5_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{0.5 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
constexpr units::meter_t kTrackWidth =
    0.617_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.617_m;  // Distance between centers of front and back wheels on robot

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = 3.483;
constexpr double kFrontRightChassisAngularOffset = 4.31;
constexpr double kRearLeftChassisAngularOffset = 3.262;
constexpr double kRearRightChassisAngularOffset = 5.629;

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId = 14;  //14
constexpr int kRearLeftDrivingCanId = 13; ///13
constexpr int kFrontRightDrivingCanId = 11; //11
constexpr int kRearRightDrivingCanId = 12; //12

constexpr int kFrontLeftTurningCanId = 18; //18
constexpr int kRearLeftTurningCanId = 17; //17
constexpr int kFrontRightTurningCanId = 15; //15
constexpr int kRearRightTurningCanId = 16; //16

constexpr int kIntakeCanID = 23; //23

constexpr int kIndexerCanID = 26; // May not exist

constexpr int kShooterDriving1 = 21; //21
constexpr int kShooterDriving2 = 22;
constexpr int kShooterTurret = 25;
constexpr int kShooterHood = 24;
}  // namespace DriveConstants

namespace ModuleConstants {
// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 13;
// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps = 
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr double kDriveDeadband = 0.1;
}  // namespace OIConstants
