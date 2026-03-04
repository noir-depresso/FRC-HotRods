#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <optional>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"

/**
 * Drives to a field pose while optionally routing around a circular obstacle.
 *
 * Note: This command relies on DriveSubsystem pose, which in your project is
 * corrected by AprilTag vision in autonomous.
 */
class AutoDriveToFieldPoseSafe
    : public frc2::CommandHelper<frc2::Command, AutoDriveToFieldPoseSafe> {
 public:
  AutoDriveToFieldPoseSafe(DriveSubsystem* drive,
                           frc::Pose2d finalGoal,
                           frc::Translation2d obstacleCenter,
                           units::meter_t obstacleRadius,
                           units::meter_t clearance)
      : m_drive(drive),
        m_finalGoal(finalGoal),
        m_obstacleCenter(obstacleCenter),
        m_obstacleRadius(obstacleRadius),
        m_clearance(clearance),
        m_xPid(1.3, 0.0, 0.0),
        m_yPid(1.3, 0.0, 0.0),
        m_thetaPid(2.5, 0.0, 0.0) {
    AddRequirements({m_drive});
    m_thetaPid.EnableContinuousInput(-180.0, 180.0);
  }

  void Initialize() override {
    m_timer.Reset();
    m_timer.Start();
    m_waypoints.clear();
    m_currentIndex = 0;

    const frc::Pose2d start = m_drive->GetPose();
    const auto maybeWaypoint = ComputeAvoidanceWaypoint(start.Translation(), m_finalGoal.Translation());
    if (maybeWaypoint.has_value()) {
      m_waypoints.emplace_back(maybeWaypoint.value(), m_finalGoal.Rotation());
    }
    m_waypoints.push_back(m_finalGoal);

    m_xPid.SetTolerance(0.20);
    m_yPid.SetTolerance(0.20);
    m_thetaPid.SetTolerance(8.0);
  }

  void Execute() override {
    if (m_currentIndex >= static_cast<int>(m_waypoints.size())) {
      m_finished = true;
      return;
    }

    const frc::Pose2d current = m_drive->GetPose();
    const frc::Pose2d target = m_waypoints[m_currentIndex];

    double vx = m_xPid.Calculate(current.X().value(), target.X().value());
    double vy = m_yPid.Calculate(current.Y().value(), target.Y().value());
    const double omegaDegPerSec = m_thetaPid.Calculate(
        current.Rotation().Degrees().value(), target.Rotation().Degrees().value());

    vx = std::clamp(vx, -m_maxLinearMps, m_maxLinearMps);
    vy = std::clamp(vy, -m_maxLinearMps, m_maxLinearMps);

    const auto omega = units::radians_per_second_t{
        units::degrees_per_second_t{omegaDegPerSec}};

    m_drive->Drive(units::meters_per_second_t{vx},
                   units::meters_per_second_t{vy},
                   omega,
                   true);

    if (m_xPid.AtSetpoint() && m_yPid.AtSetpoint() && m_thetaPid.AtSetpoint()) {
      ++m_currentIndex;
      if (m_currentIndex >= static_cast<int>(m_waypoints.size())) {
        m_finished = true;
      }
    }
  }

  bool IsFinished() override { return m_finished || m_timer.Get() > 8_s; }

  void End(bool interrupted) override {
    m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, true);
  }

 private:
  std::optional<frc::Translation2d> ComputeAvoidanceWaypoint(frc::Translation2d start,
                                                              frc::Translation2d goal) {
    const double sx = start.X().value();
    const double sy = start.Y().value();
    const double gx = goal.X().value();
    const double gy = goal.Y().value();
    const double cx = m_obstacleCenter.X().value();
    const double cy = m_obstacleCenter.Y().value();

    const double dx = gx - sx;
    const double dy = gy - sy;
    const double segLen2 = dx * dx + dy * dy;
    if (segLen2 < 1e-6) return std::nullopt;

    double t = ((cx - sx) * dx + (cy - sy) * dy) / segLen2;
    t = std::clamp(t, 0.0, 1.0);

    const double nearestX = sx + t * dx;
    const double nearestY = sy + t * dy;
    const double distX = cx - nearestX;
    const double distY = cy - nearestY;
    const double nearestDist = std::hypot(distX, distY);

    const double safeRadius = m_obstacleRadius.value() + m_clearance.value();
    if (nearestDist >= safeRadius) {
      return std::nullopt;
    }

    const double segLen = std::sqrt(segLen2);
    const double nx = -dy / segLen;
    const double ny = dx / segLen;

    const double side = ((sx - cx) * nx + (sy - cy) * ny) >= 0.0 ? 1.0 : -1.0;
    const double wx = cx + side * nx * safeRadius;
    const double wy = cy + side * ny * safeRadius;

    return frc::Translation2d{units::meter_t{wx}, units::meter_t{wy}};
  }

  DriveSubsystem* m_drive;
  frc::Pose2d m_finalGoal;
  frc::Translation2d m_obstacleCenter;
  units::meter_t m_obstacleRadius;
  units::meter_t m_clearance;

  frc::PIDController m_xPid;
  frc::PIDController m_yPid;
  frc::PIDController m_thetaPid;

  frc::Timer m_timer;
  std::vector<frc::Pose2d> m_waypoints;
  int m_currentIndex = 0;
  bool m_finished = false;

  const double m_maxLinearMps = 1.25;
};
