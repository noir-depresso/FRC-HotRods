#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/velocity.h>

#include <optional>
#include <vector>

struct ShotPlanPoint {
  units::meter_t range;
  units::radian_t hoodAngle;
  units::revolutions_per_minute_t flywheelRpm;
};

struct ShotPlanSolution {
  units::meter_t range;
  units::radian_t hoodAngle;
  units::revolutions_per_minute_t flywheelRpm;
  bool clampedToMap;
};

class ShotPlanner {
 public:
  ShotPlanner();

   // Solves projectile launch angle (theta) to hit (x, y) with speed v0.
  // Coordinate system assumes +y is up and acceleration is negative.
  // Returns nullopt when the shot is physically unreachable.
  static std::optional<units::radian_t> SolveLaunchAngle(
      units::meter_t x, units::meter_t y, units::meters_per_second_t v0,
      bool preferHighArc, units::meters_per_second_squared_t accel = -9.81_mps_sq);


  void SetMap(std::vector<ShotPlanPoint> points);
  std::optional<ShotPlanSolution> Solve(units::meter_t range) const;

  const std::vector<ShotPlanPoint>& GetMap() const;

 private:
  std::vector<ShotPlanPoint> m_points;
};
