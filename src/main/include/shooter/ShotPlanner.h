#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

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

  void SetMap(std::vector<ShotPlanPoint> points);
  std::optional<ShotPlanSolution> Solve(units::meter_t range) const;

  const std::vector<ShotPlanPoint>& GetMap() const;

 private:
  std::vector<ShotPlanPoint> m_points;
};
