#include "shooter/ShotPlanner.h"

#include <algorithm>
#include <utility>
#include <cmath>

using namespace units::literals;

namespace {

constexpr auto Lerp(double t, double a, double b) {
  // Linear interpolation helper used for both angle and flywheel RPM.
  return a + (t * (b - a));
}

}  // namespace

std::optional<units::radian_t> ShotPlanner::SolveLaunchAngle(
    units::meter_t x, units::meter_t y, units::meters_per_second_t v0,
    bool preferHighArc, units::meters_per_second_squared_t accel) {
  if (x <= 1e-6_m || v0 <= 1e-6_mps) {
    return std::nullopt;
  }

  const auto v0Squared = v0 * v0;
  const double A = (accel * x * x / (2.0 * v0Squared)).value();
  const double B = x.value();
  const double C = A - y.value();

  const double discriminant = (B * B) - (4.0 * A * C);
  if (discriminant < 0.0 || std::abs(A) < 1e-9) {
    return std::nullopt;
  }

  const double sqrtDisc = std::sqrt(discriminant);
  const double t1 = (-B + sqrtDisc) / (2.0 * A);
  const double t2 = (-B - sqrtDisc) / (2.0 * A);

  const units::radian_t a1{std::atan(t1)};
  const units::radian_t a2{std::atan(t2)};

  const units::radian_t high = (a1 >= a2) ? a1 : a2;
  const units::radian_t low = (a1 >= a2) ? a2 : a1;
  return preferHighArc ? high : low;
}


ShotPlanner::ShotPlanner() {
  // Starter map values are placeholders until on-robot characterization is done.
  // Replace these with measured values from your team's practice field.
  m_points = {
      {1.5_m, units::degree_t{20.0}, 2500_rpm},
      {2.5_m, units::degree_t{28.0}, 3200_rpm},
      {3.5_m, units::degree_t{34.0}, 3900_rpm},
      {4.5_m, units::degree_t{39.0}, 4500_rpm},
  };
}

void ShotPlanner::SetMap(std::vector<ShotPlanPoint> points) {
  // Ensure ascending range order so Solve() can walk neighbors safely.
  std::sort(points.begin(), points.end(),
            [](const ShotPlanPoint& a, const ShotPlanPoint& b) {
              return a.range < b.range;
            });
  m_points = std::move(points);
}

std::optional<ShotPlanSolution> ShotPlanner::Solve(units::meter_t range) const {
  if (m_points.empty()) {
    // No calibration map loaded: caller must handle missing solution.
    return std::nullopt;
  }

  if (m_points.size() == 1) {
    return ShotPlanSolution{range, m_points.front().hoodAngle,
                            m_points.front().flywheelRpm, true};
  }

  if (range <= m_points.front().range) {
    // Clamp low instead of extrapolating to keep commands in known-safe region.
    return ShotPlanSolution{range, m_points.front().hoodAngle,
                            m_points.front().flywheelRpm, true};
  }

  if (range >= m_points.back().range) {
    // Clamp high for the same reason: no uncharacterized extrapolation.
    return ShotPlanSolution{range, m_points.back().hoodAngle,
                            m_points.back().flywheelRpm, true};
  }

  for (size_t i = 0; i < (m_points.size() - 1); ++i) {
    const auto& lower = m_points[i];
    const auto& upper = m_points[i + 1];

    if (range >= lower.range && range <= upper.range) {
      // Interpolate between two characterized points.
      const double span = (upper.range - lower.range).value();
      const double t = span > 0.0 ? ((range - lower.range).value() / span) : 0.0;

      return ShotPlanSolution{
          range,
          units::radian_t{Lerp(t, lower.hoodAngle.value(), upper.hoodAngle.value())},
          units::revolutions_per_minute_t{
              Lerp(t, lower.flywheelRpm.value(), upper.flywheelRpm.value())},
          false,
      };
    }
  }

  return std::nullopt;
}

const std::vector<ShotPlanPoint>& ShotPlanner::GetMap() const { return m_points; }
