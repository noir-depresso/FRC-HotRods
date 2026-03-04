#include "shooter/ShotPlanner.h"

#include <algorithm>
#include <utility>

using namespace units::literals;

namespace {

constexpr auto Lerp(double t, double a, double b) {
  // Linear interpolation helper used for both angle and flywheel RPM.
  return a + (t * (b - a));
}

}  // namespace

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
