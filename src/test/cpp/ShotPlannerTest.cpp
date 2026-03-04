#include "gtest/gtest.h"

#include <units/math.h>

#include "shooter/ShotPlanner.h"

namespace {

using namespace units::literals;

TEST(ShotPlannerTest, InterpolatesWithinRange) {
  // Midpoint query should produce midpoint angle/RPM when map is linear.
  ShotPlanner planner;
  planner.SetMap({
      {2.0_m, units::degree_t{20.0}, 3000_rpm},
      {4.0_m, units::degree_t{40.0}, 5000_rpm},
  });

  const auto result = planner.Solve(3.0_m);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->clampedToMap);
  EXPECT_NEAR(units::degree_t{result->hoodAngle}.value(), 30.0, 1e-9);
  EXPECT_NEAR(result->flywheelRpm.value(), 4000.0, 1e-9);
}

TEST(ShotPlannerTest, ClampsBelowMinimumRange) {
  // Out-of-range low should clamp to first point and mark as clamped.
  ShotPlanner planner;
  planner.SetMap({
      {2.0_m, units::degree_t{20.0}, 3000_rpm},
      {4.0_m, units::degree_t{40.0}, 5000_rpm},
  });

  const auto result = planner.Solve(1.0_m);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->clampedToMap);
  EXPECT_NEAR(units::degree_t{result->hoodAngle}.value(), 20.0, 1e-9);
  EXPECT_NEAR(result->flywheelRpm.value(), 3000.0, 1e-9);
}

TEST(ShotPlannerTest, SortsMapBeforeSolving) {
  // SetMap() accepts unsorted inputs and normalizes ordering internally.
  ShotPlanner planner;
  planner.SetMap({
      {4.0_m, units::degree_t{40.0}, 5000_rpm},
      {2.0_m, units::degree_t{20.0}, 3000_rpm},
  });

  const auto result = planner.Solve(3.0_m);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->clampedToMap);
  EXPECT_NEAR(units::degree_t{result->hoodAngle}.value(), 30.0, 1e-9);
  EXPECT_NEAR(result->flywheelRpm.value(), 4000.0, 1e-9);
}


TEST(ShotPlannerTest, SolveLaunchAngleReturnsReachableLowAndHighArcs) {
  const auto lowArc = ShotPlanner::SolveLaunchAngle(4.0_m, 1.5_m, 10.0_mps, false);
  const auto highArc = ShotPlanner::SolveLaunchAngle(4.0_m, 1.5_m, 10.0_mps, true);

  ASSERT_TRUE(lowArc.has_value());
  ASSERT_TRUE(highArc.has_value());
  EXPECT_LT(lowArc->value(), highArc->value());
  EXPECT_GT(units::degree_t{*lowArc}.value(), 0.0);
}

TEST(ShotPlannerTest, SolveLaunchAngleReturnsNullForUnreachableTarget) {
  const auto unreachable =
      ShotPlanner::SolveLaunchAngle(9.0_m, 3.5_m, 5.0_mps, false);

  EXPECT_FALSE(unreachable.has_value());
}

}
