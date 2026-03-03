#include "gtest/gtest.h"

#include <units/math.h>

#include "shooter/ShotPlanner.h"

namespace {

using namespace units::literals;

TEST(ShotPlannerTest, InterpolatesWithinRange) {
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

}  // namespace
