# 2026 FRC Shooter Software Plan (Canada Welded Field)

This plan is based on the current code in this repository and extends it with a low-risk path to a distance-based shooter planner.

## 1) Current code snapshot (what already exists)

- **ShooterSubsystem** currently runs shooter motors in open-loop percentage mode (`SetPercent`, `SpinDrivingMotors`, `StopDrivingMotors`).
- A **turning motor** for shooter angle is declared, configured, and available in hardware mapping, but not yet used in a closed-loop aiming workflow.
- **Vision plumbing** already exists through `VisionIO`, `VisionIOLimelight`, and `VisionIOSim`, and `RobotContainer` constructs one of these based on real robot vs sim.
- The robot has a working **swerve drive base** and autonomous command that already uses AprilTag concepts (`AutoDriveToTagPose`).

This means the best near-term architecture is to add a **Shooter Planner layer** that computes setpoints first, then integrate those setpoints into new closed-loop shooter controls.

## 2) Software architecture for the fixed-camera + pivoting-shooter setup

Because the Limelight 3A is mounted below the shooter and does not move with hood/elevation:

1. **Vision Range Estimation Module** (camera frame -> target range)
   - Produces `rangeMeters` to the hub/goal.
   - Prefer AprilTag pose-based range if available.
   - Fallback: `ty` geometry-based range.

2. **Shot Planner / Kinematics Module** (range -> hood angle + wheel rpm)
   - Input: range.
   - Output: `(hoodAngle, flywheelRPM)`.
   - Uses interpolation map built from real shot data.

3. **Mechanism Control Module**
   - Hood/elevator angle loop (position control).
   - Flywheel velocity loop (RPM control).
   - Readiness gate (`atAngle && atRPM` for several cycles) before feeding.

## 3) New code added in this branch

### `ShotPlanner` utility

A new lightweight interpolation planner has been added:

- `src/main/include/shooter/ShotPlanner.h`
- `src/main/cpp/shooter/ShotPlanner.cpp`

Features:
- Stores an ordered distance map of `ShotPlanPoint` values.
- Solves for interpolated setpoints via `Solve(range)`.
- Clamps to nearest endpoint when range is outside map.
- Returns `std::optional` so caller can safely handle missing map cases.

This is intentionally decoupled from existing subsystems so you can integrate with minimal disruption.

## 4) Suggested implementation phases (minimal-risk)

### Phase 1 - Data and plumbing
- Keep existing shooter behavior intact.
- Add telemetry for:
  - current range estimate,
  - requested hood angle,
  - requested flywheel RPM,
  - actual hood angle and flywheel RPM.

### Phase 2 - Flywheel velocity closed loop
- Keep intake/feed behavior unchanged.
- Replace open-loop shooter percent with closed-loop velocity hold.
- Add feedforward + PID.

### Phase 3 - Hood/elevation closed loop
- Use absolute/relative encoder for shooter pivot.
- Add profiled position control to reach target angle smoothly.

### Phase 4 - Readiness gate and safe feed
- Implement `IsReadyToShoot()`:
  - target visible/valid,
  - flywheel at speed tolerance,
  - hood at angle tolerance,
  - stable for N cycles.
- Only feed when ready.

### Phase 5 - Characterization & map tuning
- Shoot from multiple marked distances.
- Tune map points until hit rate is consistent.
- Update `ShotPlanner` map constants with measured values.

## 5) Recommended WPILib classes to use (verified for 2026 line)

The following classes are appropriate for this workflow and should be available in 2026 WPILib:

- `frc::PIDController`
- `frc::ProfiledPIDController`
- `frc::TrapezoidProfile`
- `frc::SimpleMotorFeedforward`
- `frc::SwerveDrivePoseEstimator`
- `frc::SwerveDriveKinematics`
- `frc::ChassisSpeeds`
- `frc2::PIDCommand` / `frc2::PIDSubsystem` (optional command-based wrappers)

Repository evidence that this project is on 2026 tooling:
- `GradleRIO` plugin version is `2026.2.1`.

## 6) Integration points in this repository

1. **Range source**
   - Extend `VisionIO` interface to expose cleaned range (or robot pose if using tag pose fusion).

2. **Shooter subsystem**
   - Add methods like:
     - `SetFlywheelRPM(units::revolutions_per_minute_t rpm)`
     - `SetHoodAngle(units::radian_t angle)`
     - `AtFlywheelSetpoint()`, `AtHoodSetpoint()`

3. **Coordinator command**
   - New command `AimAndSpinUp`:
     - reads range,
     - calls `ShotPlanner::Solve(range)`,
     - applies setpoints.

4. **Fire command**
   - `ShootWhenReady` waits for readiness gate before feeding ball.

## 7) Why this plan fits your robot constraints

- Camera is fixed: no dynamic camera-to-hood compensation needed.
- Shooter pivots independently: planner can directly output hood angle.
- Current code already has the right separation (vision/drivetrain/shooter subsystems), so this adds functionality with minimal invasive changes.


## 8) Hallucination check for *new code in this branch*

Only these new software artifacts were added in code:

- `ShotPlanPoint`
- `ShotPlanSolution`
- `ShotPlanner`

These are team-defined classes/structs in this repository (not WPILib APIs), so there is no risk of a "wrong WPILib class name" for those types.

For dependencies used directly by the new code, documentation locations are:

- `std::optional` and `std::vector`: C++ reference (`cppreference.com`)
- WPILib C++ units literals/types (e.g., `units::meter_t`, `units::radian_t`, `units::revolutions_per_minute_t`): WPILib C++ units docs and generated C++ API docs

Suggested docs pages:

- https://en.cppreference.com/w/cpp/utility/optional
- https://en.cppreference.com/w/cpp/container/vector
- https://docs.wpilib.org/en/stable/docs/software/basic-programming/cpp-units.html
- https://github.wpilib.org/allwpilib/docs/release/cpp/

## 9) How the new `ShotPlanner` works internally

1. **Stores a shot map** as ordered points of `(range, hood angle, RPM)`.
2. **Sorts input points** in `SetMap()` so call sites can provide data in any order.
3. In `Solve(range)`:
   - returns `nullopt` if map is empty,
   - clamps to first/last point when outside bounds,
   - linearly interpolates between nearest bounding points when in bounds.
4. Returns a `ShotPlanSolution` with a `clampedToMap` flag so commands can choose whether to shoot or warn drivers.
