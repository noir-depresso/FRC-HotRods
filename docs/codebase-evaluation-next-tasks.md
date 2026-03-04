# Codebase Evaluation and Readjusted Next Tasks (Current Environment)

## What changed in this reassessment
This list was readjusted for the **current development environment** (non-executable `gradlew` file and blocked Gradle distribution download behind proxy), while still prioritizing robot-impactful work.

## Environment snapshot (today)

- `gradlew` is present but is not executable (`-rw-r--r--`), so direct `./gradlew ...` commands fail.
- Running `bash gradlew ...` starts wrapper execution, but Gradle distribution download fails with `HTTP/1.1 403 Forbidden` due to proxy restrictions.
- Result: local CI-like checks are currently blocked unless Gradle is preinstalled/cached or proxy settings are fixed.

## Autonomous code found in the current branch

Yes — autonomous code exists and is active:

- `AutoDriveToTagPose` is implemented with XY/theta PID loops, AprilTag layout lookup, and a timeout.
- `AutoDriveForward` is also present as a simple distance-based command.
- `RobotContainer::GetAutonomousCommand()` currently returns `AutoDriveToTagPose` (tag 7, fixed transform offset).

Because this code is already present, the plan is now explicitly adjusted to include **autonomous hardening** tasks (below) instead of treating auto as only future work.

## Readjusted priority queue

## Priority 0 — Unblock development flow (do first)

1. **Fix build/test execution in this environment**
   - Make wrapper executable (`chmod +x gradlew`) for consistent local usage.
   - Decide one unblock path for Gradle distribution:
     - allow outbound access to `services.gradle.org`, or
     - provide internal mirror/proxy allowlist, or
     - pre-seed Gradle 9.2.1 in build agents/dev images.
   - Add a short `README` section documenting the approved build path for this environment.

2. **Create a minimal “verification contract”**
   - Define required checks before merge (at minimum: compile + unit tests).
   - Add explicit fallback behavior when network is unavailable (e.g., cached Gradle required).

## Priority 1 — Complete shooter MVP loop (highest robot impact)

3. **Finish shooter motor control implementation**
   - `ShooterSubsystem` currently configures 3 motors but only commands one drive motor in active methods.
   - Implement dual-flywheel output commands and re-enable/implement turning motor control path.

4. **Add closed-loop shooter APIs**
   - Add subsystem methods for setpoint-based control:
     - `SetFlywheelRPM(...)`
     - `SetHoodAngle(...)`
     - `AtFlywheelSetpoint()` / `AtHoodSetpoint()`
   - Keep existing open-loop controls as fallback for debugging.

5. **Implement readiness gating**
   - Add a single gate used by feed commands:
     - target validity,
     - flywheel tolerance,
     - hood tolerance,
     - stable-for-N-cycles requirement.

## Priority 2 — Integrate planner + vision plumbing

6. **Wire `ShotPlanner` into command flow**
   - Add command (`AimAndSpinUp`) that:
     - gets range,
     - solves planner,
     - applies shooter setpoints each cycle.
   - Surface planner state + clamped flag to dashboard/logging.

7. **Make `VisionIO` either first-class or remove dead abstraction**
   - `VisionIO` is instantiated in `RobotContainer`, but not consistently consumed by shooter/drivetrain loops.
   - Pick one direction now:
     - fully integrate into estimator/range pipeline, or
     - remove and reintroduce when full estimator integration begins.

## Priority 3 — Autonomous hardening + maintainability

8. **Harden existing autonomous commands**
   - Add autonomous-focused checks for:
     - goal convergence behavior,
     - timeout behavior,
     - missing-tag fallback handling,
     - clamped drive outputs (to avoid unstable command outputs).
   - Expose key auto telemetry (goal pose, current error, command timeout reason).

9. **Modernize command ownership and reduce raw pointers**
   - Migrate autonomous command creation toward `frc2::CommandPtr` patterns.

10. **Gate high-frequency debug logging**
   - Wrap periodic limelight/pose prints with a dashboard or compile-time debug flag to keep DS logs usable in matches.

11. **Retune drivetrain limits for competition**
   - Revisit `kMaxSpeed` and `kMaxAngularSpeed` once driver practice starts.
   - Validate slew limits and control feel under match-like operation.

## Two-week execution plan (revised)

- **Days 1–2:** Priority 0 unblocking (build/test path + docs + verification contract).
- **Days 3–6:** Shooter MVP (dual flywheel + hood control + readiness gate skeleton).
- **Days 7–9:** Planner + vision integration command path.
- **Days 10–14:** Autonomous hardening + reliability cleanup (ownership, log gating, drivetrain tune pass).

## Definition of done for this phase

- Team can run documented build/test commands successfully in the target dev/CI environment.
- Shooter has closed-loop setpoint interfaces and a single readiness gate used by feed logic.
- Planner is actively used by a command path (not only unit-tested in isolation).
- Autonomous commands have explicit convergence/timeout/fallback checks and telemetry.
- Vision integration approach is decided and reflected in code (integrated or removed).
