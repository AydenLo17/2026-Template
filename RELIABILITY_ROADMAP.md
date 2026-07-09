# Reliability Roadmap

This file tracks the reliability work that still belongs in the repo.

<!-- ## Priority 1: Tests and CI

- Create unit tests for RobotState fusion math, trajectory loading, and trajectory sampling.
- Add a GitHub Actions workflow that runs build, test, and one headless simulateJavaAgent smoke test.
- Add at least one regression test that fails if a deploy trajectory disappears or changes shape unexpectedly. -->

## Priority 2: Superstructure state machine

- Keep the current Commands v3 and OpMode structure.
- Promote the state-machine demo into the real superstructure control model.
- Encode piece possession, scoring prep, scoring release, bailout, and recovery as explicit states.
- Make illegal transitions impossible in code instead of relying on driver timing.

## Priority 3: Autonomous framework

- Keep CTRE swerve as the drivetrain backend.
- Add prematch auto validation: selected routine, starting pose, required path files, and mechanism readiness.
- Add fallback branches when intake or score steps fail.
- Add autonomous routines that overlap drive and mechanism actions intentionally instead of only sequencing them.

## Priority 4: Characterization and tuning

- Add repeatable characterization commands for drive, arm, and flywheel.
- Log gains, measured response, and settle times so tuning sessions produce reusable data.
- Add utility OpModes for zeroing, sensor verification, and mechanism hold tests.

## Priority 5: Expanded health checks

- Add controller disconnect alerts.
- Add CAN bus utilization and CANivore status alerts.
- Add mechanism zeroed or not-zeroed status for prematch checks.
- Add match-ready status that combines battery, sensor health, selected auto, and vision connectivity.

## Priority 6: Targeted IO seams

- Keep the current CTRE-generated drivetrain and DriveMechanism wrapper.
- If more simulation or replay control is needed, add IO-style seams only for non-drive mechanisms first.
- If drive abstraction is revisited later, keep CTRE swerve as the implementation behind the wrapper rather than replacing it with a different swerve stack.

## Priority 7: Simulation realism

- Add mechanism simulation for arm and flywheel so autonomous and state-machine logic can be validated without hardware.
- Add simulated sensor failures and stale-camera cases to exercise the alert system.
- Add a scripted sim checklist that verifies the robot can boot, enable, drive, and run a sample autonomous without manual intervention.