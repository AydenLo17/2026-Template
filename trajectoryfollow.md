# Trajectory Following Guide (Swerve + Choreo)

This document explains the new trajectory-following implementation in this robot project:

- How it works (control logic)
- What math it uses
- How to use it with Choreo
- Why this design was chosen
- What is even more accurate if you want to push performance

## TL;DR

The current implementation is strong and practical:

- It follows Choreo paths well in sim.
- It supports generated goal-to-goal profiles and file-based paths with one follower.
- It includes start-pose seeding, trim feedback, and robust finish criteria.

It is not the absolute theoretical maximum accuracy architecture, but it is a very good "best-fit" design for this codebase's goals (teachable, robust, maintainable, high performance).

## Architecture Overview

Main files:

- `src/main/java/frc/robot/utils/CustomTrajectoryEngine.java`
- `src/main/java/frc/robot/commands/AdvancedTrackTrajectory.java`
- `src/main/java/frc/robot/utils/ChoreoTrajectory.java`
- `src/main/java/frc/robot/commands/Autos.java`
- `src/main/java/frc/robot/opmodes/OpModes.java`
- `src/main/java/frc/robot/subsystems/DriveMechanism.java`

### Responsibilities

`CustomTrajectoryEngine`
- Generates on-the-fly trapezoid profiles to a goal pose (X, Y, heading).
- Or plays back precomputed Choreo samples with interpolation.
- Supports disturbance recovery (state reset).

`AdvancedTrackTrajectory`
- Runs each loop and computes the command velocity.
- Combines feedforward from trajectory + position-trim feedback.
- Uses CTRE field-centric facing-angle request for heading behavior.
- Handles completion logic (tolerances + settle timeout).

`ChoreoTrajectory`
- Loads `deploy/choreo/*.traj` JSON.
- Converts `trajectory.samples[]` into plain sample objects for the engine.

`Autos`
- Builds autonomous routines from one or more paths.

`OpModes`
- Exposes autos to the DS as selectable opmodes.

## Control Loop Logic

Every loop during follow:

1. Sample trajectory state at current time.
2. Measure robot pose from odometry.
3. Compute position-trim PID on X and Y using planned pose as setpoint.
4. Add trim to trajectory feedforward velocity.
5. In generated mode, optionally pass through traction limiter.
6. Send velocity + target heading to CTRE swerve request.

This gives:

- smooth path-driven motion from feedforward
- correction for drift/model mismatch from feedback

## Core Math

### 1) Commanded translation velocity

$$
\mathbf{v}_{cmd} = \mathbf{v}_{ff} + \mathbf{v}_{fb}
$$

with

$$
v_{fb,x} = PID_x(x_{meas}, x_{plan}), \qquad
v_{fb,y} = PID_y(y_{meas}, y_{plan})
$$

### 2) Traction limit (generated trajectories)

The traction filter bounds translational acceleration demand by friction circle:

$$
a_{max} = \mu g
$$

If required acceleration exceeds $a_{max}$, scale the translational acceleration vector back to the circle boundary while preserving direction.

### 3) Heading handling

Heading is controlled through CTRE `FieldCentricFacingAngle` with heading PID gains.
Translation and heading are intentionally separated:

- Translation from trajectory + trim
- Heading target from trajectory orientation

## Choreo Path Flow

1. Export `.traj` from Choreo.
2. Place it under `src/main/deploy/choreo/`.
3. Runtime loader parses `trajectory.samples[]`.
4. `AdvancedTrackTrajectory` consumes samples in playback mode.

Fields used per sample:

- `t, x, y, heading`
- `vx, vy, omega`
- `ax, ay, alpha`

Per-module force arrays are ignored by this follower.

## Start Pose Reseed (Critical)

On path start, odometry is reset to the path's first sample pose.

Why this is necessary:

- If robot odometry starts far from path start, follower sees huge initial error.
- Disturbance logic can then regenerate a straight-to-goal path and skip intended waypoints.

In multi-leg sequences, only the first leg reseeds.

## Why This Design Is Good

For this WPILib 2027 + CTRE + teaching template, this solution is a strong practical choice:

- One follower for both generated and planner paths.
- Good accuracy with modest tuning effort.
- Easy to reason about and teach to students.
- Leverages CTRE's proven lower-level request/controller stack.
- Handles real-world mismatch (drift, bumps, imperfect starts).

## Is This The Most Accurate Possible?

Short answer: not the absolute maximum possible, but close enough for strong competitive use when tuned.

If your goal is highest possible tracking precision, you can improve further with a modified architecture:

1. Full holonomic tracking law (time-varying trajectory tracking controller) rather than simple X/Y trim.
2. Module-state feedforward using planned chassis accelerations and identified drivetrain model.
3. Better state estimation (fused vision, latency compensation, robust covariance tuning).
4. Path-parameterized tuning (different gains at high curvature/high speed segments).
5. Choreo constraints tuned from measured robot limits (not optimistic nominal values).

In practice, the biggest accuracy gains usually come from:

- Better estimation (pose quality)
- Better planner constraints matching real robot
- Better gain tuning

not from replacing the entire follower math first.

## Recommended "Best" Path for This Team

Use current implementation as baseline, then iterate in this order:

1. Lock in reliable Choreo workflow and opmode routines.
2. Tune translation/heading gains on carpet.
3. Tune Choreo constraints to real robot capability.
4. Improve pose estimation with robust vision fusion.
5. If still needed, upgrade follower law (holonomic controller style).

## Why it is a strong choice now:

- You already verified good sim accuracy and correct waypoint behavior
- It is robust (start reseed, disturbance handling, bounded finish logic)
- It is teachable and maintainable for this codebase
- It supports both generated and Choreo paths in one clean system

## What can beat it in raw accuracy (if you want to push further):

- Stronger state estimation first (vision fusion/latency tuning)
- Real-world constraint ID in Choreo (match actual accel/CoF limits)
- More advanced holonomic tracking law (time-varying trajectory controller)
- More model-based feedforward using measured drivetrain dynamics

This gets you the largest performance return per engineering hour.

## Usage Examples

Single path:

```java
super(Autos.choreoPath(robot.drivetrain, "TestPath1"));
```

Sequence:

```java
super(Autos.choreoSequence(robot.drivetrain, "Leave", "Pickup", "Score"));
```

Generated pose goals (no file):

```java
super(Autos.drivePoses(
    robot.drivetrain,
    new Pose2d(3.0, 0.0, Rotation2d.kZero),
    new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(90))));
```

## Tuning Checklist

1. Verify odometry reseed to first path point at auto start.
2. Tune `kTranslationP`, then `kTranslationD` if needed.
3. Tune `kHeadingP`, then `kHeadingD`.
4. Validate midpoint passage and endpoint errors in logs.
5. Reduce Choreo aggressiveness if overshoot/slip appears.
6. Re-test with battery sag and realistic match conditions.

## Common Failure Modes

Skips mid-waypoints / drives directly to end:
- Start pose mismatch or missing reseed.

Cuts corners / lags then overshoots:
- Planner too aggressive for real limits or poor gain tuning.

Never finishes:
- Tolerances too tight, heading gains too low, or settle timeout too short.

## Final Recommendation

This implementation is a very good solution for this project right now.

If your target is absolute max autonomous precision, treat this as v1.5 and evolve it (estimation + constraints + controller upgrade), rather than replacing everything immediately.
