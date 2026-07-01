# Tuning & Troubleshooting — Odometry, Vision, and Path Following

This robot estimates **where it is** by fusing wheel/gyro odometry with AprilTag vision, and it
follows autonomous paths off that estimate. This doc explains how to **tune** each layer on a real
robot and how to **diagnose** the things that usually go wrong. It assumes you have already read
[trajectoryfollow.md](trajectoryfollow.md) (the path-following math) and the `robot-description`
and `log-reading` skills.

If you have never tuned this robot before, work top-to-bottom: **drivetrain → odometry → vision →
path following**. Each layer trusts the one above it, so fixing a lower layer before the one above
it is settled just moves the problem around.

---

## 1. The big picture

```
 wheels + gyro ──▶ Phoenix odometry ─┐
                                     ├─▶ RobotState (fuses) ─▶ getPose() ─▶ path followers
 AprilTag cameras ─▶ Limelight ──────┘        │                             (Choreo, DriveToPose)
        (gates + trust shaping)               └─▶ NT:/RobotState/* logs
```

- **Odometry** (wheels + gyro) is smooth and precise short-term but **drifts** over time.
- **Vision** (AprilTags) is drift-free but **noisy** and **delayed** (~50–100 ms).
- [`RobotState`](src/main/java/frc/robot/subsystems/RobotState.java) fuses them with a
  *rewind → fuse → forward* Kalman blend so you get the best of both. This is the same idea Teams
  6328, 254, and 2910 build their code around.
- The path followers read the **fused** pose from `DriveMechanism.getPose()`.

Everything below is controlled by three constant blocks in
[Constants.java](src/main/java/frc/robot/Constants.java): `Estimator`, `Vision`, and `Trajectory`
(plus `Traction`).

---

## 2. Logs are how you tune

There is no AdvantageKit here — `DataLogManager` records **every NetworkTables value** to a
`.wpilog`. Open the log (or go live) in **AdvantageScope** and watch these keys. You cannot tune
what you cannot see, so start every tuning session by pulling up the relevant channels.

### Pose estimator — `NT:/RobotState/*`
| Key | What it tells you |
|-----|-------------------|
| `EstimatedPose` | the fused pose the whole robot uses |
| `OdometryPose` | odometry-only pose (no vision) — compare against `EstimatedPose` |
| `VisionMinusOdometryMeters` | how far vision has pulled the estimate from raw odometry |
| `LastCorrectionMeters` | size of the most recent vision nudge |
| `AcceptedVisionCount` / `RejectedVisionCount` | running totals — is vision actually being used? |
| `SecondsSinceVision` | time since the last accepted frame (−1 = never) |

### Per-camera vision — `NT:/Vision/<camera>/*` (e.g. `limelight-br`)
| Key | What it tells you |
|-----|-------------------|
| `Status` | **the most useful channel** — `accept:megatag1/2` or `reject:<reason>` |
| `Accepted` | 1 if the last frame was fused, else 0 |
| `TagCount`, `AvgTagDistMeters`, `AvgTagArea` | quality of what the camera saw |
| `LatencyMs`, `MeasurementAgeSec` | how delayed the measurement was |
| `WorstAmbiguity` | worst single-tag ambiguity (single-tag only) |
| `PoseJumpMeters` | how far the solve was from current odometry |
| `AppliedXYStdDev`, `AppliedThetaStdDev` | the trust actually handed to the filter |

### Path following — `NT:/Trajectory/AdvancedFollow/*`
| Key | What it tells you |
|-----|-------------------|
| `PlannedPose` vs `MeasuredPose` | where the path wanted us vs where we were |
| `ErrorX`, `ErrorY`, `ErrorNormMeters` | translational tracking error (the headline number) |
| `HeadingErrorRad` | heading tracking error |
| `FeedforwardSpeedMps` vs `CommandedSpeedMps` | plan speed vs what we actually sent |
| `PathProgress01` | 0→1 progress along a Choreo path |
| `LookaheadMeters` | current position-based lookahead distance |

### Drivetrain — `NT:/Drivetrain/*`
`Pose`, `Velocity`, `ModuleStates`/`ModuleTargets` (swerve widget), `OdometryFrequencyHz`
(should sit near 250 on CAN FD), `TranslationSpeedMps`.

---

## 3. Tune the drivetrain first

None of the pose/vision tuning matters if the drivetrain itself is wrong. Before anything else:

1. **Regenerate `TunerConstants` from Tuner X for your real robot.** The checked-in
   [TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java) is an **example
   placeholder** with fake device IDs and gains.
2. Run Tuner X's **drive/steer PID and feedforward** routines and its **wheel-radius / slip**
   calibration. Good `kSpeedAt12Volts` and wheel radius are what make odometry accurate.
3. Confirm the swerve widget in AdvantageScope shows module targets ≈ module states while driving.

Set `Constants.Trajectory.kMaxLinearVelocity` at or below the measured `kSpeedAt12Volts`, and
`Traction.kWheelCoF` to your real tread-on-carpet value (start ~0.9, lower is safer).

---

## 4. Tune odometry

Odometry should be nearly perfect over short distances. To check it:

- Push the robot a **measured 3 m** by hand (disabled) and read `Drivetrain/Pose` — it should read
  ~3 m. If it's off by a few percent, your **wheel radius** is wrong → recalibrate in Tuner X.
- Spin the robot **360°** and confirm heading returns to start. Gyro error → check the Pigeon mount
  and Tuner config.

`Constants.Estimator` controls how much the fuser **trusts odometry between vision corrections**:

| Constant | Default | Raise it to… | Lower it to… |
|----------|---------|--------------|--------------|
| `kOdometryStdDevMeters` | 0.003 | let vision pull translation **harder** | lean more on wheels (smoother) |
| `kOdometryStdDevRadians` | 0.002 | let MegaTag1 pull **heading** harder | trust the gyro's heading more |
| `kPoseBufferSizeSeconds` | 2.0 | tolerate more vision latency | (rarely needed) |

Rule of thumb: **good wheels + good gyro ⇒ small odometry std-devs**, so vision only nudges. If your
drivetrain slips a lot (defense, bad tread), raise `kOdometryStdDevMeters` so vision corrects more.

---

## 5. Tune vision

Vision tuning is **two stages**: hard **gates** (reject bad frames) and soft **trust shaping**
(how much to believe accepted frames). Everything lives in `Constants.Vision`.

### 5a. Watch `Status` first
Drive around the field and watch `NT:/Vision/<camera>/Status`. Every rejected frame names its
reason. The gate reasons and their constants:

| `reject:` reason | Constant | Meaning |
|------------------|----------|---------|
| `invalid` | — | camera returned no valid solve (no tags) |
| `far_tags` | `kMaxTagDistanceMeters` (4.0) | nearest tag too far to trust |
| `high_latency` | `kMaxLatencyMs` (120) | pipeline reported too much latency |
| `stale` | `kMaxMeasurementAgeSec` (0.20) | frame older than our buffer tolerance |
| `low_area` | `kMinAvgTagArea` (0.05) | tags too small in frame (far/steep) |
| `single_tag_ambiguity` | `kMaxSingleTagAmbiguity` (0.20) | lone tag's pose is ambiguous |
| `pose_jump` | `kMaxPoseJumpMeters{Multi,Single}Tag` | solve too far from odometry (outlier) |
| `near_origin` | `kMinPoseNormMeters` (0.10) | solve sitting on (0,0) — bad read |
| `off_field` | `kFieldLength/Width/BoundaryMargin` | solve landed outside the field |
| `fast_yaw` | `kMaxAngularSpeedForVision` (4.0) | spinning too fast to trust a single-tag time-sync |

> **Set the field size!** `kFieldLengthMeters` / `kFieldWidthMeters` are placeholders (2025 field).
> Update them for the real 2026 game or the `off_field` gate will be wrong. See the `game-info` skill.

If vision is **never accepted**, one of these gates is too strict — the `Status` channel tells you
exactly which one. If vision accepts **garbage**, a gate is too loose.

### 5b. Trust shaping (the std-devs)
Accepted frames get a standard deviation (smaller = trusted more). Base model:
`xy = kXYStdDevCoefficient · dist^1.2 / tagCount²`, then inflated by speed, ambiguity, low area, and
correction size:

| Constant | Effect |
|----------|--------|
| `kXYStdDevCoefficient` (0.333) | overall vision trust — **lower = trust vision more** |
| `kVelocityStdDevInflationGain` | distrust vision more while driving fast |
| `kAmbiguityStdDevInflationGain` | distrust ambiguous single-tag solves |
| `kLowAreaStdDevInflationGain` | distrust small/distant tags |
| `kCorrectionStdDevInflationGain` | distrust solves that disagree a lot with odometry |
| `kHeadingStdDevCoefficient` | how much MegaTag1 heading is trusted (MegaTag2 heading is ignored) |

Verify with `AppliedXYStdDev`: it should be **small (a few cm)** when parked in front of two close
tags and **grow** as you speed up or the tags get far/ambiguous.

### 5c. MegaTag1 vs MegaTag2
- **2+ tags → MegaTag1**: vision provides its own heading (trusted via `kHeadingStdDevCoefficient`).
- **1 tag → MegaTag2**: uses **our** heading, so the gyro must be seeded correctly. We feed MegaTag2
  the **odometry-only** heading (`getOdometryPose()`) on purpose, to avoid a vision→heading→vision
  feedback loop. If single-tag poses are consistently rotated, your gyro seed/mounting is off.

---

## 6. Tune path following

Covered in depth in [trajectoryfollow.md](trajectoryfollow.md); the knobs are in
`Constants.Trajectory`. Tune with `Trajectory/AdvancedFollow/ErrorNormMeters` on screen.

| Symptom in the log | Knob |
|--------------------|------|
| Robot lags behind `PlannedPose` on straights | raise `kTranslationP` |
| Robot oscillates / overshoots on straights | lower `kTranslationP`, add a little `kTranslationD` |
| Settles short of / past the final heading | `kHeadingP` (add `kHeadingD` to damp) |
| Cuts corners at speed | lower `kPlaybackLookaheadMax` / `kPlaybackLookaheadSpeedGain` |
| Twitchy / noisy tracking when slow | raise `kPlaybackLookaheadMin` |
| Leg never reports "finished" | check `kPositionTolerance` / `kHeadingTolerance` / `kSettleTimeout` |
| Abandons the path and drives straight to goal | `kStateResetThreshold` too small, or start pose not reseeded |

Keep planned limits (`kMaxLinearVelocity/Accel`) at or under the traction limit
(`Traction.kMaxTranslationAccel = kWheelCoF · kGravity`) so the anti-slip filter rarely intervenes.

---

## 7. Troubleshooting on the real robot

Symptom → probable cause → fix. Use the log channel in **[brackets]** to confirm the cause.

### Pose estimate jumps / teleports
- **Vision outliers being accepted.** `[Vision/*/PoseJumpMeters]` spikes with `Accepted=1`. Tighten
  `kMaxPoseJumpMeters*`; confirm the `off_field`/`near_origin` gates are on.
- **Two cameras disagree.** Compare each camera's accepted poses. A mis-measured camera mount
  (position/angle in the Limelight config) throws its solves off — re-measure the robot-to-camera
  transform in the Limelight web UI.

### Pose drifts and vision never corrects it
- **All frames rejected.** `[Vision/*/Status]` shows a `reject:` reason every loop → loosen that one
  gate (usually `far_tags`, `low_area`, or `stale`).
- **Vision trusted too little.** `[AppliedXYStdDev]` is huge → lower `kXYStdDevCoefficient`.
- **Camera not on NetworkTables.** `Status` never updates → wrong camera name in
  [Robot.java](src/main/java/frc/robot/Robot.java) `Limelight.registerAll(...)`, or the camera's
  hostname doesn't match.

### Single-tag poses are rotated / wrong heading
- **Gyro seed wrong.** MegaTag2 uses our heading. Seed field-centric heading at auto start and check
  the Pigeon mount orientation. `[OdometryPose]` heading should match reality before you trust
  single-tag vision.

### Robot won't stop reef/feeder aligning, or aligns to the wrong spot
- **Alliance flip.** Poses are blue-origin and do **not** flip with alliance. Confirm your target
  pose is in the correct frame (see `game-info`).

### Auto path skipped a waypoint / overshot then came back
- **Start pose not reseeded.** The path starts far from where the robot booted, so the follower saw
  a huge error and replanned straight to the goal. Confirm the first leg reseeds odometry to the
  path's start (`resetOdometryOnStart`), and that `[PathProgress01]` climbs smoothly from 0.

### Auto is accurate in sim but not on the real robot
- **Drivetrain not calibrated.** Sim has perfect wheels. Re-run Tuner X wheel-radius/slip and
  drive/steer FF on the real bot. `[Drivetrain/Pose]` over a measured push is the test.
- **Optimistic Choreo model.** Keep the Choreo `cof` realistic (~0.9–1.1, not the file's default
  1.5); playback no longer clamps the feedforward follower-side, so an unrealistic model shows up
  as overshoot.

### Odometry frequency is low / loop overruns
- `[Drivetrain/OdometryFrequencyHz]` well below 250 → CAN bus utilization too high or not on CAN FD.
  Reduce signal update rates or move the drivetrain to the CANivore/FD bus.

---

## 8. Why Choreo and not a polyline (BLine-style) follower?

We evaluated the polyline point-to-point approach (BLine, used by 2056/2910/1323) and chose **not**
to add it. For **pre-planned autonomous**, our Choreo + position-based playback is
already speed-optimal, dynamically feasible, and sub-centimeter accurate — better than a polyline
P2P follower, whose strengths are *forgiving tuning* and *on-the-fly paths* rather than peak
accuracy. For teleop point-to-point (reef/feeder align) we already have
[`DriveToPose`](src/main/java/frc/robot/commands/DriveToPose.java) and
[`DriveToTag`](src/main/java/frc/robot/commands/DriveToTag.java). Adding a third path-following
paradigm would be redundant complexity for this template. (BLine is also written against WPILib
2025/2026 `edu.wpi.first.*`; it would not compile against this template's 2027 `org.wpilib.*` API.)

---

## 9. Quick tuning checklist

- [ ] `TunerConstants` regenerated for the real robot; drive/steer FF + wheel radius calibrated.
- [ ] `Traction.kWheelCoF` set to real tread-on-carpet value.
- [ ] Odometry reads true over a measured 3 m push and a 360° spin.
- [ ] `Estimator` std-devs set (small if wheels/gyro are good).
- [ ] Field size (`Vision.kFieldLength/WidthMeters`) set for the real game.
- [ ] Each camera accepts frames when it should (`Vision/*/Status` = `accept:*`) and rejects garbage.
- [ ] `AppliedXYStdDev` small in front of close tags, grows with speed/distance.
- [ ] Camera names in `Robot.java` match the real Limelight hostnames.
- [ ] Path `ErrorNormMeters` stays small at full speed; final error within `kPositionTolerance`.
