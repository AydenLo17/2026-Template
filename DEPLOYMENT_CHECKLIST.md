# Deployment Checklist — Real Robot Setup

This checklist documents every step needed to move from the template to a competition-ready robot.
Work **top-to-bottom** after building the hardware, as each layer depends on the one above it.

## ✅ Phase 1: Hardware & CAN Bus

- [ ] **Hardware assembled** — drivetrain, arm, flywheel, vision cameras all mounted
- [ ] **CAN bus wired** — all devices on same CANivore or RIO bus; confirm CAN health in firmware
- [ ] **Motor directions verified** — apply manual 12V to each motor; confirm forward matches intention
- [ ] **Sensors mounted**:
  - [ ] Drivetrain encoder (CANcoder) zeroed and aligned with wheel
  - [ ] Arm absolute encoder (CANcoder) aligned with arm mechanical zero
  - [ ] Gyroscope mounted flat and accessible for zero calibration
  - [ ] Cameras (Limelight) mounted on robot; note pitch, yaw, height, forward/left offsets
- [ ] **Brake/coast modes set** — verify each subsystem uses the correct mode (drivetrain = coast, arm = coast, flywheel = coast)

---

## ✅ Phase 2: Generate & Verify Hardware Constants

Tuner X generates device IDs, gear ratios, and PID gains specific to your hardware.

- [ ] **Run Tuner X Swerve Project Generator** → generates `TunerConstants.java`
  - Regenerate [generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java) and [CommandSwerveDrivetrain.java](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java)
  - Verify all **device IDs** match your hardware (drivetrain motors/encoders, arm motor/encoder, flywheel motor)
  - Verify **CAN bus name** in generated code matches your physical bus (`kCANBus` constant)
- [ ] **Verify TunerConstants device IDs in all subsystems**:
  - [Arm.java](src/main/java/frc/robot/subsystems/arm/Arm.java) — motor CAN 31, encoder CAN 32
  - [Flywheel.java](src/main/java/frc/robot/subsystems/flywheel/Flywheel.java) — motor CAN 21
  - [DriveMechanism.java](src/main/java/frc/robot/subsystems/DriveMechanism.java) — uses `TunerConstants.createDrivetrain()`
- [ ] **Confirm CAN devices boot without errors** — deploy and check Robot Console for CAN errors

---

## ✅ Phase 3: Motion Characterization & Tuning

Measure motor response and set PID/FF gains using the built-in characterization OpModes. After each OpMode runs, plot the data in AdvantageScope to extract the gains.

### Arm Characterization

Select each utility OpMode from the driver station, run it with the robot disabled, and plot the resulting telemetry:

- [ ] **"Arm Gravity FF"** — arm sags under gravity with 0V applied
  - Plot: `Arm/Position` (Y-axis, degrees) vs `Arm/AppliedVoltage` (X-axis, volts)
  - At vertical (0°): voltage ≈ 0. At horizontal (90°): voltage ≈ max
  - At 90°, read the voltage needed to hold position; that's `kG = voltage / 12.0`
  - Safe starting point: `kG ≈ 0.2` (typical for arm-mass robots)
- [ ] **"Arm Static Friction"** — voltage ramps slowly until arm moves
  - Plot: `Arm/Position` over time and `Arm/AppliedVoltage` over time
  - Find the voltage where position starts changing (the breakpoint)
  - Set `kS ≈ 90% of breakpoint voltage` (if it moves at 2.5V, use kS = 2.25V)
- [ ] **"Arm PID Tuning"** — arm oscillates between vertical and horizontal
  - Plot: `Arm/Position` and `Arm/AppliedVoltage` over time
  - Does the arm overshoot (go past target)? Lag behind (move too slowly)?
  - Start with `kP ≈ 80`, `kD ≈ 8.0`; adjust up if sluggish, down if jerky
  - Re-run the characterization after each gain change to see the effect
- [ ] **Update [Arm.java](src/main/java/frc/robot/subsystems/arm/Arm.java#L40)** with measured values:
  ```java
  private static final double kG = 0.2;   // Measured gravity FF
  private static final double kS = 0.2;   // Measured static friction
  private static final double kP = 80.0;  // Tuned proportional gain
  private static final double kD = 8.0;   // Tuned derivative gain
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 2.0;   // rot/s
  private static final double MOTION_MAGIC_ACCELERATION = 4.0;     // rot/s²
  ```

### Flywheel Characterization

- [ ] **"Flywheel Voltage Ramp"** — voltage ramps 0→12V over 15 seconds
  - Plot: `Flywheel/AppliedVoltage` (X-axis) vs `Flywheel/Velocity` (Y-axis)
  - Should see roughly linear relationship (higher voltage = higher speed)
  - Fit a line through the data; the slope is `kV` (volts per rps)
  - Typical range: 0.1–0.15. Calculate: `kV = max_voltage / max_speed`
- [ ] **"Flywheel Step Response"** — apply 8V and measure acceleration
  - Plot: `Flywheel/Velocity` and `Flywheel/AppliedVoltage` over time
  - Measure how fast the flywheel accelerates to steady state
  - Measure steady-state speed at 8V; use to verify kV estimate
- [ ] **"Flywheel PID Tuning"** — ramp to 20 RPS and hold for 10s
  - Plot: `Flywheel/Velocity` over time
  - Does it undershoot (stay below 20 RPS)? Overshoot (spike above)?
  - If undershooting: increase `kP` slightly (start 0.01, try 0.02, 0.05, etc.)
  - Re-run the characterization after each change to see the effect
- [ ] **Update [Flywheel.java](src/main/java/frc/robot/subsystems/flywheel/Flywheel.java#L27)** with measured values:
  ```java
  private static final double kS = 0.0;    // Static friction (usually small for flywheel)
  private static final double kV = 0.125;  // Measured velocity FF (volts per rps)
  private static final double kP = 0.01;   // Tuned proportional gain
  ```

### How to Use Characterization OpModes

1. Connect robot to driver station
2. Select a characterization OpMode (e.g., "Arm Gravity FF") from the driver station
3. Put robot in **disabled mode** (not autonomous or teleop)
4. Let it run for the full duration (10–15 seconds)
5. Download the `.wpilog` file from the robot (or read live via AdvantageScope over USB)
6. In AdvantageScope, open the log and plot the relevant channels
7. Measure the data and update constants in the subsystem file
8. Redeploy and re-run the same characterization to verify improvement

---

### Drivetrain (CTRE Tuner X)

- [ ] **Run wheel radius calibration** — drive a measured distance, record drift, tune `effectiveWheelRadiusMeters`
- [ ] **Run drive PID + FF routine** — captures `kS`, `kV`, `kA` for wheel motor control
- [ ] **Run steer PID routine** — captures steer module gains
- [ ] **Update [Constants.Trajectory](src/main/java/frc/robot/Constants.java#L65)** — set `kMaxLinearVelocity` ≤ measured `TunerConstants.kSpeedAt12Volts`

---

## ✅ Phase 4: Vision & Camera Calibration

Accurate vision depends entirely on knowing where cameras are mounted relative to the robot center.

- [ ] **Measure camera mount positions** (mount one Limelight at a time):
  - **Pitch** — angle down from horizontal (degrees). Positive = pointing downward.
  - **Yaw** — rotation left/right (degrees). Positive = pointing to robot-left.
  - **Height** — vertical distance from carpet (meters).
  - **Forward offset** — distance forward from robot center (meters).
  - **Left offset** — distance to robot-left from center (meters).
- [ ] **Update [Constants.Gamepiece](src/main/java/frc/robot/Constants.java#L347)** camera calibration:
  ```java
  public static final double kCameraPitchDegrees = 24.0;     // Measured value
  public static final double kCameraYawDegrees = 0.0;        // Measured value
  public static final double kCameraHeightMeters = 0.56;     // Measured value
  public static final double kCameraForwardMeters = 0.30;    // Measured value
  public static final double kCameraLeftMeters = 0.00;       // Measured value
  ```
- [ ] **Verify Limelight NetworkTables names** match code:
  - Default: `"limelight"`, `"limelight-br"`, `"limelight-bl"` (see [Constants.Gamepiece](src/main/java/frc/robot/Constants.java#L349))
  - Update [OpModes.java](src/main/java/frc/robot/opmodes/OpModes.java#L79) if names differ
- [ ] **Position Limelight crop window** (AprilTag detection):
  - Test at various distances (near tag, far tag)
  - Verify crop window reduces latency without losing target reacquisition
  - No code change needed (crop is tuned in [Constants.Vision](src/main/java/frc/robot/Constants.java#L248))

---

## ✅ Phase 5: Field Constants & Scoring Targets

Game-specific configuration — update once 2027 game rules are final.

- [ ] **Update field dimensions** in [Constants.Vision](src/main/java/frc/robot/Constants.java#L231):
  ```java
  public static final double kFieldLengthMeters = 17.55;  // Replace with 2027 field length
  public static final double kFieldWidthMeters = 8.05;    // Replace with 2027 field width
  ```
  *(Currently set to 2025 Reefscape placeholder values.)*
- [ ] **Identify scoring AprilTag IDs** for your alliance:
  - Confirm layout from official 2027 game specs
  - Blue alliance tag IDs vs red alliance tag IDs
- [ ] **Update scoring target** in [OpModes.DriverTeleop](src/main/java/frc/robot/opmodes/OpModes.java#L79):
  ```java
  private static final int ALIGN_TAG_ID = 1;  // Replace with actual scoring tag for your position
  ```
- [ ] **Create alliance-aware auto routines** in [Autos.java](src/main/java/frc/robot/commands/Autos.java):
  - Replace demo routines with real game scoring sequences
  - Ensure coordinates are in **blue-origin frame** (per [game-info skill](https://frc5712.com))

---

## ✅ Phase 6: Path Following Tuning

Trajectory constants control autonomous motion — tune these after odometry is accurate.

- [ ] **Tune position-trim feedback** in [Constants.Trajectory](src/main/java/frc/robot/Constants.java#L65):
  - `kTranslationP` — proportional gain on X/Y position error (m/s per m). Start at 4.0, adjust up if lag, down if overshoot.
  - `kTranslationD` — derivative (damping). Start at 0.0, increase if oscillation.
  - `kHeadingP` / `kHeadingD` — onboard gyro-based heading controller. Start at 8.0 / 0.3.
- [ ] **Verify at-goal tolerances** make sense for your game:
  ```java
  public static final double kPositionTolerance = 0.02;    // 2 cm — may need tighter for scoring
  public static final double kHeadingTolerance = Math.toRadians(1.0); // 1 degree
  ```
- [ ] **Tune Choreo playback** if using path files:
  - `kPlaybackLookaheadMin` / `kPlaybackLookaheadMax` — how far ahead to lock onto the path
  - `kPlaybackLookaheadSpeedGain` — how lookahead scales with speed

---

## ✅ Phase 7: Odometry & Vision Fusion

Fine-tune the pose estimator once drivetrain and vision are working.

- [ ] **Tune odometry trust** in [Constants.Estimator](src/main/java/frc/robot/Constants.java#L267):
  - `kOdometryStdDevMeters` — how much drift to expect (smaller = trust odometry more)
  - `kOdometryStdDevRadians` — gyro drift trust
  - These values multiply with vision-reported uncertainty; no tuning needed if odometry/vision each work alone
- [ ] **Verify vision gates** in [Constants.Vision](src/main/java/frc/robot/Constants.java#L183):
  - `kMaxTagDistanceMeters` — reject vision if tags are farther than this
  - `kMaxLatencyMs` — reject old frames
  - `kMaxSingleTagAmbiguity` — reject bad single-tag solves
  - These are conservative by default; loosen if losing valid measurements
- [ ] **Check estimator logs** in AdvantageScope (`NT:/RobotState/*`):
  - `AcceptedVisionCount` should increase during auto
  - `RejectedVisionCount` should be low (only bad frames)
  - `VisionMinusOdometryMeters` shows how much vision pulls the estimate

---

## ✅ Phase 8: Test on Real Field

Before competition, validate behavior in competition-like conditions.

- [ ] **Test drivetrain**:
  - [ ] Drive in teleop — smooth, responsive, no jitter
  - [ ] Run autonomous legs — odometry pose ≈ measured position
  - [ ] Confirm swerve widget in dashboard matches wheel angles
- [ ] **Test vision**:
  - [ ] Stand near AprilTags — Limelight detects targets
  - [ ] Check `NT:/Vision/<camera>/Status` — showing `accept:megatag` or similar
  - [ ] Move robot 1 meter and verify estimated pose drifts < 2 cm from truth
- [ ] **Test autonomous**:
  - [ ] Run each auto routine 3× — should repeat within 10 cm
  - [ ] Verify superstructure (arm/flywheel) moves at correct times
  - [ ] Log and inspect in AdvantageScope
- [ ] **Test game-specific moves**:
  - [ ] Intake a game piece — confirm gamepiece detection works
  - [ ] Score — confirm arm/flywheel move correctly
  - [ ] Test all button bindings in teleop

---

## ✅ Phase 9: Pre-Match Checklist

Before every match, verify robot is in a known good state.

- [ ] **Run pre-match script**:
  ```
  # Verify robot boots and all devices enumerate:
  ./gradlew deploy  # Deploy to robot
  ```
  *(Add a Utility OpMode that displays battery, CAN health, mechanism positions, vision status.)*
- [ ] **Verify sensors** in dashboard:
  - [ ] Arm position reads correct angle when moved by hand
  - [ ] Flywheel velocity shows realistic numbers when spun by hand
  - [ ] Drivetrain heading matches gyro
  - [ ] Limelight shows targets when pointed at AprilTag
- [ ] **Verify autonomous routine selected** on driver station
- [ ] **Check battery voltage** — should be ≥ 12.0 V
- [ ] **Clear logs** — delete old `.wpilog` files to free USB space

---

## ✅ Phase 10: Production Readiness

Final checks before shipping to competition.

- [ ] **Code compiles without warnings** — `./gradlew build`
- [ ] **All tests pass** — `./gradlew test`
- [ ] **Headless simulation smoke test passes** — `./gradlew simulateJavaAgent -Pmode=auto`
- [ ] **Code is formatted** — `./gradlew spotlessApply`
- [ ] **No merge conflicts** — git status is clean
- [ ] **Documentation is current** — update `TUNING.md` with actual values for team reference
- [ ] **Backup created** — commit code to git, push to team repository

---

## Reference

- [ONBOARDING.md](ONBOARDING.md) — OpMode/Commands-v3 structure and wiring
- [TUNING.md](TUNING.md) — detailed tuning procedures for odometry, vision, path following
- [robot-description skill](https://frc5712.com) — where every piece lives in the code
- [game-info skill](https://frc5712.com) — field frame conventions and AprilTag layout
- [log-reading skill](https://frc5712.com) — how to read `.wpilog` in AdvantageScope
- [run-sim skill](https://frc5712.com) — simulation and headless agent mode
- [RELIABILITY_ROADMAP.md](RELIABILITY_ROADMAP.md) — future enhancements (state machines, characterization utilities, etc.)

---

**Last updated:** July 2026 | **Tested on:** WPILib 2027 alpha-6
