// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.ClassicCommand;
import frc.robot.utils.CustomTrajectoryEngine;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Follow a {@link CustomTrajectoryEngine} plan to a goal {@link Pose2d}, binding the engine's
 * feedforward directly to CTRE's hardware-optimized swerve control.
 *
 * <p>This is the high-accuracy counterpart to {@link DriveToPose}. Two things set it apart:
 *
 * <ol>
 *   <li><b>Closed-loop drive on the Talon FX.</b> The request uses {@link
 *       DriveRequestType#Velocity} so each module runs its drive motor on the onboard velocity
 *       control loop. (In this Phoenix 6 line "closed-loop torque-current FOC" is selected at the
 *       <i>module</i> level via the generated {@code TunerConstants} {@code
 *       DriveMotorClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC}; there is no
 *       per-request torque-current {@code DriveRequestType}. Closed-loop velocity here gives the
 *       crisp, deterministic tracking the Kraken/Talon FX hardware is capable of.)
 *   <li><b>{@link SwerveRequest.FieldCentricFacingAngle}.</b> We hand it field-relative translation
 *       velocities and a target heading; its onboard heading controller produces the rotational
 *       rate to face that angle. Translation is our traction-limited feedforward; heading is
 *       closed-loop to the goal facing.
 * </ol>
 *
 * <p><b>Feedforward + feedback.</b> The engine's velocity is only a feedforward - it says how fast
 * to move, not where to be, so on its own tiny tracking errors integrate into a steady-state
 * position offset. Two PID controllers (field X, field Y) close that gap: each loop they trim the
 * error between the PLANNED pose at time {@code t} and the MEASURED pose, and that correction is
 * added to the feedforward <i>before</i> the traction filter, so the friction circle still bounds
 * the combined command. When the profile ends the feedforward goes to zero and the trim
 * "magnetizes" the robot the last few centimeters onto the exact goal.
 *
 * <p>Every loop: read the timebase, recover from any disturbance, sample the engine, add the
 * position trim, push the combined velocity through {@code applyTractionFilter} to stay inside the
 * friction circle, then feed the safe velocity + target heading into the request.
 *
 * <p>"Classic-style" Commands v3 command on {@link ClassicCommand}, matching {@link DriveToPose}.
 */
public class AdvancedTrackTrajectory extends ClassicCommand {
  private final DriveMechanism drivetrain;
  private final Pose2d goal;
  private final CustomTrajectoryEngine engine = new CustomTrajectoryEngine();

  // When non-null, this command FOLLOWS a precomputed path (e.g. parsed from Choreo) instead of
  // generating an on-the-fly profile to a single goal pose. The follow vs. generate choice is made
  // in initialize(); everything downstream (trim, traction filter, request) is identical.
  private final CustomTrajectoryEngine.Sample[] samples;

  // Whether initialize() should seed odometry to the path's first sample. Only meaningful in
  // playback mode. True for a standalone path (so the robot's believed pose lines up with where the
  // path begins); false for the 2nd+ leg of a sequence, where the previous leg already left us at
  // the right spot and re-seeding would erase real accumulated error.
  private final boolean resetOdometryOnStart;

  // Position-trim feedback, one controller per field axis. These correct measured-vs-PLANNED drift;
  // the engine velocity is the feedforward that actually moves the robot. kP is (m/s) per meter of
  // error. Gains live in Constants so all tuning is in one place.
  private final PIDController xController =
      new PIDController(
          Constants.Trajectory.kTranslationP, 0.0, Constants.Trajectory.kTranslationD);
  private final PIDController yController =
      new PIDController(
          Constants.Trajectory.kTranslationP, 0.0, Constants.Trajectory.kTranslationD);

  // Wall-clock time the command started; used to bound the post-profile settle window.
  private double startTime;

  // Field-centric request that also closed-loop controls heading toward a target angle.
  // - DriveRequestType.Velocity: run the module drive motors on their onboard closed loop (with
  //   torque-current FOC selected in TunerConstants) for responsive, deterministic tracking.
  // - BlueAlliance perspectives: our velocities and target heading are in the blue-origin field
  //   frame (the same frame as odometry and the trajectory engine), so don't re-interpret them.
  // - HeadingPID: the onboard controller that turns the target angle into a rotational rate. TODO:
  //   tune (raise kP if heading lags; add kD if it oscillates).
  private final SwerveRequest.FieldCentricFacingAngle driveRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
          .withTargetDirectionPerspective(
              SwerveRequest.TargetDirectionPerspectiveValue.BlueAlliance)
          .withHeadingPID(Constants.Trajectory.kHeadingP, 0.0, Constants.Trajectory.kHeadingD);

  /**
   * Follow a single on-the-fly profile to a goal pose.
   *
   * @param drivetrain the swerve drive to command
   * @param goal the field pose (blue-origin) to drive to, including the goal heading
   */
  public AdvancedTrackTrajectory(DriveMechanism drivetrain, Pose2d goal) {
    super("AdvancedTrackTrajectory", drivetrain); // name + requirement
    this.drivetrain = drivetrain;
    this.goal = goal;
    this.samples = null;
    this.resetOdometryOnStart = false;
  }

  /**
   * Follow a precomputed path (e.g. {@link frc.robot.utils.ChoreoTrajectory#load parsed from
   * Choreo}). The engine plays the samples back as the feedforward; the position trim and traction
   * filter behave exactly as in the goal-pose mode. The termination goal is the path's last sample.
   *
   * <p>Odometry is seeded to the path's first sample on start (see the {@code resetOdometryOnStart}
   * overload to opt out - e.g. for a mid-sequence leg).
   *
   * @param drivetrain the swerve drive to command
   * @param path the ordered trajectory samples (blue-origin), must be non-empty
   */
  public AdvancedTrackTrajectory(DriveMechanism drivetrain, CustomTrajectoryEngine.Sample[] path) {
    this(drivetrain, path, true);
  }

  /**
   * Follow a precomputed path, choosing whether to seed odometry to the path's first sample on
   * start. Use {@code false} for the 2nd+ leg of a multi-path sequence, where the robot is already
   * at the leg's start and re-seeding would discard real tracking error.
   *
   * @param drivetrain the swerve drive to command
   * @param path the ordered trajectory samples (blue-origin), must be non-empty
   * @param resetOdometryOnStart seed odometry to {@code path[0]} in {@code initialize()}
   */
  public AdvancedTrackTrajectory(
      DriveMechanism drivetrain,
      CustomTrajectoryEngine.Sample[] path,
      boolean resetOdometryOnStart) {
    super("AdvancedTrackTrajectory", drivetrain); // name + requirement
    if (path == null || path.length == 0) {
      throw new IllegalArgumentException("AdvancedTrackTrajectory path must be non-empty");
    }
    this.drivetrain = drivetrain;
    this.samples = path;
    this.goal = path[path.length - 1].pose(); // final waypoint - used by the at-goal termination
    this.resetOdometryOnStart = resetOdometryOnStart;
  }

  /**
   * Generates the on-the-fly profile, or loads the precomputed path, from {@code t = 0} now. Either
   * way the engine is primed and the trim controllers are reset.
   */
  @Override
  protected void initialize() {
    startTime = Utils.getCurrentTimeSeconds();
    if (samples != null) {
      // Seed odometry to the path's start so the follower begins aligned with the plan. Without
      // this a path starting away from the robot's current pose reads as a large disturbance and
      // the engine re-plans straight to the goal, skipping the intermediate waypoints.
      if (resetOdometryOnStart) {
        drivetrain.resetPose(samples[0].pose());
      }
      engine.loadSamples(samples, startTime);
    } else {
      engine.generate(drivetrain.getPose(), drivetrain.getFieldVelocity(), goal, startTime);
    }
    xController.reset();
    yController.reset();
  }

  /** Runs every robot loop while the command is active. */
  @Override
  protected void execute() {
    double now = Utils.getCurrentTimeSeconds();

    // Recover from any big hit/shove first: if we've been knocked off the plan, re-anchor the
    // trajectory to where we actually are before sampling.
    engine.handleDisturbance(drivetrain.getPose(), drivetrain.getFieldVelocity(), now);

    // The planned feedforward at this instant (field-relative).
    CustomTrajectoryEngine.TrajectoryState setpoint = engine.sample(now);
    Pose2d measured = drivetrain.getPose();

    // Position trim: PID on the error between where the profile says we should be RIGHT NOW and
    // where odometry says we are. This is what kills the integration drift - it's added to the
    // feedforward so the wheels both follow the profile AND get pulled back onto the planned point.
    double fbX = xController.calculate(measured.getX(), setpoint.targetPose.getX());
    double fbY = yController.calculate(measured.getY(), setpoint.targetPose.getY());

    // Combine feedforward + feedback FIRST, then (for on-the-fly profiles) traction-filter the
    // total, so the friction circle bounds the real command. omega is carried for the filter's
    // signature but ignored downstream - the request derives its own yaw rate to face the target
    // heading.
    ChassisVelocities commanded =
        new ChassisVelocities(
            setpoint.targetVx + fbX, setpoint.targetVy + fbY, setpoint.targetOmega);

    // Anti-slip only for the GENERATE mode. There we invented the acceleration demand, so it needs
    // the friction-circle limiter. A precomputed path (Choreo) is already dynamically feasible, so
    // its feedforward is trusted directly. Running playback through the measured-velocity traction
    // filter is actively harmful: the drivetrain's natural tracking lag makes (v_target - v_now)/dt
    // look like a huge acceleration every loop, so the filter throttles both the feedforward AND
    // the position trim added above - the robot then lags the path, cuts corners, and overshoots
    // the end. (If a real-carpet path is too aggressive, bound acceleration in Choreo's config,
    // which is where trajectory feasibility belongs - not in a follower-side clamp.)
    ChassisVelocities safe =
        samples != null ? commanded : drivetrain.applyTractionFilter(commanded);

    drivetrain.setControl(
        driveRequest
            .withVelocityX(safe.vx)
            .withVelocityY(safe.vy)
            .withTargetDirection(setpoint.targetHeading));
  }

  /**
   * Done when the robot is physically AT the goal - not merely when the profile clock expires.
   *
   * <p>Requires the profile to have finished AND the measured pose to be within the position and
   * heading tolerances. If a late bump leaves us short when the profile ends, the feedforward is
   * already zero and the position trim magnetizes us in before this returns true - so the next leg
   * starts from the right spot. A settle timeout backstops the tolerance so a leg can't hang
   * forever if it's pinned and the tolerance is never met.
   */
  @Override
  protected boolean isFinished() {
    double now = Utils.getCurrentTimeSeconds();
    if (!engine.isFinished(now)) {
      return false;
    }

    Pose2d measured = drivetrain.getPose();
    double positionError = measured.getTranslation().getDistance(goal.getTranslation());
    double headingError = Math.abs(measured.getRotation().minus(goal.getRotation()).getRadians());
    boolean atGoal =
        positionError <= Constants.Trajectory.kPositionTolerance
            && headingError <= Constants.Trajectory.kHeadingTolerance;

    // Backstop: stop trimming after the profile duration + a bounded settle window.
    boolean settleExpired =
        (now - startTime) >= (engine.totalTime() + Constants.Trajectory.kSettleTimeout);

    return atGoal || settleExpired;
  }

  /** Idles the drivetrain. Runs on both natural finish and interruption. */
  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
