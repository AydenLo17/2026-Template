// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * Physics-informed, on-the-fly trajectory engine for the swerve drive.
 *
 * <p>It generates a smooth motion plan from wherever the robot <i>is</i> to a target {@link
 * Pose2d}, using three <b>decoupled</b> 1-D trapezoidal profiles - one each for field X, field Y,
 * and heading. Decoupling keeps the math deterministic and trivial to reason about: each axis is
 * just a velocity-and-acceleration-limited move, and the three are sampled at a common time {@code
 * t}.
 *
 * <p>Two things make this more than a static profile:
 *
 * <ol>
 *   <li><b>Boundary-condition generation.</b> A profile is built from the robot's actual pose
 *       <i>and</i> actual field-relative velocity ({@code drivetrain.getState().Velocity}). Because
 *       the start velocity is a real boundary condition, the plan is continuous with the robot's
 *       current motion - no jump at t=0.
 *   <li><b>State Reset (disturbance recovery).</b> Each loop the command asks the engine whether
 *       the robot has been knocked far off the planned pose (a hard hit, defense, a bump). If the
 *       position error exceeds {@link Constants.Trajectory#kStateResetThreshold}, the engine treats
 *       the current drifted pose + measured chassis speeds as a fresh {@code t = 0} and regenerates
 *       a new smooth curve to the same target. The robot recovers gracefully instead of fighting a
 *       stale plan.
 * </ol>
 *
 * <p>The engine can also <b>play back a precomputed path</b> (e.g. JSON vectors compiled by
 * TrajoptLib): hand it an array of {@link Sample}s via {@link #loadSamples(Sample[], double)} and
 * it interpolates them for the feedforward. Disturbance recovery still works - a large deviation
 * drops back to live profile generation from the current state to the goal.
 *
 * <p>All geometry is in the blue-alliance-origin field frame (the Phoenix odometry convention), the
 * same frame {@code DriveToPose} uses, so the output velocities can be fed straight into a
 * field-centric swerve request.
 */
public class CustomTrajectoryEngine {

  /**
   * One sampled point of the planned trajectory: the feedforward the controller should apply at
   * this instant. {@code targetVx/targetVy} are field-relative translational velocities (m/s),
   * {@code targetHeading} is the desired robot facing, and {@code ax/ay/alpha} are the planned
   * accelerations (m/s^2, rad/s^2) - useful both as a feedforward term and as the quantity the
   * traction limiter checks against the friction circle.
   */
  public static final class TrajectoryState {
    /** Planned field pose (x, y, heading) at this instant. */
    public final Pose2d targetPose;

    /** Field-relative translational velocity feedforward (m/s). */
    public final double targetVx;

    public final double targetVy;

    /** Desired robot facing at this instant. */
    public final Rotation2d targetHeading;

    /** Angular velocity feedforward (rad/s). */
    public final double targetOmega;

    /** Planned translational acceleration (m/s^2). */
    public final double ax;

    public final double ay;

    /** Planned angular acceleration (rad/s^2). */
    public final double alpha;

    public TrajectoryState(
        Pose2d targetPose,
        double targetVx,
        double targetVy,
        Rotation2d targetHeading,
        double targetOmega,
        double ax,
        double ay,
        double alpha) {
      this.targetPose = targetPose;
      this.targetVx = targetVx;
      this.targetVy = targetVy;
      this.targetHeading = targetHeading;
      this.targetOmega = targetOmega;
      this.ax = ax;
      this.ay = ay;
      this.alpha = alpha;
    }

    /**
     * The translational feedforward as a (field-relative, zero-omega is up to the caller) tuple.
     */
    public ChassisVelocities fieldVelocities() {
      return new ChassisVelocities(targetVx, targetVy, targetOmega);
    }
  }

  /**
   * One precomputed waypoint, e.g. a row of a TrajoptLib-compiled path. A JSON loader (Jackson,
   * etc.) deserializes the path file into an array of these; the engine then interpolates between
   * them.
   *
   * @param timeSeconds time from the start of the path
   * @param pose field pose at this point
   * @param vx field-relative X velocity (m/s)
   * @param vy field-relative Y velocity (m/s)
   * @param omega angular velocity (rad/s)
   * @param ax field-relative X acceleration (m/s^2)
   * @param ay field-relative Y acceleration (m/s^2)
   * @param alpha angular acceleration (rad/s^2)
   */
  public record Sample(
      double timeSeconds,
      Pose2d pose,
      double vx,
      double vy,
      double omega,
      double ax,
      double ay,
      double alpha) {}

  // Small finite-difference step (seconds) used to read acceleration off the velocity profile.
  private static final double kAccelEpsilon = 1.0e-3;

  // --- Profile (live-generation) backend --------------------------------------------------------
  private final TrapezoidProfile xProfile;
  private final TrapezoidProfile yProfile;
  private final TrapezoidProfile headingProfile;

  private TrapezoidProfile.State xStart = new TrapezoidProfile.State();
  private TrapezoidProfile.State yStart = new TrapezoidProfile.State();
  private TrapezoidProfile.State headingStart = new TrapezoidProfile.State();
  private TrapezoidProfile.State xGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State yGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State headingGoal = new TrapezoidProfile.State();

  private Pose2d goal = Pose2d.kZero;
  private double originTime;
  private double totalTime;

  // --- Playback backend (optional) --------------------------------------------------------------
  private Sample[] samples = null;
  private boolean playback = false;

  /** Builds an engine with the default per-axis constraints from {@link Constants.Trajectory}. */
  public CustomTrajectoryEngine() {
    this(
        new TrapezoidProfile.Constraints(
            Constants.Trajectory.kMaxLinearVelocity, Constants.Trajectory.kMaxLinearAccel),
        new TrapezoidProfile.Constraints(
            Constants.Trajectory.kMaxAngularVelocity, Constants.Trajectory.kMaxAngularAccel));
  }

  /**
   * @param linear translational constraints (max m/s, max m/s^2) applied to BOTH the X and Y axes
   * @param angular heading constraints (max rad/s, max rad/s^2)
   */
  public CustomTrajectoryEngine(
      TrapezoidProfile.Constraints linear, TrapezoidProfile.Constraints angular) {
    xProfile = new TrapezoidProfile(linear);
    yProfile = new TrapezoidProfile(linear);
    headingProfile = new TrapezoidProfile(angular);
  }

  /**
   * Generates a fresh trajectory from the given boundary conditions to {@code goal}. Call this once
   * when a follow command starts (and again automatically on a State Reset).
   *
   * @param start the pose the trajectory begins from (usually the robot's current pose)
   * @param startFieldVelocity the robot's current field-relative velocity - the velocity boundary
   *     condition that makes the plan continuous with real motion
   * @param goal the target field pose, including the goal heading
   * @param timestampSeconds the current timebase reading; becomes this trajectory's {@code t = 0}
   */
  public void generate(
      Pose2d start, ChassisVelocities startFieldVelocity, Pose2d goal, double timestampSeconds) {
    this.goal = goal;
    this.originTime = timestampSeconds;
    this.playback = false; // live generation overrides any loaded path

    // X and Y are independent 1-D moves: position boundary = pose component, velocity boundary =
    // the matching field-relative velocity component. Goal velocity is 0 (we stop on the target).
    xStart = new TrapezoidProfile.State(start.getX(), startFieldVelocity.vx);
    yStart = new TrapezoidProfile.State(start.getY(), startFieldVelocity.vy);
    xGoal = new TrapezoidProfile.State(goal.getX(), 0.0);
    yGoal = new TrapezoidProfile.State(goal.getY(), 0.0);

    // Heading is profiled on an UNWRAPPED radian axis so the trapezoid takes the shortest turn and
    // never sees a +/-pi discontinuity. We express the goal as start + (shortest signed delta).
    double startRad = start.getRotation().getRadians();
    double delta =
        goal.getRotation().minus(start.getRotation()).getRadians(); // already in [-pi, pi]
    headingStart = new TrapezoidProfile.State(startRad, startFieldVelocity.omega);
    headingGoal = new TrapezoidProfile.State(startRad + delta, 0.0);

    // Cache the overall duration (the slowest axis) so isFinished() is a cheap comparison.
    totalTime = Math.max(durationOf(xProfile, xStart, xGoal), durationOf(yProfile, yStart, yGoal));
    totalTime = Math.max(totalTime, durationOf(headingProfile, headingStart, headingGoal));
  }

  /**
   * Loads a precomputed path (e.g. TrajoptLib JSON, deserialized into {@link Sample}s) and switches
   * the engine to playback mode. The samples must be sorted by {@link Sample#timeSeconds()} and
   * start at 0.
   *
   * @param path the ordered waypoints
   * @param timestampSeconds the current timebase reading; becomes the path's {@code t = 0}
   */
  public void loadSamples(Sample[] path, double timestampSeconds) {
    if (path == null || path.length == 0) {
      throw new IllegalArgumentException("Trajectory sample array must be non-empty");
    }
    this.samples = path;
    this.originTime = timestampSeconds;
    this.goal = path[path.length - 1].pose();
    this.totalTime = path[path.length - 1].timeSeconds();
    this.playback = true;
  }

  /**
   * Checks for a major disturbance and, if found, performs a State Reset. The robot's drifted pose
   * and measured field velocity become a new {@code t = 0} and a fresh smooth curve is generated to
   * the same goal. Call this once per loop, before {@link #sample(double)}.
   *
   * @param currentPose the robot's measured pose right now
   * @param currentFieldVelocity the robot's measured field-relative velocity right now
   * @param timestampSeconds the current timebase reading
   * @return true if a reset (regeneration) happened this call
   */
  public boolean handleDisturbance(
      Pose2d currentPose, ChassisVelocities currentFieldVelocity, double timestampSeconds) {
    Pose2d planned = sample(timestampSeconds).targetPose;
    double error = currentPose.getTranslation().getDistance(planned.getTranslation());
    if (error > Constants.Trajectory.kStateResetThreshold) {
      // Re-anchor: regenerate live from the current (drifted) state to the goal. This also drops us
      // out of playback mode, since the precomputed path no longer matches reality.
      generate(currentPose, currentFieldVelocity, goal, timestampSeconds);
      return true;
    }
    return false;
  }

  /**
   * Samples the planned feedforward at the given timebase reading.
   *
   * @param timestampSeconds the current timebase reading (the engine subtracts its own origin)
   * @return the planned pose, velocities, and accelerations at this instant
   */
  public TrajectoryState sample(double timestampSeconds) {
    double t = Math.max(0.0, timestampSeconds - originTime);
    return playback ? samplePlayback(t) : sampleProfile(t);
  }

  /** Whether the planned motion has finished (the slowest axis has reached its goal). */
  public boolean isFinished(double timestampSeconds) {
    return (timestampSeconds - originTime) >= totalTime;
  }

  /** The planned duration (seconds) of the current trajectory - the slowest axis's profile time. */
  public double totalTime() {
    return totalTime;
  }

  /** The goal pose this engine is currently driving toward. */
  public Pose2d getGoal() {
    return goal;
  }

  // --- internals --------------------------------------------------------------------------------

  private TrajectoryState sampleProfile(double t) {
    TrapezoidProfile.State x = xProfile.calculate(t, xStart, xGoal);
    TrapezoidProfile.State y = yProfile.calculate(t, yStart, yGoal);
    TrapezoidProfile.State h = headingProfile.calculate(t, headingStart, headingGoal);

    // Acceleration = d(velocity)/dt. The trapezoid profile only reports position and velocity, so
    // we
    // read it numerically: how much does the profiled velocity change over a tiny step ahead?
    TrapezoidProfile.State xNext = xProfile.calculate(t + kAccelEpsilon, xStart, xGoal);
    TrapezoidProfile.State yNext = yProfile.calculate(t + kAccelEpsilon, yStart, yGoal);
    TrapezoidProfile.State hNext =
        headingProfile.calculate(t + kAccelEpsilon, headingStart, headingGoal);
    double ax = (xNext.velocity - x.velocity) / kAccelEpsilon;
    double ay = (yNext.velocity - y.velocity) / kAccelEpsilon;
    double alpha = (hNext.velocity - h.velocity) / kAccelEpsilon;

    Rotation2d heading = Rotation2d.fromRadians(h.position);
    Pose2d pose = new Pose2d(x.position, y.position, heading);
    return new TrajectoryState(pose, x.velocity, y.velocity, heading, h.velocity, ax, ay, alpha);
  }

  private TrajectoryState samplePlayback(double t) {
    // Before the first / after the last sample, clamp to the endpoints.
    if (t <= samples[0].timeSeconds()) {
      return stateOf(samples[0]);
    }
    Sample last = samples[samples.length - 1];
    if (t >= last.timeSeconds()) {
      return stateOf(last);
    }
    // Find the bracketing pair [a, b] and linearly interpolate the feedforward between them.
    for (int i = 1; i < samples.length; i++) {
      Sample b = samples[i];
      if (t <= b.timeSeconds()) {
        Sample a = samples[i - 1];
        double span = b.timeSeconds() - a.timeSeconds();
        double f = span <= 0.0 ? 0.0 : (t - a.timeSeconds()) / span;
        Pose2d pose = a.pose().interpolate(b.pose(), f);
        double vx = lerp(a.vx(), b.vx(), f);
        double vy = lerp(a.vy(), b.vy(), f);
        double omega = lerp(a.omega(), b.omega(), f);
        double ax = lerp(a.ax(), b.ax(), f);
        double ay = lerp(a.ay(), b.ay(), f);
        double alpha = lerp(a.alpha(), b.alpha(), f);
        return new TrajectoryState(pose, vx, vy, pose.getRotation(), omega, ax, ay, alpha);
      }
    }
    return stateOf(last); // unreachable, but keeps the compiler happy
  }

  private static TrajectoryState stateOf(Sample s) {
    return new TrajectoryState(
        s.pose(), s.vx(), s.vy(), s.pose().getRotation(), s.omega(), s.ax(), s.ay(), s.alpha());
  }

  private static double lerp(double a, double b, double f) {
    return a + (b - a) * f;
  }

  // Computes a profile's total duration for a given start/goal without disturbing later sampling
  // (TrapezoidProfile is stateless across calculate() calls - totalTime() reflects the last pair).
  private static double durationOf(
      TrapezoidProfile profile, TrapezoidProfile.State start, TrapezoidProfile.State goal) {
    profile.calculate(0.0, start, goal);
    return profile.totalTime();
  }
}
