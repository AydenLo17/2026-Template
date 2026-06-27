// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.ClassicCommand;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * Drive in a straight line to a field pose using CTRE's {@link LinearPath}, on odometry.
 *
 * <p>This is the odometry counterpart to {@link DriveToTag} (which is vision-only). It's the
 * building block for an autonomous routine: give it a goal {@link Pose2d} and it profiles a
 * straight-line approach, simultaneously rotating to the goal heading.
 *
 * <p>{@code LinearPath} is a <i>trajectory generator</i>, not a controller. We capture the start
 * pose + velocity once, then each loop sample the path at the absolute elapsed time {@code t}:
 * {@code path.calculate(t, startState, goal)} returns the profiled field-relative pose + velocity
 * at that instant. Because the start state is fixed, {@link LinearPath#totalTime() totalTime()}
 * stays constant and {@link LinearPath#isFinished(double) isFinished(t)} is the entire
 * done-condition - no pose-tolerance bookkeeping.
 *
 * <p>The command output is feedforward + feedback: the profile <b>velocity</b> is the feedforward
 * (this is what actually drives the robot along the line), and three PID controllers (X, Y,
 * heading) add a correction that trims the measured pose back onto the profiled pose to cancel
 * drift.
 *
 * <p>"Classic-style" Commands v3 command on {@link ClassicCommand}, matching {@link DriveToTag}:
 * the v2 lifecycle hooks do the work and the framework wires the coroutine.
 */
public class DriveToPose extends ClassicCommand {
  // --- Translation limits. TODO: tune to the drivetrain's real capability. ---
  private static final double MAX_LINEAR_VELOCITY = 2.5; // m/s
  private static final double MAX_LINEAR_ACCEL = 3.0; // m/s^2

  // --- Rotation limits. ---
  private static final double MAX_ANGULAR_VELOCITY = Math.PI; // rad/s
  private static final double MAX_ANGULAR_ACCEL = 2.0 * Math.PI; // rad/s^2

  // --- Feedback gains. The profile feedforward does the bulk of the driving; PID only pulls the
  // measured pose back onto the profiled pose. TODO: tune. Raise kP if the bot lags the profile or
  // settles short of the goal; lower it (or add kD) if it oscillates. ---
  private static final double TRANSLATION_KP = 3.0; // (m/s) per meter of error
  private static final double HEADING_KP = 4.0; // (rad/s) per radian of error

  private final DriveMechanism drivetrain;
  private final Pose2d goal;

  // The straight-line profile generator: linear constraints for translation, angular for heading.
  private final LinearPath path =
      new LinearPath(
          new TrapezoidProfile.Constraints(MAX_LINEAR_VELOCITY, MAX_LINEAR_ACCEL),
          new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCEL));

  // Pose-error feedback, one controller per field axis. These correct measured-vs-profiled drift;
  // the profile velocity is the feedforward that actually moves the robot.
  private final PIDController xController = new PIDController(TRANSLATION_KP, 0.0, 0.0);
  private final PIDController yController = new PIDController(TRANSLATION_KP, 0.0, 0.0);
  private final PIDController headingController = new PIDController(HEADING_KP, 0.0, 0.0);

  // Field-relative velocity request. Blue-origin perspective so the commanded velocity is in the
  // same frame as the odometry pose (which is always blue-origin); open-loop drive so no
  // drive-velocity PID tuning is required.
  private final SwerveRequest.ApplyFieldVelocity driveRequest =
      new SwerveRequest.ApplyFieldVelocity()
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Captured once at start: the pose + field velocity the trajectory is generated from. The path is
  // sampled at absolute elapsed time against this fixed state, so totalTime() stays constant.
  private LinearPath.State startState = new LinearPath.State();
  // Wall-clock time the command started; elapsed = now - startTime is our trajectory time t.
  private double startTime;

  /**
   * @param drivetrain the swerve drive to command
   * @param goal the field pose (blue-origin) to drive to, including the goal heading
   */
  public DriveToPose(DriveMechanism drivetrain, Pose2d goal) {
    super("DriveToPose", drivetrain); // name + requirement, like v2 addRequirements(drivetrain)
    this.drivetrain = drivetrain;
    this.goal = goal;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Captures the start state the trajectory is generated from and starts the clock. */
  @Override
  protected void initialize() {
    startState = new LinearPath.State(drivetrain.getPose(), drivetrain.getFieldVelocity());
    startTime = Utils.getCurrentTimeSeconds();
    xController.reset();
    yController.reset();
    headingController.reset();
  }

  /** Runs every robot loop while the command is active. */
  @Override
  protected void execute() {
    // Sample the trajectory at the elapsed time since start. startState is fixed, so this is a
    // time-parameterized straight-line path, not a self-advancing generator.
    double t = Utils.getCurrentTimeSeconds() - startTime;
    LinearPath.State setpoint = path.calculate(t, startState, goal);

    Pose2d measuredPose = drivetrain.getPose();

    // Feedforward: the profile's field-relative velocity at time t. This is what drives the robot
    // along the line - the PID below only corrects the difference between where the profile says we
    // should be and where odometry says we are. LinearPath wrapped the heading goal to the shortest
    // path; the heading controller's continuous input keeps its error wrapped too.
    ChassisVelocities feedforward = setpoint.velocity;
    double vx = feedforward.vx + xController.calculate(measuredPose.getX(), setpoint.pose.getX());
    double vy = feedforward.vy + yController.calculate(measuredPose.getY(), setpoint.pose.getY());
    double omega =
        feedforward.omega
            + headingController.calculate(
                measuredPose.getRotation().getRadians(), setpoint.pose.getRotation().getRadians());

    drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities(vx, vy, omega)));
  }

  /**
   * Done when the trajectory's total time has elapsed. {@code execute()} ran first this loop, so
   * the path is primed for this start/goal and {@link LinearPath#isFinished(double)} compares the
   * elapsed time against the (constant) {@link LinearPath#totalTime() totalTime()}.
   */
  @Override
  protected boolean isFinished() {
    return path.isFinished(Utils.getCurrentTimeSeconds() - startTime);
  }

  /** Idles the drivetrain. Runs on both natural finish and interruption. */
  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
