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
import org.wpilib.math.controller.PIDController;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * Heading-alignment command that doubles as the target for the headless AI tuning loop described in
 * {@code ISAAC_SIM_AUTOMATION.md}.
 *
 * <p><b>What it does.</b> It runs a single PID (gains from the {@link Constants.VisionAutoAlign}
 * tunable zone) on the robot's heading <i>error in degrees</i> and commands a field-centric yaw
 * rate to drive that error to zero, holding translation at zero. When the error stays inside {@link
 * Constants.VisionAutoAlign#kMaxAlignmentTargetErrorDegrees} for a short dwell it reports the
 * elapsed time as the alignment metric and finishes.
 *
 * <p><b>Why it publishes to NetworkTables.</b> The optional Isaac Sim bridge ({@code
 * isaac_sim_bridge.py}) is a NetworkTables client: this command publishes the commanded yaw rate
 * and goal heading under {@code SmartDashboard/Swerve/*} (so the bridge can drive the physics
 * robot) and reads a ground-truth heading back from {@code SmartDashboard/SimSensors/GyroHeading}
 * (so alignment is scored against Isaac's physics when it is running). When Isaac is <i>not</i>
 * running that key is absent, so the command falls back to the WPILib/CTRE simulated heading and
 * the whole tuning loop still runs headless with no external dependencies.
 *
 * <p><b>The metric.</b> On convergence (or timeout) it publishes {@code
 * SmartDashboard/Vision/MetricAlignmentTime} <i>and</i> prints an {@code [AI_METRIC]} line to
 * stdout. The orchestrator ({@code run_ai_tuning_cycle.py}) greps that line to score each gain set,
 * so the loop works even in a pure-WPILib headless sim with no ntcore Python client.
 */
public class VisionAutoAlign extends ClassicCommand {
  private final DriveMechanism drivetrain;
  private final double targetAngleDegrees;

  // Single heading PID. Gains are pulled from the tunable zone in initialize() (not here) so each
  // schedule picks up whatever the AI tuning loop last wrote to Constants.
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);

  // Rotate-in-place: field-centric with zero translation and a PID-computed yaw rate.
  private final SwerveRequest.FieldCentric rotate =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // NetworkTables bridge to the (optional) Isaac Sim physics engine. All under "SmartDashboard" to
  // match isaac_sim_bridge.py, which subscribes to Swerve/* and publishes SimSensors/GyroHeading.
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
  private final DoublePublisher targetSpeedPub =
      table.getDoubleTopic("Swerve/TargetSpeed").publish();
  private final DoublePublisher targetAnglePub =
      table.getDoubleTopic("Swerve/TargetAngle").publish();
  private final DoublePublisher metricPub =
      table.getDoubleTopic("Vision/MetricAlignmentTime").publish();
  // Ground-truth heading from Isaac; NaN default means "Isaac not connected" -> use sim heading.
  private final DoubleSubscriber gyroSub =
      table.getDoubleTopic("SimSensors/GyroHeading").subscribe(Double.NaN);

  private double startTimeSeconds;
  private int settledLoops;
  private boolean converged;

  /**
   * @param drivetrain the swerve mechanism to rotate
   * @param targetAngleDegrees field-relative heading to align to, in degrees
   */
  public VisionAutoAlign(DriveMechanism drivetrain, double targetAngleDegrees) {
    super("VisionAutoAlign", drivetrain);
    this.drivetrain = drivetrain;
    this.targetAngleDegrees = targetAngleDegrees;
    // Heading wraps at +/-180 degrees, so the shortest-path error is continuous across the seam.
    controller.enableContinuousInput(-180.0, 180.0);
    controller.setTolerance(Constants.Tunable.kMaxAlignmentTargetErrorDegrees);
  }

  @Override
  protected void initialize() {
    // Re-read the tunable gains every schedule so a fresh run picks up the AI's latest write.
    controller.setPID(Constants.Tunable.kP, Constants.Tunable.kI, Constants.Tunable.kD);
    controller.reset();
    startTimeSeconds = Utils.getCurrentTimeSeconds();
    settledLoops = 0;
    converged = false;
    targetAnglePub.set(targetAngleDegrees);
  }

  @Override
  protected void execute() {
    double headingDeg = currentHeadingDegrees();

    // PID on heading error (degrees) -> yaw rate (rad/s), clamped so a hot kP can't ask for a
    // physically impossible spin.
    double omega =
        clamp(
            controller.calculate(headingDeg, targetAngleDegrees),
            -Constants.Tunable.kMaxAngularRateRadPerSec,
            Constants.Tunable.kMaxAngularRateRadPerSec);

    drivetrain.setControl(rotate.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(omega));

    // Publish the command so the Isaac bridge can apply it to the physics robot.
    targetSpeedPub.set(omega);

    // Shortest-path heading error in degrees, wrapped to [-180, 180] to match continuous input.
    double errorDeg = wrapDegrees(targetAngleDegrees - headingDeg);

    // Count consecutive in-tolerance loops; an overshoot resets the dwell so oscillation costs
    // time.
    if (Math.abs(errorDeg) <= Constants.Tunable.kMaxAlignmentTargetErrorDegrees) {
      settledLoops++;
    } else {
      settledLoops = 0;
    }
  }

  @Override
  protected boolean isFinished() {
    if (settledLoops >= Constants.Tunable.kSettleLoops) {
      converged = true;
      return true;
    }
    // Give up on a non-converging gain set so the loop doesn't hang; reported as a high time below.
    return elapsedSeconds() >= Constants.Tunable.kAlignmentTimeoutSeconds;
  }

  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
    targetSpeedPub.set(0.0);

    if (interrupted) {
      return; // A mode switch / cancel is not a measurement.
    }

    double duration = elapsedSeconds();
    metricPub.set(duration);
    // The orchestrator greps stdout for this exact tag/format; the sim bridge echoes it too.
    System.out.println(
        String.format(
            "[AI_METRIC] AlignmentTime: %.2fs | Success: %s",
            duration, converged ? "True" : "False"));
  }

  /** Elapsed time since {@link #initialize()} in seconds (Phoenix time base, works in sim). */
  private double elapsedSeconds() {
    return Utils.getCurrentTimeSeconds() - startTimeSeconds;
  }

  /**
   * Current heading in degrees. Prefers Isaac's ground-truth gyro when the bridge is connected (the
   * {@code SimSensors/GyroHeading} key is present, i.e. not NaN); otherwise uses the WPILib / CTRE
   * simulated pose heading so the loop runs with no external process.
   */
  private double currentHeadingDegrees() {
    double isaacHeading = gyroSub.get();
    if (!Double.isNaN(isaacHeading)) {
      return isaacHeading;
    }
    return drivetrain.getPose().getRotation().getDegrees();
  }

  /** Clamps {@code value} to the inclusive range {@code [min, max]}. */
  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  /** Wraps an angle in degrees to the shortest-path range {@code [-180, 180)}. */
  private static double wrapDegrees(double degrees) {
    double wrapped = (degrees + 180.0) % 360.0;
    if (wrapped < 0.0) {
      wrapped += 360.0;
    }
    return wrapped - 180.0;
  }
}
