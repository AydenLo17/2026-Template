// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.utils.ClassicCommand;

/**
 * Drive to a standoff point in front of an AprilTag, vision-only (no odometry).
 *
 * <p>The standoff isn't a code parameter: it's the Limelight's 3D point-of-interest offset (set in
 * the LL web UI, per-fiducial in the field map), so this command just drives the measured distance
 * to zero. Change the standoff by editing the POI offset on the camera.
 *
 * <p>"Classic-style" Commands v3 command on {@link ClassicCommand}: the v2 lifecycle hooks
 * ({@link #initialize}, {@link #execute}, {@link #isFinished}, {@link #end}) do the work and the
 * framework wires the coroutine. The whole feature - Limelight read, three trapezoidal PIDs, drive
 * request, done-condition - lives in this one file.
 */
public class DriveToTag extends ClassicCommand {
  // NetworkTables name of the Limelight ("limelight" is the default hostname).
  private static final String CAMERA = "limelight";

  // --- Translation limits. TODO: tune to the drivetrain's real capability. ---
  private static final double MAX_LINEAR_VELOCITY = 2.5; // m/s
  private static final double MAX_LINEAR_ACCEL = 3.0;    // m/s^2

  // --- Rotation limits. ---
  private static final double MAX_ANGULAR_VELOCITY = Math.PI;     // rad/s
  private static final double MAX_ANGULAR_ACCEL = 2.0 * Math.PI;  // rad/s^2

  // --- Feedback gains. Default 0: the feedforward in execute() does the work, PID only corrects
  // drift. TODO: tune. Raise kP if the bot trails the profile; lower it (or add kD) if it
  // oscillates near the goal. ---
  private static final double TRANSLATION_KP = 0.0;
  private static final double HEADING_KP = 0.0;

  // --- "Close enough" tolerances. ---
  private static final double TRANSLATION_TOLERANCE = 0.03;             // meters
  private static final double HEADING_TOLERANCE = Math.toRadians(2.0);  // radians

  private final DriveMechanism drivetrain;

  // One trapezoidal PID per axis. The profile inside each gives acceleration limiting - a plain
  // PIDController would command full output instantly.
  private final ProfiledPIDController distance = new ProfiledPIDController(
      TRANSLATION_KP, 0.0, 0.0,
      new TrapezoidProfile.Constraints(MAX_LINEAR_VELOCITY, MAX_LINEAR_ACCEL));
  private final ProfiledPIDController lateral = new ProfiledPIDController(
      TRANSLATION_KP, 0.0, 0.0,
      new TrapezoidProfile.Constraints(MAX_LINEAR_VELOCITY, MAX_LINEAR_ACCEL));
  private final ProfiledPIDController heading = new ProfiledPIDController(
      HEADING_KP, 0.0, 0.0,
      new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCEL));

  // Robot-relative velocity request: open-loop drive so no drive-velocity PID tuning is required.
  private final SwerveRequest.ApplyRobotVelocity driveRequest = new SwerveRequest.ApplyRobotVelocity()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public DriveToTag(DriveMechanism drivetrain) {
    super("DriveToTag", drivetrain); // name + requirement, like v2 addRequirements(drivetrain)
    this.drivetrain = drivetrain;
    heading.enableContinuousInput(-Math.PI, Math.PI);
    distance.setTolerance(TRANSLATION_TOLERANCE);
    lateral.setTolerance(TRANSLATION_TOLERANCE);
    heading.setTolerance(HEADING_TOLERANCE);
  }

  /**
   * Seeds the profiles to the current measurement so the approach starts from a standstill. If no
   * tag is in view, skip seeding: {@link #execute} holds still until the tag appears and
   * {@link #isFinished} gates on visibility, so we won't falsely report "done."
   */
  @Override
  protected void initialize() {
    Pose3d robotInTag = LimelightHelpers.getBotPose3d_TargetSpace(CAMERA);
    if (robotInTag.equals(Pose3d.kZero)) {
      return;
    }
    
    distance.reset(Math.abs(robotInTag.getZ()));
    lateral.reset(-robotInTag.getX());
    heading.reset(-robotInTag.getRotation().getY());
  }

  /** Runs every robot loop while the command is active. */
  @Override
  protected void execute() {
    // Robot pose in target space (+X right of tag, +Y down, +Z out of the tag face); zero Pose3d
    // means no valid target. Preferred over getTV(), which can flip true before target-space fills.
    Pose3d robotInTag = LimelightHelpers.getBotPose3d_TargetSpace(CAMERA);
    if (robotInTag.equals(Pose3d.kZero)) {
      drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities()));
      return;
    }

    double measuredDistance = Math.abs(robotInTag.getZ());      // out from the tag, always positive
    double measuredLateral = -robotInTag.getX();                // + = robot LEFT of the tag normal
    double measuredHeading = -robotInTag.getRotation().getY();  // + = robot rotated CCW

    // Tag-frame velocities: PID + profile-velocity feedforward. FF commands the profile's velocity
    // directly; PID only corrects drift. Without FF the robot trails the setpoint.
    double vx = -(distance.calculate(measuredDistance, 0.0) + distance.getSetpoint().velocity); // + toward tag (standoff = POI offset)
    double vy = lateral.calculate(measuredLateral, 0.0) + lateral.getSetpoint().velocity;       // + to the robot's left
    double omega = heading.calculate(measuredHeading, 0.0) + heading.getSetpoint().velocity;    // + CCW

    // Rotate tag-frame velocities into the body frame and command the swerve.
    ChassisVelocities body = new ChassisVelocities(vx, vy, omega)
        .toRobotRelative(Rotation2d.fromRadians(measuredHeading));
    drivetrain.setControl(driveRequest.withVelocity(body));
  }

  /**
   * Done when we have a valid target-space reading AND all three controllers are at-goal. The
   * validity gate avoids a false "done" before the first calculate() sets the goal - a fresh
   * controller reports at-goal because its goal and setpoint both default to zero.
   */
  @Override
  protected boolean isFinished() {
    return !LimelightHelpers.getBotPose3d_TargetSpace(CAMERA).equals(Pose3d.kZero)
        && distance.atGoal()
        && lateral.atGoal()
        && heading.atGoal();
  }

  /** Idles the drivetrain. Runs on both natural finish and interruption. */
  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
}
