// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.vision.LimelightHelpers;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * Drive to a standoff point in front of an AprilTag, vision-only (no odometry).
 *
 * <p>The standoff isn't a code parameter: it's the Limelight's 3D point-of-interest offset (set in
 * the LL web UI, per-fiducial in the field map), so this command just drives the measured distance
 * to zero. Change the standoff by editing the POI offset on the camera.
 *
 * <p>"Inline-style" Commands v3 command: a single linear coroutine body does the work - seed the
 * profiles, then a while-loop that reads the camera, drives the three trapezoidal PIDs, and yields
 * each iteration until at-goal. The whole feature - Limelight read, three trapezoidal PIDs, drive
 * request, done-condition - lives in this one file.
 *
 * <p><b>Two styles, same behavior:</b> {@link DriveToTag} is this exact command written with the
 * classic {@code initialize/execute/isFinished/end} lifecycle on {@link
 * frc.robot.utils.ClassicCommand}. Compare the two to see the trade-off - one linear body here vs.
 * explicit lifecycle hooks there.
 */
public final class DriveToTagInline {
  private DriveToTagInline() {}

  /**
   * Creates the drive-to-tag command. The controllers are locals, so each schedule starts fresh.
   */
  public static Command create(DriveMechanism drivetrain, String camera, int targetTagId) {
    // One trapezoidal PID per axis. The profile inside each (max velocity, max accel) gives
    // acceleration limiting - a plain PIDController would command full output instantly.
    // Translation
    // limits are m/s and m/s^2, rotation rad/s and rad/s^2. The kP gains default to 0: the
    // feedforward below does the work, PID only corrects drift. TODO: tune both - the limits to the
    // drivetrain's real capability, and kP (raise if the bot trails the profile; lower, or add kD,
    // if it oscillates near the goal).
    ProfiledPIDController distance =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
    ProfiledPIDController lateral =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.5, 3.0));
    ProfiledPIDController heading =
        new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, 2.0 * Math.PI));

    // Robot-relative velocity request: open-loop drive so no drive-velocity PID tuning is required.
    SwerveRequest.ApplyRobotVelocity driveRequest =
        new SwerveRequest.ApplyRobotVelocity()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    heading.enableContinuousInput(-Math.PI, Math.PI);
    distance.setTolerance(0.03); // meters
    lateral.setTolerance(0.03); // meters
    heading.setTolerance(Math.toRadians(2.0)); // radians

    return drivetrain
        .run(
            (Coroutine coroutine) -> {
              // Make our tag the camera's primary target, so target-space pose is measured against
              // it. In the lambda (not create()) so it re-applies on every schedule.
              LimelightHelpers.setPriorityTagID(camera, targetTagId);

              // initialize: seed the profiles to the current measurement so the approach starts
              // from a
              // standstill. If no tag is in view, skip seeding: the loop below holds still until
              // the tag
              // appears and the finish check gates on visibility, so we won't falsely report
              // "done."
              Pose3d robotInTag =
                  onTargetTag(camera, targetTagId)
                      ? LimelightHelpers.getBotPose3d_TargetSpace(camera)
                      : Pose3d.kZero;
              if (!robotInTag.equals(Pose3d.kZero)) {
                distance.reset(Math.abs(robotInTag.getZ()));
                lateral.reset(-robotInTag.getX());
                heading.reset(-robotInTag.getRotation().getY());
              }

              // execute + isFinished: runs every robot loop while the command is active.
              while (true) {
                // If the camera's primary tag isn't ours (wrong tag, or none in view), don't drive
                // - idle and wait for it.
                if (!onTargetTag(camera, targetTagId)) {
                  drivetrain.setControl(new SwerveRequest.Idle());
                  coroutine.yield();
                  continue;
                }

                // Robot pose in target space (+X right of tag, +Y down, +Z out of the tag face);
                // zero Pose3d means the target-space data hasn't filled yet (it can lag the tag id
                // by a frame). Preferred over getTV(), which can flip true before target-space
                // fills.
                robotInTag = LimelightHelpers.getBotPose3d_TargetSpace(camera);
                if (robotInTag.equals(Pose3d.kZero)) {
                  drivetrain.setControl(new SwerveRequest.Idle());
                  coroutine.yield();
                  continue;
                }

                double measuredDistance =
                    Math.abs(robotInTag.getZ()); // out from the tag, always positive
                double measuredLateral = -robotInTag.getX(); // + = robot LEFT of the tag normal
                double measuredHeading = -robotInTag.getRotation().getY(); // + = robot rotated CCW

                // Tag-frame velocities: PID + profile-velocity feedforward. FF commands the
                // profile's velocity
                // directly; PID only corrects drift. Without FF the robot trails the setpoint.
                double vx =
                    -(distance.calculate(measuredDistance, 0.0)
                        + distance.getSetpoint().velocity); // + toward tag (standoff = POI offset)
                double vy =
                    lateral.calculate(measuredLateral, 0.0)
                        + lateral.getSetpoint().velocity; // + to the robot's left
                double omega =
                    heading.calculate(measuredHeading, 0.0)
                        + heading.getSetpoint().velocity; // + CCW

                // Rotate tag-frame velocities into the body frame and command the swerve.
                ChassisVelocities body =
                    new ChassisVelocities(vx, vy, omega)
                        .toRobotRelative(Rotation2d.fromRadians(measuredHeading));
                drivetrain.setControl(driveRequest.withVelocity(body));

                // Done when we have a valid target-space reading AND all three controllers are
                // at-goal. The
                // validity gate avoids a false "done" before the first calculate() sets the goal -
                // a fresh
                // controller reports at-goal because its goal and setpoint both default to zero.
                if (distance.atGoal() && lateral.atGoal() && heading.atGoal()) {
                  break;
                }
                coroutine.yield();
              }

              // end: idle the drivetrain and clear the tag priority.
              drivetrain.setControl(new SwerveRequest.Idle());
              LimelightHelpers.setPriorityTagID(camera, -1); // back to normal targeting
            })
        .whenCanceled(
            () -> {
              drivetrain.setControl(new SwerveRequest.Idle());
              LimelightHelpers.setPriorityTagID(camera, -1);
            })
        .named("DriveToTag");
  }

  /**
   * True when the camera's primary in-view tag is {@code targetTagId}. setPriorityTagID makes it
   * the primary target when visible; this guards the case where it isn't (wrong tag or none), so we
   * idle instead of aligning to the wrong tag.
   */
  private static boolean onTargetTag(String camera, int targetTagId) {
    return (int) LimelightHelpers.getFiducialID(camera) == targetTagId;
  }
}
