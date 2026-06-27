// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.commands.DriveToPose;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * An autonomous routine. In the OpMode model each routine is its own {@code @Autonomous} class -
 * this replaces the {@code SendableChooser}. Add another annotated class per routine and they all
 * appear by name on the driver station; the one the driver selects is the only one constructed.
 *
 * <p>This example sequences two {@link DriveToPose} legs with {@code andThen}: each leg runs to
 * completion (the bot reaches the pose) before the next starts. The resulting sequential group
 * inherits its children's requirements (the drivetrain), and the scheduler hands the drivetrain off
 * between legs. Swap in your real field poses, or add more legs / superstructure actions.
 *
 * <p>{@link #start()} fires once when the robot is enabled, which is where the routine is
 * scheduled.
 */
@Autonomous(name = "Drive To Pose")
public class AutonomousOpMode extends PeriodicOpMode {
  private final Command routine;

  public AutonomousOpMode(Robot robot) {
    // Field poses are blue-origin (x forward from the blue wall, y left). TODO: replace with the
    // real poses for your routine. Assumes odometry has been seeded to the starting pose.
    final Pose2d firstLeg = new Pose2d(2.0, 0.0, Rotation2d.kZero); // 2 m straight ahead
    final Pose2d secondLeg =
        new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(90)); // then 1.5 m left, facing +y

    routine =
        Command.sequence(
                new DriveToPose(robot.drivetrain, firstLeg),
                new DriveToPose(robot.drivetrain, secondLeg))
            .named("DriveToPose Auto");
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(routine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(routine);
  }
}
