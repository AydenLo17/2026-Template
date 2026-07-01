// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.ChoreoTrajectory;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Autonomous routine factory - the one place to compose auto commands.
 *
 * <p>This replaces the old per-routine {@code @Autonomous} OpMode classes as the place where the
 * actual <i>command</i> for a routine is built. An OpMode in {@link frc.robot.opmodes.OpModes} just
 * picks a routine from here by name and schedules it; the sequencing logic lives here so routines
 * are easy to find, reuse, and unit-test without the OpMode lifecycle.
 *
 * <p>Two flavors of routine:
 *
 * <ul>
 *   <li><b>Choreo paths</b> - {@link #choreoPath} / {@link #choreoSequence} load {@code .traj}
 *       files from {@code src/main/deploy/choreo/} and follow them with {@link
 *       AdvancedTrackTrajectory} (feedforward + position trim + traction limiting).
 *   <li><b>On-the-fly poses</b> - {@link #drivePoses} profiles straight to a list of goal poses
 *       with no path file, handy for quick tests or simple moves.
 * </ul>
 */
public final class Autos {
  private Autos() {} // factory of static methods - never instantiated

  /**
   * Follows a single Choreo path by file name (no extension needed).
   *
   * @param drivetrain the swerve drive
   * @param pathName the {@code .traj} file in {@code deploy/choreo/}, e.g. {@code "TestPath1"}
   * @return a command that follows the path and finishes at its last sample
   */
  public static Command choreoPath(DriveMechanism drivetrain, String pathName) {
    var path = ChoreoTrajectory.load(pathName);
    return Command.sequence(
            Command.requiring(drivetrain)
                .executing(coroutine -> drivetrain.resetPose(path[0].pose()))
                .named("Reset Auto Pose"),
            new AdvancedTrackTrajectory(drivetrain, path))
        .named("Choreo Path");
  }

  /**
   * Follows several Choreo paths back-to-back. Each leg finishes (at-goal) before the next starts,
   * so the legs hand the drivetrain off cleanly - drop superstructure actions between legs as
   * needed.
   *
   * @param drivetrain the swerve drive
   * @param pathNames the {@code .traj} file names in order
   * @return a sequential command running every path
   */
  public static Command choreoSequence(DriveMechanism drivetrain, String... pathNames) {
    if (pathNames.length == 0) {
      return Command.noRequirements(coroutine -> {}).named("Choreo Sequence");
    }

    var loadedPaths = new frc.robot.utils.CustomTrajectoryEngine.Sample[pathNames.length][];
    for (int i = 0; i < pathNames.length; i++) {
      loadedPaths[i] = ChoreoTrajectory.load(pathNames[i]);
    }

    Command[] auto = new Command[pathNames.length + 1];
    auto[0] =
        Command.requiring(drivetrain)
            .executing(coroutine -> drivetrain.resetPose(loadedPaths[0][0].pose()))
            .named("Reset Auto Pose");
    for (int i = 0; i < pathNames.length; i++) {
      auto[i + 1] = new AdvancedTrackTrajectory(drivetrain, loadedPaths[i]);
    }
    return Command.sequence(auto).named("Choreo Sequence");
  }

  /**
   * Drives straight to each goal pose in turn using on-the-fly profiles (no path file). Useful for
   * a quick routine or sim test before any Choreo paths exist.
   *
   * @param drivetrain the swerve drive
   * @param goals the field poses (blue-origin) to visit in order
   * @return a sequential command visiting every pose
   */
  public static Command drivePoses(DriveMechanism drivetrain, Pose2d... goals) {
    Command[] legs = new Command[goals.length];
    for (int i = 0; i < goals.length; i++) {
      legs[i] = new AdvancedTrackTrajectory(drivetrain, goals[i]);
    }
    return Command.sequence(legs).named("Drive Poses");
  }

  /**
   * Example on-the-fly routine (no Choreo file required): forward 3 m, then left 2 m facing +Y.
   * This is what the "Advanced Trajectory" OpMode runs, and is a good sim smoke test.
   */
  public static Command exampleTwoPose(DriveMechanism drivetrain) {
    return drivePoses(
        drivetrain,
        new Pose2d(3.0, 0.0, Rotation2d.kZero),
        new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(90)));
  }
}
