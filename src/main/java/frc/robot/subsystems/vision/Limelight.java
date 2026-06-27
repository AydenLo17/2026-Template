// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.linalg.VecBuilder;

/**
 * One Limelight camera that feeds AprilTag pose guesses into the drivetrain's pose estimator. Call
 * {@link #registerAll} once from {@link frc.robot.Robot} to wire up every camera.
 *
 * <p>Uses MegaTag1 for 2+ tags (vision heading) and MegaTag2 for a lone tag (gyro heading), so seed
 * the gyro or single-tag vision will be off. Does nothing in sim (no camera). Publishes no
 * telemetry yet - waiting on WPILib's new logging API.
 */
public class Limelight {
  // Trust model: stdDev = COEFFICIENT * distance^1.2 / tagCount^2 (bigger = trust less).
  private static final double XY_STD_DEV_COEFFICIENT = 0.333;
  private static final double ROTATION_STD_DEV_COEFFICIENT = 1.5; // MegaTag1 heading trust
  private static final double MAX_TAG_DISTANCE_METERS = 4.0; // skip far, noisy tags
  private static final double IGNORE_VISION_HEADING = 9_999_999; // MT2: gyro owns heading

  private final String name;
  private final DriveMechanism drivetrain;

  private Limelight(String name, DriveMechanism drivetrain) {
    this.name = name;
    this.drivetrain = drivetrain;
  }

  /**
   * Creates one camera per name and registers them all on the scheduler - each camera's update
   * every loop, then one shared flush. Names must match each camera's NetworkTables name.
   */
  public static void registerAll(DriveMechanism drivetrain, String... cameraNames) {
    for (String name : cameraNames) {
      Limelight camera = new Limelight(name, drivetrain);
      Scheduler.getDefault().addPeriodic(camera::update);
    }
    // One flush per loop pushes every camera's NoFlush heading write at once.
    Scheduler.getDefault().addPeriodic(LimelightHelpers::Flush);
  }

  /** Runs one vision update for this camera. */
  private void update() {
    // Feed the camera our heading for MegaTag2. NoFlush: Robot flushes once for all cameras.
    double headingDegrees = drivetrain.getPose().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation_NoFlush(name, headingDegrees, 0, 0, 0, 0, 0);

    // MegaTag1 for 2+ tags; switch to MegaTag2 for a single tag.
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    if (LimelightHelpers.validPoseEstimate(estimate) && estimate.tagCount == 1) {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    // Skip if no tag is in view, or the tags are too far to trust.
    if (!LimelightHelpers.validPoseEstimate(estimate)
        || estimate.avgTagDist > MAX_TAG_DISTANCE_METERS) {
      return;
    }

    // Trust closer tags and more tags more. MT1 gives a real heading; MT2 lets the gyro own it.
    double distanceFactor = Math.pow(estimate.avgTagDist, 1.2);
    double tagFactor = estimate.tagCount * estimate.tagCount;
    double xyStdDev = XY_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;
    double headingStdDev =
        estimate.isMegaTag2
            ? IGNORE_VISION_HEADING
            : ROTATION_STD_DEV_COEFFICIENT * distanceFactor / tagFactor;
    drivetrain.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, headingStdDev));
  }
}
