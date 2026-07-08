// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import frc.robot.Constants;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;

/**
 * One Limelight camera that feeds AprilTag pose guesses into the drivetrain's pose estimator. Call
 * {@link #registerAll} once from {@link frc.robot.Robot} to wire up every camera.
 *
 * <p>Uses MegaTag1 for 2+ tags (vision heading) and MegaTag2 for a lone tag (gyro heading), so seed
 * the gyro or single-tag vision will be off. Accepted measurements are fused by {@link
 * frc.robot.subsystems.RobotState} (via {@code DriveMechanism.addVisionMeasurement}). Does nothing
 * in sim (no camera). Publishes per-camera diagnostics under {@code NT:/Vision/<camera>/*}.
 */
public class Limelight {
  // Limelight 3 FOV scaling used to convert txnc/tync degrees into normalized crop center.
  // Keep internal to avoid overloading the public tuning surface.
  private static final double kTxDegreesForFullScale = 29.8;
  private static final double kTyDegreesForFullScale = 24.85;

  // Internal safety pad so slight target motion doesn't clip out of the crop.
  private static final double kCropCenterPadding = 0.08;

  /** Enable distance-based dynamic Limelight crop windows. */
  public static final boolean kEnableDynamicCropWindow = true;

  private final String name;
  private final DriveMechanism drivetrain;

  // Vision diagnostics for tuning in AdvantageScope. Everything is under
  // NT:/Vision/<camera>/* and therefore also in WPILOG through DataLogManager.
  private final NetworkTable table;
  private final StringPublisher status;
  private final DoublePublisher tagCount;
  private final DoublePublisher avgTagDist;
  private final DoublePublisher avgTagArea;
  private final DoublePublisher latencyMs;
  private final DoublePublisher measurementAgeSec;
  private final DoublePublisher ambiguity;
  private final DoublePublisher poseJumpMeters;
  private final DoublePublisher xyStdDev;
  private final DoublePublisher thetaStdDev;
  private final DoublePublisher accepted;
  private final DoublePublisher cropXMin;
  private final DoublePublisher cropXMax;
  private final DoublePublisher cropYMin;
  private final DoublePublisher cropYMax;
  private final DoublePublisher cropScale;
  private final StringPublisher cropMode;

  private double lastCropXMin = -1.0;
  private double lastCropXMax = 1.0;
  private double lastCropYMin = -1.0;
  private double lastCropYMax = 1.0;

  private Limelight(String name, DriveMechanism drivetrain) {
    this.name = name;
    this.drivetrain = drivetrain;

    this.table = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(name);
    this.status = table.getStringTopic("Status").publish();
    this.tagCount = table.getDoubleTopic("TagCount").publish();
    this.avgTagDist = table.getDoubleTopic("AvgTagDistMeters").publish();
    this.avgTagArea = table.getDoubleTopic("AvgTagArea").publish();
    this.latencyMs = table.getDoubleTopic("LatencyMs").publish();
    this.measurementAgeSec = table.getDoubleTopic("MeasurementAgeSec").publish();
    this.ambiguity = table.getDoubleTopic("WorstAmbiguity").publish();
    this.poseJumpMeters = table.getDoubleTopic("PoseJumpMeters").publish();
    this.xyStdDev = table.getDoubleTopic("AppliedXYStdDev").publish();
    this.thetaStdDev = table.getDoubleTopic("AppliedThetaStdDev").publish();
    this.accepted = table.getDoubleTopic("Accepted").publish();
    this.cropXMin = table.getDoubleTopic("CropXMin").publish();
    this.cropXMax = table.getDoubleTopic("CropXMax").publish();
    this.cropYMin = table.getDoubleTopic("CropYMin").publish();
    this.cropYMax = table.getDoubleTopic("CropYMax").publish();
    this.cropScale = table.getDoubleTopic("CropScale").publish();
    this.cropMode = table.getStringTopic("CropMode").publish();
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
    accepted.set(0.0);

    // Feed the camera our heading for MegaTag2. Use the odometry-only heading (not
    // the fused pose)
    // so single-tag vision never depends on a heading that vision itself corrected
    // - that would be
    // a feedback loop. NoFlush: Robot flushes once for all cameras.
    double headingDegrees = drivetrain.getOdometryPose().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation_NoFlush(name, headingDegrees, 0, 0, 0, 0, 0);

    // MegaTag1 for 2+ tags; switch to MegaTag2 for a single tag.
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    if (LimelightHelpers.validPoseEstimate(estimate) && estimate.tagCount == 1) {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    updateDynamicCropWindow(estimate);

    tagCount.set(estimate.tagCount);
    avgTagDist.set(estimate.avgTagDist);
    avgTagArea.set(estimate.avgTagArea);
    latencyMs.set(estimate.latency);

    double ageSec = Math.max(0.0, Utils.getCurrentTimeSeconds() - estimate.timestampSeconds);
    measurementAgeSec.set(ageSec);

    double worstAmbiguity = worstFiducialAmbiguity(estimate);
    ambiguity.set(worstAmbiguity);

    Pose2d currentPose = drivetrain.getPose();
    double correctionJump =
        currentPose.getTranslation().getDistance(estimate.pose.getTranslation());
    poseJumpMeters.set(correctionJump);

    // Hard quality gates first - reject clearly bad measurements to prevent
    // estimator pollution.
    if (!LimelightHelpers.validPoseEstimate(estimate)) {
      status.set("reject:invalid");
      return;
    }

    if (estimate.avgTagDist > Constants.Vision.kMaxTagDistanceMeters) {
      status.set("reject:far_tags");
      return;
    }

    if (estimate.latency > Constants.Vision.kMaxLatencyMs) {
      status.set("reject:high_latency");
      return;
    }

    if (ageSec > Constants.Vision.kMaxMeasurementAgeSec) {
      status.set("reject:stale");
      return;
    }

    if (estimate.avgTagArea < Constants.Vision.kMinAvgTagArea) {
      status.set("reject:low_area");
      return;
    }

    if (estimate.tagCount == 1 && worstAmbiguity > Constants.Vision.kMaxSingleTagAmbiguity) {
      status.set("reject:single_tag_ambiguity");
      return;
    }

    double jumpGate =
        estimate.tagCount >= 2
            ? Constants.Vision.kMaxPoseJumpMetersMultiTag
            : Constants.Vision.kMaxPoseJumpMetersSingleTag;
    if (correctionJump > jumpGate) {
      status.set("reject:pose_jump");
      return;
    }

    // Near-origin reject: a solve sitting on the field origin is the signature of a
    // bad/empty read.
    if (estimate.pose.getTranslation().getNorm() < Constants.Vision.kMinPoseNormMeters) {
      status.set("reject:near_origin");
      return;
    }

    // Off-field reject: a real measurement is always on the field (plus a small
    // footprint margin);
    // anything well outside is an outlier that would yank the estimate off the
    // field.
    double margin = Constants.Vision.kFieldBoundaryMarginMeters;
    double x = estimate.pose.getX();
    double y = estimate.pose.getY();
    if (x < -margin
        || y < -margin
        || x > Constants.Vision.kFieldLengthMeters + margin
        || y > Constants.Vision.kFieldWidthMeters + margin) {
      status.set("reject:off_field");
      return;
    }

    // Fast-yaw reject: while spinning quickly the camera-to-robot time sync is
    // unreliable, so a
    // single-tag solve (which leans on our heading) can be wildly wrong. Multi-tag
    // solves carry
    // their own heading and are far more robust, so only gate the single-tag case.
    double yawRate = Math.abs(drivetrain.getFieldVelocity().omega);
    if (estimate.tagCount == 1 && yawRate > Constants.Vision.kMaxAngularSpeedForVision) {
      status.set("reject:fast_yaw");
      return;
    }

    // Base trust model: closer tags + more tags => lower std-dev.
    double distanceFactor = Math.pow(estimate.avgTagDist, 1.2);
    double tagFactor = estimate.tagCount * estimate.tagCount;
    double xy = Constants.Vision.kXYStdDevCoefficient * distanceFactor / tagFactor;

    // Adaptive trust shaping. Inflate std-dev when robot is moving faster, tag
    // ambiguity is high,
    // observed area is low, or correction is large relative to odometry.
    double speed = Math.hypot(drivetrain.getFieldVelocity().vx, drivetrain.getFieldVelocity().vy);
    xy *= (1.0 + Constants.Vision.kVelocityStdDevInflationGain * speed);
    xy *= (1.0 + Constants.Vision.kAmbiguityStdDevInflationGain * worstAmbiguity);
    xy *=
        (1.0
            + Constants.Vision.kLowAreaStdDevInflationGain
                * Math.max(0.0, 1.0 - estimate.avgTagArea));
    xy *= (1.0 + Constants.Vision.kCorrectionStdDevInflationGain * correctionJump);

    double heading =
        estimate.isMegaTag2
            ? Constants.Vision.kIgnoreVisionHeadingStdDev
            : Constants.Vision.kHeadingStdDevCoefficient * distanceFactor / tagFactor;

    xyStdDev.set(xy);
    thetaStdDev.set(heading);

    drivetrain.addVisionMeasurement(
        estimate.pose, estimate.timestampSeconds, VecBuilder.fill(xy, xy, heading));
    accepted.set(1.0);
    status.set(estimate.isMegaTag2 ? "accept:megatag2" : "accept:megatag1");
  }

  private void updateDynamicCropWindow(PoseEstimate estimate) {
    if (!kEnableDynamicCropWindow) {
      applyCropWindow(-1.0, 1.0, -1.0, 1.0);
      cropScale.set(1.0);
      cropMode.set("disabled");
      return;
    }

    if (!LimelightHelpers.validPoseEstimate(estimate) || estimate.tagCount <= 0) {
      applyCropWindow(-1.0, 1.0, -1.0, 1.0);
      cropScale.set(1.0);
      cropMode.set("fallback:no_target");
      return;
    }

    // Distance-based crop sizing: closer tags => wider crop, farther tags =>
    // tighter crop.
    double t =
        normalize01(
            estimate.avgTagDist,
            Constants.Vision.kCropNearDistanceMeters,
            Constants.Vision.kCropFarDistanceMeters);
    double halfWindow =
        lerp(Constants.Vision.kCropWindowHalfSizeNear, Constants.Vision.kCropWindowHalfSizeFar, t);

    LimelightHelpers.RawFiducial centerFiducial = selectBestFiducial(estimate);
    if (centerFiducial == null) {
      applyCropWindow(-1.0, 1.0, -1.0, 1.0);
      cropScale.set(1.0);
      cropMode.set("fallback:no_fiducial");
      return;
    }

    // Center crop around the strongest fiducial's normalized crosshair angles.
    double centerX = clamp(centerFiducial.txnc / kTxDegreesForFullScale, -1.0, 1.0);
    double centerY = clamp(centerFiducial.tync / kTyDegreesForFullScale, -1.0, 1.0);

    double xMin = centerX - halfWindow - kCropCenterPadding;
    double xMax = centerX + halfWindow + kCropCenterPadding;
    double yMin = centerY - halfWindow - kCropCenterPadding;
    double yMax = centerY + halfWindow + kCropCenterPadding;

    // Shift into bounds while preserving requested window size as much as possible.
    double desiredWidth = xMax - xMin;
    double desiredHeight = yMax - yMin;
    if (xMin < -1.0) {
      xMax += (-1.0 - xMin);
      xMin = -1.0;
    }
    if (xMax > 1.0) {
      xMin -= (xMax - 1.0);
      xMax = 1.0;
    }
    if (yMin < -1.0) {
      yMax += (-1.0 - yMin);
      yMin = -1.0;
    }
    if (yMax > 1.0) {
      yMin -= (yMax - 1.0);
      yMax = 1.0;
    }

    // Final clamp plus minimum span protection.
    xMin = clamp(xMin, -1.0, 1.0);
    xMax = clamp(xMax, -1.0, 1.0);
    yMin = clamp(yMin, -1.0, 1.0);
    yMax = clamp(yMax, -1.0, 1.0);
    if (xMax - xMin < 0.05) {
      double mid = 0.5 * (xMin + xMax);
      xMin = clamp(mid - 0.025, -1.0, 1.0);
      xMax = clamp(mid + 0.025, -1.0, 1.0);
    }
    if (yMax - yMin < 0.05) {
      double mid = 0.5 * (yMin + yMax);
      yMin = clamp(mid - 0.025, -1.0, 1.0);
      yMax = clamp(mid + 0.025, -1.0, 1.0);
    }

    applyCropWindow(xMin, xMax, yMin, yMax);
    double areaScale = ((xMax - xMin) * (yMax - yMin)) / 4.0;
    cropScale.set(clamp(areaScale, 0.0, 1.0));
    cropMode.set(
        String.format(
            "dynamic:t=%.2f,d=%.2f,w=%.2f,h=%.2f",
            t, estimate.avgTagDist, desiredWidth, desiredHeight));
  }

  private void applyCropWindow(double xMin, double xMax, double yMin, double yMax) {
    if (Math.abs(xMin - lastCropXMin) < Constants.Vision.kCropApplyDeadband
        && Math.abs(xMax - lastCropXMax) < Constants.Vision.kCropApplyDeadband
        && Math.abs(yMin - lastCropYMin) < Constants.Vision.kCropApplyDeadband
        && Math.abs(yMax - lastCropYMax) < Constants.Vision.kCropApplyDeadband) {
      cropXMin.set(lastCropXMin);
      cropXMax.set(lastCropXMax);
      cropYMin.set(lastCropYMin);
      cropYMax.set(lastCropYMax);
      return;
    }

    LimelightHelpers.setCropWindow(name, xMin, xMax, yMin, yMax);
    lastCropXMin = xMin;
    lastCropXMax = xMax;
    lastCropYMin = yMin;
    lastCropYMax = yMax;
    cropXMin.set(xMin);
    cropXMax.set(xMax);
    cropYMin.set(yMin);
    cropYMax.set(yMax);
  }

  private static LimelightHelpers.RawFiducial selectBestFiducial(PoseEstimate estimate) {
    if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
      return null;
    }
    LimelightHelpers.RawFiducial best = estimate.rawFiducials[0];
    for (int i = 1; i < estimate.rawFiducials.length; i++) {
      LimelightHelpers.RawFiducial candidate = estimate.rawFiducials[i];
      if (candidate.ta > best.ta) {
        best = candidate;
      }
    }
    return best;
  }

  private static double normalize01(double value, double min, double max) {
    if (max <= min) {
      return 0.0;
    }
    return clamp((value - min) / (max - min), 0.0, 1.0);
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static double worstFiducialAmbiguity(PoseEstimate estimate) {
    if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
      return 0.0;
    }
    double worst = 0.0;
    for (LimelightHelpers.RawFiducial fiducial : estimate.rawFiducials) {
      worst = Math.max(worst, fiducial.ambiguity);
    }
    return worst;
  }
}
