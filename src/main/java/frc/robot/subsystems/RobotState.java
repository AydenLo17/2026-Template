// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import frc.robot.Constants;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;

/**
 * The robot's single source of truth for "where are we on the field", fusing wheel/gyro odometry
 * with AprilTag vision. This is our version of the {@code RobotState} class the top FRC teams (6328
 * Mechanical Advantage, 254, 2910) build their whole codebase around.
 *
 * <h2>Why a separate estimator at all?</h2>
 *
 * <p>The Phoenix swerve object already fuses vision internally, but it's a black box: you can't see
 * the odometry-only pose, you can't log the correction it applied, and you can't tune the fusion.
 * This class re-implements the fusion in the open so every knob and every intermediate value is
 * visible in the log (under {@code RobotState/*}) and controllable from {@link Constants.Estimator}
 * and {@link Constants.Vision}. That is exactly what lets you tune vision on a real field.
 *
 * <h2>The core trick: rewind, fuse, forward</h2>
 *
 * <p>A camera frame that lands at time {@code t} describes where the robot was ~50-100 ms ago, not
 * now. Naively snapping the current pose to a stale measurement fights the odometry. Instead we:
 *
 * <ol>
 *   <li>keep a short history of odometry poses in a {@link TimeInterpolatableBuffer} (rewind),
 *   <li>look up where odometry thought we were at the frame's timestamp and blend that with the
 *       vision pose using a Kalman gain (fuse),
 *   <li>re-apply all the odometry motion since that timestamp to bring the corrected pose back to
 *       the present (forward).
 * </ol>
 *
 * <h2>Threading</h2>
 *
 * <p>Every method here runs on the main robot loop ({@link DriveMechanism} feeds odometry from a
 * scheduler periodic; {@link frc.robot.subsystems.vision.Limelight} feeds vision from another). No
 * locking is needed as long as nothing calls in from the 250 Hz Phoenix odometry thread.
 */
public class RobotState {
  /** Odometry pose history, keyed by timestamp, for latency-compensated vision fusion. */
  private final TimeInterpolatableBuffer<Pose2d> odometryPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(Constants.Estimator.kPoseBufferSizeSeconds);

  /** Odometry trust as variances [x, y, theta] (std-dev squared). Higher => vision pulls harder. */
  private final double[] qStdDevs = new double[3];

  /** Pure odometry pose (no vision). This is the input we blend vision into. */
  private Pose2d odometryPose = Pose2d.kZero;

  /** Fused best-estimate pose (odometry + vision). This is what the rest of the robot reads. */
  private Pose2d estimatedPose = Pose2d.kZero;

  /** Latest field-relative velocity, forwarded straight from the drivetrain. */
  private ChassisVelocities fieldVelocity = new ChassisVelocities();

  /** False until the first odometry sample arrives, so we don't diff against a bogus baseline. */
  private boolean haveOdometryBaseline = false;

  // --- Logging (NT:/RobotState/*), captured to the WPILOG by DataLogManager. ---
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("RobotState");
  private final StructPublisher<Pose2d> estimatedPosePub =
      table.getStructTopic("EstimatedPose", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> odometryPosePub =
      table.getStructTopic("OdometryPose", Pose2d.struct).publish();
  private final DoublePublisher visionOdometryDeltaPub =
      table.getDoubleTopic("VisionMinusOdometryMeters").publish();
  private final DoublePublisher lastCorrectionPub =
      table.getDoubleTopic("LastCorrectionMeters").publish();
  private final DoublePublisher acceptedCountPub =
      table.getDoubleTopic("AcceptedVisionCount").publish();
  private final DoublePublisher rejectedCountPub =
      table.getDoubleTopic("RejectedVisionCount").publish();
  private final DoublePublisher secondsSinceVisionPub =
      table.getDoubleTopic("SecondsSinceVision").publish();

  private double acceptedCount = 0;
  private double rejectedCount = 0;
  private double lastVisionTimestamp = 0;

  public RobotState() {
    qStdDevs[0] = Math.pow(Constants.Estimator.kOdometryStdDevMeters, 2);
    qStdDevs[1] = Math.pow(Constants.Estimator.kOdometryStdDevMeters, 2);
    qStdDevs[2] = Math.pow(Constants.Estimator.kOdometryStdDevRadians, 2);
  }

  /**
   * Teleports both the odometry and fused pose to {@code pose} and clears the history. Autonomous
   * calls this at the start so the estimator's belief matches the trajectory's first point.
   */
  public void resetPose(Pose2d pose) {
    odometryPose = pose;
    estimatedPose = pose;
    odometryPoseBuffer.clear();
    odometryPoseBuffer.addSample(Utils.getCurrentTimeSeconds(), odometryPose);
    haveOdometryBaseline = true;
    publish();
  }

  /**
   * Feeds one odometry snapshot from the drivetrain. Call this every loop, before vision. The
   * incoming pose is the drivetrain's own odometry (which must NOT itself contain vision, or the
   * correction would be double-counted).
   *
   * @param newOdometryPose the drivetrain's latest odometry-only pose
   * @param fieldVel latest field-relative velocity (forwarded to consumers)
   * @param timestamp measurement time in the {@code Utils.getCurrentTimeSeconds()} epoch
   */
  public void addOdometryObservation(
      Pose2d newOdometryPose, ChassisVelocities fieldVel, double timestamp) {
    fieldVelocity = fieldVel;

    if (!haveOdometryBaseline) {
      odometryPose = newOdometryPose;
      estimatedPose = newOdometryPose;
      haveOdometryBaseline = true;
      odometryPoseBuffer.addSample(timestamp, odometryPose);
      publish();
      return;
    }

    // The robot-frame motion odometry reports since the last snapshot. Applying the same delta to
    // the fused pose keeps vision corrections intact while still integrating wheel/gyro motion.
    Transform2d delta = new Transform2d(odometryPose, newOdometryPose);
    odometryPose = newOdometryPose;
    estimatedPose = estimatedPose.plus(delta);
    odometryPoseBuffer.addSample(timestamp, odometryPose);
    publish();
  }

  /**
   * Fuses one accepted vision pose using rewind/fuse/forward. The caller ({@link
   * frc.robot.subsystems.vision.Limelight}) is responsible for gating out bad frames and for
   * shaping {@code visionStdDevs}; this method only does the time-aligned blend.
   *
   * @param visionPose robot pose measured by vision, blue-alliance-origin
   * @param timestamp measurement time in the {@code Utils.getCurrentTimeSeconds()} epoch
   * @param visionStdDevs measurement std-devs [x, y, theta] (meters, radians)
   */
  public void addVisionObservation(
      Pose2d visionPose, double timestamp, Matrix<N3, N1> visionStdDevs) {
    // Where was odometry at the frame's timestamp? Empty => the frame is older than our history
    // (or arrived before the first odometry sample); drop it rather than fight the buffer edge.
    Optional<Pose2d> sample = odometryPoseBuffer.getSample(timestamp);
    if (sample.isEmpty() || !haveOdometryBaseline) {
      rejectedCount++;
      publish();
      return;
    }
    Pose2d odometrySample = sample.get();

    // Rewind: express the fused estimate as of the measurement time.
    Transform2d latestToSample = new Transform2d(odometryPose, odometrySample);
    Transform2d sampleToLatest = new Transform2d(odometrySample, odometryPose);
    Pose2d estimateAtSample = estimatedPose.plus(latestToSample);

    // Residual between the rewound estimate and what vision saw.
    Transform2d residual = new Transform2d(estimateAtSample, visionPose);

    // Fuse: steady-state per-axis Kalman gain K = q / (q + sqrt(q * r)). Diagonal Q and R make the
    // full matrix form collapse to this scalar-per-axis expression (same result 6328/254 get).
    double kx = kalmanGain(qStdDevs[0], visionStdDevs.get(0, 0));
    double ky = kalmanGain(qStdDevs[1], visionStdDevs.get(1, 0));
    double kTheta = kalmanGain(qStdDevs[2], visionStdDevs.get(2, 0));

    Transform2d correction =
        new Transform2d(
            kx * residual.getX(),
            ky * residual.getY(),
            new Rotation2d(kTheta * residual.getRotation().getRadians()));

    // Forward: apply the correction at the measurement time, then replay odometry back to now.
    estimatedPose = estimateAtSample.plus(correction).plus(sampleToLatest);

    acceptedCount++;
    lastVisionTimestamp = timestamp;
    lastCorrectionPub.set(Math.hypot(kx * residual.getX(), ky * residual.getY()));
    publish();
  }

  /** The fused best-estimate field pose (odometry + vision). */
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  /** The odometry-only field pose (no vision). Feed this to MegaTag2 to avoid a vision feedback. */
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  /** Latest field-relative velocity, as reported by the drivetrain. */
  public ChassisVelocities getFieldVelocity() {
    return fieldVelocity;
  }

  private static double kalmanGain(double q, double stdDev) {
    if (q == 0.0) {
      return 0.0;
    }
    double r = stdDev * stdDev;
    return q / (q + Math.sqrt(q * r));
  }

  private void publish() {
    estimatedPosePub.set(estimatedPose);
    odometryPosePub.set(odometryPose);
    visionOdometryDeltaPub.set(
        estimatedPose.getTranslation().getDistance(odometryPose.getTranslation()));
    acceptedCountPub.set(acceptedCount);
    rejectedCountPub.set(rejectedCount);
    secondsSinceVisionPub.set(
        lastVisionTimestamp == 0 ? -1.0 : Utils.getCurrentTimeSeconds() - lastVisionTimestamp);
  }
}
