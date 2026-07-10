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
 * Single source of truth for "where are we on the field" — the robot's pose estimate fusing
 * wheel/gyro odometry with AprilTag vision. This is the team-owned equivalent of the {@code
 * RobotState} class top FRC teams (6328, 254, 2910) build their entire codebase around.
 *
 * <h2>Why a separate estimator?</h2>
 *
 * <p>Phoenix's swerve odometry already fuses vision internally, but it's a black box — you can't
 * see the odometry-only pose, you can't log the correction applied, and you can't tune the gains.
 * This class re-implements the fusion in the open so <b>every knob and every value is visible in
 * the log</b> (under {@code NT:/RobotState/*}) and controllable via {@link
 * frc.robot.Constants.Estimator} and {@link frc.robot.Constants.Vision}. That transparency is what
 * lets you tune vision fusion on a real field with real AprilTag layouts.
 *
 * <h2>The core algorithm: rewind, fuse, forward</h2>
 *
 * <p>A camera frame describing "the robot is at pose X" lands ~50–100 ms <b>after</b> the robot was
 * actually at that pose. If we naively set the current pose to the vision measurement, it fights
 * the odometry and causes jumps. Instead:
 *
 * <ol>
 *   <li><b>Rewind:</b> Keep a short history of odometry poses in a {@link
 *       TimeInterpolatableBuffer}. When a vision frame arrives, look up the pose odometry computed
 *       at the frame's original timestamp.
 *   <li><b>Fuse:</b> Blend the vision pose with the historical odometry pose using a Kalman gain
 *       (weighted by their reported uncertainties).
 *   <li><b>Forward:</b> Re-apply all the odometry motion since that timestamp to bring the
 *       corrected pose back to the present.
 * </ol>
 *
 * <p>Result: the estimate tracks odometry short-term (smooth, accurate) but is drift-corrected by
 * vision long-term (stays on the field).
 *
 * <h2>Trust and Gating</h2>
 *
 * <p><b>Odometry trust:</b> {@link frc.robot.Constants.Estimator} defines how much drift we expect
 * from wheels/gyro. Lower values = more drift assumed = vision pulls harder.
 *
 * <p><b>Vision trust:</b> Each vision measurement reports a 3-element covariance vector (X/Y/θ
 * standard deviations). {@link frc.robot.Constants.Vision} defines hard gates (reject obviously bad
 * frames) and soft inflation factors (reduce trust if tags are far, noisy, ambiguous, etc.). Only
 * measurements that pass the gates are fused.
 *
 * <h2>Disturbance Handling</h2>
 *
 * <p>When the robot hits something or skids, odometry briefly becomes unreliable. This class
 * detects collisions (sudden accel spike) and skids (velocity mismatch), and temporarily scales up
 * odometry variance so vision corrections pull harder. See {@link frc.robot.Constants.Estimator}
 * disturbance thresholds.
 *
 * <h2>Threading Model</h2>
 *
 * <p>All methods run on the main robot loop. {@link DriveMechanism} feeds odometry updates via
 * periodic calls, and {@link frc.robot.subsystems.vision.Limelight} feeds vision via asynchronous
 * NT callbacks. No locking is needed as long as nothing calls in from the 250 Hz Phoenix low-level
 * odometry thread.
 *
 * <h2>Logging</h2>
 *
 * <p>Publishes to <b>{@code NT:/RobotState/*}</b>:
 *
 * <ul>
 *   <li>{@code EstimatedPose} — the fused pose (what the rest of the robot reads)
 *   <li>{@code OdometryPose} — odometry-only pose (no vision). Compare against EstimatedPose to see
 *       how much vision is pulling.
 *   <li>{@code VisionMinusOdometryMeters} — magnitude of the most recent vision correction
 *   <li>{@code AcceptedVisionCount} / {@code RejectedVisionCount} — running counters
 *   <li>{@code SecondsSinceVision} — age of last accepted frame (-1 = never received one)
 *   <li>{@code OdometryVarianceScale} — disturbance flag (should be 1.0 most of the time)
 * </ul>
 *
 * <p>Use AdvantageScope to plot these channels and tune vision gates and fusion gains. See the
 * {@code log-reading} and {@code TUNING} skills.
 *
 * @see DriveMechanism — owns this and feeds odometry updates
 * @see frc.robot.subsystems.vision.Limelight — feeds vision measurements via NT
 * @see frc.robot.Constants.Estimator — odometry variance tuning
 * @see frc.robot.Constants.Vision — vision trust and gating tuning
 * @see frc.robot.TUNING — step-by-step pose estimation tuning procedures
 */
public class RobotState {
  /** Odometry pose history, keyed by timestamp, for latency-compensated vision fusion. */
  private final TimeInterpolatableBuffer<Pose2d> odometryPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(Constants.Estimator.kPoseBufferSizeSeconds);

  /** Odometry trust as variances [x, y, theta] (std-dev squared). Higher => vision pulls harder. */
  private final double[] qStdDevs = new double[3];

  private double odometryVarianceScale = 1.0;

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
  private final DoublePublisher odometryVarianceScalePub =
      table.getDoubleTopic("OdometryVarianceScale").publish();
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
    double qx = qStdDevs[0] * odometryVarianceScale;
    double qy = qStdDevs[1] * odometryVarianceScale;
    double qTheta = qStdDevs[2] * odometryVarianceScale;
    double kx = kalmanGain(qx, visionStdDevs.get(0, 0));
    double ky = kalmanGain(qy, visionStdDevs.get(1, 0));
    double kTheta = kalmanGain(qTheta, visionStdDevs.get(2, 0));

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

  /**
   * Scales odometry variance used by vision fusion. Values greater than 1.0 reduce trust in
   * odometry, so accepted vision pulls the estimate harder.
   */
  public void setOdometryVarianceScale(double scale) {
    odometryVarianceScale = Math.max(1.0, scale);
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
    odometryVarianceScalePub.set(odometryVarianceScale);
    secondsSinceVisionPub.set(
        lastVisionTimestamp == 0 ? -1.0 : Utils.getCurrentTimeSeconds() - lastVisionTimestamp);
  }
}
