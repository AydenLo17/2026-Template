// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.utils.ClassicCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Default teleop drive that adds a 1690-style intake assist toward camera-detected gamepieces.
 *
 * <p>Driver input is always primary. While intake is active, this command estimates gamepiece
 * position from Limelight tx/ty using camera geometry, keeps a short "last seen" memory, and blends
 * in a smooth velocity-aware assist vector.
 */
public class GamepieceAssistDrive extends ClassicCommand {
  private final DriveMechanism drivetrain;
  private final DoubleSupplier leftY;
  private final DoubleSupplier leftX;
  private final DoubleSupplier rightX;
  private final BooleanSupplier intakeActive;
  private final double maxSpeed;
  private final double maxAngularRate;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.0)
          .withRotationalDeadband(0.0)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SlewRateLimiter vxLimiter =
      new SlewRateLimiter(Constants.Gamepiece.kTranslationSlewRate);
  private final SlewRateLimiter vyLimiter =
      new SlewRateLimiter(Constants.Gamepiece.kTranslationSlewRate);
  private final SlewRateLimiter omegaLimiter =
      new SlewRateLimiter(Constants.Gamepiece.kRotationSlewRate);

  // private final NetworkTable telemetryTable =
  //     NetworkTableInstance.getDefault().getTable("GamepieceAssistDrive");
  // private final DoublePublisher manualSpeedPub =
  //     telemetryTable.getDoubleTopic("ManualSpeedMps").publish();
  // private final DoublePublisher commandedSpeedPub =
  //     telemetryTable.getDoubleTopic("CommandedSpeedMps").publish();
  // private final DoublePublisher smoothedSpeedPub =
  //     telemetryTable.getDoubleTopic("SmoothedSpeedMps").publish();
  // private final DoublePublisher filteredSpeedPub =
  //     telemetryTable.getDoubleTopic("FilteredSpeedMps").publish();

  private Translation2d lastSeenGamepieceField = null;
  private double lastSeenTimestamp = 0.0;

  public GamepieceAssistDrive(
      DriveMechanism drivetrain,
      DoubleSupplier leftY,
      DoubleSupplier leftX,
      DoubleSupplier rightX,
      BooleanSupplier intakeActive,
      double maxSpeed,
      double maxAngularRate) {
    super("GamepieceAssistDrive", drivetrain);
    this.drivetrain = drivetrain;
    this.leftY = leftY;
    this.leftX = leftX;
    this.rightX = rightX;
    this.intakeActive = intakeActive;
    this.maxSpeed = maxSpeed;
    this.maxAngularRate = maxAngularRate;
  }

  @Override
  protected void initialize() {
    Pose2d pose = drivetrain.getPose();
    ChassisVelocities fieldVelocity = drivetrain.getFieldVelocity();
    vxLimiter.reset(fieldVelocity.vx);
    vyLimiter.reset(fieldVelocity.vy);
    omegaLimiter.reset(fieldVelocity.omega);
    if (pose == null) {
      lastSeenGamepieceField = null;
      lastSeenTimestamp = 0.0;
    }
  }

  @Override
  protected void execute() {
    double now = Utils.getCurrentTimeSeconds();

    // Manual field-centric command from sticks (same sign convention as existing teleop).
    double manualVx = -leftY.getAsDouble() * maxSpeed;
    double manualVy = -leftX.getAsDouble() * maxSpeed;
    double manualOmega = -rightX.getAsDouble() * maxAngularRate;
    // manualSpeedPub.set(Math.hypot(manualVx, manualVy));

    updateGamepieceEstimate(now);

    double commandVx = manualVx;
    double commandVy = manualVy;

    if (intakeActive.getAsBoolean()) {
      Translation2d assist = calculateAssistFieldVelocity();
      commandVx += assist.getX() * Constants.Gamepiece.kAssistBlend;
      commandVy += assist.getY() * Constants.Gamepiece.kAssistBlend;
    }

    // Keep commanded translation within drivetrain limits before smoothing.
    double norm = Math.hypot(commandVx, commandVy);
    if (norm > maxSpeed && norm > 1e-9) {
      double scale = maxSpeed / norm;
      commandVx *= scale;
      commandVy *= scale;
    }
    // commandedSpeedPub.set(Math.hypot(commandVx, commandVy));

    double smoothVx = vxLimiter.calculate(commandVx);
    double smoothVy = vyLimiter.calculate(commandVy);
    double smoothOmega = omegaLimiter.calculate(manualOmega);
    // smoothedSpeedPub.set(Math.hypot(smoothVx, smoothVy));

    // Manual teleop already limits acceleration through the slew-rate limiters above. Running the
    // measured-velocity traction filter here makes stick control feel sluggish because any normal
    // drivetrain tracking lag looks like an "unachievable" acceleration request every loop.
    ChassisVelocities limited = new ChassisVelocities(smoothVx, smoothVy, smoothOmega);
    // filteredSpeedPub.set(Math.hypot(limited.vx, limited.vy));

    drivetrain.setControl(
        driveRequest
            .withVelocityX(limited.vx)
            .withVelocityY(limited.vy)
            .withRotationalRate(limited.omega));
  }

  @Override
  protected boolean isFinished() {
    return false; // default command
  }

  @Override
  protected void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());
  }

  private void updateGamepieceEstimate(double nowSeconds) {
    LimelightHelpers.RawDetection best = getBestDetection();
    if (best != null) {
      Translation2d robotRelative = estimateGamepieceRobotRelative(best.txnc, best.tync);
      Pose2d robotPose = drivetrain.getPose();
      Translation2d fieldRelative =
          robotRelative.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
      lastSeenGamepieceField = fieldRelative;
      lastSeenTimestamp = nowSeconds;
      return;
    }

    if (lastSeenGamepieceField != null
        && nowSeconds - lastSeenTimestamp > Constants.Gamepiece.kTargetMemorySeconds) {
      lastSeenGamepieceField = null;
    }
  }

  private LimelightHelpers.RawDetection getBestDetection() {
    LimelightHelpers.RawDetection[] detections =
        LimelightHelpers.getRawDetections(Constants.Gamepiece.kDetectionCamera);
    LimelightHelpers.RawDetection best = null;
    double bestScore = Double.NEGATIVE_INFINITY;

    for (LimelightHelpers.RawDetection detection : detections) {
      if (Constants.Gamepiece.kDetectionClassId >= 0
          && detection.classId != Constants.Gamepiece.kDetectionClassId) {
        continue;
      }
      if (detection.ta < Constants.Gamepiece.kMinDetectionArea) {
        continue;
      }

      // Favor larger targets near image center for more stable tx/ty-to-range solves.
      double centerPenalty = Math.hypot(detection.txnc, detection.tync);
      double score = detection.ta - 0.20 * centerPenalty;
      if (score > bestScore) {
        bestScore = score;
        best = detection;
      }
    }

    return best;
  }

  private Translation2d estimateGamepieceRobotRelative(double txDegrees, double tyDegrees) {
    double totalPitchDeg = Constants.Gamepiece.kCameraPitchDegrees + tyDegrees;
    double totalPitchRad = Math.toRadians(totalPitchDeg);

    // Avoid exploding range when the ray is nearly parallel to the carpet.
    double tanPitch = Math.tan(totalPitchRad);
    if (Math.abs(tanPitch) < 1e-3) {
      tanPitch = Math.copySign(1e-3, tanPitch == 0.0 ? 1.0 : tanPitch);
    }

    double cameraToPieceDistance =
        (Constants.Gamepiece.kCameraHeightMeters - Constants.Gamepiece.kGamepieceHeightMeters)
            / tanPitch;
    cameraToPieceDistance = Math.max(0.0, cameraToPieceDistance);

    double bearingRad = Math.toRadians(Constants.Gamepiece.kCameraYawDegrees + txDegrees);

    double forward =
        Constants.Gamepiece.kCameraForwardMeters + cameraToPieceDistance * Math.cos(bearingRad);
    double left =
        Constants.Gamepiece.kCameraLeftMeters + cameraToPieceDistance * Math.sin(bearingRad);

    return new Translation2d(forward, left);
  }

  private Translation2d calculateAssistFieldVelocity() {
    if (lastSeenGamepieceField == null) {
      return Translation2d.kZero;
    }

    Pose2d robotPose = drivetrain.getPose();
    Translation2d error = lastSeenGamepieceField.minus(robotPose.getTranslation());
    double distance = error.getNorm();

    if (distance < Constants.Gamepiece.kStopDistanceMeters) {
      return Translation2d.kZero;
    }

    ChassisVelocities velocity = drivetrain.getFieldVelocity();

    // Velocity-aware assist: pursuit term pulls us toward the piece, damping term reduces
    // overshoot/skid by bleeding current field velocity.
    double assistVx =
        Constants.Gamepiece.kPursuitGain * error.getX()
            - Constants.Gamepiece.kVelocityDamping * velocity.vx;
    double assistVy =
        Constants.Gamepiece.kPursuitGain * error.getY()
            - Constants.Gamepiece.kVelocityDamping * velocity.vy;

    double mag = Math.hypot(assistVx, assistVy);
    if (mag > Constants.Gamepiece.kMaxAssistSpeedMetersPerSecond && mag > 1e-9) {
      double scale = Constants.Gamepiece.kMaxAssistSpeedMetersPerSecond / mag;
      assistVx *= scale;
      assistVy *= scale;
    }

    return new Translation2d(assistVx, assistVy);
  }
}
