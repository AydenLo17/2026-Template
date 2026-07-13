// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.subsystems.DriveMechanism;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.util.Units;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

/** Simulation-only PhotonVision bridge that feeds the existing Limelight NT keys. */
public final class PhotonVisionSim implements AutoCloseable {
  private static final double kLatencyMs = 35.0;
  private static final double kGamepieceDiameterMeters = Units.inchesToMeters(6.0);
  private static final double[][] kGamepieces = {
    {2.50, 1.80},
    {2.50, 4.10},
    {2.50, 6.30},
    {5.00, 2.70},
    {5.00, 5.50},
    {8.27, 4.10}
  };

  private final DriveMechanism drivetrain;
  private final VisionSystemSim visionSim = new VisionSystemSim("PhotonVisionSim");
  private final AprilTagFieldLayout fieldLayout;
  private final List<CameraRuntime> cameras = new ArrayList<>();

  private double heartbeat = 0.0;

  public PhotonVisionSim(DriveMechanism drivetrain) {
    this.drivetrain = drivetrain;
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    visionSim.addAprilTags(fieldLayout);
    addGamepieceTargets();

    for (VisionSimConfig.CameraSpec spec : VisionSimConfig.cameras()) {
      addCamera(spec);
    }
  }

  public void update() {
    Pose2d robotPose = drivetrain.getOdometryPose();
    visionSim.update(robotPose);

    for (CameraRuntime camera : cameras) {
      publishCamera(camera);
    }

    LimelightHelpers.Flush();
  }

  @Override
  public void close() {
    for (CameraRuntime camera : cameras) {
      camera.camera.close();
    }
  }

  /** Simulated camera + pose estimator pair. */
  private record CameraRuntime(
      PhotonCamera camera,
      PhotonCameraSim cameraSim,
      PhotonPoseEstimator poseEstimator,
      VisionSimConfig.CameraSpec spec) {}

  private void addCamera(VisionSimConfig.CameraSpec spec) {
    PhotonCamera camera = new PhotonCamera(spec.name());
    SimCameraProperties properties = new SimCameraProperties();
    properties.setFPS(20);

    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, properties);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    visionSim.addCamera(cameraSim, spec.robotToCamera());

    cameras.add(
        new CameraRuntime(
            camera, cameraSim, new PhotonPoseEstimator(fieldLayout, spec.robotToCamera()), spec));
  }

  private void addGamepieceTargets() {
    TargetModel model = new TargetModel(kGamepieceDiameterMeters);
    for (double[] gamepiece : kGamepieces) {
      visionSim.addVisionTargets(
          new VisionTargetSim(
              new Pose3d(gamepiece[0], gamepiece[1], 0.0, new Rotation3d()), model, 0, 0.95f));
    }
  }

  private void publishCamera(CameraRuntime camera) {
    PhotonCamera photonCamera = camera.camera();
    PhotonPoseEstimator poseEstimator = camera.poseEstimator();
    PipelineMode pipelineMode = getPipelineMode(camera.spec());

    LimelightHelpers.setLimelightNTDouble(
        photonCamera.getName(), "getpipe", getActivePipelineIndex(camera.spec(), pipelineMode));
    LimelightHelpers.getLimelightNTTableEntry(photonCamera.getName(), "getpipetype")
        .setString(pipelineMode == PipelineMode.GAMEPIECE ? "detector" : "apriltag");

    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      publishNoTarget(photonCamera.getName());
      return;
    }

    PhotonPipelineResult latestResult = results.get(results.size() - 1);
    heartbeat += 1.0;
    LimelightHelpers.setLimelightNTDouble(photonCamera.getName(), "hb", heartbeat);

    List<PhotonTrackedTarget> tagTargets = new ArrayList<>();
    List<PhotonTrackedTarget> gamepieceTargets = new ArrayList<>();
    for (PhotonTrackedTarget target : latestResult.getTargets()) {
      if (target.getFiducialId() >= 0) {
        tagTargets.add(target);
      }
      if (target.getDetectedObjectClassID() >= 0) {
        gamepieceTargets.add(target);
      }
    }

    if (pipelineMode == PipelineMode.GAMEPIECE) {
      clearAprilTagState(photonCamera.getName());
      publishGamepieceState(photonCamera.getName(), gamepieceTargets);
    } else {
      clearGamepieceState(photonCamera.getName());
      publishAprilTagState(photonCamera.getName(), latestResult, tagTargets, poseEstimator);
    }
  }

  private void publishNoTarget(String name) {
    LimelightHelpers.setLimelightNTDouble(name, "tv", 0.0);
    LimelightHelpers.setLimelightNTDouble(name, "tid", -1.0);
    clearAprilTagState(name);
    clearGamepieceState(name);
    LimelightHelpers.setLimelightNTDouble(name, "tx", 0.0);
    LimelightHelpers.setLimelightNTDouble(name, "ty", 0.0);
    LimelightHelpers.setLimelightNTDouble(name, "txnc", 0.0);
    LimelightHelpers.setLimelightNTDouble(name, "tync", 0.0);
    LimelightHelpers.setLimelightNTDouble(name, "ta", 0.0);
    LimelightHelpers.setLimelightNTDouble(name, "tl", 0.0);
    LimelightHelpers.setLimelightNTDouble(name, "cl", 0.0);
    LimelightHelpers.setLimelightNTDouble(name, "tv", 0.0);
  }

  private void clearAprilTagState(String name) {
    LimelightHelpers.setLimelightNTDoubleArray(name, "rawfiducials", new double[0]);
    LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_wpiblue", new double[0]);
    LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_orb_wpiblue", new double[0]);
    LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_targetspace", new double[0]);
    LimelightHelpers.setLimelightNTDoubleArray(name, "camerapose_targetspace", new double[0]);
    LimelightHelpers.setLimelightNTDoubleArray(name, "targetpose_cameraspace", new double[0]);
    LimelightHelpers.setLimelightNTDoubleArray(name, "targetpose_robotspace", new double[0]);
  }

  private void clearGamepieceState(String name) {
    LimelightHelpers.setLimelightNTDoubleArray(name, "rawdetections", new double[0]);
  }

  private void publishAprilTagState(
      String name,
      PhotonPipelineResult result,
      List<PhotonTrackedTarget> tagTargets,
      PhotonPoseEstimator poseEstimator) {
    if (tagTargets.isEmpty()) {
      LimelightHelpers.setLimelightNTDouble(name, "tv", 0.0);
      LimelightHelpers.setLimelightNTDouble(name, "tid", -1.0);
      LimelightHelpers.setLimelightNTDoubleArray(name, "rawfiducials", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_wpiblue", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_orb_wpiblue", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_targetspace", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "camerapose_targetspace", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "targetpose_cameraspace", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "targetpose_robotspace", new double[0]);
      return;
    }

    PhotonTrackedTarget best = tagTargets.getFirst();
    for (int i = 1; i < tagTargets.size(); i++) {
      PhotonTrackedTarget candidate = tagTargets.get(i);
      if (candidate.getArea() > best.getArea()) {
        best = candidate;
      }
    }

    LimelightHelpers.setLimelightNTDouble(name, "tv", 1.0);
    LimelightHelpers.setLimelightNTDouble(name, "tid", best.getFiducialId());
    LimelightHelpers.setLimelightNTDouble(name, "tx", best.getYaw());
    LimelightHelpers.setLimelightNTDouble(name, "ty", best.getPitch());
    LimelightHelpers.setLimelightNTDouble(name, "txnc", best.getYaw());
    LimelightHelpers.setLimelightNTDouble(name, "tync", best.getPitch());
    LimelightHelpers.setLimelightNTDouble(name, "ta", best.getArea());
    LimelightHelpers.setLimelightNTDouble(name, "tl", kLatencyMs);
    LimelightHelpers.setLimelightNTDouble(name, "cl", kLatencyMs);

    Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
    if (estimatedPose.isEmpty()) {
      estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
    }

    if (estimatedPose.isEmpty()) {
      LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_wpiblue", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_orb_wpiblue", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_targetspace", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "camerapose_targetspace", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "targetpose_cameraspace", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "targetpose_robotspace", new double[0]);
      LimelightHelpers.setLimelightNTDoubleArray(name, "rawfiducials", toRawFiducials(tagTargets));
      return;
    }

    Pose3d estimatedRobotPose = estimatedPose.get().estimatedPose;
    double[] poseArray = toPoseArray(estimatedRobotPose);
    LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_wpiblue", poseArray);
    LimelightHelpers.setLimelightNTDoubleArray(name, "botpose_orb_wpiblue", poseArray);

    Pose3d tagPose = fieldLayout.getTagPose(best.getFiducialId()).orElse(new Pose3d());
    Pose3d robotInTag = estimatedRobotPose.relativeTo(tagPose);
    Pose3d tagInRobot = tagPose.relativeTo(estimatedRobotPose);
    LimelightHelpers.setLimelightNTDoubleArray(
        name, "botpose_targetspace", toPoseArray(robotInTag));
    LimelightHelpers.setLimelightNTDoubleArray(
        name, "camerapose_targetspace", toPoseArray(robotInTag));
    LimelightHelpers.setLimelightNTDoubleArray(
        name, "targetpose_cameraspace", toPoseArray(tagInRobot));
    LimelightHelpers.setLimelightNTDoubleArray(
        name, "targetpose_robotspace", toPoseArray(tagInRobot));
    LimelightHelpers.setLimelightNTDoubleArray(name, "rawfiducials", toRawFiducials(tagTargets));
  }

  private void publishGamepieceState(String name, List<PhotonTrackedTarget> detections) {
    if (detections.isEmpty()) {
      LimelightHelpers.setLimelightNTDoubleArray(name, "rawdetections", new double[0]);
      return;
    }

    PhotonTrackedTarget best = detections.getFirst();
    for (int i = 1; i < detections.size(); i++) {
      PhotonTrackedTarget candidate = detections.get(i);
      if (candidate.getArea() > best.getArea()) {
        best = candidate;
      }
    }

    LimelightHelpers.setLimelightNTDouble(name, "tv", 1.0);
    LimelightHelpers.setLimelightNTDouble(name, "tx", best.getYaw());
    LimelightHelpers.setLimelightNTDouble(name, "ty", best.getPitch());
    LimelightHelpers.setLimelightNTDouble(name, "txnc", best.getYaw());
    LimelightHelpers.setLimelightNTDouble(name, "tync", best.getPitch());
    LimelightHelpers.setLimelightNTDouble(name, "ta", best.getArea());
    LimelightHelpers.setLimelightNTDouble(name, "tid", -1.0);

    double[] rawDetections = new double[detections.size() * 12];
    for (int i = 0; i < detections.size(); i++) {
      PhotonTrackedTarget detection = detections.get(i);
      int base = i * 12;
      rawDetections[base] = detection.getDetectedObjectClassID();
      rawDetections[base + 1] = detection.getYaw();
      rawDetections[base + 2] = detection.getPitch();
      rawDetections[base + 3] = detection.getArea();
      rawDetections[base + 4] = 0.0;
      rawDetections[base + 5] = 0.0;
      rawDetections[base + 6] = 0.0;
      rawDetections[base + 7] = 0.0;
      rawDetections[base + 8] = 0.0;
      rawDetections[base + 9] = 0.0;
      rawDetections[base + 10] = 0.0;
      rawDetections[base + 11] = 0.0;
    }
    LimelightHelpers.setLimelightNTDoubleArray(name, "rawdetections", rawDetections);
  }

  private PipelineMode getPipelineMode(VisionSimConfig.CameraSpec spec) {
    double requestedPipeline = LimelightHelpers.getLimelightNTDouble(spec.name(), "pipeline");
    if ((int) requestedPipeline == spec.gamepiecePipelineIndex()) {
      return PipelineMode.GAMEPIECE;
    }
    return PipelineMode.APRILTAG;
  }

  private int getActivePipelineIndex(VisionSimConfig.CameraSpec spec, PipelineMode pipelineMode) {
    return pipelineMode == PipelineMode.GAMEPIECE
        ? spec.gamepiecePipelineIndex()
        : spec.aprilTagPipelineIndex();
  }

  private enum PipelineMode {
    APRILTAG,
    GAMEPIECE
  }

  private static double[] toPoseArray(Pose3d pose) {
    return new double[] {
      pose.getTranslation().getX(),
      pose.getTranslation().getY(),
      pose.getTranslation().getZ(),
      Units.radiansToDegrees(pose.getRotation().getX()),
      Units.radiansToDegrees(pose.getRotation().getY()),
      Units.radiansToDegrees(pose.getRotation().getZ())
    };
  }

  private static double[] toRawFiducials(List<PhotonTrackedTarget> targets) {
    double[] raw = new double[targets.size() * 7];
    for (int i = 0; i < targets.size(); i++) {
      PhotonTrackedTarget target = targets.get(i);
      int base = i * 7;
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      raw[base] = target.getFiducialId();
      raw[base + 1] = target.getYaw();
      raw[base + 2] = target.getPitch();
      raw[base + 3] = target.getArea();
      raw[base + 4] = distance;
      raw[base + 5] = distance;
      raw[base + 6] = target.getPoseAmbiguity();
    }
    return raw;
  }
}
