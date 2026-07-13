// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Simulation-only camera definitions.
 *
 * <p>Keep these separate from {@code Constants}: the real robot uses the constants for intake and
 * estimator behavior, while this file only exists to define what the simulator should pretend is
 * mounted on the robot.
 *
 * <p>Adding a camera should be a one-line edit to the {@link #cameras()} list.
 */
public final class VisionSimConfig {
  private VisionSimConfig() {}

  /** Simple camera descriptor for PhotonVision sim. */
  public record CameraSpec(
      String name,
      Transform3d robotToCamera,
      int aprilTagPipelineIndex,
      int gamepiecePipelineIndex) {}

  /**
   * Camera poses for simulation.
   *
   * <p>The first camera is the intake/gamepiece view. The second is the AprilTag/pose-estimation
   * view. Add more entries here as your sim model grows.
   */
  public static List<CameraSpec> cameras() {
    return List.of(
        new CameraSpec(
            "limelight-front",
            new Transform3d(
                new Translation3d(0.30, 0.00, 0.56),
                new Rotation3d(0.0, Math.toRadians(-32.0), 0.0)),
            0,
            1),
        new CameraSpec(
            "limelight-rear",
            new Transform3d(
                new Translation3d(-0.22, 0.24, 0.74),
                new Rotation3d(0.0, Math.toRadians(16.0), Math.toRadians(172.0))),
            0,
            1));
  }
}
