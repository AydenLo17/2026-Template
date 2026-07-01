// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.utils.CustomTrajectoryEngine.Sample;
import io.avaje.json.mapper.JsonMapper;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Filesystem;

/**
 * Reads <a href="https://choreo.autos">Choreo</a> {@code .traj} files and converts them into the
 * plain {@link Sample} array the {@link CustomTrajectoryEngine} plays back.
 *
 * <p><b>Where the files go.</b> Put your {@code .traj} files in {@code src/main/deploy/choreo/}.
 * The build copies {@code src/main/deploy} to the robot (and to the sim's deploy directory), and
 * {@link Filesystem#getDeployDirectory()} resolves to it in both places - so {@code load("MyPath")}
 * works identically in sim and on the SystemCore.
 *
 * <p><b>What it parses.</b> A Choreo swerve {@code .traj} is JSON shaped like
 *
 * <pre>{@code
 * { "name": "...", "version": 1,
 *   "trajectory": {
 *     "sampleType": "Swerve",
 *     "samples": [
 *       { "t":0.0, "x":0.0, "y":0.0, "heading":0.0,
 *         "vx":0.0, "vy":0.0, "omega":0.0,
 *         "ax":0.0, "ay":0.0, "alpha":0.0, "fx":[...], "fy":[...] },
 *       ... ] } }
 * }</pre>
 *
 * We read {@code trajectory.samples} (the per-module forces {@code fx}/{@code fy} are ignored - the
 * engine works in chassis space). Positions are blue-alliance-origin, matching odometry and the
 * engine, so a parsed path can be fed straight to the follower.
 *
 * <p>Parsing uses Avaje's lightweight {@link JsonMapper} (already on the WPILib 2027 classpath), so
 * there's no extra dependency and no code generation - the JSON becomes plain {@code Map}/{@code
 * List} which we walk by hand.
 */
public final class ChoreoTrajectory {
  private ChoreoTrajectory() {} // utility class - never instantiated

  /** Subfolder of the deploy directory where {@code .traj} files live. */
  public static final String CHOREO_DIR = "choreo";

  private static final String EXTENSION = ".traj";

  // No-codegen JSON reader: parses a document into nested Map/List/Number/String/Boolean.
  private static final JsonMapper MAPPER = JsonMapper.builder().build();

  /**
   * Loads a Choreo path from the deploy directory by name.
   *
   * @param name the trajectory name, with or without the {@code .traj} extension (e.g. {@code
   *     "TestPath1"} or {@code "TestPath1.traj"})
   * @return the ordered samples ready for {@link CustomTrajectoryEngine#loadSamples}
   * @throws UncheckedIOException if the file can't be read
   * @throws IllegalArgumentException if the JSON has no swerve samples
   */
  public static Sample[] load(String name) {
    String fileName = name.endsWith(EXTENSION) ? name : name + EXTENSION;
    Path path = Filesystem.getDeployDirectory().toPath().resolve(CHOREO_DIR).resolve(fileName);
    return loadFile(path);
  }

  /**
   * Loads a Choreo path from an explicit file path.
   *
   * @param file the {@code .traj} file
   * @return the ordered samples ready for {@link CustomTrajectoryEngine#loadSamples}
   * @throws UncheckedIOException if the file can't be read
   * @throws IllegalArgumentException if the JSON has no swerve samples
   */
  public static Sample[] loadFile(Path file) {
    final String json;
    try {
      json = Files.readString(file);
    } catch (IOException e) {
      throw new UncheckedIOException("Could not read Choreo trajectory: " + file, e);
    }
    return parse(json);
  }

  /**
   * Parses a Choreo {@code .traj} document (already read into a string) into samples. Exposed for
   * tests and for callers that fetch the JSON some other way.
   *
   * @param json the full {@code .traj} file contents
   * @return the ordered samples
   * @throws IllegalArgumentException if the JSON has no swerve samples
   */
  public static Sample[] parse(String json) {
    Map<String, Object> root = MAPPER.fromJsonObject(json);

    // Current schema nests the samples under "trajectory"; fall back to a top-level "samples"
    // array for tolerance against older/flattened exports.
    List<?> rawSamples = null;
    if (root.get("trajectory") instanceof Map<?, ?> trajectory
        && trajectory.get("samples") instanceof List<?> nested) {
      rawSamples = nested;
    } else if (root.get("samples") instanceof List<?> flat) {
      rawSamples = flat;
    }

    if (rawSamples == null || rawSamples.isEmpty()) {
      throw new IllegalArgumentException(
          "Choreo trajectory has no samples (expected trajectory.samples[])");
    }

    List<Sample> out = new ArrayList<>(rawSamples.size());
    for (Object element : rawSamples) {
      if (element instanceof Map<?, ?> raw) {
        out.add(toSample(raw));
      }
    }
    return out.toArray(new Sample[0]);
  }

  /** Converts one Choreo swerve sample object into a {@link Sample}. */
  private static Sample toSample(Map<?, ?> s) {
    double t = num(s, "t"); // schema key is "t"; some tools emit "timestamp"
    if (s.get("t") == null && s.get("timestamp") != null) {
      t = num(s, "timestamp");
    }
    Pose2d pose = new Pose2d(num(s, "x"), num(s, "y"), Rotation2d.fromRadians(num(s, "heading")));
    return new Sample(
        t,
        pose,
        num(s, "vx"),
        num(s, "vy"),
        num(s, "omega"),
        num(s, "ax"),
        num(s, "ay"),
        num(s, "alpha"));
  }

  /** Reads a numeric field as a double, defaulting to 0 when missing or non-numeric. */
  private static double num(Map<?, ?> map, String key) {
    return map.get(key) instanceof Number n ? n.doubleValue() : 0.0;
  }
}
