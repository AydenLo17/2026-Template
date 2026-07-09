package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.utils.CustomTrajectoryEngine.Sample;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ChoreoTrajectoryTest {

  @TempDir Path tempDir;

  @Test
  void parseReadsTrajectorySamplesFromNestedSchema() {
    String json =
        """
        {
          "trajectory": {
            "samples": [
              {"t": 0.0, "x": 1.0, "y": 2.0, "heading": 0.5, "vx": 3.0, "vy": 4.0, "omega": 5.0, "ax": 6.0, "ay": 7.0, "alpha": 8.0},
              {"t": 1.0, "x": 9.0, "y": 8.0, "heading": 1.5, "vx": 7.0, "vy": 6.0, "omega": 5.0, "ax": 4.0, "ay": 3.0, "alpha": 2.0}
            ]
          }
        }
        """;

    Sample[] samples = ChoreoTrajectory.parse(json);

    assertEquals(2, samples.length);
    assertEquals(0.0, samples[0].timeSeconds(), 1.0e-9);
    assertEquals(1.0, samples[0].pose().getX(), 1.0e-9);
    assertEquals(2.0, samples[0].pose().getY(), 1.0e-9);
    assertEquals(0.5, samples[0].pose().getRotation().getRadians(), 1.0e-9);
    assertEquals(6.0, samples[1].vy(), 1.0e-9);
    assertEquals(2.0, samples[1].alpha(), 1.0e-9);
  }

  @Test
  void parseAcceptsFlattenedSampleSchema() {
    String json =
        """
        {
          "samples": [
            {"timestamp": 0.25, "x": 5.0, "y": 6.0, "heading": 1.0, "vx": 0.0, "vy": 1.0, "omega": 2.0, "ax": 3.0, "ay": 4.0, "alpha": 5.0}
          ]
        }
        """;

    Sample[] samples = ChoreoTrajectory.parse(json);

    assertEquals(1, samples.length);
    assertEquals(0.25, samples[0].timeSeconds(), 1.0e-9);
    assertEquals(5.0, samples[0].pose().getX(), 1.0e-9);
    assertEquals(6.0, samples[0].pose().getY(), 1.0e-9);
    assertEquals(1.0, samples[0].pose().getRotation().getRadians(), 1.0e-9);
  }

  @Test
  void loadReadsTrackedDeployTrajectory() {
    Sample[] samples = ChoreoTrajectory.load("TestPath1");

    assertTrue(samples.length > 10);
    assertEquals(0.0, samples[0].timeSeconds(), 1.0e-9);
    assertEquals(4.61024, samples[0].pose().getX(), 1.0e-5);
    assertEquals(0.64439, samples[0].pose().getY(), 1.0e-5);
    assertEquals(2.28794, samples[samples.length - 1].timeSeconds(), 1.0e-5);
    assertEquals(8.23726, samples[samples.length - 1].pose().getX(), 1.0e-5);
    assertEquals(4.05953, samples[samples.length - 1].pose().getY(), 1.0e-5);
  }

  @Test
  void loadFileRejectsMissingTrajectoryData() throws IOException {
    Path file = tempDir.resolve("empty.traj");
    Files.writeString(file, "{\"name\":\"empty\"}");

    assertThrows(IllegalArgumentException.class, () -> ChoreoTrajectory.loadFile(file));
  }
}
