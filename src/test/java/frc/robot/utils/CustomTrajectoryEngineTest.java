package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.utils.CustomTrajectoryEngine.Sample;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

class CustomTrajectoryEngineTest {

  @Test
  void generateSamplesStartAndEndOfProfile() {
    CustomTrajectoryEngine engine =
        new CustomTrajectoryEngine(
            new TrapezoidProfile.Constraints(1.0, 1.0),
            new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    Pose2d goal = new Pose2d(1.0, 0.0, Rotation2d.kZero);

    engine.generate(start, new ChassisVelocities(0.0, 0.0, 0.0), goal, 0.0);

    assertEquals(start, engine.sample(0.0).targetPose);
    assertEquals(0.0, engine.sample(0.0).targetVx, 1.0e-9);
    assertEquals(0.0, engine.sample(0.0).targetVy, 1.0e-9);

    double totalTime = engine.totalTime();
    Sample endState = toSample(engine.sample(totalTime));
    assertEquals(goal.getX(), endState.pose().getX(), 1.0e-6);
    assertEquals(goal.getY(), endState.pose().getY(), 1.0e-6);
    assertEquals(
        goal.getRotation().getRadians(), endState.pose().getRotation().getRadians(), 1.0e-6);
    assertEquals(0.0, endState.vx(), 1.0e-6);
    assertEquals(0.0, endState.vy(), 1.0e-6);
    assertTrue(engine.isFinished(totalTime));
    assertFalse(engine.isFinished(Math.max(0.0, totalTime - 0.01)));
  }

  @Test
  void loadSamplesInterpolatesBetweenWaypoints() {
    CustomTrajectoryEngine engine = new CustomTrajectoryEngine();
    Sample[] path = {
      new Sample(0.0, new Pose2d(0.0, 0.0, Rotation2d.kZero), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      new Sample(2.0, new Pose2d(2.0, 2.0, Rotation2d.kZero), 2.0, 2.0, 1.0, 4.0, 4.0, 2.0)
    };

    engine.loadSamples(path, 5.0);

    assertEquals(2.0, engine.totalTime(), 1.0e-9);
    assertEquals(path[1].pose(), engine.getGoal());

    Sample halfway = toSample(engine.sample(6.0));
    assertEquals(1.0, halfway.pose().getX(), 1.0e-9);
    assertEquals(1.0, halfway.pose().getY(), 1.0e-9);
    assertEquals(0.0, halfway.pose().getRotation().getRadians(), 1.0e-9);
    assertEquals(1.0, halfway.vx(), 1.0e-9);
    assertEquals(1.0, halfway.vy(), 1.0e-9);
    assertEquals(0.5, halfway.omega(), 1.0e-9);
    assertEquals(2.0, halfway.ax(), 1.0e-9);
    assertEquals(2.0, halfway.ay(), 1.0e-9);
    assertEquals(1.0, halfway.alpha(), 1.0e-9);

    Sample clampedStart = toSample(engine.sample(5.0));
    assertEquals(path[0].pose(), clampedStart.pose());

    Sample clampedEnd = toSample(engine.sample(7.0));
    assertEquals(path[1].pose(), clampedEnd.pose());
    assertTrue(engine.isFinished(7.0));
  }

  private static Sample toSample(CustomTrajectoryEngine.TrajectoryState state) {
    return new Sample(
        0.0,
        state.targetPose,
        state.targetVx,
        state.targetVy,
        state.targetOmega,
        state.ax,
        state.ay,
        state.alpha);
  }
}
