// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.DriveMechanism;
import java.util.OptionalDouble;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;

/**
 * Simulation-only NetworkTables bridge between the WPILib robot code and an (optional) external
 * physics engine such as Isaac Sim. This is the WPILib half of the loopback co-simulation contract
 * described in {@code ISAAC_SIM_AUTOMATION.md}; the Python half is {@code isaac_sim_bridge.py}.
 *
 * <p>Call {@link #update()} once per loop from {@link frc.robot.Robot#simulationPeriodic()}. Each
 * loop it:
 *
 * <ol>
 *   <li><b>Publishes</b> the drivetrain's target module velocities (m/s) to NetworkTables so the
 *       external engine can drive the physics robot with them.
 *   <li><b>Reads</b> the simulated sensor feedback the external engine publishes back - the
 *       ground-truth gyro yaw and the intake contact ("did the gripper touch a game piece") state -
 *       and re-publishes them under a {@code Sim/*} namespace so they are visible live and captured
 *       in the WPILOG by {@code DataLogManager}. The cached values are also exposed via {@link
 *       #gyroYawDegrees()} and {@link #intakeContact()} for commands that want the ground truth.
 * </ol>
 *
 * <p>All keys sit under the {@code SmartDashboard} table so they match the topic names the Python
 * bridge and {@link frc.robot.commands.VisionAutoAlign} use. When no external engine is connected
 * the inbound keys are simply absent, so the reads report "unavailable" and the robot's own
 * WPILib/CTRE physics stays authoritative - the whole thing degrades cleanly to a pure-WPILib
 * headless sim.
 */
public final class SimBridge {
  private final DriveMechanism drivetrain;

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");

  // OUT (WPILib -> external engine): commanded module target velocities this loop. Published as a
  // struct array (the idiomatic WPILib swerve surface, the same shape Telemetry logs).
  private final StructArrayPublisher<SwerveModuleVelocity> targetModuleVelocities =
      table
          .getStructArrayTopic("Sim/TargetModuleVelocities", SwerveModuleVelocity.struct)
          .publish();

  // IN (external engine -> WPILib): ground-truth gyro yaw (deg) and intake contact (bool).
  // NaN default on the gyro means "not connected"; the intake contact defaults to false.
  private final DoubleSubscriber gyroYawSub =
      table.getDoubleTopic("SimSensors/GyroHeading").subscribe(Double.NaN);
  private final DoubleSubscriber intakeContactSub =
      table.getDoubleTopic("SimSensors/IntakeContact").subscribe(0.0);

  // Echoes of the inbound feedback, so it lands in the WPILOG and Glass/AdvantageScope.
  private final DoublePublisher gyroYawEcho = table.getDoubleTopic("Sim/GyroYawDeg").publish();
  private final BooleanPublisher intakeContactEcho =
      table.getBooleanTopic("Sim/IntakeContact").publish();

  private double cachedGyroYaw = Double.NaN;
  private boolean cachedIntakeContact = false;

  public SimBridge(DriveMechanism drivetrain) {
    this.drivetrain = drivetrain;
  }

  /** Publishes commanded module speeds and refreshes the cached simulated-sensor feedback. */
  public void update() {
    if (!RobotBase.isSimulation()) {
      return; // Sim-only surface; no-op on the real robot.
    }

    // Publish target module velocities for the external physics engine.
    targetModuleVelocities.set(drivetrain.getModuleTargets());

    // Read simulated sensor feedback from the external engine and log it.
    cachedGyroYaw = gyroYawSub.get();
    cachedIntakeContact = intakeContactSub.get() > 0.5;
    if (!Double.isNaN(cachedGyroYaw)) {
      gyroYawEcho.set(cachedGyroYaw);
    }
    intakeContactEcho.set(cachedIntakeContact);
  }

  /**
   * The most recent ground-truth gyro yaw (degrees) from the external engine, or empty when no
   * engine is connected (so callers fall back to the WPILib/CTRE simulated heading).
   */
  public OptionalDouble gyroYawDegrees() {
    return Double.isNaN(cachedGyroYaw) ? OptionalDouble.empty() : OptionalDouble.of(cachedGyroYaw);
  }

  /** True when the external engine last reported the intake in contact with a game piece. */
  public boolean intakeContact() {
    return cachedIntakeContact;
  }
}
