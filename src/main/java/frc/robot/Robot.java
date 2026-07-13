// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.PhotonVisionSim;
import frc.robot.utils.SimStartup;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.DataLogManager;

/**
 * Owns the robot's shared hardware in one place. With the OpMode framework there is no {@code
 * RobotContainer}: the subsystems live here as public fields, and each OpMode in {@code
 * frc.robot.opmodes} reaches them through the {@link Robot} reference it is constructed with.
 *
 * <p>The framework auto-discovers the {@code @Teleop}/{@code @Autonomous} classes in this package
 * (and subpackages) and handles every mode transition, so this class has no per-mode init/periodic
 * methods - only the always-on scheduler tick. Selecting a different mode on the driver station
 * constructs that OpMode and tears down the previous one (its button bindings are scoped to it and
 * removed automatically).
 */
public class Robot extends OpModeRobot {
  public final DriveMechanism drivetrain = new DriveMechanism();

  /* Example mechanisms. */
  public final Arm arm = new Arm();
  public final Flywheel flywheel = new Flywheel();

  private final PhotonVisionSim photonVisionSim;

  public Robot() {
    // Start on-robot logging. There is no AdvantageKit in this template; the "logging-only" story
    // is DataLogManager - it records every NetworkTables value change (including everything
    // Telemetry publishes under Drivetrain/*) to a .wpilog, plus console output. startDataLog adds
    // the driver-station state and joystick data. Logs go to ./logs in sim and to a USB drive (or
    // /home/systemcore/logs) on the real robot. See the log-reading skill.
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Brake while disabled, in every mode. Created here (before any OpMode is selected) so the
    // binding is global; the opmodes' bindings are scoped to their OpMode and removed on a switch.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle));

    // Vision: wire up every Limelight in one call (names must match each camera's NT name).
    Limelight.registerAll(drivetrain, "limelight-front", "limelight-rear");

    photonVisionSim = new PhotonVisionSim(drivetrain);
  }

  @Override
  public void simulationInit() {
    // Headless auto-enable for agent / CI runs. No-op unless -Dfrc.sim.startMode is set (the
    // simulateJavaAgent Gradle task sets it). See SimStartup and the run-sim skill.
    SimStartup.arm();
  }

  @Override
  public void simulationPeriodic() {
    photonVisionSim.update();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
  }

  /** Stow for travel: arm vertical, flywheel stopped. */
  public Command stow() {
    return Command.parallel(arm.vertical(), flywheel.stop()).named("Stow");
  }

  /** Ground intake: arm down, flywheel stopped. */
  public Command intake() {
    return Command.parallel(arm.horizontal(), flywheel.stop()).named("Intake");
  }

  /** Prepare to score: arm up, flywheel spinning. */
  public Command score() {
    return Command.parallel(arm.scoring(), flywheel.spinUp()).named("Score");
  }

  /**
   * Auto-score prep: raise the arm to its scoring pose and hold shooting speed. Like {@link
   * #score()} but the arm command finishes once it reaches the pose ({@code scoringAndWait});
   * {@code spinUp} runs forever, so the group runs until it is cancelled.
   */
  public Command autoScore() {
    return Command.parallel(arm.scoringAndWait(), flywheel.spinUp()).named("AutoScore");
  }
}
