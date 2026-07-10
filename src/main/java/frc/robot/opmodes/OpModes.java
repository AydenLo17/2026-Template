// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import frc.robot.Robot;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.GamepieceAssistDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.PreMatchCheck;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.opmode.Utility;

/**
 * All of the robot's OpModes in one place.
 *
 * <p>In the 2027 OpMode model each selectable routine is an annotated class ({@code @Teleop},
 * {@code @Autonomous}, {@code @Utility}); the framework scans this package and lists each by name
 * on the driver station. Rather than scatter those across a file each, they're collected here as
 * {@code public static} nested classes - the scanner discovers nested classes the same way, and
 * having them side by side makes the robot's whole mode surface readable at a glance. Add a routine
 * by adding a nested class; the command it runs should live in {@link Autos} (autonomous) or be
 * composed inline (teleop bindings).
 *
 * <p>Most OpModes here are command-based: they build one {@link Command} and schedule it on enable.
 * That shared lifecycle is factored into {@link CommandOpMode}; a routine subclass just passes the
 * command up. Teleop is different - it sets a default command and button bindings in its
 * constructor (all scoped to the OpMode, so they're torn down automatically on a mode switch).
 */
public final class OpModes {
  private OpModes() {} // container for the nested OpMode classes - never instantiated

  /**
   * Shared base for the command-based OpModes: schedule a command on enable, cancel it on
   * disable/switch. Not annotated and abstract, so the framework's scan skips it.
   */
  private abstract static class CommandOpMode extends PeriodicOpMode {
    private final Command routine;

    protected CommandOpMode(Command routine) {
      this.routine = routine;
    }

    @Override
    public void start() {
      Scheduler.getDefault().schedule(routine);
    }

    @Override
    public void end() {
      Scheduler.getDefault().cancel(routine);
    }
  }

  // ---- Teleop ----------------------------------------------------------------------------------

  /**
   * Driver teleop. The OpMode-model replacement for {@code RobotContainer.configureBindings}: the
   * drivetrain's default (joystick) command and all button bindings are created here and scoped to
   * this OpMode, so the framework removes them automatically on a mode switch. Add another
   * {@code @Teleop} nested class for a second driver layout and it shows up as another DS choice.
   */
  @Teleop(name = "Teleop")
  public static class DriverTeleop extends PeriodicOpMode {
    // Which Limelight to align with, and the AprilTag to align to. TODO: pick the real scoring tag
    // (and flip per alliance) once the game is wired - see the game-info conventions.
    private static final String ALIGN_CAMERA = "limelight";
    private static final int ALIGN_TAG_ID = 1;

    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // top speed
    private final double maxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond); // .75rps

    private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

    public DriverTeleop(Robot robot) {
      final DriveMechanism drivetrain = robot.drivetrain;

      // X is forward and Y is left, per WPILib convention. While intaking, blend a smooth
      // gamepiece assist vector from camera detections into the driver's translation request.
      drivetrain.setDefaultCommand(
          new GamepieceAssistDrive(
              drivetrain,
              driver::getLeftY,
              driver::getLeftX,
              driver::getRightX,
              () -> driver.leftTrigger().getAsBoolean(),
              maxSpeed,
              maxAngularRate));

      // Reset the field-centric heading on left bumper press.
      driver.leftBumper().onTrue(drivetrain.seedFieldCentric());

      // Superstructure presets (arm + flywheel move together), held while the button is down.
      driver.leftTrigger().whileTrue(robot.intake()); // pick up game piece
      driver.rightBumper().whileTrue(robot.score()); // prepare to score
      driver.rightTrigger().whileTrue(robot.stow()); // back to safe travel pose

      // Hold A: vision-only auto-align to the tag standoff.
      driver.a().whileTrue(new DriveToTag(drivetrain, ALIGN_CAMERA, ALIGN_TAG_ID));

      // Hold Y: auto-score prep - raise the arm and spin up the flywheel together.
      driver.y().whileTrue(robot.autoScore()).whileFalse(robot.flywheel.stop());
    }
  }

  // ---- Autonomous routines ---------------------------------------------------------------------

  /**
   * Straight-line odometry auto using {@link DriveToPose}: two legs sequenced with the drivetrain
   * handed off between them. The simple baseline routine.
   */
  @Autonomous(name = "Drive To Pose")
  public static class DriveToPoseAuto extends CommandOpMode {
    public DriveToPoseAuto(Robot robot) {
      super(
          Command.sequence(
                  new DriveToPose(robot.drivetrain, new Pose2d(2.0, 0.0, Rotation2d.kZero)),
                  new DriveToPose(
                      robot.drivetrain, new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(90))))
              .named("DriveToPose Auto"));
    }
  }

  /**
   * Physics-informed follower demo (no path file): the traction-limited {@link
   * frc.robot.commands.AdvancedTrackTrajectory} driving the {@link Autos#exampleTwoPose} routine.
   */
  @Autonomous(name = "Advanced Trajectory")
  public static class AdvancedTrajectoryAuto extends CommandOpMode {
    public AdvancedTrajectoryAuto(Robot robot) {
      super(Autos.exampleTwoPose(robot.drivetrain));
    }
  }

  /**
   * Follows a Choreo path from {@code deploy/choreo/}. Swap {@code "TestPath1"} for your path name
   * (or use {@link Autos#choreoSequence} for several legs). Add one nested class per competition
   * auto.
   */
  @Autonomous(name = "Choreo Path")
  public static class ChoreoAuto extends CommandOpMode {
    public ChoreoAuto(Robot robot) {
      super(Autos.choreoPath(robot.drivetrain, "TestPath1"));
    }
  }

  // ---- Utility ---------------------------------------------------------------------------------

  /**
   * Pre-match readiness check. Select this before a match to verify battery voltage, CAN bus
   * health, vision connectivity, and auto selection. Results display on any dashboard via {@code
   * NT:/MatchReady/*} and are logged to the {@code .wpilog}. Works while the robot is disabled. See
   * {@link PreMatchCheck} for the full list of checks.
   */
  @Utility(name = "Pre-Match Check")
  public static class PreMatchCheckMode extends PeriodicOpMode {
    private final PreMatchCheck check;

    public PreMatchCheckMode(Robot robot) {
      this.check = new PreMatchCheck(robot);
    }

    @Override
    public void periodic() {
      // Runs continuously while the mode is selected, even while disabled, so the dashboard stays
      // live and results update as hardware comes online during pit setup.
      check.runChecks();
    }
  }

  /**
   * Zero the arm encoder at its current position. Use when the robot is at a known mechanical zero
   * (e.g., arm resting on a hard stop). The CANcoder position is set to 0.0; update the position
   * setpoint constants in {@link frc.robot.subsystems.arm.Arm} to match your zero.
   */
  @Utility(name = "Zero Arm")
  public static class ZeroArm extends PeriodicOpMode {
    private final Robot robot;
    private boolean zeroed = false;

    public ZeroArm(Robot robot) {
      this.robot = robot;
    }

    @Override
    public void start() {
      robot.arm.zeroEncoder();
      zeroed = true;
      System.out.println("[ZERO] Arm encoder zeroed at current position");
    }

    @Override
    public void periodic() {
      if (zeroed) {
        System.out.println(
            "[ZERO] Arm zeroed — current angle: "
                + String.format("%.2f", robot.arm.getPosition().baseUnitMagnitude() * 360)
                + "°");
      }
    }
  }

  /** Stow for travel: arm vertical, flywheel stopped. */
  @Utility(name = "Stow")
  public static class Stow extends CommandOpMode {
    public Stow(Robot robot) {
      super(robot.stow());
    }
  }
}
