// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;

public class RobotContainer {
    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // 10% stick deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // open-loop drive motors

    private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

    public final DriveMechanism drivetrain = new DriveMechanism();

    /* Example mechanisms coordinated by the Superstructure */
    public final Arm arm = new Arm();
    public final Flywheel flywheel = new Flywheel();
    private final Superstructure superstructure = new Superstructure(arm, flywheel);

    /* Autonomous selector (no PathPlanner - add real routines here later) */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        autoChooser.setDefaultOption("Do Nothing", Command.noRequirements(coroutine -> {}).named("Do Nothing"));
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * maxAngularRate) // CCW with negative X (left)
            )
        );

        // Hold the configured neutral mode (e.g. brake) while the robot is disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle)
        );

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.seedFieldCentric());

        // Superstructure presets (arm + flywheel move together).
        driver.leftTrigger().onTrue(superstructure.intake());   // pick up game piece
        driver.rightBumper().onTrue(superstructure.score());    // prepare to score
        driver.rightTrigger().onTrue(superstructure.stow());    // back to safe travel pose
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
