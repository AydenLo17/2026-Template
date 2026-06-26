// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

/**
 * Command-based wrapper around {@link CommandSwerveDrivetrain}. The drivetrain already extends the
 * Tuner-generated class, so it can't also be a {@code Mechanism} (which is a class in Commands v3);
 * this owns the drivetrain instead and exposes the drive commands.
 */
public class DriveMechanism extends Mechanism {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public DriveMechanism() {
    super("Drivetrain");
    // The drivetrain's perspective update used to live in periodic(); run it every loop.
    Scheduler.getDefault().addPeriodic(drivetrain::applyOperatorPerspective);
  }

  /** Returns a command that continuously applies the supplied control request to the drivetrain. */
  public Command applyRequest(Supplier<SwerveRequest> request) {
    return runRepeatedly(() -> drivetrain.setControl(request.get())).named("applyRequest");
  }

  /** Resets the field-centric heading so "forward" matches the driver's current facing. */
  public Command seedFieldCentric() {
    return run(coroutine -> {
          drivetrain.seedFieldCentric();
        })
        .named("seedFieldCentric");
  }

  /**
   * Applies a swerve control request to the underlying drivetrain. Exposed so a command that
   * already requires this mechanism (e.g. {@code DriveToTag}) can drive it without needing direct
   * access to the Phoenix swerve object.
   */
  public void setControl(SwerveRequest request) {
    drivetrain.setControl(request);
  }
}
