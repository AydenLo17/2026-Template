// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import frc.robot.utils.ClassicCommand;

/**
 * FlywheelCommand - the {@link Flywheel#spinUp()} factory written in the classic
 * {@code initialize} / {@code execute} / {@code isFinished} / {@code end} style instead of a
 * one-line command factory.
 *
 * <p>Spins the flywheel up to shooting speed and finishes once it is holding that speed within
 * tolerance. If something interrupts it before it gets there, the flywheel is stopped; on a
 * natural finish it is left spinning (ready to shoot), matching {@link Flywheel#spinUp()}.
 *
 * <p>This is the verbose counterpart to {@link Flywheel#spinUp()} - reach for {@link ClassicCommand}
 * when you want explicit, stateful steps, and for the one-line factory when that is all you need.
 */
public class FlywheelCommand extends ClassicCommand {
  private final Flywheel flywheel;

  /**
   * Creates a command that spins the flywheel up to shooting speed.
   *
   * @param flywheel the flywheel to spin up (claimed while this command runs)
   */
  public FlywheelCommand(Flywheel flywheel) {
    super("FlywheelSpinUp", flywheel); // name + requirement, like v2 addRequirements(flywheel)
    this.flywheel = flywheel;
  }

  // Start the flywheel spinning toward shooting speed.
  @Override
  protected void initialize() {
    flywheel.spinUpDirect();
  }

  // Motion Magic holds the target speed on the controller, so there is nothing to do each loop.
  @Override
  protected void execute() {}

  // Finish once we are holding shooting speed within tolerance.
  @Override
  protected boolean isFinished() {
    return flywheel.isAtTarget();
  }

  // Only stop if we were interrupted before reaching speed; a natural finish leaves it spinning.
  @Override
  protected void end(boolean interrupted) {
    if (interrupted) {
      flywheel.stopDirect();
    }
  }
}
