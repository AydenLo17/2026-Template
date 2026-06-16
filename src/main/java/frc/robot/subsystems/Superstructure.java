// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;

/**
 * Superstructure - coordinates the Arm and Flywheel together so the driver gets one button per
 * robot "pose" instead of juggling both mechanisms by hand.
 *
 * <p>This is a plain coordinator (not a subsystem): each method returns a command built from the
 * arm and flywheel commands, so the composed command automatically <i>requires</i> both the arm and
 * the flywheel. {@code Commands.parallel(...)} runs them at the same time.
 */
public class Superstructure {
  private final Arm arm;
  private final Flywheel flywheel;

  public Superstructure(Arm arm, Flywheel flywheel) {
    this.arm = arm;
    this.flywheel = flywheel;
  }

  /** Stow for travel: arm vertical, flywheel stopped. */
  public Command stow() {
    return Commands.parallel(arm.vertical(), flywheel.stop()).withName("Stow");
  }

  /** Ground intake: arm down, flywheel stopped. */
  public Command intake() {
    return Commands.parallel(arm.horizontal(), flywheel.stop()).withName("Intake");
  }

  /** Prepare to score: arm up, flywheel spinning. */
  public Command score() {
    return Commands.parallel(arm.scoring(), flywheel.spinUp()).withName("Score");
  }
}
