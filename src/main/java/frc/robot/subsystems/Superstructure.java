// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import org.wpilib.command3.Command;

/**
 * Superstructure - coordinates the Arm and Flywheel so the driver gets one button per robot "pose"
 * instead of juggling both mechanisms by hand.
 *
 * <p>Each method returns a command composed of arm and flywheel commands. Because commands inherit
 * their children's requirements, the result requires both subsystems, and {@code
 * Command.parallel(...)} runs them at the same time.
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
}
