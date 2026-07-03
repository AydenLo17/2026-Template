// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * The superstructure run as a {@link StateMachine} - the state-machines lesson at <a
 * href="https://frc5712.com/state-based">frc5712.com/state-based</a>, as real code.
 *
 * <p>Compare with {@link TeleopOpMode}: there, each button <i>holds</i> a superstructure preset
 * (whileTrue). Here the robot is always in exactly one named state - stowed, pickup, or scoring -
 * and buttons/sensors <i>move it between states</i>. The machine cancels the old state's command
 * and starts the new one for you; illegal jumps simply don't exist because no transition was
 * declared for them.
 *
 * <p>Building one takes four steps (numbered below): construct, add states, pick the initial state,
 * wire transitions. {@code setInitialState} is enforced at build time - forget it and the build
 * fails, not the match. This demo binds no drive controls; it is a superstructure showcase. Select
 * "Teleop" on the driver station for the full driving layout.
 */
@Teleop(name = "StateMachine Demo")
public class StateMachineTeleop extends PeriodicOpMode {
  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);
  private final Command machine;

  public StateMachineTeleop(Robot robot) {
    // 1. Construct - the name is required and shows up in telemetry.
    StateMachine sm = new StateMachine("Superstructure");

    // 2. Add states. Each state owns one command; these presets hold their pose until the machine
    //    cancels them on a transition.
    State stowed = sm.addState(robot.superstructure.stow());
    State pickup = sm.addState(robot.superstructure.intake());
    State scoring = sm.addState(robot.superstructure.score());

    // 3. Every machine needs a starting state.
    sm.setInitialState(stowed);

    // 4. Wire transitions. Each condition is checked every scheduler tick while its state is
    //    active, and fires on the rising edge (false -> true).
    stowed.switchTo(pickup).when(driver.leftTrigger()); // driver asks to intake
    pickup.switchTo(scoring).when(robot.arm::isAtTarget); // arm reached the ground - on a real
    // robot this would be a game-piece sensor ("we have a piece"), not the arm angle
    scoring.switchTo(stowed).when(driver.rightTrigger()); // shot taken - pack up

    // Any-state interrupt: B means "get safe now", no matter which state is active.
    // switchFromAny() with no args applies to every state added so far, so declare it last.
    sm.switchFromAny().to(stowed).when(driver.b());

    machine = sm; // a StateMachine is just a Command - schedule it like any other
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(machine);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(machine);
  }
}
