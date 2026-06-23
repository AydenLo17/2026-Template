// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.BooleanSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

/**
 * A {@link Mechanism} with the common command factories built in, so subsystems read like they did
 * with v2's {@code Subsystem}. Instead of building through the staged v3 builder
 * ({@code run(...).named(...)}), a subsystem that extends this can just write
 * {@code runOnce("name", action)}.
 *
 * <p>These mirror the v2 {@code Subsystem} convenience factories ({@code runOnce}, {@code run},
 * {@code startEnd}, {@code runEnd}, {@code startRun}) plus {@code runUntil}/{@code idleUntil}.
 * Extend this for your own subsystems:
 *
 * <pre>{@code
 * public class Arm extends AdvancedMechanism {
 *   public Command vertical() {
 *     return runOnce("vertical", () -> setPosition(VERTICAL_POSITION));
 *   }
 * }
 * }</pre>
 *
 * <p><b>Cleanup note (important in v3):</b> when a command is interrupted, the scheduler drops its
 * coroutine without resuming it, so a {@code try/finally} in the body will <i>not</i> run on
 * interruption. The "end" actions below are wired through the builder's {@code whenCanceled(...)}
 * hook, which is the correct place for interrupt cleanup.
 */
public abstract class AdvancedMechanism extends Mechanism {

  /** Creates a mechanism named after its class, registered with the default scheduler. */
  protected AdvancedMechanism() {
    super();
  }

  /**
   * Creates a mechanism with an explicit name, registered with the default scheduler.
   *
   * @param name The mechanism name.
   */
  protected AdvancedMechanism(String name) {
    super(name);
  }

  /**
   * Creates a mechanism with an explicit name and scheduler.
   *
   * @param name The mechanism name.
   * @param scheduler The scheduler to register with.
   */
  protected AdvancedMechanism(String name, Scheduler scheduler) {
    super(name, scheduler);
  }

  /**
   * Runs {@code action} a single time, then finishes. (v2: {@code runOnce})
   *
   * @param name The command name.
   * @param action What to do once.
   * @return The built command.
   */
  protected Command runOnce(String name, Runnable action) {
    return run(coroutine -> action.run()).named(name);
  }

  /**
   * Runs {@code action} every loop until interrupted. (v2: {@code run})
   *
   * @param name The command name.
   * @param action What to do every loop.
   * @return The built command.
   */
  protected Command run(String name, Runnable action) {
    return runRepeatedly(action).named(name);
  }

  /**
   * Runs {@code start} once, holds the mechanism, then runs {@code end} when interrupted. (v2:
   * {@code startEnd})
   *
   * @param name The command name.
   * @param start What to do once at the start.
   * @param end What to do when interrupted.
   * @return The built command.
   */
  protected Command startEnd(String name, Runnable start, Runnable end) {
    return run(
            coroutine -> {
              start.run();
              coroutine.park(); // own the mechanism, do nothing, until something interrupts us
            })
        .whenCanceled(end)
        .named(name);
  }

  /**
   * Runs {@code action} every loop, then runs {@code end} when interrupted. (v2: {@code runEnd})
   *
   * @param name The command name.
   * @param action What to do every loop.
   * @param end What to do when interrupted.
   * @return The built command.
   */
  protected Command runEnd(String name, Runnable action, Runnable end) {
    return runRepeatedly(action).whenCanceled(end).named(name);
  }

  /**
   * Runs {@code start} once, then runs {@code action} every loop until interrupted. (v2: {@code
   * startRun})
   *
   * @param name The command name.
   * @param start What to do once at the start.
   * @param action What to do every loop after that.
   * @return The built command.
   */
  protected Command startRun(String name, Runnable start, Runnable action) {
    return run(
            coroutine -> {
              start.run();
              while (true) {
                action.run();
                coroutine.yield();
              }
            })
        .named(name);
  }

  /**
   * Runs {@code action} every loop until {@code done} becomes true, then finishes.
   *
   * @param name The command name.
   * @param action What to do every loop.
   * @param done When to stop.
   * @return The built command.
   */
  protected Command runUntil(String name, Runnable action, BooleanSupplier done) {
    return runRepeatedly(action).until(done).named(name);
  }

  /**
   * Owns the mechanism doing nothing until {@code condition} becomes true, then finishes. Like
   * {@link Mechanism#idleFor} but waits on a condition instead of a duration.
   *
   * @param name The command name.
   * @param condition When to stop idling.
   * @return The built command.
   */
  protected Command idleUntil(String name, BooleanSupplier condition) {
    return run(Coroutine::park).until(condition).named(name);
  }
}
