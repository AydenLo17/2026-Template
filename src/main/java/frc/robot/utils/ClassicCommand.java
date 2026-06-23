// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Arrays;
import java.util.Set;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

/**
 * The classic four-method command style on top of Commands v3 coroutines.
 *
 * <p>Commands v3 commands are normally written as a single linear coroutine body (see {@link
 * AdvancedMechanism} and {@code mechanism.run(...)}). For something with explicit, stateful steps -
 * or for students coming from the v2 docs - the familiar {@code initialize} / {@code execute} /
 * {@code isFinished} / {@code end} lifecycle can be clearer. Extend this class and override the
 * pieces you need; the instance <i>is</i> a {@link Command}, so it can be scheduled or bound to a
 * trigger directly.
 *
 * <p>The lifecycle matches v2:
 *
 * <ul>
 *   <li>{@link #initialize()} runs once when the command starts.
 *   <li>{@link #execute()} runs every loop while the command is active.
 *   <li>{@link #isFinished()} is checked every loop, right after {@code execute}; return true to
 *       finish.
 *   <li>{@link #end(boolean)} runs once when the command ends - {@code interrupted=false} when
 *       {@code isFinished} returned true, {@code interrupted=true} when another command stole one of
 *       this command's mechanisms.
 * </ul>
 *
 * <p>Under the hood this is just a coroutine: {@code initialize}, then a {@code while} loop that
 * calls {@code execute}, checks {@code isFinished}, and {@link Coroutine#yield() yields} a loop;
 * {@code end(false)} runs after the loop, and {@code end(true)} runs from the {@code onCancel} hook
 * (the scheduler drops the coroutine on interruption, so a {@code finally} would not run).
 *
 * <p>Example:
 *
 * <pre>{@code
 * public class DriveDistance extends ClassicCommand {
 *   private final Drive drive;
 *   private final double meters;
 *
 *   public DriveDistance(Drive drive, double meters) {
 *     super("DriveDistance", drive); // name + requirements, like v2 addRequirements(drive)
 *     this.drive = drive;
 *     this.meters = meters;
 *   }
 *
 *   @Override protected void initialize()       { drive.resetEncoders(); }
 *   @Override protected void execute()          { drive.arcade(0.5, 0); }
 *   @Override protected boolean isFinished()    { return drive.distance() >= meters; }
 *   @Override protected void end(boolean intr)  { drive.stop(); }
 * }
 * }</pre>
 */
public abstract class ClassicCommand implements Command {
  private final String name;
  private final Set<Mechanism> requirements;

  /**
   * Creates a classic-style command.
   *
   * @param name The command name (shows up in telemetry).
   * @param requirements The mechanisms this command owns while it runs.
   */
  protected ClassicCommand(String name, Mechanism... requirements) {
    this.name = name;
    this.requirements = Set.copyOf(Arrays.asList(requirements));
  }

  /** Runs once when the command starts. Override to set up state. */
  protected void initialize() {}

  /** Runs every loop while the command is active. Override to do the work. */
  protected void execute() {}

  /**
   * Checked every loop, right after {@link #execute()}.
   *
   * @return true to finish the command, false to keep running.
   */
  protected boolean isFinished() {
    return false;
  }

  /**
   * Runs once when the command ends. Keep this to single-shot cleanup (for example, stopping a
   * motor); don't loop here.
   *
   * @param interrupted false if {@link #isFinished()} ended the command, true if it was interrupted
   *     by another command claiming one of its mechanisms.
   */
  protected void end(boolean interrupted) {}

  @Override
  public final void run(Coroutine coroutine) {
    initialize();
    while (true) {
      execute();
      if (isFinished()) {
        break;
      }
      coroutine.yield();
    }
    end(false); // natural finish
  }

  @Override
  public final void onCancel() {
    end(true); // interrupted finish (coroutine was dropped; this is the only cleanup hook that runs)
  }

  @Override
  public final String name() {
    return name;
  }

  @Override
  public final Set<Mechanism> requirements() {
    return requirements;
  }
}
