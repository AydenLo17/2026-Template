// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.Angle;

/**
 * Arm - example rotating mechanism driven by a Phoenix 6 TalonFX + CANcoder.
 *
 * <h2>Pattern</h2>
 *
 * <p>The subsystem owns the motor and encoder, keeps setters private, and exposes only
 * <b>commands</b> (each returns a {@link Command}). Anything that wants to move the arm does it
 * through a command, which prevents two things fighting over the motor via the scheduler.
 *
 * <h2>Hardware</h2>
 *
 * <ul>
 *   <li><b>Motor:</b> TalonFX on CAN 31 (update in constructor if different)
 *   <li><b>Encoder:</b> CANcoder on CAN 32 (update in constructor if different)
 *   <li><b>Gear ratio:</b> 1:1 (update {@link TalonFXConfiguration#Feedback} if different)
 *   <li><b>Control mode:</b> {@link MotionMagicVoltage} position control with gravity feedforward
 *   <li><b>Neutral mode:</b> Coast (arm falls under gravity when not powered)
 * </ul>
 *
 * <h2>Tuning</h2>
 *
 * <p>Before driving the arm on a real robot, you <b>must</b> characterize and tune:
 *
 * <ul>
 *   <li><b>Gravity feedforward ({@code kG}):</b> Measures how much voltage the arm needs to hold
 *       itself at 90° (horizontal). Safe starting point: {@code 0.2} (typical for arm-mass robots).
 *       Too low: arm droops under its own weight. Too high: arm kicks upward at high power.
 *   <li><b>Static friction ({@code kS}):</b> Voltage to overcome friction. Safe start: {@code 0.2}.
 *   <li><b>Proportional gain ({@code kP}):</b> Correction strength; in (V) per degree of error.
 *       Safe start: {@code 80} (strong correction). Lower if jerky, raise if sluggish.
 *   <li><b>Derivative gain ({@code kD}):</b> Damping to prevent overshoot. Safe start: {@code 8.0}.
 *   <li><b>Motion Magic limits:</b> {@code MOTION_MAGIC_CRUISE_VELOCITY} (how fast it moves, rot/s)
 *       and {@code MOTION_MAGIC_ACCELERATION} (how quickly it accelerates, rot/s²). Safe start: 2.0
 *       rot/s and 4.0 rot/s².
 * </ul>
 *
 * <p>Once tuned, update the static final constants at the top of this file. See <a
 * href="../../../../../../DEPLOYMENT_CHECKLIST.md">DEPLOYMENT_CHECKLIST.md</a> for the step-by-step
 * tuning procedure.
 *
 * <h2>Commands</h2>
 *
 * <ul>
 *   <li>{@link #vertical()} — move to stowed position (90°)
 *   <li>{@link #horizontal()} — move to ground intake position (180°)
 *   <li>{@link #scoring()} — move to scoring position (~30°)
 *   <li>{@link #scoringAndWait()} — move to scoring and finish once arrived
 * </ul>
 *
 * @see frc.robot.Robot#arm — owned by the robot
 * @see frc.robot.commands.DriveToPose — uses arm position for part of autonomous routine
 */
public class Arm extends Mechanism {
  // Position setpoints (rotations, 1.0 = full turn).
  private static final double VERTICAL_POSITION = 0.25; // 90°  - stowed / safe transport
  private static final double HORIZONTAL_POSITION = 0.5; // 180° - ground intake
  private static final double SCORING_POSITION = 0.083; // ~30° - scoring

  // How close counts as "at target".
  private static final double POSITION_TOLERANCE_DEGREES = 1.0;

  // PID + feedforward gains.
  // TODO: CRITICAL - tune on the real robot before driving the arm under power.
  // Safe starting values: kG=0.2 (fights gravity), kS=0.2 (overcomes friction),
  //                       kP=160 (correction strength), kD=30 (smoothness).
  // If the arm jerks or moves too fast, make these smaller.
  private static final double kG = 0.0; // NEEDS TUNING - gravity feedforward
  private static final double kS = 0.0; // NEEDS TUNING - static friction feedforward
  private static final double kP = 0.0; // NEEDS TUNING - proportional gain
  private static final double kD = 0.0; // NEEDS TUNING - derivative gain

  // Motion Magic speed limits.
  // TODO: CRITICAL - set how fast the arm can move.
  // Recommended start: cruise=2 rot/s, accel=4 rot/s².
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0; // NEEDS SETTING
  private static final double MOTION_MAGIC_ACCELERATION = 0.0; // NEEDS SETTING

  private final TalonFX motor = new TalonFX(31, TunerConstants.kCANBus);
  private final CANcoder encoder = new CANcoder(32, TunerConstants.kCANBus);

  // Drives the arm to a target angle with a smooth Motion Magic profile.
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  private final Angle tolerance = Degrees.of(POSITION_TOLERANCE_DEGREES);

  public Arm() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // fights gravity automatically

    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
    config.Feedback.withRemoteCANcoder(encoder);

    TalonFXUtil.applyConfigWithRetries(motor, config);
  }

  // The "move and hold" factories use runRepeatedly, which re-sends the Motion Magic request every
  // loop. Phoenix already holds the last request; re-sending just re-asserts it after a reboot.

  /** Move to the vertical (stowed) position. */
  public Command vertical() {
    return runRepeatedly(() -> setPosition(VERTICAL_POSITION)).named("vertical");
  }

  /** Move to the horizontal (ground intake) position. */
  public Command horizontal() {
    return runRepeatedly(() -> setPosition(HORIZONTAL_POSITION)).named("horizontal");
  }

  /** Move to the scoring position. */
  public Command scoring() {
    return runRepeatedly(() -> setPosition(SCORING_POSITION)).named("scoring");
  }

  /**
   * Move to the scoring position and finish once the arm is there. Await this in a sequence (e.g.
   * auto-score). The arm holds its angle after this finishes - the last Motion Magic request stays
   * applied - until another command moves it.
   */
  public Command scoringAndWait() {
    return runRepeatedly(() -> setPosition(SCORING_POSITION))
        .until(this::isAtTarget)
        .named("scoringAndWait");
  }

  /** True when the arm has reached its target angle. */
  public boolean isAtTarget() {
    return getPosition().isNear(getTargetPosition(), tolerance);
  }

  /**
   * True when the arm motor controller is alive on the CAN bus. Returns false if the device has
   * never responded since boot (version == 0), which indicates a wiring or ID problem.
   */
  public boolean isMotorAlive() {
    return motor.getVersion().getValue() > 0;
  }

  /**
   * Zero the arm encoder at the current physical position. Call this when the arm is resting on a
   * known mechanical zero point (e.g., a hard stop). After zeroing, the position setpoints in this
   * file should be measured and updated to match the new zero. See the "Zero Arm" Utility OpMode.
   */
  public void zeroEncoder() {
    encoder.setPosition(0.0);
  }

  /** Current measured arm angle. */
  public Angle getPosition() {
    return encoder.getPosition().getValue();
  }

  /** Angle the arm is currently driving toward. */
  public Angle getTargetPosition() {
    return positionOut.getPositionMeasure();
  }

  private void setPosition(double rotations) {
    motor.setControl(positionOut.withPosition(rotations));
  }
}
