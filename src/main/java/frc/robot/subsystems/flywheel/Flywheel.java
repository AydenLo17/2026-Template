// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Flywheel - example velocity mechanism driven by a Phoenix 6 TalonFX.
 *
 * <h2>Pattern</h2>
 *
 * <p>Like {@link frc.robot.subsystems.arm.Arm}, the subsystem owns the motor, keeps setters
 * private, and exposes only <b>commands</b>. The scheduler prevents multiple commands fighting for
 * the motor.
 *
 * <h2>Hardware</h2>
 *
 * <ul>
 *   <li><b>Motor:</b> TalonFX on CAN 21 (update in constructor if different)
 *   <li><b>Control mode:</b> {@link MotionMagicVelocityVoltage} speed control
 *   <li><b>Neutral mode:</b> Coast (flywheel spins freely when unpowered)
 * </ul>
 *
 * <h2>Tuning</h2>
 *
 * <p>Flywheel gains are simpler than arm (no gravity), but still need tuning:
 *
 * <ul>
 *   <li><b>Velocity feedforward ({@code kV}):</b> In (volts per rps). Measures the "slope" of the
 *       motor speed curve. Safe starting point: {@code 0.125} (typical for FRC motors). Lower if
 *       overshooting target speed, higher if undershooting.
 *   <li><b>Static friction ({@code kS}):</b> Voltage to overcome friction and get the motor
 *       spinning. Safe start: {@code 0.0} (flywheels typically have low static friction).
 *   <li><b>Proportional gain ({@code kP}):</b> Correction strength. Safe start: {@code 0.01} (small
 *       because kV does most of the work). Raise if slow to settle, lower if jittery.
 *   <li><b>Motion Magic limits:</b> {@code MOTION_MAGIC_CRUISE_VELOCITY} (max flywheel speed,
 *       rot/s) and {@code MOTION_MAGIC_ACCELERATION} (acceleration limit, rot/s²). These are
 *       typically large; set based on what speed your game requires.
 * </ul>
 *
 * <p>Once tuned, update the static final constants at the top of this file. See <a
 * href="../../../../../../DEPLOYMENT_CHECKLIST.md">DEPLOYMENT_CHECKLIST.md</a> for the step-by-step
 * tuning procedure.
 *
 * <h2>Commands</h2>
 *
 * <ul>
 *   <li>{@link #spinUp()} — command flywheel to shooting speed, hold until interrupted
 *   <li>{@link #stop()} — stop and coast
 * </ul>
 *
 * @see frc.robot.Robot#flywheel — owned by the robot
 * @see frc.robot.Robot#autoScore() — uses flywheel as part of superstructure
 */
public class Flywheel extends Mechanism {
  // Shooting speed (rotations per second).
  private static final double SHOOTING_SPEED_RPS = 25.0;

  // How close the measured speed needs to be to count as "at target".
  private static final double VELOCITY_TOLERANCE_RPS = 0.25;

  // PID + feedforward gains.
  private static final double kS = 0.0; // static friction compensation
  private static final double kV = 0.125; // velocity feedforward (volts per rps)
  private static final double kP = 0.0; // proportional gain on velocity error

  // Motion Magic speed limits.
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 100.0; // max rps
  private static final double MOTION_MAGIC_ACCELERATION = 1000.0; // rps² ramp

  private final TalonFX motor = new TalonFX(21, TunerConstants.kCANBus);

  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);
  private final AngularVelocity tolerance = RotationsPerSecond.of(VELOCITY_TOLERANCE_RPS);

  public Flywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;
    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;

    TalonFXUtil.applyConfigWithRetries(motor, config);
  }

  // The hold commands below use runRepeatedly, which re-sends the request every loop. Phoenix
  // already holds the last request; re-sending just re-asserts it if the controller reboots.

  /** Command the flywheel to shooting speed and hold it there until interrupted or superseded. */
  public Command spinUp() {
    return runRepeatedly(() -> setVelocity(SHOOTING_SPEED_RPS)).named("spinUp");
  }

  /** Stop the flywheel. */
  public Command stop() {
    return runRepeatedly(motor::stopMotor).named("stop");
  }

  /** True when the flywheel is within tolerance of its target speed. */
  public boolean isAtTarget() {
    return motor.getVelocity().getValue().isNear(velocityOut.getVelocityMeasure(), tolerance);
  }

  /**
   * True when the flywheel motor controller is alive on the CAN bus. Returns false if the device
   * has never responded since boot (version == 0), which indicates a wiring or ID problem.
   */
  public boolean isMotorAlive() {
    return motor.getVersion().getValue() > 0;
  }

  private void setVelocity(double rps) {
    motor.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }
}
