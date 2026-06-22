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

import org.wpilib.command3.Command;
import org.wpilib.units.measure.AngularVelocity;

import frc.robot.generated.TunerConstants;
import frc.robot.utils.AdvancedMechanism;
import frc.robot.utils.TalonFXUtil;

/**
 * Flywheel - second example subsystem. Same pattern as {@link frc.robot.subsystems.arm.Arm}:
 * owns its motor, hides setters, exposes commands.
 */
public class Flywheel extends AdvancedMechanism {
  // Shooting speed (rotations per second).
  private static final double SHOOTING_SPEED_RPS = 25.0;

  // How close the measured speed needs to be to count as "at target".
  private static final double VELOCITY_TOLERANCE_RPS = 0.25;

  // PID + feedforward gains.
  private static final double kS = 0.0;   // static friction compensation
  private static final double kV = 0.125; // velocity feedforward (volts per rps)
  private static final double kP = 0.0;   // proportional gain on velocity error

  // Motion Magic speed limits.
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = 100.0; // max rps
  private static final double MOTION_MAGIC_ACCELERATION = 1000.0;   // rps² ramp

  private final TalonFX motor = new TalonFX(21, TunerConstants.kCANBus);

  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);
  private final AngularVelocity tolerance = RotationsPerSecond.of(VELOCITY_TOLERANCE_RPS);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public Flywheel() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;
    config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;

    TalonFXUtil.applyConfigWithRetries(motor, config);
  }

  /** Fire-and-forget: command the flywheel toward shooting speed and finish immediately. */
  public Command spinUp() {
    return runOnce("spinUp", () -> setVelocity(SHOOTING_SPEED_RPS));
  }

  /**
   * Command the flywheel toward shooting speed and hold the mechanism until it is at speed. If
   * interrupted before reaching speed the motor is stopped; on a natural finish it is left
   * spinning, ready to shoot.
   */
  public Command spinUpAndWait() {
    return runRepeatedly(() -> setVelocity(SHOOTING_SPEED_RPS))
        .until(this::isAtTarget)
        .whenCanceled(motor::stopMotor)
        .named("spinUpAndWait");
  }

  /** Stop the flywheel. */
  public Command stop() {
    return runOnce("stop", motor::stopMotor);
  }

  /** True when the flywheel is within tolerance of its target speed. */
  public boolean isAtTarget() {
    return motor.getVelocity().getValue().isNear(velocityOut.getVelocityMeasure(), tolerance);
  }

  // Direct actuators for the in-package classic-style command ({@link FlywheelCommand}). Kept
  // package-private so the public API stays commands-only - everything outside this package still
  // goes through spinUp()/spinUpAndWait()/stop() and the scheduler's mechanism ownership.

  /** Command the flywheel toward shooting speed (no command wrapper). */
  void spinUpDirect() {
    setVelocity(SHOOTING_SPEED_RPS);
  }

  /** Stop the flywheel (no command wrapper). */
  void stopDirect() {
    motor.stopMotor();
  }

  private void setVelocity(double rps) {
    motor.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }
}
