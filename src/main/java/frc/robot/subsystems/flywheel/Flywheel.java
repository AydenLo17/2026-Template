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

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.AngularVelocity;

import frc.robot.constants.FlywheelConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

/**
 * Flywheel - second example subsystem. Same pattern as {@link frc.robot.subsystems.arm.Arm}:
 * owns its motor, hides setters, exposes commands.
 */
public class Flywheel extends SubsystemBase {

  private final TalonFX leader = new TalonFX(21, TunerConstants.kCANBus);

  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);
  private final AngularVelocity tolerance =
      RotationsPerSecond.of(FlywheelConstants.VELOCITY_TOLERANCE_RPS);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public Flywheel() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = FlywheelConstants.kS;
    config.Slot0.kV = FlywheelConstants.kV;
    config.Slot0.kP = FlywheelConstants.kP;
    config.MotionMagic.MotionMagicCruiseVelocity = FlywheelConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = FlywheelConstants.MOTION_MAGIC_ACCELERATION;

    TalonFXUtil.applyConfigWithRetries(leader, config);
  }

  @Override
  public void periodic() {}

  // ==================== Commands ====================

  /** Spin up to shooting speed. */
  public Command spinUp() {
    return runOnce(() -> setVelocity(FlywheelConstants.SHOOTING_SPEED_RPS));
  }

  /** Stop the flywheel. */
  public Command stop() {
    return runOnce(leader::stopMotor);
  }

  // ==================== Queries ====================

  /** True when the flywheel is within tolerance of its target speed. */
  public boolean isAtTarget() {
    return leader.getVelocity().getValue().isNear(velocityOut.getVelocityMeasure(), tolerance);
  }

  private void setVelocity(double rps) {
    leader.setControl(velocityOut.withVelocity(RotationsPerSecond.of(rps)));
  }
}
