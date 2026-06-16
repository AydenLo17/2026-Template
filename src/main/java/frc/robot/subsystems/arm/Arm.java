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

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.Angle;

import frc.robot.constants.ArmConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

/**
 * Arm - an example subsystem driven by a Phoenix 6 TalonFX + CANcoder.
 *
 * <p>Pattern to teach: the subsystem owns the hardware, keeps its setters {@code private}, and
 * exposes <b>commands</b> (each returns a {@link Command}). Anything that wants to move the arm
 * does it through a command, which is how the scheduler prevents two things fighting over the motor.
 */
public class Arm extends SubsystemBase {
  // Shares the same CAN bus as the drivetrain (defined once in the Tuner X output).
  private final TalonFX leader = new TalonFX(31, TunerConstants.kCANBus);
  private final CANcoder encoder = new CANcoder(32, TunerConstants.kCANBus);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Drives the arm to a target angle with a smooth Motion Magic profile.
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  // How close counts as "at target".
  private final Angle tolerance = Degrees.of(ArmConstants.POSITION_TOLERANCE_DEGREES);

  public Arm() {
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // fights gravity automatically

    // Control gains (TODO: CRITICAL - tune on the real robot).
    config.Slot0.kG = ArmConstants.kG;
    config.Slot0.kS = ArmConstants.kS;
    config.Slot0.kP = ArmConstants.kP;
    config.Slot0.kD = ArmConstants.kD;

    config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ArmConstants.MOTION_MAGIC_ACCELERATION;
    config.Feedback.withRemoteCANcoder(encoder);

    TalonFXUtil.applyConfigWithRetries(leader, config);
  }

  @Override
  public void periodic() {
    // No periodic work needed - control is entirely feedforward/feedback on the motor controller.
  }

  // ==================== Commands ====================

  /** Move to the vertical (stowed) position. */
  public Command vertical() {
    return runOnce(() -> setPosition(ArmConstants.VERTICAL_POSITION_ROTATIONS));
  }

  /** Move to the horizontal (ground intake) position. */
  public Command horizontal() {
    return runOnce(() -> setPosition(ArmConstants.HORIZONTAL_POSITION_ROTATIONS));
  }

  /** Move to the scoring position. */
  public Command scoring() {
    return runOnce(() -> setPosition(ArmConstants.SCORING_POSITION_ROTATIONS));
  }

  /** Move to the high scoring position (far shots). */
  public Command scoringHigh() {
    return runOnce(() -> setPosition(ArmConstants.SCORING_HIGH_POSITION_ROTATIONS));
  }

  // ==================== Queries ====================

  /** True when the arm has reached its target angle. */
  public boolean isAtTarget() {
    return getPosition().isNear(getTargetPosition(), tolerance);
  }

  /** Current measured arm angle. */
  public Angle getPosition() {
    return encoder.getPosition().getValue();
  }

  /** Angle the arm is currently driving toward. */
  public Angle getTargetPosition() {
    return positionOut.getPositionMeasure();
  }

  // private: callers move the arm through commands, not direct setters.
  private void setPosition(double rotations) {
    leader.setControl(positionOut.withPosition(rotations));
  }
}
