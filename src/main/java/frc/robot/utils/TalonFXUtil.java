package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Utility class for common TalonFX motor operations.
 *
 * <p>Phoenix 6's {@code Configurator.apply(...)} runs once and returns a status code — it does NOT
 * retry on its own. {@link #applyConfigWithRetries} retries up to 5 times, which catches transient
 * CAN faults at robot boot.
 */
public final class TalonFXUtil {

  private TalonFXUtil() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Applies a configuration to a TalonFX motor with automatic retries.
   *
   * @param motor The motor to configure
   * @param config The configuration to apply
   * @param maxRetries Maximum number of retry attempts (default: 5)
   * @return true if configuration was successfully applied, false otherwise
   */
  public static boolean applyConfigWithRetries(
      TalonFX motor, TalonFXConfiguration config, int maxRetries) {
    for (int i = 0; i < maxRetries; i++) {
      StatusCode status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        return true;
      }
    }
    return false;
  }

  /**
   * Applies a configuration to a TalonFX motor with default retry count (5).
   *
   * @param motor The motor to configure
   * @param config The configuration to apply
   * @return true if configuration was successfully applied, false otherwise
   */
  public static boolean applyConfigWithRetries(TalonFX motor, TalonFXConfiguration config) {
    return applyConfigWithRetries(motor, config, 5);
  }
}
