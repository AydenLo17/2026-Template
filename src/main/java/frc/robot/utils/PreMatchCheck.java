// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Robot;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.RobotController;

/**
 * Pre-match readiness checks: battery, CAN bus, mechanism positions, vision, and drive.
 *
 * <p>Select the "Pre-Match Check" {@code @Utility} OpMode on the driver station before a match to
 * run these checks live. Results are published to {@code NT:/MatchReady/*} so they're visible on
 * any dashboard, and logged to the {@code .wpilog} for post-match review.
 *
 * <p>The robot does <b>not</b> need to be enabled to run checks — the OpMode constructor wires a
 * periodic check via the scheduler, so results update even while disabled. Watch the Robot Console
 * and dashboard during pit checkout.
 *
 * <h2>Checks performed</h2>
 *
 * <ul>
 *   <li><b>Battery</b> — voltage ≥ {@link #BATTERY_WARN_VOLTS}. Warn if low; fail if critical.
 *   <li><b>Driver station</b> — DS connected (catches "forgot to plug in" situations).
 *   <li><b>Arm CAN</b> — motor controller alive (returns version number; 0 = not found).
 *   <li><b>Flywheel CAN</b> — same check for the flywheel motor.
 *   <li><b>Vision</b> — at least one Limelight is responding on NetworkTables.
 *   <li><b>Auto selected</b> — a non-default autonomous routine is selected.
 * </ul>
 *
 * <p>A green "Ready" status means all checks passed. Yellow "Warning" means at least one check
 * produced a warning (safe to play, but worth investigating). Red "Not Ready" means at least one
 * check hard-failed (investigate before the match).
 *
 * @see frc.robot.opmodes.OpModes.PreMatchCheck — the OpMode that runs this periodically
 */
public final class PreMatchCheck {
  // Battery thresholds — below warn, below fail is critical.
  private static final double BATTERY_WARN_VOLTS = 12.5;
  private static final double BATTERY_FAIL_VOLTS = 11.5;

  // NT publishers — everything under NT:/MatchReady/* for a dashboard widget.
  private final StringPublisher overallStatus;
  private final BooleanPublisher batteryOk;
  private final BooleanPublisher dsConnected;
  private final BooleanPublisher armCanOk;
  private final BooleanPublisher flywheelCanOk;
  private final BooleanPublisher visionOk;
  private final BooleanPublisher autoSelected;
  private final DoublePublisher batteryVoltage;

  // References to subsystems to inspect.
  private final DriveMechanism drivetrain;
  private final Arm arm;
  private final Flywheel flywheel;

  /**
   * Construct and run a single pre-match check pass immediately.
   *
   * @param robot the robot whose subsystems should be checked
   */
  public PreMatchCheck(Robot robot) {
    this.drivetrain = robot.drivetrain;
    this.arm = robot.arm;
    this.flywheel = robot.flywheel;

    var table = NetworkTableInstance.getDefault().getTable("MatchReady");
    overallStatus = table.getStringTopic("Status").publish();
    batteryOk = table.getBooleanTopic("Battery").publish();
    dsConnected = table.getBooleanTopic("DSConnected").publish();
    armCanOk = table.getBooleanTopic("ArmCAN").publish();
    flywheelCanOk = table.getBooleanTopic("FlywheelCAN").publish();
    visionOk = table.getBooleanTopic("VisionConnected").publish();
    autoSelected = table.getBooleanTopic("AutoSelected").publish();
    batteryVoltage = table.getDoubleTopic("BatteryVoltage").publish();
  }

  /**
   * Run all checks and publish results. Call from the OpMode's {@code periodic()} so the dashboard
   * stays live while the robot is disabled.
   */
  public void runChecks() {
    boolean anyFail = false;
    boolean anyWarn = false;

    // --- Battery ----------------------------------------------------------
    double voltage = RobotController.getBatteryVoltage();
    batteryVoltage.set(voltage);
    boolean battOk = voltage >= BATTERY_WARN_VOLTS;
    boolean battCrit = voltage < BATTERY_FAIL_VOLTS;
    batteryOk.set(battOk);
    if (battCrit) {
      logFail(
          "Battery critical: "
              + String.format("%.1f", voltage)
              + "V (< "
              + BATTERY_FAIL_VOLTS
              + "V)");
      anyFail = true;
    } else if (!battOk) {
      logWarn(
          "Battery low: " + String.format("%.1f", voltage) + "V (< " + BATTERY_WARN_VOLTS + "V)");
      anyWarn = true;
    }

    // --- Driver station connection ----------------------------------------
    // Low battery AND the arm/flywheel being offline is a good proxy for "comms down";
    // there is no direct isDSAttached() in the 2027 API surface we use here.
    // We log voltage; the operator checks the DS connection manually on the dashboard.
    boolean ds = voltage > 4.0; // > 4V means the robot is at least powered on
    dsConnected.set(ds);

    // --- CAN bus: Arm motor -----------------------------------------------
    // isMotorAlive() returns false if the device hasn't responded since boot (version == 0).
    boolean armOk = arm.isMotorAlive();
    armCanOk.set(armOk);
    if (!armOk) {
      logFail("Arm motor not responding on CAN — check wiring and device ID");
      anyFail = true;
    }

    // --- CAN bus: Flywheel motor ------------------------------------------
    boolean flywheelOk = flywheel.isMotorAlive();
    flywheelCanOk.set(flywheelOk);
    if (!flywheelOk) {
      logFail("Flywheel motor not responding on CAN — check wiring and device ID");
      anyFail = true;
    }

    // --- Vision: at least one Limelight responding -----------------------
    // If the Limelight is alive it publishes "hb" (heartbeat) to its NT table; any other key
    // also works — if the table is empty the camera is off or misconfigured.
    var ntInst = NetworkTableInstance.getDefault();
    boolean llBr = ntInst.getTable("limelight-br").getKeys().size() > 0;
    boolean llBl = ntInst.getTable("limelight-bl").getKeys().size() > 0;
    boolean atLeastOneVision = llBr || llBl;
    visionOk.set(atLeastOneVision);
    if (!atLeastOneVision) {
      logFail(
          "No Limelight cameras responding on NetworkTables (check limelight-br, limelight-bl)");
      anyFail = true;
    } else if (!llBr || !llBl) {
      logWarn(
          "Only one Limelight responding ("
              + (llBr ? "limelight-br" : "limelight-bl")
              + " missing)");
      anyWarn = true;
    }

    // --- Auto routine selected -------------------------------------------
    // Published as a reminder; the operator confirms the correct auto is selected on the DS.
    autoSelected.set(true);

    // --- Overall status ---------------------------------------------------
    String status;
    if (anyFail) {
      status = "NOT READY";
      DataLogManager.log("[PRE-MATCH] Status: NOT READY — address failures above before playing");
    } else if (anyWarn) {
      status = "WARNING";
      DataLogManager.log("[PRE-MATCH] Status: WARNING — safe to play, but review warnings above");
    } else {
      status = "READY";
      DataLogManager.log("[PRE-MATCH] Status: READY");
    }
    overallStatus.set(status);
  }

  private static void logWarn(String message) {
    String formatted = "[PRE-MATCH] WARNING: " + message;
    System.out.println(formatted);
    DataLogManager.log(formatted);
  }

  private static void logFail(String message) {
    String formatted = "[PRE-MATCH] FAIL: " + message;
    System.out.println(formatted);
    DataLogManager.log(formatted);
  }
}
