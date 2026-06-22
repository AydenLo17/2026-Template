// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.framework.TimedRobot;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getDefault().run();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            Scheduler.getDefault().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            Scheduler.getDefault().cancel(autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    /**
     * Utility mode is a 2027/Systemcore-only state for safely interacting with the robot off-field
     * (e.g. configuring devices, manually pushing it around). Cancel any leftover commands so nothing
     * fights you.
     */
    @Override
    public void utilityInit() {
        Scheduler.getDefault().cancelAll();
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void simulationPeriodic() {}
}
