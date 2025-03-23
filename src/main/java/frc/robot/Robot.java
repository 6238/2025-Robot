// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.telemetry.GeneralLogger;
import frc.robot.util.ReefUtils;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog()); // Log controller and ds data

    m_robotContainer = new RobotContainer();
    WebServer.start(5800, Filesystem.getDeployDirectory().toString());
    Epilogue.bind(this);
    SmartDashboard.putNumber("INTAKE_SPEED", Constants.AlgaeEndEffector.INTAKE_SPEED);
    SmartDashboard.putNumber("OUTAKE_SPEED", Constants.AlgaeEndEffector.OUTAKE_SPEED);
  }

  @Override
  public void robotPeriodic() {
    Constants.AlgaeEndEffector.INTAKE_SPEED =
        SmartDashboard.getNumber("INTAKE_SPEED", Constants.AlgaeEndEffector.INTAKE_SPEED);
    Constants.AlgaeEndEffector.OUTAKE_SPEED =
        SmartDashboard.getNumber("OUTAKE_SPEED", Constants.AlgaeEndEffector.OUTAKE_SPEED);

    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
    GeneralLogger.log();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.OnDisable();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    m_robotContainer.OnEnable();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.m_elevator.resetEncoder();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
