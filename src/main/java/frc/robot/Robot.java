// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.L1;
import frc.robot.telemetry.GeneralLogger;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog()); // Log controller and ds data

    m_robotContainer = new RobotContainer();
    // WebServer.start(5800, Filesystem.getDeployDirectory().toString());
    Epilogue.bind(this);
    // SmartDashboard.putBoolean("VISION_ENABLE", Constants.Vision.ENABLE);
    SmartDashboard.putNumber("INTAKE_SPEED", Constants.AlgaeEndEffector.INTAKE_SPEED);
    SmartDashboard.putNumber("OUTAKE_SPEED", Constants.AlgaeEndEffector.OUTAKE_SPEED);

    SmartDashboard.putNumber("L1_STOW_POS", Units.rotationsToDegrees(L1.ARM_STOW));
    SmartDashboard.putNumber("L1_GROUND_POS", Units.rotationsToDegrees(L1.ARM_GROUND));
    SmartDashboard.putNumber("L1_SCORE_POS", Units.rotationsToDegrees(L1.ARM_L1));

    SmartDashboard.putNumber("L1_kV", L1.ARM_kV);
    SmartDashboard.putNumber("L1_kA", L1.ARM_kA);
    SmartDashboard.putNumber("L1_kG", L1.ARM_kG);
    SmartDashboard.putNumber("L1_kP", L1.ARM_kP);
    SmartDashboard.putNumber("L1_kI", L1.ARM_kI);
    SmartDashboard.putNumber("L1_kD", L1.ARM_kD);

    SmartDashboard.putNumber("L1_VELOCITY", L1.ARM_VELOCITY);
    SmartDashboard.putNumber("L1_ACCEL", L1.ARM_ACCEL);
    SmartDashboard.putNumber("L1_JERK", L1.ARM_JERK);

    SmartDashboard.putNumber("L1_INTAKE_VOLTAGE", L1.INTAKE_MOTOR_VOLTAGE);
    SmartDashboard.putNumber("L1_HOLD_VOLTAGE", L1.HOLD_MOTOR_VOLTAGE);
    SmartDashboard.putNumber("L1_OUTTAKE_VOLTAGE", L1.OUTTAKE_MOTOR_VOLTAGE);
    
  }

  @Override
  public void robotPeriodic() {
    L1.ARM_STOW = Units.degreesToRotations(SmartDashboard.getNumber("L1_STOW_POS", Units.rotationsToDegrees(L1.ARM_STOW)));
    L1.ARM_GROUND = Units.degreesToRotations(SmartDashboard.getNumber("L1_GROUND_POS", Units.rotationsToDegrees(L1.ARM_GROUND)));
    L1.ARM_L1 = Units.degreesToRotations(SmartDashboard.getNumber("L1_SCORE_POS", Units.rotationsToDegrees(L1.ARM_L1)));

    L1.ARM_kV = SmartDashboard.getNumber("L1_kV", L1.ARM_kV);
    L1.ARM_kA = SmartDashboard.getNumber("L1_kA", L1.ARM_kA);
    L1.ARM_kG = SmartDashboard.getNumber("L1_kG", L1.ARM_kG);
    L1.ARM_kP = SmartDashboard.getNumber("L1_kP", L1.ARM_kP);
    L1.ARM_kI = SmartDashboard.getNumber("L1_kI", L1.ARM_kI);
    L1.ARM_kD = SmartDashboard.getNumber("L1_kD", L1.ARM_kD);

    L1.ARM_VELOCITY = SmartDashboard.getNumber("L1_VELOCITY", L1.ARM_VELOCITY);
    L1.ARM_ACCEL = SmartDashboard.getNumber("L1_ACCEL", L1.ARM_ACCEL);
    L1.ARM_JERK = SmartDashboard.getNumber("L1_JERK", L1.ARM_JERK);

    L1.INTAKE_MOTOR_VOLTAGE = SmartDashboard.getNumber("L1_INTAKE_VOLTAGE", L1.INTAKE_MOTOR_VOLTAGE);
    L1.HOLD_MOTOR_VOLTAGE = SmartDashboard.getNumber("L1_HOLD_VOLTAGE", L1.HOLD_MOTOR_VOLTAGE);
    L1.OUTTAKE_MOTOR_VOLTAGE = SmartDashboard.getNumber("L1_OUTTAKE_VOLTAGE", L1.OUTTAKE_MOTOR_VOLTAGE);

    // Constants.Vision.ENABLE = SmartDashboard.getBoolean("VISION_ENABLE", Constants.Vision.ENABLE);
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
    // m_robotContainer.m_elevator.resetEncoder();
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
