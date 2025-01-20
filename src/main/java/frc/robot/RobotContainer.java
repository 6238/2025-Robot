// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Logging;

import java.io.File;

/**
 * This class is where almost all of the robot is defined - logic and subsystems are all set up
 * here.
 */
public class RobotContainer {

  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
  ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DataLogManager.start();
    Logging.logMetadata();
    Logging.initializeCommandSchedulerHooks();

    configureTriggers();
    
    NamedCommands.registerCommand ("Elevator_L1", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1));
    NamedCommands.registerCommand ("Elevator_L2", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2));
    NamedCommands.registerCommand ("Elevator_L3", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3));
    NamedCommands.registerCommand ("Elevator_L4", m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L4));

    Command driveCommand = swerve.driveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.02),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.02),
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.08));
      
      swerve.setDefaultCommand(driveCommand);
    
    // Initialize autonomous chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Path", autoChooser);
  }

  /**
   * This method is where all of the robot's logic is defined. We link {@link
   * edu.wpi.first.wpilibj2.command.button.Trigger}s, such as controller buttons and subsystem
   * state, to {@link edu.wpi.first.wpilibj2.command.Command} instances. The advantage of
   * configuring all the robot's logic here is that it's easy to find, and therefore easy to modify,
   * what the robot does when something happens and why.
   */
  private void configureTriggers() {
    // Controls
    driverXbox.start().onTrue(swerve.zeroYawCommand());
    
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L1));

    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L2));

    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L3));

    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(m_elevator.setHeightCommand(Constants.Elevator.ElevatorHeights.L4));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}