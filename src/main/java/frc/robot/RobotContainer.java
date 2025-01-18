// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.util.AutonTeleController;

import java.io.File;

/**
 * This class is where almost all of the robot is defined - logic and subsystems are all set up
 * here.
 */
public class RobotContainer {

  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  CommandXboxController driverXbox = new CommandXboxController(0);
  AutonTeleController autonTeleController = new AutonTeleController(driverXbox);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    PathfindingCommand.warmupCommand().schedule();
    configureTriggers();

    Command driveFieldOrientedDirectAngle = swerve.debugDriveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.02),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.02),
      () -> -driverXbox.getRightX(),
      () -> -driverXbox.getRightY());
      
      swerve.setDefaultCommand(driveFieldOrientedDirectAngle);
    
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
    autonTeleController.SetupPoseCommands();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}